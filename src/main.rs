#![no_std]
#![no_main]
#![warn(clippy::pedantic)]
#![warn(clippy::style)]
#![warn(clippy::cargo)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::missing_panics_doc)]
#![warn(clippy::use_self)]

use core::str::from_utf8;

use bsp::entry;
use bsp::hal::{self, rosc::RingOscillator, Timer};
use defmt::{debug, error, warn};
use defmt_rtt as _;
use hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};
use heapless::Vec;
use panic_probe as _;
use rp_pico as bsp;
use smoltcp::socket::dhcpv4::RetryConfig;
use smoltcp::time::Duration;
use smoltcp::wire::DhcpOption;
use smoltcp::{
    iface::{Config, Interface, SocketSet},
    socket::{dhcpv4, tcp},
    time::Instant,
    wire::{EthernetAddress, IpCidr, Ipv4Address, Ipv4Cidr},
};
#[allow(clippy::wildcard_imports)]
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use web_server::WebServer;

use crate::usbd_ethernet::cdc_ncm::CdcNcmClass;
use crate::usbd_ethernet::cdc_ncm::DeviceState;
use crate::usbd_ethernet::cdc_ncm::USB_CLASS_CDC;

mod usbd_ethernet;
mod web_server;

const HOST_MAC_ADDR: [u8; 6] = [0x88, 0x88, 0x88, 0x88, 0x88, 0x88];
const DEVICE_MAC_ADDR: [u8; 6] = [0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
const HOST_NAME: &[u8] = b"pico";

#[entry]
fn main() -> ! {
    static mut TELNET_SOCKET_RX_DATA: [u8; 1024] = [0; 1024];
    static mut TELNET_SOCKET_TX_DATA: [u8; 1024] = [0; 1024];
    static mut HTTP_SOCKET_RX_DATA: [u8; 1024] = [0; 1024];
    static mut HTTP_SOCKET_TX_DATA: [u8; 1024] = [0; 1024];
    static mut NCM_IN_BUFFER: [u8; 2048] = [0; 2048];
    static mut NCM_OUT_BUFFER: [u8; 2048] = [0; 2048];
    static mut TELNET_BUFFER: Vec<u8, 1024> = Vec::new();

    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let ring_oscillator = RingOscillator::new(pac.ROSC);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let usb_bus = hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );

    let usb_alloc = UsbBusAllocator::new(usb_bus);
    let mut cdc_ncm =
        CdcNcmClass::new(&usb_alloc, HOST_MAC_ADDR, 64, NCM_IN_BUFFER, NCM_OUT_BUFFER);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_alloc, UsbVidPid(0x1209, 0x0004))
        .manufacturer("pico-networking")
        .product("usb-ethernet")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build();

    let mut interface_config = Config::new();
    interface_config.hardware_addr = Some(EthernetAddress(DEVICE_MAC_ADDR).into());
    let ring_oscillator = ring_oscillator.initialize();
    interface_config.random_seed = generate_random_seed(&ring_oscillator);

    let mut interface = Interface::new(interface_config, &mut cdc_ncm);
    interface.update_ip_addrs(|ip_addrs| {
        ip_addrs
            .push(Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0).into())
            .unwrap();
    });

    // Create sockets
    let mut dhcp_socket = dhcpv4::Socket::new();
    dhcp_socket.set_retry_config(RetryConfig {
        discover_timeout: Duration::from_secs(5),
        ..RetryConfig::default()
    });
    // register hostname with dhcp
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12, // Host Name
        data: HOST_NAME,
    }]);

    let telnet_socket = tcp::Socket::new(
        tcp::SocketBuffer::new(&mut TELNET_SOCKET_RX_DATA[..]),
        tcp::SocketBuffer::new(&mut TELNET_SOCKET_TX_DATA[..]),
    );
    let http_socket = tcp::Socket::new(
        tcp::SocketBuffer::new(&mut HTTP_SOCKET_RX_DATA[..]),
        tcp::SocketBuffer::new(&mut HTTP_SOCKET_TX_DATA[..]),
    );

    let mut sockets: [_; 3] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets[..]);
    let telnet_handle = sockets.add(telnet_socket);
    let http_handle = sockets.add(http_socket);
    let dhcp_handle = sockets.add(dhcp_socket);

    let mut web_server = WebServer::new(http_handle);

    loop {
        if usb_dev.poll(&mut [&mut cdc_ncm]) && cdc_ncm.state() == DeviceState::Disconnected {
            if cdc_ncm.connection_speed().is_none() {
                // 1000 Kps upload and download
                match cdc_ncm.set_connection_speed(1_000_000, 1_000_000) {
                    Ok(_) | Err(UsbError::WouldBlock) => {}
                    Err(e) => error!("Failed to set connection speed: {}", e),
                }
            } else {
                match cdc_ncm.connect() {
                    Ok(_) | Err(UsbError::WouldBlock) => {}
                    Err(e) => error!("Failed to connect: {}", e),
                }
            }
        }

        if cdc_ncm.state() == DeviceState::Connected {
            // panic safety - will take 292_277 years to overflow at one tick per microsecond
            let timestamp =
                Instant::from_micros(i64::try_from(timer.get_counter().ticks()).unwrap());

            if interface.poll(timestamp, &mut cdc_ncm, &mut sockets) {
                let telnet_socket = sockets.get_mut::<tcp::Socket>(telnet_handle);
                telnet_poll(telnet_socket, TELNET_BUFFER);
                web_server.poll(|handle| sockets.get_mut(handle));
                dhcp_poll(
                    &mut interface,
                    sockets.get_mut::<dhcpv4::Socket>(dhcp_handle),
                );
            }
        }
    }
}

fn generate_random_seed(ring_oscillator: &RingOscillator<hal::rosc::Enabled>) -> u64 {
    let mut seed = 0u64;
    for i in 0..64 {
        seed += u64::from(ring_oscillator.get_random_bit()) << i;
    }
    seed
}

fn telnet_poll<const LEN: usize>(socket: &mut tcp::Socket, buffer: &mut Vec<u8, LEN>) {
    buffer.clear();

    if !socket.is_open() {
        socket.listen(22).unwrap();
        debug!("telnet: listening on 22");
    }

    if socket.may_recv() {
        socket
            .recv(|data| {
                if !data.is_empty() {
                    if let Ok(s) = from_utf8(data) {
                        debug!("telnet: recv data: {:?}", s);
                    } else {
                        debug!("telnet: recv data: {:?}", data);
                    }
                }
                // Echo the data back in upper case
                buffer.extend(data.iter().copied().map(|c| c.to_ascii_uppercase()));
                (buffer.len(), ())
            })
            .unwrap();
        if socket.can_send() && !buffer.is_empty() {
            socket.send_slice(&buffer[..]).unwrap();
        }
    } else if socket.may_send() {
        debug!("telnet: socket close");
        socket.close();
    }
}

fn dhcp_poll(iface: &mut Interface, socket: &mut dhcpv4::Socket) {
    let event = socket.poll();
    match event {
        None => {}
        Some(dhcpv4::Event::Configured(config)) => {
            debug!("dhcp: DHCP configured");

            debug!("     IP address:      {}", config.address);
            set_ipv4_addr(iface, config.address);

            if let Some(router) = config.router {
                debug!("     Default gateway: {}", router);
                iface.routes_mut().add_default_ipv4_route(router).unwrap();
            } else {
                debug!("     Default gateway: None");
                iface.routes_mut().remove_default_ipv4_route();
            }

            for (i, s) in config.dns_servers.iter().enumerate() {
                debug!("     DNS server {}:    {}", i, s);
            }
        }
        Some(dhcpv4::Event::Deconfigured) => {
            debug!("dhcp: DHCP deconfigured");
            set_ipv4_addr(iface, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
            iface.routes_mut().remove_default_ipv4_route();
        }
    }
}

fn set_ipv4_addr(iface: &mut Interface, cidr: Ipv4Cidr) {
    iface.update_ip_addrs(|addrs| {
        let dest = addrs.iter_mut().next().unwrap();
        *dest = IpCidr::Ipv4(cidr);
    });
}
