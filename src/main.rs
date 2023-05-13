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
use defmt::{debug, warn};
use defmt_rtt as _;
use hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};
use heapless::Vec;
use panic_probe as _;
use rp_pico as bsp;
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

use crate::usbd_ethernet::cdc_ncm::CdcNcmClass;
use crate::usbd_ethernet::cdc_ncm::State;
use crate::usbd_ethernet::cdc_ncm::USB_CLASS_CDC;

mod usbd_ethernet;

const HOST_MAC_ADDR: [u8; 6] = [0x88, 0x88, 0x88, 0x88, 0x88, 0x88];
const DEVICE_MAC_ADDR: [u8; 6] = [0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
const HOST_NAME: &[u8] = b"pico";

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let ring_oscillator = RingOscillator::new(pac.ROSC).initialize();

    // Prevent https://github.com/rp-rs/rp-hal/issues/606 occurring
    let sio = hal::sio::Sio::new(pac.SIO);
    let _pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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

    let usb_alloc = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut cdc_ncm = CdcNcmClass::new(&usb_alloc, HOST_MAC_ADDR, 64);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_alloc, UsbVidPid(0x1209, 0x0004))
        .manufacturer("pico-networking")
        .product("usb-ethernet")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build();

    let mut config = Config::new();
    config.hardware_addr = Some(EthernetAddress(DEVICE_MAC_ADDR).into());
    config.random_seed = generate_random_seed(&ring_oscillator);

    let mut iface = Interface::new(config, &mut cdc_ncm);

    // Create sockets
    let mut dhcp_socket = dhcpv4::Socket::new();

    // register hostname with dhcp
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12, // Host Name
        data: HOST_NAME,
    }]);

    let telnet_socket = {
        // It is not strictly necessary to use a `static mut` and unsafe code here, but
        // on embedded systems that smoltcp targets it is far better to allocate the data
        // statically to verify that it fits into RAM rather than get undefined behavior
        // when stack overflows.
        static mut TCP_SERVER_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_SERVER_TX_DATA: [u8; 1024] = [0; 1024];
        let tcp_receive_buffer = tcp::SocketBuffer::new(unsafe { &mut TCP_SERVER_RX_DATA[..] });
        let tcp_transmit_buffer = tcp::SocketBuffer::new(unsafe { &mut TCP_SERVER_TX_DATA[..] });
        tcp::Socket::new(tcp_receive_buffer, tcp_transmit_buffer)
    };

    let http_socket = {
        // It is not strictly necessary to use a `static mut` and unsafe code here, but
        // on embedded systems that smoltcp targets it is far better to allocate the data
        // statically to verify that it fits into RAM rather than get undefined behavior
        // when stack overflows.
        static mut TCP_SERVER_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_SERVER_TX_DATA: [u8; 1024] = [0; 1024];
        let tcp_receive_buffer = tcp::SocketBuffer::new(unsafe { &mut TCP_SERVER_RX_DATA[..] });
        let tcp_transmit_buffer = tcp::SocketBuffer::new(unsafe { &mut TCP_SERVER_TX_DATA[..] });
        tcp::Socket::new(tcp_receive_buffer, tcp_transmit_buffer)
    };

    let mut sockets: [_; 3] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets[..]);
    let telnet_handle = sockets.add(telnet_socket);
    let http_handle = sockets.add(http_socket);
    let dhcp_handle = sockets.add(dhcp_socket);

    loop {
        if usb_dev.poll(&mut [&mut cdc_ncm]) && cdc_ncm.state() == State::Enabled {
            let _: usb_device::Result<()> = cdc_ncm.connect();
        }

        if cdc_ncm.state() == State::Connected {
            let timestamp =
                Instant::from_micros(i64::try_from(timer.get_counter().ticks()).unwrap());
            if iface.poll(timestamp, &mut cdc_ncm, &mut sockets) {
                telnet_poll(sockets.get_mut::<tcp::Socket>(telnet_handle));
                http_poll(sockets.get_mut::<tcp::Socket>(http_handle));
                dhcp_poll(&mut iface, sockets.get_mut::<dhcpv4::Socket>(dhcp_handle));
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

fn telnet_poll(socket: &mut tcp::Socket) {
    if !socket.is_open() {
        socket.listen(22).unwrap();
        debug!("telnet: listening on 22");
    }

    if socket.may_recv() {
        let mut data = Vec::<u8, 1024>::new();
        socket
            .recv(|buffer| {
                if !buffer.is_empty() {
                    if let Ok(s) = from_utf8(buffer) {
                        debug!("telnet: recv data: {:?}", s);
                    } else {
                        debug!("telnet: recv data: {:?}", buffer);
                    }
                }
                // Echo the data back in upper case
                data.extend(buffer.iter().copied().map(|c| c.to_ascii_uppercase()));
                (buffer.len(), ())
            })
            .unwrap();
        if socket.can_send() && !data.is_empty() {
            socket.send_slice(&data[..]).unwrap();
        }
    } else if socket.may_send() {
        debug!("telnet: socket close");
        socket.close();
    }
}

enum HttpRequest {
    Get,
    Unknown,
    ClientError,
}

fn http_poll(socket: &mut tcp::Socket) {
    if !socket.is_open() {
        socket.listen(80).unwrap();
        debug!("http: listening on 80");
    }

    if socket.may_recv() {
        let request = socket
            .recv(|buffer| {
                let request = if buffer.is_empty() {
                    None
                } else if let Ok(s) = from_utf8(buffer) {
                    debug!("http: recv data: {:?}", s);

                    if s.starts_with("GET / HTTP/1.1") {
                        debug!("http: recv get /");
                        Some(HttpRequest::Get)
                    } else {
                        debug!("http: recv unknown");
                        Some(HttpRequest::Unknown)
                    }
                } else {
                    debug!("http: recv data: {:?}", buffer);
                    Some(HttpRequest::ClientError)
                };
                (buffer.len(), request)
            })
            .unwrap();
        if socket.can_send() && request.is_some() {
            match request {
                Some(HttpRequest::Get) => {
                    socket
                        .send_slice(
                            b"HTTP/1.1 200 OK
Connection: close
Content-Length: 53
Content-Type: text/html

<!DOCTYPE html>
<html><body>Hello pico!</body></html>",
                        )
                        .unwrap();
                }

                Some(HttpRequest::Unknown) => {
                    socket
                        .send_slice(
                            b"HTTP/1.1 404 Not Found
Connection: close
Content-Length: 0

",
                        )
                        .unwrap();
                }
                Some(HttpRequest::ClientError) => {
                    socket
                        .send_slice(
                            b"HTTP/1.1 400 Bad Request
Connection: close
Content-Length: 0

",
                        )
                        .unwrap();
                }
                None => {}
            }
        }
    } else if socket.may_send() {
        debug!("http: socket close");
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
        if let Some(a) = addrs
            .iter_mut()
            .find(|addr| matches!(addr, IpCidr::Ipv4(_)))
        {
            *a = IpCidr::Ipv4(cidr);
        } else {
            addrs.push(IpCidr::Ipv4(cidr)).unwrap();
        }
    });
}
