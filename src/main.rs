#![no_std]
#![no_main]
#![warn(clippy::pedantic)]
#![warn(clippy::style)]
#![warn(clippy::cargo)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::missing_panics_doc)]
#![allow(clippy::struct_excessive_bools)]
#![warn(clippy::use_self)]

use core::fmt::Write;

use bsp::entry;
use bsp::hal;
use defmt::debug;
use defmt::{info, warn};
use defmt_rtt as _;
use hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};
use panic_probe as _;
use rp_pico as bsp;
use smoltcp::socket::dhcpv4;
use smoltcp::wire::Ipv4Cidr;
use smoltcp::{
    iface::{Config, Interface, SocketSet},
    socket::tcp,
    time::{Duration, Instant},
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
};
#[allow(clippy::wildcard_imports)]
use usb_device::class_prelude::*;
use usb_device::prelude::*;

use crate::usbd_ethernet::cdc_ncm::CdcNcmClass;
use crate::usbd_ethernet::cdc_ncm::State;
use crate::usbd_ethernet::cdc_ncm::USB_CLASS_CDC;

mod usbd_ethernet;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

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

    let host_mac_addr = [0x88, 0x88, 0x88, 0x88, 0x88, 0x88];

    let mut cdc_ncm = CdcNcmClass::new(&usb_alloc, host_mac_addr, 64);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_alloc, UsbVidPid(0x1209, 0x0004))
        .manufacturer("pico-networking")
        .product("usb-ethernet")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build();

    let our_mac_addr = [0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
    let mut config = Config::new();
    config.hardware_addr = Some(EthernetAddress(our_mac_addr).into());
    // Todo set random seed
    // config.random_seed = todo!();

    let mut iface = Interface::new(config, &mut cdc_ncm);
    iface.update_ip_addrs(|ip_addrs| {
        ip_addrs
            .push(IpCidr::new(IpAddress::v4(192, 168, 69, 1), 24))
            .unwrap();
    });
    iface
        .routes_mut()
        .add_default_ipv4_route(Ipv4Address::new(192, 168, 69, 100))
        .unwrap();

    // Create sockets
    let mut dhcp_socket = dhcpv4::Socket::new();

    let server_socket = {
        // It is not strictly necessary to use a `static mut` and unsafe code here, but
        // on embedded systems that smoltcp targets it is far better to allocate the data
        // statically to verify that it fits into RAM rather than get undefined behavior
        // when stack overflows.
        static mut TCP_SERVER_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_SERVER_TX_DATA: [u8; 1024] = [0; 1024];
        let tcp_rx_buffer = tcp::SocketBuffer::new(unsafe { &mut TCP_SERVER_RX_DATA[..] });
        let tcp_tx_buffer = tcp::SocketBuffer::new(unsafe { &mut TCP_SERVER_TX_DATA[..] });
        tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer)
    };

    let mut sockets: [_; 2] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets[..]);
    let server_handle = sockets.add(server_socket);
    let dhcp_handle = sockets.add(dhcp_socket);

    let mut timestamp = Instant::ZERO;
    loop {
        usb_dev.poll(&mut [&mut cdc_ncm]);
        iface.poll(timestamp, &mut cdc_ncm, &mut sockets);

        // Todo replace with hardware clock
        timestamp += Duration::from_micros(100);

        if cdc_ncm.state() == State::Configured && cdc_ncm.data_if_enabled() {
            cdc_ncm.connect().ok();
        }

        let socket = sockets.get_mut::<tcp::Socket>(server_handle);
        if !socket.is_active() && !socket.is_listening() {
            debug!("listening on 1234");
            socket.listen(1234).unwrap();
        }

        if socket.can_recv() {
            let r: Result<&str, core::str::Utf8Error> = socket
                .recv(|buffer| (buffer.len(), core::str::from_utf8(buffer)))
                .unwrap();
            if let Ok(s) = r {
                debug!("got {:?}", s);
                socket.write_str("*").unwrap();
            }
            //socket.close();
        }

        let event = sockets.get_mut::<dhcpv4::Socket>(dhcp_handle).poll();
        match event {
            None => {}
            Some(dhcpv4::Event::Configured(config)) => {
                debug!("DHCP config acquired!");

                debug!("IP address:      {}", config.address);
                set_ipv4_addr(&mut iface, config.address);

                if let Some(router) = config.router {
                    debug!("Default gateway: {}", router);
                    iface.routes_mut().add_default_ipv4_route(router).unwrap();
                } else {
                    debug!("Default gateway: None");
                    iface.routes_mut().remove_default_ipv4_route();
                }

                for (i, s) in config.dns_servers.iter().enumerate() {
                    debug!("DNS server {}:    {}", i, s);
                }
            }
            Some(dhcpv4::Event::Deconfigured) => {
                debug!("DHCP lost config!");
                set_ipv4_addr(&mut iface, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
                iface.routes_mut().remove_default_ipv4_route();
            }
        }
    }
}

fn set_ipv4_addr(iface: &mut Interface, cidr: Ipv4Cidr) {
    iface.update_ip_addrs(|addrs| {
        let dest = addrs.iter_mut().next().unwrap();
        *dest = IpCidr::Ipv4(cidr);
    });
}
