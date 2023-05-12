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

use bsp::entry;
use bsp::hal::{self, rosc::RingOscillator, Timer};
use defmt::{debug, warn};
use defmt_rtt as _;
use hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};
use heapless::Vec;
use panic_probe as _;
use rp_pico as bsp;
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
    let dhcp_socket = dhcpv4::Socket::new();

    let server_socket = {
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

    let mut sockets: [_; 2] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets[..]);
    let server_handle = sockets.add(server_socket);
    let dhcp_handle = sockets.add(dhcp_socket);

    loop {
        if usb_dev.poll(&mut [&mut cdc_ncm]) && cdc_ncm.state() == State::Enabled {
            let _: usb_device::Result<()> = cdc_ncm.connect();
        }

        if cdc_ncm.state() == State::Connected {
            let timestamp =
                Instant::from_micros(i64::try_from(timer.get_counter().ticks()).unwrap());
            if iface.poll(timestamp, &mut cdc_ncm, &mut sockets) {
                server_poll(sockets.get_mut::<tcp::Socket>(server_handle));
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

fn server_poll(socket: &mut tcp::Socket) {
    if !socket.is_open() {
        debug!("listening on 1234");
        socket.listen(1234).unwrap();
    }

    if socket.may_recv() {
        let mut data = Vec::<u8, 1024>::new();
        socket
            .recv(|buffer| {
                if !buffer.is_empty() {
                    debug!("recv data: {:?}", buffer);
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
        debug!("socket close");
        socket.close();
    }
}

fn dhcp_poll(iface: &mut Interface, socket: &mut dhcpv4::Socket) {
    let event = socket.poll();
    match event {
        None => {}
        Some(dhcpv4::Event::Configured(config)) => {
            debug!("DHCP config acquired!");

            debug!("IP address:      {}", config.address);
            set_ipv4_addr(iface, config.address);

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
