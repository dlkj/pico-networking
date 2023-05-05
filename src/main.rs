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
use bsp::hal;
use defmt::{info, warn};
use defmt_rtt as _;
use embedded_hal::digital::v2::ToggleableOutputPin;
use hal::{
    clocks::init_clocks_and_plls,
    gpio::{bank0::Gpio25, Output, Pin, PushPull},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use panic_probe as _;
use rp_pico as bsp;
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
    let sio = Sio::new(pac.SIO);

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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

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

    loop {
        if usb_dev.poll(&mut [&mut cdc_ncm]) {
            poll(&mut cdc_ncm, &mut led_pin);
        }

        if cdc_ncm.state() == State::Configured && cdc_ncm.data_if_enabled() {
            cdc_ncm.connect().ok();
        }
    }
}

fn poll(
    cdc_ncm: &mut CdcNcmClass<'_, hal::usb::UsbBus>,
    led_pin: &mut Pin<Gpio25, Output<PushPull>>,
) {
    let mut buf = [0u8; 2048];
    match cdc_ncm.read_packet(&mut buf) {
        Err(_e) => {}
        Ok(count) => {
            info!("rxd {:02x}", &buf[..count]);

            // Toggle led if received 'a'
            if buf.contains(&b'a') {
                info!("toggle led");
                led_pin.toggle().unwrap();
            }

            // Convert to upper case
            buf.iter_mut().take(count).for_each(|b| {
                b.make_ascii_uppercase();
            });

            // Send back to the host
            // let mut wr_ptr = &buf[..count];
            // while !wr_ptr.is_empty() {
            //     match cdcNcm.write_packet(wr_ptr) {
            //         Ok(len) => {
            //             info!("txd {:02x}", &wr_ptr[..len]);
            //             wr_ptr = &wr_ptr[len..];
            //         }

            //         Err(e) => {
            //             warn!("write error: {:?}", e);
            //             break;
            //         }
            //     };
            // }
        }
    }
}
