use core::cell::Cell;

use defmt::{debug, warn, Format};
use heapless::String;
#[allow(clippy::wildcard_imports)]
use usb_device::class_prelude::*;
use usb_device::{control::RequestType, Result};

/// This should be used as `device_class` when building the `UsbDevice`.
pub const USB_CLASS_CDC: u8 = 0x02;

const USB_CLASS_CDC_DATA: u8 = 0x0a;
const CDC_SUBCLASS_NCM: u8 = 0x0d;

const CDC_PROTOCOL_NONE: u8 = 0x00;
const CDC_PROTOCOL_NTB: u8 = 0x01;

const CS_INTERFACE: u8 = 0x24;
const CDC_TYPE_HEADER: u8 = 0x00;
const CDC_TYPE_UNION: u8 = 0x06;
const CDC_TYPE_ETHERNET: u8 = 0x0F;
const CDC_TYPE_NCM: u8 = 0x1A;

const REQ_SEND_ENCAPSULATED_COMMAND: u8 = 0x00;
//const REQ_GET_ENCAPSULATED_COMMAND: u8 = 0x01;
//const REQ_SET_ETHERNET_MULTICAST_FILTERS: u8 = 0x40;
//const REQ_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER: u8 = 0x41;
//const REQ_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER: u8 = 0x42;
//const REQ_SET_ETHERNET_PACKET_FILTER: u8 = 0x43;
//const REQ_GET_ETHERNET_STATISTIC: u8 = 0x44;
const REQ_GET_NTB_PARAMETERS: u8 = 0x80;
//const REQ_GET_NET_ADDRESS: u8 = 0x81;
//const REQ_SET_NET_ADDRESS: u8 = 0x82;
//const REQ_GET_NTB_FORMAT: u8 = 0x83;
//const REQ_SET_NTB_FORMAT: u8 = 0x84;
//const REQ_GET_NTB_INPUT_SIZE: u8 = 0x85;
const REQ_SET_NTB_INPUT_SIZE: u8 = 0x86;
//const REQ_GET_MAX_DATAGRAM_SIZE: u8 = 0x87;
//const REQ_SET_MAX_DATAGRAM_SIZE: u8 = 0x88;
//const REQ_GET_CRC_MODE: u8 = 0x89;
//const REQ_SET_CRC_MODE: u8 = 0x8A;

//const NOTIF_MAX_PACKET_SIZE: u16 = 8;
//const NOTIF_POLL_INTERVAL: u8 = 20;

const NTB_MAX_SIZE: u32 = 2048;
// const SIG_NTH: u32 = 0x484d_434e;
// const SIG_NDP_NO_FCS: u32 = 0x304d_434e;
// const SIG_NDP_WITH_FCS: u32 = 0x314d_434e;

#[derive(Default, Format, PartialEq, Eq, Clone, Copy)]
pub enum State {
    #[default]
    Init,
    Configured,
    Enabled,
}

pub struct CdcNcmClass<'a, B: UsbBus> {
    comm_if: InterfaceNumber,
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    mac_address: String<12>,
    mac_address_idx: StringIndex,
    state: Cell<State>,
}

impl<'a, B: UsbBus> CdcNcmClass<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>, mac_address: [u8; 6], max_packet_size: u16) -> Self {
        let mac_address_idx = alloc.string();

        let mut s = String::new();
        for i in 0..12 {
            let n = (mac_address[i / 2] >> ((1 - i % 2) * 4)) & 0xF;
            match n {
                0x0..=0x9 => s.push(char::from(b'0' + n)),
                0xA..=0xF => s.push(char::from(b'A' + n - 0xA)),
                _ => unreachable!(),
            }
            .unwrap();
        }

        Self {
            comm_if: alloc.interface(),
            comm_ep: alloc.interrupt(8, 255),
            data_if: alloc.interface(),
            read_ep: alloc.bulk(max_packet_size),
            write_ep: alloc.bulk(max_packet_size),
            mac_address: s,
            mac_address_idx,
            state: Cell::default(),
        }
    }

    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.read_ep.max_packet_size()
    }

    /// Writes a single packet into the IN endpoint.
    pub fn write_packet(&mut self, data: &[u8]) -> Result<usize> {
        self.write_ep.write(data)
    }

    /// Reads a single packet from the OUT endpoint.
    pub fn read_packet(&mut self, data: &mut [u8]) -> Result<usize> {
        self.read_ep.read(data)
    }

    /// Gets the address of the IN endpoint.
    pub(crate) fn write_ep_address(&self) -> EndpointAddress {
        self.write_ep.address()
    }

    pub fn connect(&mut self) -> Result<usize> {
        let result = self.write_packet(&[
            0xA1, //bmRequestType
            0x00, //bNotificationType = NETWORK_CONNECTION
            0x01, // wValue = connected
            0x00,
            self.data_if.into(), // wIndex = interface
            0x00,
            0x00, // wLength
            0x00,
        ]);

        if result.is_ok() && self.state.get_mut() == &State::Configured {
            debug!("Sent connect - enabled");
            *self.state.get_mut() = State::Enabled;
        }

        result
    }

    pub fn state(&self) -> State {
        self.state.get()
    }
}

impl<B: UsbBus> UsbClass<B> for CdcNcmClass<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        debug!("get_configuration_descriptors start");
        writer.iad(
            self.comm_if,
            2,
            USB_CLASS_CDC,
            CDC_SUBCLASS_NCM,
            CDC_PROTOCOL_NONE,
        )?;

        // Control interface

        writer.interface(
            self.comm_if,
            USB_CLASS_CDC,
            CDC_SUBCLASS_NCM,
            CDC_PROTOCOL_NONE,
        )?;

        writer.write(
            CS_INTERFACE,
            &[
                CDC_TYPE_HEADER, // bDescriptorSubtype
                0x10,
                0x01, // bcdCDC (1.10)
            ],
        )?;

        writer.write(
            CS_INTERFACE,
            &[
                CDC_TYPE_UNION,      // bDescriptorSubtype
                self.comm_if.into(), // bControlInterface
                self.data_if.into(), // bSubordinateInterface
            ],
        )?;

        writer.write(
            CS_INTERFACE,
            &[
                CDC_TYPE_ETHERNET,           // bDescriptorSubtype
                self.mac_address_idx.into(), // iMACAddress
                0,                           // bmEthernetStatistics
                0,                           // |
                0,                           // |
                0,                           // |
                0xea,                        // wMaxSegmentSize = 1514
                0x05,                        // |
                0,                           // wNumberMCFilters
                0,                           // |
                0,                           // bNumberPowerFilters
            ],
        )?;

        writer.write(
            CS_INTERFACE,
            &[
                CDC_TYPE_NCM, // bDescriptorSubtype
                0x00,         // bcdNCMVersion
                0x01,         // |
                0x00,         // bmCapabilities
            ],
        )?;

        writer.endpoint(&self.comm_ep)?;

        // Data interface

        writer.interface_alt(
            self.data_if,
            0,
            USB_CLASS_CDC_DATA,
            0x00,
            CDC_PROTOCOL_NTB,
            None,
        )?;

        writer.interface_alt(
            self.data_if,
            1,
            USB_CLASS_CDC_DATA,
            0x00,
            CDC_PROTOCOL_NTB,
            None,
        )?;

        writer.endpoint(&self.write_ep)?;
        writer.endpoint(&self.read_ep)?;

        debug!("get_configuration_descriptors done");
        if self.state.get() == State::Init {
            debug!("Configured");
            self.state.set(State::Configured);
        }

        Ok(())
    }

    fn control_in(&mut self, transfer: ControlIn<B>) {
        let req = transfer.request();

        if (req.recipient, req.index)
            == (
                control::Recipient::Interface,
                u16::from(u8::from(self.data_if)),
            )
        {
            debug!(
                "ncm: unhandled DATA_INTERFACE control_in {} {}",
                req.request_type, req.request
            );

            return;
        }

        if (req.recipient, req.index)
            != (
                control::Recipient::Interface,
                u16::from(u8::from(self.comm_if)),
            )
        {
            return;
        }

        if req.request_type != RequestType::Class {
            debug!(
                "ncm: unhandled control_in {} {}",
                req.request_type, req.request
            );
            return;
        }

        match req.request {
            REQ_GET_NTB_PARAMETERS => {
                debug!("control_in REQ_GET_NTB_PARAMETERS");

                transfer
                    .accept(|data| {
                        data[0..2].copy_from_slice(&28u16.to_le_bytes()); // length
                        data[2..4].copy_from_slice(&1u16.to_le_bytes()); // bmNtbFormatsSupported - 16-bit
                        data[4..8].copy_from_slice(&NTB_MAX_SIZE.to_le_bytes()); // dwNtbInMaxSize
                        data[8..10].copy_from_slice(&4u16.to_le_bytes()); // wNdpInDivisor
                        data[10..12].copy_from_slice(&0u16.to_le_bytes()); // wNdpInPayloadRemainder
                        data[12..14].copy_from_slice(&4u16.to_le_bytes()); // wNdpInAlignment
                        data[14..16].copy_from_slice(&0u16.to_le_bytes()); // reserved
                        data[16..20].copy_from_slice(&NTB_MAX_SIZE.to_le_bytes()); // dwNtbOutMaxSize
                        data[20..22].copy_from_slice(&4u16.to_le_bytes()); // wNdpOutDivisor
                        data[22..24].copy_from_slice(&0u16.to_le_bytes()); // wNdpOutPayloadRemainder
                        data[24..26].copy_from_slice(&4u16.to_le_bytes()); // wNdpOutAlignment
                        data[26..28].copy_from_slice(&1u16.to_le_bytes()); // wNtbOutMaxDatagrams
                        data[28] = 0xFF;
                        Ok(28)
                    })
                    .ok();
            }
            e => {
                debug!("ncm: control_in rejected {}", e);
                transfer.reject().ok();
            }
        }
    }

    fn control_out(&mut self, transfer: ControlOut<B>) {
        let req = transfer.request();

        if (req.recipient, req.index)
            == (
                control::Recipient::Interface,
                u16::from(u8::from(self.data_if)),
            )
        {
            match req.request {
                11 => {
                    debug!("data interface set alternate setting {}", req.value);
                    transfer.accept().ok();
                }
                _ => {
                    debug!(
                        "ncm: unhandled DATA_INTERFACE control_out {} {}",
                        req.request_type, req.request
                    );
                }
            }
            return;
        }

        if (req.recipient, req.index)
            != (
                control::Recipient::Interface,
                u16::from(u8::from(self.comm_if)),
            )
        {
            return;
        }

        if req.request_type != RequestType::Class {
            debug!(
                "ncm: unhandled control_out {} {}",
                req.request_type, req.request
            );
            return;
        }

        match req.request {
            REQ_SEND_ENCAPSULATED_COMMAND => {
                debug!("ncm: OUT REQ_SEND_ENCAPSULATED_COMMAND");
                // We don't actually support encapsulated commands but pretend we do for standards
                // compatibility.
                transfer.accept().ok();
            }
            REQ_SET_NTB_INPUT_SIZE => {
                debug!("ncm: OUT REQ_SET_NTB_INPUT_SIZE");
                // TODO
                transfer.accept().ok();
            }
            e => {
                debug!("ncm: control_out rejected {}", e);
                transfer.reject().ok();
            }
        };
    }

    fn get_string(&self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        debug!("get_string");

        if index == self.mac_address_idx {
            debug!("mac: {:?}", self.mac_address);
            Some(&self.mac_address)
        } else {
            warn!("unknown string index requested");
            None
        }
    }
}
