use core::{cell::Cell, char::MAX, mem::size_of, ptr::copy_nonoverlapping};

use defmt::{debug, error, warn, Format};
use heapless::{String, Vec};
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

const REQ_SET_INTERFACE: u8 = 11;

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

const NTB_MAX_SIZE: usize = 2048;
const SIG_NTH: u32 = 0x484d_434e;
const SIG_NDP_NO_FCS: u32 = 0x304d_434e;
const SIG_NDP_WITH_FCS: u32 = 0x314d_434e;

#[derive(Default, Format, PartialEq, Eq, Clone, Copy)]
pub enum State {
    #[default]
    Init,
    Configured,
    Connected,
}

pub struct CdcNcmClass<'a, B: UsbBus> {
    comm_if: InterfaceNumber,
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    data_if_enabled: bool,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    mac_address: String<12>,
    mac_address_idx: StringIndex,
    state: Cell<State>,
    read_ntb_buffer: [u8; NTB_MAX_SIZE],
    read_ntb_size: usize,
    write_ntb_buffer: Vec<u8, NTB_MAX_SIZE>,
    write_ntb_sent: usize,
    seq: u16,
}

/// Simple NTB header (NTH+NDP all in one) for sending packets
#[repr(packed)]
#[allow(unused)]
struct NtbOutHeader {
    // NTH
    nth_sig: u32,
    nth_len: u16,
    nth_seq: u16,
    nth_total_len: u16,
    nth_first_index: u16,

    // NDP
    ndp_sig: u32,
    ndp_len: u16,
    ndp_next_index: u16,
    ndp_datagram_index: u16,
    ndp_datagram_len: u16,
    ndp_term1: u16,
    ndp_term2: u16,
}

fn byteify<T>(buf: &mut [u8], data: T) -> &[u8] {
    let len = size_of::<T>();
    unsafe { copy_nonoverlapping(&data as *const _ as *const u8, buf.as_mut_ptr(), len) }
    &buf[..len]
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
            data_if_enabled: false,
            read_ep: alloc.bulk(max_packet_size),
            write_ep: alloc.bulk(max_packet_size),
            mac_address: s,
            mac_address_idx,
            state: Cell::default(),
            read_ntb_buffer: [0; NTB_MAX_SIZE],
            read_ntb_size: 0,
            write_ntb_buffer: Vec::default(),
            write_ntb_sent: 0,
            seq: 0,
        }
    }

    // /// Gets the maximum packet size in bytes.
    // pub fn max_packet_size(&self) -> u16 {
    //     // The size is the same for both endpoints.
    //     self.read_ep.max_packet_size()
    // }

    /// Writes a single packet into the IN endpoint.
    pub fn write_packet(&mut self, data: &[u8]) -> Result<usize> {
        const OUT_HEADER_LEN: usize = 28;
        const MAX_PACKET_SIZE: usize = 64; // TODO unhardcode

        if self.write_ntb_buffer.is_empty() {
            let seq = self.seq;
            self.seq = self.seq.wrapping_add(1);

            let header = NtbOutHeader {
                nth_sig: SIG_NTH,
                nth_len: 0x0c,
                nth_seq: seq,
                nth_total_len: (data.len() + OUT_HEADER_LEN) as u16,
                nth_first_index: 0x0c,

                ndp_sig: SIG_NDP_NO_FCS,
                ndp_len: 0x10,
                ndp_next_index: 0x00,
                ndp_datagram_index: OUT_HEADER_LEN as u16,
                ndp_datagram_len: data.len() as u16,
                ndp_term1: 0x00,
                ndp_term2: 0x00,
            };

            let mut buf = [0; MAX_PACKET_SIZE];
            let n = byteify(&mut buf, header);
            assert_eq!(n.len(), OUT_HEADER_LEN);

            if OUT_HEADER_LEN + data.len() < MAX_PACKET_SIZE {
                // First packet is not full, just send it.
                // No need to send ZLP because it's short for sure.
                buf[OUT_HEADER_LEN..][..data.len()].copy_from_slice(data);
                self.write_ep.write(&buf[..OUT_HEADER_LEN + data.len()])?;
            } else {
                if data.len() > self.write_ntb_buffer.capacity() {
                    return Err(UsbError::BufferOverflow);
                }
                let (d1, d2) = data.split_at(MAX_PACKET_SIZE - OUT_HEADER_LEN);

                buf[OUT_HEADER_LEN..].copy_from_slice(d1);
                match self.write_ep.write(&buf) {
                    Ok(_) => {
                        self.write_ntb_buffer
                            .extend_from_slice(d2)
                            .map_err(|_| UsbError::BufferOverflow)?;

                        self.write_ntb_sent = 0;
                    }
                    Err(e) => {
                        return Err(e);
                    }
                }
            }
            return Ok(data.len());
        }

        // TODO replace write_ntb_sent with slice

        // ZLP if % MAX_PACKET_SIZE
        if self.write_ntb_sent == self.write_ntb_buffer.len() {
            self.write_ep.write(&[])?;
            // TODO Handle errors
            self.write_ntb_sent = 0;
            self.write_ntb_buffer.clear();
            return Err(UsbError::WouldBlock);
        }
        // Full packet
        else if self.write_ntb_buffer.len() - self.write_ntb_sent >= MAX_PACKET_SIZE {
            self.write_ep
                .write(&self.write_ntb_buffer[self.write_ntb_sent..MAX_PACKET_SIZE])?;
            // TODO Handle errors
            self.write_ntb_sent += MAX_PACKET_SIZE;
            return Err(UsbError::WouldBlock);
        }
        // Short packet
        self.write_ep
            .write(&self.write_ntb_buffer[self.write_ntb_sent..])?;
        self.write_ntb_buffer.clear();
        self.write_ntb_sent = 0;
        Err(UsbError::WouldBlock)
    }

    /// Reads a single packet from the OUT endpoint.
    pub fn read_packet(&mut self, data: &mut [u8]) -> Result<usize> {
        let n = self
            .read_ep
            .read(&mut self.read_ntb_buffer[self.read_ntb_size..])?;
        self.read_ntb_size += n;

        if n == usize::from(self.read_ep.max_packet_size()) && self.read_ntb_size <= NTB_MAX_SIZE {
            return Err(UsbError::WouldBlock);
        }

        let ntb = &self.read_ntb_buffer[..self.read_ntb_size];

        // Process NTB header (NTH)
        let nth = match ntb.get(..12) {
            Some(x) => x,
            None => {
                warn!("Received too short NTB");
                self.read_ntb_size = 0;
                return Err(UsbError::ParseError);
            }
        };
        let sig = u32::from_le_bytes(nth[0..4].try_into().unwrap());
        if sig != SIG_NTH {
            warn!("Received bad NTH sig.");
            self.read_ntb_size = 0;
            return Err(UsbError::ParseError);
        }
        let ndp_idx = u16::from_le_bytes(nth[10..12].try_into().unwrap()) as usize;

        // Process NTB Datagram Pointer (NDP)
        let ndp = match ntb.get(ndp_idx..ndp_idx + 12) {
            Some(x) => x,
            None => {
                warn!("NTH has an NDP pointer out of range.");
                self.read_ntb_size = 0;
                return Err(UsbError::ParseError);
            }
        };
        let sig = u32::from_le_bytes(ndp[0..4].try_into().unwrap());
        if sig != SIG_NDP_NO_FCS && sig != SIG_NDP_WITH_FCS {
            warn!("Received bad NDP sig.");
            self.read_ntb_size = 0;
            return Err(UsbError::ParseError);
        }
        let datagram_index = u16::from_le_bytes(ndp[8..10].try_into().unwrap()) as usize;
        let datagram_len = u16::from_le_bytes(ndp[10..12].try_into().unwrap()) as usize;

        if datagram_index == 0 || datagram_len == 0 {
            // empty, ignore. This is allowed by the spec, so don't warn.
            self.read_ntb_size = 0;
            return Err(UsbError::WouldBlock);
        }

        // Process actual datagram, finally.
        let datagram = match ntb.get(datagram_index..datagram_index + datagram_len) {
            Some(x) => x,
            None => {
                warn!("NDP has a datagram pointer out of range.");
                self.read_ntb_size = 0;

                return Err(UsbError::ParseError);
            }
        };
        data[..datagram_len].copy_from_slice(datagram);

        return Ok(datagram_len);
    }

    // /// Gets the address of the IN endpoint.
    // pub(crate) fn write_ep_address(&self) -> EndpointAddress {
    //     self.write_ep.address()
    // }

    pub fn connect(&mut self) -> Result<usize> {
        let result = self.comm_ep.write(&[
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
            *self.state.get_mut() = State::Connected;
        }

        result
    }

    pub fn state(&self) -> State {
        self.state.get()
    }

    pub fn data_if_enabled(&self) -> bool {
        self.data_if_enabled
    }
}

impl<B: UsbBus> UsbClass<B> for CdcNcmClass<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
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

        debug!("get_configuration_descriptors sent");
        if self.state.get() == State::Init {
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
                REQ_SET_INTERFACE => {
                    if req.value == 0 {
                        debug!("data interface disabled");
                        self.data_if_enabled = false;
                        transfer.accept().ok();
                    } else if req.value == 1 {
                        debug!("data interface enabled");
                        self.data_if_enabled = true;
                        transfer.accept().ok();
                    } else {
                        error!("SET_INTERFACE out of range {}", req.request);
                        transfer.reject().ok();
                    }
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
        if index == self.mac_address_idx {
            Some(&self.mac_address)
        } else {
            warn!("unknown string index requested");
            None
        }
    }
}
