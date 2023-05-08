use core::cell::Cell;

use defmt::{debug, error, warn, Format};
use heapless::{String, Vec};
use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
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
const CDC_TYPE_ETHERNET: u8 = 0x0F;
const CDC_TYPE_NCM: u8 = 0x1A;

const REQ_SET_INTERFACE: u8 = 11;

const REQ_GET_NTB_PARAMETERS: u8 = 0x80;
const REQ_SET_NTB_INPUT_SIZE: u8 = 0x86;

const NTB_MAX_SIZE: usize = 2048;
const SIG_NTH: u32 = 0x484d_434e;
const SIG_NDP_NO_FCS: u32 = 0x304d_434e;
const SIG_NDP_WITH_FCS: u32 = 0x314d_434e;

const MAX_PACKET_SIZE: usize = 64; // TODO more to type level generic

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
    mac_address: String<12>,
    mac_address_idx: StringIndex,
    state: Cell<State>,
    ncm_in: NcmIn<'a, B>,
    ncm_out: NcmOut<'a, B>,
}

struct NcmIn<'a, B: UsbBus> {
    write_ep: EndpointIn<'a, B>,
    write_ntb_buffer: Vec<u8, NTB_MAX_SIZE>,
    write_ntb_sent: usize,
    seq: u16,
}
struct NcmOut<'a, B: UsbBus> {
    read_ep: EndpointOut<'a, B>,
    read_ntb_buffer: [u8; NTB_MAX_SIZE],
    read_ntb_size: usize,
    read_ntb_idx: Option<usize>,
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
            mac_address: s,
            mac_address_idx,
            state: Cell::default(),
            ncm_in: NcmIn {
                write_ep: alloc.bulk(max_packet_size),
                write_ntb_buffer: Vec::default(),
                write_ntb_sent: 0,
                seq: 0,
            },
            ncm_out: NcmOut {
                read_ep: alloc.bulk(max_packet_size),
                read_ntb_buffer: [0; NTB_MAX_SIZE],
                read_ntb_size: 0,
                read_ntb_idx: None,
            },
        }
    }

    pub fn connect(&mut self) -> Result<usize> {
        // TODO implement ConnectionSpeedChange 7.1

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
impl<'a, B: UsbBus> NcmIn<'a, B> {
    /// Writes a single packet into the IN endpoint.
    // pub fn write_packet(&mut self, data: &[u8]) -> Result<usize> {
    pub fn write_datagram<F, R>(&mut self, len: u16, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        const OUT_HEADER_LEN: u16 = 28;

        if !self.can_write() {
            return Err(UsbError::WouldBlock);
        }

        if usize::from(len + OUT_HEADER_LEN) > self.write_ntb_buffer.capacity() {
            return Err(UsbError::BufferOverflow);
        }

        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);

        self.write_ntb_sent = 0;

        // Write the header
        let header_fields: [&[u8]; 12] = [
            // NTH
            &SIG_NTH.to_le_bytes(),                // dwSignature
            &0x000c_u16.to_le_bytes(),             // wHeaderLength
            &seq.to_le_bytes(),                    // wSequence
            &(len + OUT_HEADER_LEN).to_le_bytes(), // wBlockLength
            &0x000c_u16.to_le_bytes(),             // wNdpIndex
            // NDP
            &SIG_NDP_NO_FCS.to_le_bytes(), // dwSignature
            &0x0010_u16.to_le_bytes(),     // wLength
            &0x0000u16.to_le_bytes(),      // wNextNdpIndex
            &OUT_HEADER_LEN.to_le_bytes(), // wDatagramIndex
            &len.to_le_bytes(),            // wDatagramLength
            &0x0000u16.to_le_bytes(),      // wZeroIndex
            &0x0000u16.to_le_bytes(),      // wZeroLength
        ];

        for s in header_fields {
            self.write_ntb_buffer
                .extend_from_slice(s)
                .map_err(|()| UsbError::BufferOverflow)?;
        }

        assert_eq!(self.write_ntb_buffer.len(), OUT_HEADER_LEN.into());
        self.write_ntb_buffer
            .resize_default((OUT_HEADER_LEN + len).into())
            .map_err(|()| UsbError::BufferOverflow)?;

        // Write the datagram
        let result = f(&mut self.write_ntb_buffer[OUT_HEADER_LEN.into()..]);

        match self.write_packet() {
            Err(UsbError::WouldBlock) | Ok(_) => Ok(result),
            Err(e) => Err(e),
        }
    }

    fn can_write(&mut self) -> bool {
        match self.write_packet() {
            Ok(_) => self.write_ntb_buffer.is_empty(),
            Err(_) => false,
        }
    }

    fn write_packet(&mut self) -> Result<()> {
        if self.write_ntb_buffer.is_empty() {
            Ok(())
        }
        // ZLP if % MAX_PACKET_SIZE
        else if self.write_ntb_sent == self.write_ntb_buffer.len() {
            self.write_ep.write(&[])?;
            // TODO Handle errors
            self.write_ntb_sent = 0;
            self.write_ntb_buffer.clear();
            Ok(())
        }
        // Full packet
        else if self.write_ntb_buffer.len() - self.write_ntb_sent >= MAX_PACKET_SIZE {
            self.write_ep.write(
                &self.write_ntb_buffer[self.write_ntb_sent..self.write_ntb_sent + MAX_PACKET_SIZE],
            )?;
            // TODO Handle errors
            self.write_ntb_sent += MAX_PACKET_SIZE;
            Err(UsbError::WouldBlock)
        } else {
            // Short packet
            self.write_ep
                .write(&self.write_ntb_buffer[self.write_ntb_sent..])?;
            self.write_ntb_sent = 0;
            self.write_ntb_buffer.clear();
            Ok(())
        }
    }
}

impl<'a, B: UsbBus> NcmOut<'a, B> {
    fn read_datagram<R, F>(&mut self, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        if !self.can_read() {
            debug!("can't read");
            return Err(UsbError::WouldBlock);
        }

        let result = if let Some(idx) = self.read_ntb_idx {
            // Read the datagram
            f(&mut self.read_ntb_buffer[idx..self.read_ntb_size])
        } else {
            return Err(UsbError::WouldBlock);
        };
        self.read_ntb_idx = None;
        self.read_ntb_size = 0;

        match self.read_packet() {
            Err(UsbError::WouldBlock) | Ok(_) => Ok(result),
            Err(e) => Err(e),
        }
    }

    /// Reads a single packet from the OUT endpoint.
    pub fn read_packet(&mut self) -> Result<()> {
        if self.read_ntb_idx.is_some() {
            return Ok(());
        }

        let n = self
            .read_ep
            .read(&mut self.read_ntb_buffer[self.read_ntb_size..])?;
        self.read_ntb_size += n;

        if n == usize::from(self.read_ep.max_packet_size()) && self.read_ntb_size <= NTB_MAX_SIZE {
            return Err(UsbError::WouldBlock);
        }

        let ntb = &self.read_ntb_buffer[..self.read_ntb_size];

        // Process NTB header
        let Some(ntb_header) = ntb.get(..12)
        else {
            warn!("Received too short NTB");
            self.read_ntb_size = 0;
            return Err(UsbError::ParseError);
        };
        let sig = u32::from_le_bytes(ntb_header[0..4].try_into().unwrap());
        if sig != SIG_NTH {
            warn!("Received bad NTH sig.");
            self.read_ntb_size = 0;
            return Err(UsbError::ParseError);
        }
        let ndp_idx = u16::from_le_bytes(ntb_header[10..12].try_into().unwrap()) as usize;

        // Process NTB Datagram Pointer
        let Some(ntb_datagram_pointer) = ntb.get(ndp_idx..ndp_idx + 12)
        else {
                warn!("NTH has an NDP pointer out of range.");
                self.read_ntb_size = 0;
                return Err(UsbError::ParseError);
        };
        let sig = u32::from_le_bytes(ntb_datagram_pointer[0..4].try_into().unwrap());
        if sig != SIG_NDP_NO_FCS && sig != SIG_NDP_WITH_FCS {
            warn!("Received bad NDP sig.");
            self.read_ntb_size = 0;
            return Err(UsbError::ParseError);
        }
        let datagram_index =
            u16::from_le_bytes(ntb_datagram_pointer[8..10].try_into().unwrap()) as usize;
        let datagram_len =
            u16::from_le_bytes(ntb_datagram_pointer[10..12].try_into().unwrap()) as usize;

        if datagram_index == 0 || datagram_len == 0 {
            // empty, ignore. This is allowed by the spec, so don't warn.
            self.read_ntb_size = 0;
            debug!("empty datagram");
            return Err(UsbError::WouldBlock);
        }

        if ntb
            .get(datagram_index..datagram_index + datagram_len)
            .is_none()
        {
            warn!("NDP has a datagram pointer out of range.");
            self.read_ntb_size = 0;

            return Err(UsbError::ParseError);
        };

        self.read_ntb_idx = Some(datagram_index);
        Ok(())
    }

    fn can_read(&mut self) -> bool {
        match self.read_packet() {
            Ok(_) => self.read_ntb_idx.is_some(),
            Err(_) => false,
        }
    }
}

impl<B: UsbBus> UsbClass<B> for CdcNcmClass<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        // Interface Association Descriptor

        writer.iad(
            self.comm_if,
            2,
            USB_CLASS_CDC,
            CDC_SUBCLASS_NCM,
            CDC_PROTOCOL_NONE,
        )?;

        // Communication Class Interface (interface n)
        // Functional descriptors for the Communication Class Interface

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

        // Endpoint descriptors for the Communication Class Interface

        writer.endpoint(&self.comm_ep)?;

        // Data Class Interface (interface n+1, alternate setting 0)
        // Functional descriptors for Data Class Interface (interface n+1, alternate setting 0)

        writer.interface_alt(
            self.data_if,
            0,
            USB_CLASS_CDC_DATA,
            0x00,
            CDC_PROTOCOL_NTB,
            None,
        )?;

        // Data Class Interface (interface n+1, alternate setting 1)
        // Functional descriptors for Data Class Interface (interface n+1, alternate setting 1)

        writer.interface_alt(
            self.data_if,
            1,
            USB_CLASS_CDC_DATA,
            0x00,
            CDC_PROTOCOL_NTB,
            None,
        )?;

        // Endpoint descriptors for Data Class Interface (interface n+1, alternate setting 1)

        writer.endpoint(&self.ncm_in.write_ep)?;
        writer.endpoint(&self.ncm_out.read_ep)?;

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

                        // TODO do a reset as per 7.2
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

    fn reset(&mut self) {
        // TODO
    }
}

impl<'a, B: UsbBus> Device for CdcNcmClass<'a, B> {
    type RxToken<'b> = NcmRxToken<'a, 'b, B> where
    Self: 'b;
    type TxToken<'b> = NcmTxToken<'a, 'b, B> where
    Self: 'b;

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if self.ncm_in.can_write() && self.ncm_out.can_read() {
            Some((
                NcmRxToken::new(&mut self.ncm_out),
                NcmTxToken::new(&mut self.ncm_in),
            ))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        if self.ncm_in.can_write() {
            Some(NcmTxToken::new(&mut self.ncm_in))
        } else {
            None
        }
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1536;
        caps.max_burst_size = Some(1);
        caps.medium = Medium::Ethernet;
        caps
    }
}

pub struct NcmRxToken<'a, 'b, B: UsbBus> {
    ncm: &'b mut NcmOut<'a, B>,
}
impl<'a, 'b, B: UsbBus> NcmRxToken<'a, 'b, B> {
    fn new(ncm: &'b mut NcmOut<'a, B>) -> Self {
        Self { ncm }
    }
}

impl<'a, 'b, B: UsbBus> phy::RxToken for NcmRxToken<'a, 'b, B> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.ncm.read_datagram(f).unwrap()
    }
}

pub struct NcmTxToken<'a, 'b, B: UsbBus> {
    ncm: &'b mut NcmIn<'a, B>,
}
impl<'a, 'b, B: UsbBus> NcmTxToken<'a, 'b, B> {
    fn new(ncm: &'b mut NcmIn<'a, B>) -> Self {
        Self { ncm }
    }
}

impl<'a, 'b, B: UsbBus> phy::TxToken for NcmTxToken<'a, 'b, B> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.ncm.write_datagram(len.try_into().unwrap(), f).unwrap()
    }
}
