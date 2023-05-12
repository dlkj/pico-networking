use defmt::{debug, error, info, warn, Format};
use heapless::{String, Vec};
use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
#[allow(clippy::wildcard_imports)]
use usb_device::class_prelude::*;
use usb_device::Result;

/// This should be used as `device_class` when building the `UsbDevice`.
pub const USB_CLASS_CDC: u8 = 0x02;

const NTB_MAX_SIZE: u32 = 2048;
const NTB_MAX_SIZE_USIZE: usize = NTB_MAX_SIZE as usize;
const MAX_SEGMENT_SIZE: u16 = 1514;

// TODO represent these as strings rather than endian dependant u32s
const SIG_NTH: u32 = 0x484d_434e;
const SIG_NDP_NO_FCS: u32 = 0x304d_434e;
const SIG_NDP_WITH_FCS: u32 = 0x314d_434e;

const MAX_PACKET_SIZE: usize = 64; // TODO change to type level generics

#[derive(Format, PartialEq, Eq, Clone, Copy)]
pub enum State {
    Disabled,
    Enabled,
    Connecting,
    Connected,
}

pub struct CdcNcmClass<'a, B: UsbBus> {
    comm_if: InterfaceNumber,
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    mac_address: String<12>,
    mac_address_idx: StringIndex,
    state: State,
    ncm_in: NcmIn<'a, B>,
    ncm_out: NcmOut<'a, B>,
}

struct NcmIn<'a, B: UsbBus> {
    write_ep: EndpointIn<'a, B>,
    write_ntb_buffer: Vec<u8, NTB_MAX_SIZE_USIZE>,
    write_ntb_sent: usize,
    seq: u16,
}
struct NcmOut<'a, B: UsbBus> {
    read_ep: EndpointOut<'a, B>,
    read_ntb_buffer: [u8; NTB_MAX_SIZE_USIZE],
    read_ntb_size: usize,
    read_ntb_idx: Option<usize>,
}

impl<'a, B: UsbBus> CdcNcmClass<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>, mac_address: [u8; 6], max_packet_size: u16) -> Self {
        let mac_address_idx = alloc.string();

        Self {
            comm_if: alloc.interface(),
            comm_ep: alloc.interrupt(8, 255),
            data_if: alloc.interface(),
            mac_address: mac_bytes_to_string(mac_address),
            mac_address_idx,
            state: State::Disabled,
            ncm_in: NcmIn {
                write_ep: alloc.bulk(max_packet_size),
                write_ntb_buffer: Vec::default(),
                write_ntb_sent: 0,
                seq: 0,
            },
            ncm_out: NcmOut {
                read_ep: alloc.bulk(max_packet_size),
                read_ntb_buffer: [0; NTB_MAX_SIZE_USIZE],
                read_ntb_size: 0,
                read_ntb_idx: None,
            },
        }
    }

    pub fn connect(&mut self) -> Result<()> {
        const REQ_TYPE_DEVICE_TO_HOST: u8 = 0xA1;
        const NETWORK_CONNECTION_CONNECTED: u8 = 0x01;
        const NOTE_TYPE_NETWORK_CONNECTION: u8 = 0x00;

        if self.state != State::Enabled {
            error!(
                "ncm: state must be Enabled to connect. State: {}",
                self.state
            );
            return Err(UsbError::InvalidState);
        }

        // const NOTE_TYPE_CONNECTION_SPEED_CHANGE: u8 = 0x2A;

        // TODO implement ConnectionSpeedChange 7.1

        let result = self.comm_ep.write(&[
            REQ_TYPE_DEVICE_TO_HOST,      // bmRequestType
            NOTE_TYPE_NETWORK_CONNECTION, // bNotificationType
            NETWORK_CONNECTION_CONNECTED, // wValue
            0x00,
            self.data_if.into(), // wIndex = interface
            0x00,
            0x00, // wLength
            0x00,
        ]);

        if result.is_ok() {
            debug!("ncm: connecting");
            self.state = State::Connecting;
        }

        result.map(drop)
    }

    pub fn state(&self) -> State {
        self.state
    }
}

fn mac_bytes_to_string(mac_address: [u8; 6]) -> String<12> {
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
    s
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

        if n == usize::from(self.read_ep.max_packet_size())
            && self.read_ntb_size <= NTB_MAX_SIZE_USIZE
        {
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
        const CDC_PROTOCOL_NONE: u8 = 0x00;
        const CDC_PROTOCOL_NTB: u8 = 0x01;
        const CDC_SUBCLASS_NCM: u8 = 0x0d;
        const CDC_TYPE_ETHERNET: u8 = 0x0F;
        const CDC_TYPE_HEADER: u8 = 0x00;
        const CDC_TYPE_NCM: u8 = 0x1A;
        const CS_INTERFACE: u8 = 0x24;
        const USB_CLASS_CDC_DATA: u8 = 0x0a;

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

        writer.write_with(CS_INTERFACE, |buf| {
            const LEN: usize = 3;
            if let Some(mut buf) = buf.get_mut(..LEN) {
                buf.put_u8(CDC_TYPE_HEADER); // bDescriptorSubtype
                buf.put_u16_le(0x0120); // bcdCDC (1.20)
                assert!(!buf.has_remaining_mut());
                Ok(LEN)
            } else {
                Err(UsbError::BufferOverflow)
            }
        })?;

        writer.write_with(CS_INTERFACE, |buf| {
            const LEN: usize = 11;
            if let Some(mut buf) = buf.get_mut(..LEN) {
                buf.put_u8(CDC_TYPE_ETHERNET); // bDescriptorSubtype
                buf.put_u8(self.mac_address_idx.into()); // iMACAddress
                buf.put_u32_le(0); // bmEthernetStatistics
                buf.put_u16_le(MAX_SEGMENT_SIZE); // wMaxSegmentSize
                buf.put_u16_le(0); // wNumberMCFilters
                buf.put_u8(0); // bNumberPowerFilters
                assert!(!buf.has_remaining_mut());
                Ok(LEN)
            } else {
                Err(UsbError::BufferOverflow)
            }
        })?;

        writer.write_with(CS_INTERFACE, |buf| {
            const LEN: usize = 4;
            if let Some(mut buf) = buf.get_mut(..LEN) {
                buf.put_u8(CDC_TYPE_NCM); // bDescriptorSubtype
                buf.put_u16_le(0x0100); // bcdCDC (1.00)
                buf.put_u8(0x00); // bmCapabilities - none
                assert!(!buf.has_remaining_mut());
                Ok(LEN)
            } else {
                Err(UsbError::BufferOverflow)
            }
        })?;

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

        debug!("ncm: configuration descriptors written");

        Ok(())
    }

    fn control_in(&mut self, transfer: ControlIn<B>) {
        const REQ_GET_NTB_PARAMETERS: u8 = 0x80;
        const REQ_GET_NTB_INPUT_SIZE: u8 = 0x85;

        let req = transfer.request();

        if req.recipient != control::Recipient::Interface {
            // Only handle interface requests
            return;
        }

        if req.index == u16::from(u8::from(self.data_if)) {
            warn!(
                "ncm: unhandled DATA_INTERFACE control_in {} {}",
                req.request_type, req.request
            );
            return;
        }

        if req.index != u16::from(u8::from(self.comm_if)) {
            warn!(
                "ncm: control_in unexpected interface {} - {} {}",
                req.index, req.request_type, req.request
            );
            return;
        }

        match (req.request_type, req.request) {
            (control::RequestType::Class, REQ_GET_NTB_PARAMETERS) => {
                debug!("ncm: REQ_GET_NTB_PARAMETERS");
                let _: Result<()> = transfer.accept(|data| {
                    const LEN: u16 = 28;
                    if let Some(mut data) = data.get_mut(..LEN.into()) {
                        data.put_u16_le(LEN); //wLength
                        data.put_u16_le(0x01); // bmNtbFormatsSupported - 16-bit NTB supported only
                        data.put_u32_le(NTB_MAX_SIZE); // dwNtbInMaxSize
                        data.put_u16_le(0x04); // wNdpInDivisor
                        data.put_u16_le(0x00); // wNdpInPayloadRemainder
                        data.put_u16_le(0x04); // wNdpInAlignment
                        data.put_u16_le(0x00); // wReserved
                        data.put_u32_le(NTB_MAX_SIZE); // dwNtbOutMaxSize
                        data.put_u16_le(0x04); // wNdpOutDivisor
                        data.put_u16_le(0x00); // wNdpOutPayloadRemainder
                        data.put_u16_le(0x04); // wNdpOutAlignment
                        data.put_u16_le(0x01); // wNtbOutMaxDatagrams
                        assert!(!data.has_remaining_mut());
                        Ok(LEN.into())
                    } else {
                        Err(UsbError::BufferOverflow)
                    }
                });
            }
            (control::RequestType::Class, REQ_GET_NTB_INPUT_SIZE) => {
                debug!("ncm: REQ_GET_NTB_INPUT_SIZE");
                let _: Result<()> = transfer.accept(|data| {
                    const LEN: usize = 4;

                    // We only support the minimum NTB maximum size so this can be a constant
                    if let Some(mut data) = data.get_mut(..LEN) {
                        data.put_u32_le(NTB_MAX_SIZE);
                        assert!(!data.has_remaining_mut()); // dwNtbInMaxSize
                        Ok(LEN)
                    } else {
                        Err(UsbError::BufferOverflow)
                    }
                });
            }
            _ => {
                warn!(
                    "ncm: unhandled COMMUNICATION interface control_in {} {}",
                    req.request_type, req.request
                );
            }
        }
    }

    fn control_out(&mut self, transfer: ControlOut<B>) {
        const REQ_SET_INTERFACE: u8 = 0x0B;
        const REQ_SET_NTB_INPUT_SIZE: u8 = 0x86;

        let req = transfer.request();

        if req.recipient != control::Recipient::Interface {
            // Only handle interface requests
            return;
        }

        if req.index == u16::from(u8::from(self.comm_if)) {
            match (req.request_type, req.request) {
                (control::RequestType::Class, REQ_SET_NTB_INPUT_SIZE) => {
                    debug!("ncm: REQ_SET_NTB_INPUT_SIZE");
                    // We only support the minimum NTB maximum size the value
                    // will always be NTB_MAX_SIZE
                    if transfer.data() != NTB_MAX_SIZE.to_le_bytes() {
                        warn!(
                            "ncp: unexpected REQ_SET_NTB_INPUT_SIZE data {}",
                            transfer.data()
                        );
                    }
                    let _: Result<()> = transfer.accept();
                }
                _ => {
                    warn!(
                        "ncm: unhandled COMMUNICATION interface control_out {} {}",
                        req.request_type, req.request
                    );
                }
            }
            return;
        }

        if req.index == u16::from(u8::from(self.data_if)) {
            match (req.request_type, req.request) {
                (control::RequestType::Standard, REQ_SET_INTERFACE) => {
                    debug!("ncm: REQ_SET_INTERFACE");
                    if req.value == 0 {
                        transfer.accept().ok();
                        self.state = State::Disabled;
                        info!("ncm: data interface disabled");
                        self.reset();
                        // TODO reset device as per 7.2
                    } else if req.value == 1 {
                        info!("ncm: data interface enabled");
                        self.state = State::Enabled;
                        transfer.accept().ok();
                    } else {
                        warn!("SET_INTERFACE out of range {}", req.request);
                        transfer.reject().ok();
                    }
                }
                _ => {
                    warn!(
                        "ncm: unhandled DATA_INTERFACE control_out {} {}",
                        req.request_type, req.request
                    );
                }
            }
            return;
        }

        warn!(
            "ncm: control_out unexpected interface {} - {} {}",
            req.index, req.request_type, req.request
        );
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.comm_ep.address() && self.state == State::Connecting {
            info!("ncm: connected");
            self.state = State::Connected;
        }
    }

    fn get_string(&self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        if index == self.mac_address_idx {
            Some(&self.mac_address)
        } else {
            warn!("ncm: unknown string index requested {}", index);
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
        if self.state == State::Connected && self.ncm_in.can_write() && self.ncm_out.can_read() {
            Some((
                NcmRxToken::new(&mut self.ncm_out),
                NcmTxToken::new(&mut self.ncm_in),
            ))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        if self.state == State::Connected && self.ncm_in.can_write() {
            Some(NcmTxToken::new(&mut self.ncm_in))
        } else {
            None
        }
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1500; // TODO where should this number come from?
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

// hat tip to tokio-rs/bytes
trait BufMut {
    fn remaining_mut(&self) -> usize;
    fn has_remaining_mut(&self) -> bool {
        self.remaining_mut() > 0
    }

    fn put_slice(&mut self, src: &[u8]);

    fn put_u8(&mut self, n: u8) {
        let src = [n];
        self.put_slice(&src);
    }

    fn put_u16_le(&mut self, n: u16) {
        self.put_slice(&n.to_le_bytes());
    }

    fn put_u32_le(&mut self, n: u32) {
        self.put_slice(&n.to_le_bytes());
    }

    fn put_u64_le(&mut self, n: u64) {
        self.put_slice(&n.to_le_bytes());
    }

    fn put_u128_le(&mut self, n: u128) {
        self.put_slice(&n.to_le_bytes());
    }
}

impl BufMut for &mut [u8] {
    fn remaining_mut(&self) -> usize {
        self.len()
    }

    fn put_slice(&mut self, src: &[u8]) {
        self[..src.len()].copy_from_slice(src);
        let (_, b) = core::mem::take(self).split_at_mut(src.len());
        *self = b;
    }
}
