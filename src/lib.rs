#![no_std]

extern crate defmt_rtt;
extern crate atsamd_hal;

use atsamd_hal::hal::serial::Write; // embedded_hal serial traits
use atsamd_hal::{
    sercom::v2::uart::{
        self,
        Enable,
        Rx,
        Tx,
        TxOrRx,
        UartRx,
        UartTx,
    },
    time::Hertz,
};

// TODO use const generics, so our customers don't need to see this
pub use bbqueue::{
    consts::*,
};
// See note in Cargo.toml as to why this not heapless
use bbqueue::{
    BBBuffer,
    Consumer,
    Error,
    Producer,
};
// use core::borrow::BorrowMut;
use core::cell::Cell;
use core::convert::TryInto;
use cortex_m::interrupt::{
    self,
    Mutex,
};
use usb_device::class_prelude::*;
use usb_device::Result;

/// This should be used as `device_class` when building the `UsbDevice`.
pub const USB_CLASS_CDC: u8 = 0x02;

const USB_CLASS_CDC_DATA: u8 = 0x0a;
const CDC_SUBCLASS_ACM: u8 = 0x02;
const CDC_PROTOCOL_NONE: u8 = 0x00;

const CS_INTERFACE: u8 = 0x24;
const CDC_TYPE_HEADER: u8 = 0x00;
const CDC_TYPE_CALL_MANAGEMENT: u8 = 0x01;
const CDC_TYPE_ACM: u8 = 0x02;
const CDC_TYPE_UNION: u8 = 0x06;

const REQ_SEND_ENCAPSULATED_COMMAND: u8 = 0x00;
const REQ_SET_LINE_CODING: u8 = 0x20;
const REQ_GET_LINE_CODING: u8 = 0x21;
const REQ_SET_CONTROL_LINE_STATE: u8 = 0x22;


/// TX and RX buffers used by the SerialPort
///
/// Due to the BBQueue API, combined with Rust's rules about structs that
/// contain references to other members in the struct, we need a separate struct
/// to contain the BBQueue storage.  This structure should never be moved in
/// memory once it's in use, because any outstanding grant from before the move
/// would point to memory which is no longer inside the buffer.
pub struct SerialPortStorage {
    /// For data from the UART to the USB; from the DCE to DTE, device to host
    rx_buffer: BBBuffer<U256>,
    /// Other direction from `rx_buffer`
    tx_buffer: BBBuffer<U256>,
}

impl SerialPortStorage {
    pub fn new() -> Self {
        Self {
            rx_buffer: BBBuffer::new(),
            tx_buffer: BBBuffer::new(),
        }
    }
}

/// A USB CDC to hardware UART serial port
pub struct SerialPort<'a, B, C, const ENDPOINT_SIZE: usize>
where
    B: UsbBus,
    C: uart::ValidConfig,
    C::Pads: Rx + Tx + TxOrRx,
{
    comm_if: InterfaceNumber,
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    line_coding: LineCoding,
    dtr: bool,
    rts: bool,

    rx_consumer: Consumer<'a, U256>,
    tx_producer: Producer<'a, U256>,

    write_state: WriteState,
    uart_rx: UartRx<C>,
    uart_tx: UartTx<C>,

    /// Whether we're waiting on a transmit complete callback from the UART
    ///
    /// This is manipulated in both USB and UART interrupt contexts.
    tx_active: Mutex<Cell<bool>>,
}

/// If this many full size packets have been sent in a row, a short packet will
/// be sent so that the host sees the data in a timely manner.
const SHORT_PACKET_INTERVAL: usize = 10;

/// Keeps track of the type of the last written packet.
enum WriteState {
    /// The last packet written wasn't a full packet
    NotFull,

    /// Full packet current in-flight. A full packet must be followed by a short
    /// packet for the host OS to see the transaction. The data is the number of
    /// subsequent full packets sent so far. A short packet is forced every
    /// SHORT_PACKET_INTERVAL packets so that the OS sees data in a timely
    /// manner.
    Full(usize),
}

impl<'a, B, C, const ENDPOINT_SIZE: usize> SerialPort<'a, B, C, ENDPOINT_SIZE>
where
    B: UsbBus,
    C: uart::ValidConfig,
    C::Pads: Rx + Tx + TxOrRx,
{
    /// Creates a new USB serial port
    // TODO make the uart generic
    pub fn new(alloc: &'a UsbBusAllocator<B>, storage: &'a SerialPortStorage, uart_hardware: C) -> Self
    {
        let (mut rx_producer, mut rx_consumer) = storage.rx_buffer.try_split().unwrap();
        let (mut tx_producer, mut tx_consumer) = storage.tx_buffer.try_split().unwrap();

        let (uart_rx, uart_tx) = uart_hardware
            // .baud(Hertz(115200), uart::BaudMode::Arithmetic(uart::Oversampling::Bits16))
            .enable();

        Self {
            comm_if: alloc.interface(),
            comm_ep: alloc.interrupt(8, 255),
            data_if: alloc.interface(),
            read_ep: alloc.bulk(ENDPOINT_SIZE as u16),
            write_ep: alloc.bulk(ENDPOINT_SIZE as u16),
            line_coding: LineCoding {
                stop_bits: StopBits::One,
                data_bits: 8,
                parity_type: ParityType::None,
                data_rate: 8_000,
            },
            dtr: false,
            rts: false,

            rx_consumer,
            tx_producer,
            write_state: WriteState::NotFull,
            uart_rx,
            uart_tx,
            tx_active: Mutex::new(Cell::new(false)),
        }
    }

    // TODO perhaps a better name
    fn flush_usb(&mut self) {
        match self.rx_consumer.read() {
            // There is more data to write
            Ok(grant) => {
                // Deciding what to write is a bit more complicated,
                // due to the way USB bulk transfers work...
                let full_packet_count = match self.write_state {
                    WriteState::Full(c) => c,
                    WriteState::NotFull => 0,
                };

                let max_write_size = if full_packet_count >= SHORT_PACKET_INTERVAL {
                    ENDPOINT_SIZE-1
                } else {
                    ENDPOINT_SIZE
                };

                let write_slice = if grant.buf().len() > max_write_size {
                    grant.buf().split_at(max_write_size).0
                } else {
                    grant.buf()
                };

                match self.write_ep.write(write_slice) {
                    Ok(count) => {
                        // TODO it would be nice to release only after we get
                        // the endpoint_in_complete() callback, so we're sure
                        // the data was read by the host.
                        grant.release(count);

                        self.write_state = if count >= ENDPOINT_SIZE {
                            WriteState::Full(full_packet_count + 1)
                        } else {
                            WriteState::NotFull
                        };
                        // Ok(())
                    }

                    Err(UsbError::WouldBlock) => {}

                    Err(_) => {
                        defmt::error!("Error writing packet")
                        // Err(e)
                    }
                }
            }

            // No more data to write
            Err(Error::InsufficientSize) => {
                if let WriteState::Full(_) = self.write_state {
                    // Need to send a Zero Length Packet to
                    // signal the end of a transaction
                    match self.write_ep.write(&[]) {
                        Ok(_) => {
                            self.write_state = WriteState::NotFull;
                        }
                        Err(UsbError::WouldBlock) => {}
                        Err(_) => {
                            defmt::error!("Error writing ZLP")
                        }
                    }
                } else {
                    self.write_state = WriteState::NotFull;
                }

                // Ok(())
            }

            Err(_) => {
                // TODO handle this better
                defmt::error!("Couldn't get RX grant");
                // Ok(())
            }
        }
    }

    // TODO perhaps a better name
    fn flush_uart(&mut self) {
        // match self.uart_tx.write(42u8) { // +
        //     Ok(()) => {}
        //     Err(_) => {
        //         defmt::error!("error in flush_uart()"); // TODO
        //     }
        // };
        
    }

    pub fn uart_callback(&mut self) {
        // did we receive a byte?
        // did we send a byte
        if true {

        }
        interrupt::free(|cs| {
            self.tx_active.borrow(cs).replace(true);
        });
    }
}

impl<B, C, const ENDPOINT_SIZE: usize> UsbClass<B> for SerialPort<'_, B, C, ENDPOINT_SIZE>
where
    B: UsbBus,
    C: uart::ValidConfig,
    C::Pads: Rx + Tx + TxOrRx,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.iad(
            self.comm_if,
            2,
            USB_CLASS_CDC,
            CDC_SUBCLASS_ACM,
            CDC_PROTOCOL_NONE)?;

        writer.interface(
            self.comm_if,
            USB_CLASS_CDC,
            CDC_SUBCLASS_ACM,
            CDC_PROTOCOL_NONE)?;

        writer.write(
            CS_INTERFACE,
            &[
                CDC_TYPE_HEADER, // bDescriptorSubtype
                0x10, 0x01 // bcdCDC (1.10)
            ])?;

        writer.write(
            CS_INTERFACE,
            &[
                CDC_TYPE_ACM, // bDescriptorSubtype
                0x00 // bmCapabilities
            ])?;

        writer.write(
            CS_INTERFACE,
            &[
                CDC_TYPE_UNION, // bDescriptorSubtype
                self.comm_if.into(), // bControlInterface
                self.data_if.into() // bSubordinateInterface
            ])?;

        writer.write(
            CS_INTERFACE,
            &[
                CDC_TYPE_CALL_MANAGEMENT, // bDescriptorSubtype
                0x00, // bmCapabilities
                self.data_if.into() // bDataInterface
            ])?;

        writer.endpoint(&self.comm_ep)?;

        writer.interface(
            self.data_if,
            USB_CLASS_CDC_DATA,
            0x00,
            0x00)?;

        writer.endpoint(&self.write_ep)?;
        writer.endpoint(&self.read_ep)?;

        Ok(())
    }

    fn reset(&mut self) {
        self.line_coding = LineCoding::default();
        self.dtr = false;
        self.rts = false;

        // TODO
        // self.read_buf.clear();
        // self.write_buf.clear();
        self.write_state = WriteState::NotFull;
    }

    fn poll(&mut self) {
        // See if the host has more data to send over the UART
        match self.tx_producer.grant_exact(ENDPOINT_SIZE) {
            Ok(mut grant) => {
                match self.read_ep.read(grant.buf()) {
                    Ok(count) => {
                        grant.commit(count);
                    }
                    Err(UsbError::WouldBlock) => {
                        // No data to read, just drop the grant
                    },
                    Err(_) => {
                        // TODO handle this better
                        defmt::error!("Error reading TX data");
                    }
                };
            }
            Err(_) => {
                // TODO handle this better
                defmt::error!("Couldn't get TX grant");
            }
        }

        // Send the host any data we've received over the UART
        self.flush_usb();
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        defmt::warn!("TODO write complete, addr {:?}", u8::from(addr));
        self.flush_usb();
        // if addr == self.inner.write_ep_address() {
        //     self.flush_usb().ok();
        // }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();

        if !(req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && req.index == u8::from(self.comm_if) as u16)
        {
            return;
        }

        match req.request {
            REQ_GET_LINE_CODING if req.length == 7 => {
                xfer.accept(|data| {
                    data[0..4].copy_from_slice(&self.line_coding.data_rate.to_le_bytes());
                    data[4] = self.line_coding.stop_bits as u8;
                    data[5] = self.line_coding.parity_type as u8;
                    data[6] = self.line_coding.data_bits;

                    Ok(7)
                }).ok();
            },
            _ => { xfer.reject().ok(); },
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();

        if !(req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && req.index == u8::from(self.comm_if) as u16)
        {
            return;
        }

        match req.request {
            REQ_SEND_ENCAPSULATED_COMMAND => {
                // We don't actually support encapsulated commands but pretend
                // we do for standards compatibility.
                xfer.accept().ok();
            },
            REQ_SET_LINE_CODING if xfer.data().len() >= 7 => {
                self.line_coding.data_rate =
                    u32::from_le_bytes(xfer.data()[0..4].try_into().unwrap());
                self.line_coding.stop_bits = xfer.data()[4].into();
                self.line_coding.parity_type = xfer.data()[5].into();
                self.line_coding.data_bits = xfer.data()[6];

                xfer.accept().ok();
            },
            REQ_SET_CONTROL_LINE_STATE => {
                self.dtr = (req.value & 0x0001) != 0;
                self.rts = (req.value & 0x0002) != 0;

                xfer.accept().ok();
            },
            _ => { xfer.reject().ok(); }
        };
    }
}


/// Number of stop bits for LineCoding
///
/// Copied from usbd-serial
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum StopBits {
    /// 1 stop bit
    One = 0,

    /// 1.5 stop bits
    OnePointFive = 1,

    /// 2 stop bits
    Two = 2,
}

impl From<u8> for StopBits {
    fn from(value: u8) -> Self {
        match value {
            x if x == Self::One as u8 => Self::One,
            x if x == Self::OnePointFive as u8 => Self::OnePointFive,
            x if x == Self::Two as u8 => Self::Two,
            x => {
                defmt::error!("Unexpected stop bits integer: {:?}", x);
                Self::One
            }
        }
    }
}

/// Parity for LineCoding
///
/// Copied from usbd-serial
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum ParityType {
    None = 0,
    Odd = 1,
    Event = 2,
    Mark = 3,
    Space = 4,
}

impl From<u8> for ParityType {
    fn from(value: u8) -> Self {
        match value {
            x if x == Self::None as u8 => Self::None,
            x if x == Self::Odd as u8 => Self::Odd,
            x if x == Self::Event as u8 => Self::Event,
            x if x == Self::Mark as u8 => Self::Mark,
            x if x == Self::Space as u8 => Self::Space,
            x => {
                defmt::error!("Unexpected parity integer: {:?}", x);
                Self::None
            }
        }
    }
}

/// Line coding parameters
///
/// Copied from usbd-serial
pub struct LineCoding {
    stop_bits: StopBits,
    data_bits: u8,
    parity_type: ParityType,
    data_rate: u32,
}

impl LineCoding {
    /// Gets the number of stop bits for UART communication.
    pub fn stop_bits(&self) -> StopBits { self.stop_bits }

    /// Gets the number of data bits for UART communication.
    pub fn data_bits(&self) -> u8 { self.data_bits }

    /// Gets the parity type for UART communication.
    pub fn parity_type(&self) -> ParityType { self.parity_type }

    /// Gets the data rate in bits per second for UART communication.
    pub fn data_rate(&self) -> u32 { self.data_rate }
}

impl Default for LineCoding {
    fn default() -> Self {
        LineCoding {
            stop_bits: StopBits::One,
            data_bits: 8,
            parity_type: ParityType::None,
            data_rate: 8_000,
        }
    }
}
