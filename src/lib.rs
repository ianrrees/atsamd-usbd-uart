#![no_std]

extern crate atsamd_hal;

use atsamd_hal::{
    hal::serial::{
        // embedded_hal serial traits
        Read,
        Write,
    },
    prelude::*,
    sercom::v2::uart::{
        self, BaudMode, Config, Duplex, EightBit, Error, Flags, Parity, Status, StopBits, Uart,
        ValidPads,
    },
    time::Hertz,
};

// TODO use const generics, so our customers don't need to see this
pub use bbqueue::consts::*;

// See note in Cargo.toml as to why this not heapless
use bbqueue::{BBBuffer, Consumer, Error as BBError, Producer};

use core::convert::TryInto;

use usb_device::{class_prelude::*, Result};

/// This should be used as `device_class` when building the `UsbDevice`.
pub const USB_CLASS_CDC: u8 = 0x02;

const USB_CLASS_CDC_DATA: u8 = 0x0a;
const CDC_SUBCLASS_ACM: u8 = 0x02; // PSTN Abstract Control Model
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

const CDC_NOTIFICATION_REQ_TYPE: u8 = 0xA1;
const CDC_SERIAL_STATE_NOTIFICATION: u8 = 0x20;

// USB CDC Class, PSTN SubClass, 6.5.4.  Using u8 instead of u16, because that
// makes bbqueue a better fit, and this seems highly unlikely to change
const CDC_SERIAL_STATE_OVER_RUN: u8 = 1 << 6;
const CDC_SERIAL_STATE_PARITY_ERROR: u8 = 1 << 5;
const CDC_SERIAL_STATE_FRAMING_ERROR: u8 = 1 << 4;

// TODO Only need this because we can't read settings back out of the Uart
struct LineCoding {
    stop_bits: StopBits,
    parity: Option<Parity>,
    baud: u32,
}

impl Default for LineCoding {
    fn default() -> Self {
        LineCoding {
            stop_bits: StopBits::OneBit,
            parity: None,
            baud: 115200,
        }
    }
}

const BAUDMODE: BaudMode = BaudMode::Arithmetic(uart::Oversampling::Bits16);

/// TX and RX buffers used by the UsbUart
///
/// Due to the BBQueue API, combined with Rust's rules about structs that
/// contain references to other members in the struct, we need a separate struct
/// to contain the BBQueue storage.  This structure should never be moved in
/// memory once it's in use, because any outstanding grant from before the move
/// would point to memory which is no longer inside the buffer.
pub struct UsbUartStorage {
    /// For data from the UART to the USB; from the DCE to DTE, device to host
    rx_buffer: BBBuffer<U256>,
    /// Other direction from `rx_buffer`
    tx_buffer: BBBuffer<U256>,
    /// For reporting Overflow/Parity/Framing errors, size is arbitrary
    error_buffer: BBBuffer<U16>,
}

impl UsbUartStorage {
    pub fn new() -> Self {
        Self {
            rx_buffer: BBBuffer::new(),
            tx_buffer: BBBuffer::new(),
            error_buffer: BBBuffer::new(),
        }
    }
}

/// A USB CDC to hardware UART serial port
pub struct UsbUart<'a, B, P, const ENDPOINT_SIZE: usize>
where
    B: UsbBus,
    P: ValidPads<Capability = Duplex>,
{
    comm_if: InterfaceNumber,
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    line_coding: LineCoding,

    /// User-supplied callback, when the USB host sets/clears DTR
    dtr_callback: Option<fn(bool)>,

    /// User-supplied callback, when the USB host sets/clears RTS
    rts_callback: Option<fn(bool)>,

    /// UART end of the UART->USB buffer
    uart_to_usb_producer: Producer<'a, U256>,

    /// USB end of the UART->USB buffer
    uart_to_usb_consumer: Consumer<'a, U256>,

    /// USB end of the USB->UART buffer
    ///
    /// Whenever data is committed, the UART DRE interrupt should be enabled to
    /// trigger writing the data to the SERCOM.  When the ISR is run and there
    /// isn't more data to send, the ISR disables DRE.
    usb_to_uart_producer: Producer<'a, U256>,

    /// UART end of the USB->UART buffer
    usb_to_uart_consumer: Consumer<'a, U256>,

    /// UART end of the error queue
    error_producer: Producer<'a, U16>,
 
    /// USB end of the error queue
    error_consumer: Consumer<'a, U16>,

    write_state: WriteState,

    /// The enabled UART hardware
    uart: Uart<Config<P, EightBit>, Duplex>,
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

impl<'a, B, P, const ENDPOINT_SIZE: usize> UsbUart<'a, B, P, ENDPOINT_SIZE>
where
    B: UsbBus,
    P: ValidPads<Capability = Duplex>,
{
    pub fn new(
        alloc: &'a UsbBusAllocator<B>,
        storage: &'a UsbUartStorage,
        uart_hardware: Config<P, EightBit>,
        dtr_callback: Option<fn(bool)>,
        rts_callback: Option<fn(bool)>,
    ) -> Self {
        let (uart_to_usb_producer, uart_to_usb_consumer) = storage.rx_buffer.try_split().unwrap();
        let (usb_to_uart_producer, usb_to_uart_consumer) = storage.tx_buffer.try_split().unwrap();
        let (error_producer, error_consumer) = storage.error_buffer.try_split().unwrap();

        let mut uart = uart_hardware
            .baud(Hertz(LineCoding::default().baud), BAUDMODE)
            .stop_bit(LineCoding::default().stop_bits)
            .parity(LineCoding::default().parity)
            .enable();

        uart.enable_interrupts(Flags::RXC);

        Self {
            comm_if: alloc.interface(),
            // Need 10B for SerialState transfers, hardware does 8B, 16B, 32B...
            comm_ep: alloc.interrupt(16, 255),
            data_if: alloc.interface(),
            read_ep: alloc.bulk(ENDPOINT_SIZE as u16),
            write_ep: alloc.bulk(ENDPOINT_SIZE as u16),
            line_coding: LineCoding::default(),

            dtr_callback,
            rts_callback,

            uart_to_usb_producer,
            uart_to_usb_consumer,
            usb_to_uart_producer,
            usb_to_uart_consumer,
            error_producer,
            error_consumer,

            write_state: WriteState::NotFull,
            uart,
        }
    }
    
    // TODO perhaps a better name
    pub fn flush_usb(&mut self) {
        match self.error_consumer.read() {
            Ok(grant) => {
                let mut buf = [0; 10];
                buf[0] = CDC_NOTIFICATION_REQ_TYPE; // bmRequestType
                buf[1] = CDC_SERIAL_STATE_NOTIFICATION; // bNotification
                buf[4] = u8::from(self.comm_if); // wIndex
                buf[6] = 2; // wLength
                // Data (grant is guaranteed to have at least one byte)
                buf[8] = grant.buf()[0];
            
                match self.comm_ep.write(&buf) {
                    Ok(count) => {
                        if count != buf.len() {
                            defmt::warn!("Write of serial state had unexpected length {:?}", count);
                        }
                        grant.release(1);
                    }
                    Err(UsbError::WouldBlock) => {}
                    Err(_) => {
                        defmt::info!("Error writing Serial State");
                    }
                }
            }
            Err(BBError::InsufficientSize) => {
                // No errors, yay!
            }
            Err(_) => {
                defmt::error!("Couldn't get error_consumer grant");
            }
        }
        match self.uart_to_usb_consumer.read() {
            // There is more data to write
            Ok(grant) => {
                // Deciding what to write is a bit more complicated,
                // due to the way USB bulk transfers work...
                let full_packet_count = match self.write_state {
                    WriteState::Full(c) => c,
                    WriteState::NotFull => 0,
                };

                let max_write_size = if full_packet_count >= SHORT_PACKET_INTERVAL {
                    ENDPOINT_SIZE - 1
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

                        // NOP normally, but re-enables the interrupts if the
                        // buffer filled up.  Not racy, because the SERCOM ISR
                        // has to be higher priority than the USB one we're in.
                        self.uart.enable_interrupts(Flags::RXC);

                        self.write_state = if count >= ENDPOINT_SIZE {
                            WriteState::Full(full_packet_count + 1)
                        } else {
                            WriteState::NotFull
                        };
                    }

                    Err(UsbError::WouldBlock) => {}

                    Err(_) => {
                        defmt::error!("Error writing packet")
                    }
                }
            }

            // No more data to write
            Err(BBError::InsufficientSize) => {
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
            }

            Err(_) => {
                // TODO handle this better
                defmt::error!("Couldn't get uart_to_usb_consumer grant");
            }
        }
    }

    /// Best effort at reporting the error code
    ///
    /// These wind up being reported as bit flags in interrupt transfers, so
    /// there isn't a lot of bandwidth available and we could get a lot of
    /// errors thrown, for instance if the baud rate is set incorrectly.
    fn enqueue_error(&mut self, error_code: u8) {
        match self.error_producer.grant_exact(1) {
            Ok(mut grant) => {
                grant.buf()[0] = error_code;
                grant.commit(1);
            }
            Err(_) => {
                defmt::error!("Failed to enqueue error (code {:?})", error_code);
            }
        }
    }
    // TODO split this in to another struct so users don't need two ISRs that both reference Self
    /// Called from the appropriate SERCOM ISR
    ///
    /// This *must* be run in a higher-priority (lower number) ISR, than the
    /// level that services the USB.  The issue is that in this method, we
    /// non-atomically try to read from `usb_to_uart_consumer`, and if there  is
    /// no more data to write, we then disable the TX DRE interrupt.  The USB
    /// OUT handler enables that interrupt when it receives data to begin
    /// sending over the UART.  If the UART ISR were interrupted just before
    /// disabling the interrupt, by the USB ISR setting the interrupt, then TX
    /// could stall indefinitely.
    pub fn uart_callback(&mut self) {
        match self.uart_to_usb_producer.grant_exact(1) {
            Ok(mut grant) => {
                match self.uart.read() {
                    Ok(c) => {
                        grant.buf()[0] = c;
                        grant.commit(1);
                    }
                    Err(nb::Error::WouldBlock) => {
                        // Nothing to read here
                        // Drop the grant without committing
                    }
                    Err(nb::Error::Other(error)) => {
                        match error {
                            Error::ParityError => {
                                self.uart.clear_status(Status::PERR);
                                self.enqueue_error(CDC_SERIAL_STATE_PARITY_ERROR);
                            }
                            Error::FrameError => {
                                self.uart.clear_status(Status::FERR);
                                self.enqueue_error(CDC_SERIAL_STATE_FRAMING_ERROR);
                            }
                            Error::Overflow => {
                                self.uart.clear_status(Status::BUFOVF);
                                self.enqueue_error(CDC_SERIAL_STATE_OVER_RUN);
                            }
                            Error::InconsistentSyncField => {
                                // This can happen if using auto-baud; we don't
                                defmt::error!("UART RX InconsistentSyncField");
                                self.uart.clear_status(Status::ISF);
                            }
                            Error::CollisionDetected => {
                                // Requires enabling collision detection
                                defmt::error!("UART RX CollisionDetected");
                                self.uart.clear_status(Status::COLL);
                            }
                        }
                    }
                }
            }

            Err(BBError::InsufficientSize) => {
                defmt::error!("uart_to_usb overflow");
                self.enqueue_error(CDC_SERIAL_STATE_OVER_RUN);

                 // RXC will be re-enabled when there's space in the buffer.
                 // Don't want to read from the SERCOM, in case it's using
                 // hardware flow control and doesn't need to discard data.
                self.uart.disable_interrupts(Flags::RXC);
            }

            Err(_) => {
                // No room in the uart_to_usb buffer
                // TODO ensure that the DTR does what it's supposed to in this case
            }
        }

        if self.uart.read_flags().contains(Flags::DRE) {
            match self.usb_to_uart_consumer.read() {
                Ok(grant) => {
                    // The buffer is guaranteed to have at least one byte
                    match self.uart.write(grant.buf()[0]) {
                        Ok(()) => {
                            grant.release(1);
                        }
                        Err(_) => {
                            // "Impossible" because we confirmed DRE is set
                            // before calling write().
                            defmt::error!("Impossible error in uart_callback()");
                        }
                    };
                }
                Err(BBError::InsufficientSize) => {
                    // There's no more data in the buffer to write
                    self.uart.disable_interrupts(Flags::DRE);
                }
                Err(BBError::AlreadySplit) | Err(BBError::GrantInProgress) => {
                    unreachable!();
                }
            }
        }
    }
}

impl<B, P, const ENDPOINT_SIZE: usize> UsbClass<B> for UsbUart<'_, B, P, ENDPOINT_SIZE>
where
    B: UsbBus,
    P: ValidPads<Capability = Duplex>,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.iad(
            self.comm_if,
            2,
            USB_CLASS_CDC,
            CDC_SUBCLASS_ACM,
            CDC_PROTOCOL_NONE,
        )?;

        writer.interface(
            self.comm_if,
            USB_CLASS_CDC,
            CDC_SUBCLASS_ACM,
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
                CDC_TYPE_ACM, // bDescriptorSubtype
                0x00,         // bmCapabilities
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
                CDC_TYPE_CALL_MANAGEMENT, // bDescriptorSubtype
                0x00,                     // bmCapabilities
                self.data_if.into(),      // bDataInterface
            ],
        )?;

        writer.endpoint(&self.comm_ep)?;

        writer.interface(self.data_if, USB_CLASS_CDC_DATA, 0x00, 0x00)?;

        writer.endpoint(&self.write_ep)?;
        writer.endpoint(&self.read_ep)?;

        Ok(())
    }

    fn reset(&mut self) {
        self.line_coding = LineCoding::default();

        // TODO
        // self.read_buf.clear();
        // self.write_buf.clear();
        self.write_state = WriteState::NotFull;
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr != self.read_ep.address() {
            return;
        }

        match self.usb_to_uart_producer.grant_exact(ENDPOINT_SIZE) {
            Ok(mut grant) => {
                match self.read_ep.read(grant.buf()) {
                    Ok(count) => {
                        grant.commit(count);
                        // TODO note this is only reliable if the UART ISR is higher prio than USB; may be better to put that flag back...
                        self.uart.enable_interrupts(Flags::DRE);
                    }
                    Err(UsbError::WouldBlock) => {
                        // No data to read, just drop the grant
                    }
                    Err(_) => {
                        // TODO handle this better
                        defmt::error!("Error reading TX data");
                    }
                };
            }
            Err(_) => {
                // Since we don't have anywhere to put more data, we can't read
                // data out of the USB endpoint.  The USB hardware will NAK
                // transfers, and the host will decide whether to retry, until
                // we eventually read.  In the meantime, clear interrupt flags
                // in the USB endpoint hardware with an empty read:
                let _ = self.read_ep.read(&mut []);
            }
        }
    }

    fn poll(&mut self) {
        self.flush_usb();
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.write_ep.address() {
            // We've written a bulk transfer out; flush in case there's more buffered data
            self.flush_usb();
        }
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
                    data[0..4].copy_from_slice(&self.line_coding.baud.to_le_bytes());
                    data[4] = self.line_coding.stop_bits as u8;
                    data[5] = match self.line_coding.parity {
                        None => 0,
                        Some(Parity::Even) => 2,
                        Some(Parity::Odd) => 1,
                    };
                    data[6] = 8;

                    Ok(7)
                })
                .unwrap_or_else(|_| defmt::error!("USB-UART Failed to accept REQ_GET_LINE_CODING"));
            }

            _ => {
                defmt::info!("USB-UART rejecting control_in request");
                xfer.reject().unwrap_or_else(|_| {
                    defmt::error!("USB-UART Failed to reject control IN request")
                });
            }
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
                xfer.accept().unwrap_or_else(|_| {
                    defmt::error!("USB-UART Failed to accept REQ_SEND_ENCAPSULATED_COMMAND")
                });
            }

            REQ_SET_LINE_CODING if xfer.data().len() >= 7 => {
                let new_baud = u32::from_le_bytes(xfer.data()[0..4].try_into().unwrap());
                let new_stop_bits = match xfer.data()[4] {
                    0 => Some(StopBits::OneBit),
                    // 1 means 1.5 stop bits, unsupported by hardware
                    2 => Some(StopBits::TwoBits),
                    _ => None,
                };
                let new_parity_type = match xfer.data()[5] {
                    0 => Some(None),
                    1 => Some(Some(Parity::Odd)),
                    2 => Some(Some(Parity::Even)),
                    // 3 means Mark, unsupported by hardware
                    // 4 means Space, unsupported by hardware
                    _ => None,
                };
                let new_data_bits = xfer.data()[6];

                // TODO ensure the baud is within limits
                if if new_stop_bits.is_none() {
                    defmt::warn!(
                        "Rejecting unsupported stop bit request code {:?}",
                        xfer.data()[4]
                    );
                    false
                } else if new_parity_type.is_none() {
                    defmt::warn!(
                        "Rejecting unsupported parity request code {:?}",
                        xfer.data()[5]
                    );
                    false
                } else if new_data_bits != 8 {
                    defmt::warn!(
                        "Rejecting unsupported {:?}b line coding request",
                        new_data_bits
                    );
                    false
                } else {
                    // New config is valid, so apply it
                    // TODO split this out in to a separate method, and use that when we're reset
                    self.uart.reconfigure(|c| {
                        c.baud(Hertz(new_baud), BAUDMODE)
                            .stop_bit(new_stop_bits.unwrap())
                            .parity(new_parity_type.unwrap())
                    });

                    self.line_coding.baud = new_baud;
                    self.line_coding.stop_bits = new_stop_bits.unwrap();
                    self.line_coding.parity = new_parity_type.unwrap();
                    true
                } {
                    xfer.accept().unwrap_or_else(|_| {
                        defmt::error!("USB-UART Failed to accept REQ_SET_LINE_CODING")
                    });
                } else {
                    xfer.reject().unwrap_or_else(|_| {
                        defmt::error!("USB-UART Failed to reject REQ_SET_LINE_CODING")
                    });
                }
            }

            REQ_SET_CONTROL_LINE_STATE => {
                defmt::info!("REQ_SET_CONTROL_LINE_STATE"); // TODO
                if let Some(dtr_callback) = &self.dtr_callback {
                    dtr_callback((req.value & 0x0001) != 0);
                }

                if let Some(rts_callback) = &self.rts_callback {
                    rts_callback((req.value & 0x0002) != 0);
                }

                xfer.accept().unwrap_or_else(|_| {
                    defmt::error!("USB-UART Failed to accept REQ_SET_CONTROL_LINE_STATE")
                });
            }

            _ => {
                defmt::info!("USB-UART rejecting control_out request");
                xfer.reject().unwrap_or_else(|_| {
                    defmt::error!("USB-UART Failed to reject control OUT request")
                });
            }
        };
    }
}
