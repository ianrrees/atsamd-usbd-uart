#![no_std]

extern crate defmt_rtt;
pub use bbqueue::{
    consts::*,
};
use bbqueue::{
    BBBuffer,
    Consumer,
    Error,
    Producer,
};
use usb_device::class_prelude::*;
use usb_device::Result;
use usbd_serial::{
    CdcAcmClass,
    // cdc_acm::*,
    LineCoding,
};

// struct UsbContext{

// }

// struct UartContext{

// }

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
pub struct SerialPort<'a, B, const ENDPOINT_SIZE: usize>
where
B: UsbBus,
{
    inner: CdcAcmClass<'a, B>,
    rx_consumer: Consumer<'a, U256>,
    tx_producer: Producer<'a, U256>,
    write_state: WriteState,
}

/// If this many full size packets have been sent in a row, a short packet will be sent so that the
/// host sees the data in a timely manner.
const SHORT_PACKET_INTERVAL: usize = 10;

/// Keeps track of the type of the last written packet.
enum WriteState {
    /// The last packet written wasn't a full packet
    NotFull,

    /// Full packet current in-flight. A full packet must be followed by a short packet for the host
    /// OS to see the transaction. The data is the number of subsequent full packets sent so far. A
    /// short packet is forced every SHORT_PACKET_INTERVAL packets so that the OS sees data in a
    /// timely manner.
    Full(usize),
}

impl<'a, B, const ENDPOINT_SIZE: usize> SerialPort<'a, B, ENDPOINT_SIZE>
where
    B: UsbBus
{
    /// Creates a new USB serial port
    pub fn new(alloc: &'a UsbBusAllocator<B>, storage: &'a SerialPortStorage) -> Self
    {
        // let (mut _rx_producer, mut rx_consumer) = storage.rx_buffer.try_split().unwrap();
        let (mut tx_producer, mut rx_consumer) = storage.tx_buffer.try_split().unwrap();

        // TODO something useful with the UART side of the queues
        
        Self {
            inner: CdcAcmClass::new(alloc, ENDPOINT_SIZE as u16),
            rx_consumer,
            tx_producer,
            write_state: WriteState::NotFull,
        }
    }

    // / Gets the current line coding.
    // pub fn line_coding(&self) -> &LineCoding { self.inner.line_coding() }

    // / Gets the DTR (data terminal ready) state
    // pub fn dtr(&self) -> bool { self.inner.dtr() }

    // / Gets the RTS (request to send) state
    // pub fn rts(&self) -> bool { self.inner.rts() }

    fn flush(&mut self) {
        match self.rx_consumer.read() {
            // There is more data to write
            Ok(grant) => {
                // Deciding what to write is a bit more complicated, due to the way USB
                // bulk transfers work...
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

                match self.inner.write_packet(write_slice) {
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
                    match self.inner.write_packet(&[]) {
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
}

impl<B, const ENDPOINT_SIZE: usize> UsbClass<B> for SerialPort<'_, B, ENDPOINT_SIZE>
where
    B: UsbBus,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        self.inner.get_configuration_descriptors(writer)
    }

    fn reset(&mut self) {
        self.inner.reset();
        // TODO
        // self.read_buf.clear();
        // self.write_buf.clear();
        self.write_state = WriteState::NotFull;
    }

    fn poll(&mut self) {
        // See if the host has more data to send over the UART
        match self.tx_producer.grant_exact(ENDPOINT_SIZE) {
            Ok(mut grant) => {
                match self.inner.read_packet(grant.buf()) {
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

        self.flush();
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        defmt::warn!("TODO write complete, addr {:?}", u8::from(addr));
        self.flush();
        // if addr == self.inner.write_ep_address() {
        //     self.flush().ok();
        // }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        self.inner.control_in(xfer);
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        self.inner.control_out(xfer);
    }
}
