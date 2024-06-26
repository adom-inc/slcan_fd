//! A portable serial line CAN (SLCAN) interface for Rust with support for the
//! newer CAN FD protocol.
//!
//! Since sending and receiving CAN FD frames over an SLCAN interface is not
//! standardized, this crate looks to be compatible with the commands supported
//! by the [CANable 2.0 Firmware](https://github.com/normaldotcom/canable2-fw).
//!
//! By default this crate is async and uses the
//! [`tokio-serial`](https://github.com/berkowski/tokio-serial) crate, but it
//! can also be used in a sync context (See [Features Flags](#feature-flags)).
//!     
//! ## Usage
//!
//! ```
//! use slcan_fd::{tokio::CanSocket, NominalBitRate, OperatingMode};
//! use tokio_serial::SerialPortBuilderExt;
//!
//! let mut port = tokio_serial::new("/dev/ttyUSB0", 115_200).open_native_async()?;
//!
//! #[cfg(unix)]
//! port.set_exclusive(false)
//!     .expect("Unable to set serial port exclusive to false");
//!
//! let mut can = CanSocket::new(port);
//!
//! can.close().await?;
//! can.set_operating_mode(OperatingMode::Silent).await?;
//! can.open(NominalBitRate::Rate500Kbit).await?;
//!
//! loop {
//!     match can.read().await {
//!         Ok(frame) => println!("{:?}", frame),
//!         Err(e) => eprintln!("{:?}", e),
//!     }
//! }
//! ```
//!
//! ## Feature Flags
//!
//! The `tokio` feature is enabled by default.
//!
//! - `tokio` - Implements the async API with the [`tokio-serial`](https://github.com/berkowski/tokio-serial) crate.
//! - `sync` - Implements the synchronous API with the [`serialport`](https://github.com/serialport/serialport-rs) crate.
//!
//! ## Credits
//!
//! This crate is very loosely based on a previous crate for slcan without FD support which can be found [here](https://github.com/jmaygarden/slcan).
//!     

pub use embedded_can::{ExtendedId, Id, StandardId};

mod command;
mod frame;
mod parser;

pub use command::{AutoRetransmissionMode, DataBitRate, NominalBitRate, OperatingMode};
pub use frame::{Can2Frame, CanFdFrame, CanFrame};
pub use parser::{MessageKind, MessageParseError};

/// Maximum rx buffer len: (command + extended id + dlc + data + CR + 16 bytes extra)
const SLCAN_MTU: usize = (1 + 8 + 1 + 128) + 1 + 16;

#[derive(Debug, thiserror::Error)]
pub enum ReadError {
    #[error("IO Error: {0}")]
    Io(#[from] std::io::Error),
    #[error("SLCAN message parsing error: {0}")]
    Slcan(#[from] MessageParseError),
}

#[cfg(feature = "sync")]
pub mod sync {
    //! The synchronous implementation of CanSocket for use with the
    //! [serialport] crate.

    use std::io;
    #[cfg(target_family = "unix")]
    use std::os::unix::prelude::AsRawFd;

    use serialport::SerialPort;

    use crate::{
        command::{AutoRetransmissionMode, Command, DataBitRate, OperatingMode},
        frame::CanFrame,
        parser::parse_frame_from_bytes,
        NominalBitRate, ReadError, SLCAN_MTU,
    };

    /// Represents an synchronous interface into a CAN FD network through a
    /// serial (USB) gateway device.
    ///
    /// Messages can be sent over the bus through the gateway, and messages
    /// broadcasted on the bus by other nodes can be received through the
    /// gateway.
    pub struct CanSocket<P: SerialPort> {
        port: P,
        rx_buff: [u8; SLCAN_MTU],
        rx_count: usize,
        error: bool,
    }

    #[cfg(target_family = "unix")]
    impl<P> AsRawFd for CanSocket<P>
    where
        P: SerialPort + AsRawFd,
    {
        fn as_raw_fd(&self) -> std::os::unix::prelude::RawFd {
            self.port.as_raw_fd()
        }
    }

    impl<P: SerialPort> CanSocket<P> {
        /// Constructs a new CanSocket from a generic serial port
        pub fn new(port: P) -> Self {
            CanSocket {
                port,
                rx_buff: [0; SLCAN_MTU],
                rx_count: 0,
                error: false,
            }
        }

        /// Configures the device with the supplied bit timing and requests
        /// the device to begin enable streaming of CAN frames
        pub fn open(&mut self, nominal_bit_rate: NominalBitRate) -> io::Result<()> {
            self.send_command(Command::SetNominalBitRate(nominal_bit_rate))?;
            self.send_command(Command::Open)?;
            Ok(())
        }

        /// Sends a close command to the gateway which instructs it to stop
        /// sending and receiving CAN frames
        pub fn close(&mut self) -> io::Result<()> {
            self.send_command(Command::Close)?;
            Ok(())
        }

        /// Sets the data bit rate (CAN FD frames only). See [DataBitRate].
        pub fn set_data_bit_rate(&mut self, rate: DataBitRate) -> io::Result<()> {
            self.send_command(Command::SetDataBitRate(rate))?;
            Ok(())
        }

        /// Sets the operating mode of the gateway, either `Normal` or `Silent`
        /// (a.k.a. "Listen Only" mode). See [OperatingMode].
        pub fn set_operating_mode(&mut self, mode: OperatingMode) -> io::Result<()> {
            self.send_command(Command::SetMode(mode))?;
            Ok(())
        }

        /// Sets the auto retransmission mode of the gateway, either `Enabled`
        /// or `Disabled`. See [AutoRetransmissionMode].
        pub fn set_auto_retransmission_mode(
            &mut self,
            mode: AutoRetransmissionMode,
        ) -> io::Result<()> {
            self.send_command(Command::SetAutoRetransmission(mode))?;
            Ok(())
        }

        /// Sends a CAN frame to the gateway to be broadcasted on the bus.
        ///
        /// If the frame fails to be sent, it may be retransmitted according to
        /// the current [AutoRetransmissionMode].
        pub fn send(&mut self, frame: impl Into<CanFrame>) -> io::Result<()> {
            self.send_command(Command::TransmitFrame(frame.into()))?;
            Ok(())
        }

        /// Reads a line from the serial stream and attempts to parse it as a
        /// valid CAN frame.
        ///
        /// # Errors
        ///
        /// An error will be returned if the operation would block or timed
        /// out. In this case it is safe to call `read` again until a message
        /// is received.
        ///
        /// An error will also be returned for any other kinds of I/O errors.
        ///
        /// Finally, an error will be returned if the received line cannot be
        /// parsed as a valid CAN frame for any number of reasons. See
        /// [MessageParseError](crate::MessageParseError).
        pub fn read(&mut self) -> Result<CanFrame, ReadError> {
            Ok(parse_frame_from_bytes(&self.read_line()?)?)
        }

        /// Reads from the serial stream until a line of length 1..=SLCAN_MTU
        /// is received with a terminating CR.
        ///
        /// Will return an Err if the operation would block and is safe to
        /// call again in that case without losing any state.
        fn read_line(&mut self) -> io::Result<Vec<u8>> {
            let mut buf = [0u8; 1];

            while self.port.read(&mut buf)? == 1 {
                let b = buf[0];

                if b == b'\r' {
                    let valid = !self.error && self.rx_count > 0;
                    let buffer = &self.rx_buff[..self.rx_count];

                    self.error = false;
                    self.rx_count = 0;

                    // We detected an error, move on and read the next line instead
                    if !valid {
                        continue;
                    }

                    return Ok(buffer.to_vec());
                }

                // If we already detected an error, keep reading until we find a CR
                if self.error {
                    continue;
                }

                // If we encounter a line that is too long, set the error flag and
                // keep reading until we find a CR
                if self.rx_count >= SLCAN_MTU {
                    self.error = true;
                    continue;
                }

                // If things are going normally, just store the byte
                self.rx_buff[self.rx_count] = b;
                self.rx_count += 1;
            }

            Err(io::ErrorKind::WouldBlock.into())
        }

        /// Serializes a command and sends it over the serial stream with a CR
        /// line ending appended. Crucially, the entire command is sent in one
        /// write operation which is important because the CANable does not
        /// always correctly buffer input and will fail to parse our commands
        /// if they are split into multiple USB packets.
        fn send_command(&mut self, command: Command) -> io::Result<()> {
            let mut buffer = command.as_bytes();
            buffer.push(b'\r');

            self.port.write_all(&buffer)?;
            self.port.flush()?;
            Ok(())
        }
    }
}

#[cfg(feature = "tokio")]
pub mod tokio {
    //! The async implementation of CanSocket for use with the
    //! [tokio_serial] crate.

    use std::io;
    #[cfg(target_family = "unix")]
    use std::os::unix::prelude::AsRawFd;

    use tokio::io::AsyncReadExt;
    use tokio::io::AsyncWriteExt;
    use tokio_serial::SerialStream;

    use crate::parser::parse_frame_from_bytes;
    use crate::{
        command::{AutoRetransmissionMode, Command, DataBitRate, OperatingMode},
        frame::CanFrame,
        NominalBitRate, ReadError, SLCAN_MTU,
    };

    /// Represents an asynchronous interface into a CAN FD network through a
    /// serial (USB) gateway device.
    ///
    /// Messages can be sent over the bus through the gateway, and messages
    /// broadcasted on the bus by other nodes can be received through the
    /// gateway.
    pub struct CanSocket {
        port: SerialStream,
        rx_buff: [u8; SLCAN_MTU],
        rx_count: usize,
        error: bool,
    }

    #[cfg(target_family = "unix")]
    impl AsRawFd for CanSocket {
        fn as_raw_fd(&self) -> std::os::unix::prelude::RawFd {
            self.port.as_raw_fd()
        }
    }

    impl CanSocket {
        /// Constructs a new CanSocket from an async SerialStream
        pub fn new(port: SerialStream) -> Self {
            CanSocket {
                port,
                rx_buff: [0; SLCAN_MTU],
                rx_count: 0,
                error: false,
            }
        }

        /// Configures the device with the supplied bit timing and requests
        /// the device to begin enable streaming of CAN frames
        pub async fn open(&mut self, nominal_bitrate: NominalBitRate) -> io::Result<()> {
            self.send_command(Command::SetNominalBitRate(nominal_bitrate))
                .await?;
            self.send_command(Command::Open).await?;

            Ok(())
        }

        /// Sends a close command to the gateway which instructs it to stop
        /// sending and receiving CAN frames
        pub async fn close(&mut self) -> io::Result<()> {
            self.send_command(Command::Close).await?;
            Ok(())
        }

        /// Sets the data bit rate (CAN FD frames only). See [DataBitRate].
        pub async fn set_data_bit_rate(&mut self, rate: DataBitRate) -> io::Result<()> {
            self.send_command(Command::SetDataBitRate(rate)).await?;
            Ok(())
        }

        /// Sets the operating mode of the gateway, either `Normal` or `Silent`
        /// (a.k.a. "Listen Only" mode). See [OperatingMode].
        pub async fn set_operating_mode(&mut self, mode: OperatingMode) -> io::Result<()> {
            self.send_command(Command::SetMode(mode)).await?;
            Ok(())
        }

        /// Sets the auto retransmission mode of the gateway, either `Enabled`
        /// or `Disabled`. See [AutoRetransmissionMode].
        pub async fn set_auto_retransmission_mode(
            &mut self,
            mode: AutoRetransmissionMode,
        ) -> io::Result<()> {
            self.send_command(Command::SetAutoRetransmission(mode))
                .await?;
            Ok(())
        }

        /// Sends a CAN frame to the gateway to be broadcasted on the bus.
        ///
        /// If the frame fails to be sent, it may be retransmitted according to
        /// the current [AutoRetransmissionMode].
        pub async fn send(&mut self, frame: impl Into<CanFrame>) -> io::Result<()> {
            self.send_command(Command::TransmitFrame(frame.into()))
                .await?;
            Ok(())
        }

        /// Reads a line from the serial stream and attempts to parse it as a
        /// valid CAN frame.
        ///
        /// # Errors
        ///
        /// An error will be returned for any kinds of I/O errors besides
        /// WouldBlock or TimedOut.
        ///
        /// An error will also be returned if the received line cannot be
        /// parsed as a valid CAN frame for any number of reasons. See
        /// [MessageParseError](crate::MessageParseError).
        ///
        /// # Cancel Safety
        ///
        /// This method is cancel safe. If you use it as the event in a
        /// [`tokio::select`] statement and some other branch completes first,
        /// then it is guaranteed that either no data was read, or any read
        /// data was stored appropriately. Future calls to `read` will use this
        /// buffered data to continue construction of the next frame.
        pub async fn read(&mut self) -> Result<CanFrame, ReadError> {
            Ok(parse_frame_from_bytes(&self.read_line().await?)?)
        }

        /// Reads from the serial stream until a line of length 1..=SLCAN_MTU
        /// is received with a terminating CR.
        ///
        /// Will wait until data is available to produce a line and will not
        /// return until one is received.
        async fn read_line(&mut self) -> Result<Vec<u8>, ReadError> {
            loop {
                let mut buf = [0u8; 1];

                if self.port.read(&mut buf).await? != 1 {
                    continue;
                }

                let b = buf[0];

                if b == b'\r' {
                    let valid = !self.error && self.rx_count > 0;
                    let buffer = &self.rx_buff[..self.rx_count];

                    self.error = false;
                    self.rx_count = 0;

                    // We detected an error, move on and read the next line instead
                    if !valid {
                        continue;
                    }

                    return Ok(buffer.to_vec());
                }

                // If we already detected an error, keep reading until we find a CR
                if self.error {
                    continue;
                }

                // If we encounter a line that is too long, set the error flag and
                // keep reading until we find a CR
                if self.rx_count >= SLCAN_MTU {
                    self.error = true;
                    continue;
                }

                // If things are going normally, just store the byte
                self.rx_buff[self.rx_count] = b;
                self.rx_count += 1;
            }
        }

        /// Serializes a command and sends it over the serial stream with a CR
        /// line ending appended. Crucially, the entire command is sent in one
        /// write operation which is important because the CANable does not
        /// always correctly buffer input and will fail to parse our commands
        /// if they are split into multiple USB packets.
        async fn send_command(&mut self, command: Command) -> io::Result<()> {
            let mut buffer = command.as_bytes();
            buffer.push(b'\r');

            self.port.write_all(&buffer).await?;
            self.port.flush().await?;
            Ok(())
        }
    }
}
