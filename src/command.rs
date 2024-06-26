use embedded_can::{ExtendedId, Id, StandardId};
use num_enum::IntoPrimitive;

use crate::frame::CanFrame;

/// Represents the various different commands that can be send to the CAN
/// gateway
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive)]
#[repr(u8)]
pub enum CommandKind {
    /// Set the nominal bit rate to a standard CAN [bit rate](NominalBitRate)
    SetNominalBitRate = b'S',
    /// Set the data bit rate (for CAN FD frames only) to a standard CAN FD [bit rate](DataBitRate)
    SetDataBitRate = b'Y',
    /// Sets the mode of the gateway (either normal or silent)
    SetMode = b'M',
    /// Enables or disables auto retransmission of frames
    SetAutoRetransmission = b'A',

    /// Open the CAN channel in normal mode (sending & receiving)
    Open = b'O',
    /// Close the CAN channel
    Close = b'C',

    /// Transmit a standard (11bit) CAN 2.0 data frame
    TransmitStandardDataFrame = b't',
    /// Transmit an extended (29bit) CAN 2.0 data frame
    TransmitExtendedDataFrame = b'T',
    /// Transmit a standard (11bit) CAN 2.0 remote frame
    TransmitStandardRemoteFrame = b'r',
    /// Transmit an extended (29bit) CAN 2.0 remote frame
    TransmitExtendedRemoteFrame = b'R',

    /// Transmit a standard (11bit) CAN FD frame at the nominal bit rate
    TransmitStandardFdFrameNoBrs = b'd',
    /// Transmit an extended (29bit) CAN FD frame at the nominal bit rate
    TransmitExtendedFdFrameNoBrs = b'D',
    /// Transmit a standard (11bit) CAN FD frame at the increased data bit rate
    TransmitStandardFdFrameWithBrs = b'b',
    /// Transmit an extended (29bit) CAN FD frame at the increased data bit rate
    TransmitExtendedFdFrameWithBrs = b'B',

    /// Asks the device for its firmware version
    GetFirmwareVersion = b'V',
    /// Asks the device for the value of its error register
    GetErrorRegister = b'E',
}

/// The bit rate used for CAN 2.0 frames, CAN FD frames without BRS, and the
/// message ID arbitration for CAN FD frames with BRS
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive)]
#[repr(u8)]
pub enum NominalBitRate {
    /// Transmits and receives at 10 Kbit/s
    Rate10Kbit = b'0',
    /// Transmits and receives at 20 Kbit/s
    Rate20Kbit = b'1',
    /// Transmits and receives at 50 Kbit/s
    Rate50Kbit = b'2',
    /// Transmits and receives at 100 Kbit/s
    Rate100Kbit = b'3',
    /// Transmits and receives at 125 Kbit/s
    Rate125Kbit = b'4',
    /// Transmits and receives at 250 Kbit/s
    Rate250Kbit = b'5',
    /// Transmits and receives at 500 Kbit/s
    Rate500Kbit = b'6',
    /// Transmits and receives at 800 Kbit/s
    Rate800Kbit = b'7',
    /// Transmits and receives at 1 Mbit/s
    Rate1Mbit = b'8',
    /// Transmits and receives at 83.3 Kbit/s
    Rate83_3Kbit = b'9',
}

/// The bit rate used for the data and CRC sections of CAN FD frames with BRS
/// enabled
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, Default)]
#[repr(u8)]
pub enum DataBitRate {
    /// Transmits and receives at 2 Mbit/s
    #[default]
    Rate2Mbit = b'2',
    /// Transmits and receives at 5 Mbit/s
    Rate5Mbit = b'5',
}

/// Operating mode of the gateway which changes its fundamental behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, Default)]
#[repr(u8)]
pub enum OperatingMode {
    /// Default mode where the gateway can send and receive frames on the bus
    #[default]
    Normal = b'0',
    /// Sometimes called "Listen Only" mode where the device can only listen
    /// to frames on the bus
    Silent = b'1',
}

/// The auto retransmission policy of the gateway
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, Default)]
#[repr(u8)]
pub enum AutoRetransmissionMode {
    /// Frames will not be retransmitted if an error occurs while transmitting
    Disabled = b'0',
    /// Frames will be retransmitted indefinitely until they succeed
    #[default]
    Enabled = b'1',
}

/// A command sent to the CAN gateway along with it's attached data
#[derive(Debug)]
pub enum Command {
    SetNominalBitRate(NominalBitRate),
    SetDataBitRate(DataBitRate),
    SetMode(OperatingMode),
    SetAutoRetransmission(AutoRetransmissionMode),
    Open,
    Close,
    TransmitFrame(CanFrame),
}

impl Command {
    pub fn as_bytes(&self) -> Vec<u8> {
        let mut result = Vec::new();

        match self {
            Command::SetNominalBitRate(rate) => {
                result.push(CommandKind::SetNominalBitRate.into());
                result.push((*rate).into());
            }
            Command::SetDataBitRate(rate) => {
                result.push(CommandKind::SetDataBitRate.into());
                result.push((*rate).into());
            }
            Command::SetMode(mode) => {
                result.push(CommandKind::SetMode.into());
                result.push((*mode).into());
            }
            Command::SetAutoRetransmission(mode) => {
                result.push(CommandKind::SetAutoRetransmission.into());
                result.push((*mode).into());
            }
            Command::Open => result.push(CommandKind::Open.into()),
            Command::Close => result.push(CommandKind::Close.into()),
            Command::TransmitFrame(frame) => match frame {
                CanFrame::Can2(frame) => {
                    match frame.id() {
                        Id::Standard(id) => {
                            if frame.is_remote() {
                                result.push(CommandKind::TransmitStandardRemoteFrame.into());
                            } else {
                                result.push(CommandKind::TransmitStandardDataFrame.into());
                            }

                            result.extend(standard_id_to_hex(id));
                        }
                        Id::Extended(id) => {
                            if frame.is_remote() {
                                result.push(CommandKind::TransmitExtendedRemoteFrame.into());
                            } else {
                                result.push(CommandKind::TransmitExtendedDataFrame.into());
                            }

                            result.extend(extended_id_to_hex(id));
                        }
                    }

                    result.push(to_hex_digit(frame.dlc() as u32));

                    if let Some(data) = frame.data() {
                        result.extend(bytes_to_hex(data));
                    }
                }
                CanFrame::CanFd(frame) => {
                    match frame.id() {
                        Id::Standard(id) => {
                            if frame.is_bit_rate_switched() {
                                result.push(CommandKind::TransmitStandardFdFrameWithBrs.into());
                            } else {
                                result.push(CommandKind::TransmitStandardFdFrameNoBrs.into());
                            }

                            result.extend(standard_id_to_hex(id));
                        }
                        Id::Extended(id) => {
                            if frame.is_bit_rate_switched() {
                                result.push(CommandKind::TransmitExtendedFdFrameWithBrs.into());
                            } else {
                                result.push(CommandKind::TransmitExtendedFdFrameNoBrs.into());
                            }

                            result.extend(extended_id_to_hex(id));
                        }
                    }

                    result.push(to_hex_digit(frame.dlc().get_num_bytes() as u32));
                    result.extend(bytes_to_hex(frame.data()));
                }
            },
        }

        result
    }
}

fn to_hex_digit(value: u32) -> u8 {
    const HEX_LUT: &[u8] = "0123456789ABCDEF".as_bytes();

    HEX_LUT[(value & 0xF) as usize]
}

fn standard_id_to_hex(id: StandardId) -> [u8; 3] {
    let raw = id.as_raw() as u32;

    [
        to_hex_digit(raw >> 8),
        to_hex_digit(raw >> 4),
        to_hex_digit(raw),
    ]
}

fn extended_id_to_hex(id: ExtendedId) -> [u8; 8] {
    let raw = id.as_raw();

    [
        to_hex_digit(raw >> 28),
        to_hex_digit(raw >> 24),
        to_hex_digit(raw >> 20),
        to_hex_digit(raw >> 16),
        to_hex_digit(raw >> 12),
        to_hex_digit(raw >> 8),
        to_hex_digit(raw >> 4),
        to_hex_digit(raw),
    ]
}

fn bytes_to_hex(data: &[u8]) -> Vec<u8> {
    let mut buf = Vec::<u8>::with_capacity(2 * data.len());

    for byte in data {
        buf.push(to_hex_digit((byte >> 4) as u32));
        buf.push(to_hex_digit(*byte as u32));
    }

    buf
}
