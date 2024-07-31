use embedded_can::{ExtendedId, StandardId};
use num_enum::TryFromPrimitive;

use crate::{
    frame::{CanFdFrame, CanFrame},
    Can2Frame,
};

const MAX_DATA_LENGTH: usize = 64;

/// Various errors which can arise while parsing an SLCAN message
#[derive(Debug, thiserror::Error)]
pub enum MessageParseError {
    /* Generic message parsing */
    #[error("Received a message with an unrecognized specifier ({0:?})")]
    UnrecognizedMessage(u8),
    #[error("Received a message ({0:?}) but less bytes than is required to parse it ({1:?})")]
    NotEnoughBytes(MessageKind, usize),
    #[error("Received a message ({0:?}) but more bytes than were expected ({1:?})")]
    TooManyBytes(MessageKind, usize),

    /* Frame Parsing */
    #[error("Tried to decode a hex digit but it was out of range ({0:?})")]
    IllegalHexDigit(u8),
    #[error("Tried to decode a decimal digit but it was out of range ({0:?})")]
    IllegalDecimalDigit(u8),
    #[error("Received a CAN Standard ID ({0:?}) that was out of the valid range (0..=0x7FF)")]
    StandardIdOutOfRange(u16),
    #[error("Received a CAN Extended ID ({0:?}) that was out of the valid range (0..=0x1FFFFFFF)")]
    ExtendedIdOutOfRange(u32),
    #[error("Received encoded data with a length ({0:?}) that was not a multiple of 2")]
    InvalidDataLength(u8),
    #[error("Received a message with DLC ({0:?}) but ({1:?}) bytes of data")]
    MismatchedDataLength(u8, usize),
}

/// Represents a message received from the CAN gateway
#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive)]
#[num_enum(error_type(name = MessageParseError, constructor = MessageParseError::UnrecognizedMessage))]
#[repr(u8)]
pub enum MessageKind {
    /// Received a standard (11bit) CAN 2.0 data frame
    ReceivedStandardDataFrame = b't',
    /// Received an extended (29bit) CAN 2.0 data frame
    ReceivedExtendedDataFrame = b'T',
    /// Received a standard (11bit) CAN 2.0 remote frame
    ReceivedStandardRemoteFrame = b'r',
    /// Received an extended (29bit) CAN 2.0 remote frame
    ReceivedExtendedRemoteFrame = b'R',

    /// Received a standard (11bit) CAN FD frame at the nominal bit rate
    ReceivedStandardFdFrameNoBrs = b'd',
    /// Received an extended (29bit) CAN FD frame at the nominal bit rate
    ReceivedExtendedFdFrameNoBrs = b'D',
    /// Received a standard (11bit) CAN FD frame at the increased data bit rate
    ReceivedStandardFdFrameWithBrs = b'b',
    /// Received an extended (29bit) CAN FD frame at the increased data bit rate
    ReceivedExtendedFdFrameWithBrs = b'B',
}

impl MessageKind {
    fn get_min_data_length(&self) -> usize {
        match self {
            MessageKind::ReceivedStandardDataFrame => 3 + 1, // (standard id + dlc)
            MessageKind::ReceivedExtendedDataFrame => 8 + 1, // (extended id + dlc)
            MessageKind::ReceivedStandardRemoteFrame => 3 + 1, // (standard id + dlc)
            MessageKind::ReceivedExtendedRemoteFrame => 8 + 1, // (extended id + dlc)
            MessageKind::ReceivedStandardFdFrameNoBrs => 3 + 1, // (standard id + dlc)
            MessageKind::ReceivedExtendedFdFrameNoBrs => 8 + 1, // (extended id + dlc)
            MessageKind::ReceivedStandardFdFrameWithBrs => 3 + 1, // (standard id + dlc)
            MessageKind::ReceivedExtendedFdFrameWithBrs => 8 + 1, // (extended id + dlc)
        }
    }

    fn get_max_data_length(&self) -> usize {
        match self {
            MessageKind::ReceivedStandardDataFrame => 3 + 1 + 16, // (standard id + dlc + data)
            MessageKind::ReceivedExtendedDataFrame => 8 + 1 + 16, // (extended id + dlc + data)
            MessageKind::ReceivedStandardRemoteFrame => 3 + 1,    // (standard id + dlc)
            MessageKind::ReceivedExtendedRemoteFrame => 8 + 1,    // (extended id + dlc)
            MessageKind::ReceivedStandardFdFrameNoBrs => 3 + 1 + 128, // (standard id + dlc + data)
            MessageKind::ReceivedExtendedFdFrameNoBrs => 8 + 1 + 128, // (extended id + dlc + data)
            MessageKind::ReceivedStandardFdFrameWithBrs => 3 + 1 + 128, // (standard id + dlc + data)
            MessageKind::ReceivedExtendedFdFrameWithBrs => 8 + 1 + 128, // (extended id + dlc + data)
        }
    }
}

pub fn parse_frame_from_bytes(buffer: &[u8]) -> Result<CanFrame, MessageParseError> {
    assert!(
        buffer.len() > 1,
        "Tried to parse message from empty buffer!"
    );

    let kind: MessageKind = buffer[0].try_into()?;
    let message_data = &buffer[1..];

    /* Validate data length */

    if message_data.len() < kind.get_min_data_length() {
        return Err(MessageParseError::NotEnoughBytes(kind, buffer.len()));
    }

    if message_data.len() > kind.get_max_data_length() {
        return Err(MessageParseError::TooManyBytes(kind, buffer.len()));
    }

    /* Parse data bytes */

    Ok(match kind {
        MessageKind::ReceivedStandardDataFrame => {
            let id_bytes = &message_data[..3];
            let dlc_byte = message_data[3];
            let data_bytes = &message_data[4..];

            let id = standard_id_from_hex(id_bytes.try_into().unwrap())?;
            let dlc = dec_digit_to_u8(dlc_byte)?;
            let data = unpack_data_bytes(data_bytes, dlc)?;

            Can2Frame::new_data(id, &data[..dlc as usize])
                .unwrap()
                .into()
        }
        MessageKind::ReceivedExtendedDataFrame => {
            let id_bytes = &message_data[..8];
            let dlc_byte = message_data[8];
            let data_bytes = &message_data[9..];

            let id = extended_id_from_hex(id_bytes.try_into().unwrap())?;
            let dlc = dec_digit_to_u8(dlc_byte)?;
            let data = unpack_data_bytes(data_bytes, dlc)?;

            Can2Frame::new_data(id, &data[..dlc as usize])
                .unwrap()
                .into()
        }
        MessageKind::ReceivedStandardRemoteFrame => {
            let id_bytes = &message_data[..3];
            let dlc_byte = message_data[3];

            let id = standard_id_from_hex(id_bytes.try_into().unwrap())?;
            let dlc = dec_digit_to_u8(dlc_byte)?;

            Can2Frame::new_remote(id, dlc as usize).unwrap().into()
        }
        MessageKind::ReceivedExtendedRemoteFrame => {
            let id_bytes = &message_data[..8];
            let dlc_byte = message_data[8];

            let id = extended_id_from_hex(id_bytes.try_into().unwrap())?;
            let dlc = dec_digit_to_u8(dlc_byte)?;

            Can2Frame::new_remote(id, dlc as usize).unwrap().into()
        }
        MessageKind::ReceivedStandardFdFrameNoBrs => {
            let id_bytes = &message_data[..3];
            let dlc_byte = message_data[3];
            let data_bytes = &message_data[4..];

            let id = standard_id_from_hex(id_bytes.try_into().unwrap())?;
            let dlc = dec_digit_to_u8(dlc_byte)?;
            let data = unpack_data_bytes(data_bytes, dlc)?;

            CanFdFrame::new(id, &data[..dlc as usize])
                .unwrap()
                .with_bit_rate_switched(false)
                .into()
        }
        MessageKind::ReceivedExtendedFdFrameNoBrs => {
            let id_bytes = &message_data[..8];
            let dlc_byte = message_data[8];
            let data_bytes = &message_data[9..];

            let id = extended_id_from_hex(id_bytes.try_into().unwrap())?;
            let dlc = dec_digit_to_u8(dlc_byte)?;
            let data = unpack_data_bytes(data_bytes, dlc)?;

            CanFdFrame::new(id, &data[..dlc as usize])
                .unwrap()
                .with_bit_rate_switched(false)
                .into()
        }
        MessageKind::ReceivedStandardFdFrameWithBrs => {
            let id_bytes = &message_data[..3];
            let dlc_byte = message_data[3];
            let data_bytes = &message_data[4..];

            let id = standard_id_from_hex(id_bytes.try_into().unwrap())?;
            let dlc = dec_digit_to_u8(dlc_byte)?;
            let data = unpack_data_bytes(data_bytes, dlc)?;

            CanFdFrame::new(id, &data[..dlc as usize]).unwrap().into()
        }
        MessageKind::ReceivedExtendedFdFrameWithBrs => {
            let id_bytes = &message_data[..8];
            let dlc_byte = message_data[8];
            let data_bytes = &message_data[9..];

            let id = extended_id_from_hex(id_bytes.try_into().unwrap())?;
            let dlc = dec_digit_to_u8(dlc_byte)?;
            let data = unpack_data_bytes(data_bytes, dlc)?;

            CanFdFrame::new(id, &data[..dlc as usize]).unwrap().into()
        }
    })
}

fn hex_digit_to_u8(byte: u8) -> Result<u8, MessageParseError> {
    Ok(match byte {
        b'0'..=b'9' => byte - b'0',
        b'a'..=b'f' => byte - b'a' + 10,
        b'A'..=b'A' => byte - b'A' + 10,
        _ => return Err(MessageParseError::IllegalHexDigit(byte)),
    })
}

fn dec_digit_to_u8(byte: u8) -> Result<u8, MessageParseError> {
    Ok(match byte {
        b'0'..=b'9' => byte - b'0',
        _ => return Err(MessageParseError::IllegalDecimalDigit(byte)),
    })
}

fn u8_from_hex(hex_nibbles: &[u8; 2]) -> Result<u8, MessageParseError> {
    let msn = hex_digit_to_u8(hex_nibbles[0])?;
    let lsn = hex_digit_to_u8(hex_nibbles[1])?;

    Ok((msn << 4) | lsn)
}

fn standard_id_from_hex(hex_nibbles: &[u8; 3]) -> Result<StandardId, MessageParseError> {
    let mut value = 0u16;

    for nibble in hex_nibbles.iter() {
        value <<= 4;
        value |= hex_digit_to_u8(*nibble)? as u16;
    }

    StandardId::new(value).ok_or(MessageParseError::StandardIdOutOfRange(value))
}

fn extended_id_from_hex(hex_nibbles: &[u8; 8]) -> Result<ExtendedId, MessageParseError> {
    let mut value = 0u32;

    for nibble in hex_nibbles.iter() {
        value <<= 4;

        value |= hex_digit_to_u8(*nibble)? as u32;
    }

    ExtendedId::new(value).ok_or(MessageParseError::ExtendedIdOutOfRange(value))
}

fn unpack_data_bytes(
    hex_bytes: &[u8],
    dlc: u8,
) -> Result<[u8; MAX_DATA_LENGTH], MessageParseError> {
    // Make sure data is multiple of 2 (otherwise we can't parse the hex digits)
    if hex_bytes.len() % 2 != 0 {
        return Err(MessageParseError::InvalidDataLength(hex_bytes.len() as u8));
    }

    // Make sure the data length matches the DLC
    if hex_bytes.len() != dlc as usize {
        return Err(MessageParseError::MismatchedDataLength(
            dlc,
            hex_bytes.len() / 2,
        ));
    }

    let mut buf = [0u8; MAX_DATA_LENGTH];

    // Iterate over pairs of hex digits
    hex_bytes.chunks(2).enumerate().try_for_each(|(i, chunk)| {
        buf[i] = u8_from_hex(chunk.try_into().unwrap())?;
        Ok(())
    })?;

    Ok(buf)
}
