use embedded_can::Id;
use num_enum::{IntoPrimitive, TryFromPrimitive};

/// A joint enum which can hold either a CAN 2.0 frame or a CAN FD frame. See
/// [`Can2Frame`] and [`CanFdFrame`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CanFrame {
    Can2(Can2Frame),
    CanFd(CanFdFrame),
}

impl From<Can2Frame> for CanFrame {
    fn from(frame: Can2Frame) -> Self {
        Self::Can2(frame)
    }
}

impl From<CanFdFrame> for CanFrame {
    fn from(frame: CanFdFrame) -> Self {
        Self::CanFd(frame)
    }
}

/// Represents a CAN 2.0 frame which supports RTR (Remote Transmission Request).
///
/// The DLC can be up to 8 bytes, and the data if absent means that it is an
/// RTR frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Can2Frame {
    id: Id,
    dlc: usize,
    data: Option<[u8; 8]>,
}

impl Can2Frame {
    /// Creates a new CAN 2.0 data frame. `data` must have a length in the
    /// range 0..=8 or else `None` will be returned instead.
    pub fn new_data(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }

        let mut copy = [0u8; 8];
        copy[..data.len()].copy_from_slice(data);

        Some(Self {
            id: id.into(),
            dlc: data.len(),
            data: Some(copy),
        })
    }

    /// Creates a new CAN 2.0 data frame. `dlc` must be in the range 0..=8 or
    /// else `None` will be returned instead.
    pub fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        if dlc > 8 {
            return None;
        }

        Some(Self {
            id: id.into(),
            dlc,
            data: None,
        })
    }

    /// Gets the message ID of the frame
    pub fn id(&self) -> Id {
        self.id
    }

    /// Gets the DLC (Data Length Code) of the frame
    pub fn dlc(&self) -> usize {
        self.dlc
    }

    /// Gets the data associated with the frame. Will return `None` if it is an
    /// RTR frame.
    pub fn data(&self) -> Option<&[u8]> {
        self.data.as_ref().map(|d| &d[..self.dlc])
    }

    pub fn is_remote(&self) -> bool {
        self.data.is_none()
    }
}

/// Represents all the possible DLC values for CAN FD frames.
///
/// The integer value of the enum maps to the DLC used in the CAN protocol and
/// not the actual number of bytes associated with each variant. To obtain
/// that, see [`FdDataLengthCode::get_num_bytes`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum FdDataLengthCode {
    Bytes0 = 0,
    Bytes1 = 1,
    Bytes2 = 2,
    Bytes3 = 3,
    Bytes4 = 4,
    Bytes5 = 5,
    Bytes6 = 6,
    Bytes7 = 7,
    Bytes8 = 8,
    Bytes12 = 9,
    Bytes16 = 10,
    Bytes20 = 11,
    Bytes24 = 12,
    Bytes32 = 13,
    Bytes48 = 14,
    Bytes64 = 15,
}

impl FdDataLengthCode {
    /// Returns the next closest DLC for the given length value. Values over 64
    /// will return `None`.
    pub fn for_length(length: usize) -> Option<Self> {
        Some(match length {
            x @ 0..=8 => (x as u8).try_into().unwrap(),
            9..=12 => Self::Bytes12,
            13..=16 => Self::Bytes16,
            17..=20 => Self::Bytes20,
            21..=24 => Self::Bytes24,
            25..=32 => Self::Bytes32,
            33..=48 => Self::Bytes48,
            49..=64 => Self::Bytes64,
            _ => return None,
        })
    }

    /// Returns the number of bytes that this variant can hold, which is
    /// different from the enum's integer value.
    pub fn get_num_bytes(&self) -> usize {
        match self {
            Self::Bytes0 => 0,
            Self::Bytes1 => 1,
            Self::Bytes2 => 2,
            Self::Bytes3 => 3,
            Self::Bytes4 => 4,
            Self::Bytes5 => 5,
            Self::Bytes6 => 6,
            Self::Bytes7 => 7,
            Self::Bytes8 => 8,
            Self::Bytes12 => 12,
            Self::Bytes16 => 16,
            Self::Bytes20 => 20,
            Self::Bytes24 => 24,
            Self::Bytes32 => 32,
            Self::Bytes48 => 48,
            Self::Bytes64 => 64,
        }
    }
}

/// Represents a CAN FD frame which can store up to 64 data bytes and
/// optionally supports transmitting at a higher data bit rate (this defaults
/// to true). See [`DataBitRate`](crate::DataBitRate).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFdFrame {
    id: Id,
    data: heapless::Vec<u8, 64>,
    bit_rate_switched: bool,
}

impl CanFdFrame {
    /// Creates a new CAN FD frame. Will return `None` if the data is not one
    /// of the allowed DLC values for CAN FD.
    pub fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 64 {
            return None;
        }

        if FdDataLengthCode::try_from(data.len() as u8).is_err() {
            return None;
        }

        Some(Self {
            id: id.into(),
            data: heapless::Vec::<u8, 64>::from_slice(data).unwrap(),
            bit_rate_switched: true,
        })
    }

    /// Creates a new CAN FD frame. Will return `None` if the data is longer
    /// than 64 bytes. Any lengths under 64 will be padded with 0s until they
    /// reach one of the allowed CAN FD data length codes.
    pub fn new_padded(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        let dlc = FdDataLengthCode::for_length(data.len())?;

        let mut data = heapless::Vec::<u8, 64>::from_slice(data).unwrap();
        data.extend((data.len()..dlc.get_num_bytes()).map(|_| 0));

        Some(Self {
            id: id.into(),
            data,
            bit_rate_switched: true,
        })
    }

    /// Gets the message ID of the frame
    pub fn id(&self) -> Id {
        self.id
    }

    /// Gets the DLC (Data Length Code) of the frame
    pub fn dlc(&self) -> FdDataLengthCode {
        (self.data.len() as u8).try_into().unwrap()
    }

    /// Gets the data associated with the frame (length will match DLC)
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    /// Returns whether or not this frame should be/was transmitted with the
    /// higher data bit rate
    pub fn is_bit_rate_switched(&self) -> bool {
        self.bit_rate_switched
    }

    /// Sets whether the frame should be transmitted with the higher data bit
    /// rate
    pub fn set_bit_rate_switched(&mut self, bit_rate_switched: bool) {
        self.bit_rate_switched = bit_rate_switched
    }

    /// Consumes self and returns a new self with the the supplied value for
    /// `bit_rate_switched`
    pub fn with_bit_rate_switched(mut self, bit_rate_switched: bool) -> Self {
        self.bit_rate_switched = bit_rate_switched;
        self
    }
}
