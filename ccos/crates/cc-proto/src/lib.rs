//! Klipper MCU wire protocol, as used between the Centauri Carbon's Linux
//! host and its three MCUs (R528 HiFi4 DSP, toolhead STM32F401, strain-gauge
//! STM32F401).
//!
//! Wire format (identical to upstream Klipper):
//!
//! ```text
//! | len:u8 | seq:u8 | payload: 0..59 bytes | crc16:be | 0x7e |
//! ```
//! `seq` is `0x10 | (n & 0x0f)`; a frame with an empty payload is an ACK.
//! A payload holds one or more messages: `msg_id` (VLQ) followed by its
//! parameters, every integer VLQ-encoded and every buffer a length byte plus
//! data.
//!
//! **Centauri Carbon specific:** message ids are not negotiated through the
//! identify dictionary. The C++ host has a fixed table (see
//! `data/command_table.json`, lifted from `msgproto.cpp`) and the identify
//! blobs inside the shipped MCU firmware are stale. [`Dictionary::centauri`]
//! returns that fixed table.

pub mod clock;
pub mod crc;
pub mod dict;
pub mod frame;
pub mod vlq;

pub use crc::crc16_ccitt;
pub use dict::{Arg, Dictionary, Message, MessageFormat, ParamType};
pub use frame::{encode_frame, Frame, FrameDecoder};

pub const MESSAGE_MIN: usize = 5;
pub const MESSAGE_MAX: usize = 64;
pub const MESSAGE_HEADER_SIZE: usize = 2;
pub const MESSAGE_TRAILER_SIZE: usize = 3;
pub const MESSAGE_PAYLOAD_MAX: usize = MESSAGE_MAX - MESSAGE_MIN;
pub const MESSAGE_SEQ_MASK: u8 = 0x0f;
pub const MESSAGE_DEST: u8 = 0x10;
pub const MESSAGE_SYNC: u8 = 0x7e;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    UnknownMessage(String),
    UnknownId(u32),
    MissingParam { msg: String, param: String },
    BadValue { param: String, value: String },
    Truncated,
    TooLong(usize),
    BadFormat(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::UnknownMessage(n) => write!(f, "unknown message '{n}'"),
            Error::UnknownId(i) => write!(f, "unknown message id {i}"),
            Error::MissingParam { msg, param } => write!(f, "{msg}: missing parameter '{param}'"),
            Error::BadValue { param, value } => write!(f, "bad value '{value}' for '{param}'"),
            Error::Truncated => write!(f, "truncated message"),
            Error::TooLong(n) => write!(f, "payload of {n} bytes exceeds {MESSAGE_PAYLOAD_MAX}"),
            Error::BadFormat(s) => write!(f, "bad format string '{s}'"),
        }
    }
}

impl std::error::Error for Error {}
