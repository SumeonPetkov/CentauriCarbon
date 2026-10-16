//! Klipper MCU wire protocol, as used between the Centauri Carbon's Linux
//! host and its three MCUs (R528 HiFi4 DSP, toolhead STM32F402, strain-gauge
//! STM32F402).
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
//! blobs inside the shipped MCU firmware are stale. That table is compiled
//! into static arrays by `build.rs`; [`Dictionary::centauri`] is a `const`.
//!
//! The encode/decode paths never allocate.

pub mod clock;
pub mod crc;
pub mod dict;
pub mod frame;
pub mod vlq;

pub use crc::crc16_ccitt;
pub use dict::{Arg, Dictionary, Message, MessageFormat, MsgBuf, OwnedMessage, ParamType};
pub use frame::{encode_frame, Frame, FrameBuf, FrameDecoder};

pub const MESSAGE_MIN: usize = 5;
pub const MESSAGE_MAX: usize = 64;
pub const MESSAGE_HEADER_SIZE: usize = 2;
pub const MESSAGE_TRAILER_SIZE: usize = 3;
pub const MESSAGE_PAYLOAD_MAX: usize = MESSAGE_MAX - MESSAGE_MIN;
pub const MESSAGE_SEQ_MASK: u8 = 0x0f;
pub const MESSAGE_DEST: u8 = 0x10;
pub const MESSAGE_SYNC: u8 = 0x7e;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    UnknownMessage,
    UnknownId(u32),
    MissingParam(&'static str),
    BadValue(&'static str),
    BadSyntax,
    Truncated,
    TooLong(usize),
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::UnknownMessage => write!(f, "unknown message name"),
            Error::UnknownId(i) => write!(f, "unknown message id {i}"),
            Error::MissingParam(p) => write!(f, "missing parameter '{p}'"),
            Error::BadValue(p) => write!(f, "bad value for '{p}'"),
            Error::BadSyntax => write!(f, "malformed command"),
            Error::Truncated => write!(f, "truncated message"),
            Error::TooLong(n) => write!(f, "{n} bytes exceed the {MESSAGE_PAYLOAD_MAX}-byte frame payload"),
        }
    }
}

impl std::error::Error for Error {}
