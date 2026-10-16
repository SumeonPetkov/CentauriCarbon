//! STM32 MCU update images (`/lib/firmware/upgrade_extruder.bin`,
//! `/lib/firmware/upgrade_sg.bin`), as produced by `mcu/scripts/add_magic.c`.
//!
//! ```text
//! 0x00  magic[4]  = 20 24 01 26 (decimal bytes: 0x14 0x18 0x01 0x1a)
//! 0x04  extra[8]  = major, minor, patch, board (1 = e100, 255 = generic), 1, 0, 0, 0
//! 0x0c  len: u32  = payload length (LE)
//! 0x10  md5[16]   = MD5 of the payload
//! 0x20  0xff padding up to 0x4000
//! 0x4000 payload  = raw klipper.bin linked at 0x0800C000 (bootloader at 0x08000000)
//! ```
//! There is **no signature**: anything with a valid MD5 is flashed by the
//! host's updater, which makes the toolhead and strain-gauge MCU firmware
//! fully user-replaceable.

use md5::{Digest, Md5};

use crate::{fmt_err, Error};

pub const MAGIC: [u8; 4] = [20, 24, 1, 26];
pub const PAYLOAD_OFFSET: usize = 0x4000;
pub const APP_ADDRESS: u32 = 0x0800_c000;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct McuImage {
    pub version: (u8, u8, u8),
    pub board: u8,
    pub extra: [u8; 4],
    pub payload: Vec<u8>,
}

impl McuImage {
    pub fn new(payload: Vec<u8>, version: (u8, u8, u8), board: u8) -> Self {
        Self { version, board, extra: [1, 0, 0, 0], payload }
    }

    pub fn parse(b: &[u8]) -> Result<Self, Error> {
        if b.len() < PAYLOAD_OFFSET || b[..4] != MAGIC {
            return fmt_err("not an MCU update image (bad magic)");
        }
        let len = u32::from_le_bytes(b[12..16].try_into().unwrap()) as usize;
        let payload = b.get(PAYLOAD_OFFSET..PAYLOAD_OFFSET + len).ok_or_else(|| Error::Format("payload truncated".into()))?;
        let md5: [u8; 16] = Md5::digest(payload).into();
        if md5 != b[16..32] {
            return fmt_err("payload MD5 mismatch");
        }
        Ok(Self { version: (b[4], b[5], b[6]), board: b[7], extra: b[8..12].try_into().unwrap(), payload: payload.to_vec() })
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(PAYLOAD_OFFSET + self.payload.len());
        out.extend_from_slice(&MAGIC);
        out.extend_from_slice(&[self.version.0, self.version.1, self.version.2, self.board]);
        out.extend_from_slice(&self.extra);
        out.extend_from_slice(&(self.payload.len() as u32).to_le_bytes());
        out.extend_from_slice(&Md5::digest(&self.payload));
        out.resize(PAYLOAD_OFFSET, 0xff);
        out.extend_from_slice(&self.payload);
        out
    }

    /// Cortex-M vector table sanity check: initial SP in SRAM, reset handler
    /// in the application flash region.
    pub fn vector_table(&self) -> Option<(u32, u32)> {
        let sp = u32::from_le_bytes(self.payload.get(0..4)?.try_into().ok()?);
        let reset = u32::from_le_bytes(self.payload.get(4..8)?.try_into().ok()?);
        Some((sp, reset))
    }

    pub fn looks_valid(&self) -> bool {
        matches!(self.vector_table(), Some((sp, rst))
            if (0x2000_0000..=0x2002_0000).contains(&sp) && (APP_ADDRESS..0x0804_0000).contains(&(rst & !1)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn fw(p: &str) -> Vec<u8> {
        std::fs::read(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../firmware/resources/firmware").join(p)).unwrap()
    }

    #[test]
    fn shipped_images() {
        for (name, ver) in [("upgrade_extruder.bin", (1, 2, 3)), ("upgrade_sg.bin", (1, 6, 9))] {
            let b = fw(name);
            let img = McuImage::parse(&b).unwrap();
            assert_eq!(img.version, ver, "{name}");
            assert_eq!(img.board, 255);
            assert!(img.looks_valid(), "{name}");
            assert_eq!(img.to_bytes(), b, "{name} repack");
        }
    }

    #[test]
    fn detects_tamper() {
        let mut b = fw("upgrade_sg.bin");
        let n = b.len() - 1;
        b[n] ^= 0xff;
        assert!(McuImage::parse(&b).is_err());
    }
}
