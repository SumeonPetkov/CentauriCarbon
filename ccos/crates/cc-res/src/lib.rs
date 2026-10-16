//! Centauri Carbon resource formats.
//!
//! * [`widget`]: `/app/resources/widget.bin` — every screen of the stock LVGL
//!   UI (positions, sizes, styles, images, text ids), compiled from
//!   `firmware/resources/e100/widget.csv`.
//! * [`translation`]: `/app/resources/translation.bin` — all UI strings in 11
//!   languages, compiled from `translation.csv`.
//! * [`mcu_image`]: `upgrade_extruder.bin` / `upgrade_sg.bin` — STM32 firmware
//!   update images (32-byte header + MD5, **unsigned**).
//!
//! The CSV readers deliberately mirror Elegoo's C tools (`firmware/tools/`),
//! including `strtok` quirks, so `ccres` produces byte-identical output.

pub mod mcu_image;
pub mod translation;
pub mod widget;

mod csvutil;

#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    Format(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Format(s) => write!(f, "{s}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

pub(crate) fn fmt_err<T>(s: impl Into<String>) -> Result<T, Error> {
    Err(Error::Format(s.into()))
}

/// Little-endian cursor over a byte slice.
pub(crate) struct Rd<'a> {
    pub b: &'a [u8],
    pub pos: usize,
}

impl<'a> Rd<'a> {
    pub fn at(b: &'a [u8], pos: usize) -> Self {
        Self { b, pos }
    }
    fn take(&mut self, n: usize) -> Result<&'a [u8], Error> {
        let s = self.b.get(self.pos..self.pos + n).ok_or_else(|| Error::Format(format!("truncated at {}", self.pos)))?;
        self.pos += n;
        Ok(s)
    }
    pub fn u8(&mut self) -> Result<u8, Error> {
        Ok(self.take(1)?[0])
    }
    pub fn u16(&mut self) -> Result<u16, Error> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().unwrap()))
    }
    pub fn u32(&mut self) -> Result<u32, Error> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    pub fn i32(&mut self) -> Result<i32, Error> {
        Ok(self.u32()? as i32)
    }
    pub fn bytes(&mut self, n: usize) -> Result<&'a [u8], Error> {
        self.take(n)
    }
}
