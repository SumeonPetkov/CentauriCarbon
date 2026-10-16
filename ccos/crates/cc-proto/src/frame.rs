//! Message-block framing: `| len | seq | payload | crc_hi | crc_lo | 0x7e |`.

use crate::dict::MsgBuf;
use crate::{crc16_ccitt, Error, MESSAGE_DEST, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_PAYLOAD_MAX, MESSAGE_SEQ_MASK,
            MESSAGE_SYNC, MESSAGE_TRAILER_SIZE};

/// One encoded frame, by value (≤ 64 bytes, no heap).
#[derive(Clone, Copy)]
pub struct FrameBuf {
    buf: [u8; MESSAGE_MAX],
    len: u8,
}

impl FrameBuf {
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }
}

impl core::ops::Deref for FrameBuf {
    type Target = [u8];
    fn deref(&self) -> &[u8] {
        self.as_slice()
    }
}

impl core::fmt::Debug for FrameBuf {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:02x?}", self.as_slice())
    }
}

/// Wrap `payload` in a frame with sequence number `seq` (low 4 bits used).
pub fn encode_frame(seq: u8, payload: &[u8]) -> Result<FrameBuf, Error> {
    if payload.len() > MESSAGE_PAYLOAD_MAX {
        return Err(Error::TooLong(payload.len()));
    }
    let len = payload.len() + MESSAGE_MIN;
    let mut f = FrameBuf { buf: [0; MESSAGE_MAX], len: len as u8 };
    f.buf[0] = len as u8;
    f.buf[1] = MESSAGE_DEST | (seq & MESSAGE_SEQ_MASK);
    f.buf[2..2 + payload.len()].copy_from_slice(payload);
    let crc = crc16_ccitt(&f.buf[..len - 3]);
    f.buf[len - 3..len - 1].copy_from_slice(&crc.to_be_bytes());
    f.buf[len - 1] = MESSAGE_SYNC;
    Ok(f)
}

#[derive(Debug, Clone, Copy)]
pub struct Frame {
    /// Sequence number (0..=15).
    pub seq: u8,
    pub payload: MsgBuf,
}

impl Frame {
    /// Frames with no payload are ACK/NAK sequence updates.
    pub fn is_ack(&self) -> bool {
        self.payload.is_empty()
    }
}

enum Check {
    NeedMore,
    Valid(usize),
    Discard(usize),
}

/// Streaming frame decoder with Klipper-style resynchronisation on 0x7e
/// (mirrors `msgblock_check`). Fixed 4 KiB buffer, no allocation.
pub struct FrameDecoder {
    buf: [u8; 4096],
    start: usize,
    end: usize,
    need_sync: bool,
    /// Bytes thrown away while resynchronising (diagnostics).
    pub discarded: u64,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self { buf: [0; 4096], start: 0, end: 0, need_sync: false, discarded: 0 }
    }
}

impl FrameDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Free space for [`push`](Self::push) without data loss.
    pub fn space(&self) -> usize {
        self.buf.len() - (self.end - self.start)
    }

    /// Append received bytes. If more than [`space`](Self::space) arrives the
    /// oldest unparsed bytes are dropped (and counted in `discarded`).
    pub fn push(&mut self, mut data: &[u8]) {
        if data.len() > self.buf.len() {
            self.discarded += (data.len() - self.buf.len()) as u64;
            data = &data[data.len() - self.buf.len()..];
        }
        if self.end + data.len() > self.buf.len() {
            self.buf.copy_within(self.start..self.end, 0);
            self.end -= self.start;
            self.start = 0;
        }
        if self.end + data.len() > self.buf.len() {
            let drop = self.end + data.len() - self.buf.len();
            self.discarded += drop as u64;
            self.start = drop;
            self.need_sync = true;
            self.buf.copy_within(self.start..self.end, 0);
            self.end -= self.start;
            self.start = 0;
        }
        self.buf[self.end..self.end + data.len()].copy_from_slice(data);
        self.end += data.len();
    }

    /// Pop the next complete, valid frame if one is buffered.
    pub fn next_frame(&mut self) -> Option<Frame> {
        loop {
            match self.check() {
                Check::NeedMore => return None,
                Check::Discard(n) => {
                    self.discarded += n as u64;
                    self.start += n;
                }
                Check::Valid(n) => {
                    let b = &self.buf[self.start..self.start + n];
                    let frame = Frame {
                        seq: b[1] & MESSAGE_SEQ_MASK,
                        payload: MsgBuf::from_slice(&b[2..n - MESSAGE_TRAILER_SIZE]).expect("payload ≤ 59"),
                    };
                    self.start += n;
                    return Some(frame);
                }
            }
        }
    }

    fn check(&mut self) -> Check {
        let buf = &self.buf[self.start..self.end];
        if buf.is_empty() {
            return Check::NeedMore;
        }
        if !self.need_sync {
            if buf.len() < MESSAGE_MIN {
                return Check::NeedMore;
            }
            let len = buf[0] as usize;
            if (MESSAGE_MIN..=MESSAGE_MAX).contains(&len) && buf[1] & !MESSAGE_SEQ_MASK == MESSAGE_DEST {
                if buf.len() < len {
                    return Check::NeedMore;
                }
                let crc = u16::from_be_bytes([buf[len - 3], buf[len - 2]]);
                if buf[len - 1] == MESSAGE_SYNC && crc == crc16_ccitt(&buf[..len - 3]) {
                    return Check::Valid(len);
                }
            }
        }
        // Error / resync path.
        if buf[0] == MESSAGE_SYNC {
            self.need_sync = false;
            return Check::Discard(1);
        }
        self.need_sync = true;
        match buf.iter().position(|&b| b == MESSAGE_SYNC) {
            Some(p) => {
                self.need_sync = false;
                Check::Discard(p + 1)
            }
            None => Check::Discard(buf.len()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `identify offset=0 count=40`, seq 0, built with Elegoo's C code.
    const IDENTIFY: [u8; 8] = [0x08, 0x10, 0x01, 0x00, 0x28, 0x5e, 0x9f, 0x7e];

    #[test]
    fn encode_matches_c() {
        assert_eq!(encode_frame(0, &[0x01, 0x00, 0x28]).unwrap().as_slice(), IDENTIFY);
    }

    #[test]
    fn decode_with_garbage_and_split() {
        let mut d = FrameDecoder::new();
        d.push(&[0x00, 0x55, 0x7e]); // line noise ending in a sync byte
        d.push(&IDENTIFY[..3]);
        assert!(d.next_frame().is_none());
        d.push(&IDENTIFY[3..]);
        d.push(&encode_frame(5, &[]).unwrap());
        let f = d.next_frame().unwrap();
        assert_eq!((f.seq, f.payload.as_slice()), (0, &[1u8, 0, 0x28][..]));
        let f = d.next_frame().unwrap();
        assert!(f.is_ack());
        assert_eq!(f.seq, 5);
        assert!(d.next_frame().is_none());
        assert_eq!(d.discarded, 3);
    }

    #[test]
    fn corrupted_crc_is_dropped() {
        let mut bad = IDENTIFY;
        bad[5] ^= 1;
        let mut d = FrameDecoder::new();
        d.push(&bad);
        d.push(&IDENTIFY);
        assert_eq!(d.next_frame().unwrap().payload.as_slice(), &[1, 0, 0x28]);
        assert!(d.next_frame().is_none());
    }

    #[test]
    fn long_streams_compact_without_loss() {
        let mut d = FrameDecoder::new();
        let mut n = 0;
        for i in 0..5000u32 {
            d.push(&encode_frame(i as u8, &[1, 0, (i % 90) as u8]).unwrap());
            while let Some(f) = d.next_frame() {
                assert_eq!(f.payload.as_slice()[2], (n % 90) as u8);
                n += 1;
            }
        }
        assert_eq!(n, 5000);
        assert_eq!(d.discarded, 0);
    }

    #[test]
    fn garbage_then_frame() {
        let mut d = FrameDecoder::new();
        d.push(&[0x55; 5000]);
        assert!(d.next_frame().is_none());
        // Like msgblock_check: after losing sync, data is only trusted again
        // after a 0x7e (the host sends sync bytes; the MCU's trailers do it).
        d.push(&[0x7e]);
        d.push(&IDENTIFY);
        assert_eq!(d.next_frame().unwrap().payload.as_slice(), &[1, 0, 0x28]);
        assert_eq!(d.discarded, 5001);
    }

    #[test]
    fn overflow_never_panics() {
        let mut d = FrameDecoder::new();
        for i in 0..20 {
            d.push(&[(i * 37) as u8; 1000]);
            d.push(&IDENTIFY[..4]);
        }
        d.push(&IDENTIFY);
        while d.next_frame().is_some() {}
        assert!(d.space() > 0);
    }

    #[test]
    fn rejects_oversize() {
        assert!(encode_frame(0, &[0; 60]).is_err());
        assert!(encode_frame(0, &[0; 59]).is_ok());
    }
}
