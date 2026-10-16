//! Message-block framing: `| len | seq | payload | crc_hi | crc_lo | 0x7e |`.

use crate::{crc16_ccitt, Error, MESSAGE_DEST, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_PAYLOAD_MAX,
            MESSAGE_SEQ_MASK, MESSAGE_SYNC, MESSAGE_TRAILER_SIZE};

/// Wrap `payload` in a frame with sequence number `seq` (only the low 4 bits
/// are used).
pub fn encode_frame(seq: u8, payload: &[u8]) -> Result<Vec<u8>, Error> {
    if payload.len() > MESSAGE_PAYLOAD_MAX {
        return Err(Error::TooLong(payload.len()));
    }
    let len = payload.len() + MESSAGE_MIN;
    let mut out = Vec::with_capacity(len);
    out.push(len as u8);
    out.push(MESSAGE_DEST | (seq & MESSAGE_SEQ_MASK));
    out.extend_from_slice(payload);
    let crc = crc16_ccitt(&out);
    out.extend_from_slice(&crc.to_be_bytes());
    out.push(MESSAGE_SYNC);
    Ok(out)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// Sequence number (0..=15).
    pub seq: u8,
    pub payload: Vec<u8>,
}

impl Frame {
    /// Frames with no payload are ACK/NAK sequence updates.
    pub fn is_ack(&self) -> bool {
        self.payload.is_empty()
    }
}

/// Outcome of validating the front of a buffer (mirrors `msgblock_check`).
enum Check {
    NeedMore,
    Valid(usize),
    Discard(usize),
}

/// Streaming frame decoder with Klipper-style resynchronisation on 0x7e.
#[derive(Default)]
pub struct FrameDecoder {
    buf: Vec<u8>,
    need_sync: bool,
    /// Bytes thrown away while resynchronising (for diagnostics).
    pub discarded: u64,
}

impl FrameDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, data: &[u8]) {
        self.buf.extend_from_slice(data);
    }

    /// Pop the next complete, valid frame if one is buffered.
    pub fn next_frame(&mut self) -> Option<Frame> {
        loop {
            match self.check() {
                Check::NeedMore => return None,
                Check::Discard(n) => {
                    self.discarded += n as u64;
                    self.buf.drain(..n);
                }
                Check::Valid(n) => {
                    let frame = Frame {
                        seq: self.buf[1] & MESSAGE_SEQ_MASK,
                        payload: self.buf[2..n - MESSAGE_TRAILER_SIZE].to_vec(),
                    };
                    self.buf.drain(..n);
                    return Some(frame);
                }
            }
        }
    }

    fn check(&mut self) -> Check {
        let buf = &self.buf;
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
        assert_eq!(encode_frame(0, &[0x01, 0x00, 0x28]).unwrap(), IDENTIFY);
    }

    #[test]
    fn decode_with_garbage_and_split() {
        let mut d = FrameDecoder::new();
        d.push(&[0x00, 0x55, 0x7e]); // line noise ending in a sync byte
        d.push(&IDENTIFY[..3]);
        assert_eq!(d.next_frame(), None);
        d.push(&IDENTIFY[3..]);
        let ack = encode_frame(5, &[]).unwrap();
        d.push(&ack);
        assert_eq!(d.next_frame(), Some(Frame { seq: 0, payload: vec![1, 0, 0x28] }));
        let f = d.next_frame().unwrap();
        assert!(f.is_ack());
        assert_eq!(f.seq, 5);
        assert_eq!(d.next_frame(), None);
        assert_eq!(d.discarded, 3);
    }

    #[test]
    fn corrupted_crc_is_dropped() {
        let mut bad = IDENTIFY;
        bad[5] ^= 1;
        let mut d = FrameDecoder::new();
        d.push(&bad);
        d.push(&IDENTIFY);
        assert_eq!(d.next_frame().unwrap().payload, vec![1, 0, 0x28]);
        assert_eq!(d.next_frame(), None);
    }

    #[test]
    fn rejects_oversize() {
        assert!(encode_frame(0, &[0; 60]).is_err());
        assert!(encode_frame(0, &[0; 59]).is_ok());
    }
}
