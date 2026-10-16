//! Minimal synchronous Klipper MCU session: sequencing, ACK/NAK, retransmit,
//! request/response. Enough for bring-up, sensors and simple outputs; the
//! real-time motion path will need the windowed, clock-scheduled queue that
//! Klipper's `serialqueue.c` implements.
//!
//! No allocation per message: frames and messages are fixed-size values.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use cc_proto::{encode_frame, Dictionary, FrameDecoder, MsgBuf, OwnedMessage};
use cc_transport::Link;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;

/// Messages kept while waiting for a specific response.
const INBOX_MAX: usize = 256;

pub struct Mcu {
    pub name: &'static str,
    link: Box<dyn Link>,
    pub dict: Dictionary,
    dec: FrameDecoder,
    seq: u8,
    pub inbox: VecDeque<OwnedMessage>,
    pub trace: bool,
    rx: Box<[u8; 4096]>,
}

impl Mcu {
    pub fn new(name: &'static str, link: Box<dyn Link>) -> Self {
        Self {
            name,
            link,
            dict: Dictionary::centauri(),
            dec: FrameDecoder::new(),
            seq: 0,
            inbox: VecDeque::with_capacity(INBOX_MAX),
            trace: false,
            rx: Box::new([0; 4096]),
        }
    }

    /// Read whatever arrives within `timeout`; queue decoded messages and
    /// return the sequence numbers of the frames seen (each frame from the
    /// MCU acknowledges everything before its `seq`).
    pub fn pump(&mut self, timeout: Duration, seqs: &mut [u8; 32]) -> Result<usize> {
        let n = self.link.recv(&mut self.rx[..], timeout)?;
        self.dec.push(&self.rx[..n]);
        let mut ns = 0;
        while let Some(f) = self.dec.next_frame() {
            if ns < seqs.len() {
                seqs[ns] = f.seq;
                ns += 1;
            }
            for m in self.dict.decode(f.payload.as_slice()) {
                match m {
                    Ok(m) => {
                        if self.trace {
                            eprintln!("[{}] <- {m}", self.name);
                        }
                        if self.inbox.len() == INBOX_MAX {
                            self.inbox.pop_front();
                        }
                        self.inbox.push_back(m.to_owned());
                    }
                    Err(e) => {
                        eprintln!("[{}] undecodable frame ({e}): {:?}", self.name, f.payload);
                        break;
                    }
                }
            }
        }
        Ok(ns)
    }

    /// Resynchronise the MCU's frame parser.
    pub fn sync(&mut self) -> Result<()> {
        self.link.send(&[0x7e; 4])?;
        let mut s = [0u8; 32];
        self.pump(Duration::from_millis(50), &mut s)?;
        Ok(())
    }

    /// Send one command and wait for the MCU to acknowledge it.
    pub fn send(&mut self, cmd: &str) -> Result<()> {
        let mut payload = MsgBuf::new();
        self.dict.encode_str(cmd, &mut payload)?;
        if self.trace {
            eprintln!("[{}] -> {cmd}", self.name);
        }
        let mut seqs = [0u8; 32];
        for _attempt in 0..8 {
            self.link.send(&encode_frame(self.seq, payload.as_slice())?)?;
            let deadline = Instant::now() + Duration::from_millis(250);
            'wait: while Instant::now() < deadline {
                let n = self.pump(Duration::from_millis(20), &mut seqs)?;
                for &s in &seqs[..n] {
                    let want = (self.seq + 1) & 0x0f;
                    if s == want {
                        self.seq = want;
                        return Ok(());
                    }
                    if s != self.seq {
                        // NAK: the MCU expects another sequence number. Adopt
                        // it and retransmit now (a duplicate of an accepted
                        // frame is NAKed, never executed twice).
                        self.seq = s;
                        break 'wait;
                    }
                }
            }
        }
        Err(format!("{}: no ack for '{cmd}'", self.name).into())
    }

    /// Take the oldest queued message called `name`, waiting up to `timeout`.
    pub fn wait_for(&mut self, name: &str, timeout: Duration) -> Result<OwnedMessage> {
        let deadline = Instant::now() + timeout;
        let mut seqs = [0u8; 32];
        loop {
            if let Some(i) = self.inbox.iter().position(|m| m.name() == name) {
                return Ok(self.inbox.remove(i).unwrap());
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(format!("{}: timeout waiting for '{name}'", self.name).into());
            }
            self.pump((deadline - now).min(Duration::from_millis(20)), &mut seqs)?;
        }
    }

    pub fn request(&mut self, cmd: &str, response: &str) -> Result<OwnedMessage> {
        self.inbox.retain(|m| m.name() != response);
        self.send(cmd)?;
        self.wait_for(response, Duration::from_secs(2))
    }

    /// Download the identify blob (stale on stock firmware, but proves the link).
    pub fn identify(&mut self) -> Result<Vec<u8>> {
        let mut data = Vec::new();
        for _ in 0..1024 {
            let m = self.request(&format!("identify offset={} count=40", data.len()), "identify_response")?;
            let msg = m.message();
            if msg.int("offset") != Some(data.len() as i64) {
                continue;
            }
            let chunk = msg.bytes("data").unwrap_or(&[]);
            if chunk.is_empty() {
                return Ok(data);
            }
            data.extend_from_slice(chunk);
        }
        Err(format!("{}: identify did not terminate", self.name).into())
    }

    /// Estimate the MCU clock frequency from two `get_clock` round trips.
    pub fn measure_clock(&mut self) -> Result<f64> {
        let t0 = Instant::now();
        let c0 = self.clock()?;
        std::thread::sleep(Duration::from_millis(500));
        let c1 = self.clock()?;
        Ok(c1.wrapping_sub(c0) as f64 / t0.elapsed().as_secs_f64())
    }

    pub fn clock(&mut self) -> Result<u32> {
        Ok(self.request("get_clock", "clock")?.int("clock").unwrap_or(0) as u32)
    }
}

/// NTC thermistor, beta model, 100 kΩ @ 25 °C, 4.7 kΩ pull-up (stock
/// `thermistor.cpp` default). Hotend: β 4300, bed/chamber: β 3950.
pub fn ntc_temp(adc_frac: f64, beta: f64) -> Option<f64> {
    let (r0, t0, pullup) = (100_000.0f64, 25.0 + 273.15, 4700.0);
    if !(0.0005..0.9995).contains(&adc_frac) {
        return None; // open or shorted sensor
    }
    let r = pullup * adc_frac / (1.0 - adc_frac);
    let inv_t = 1.0 / t0 + (r / r0).ln() / beta;
    let t = 1.0 / inv_t - 273.15;
    t.is_finite().then_some(t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ntc_25c_at_midscale_resistance() {
        let adc = 100_000.0 / 104_700.0;
        assert!((ntc_temp(adc, 3950.0).unwrap() - 25.0).abs() < 0.01);
        assert!(ntc_temp(0.05, 3950.0).unwrap() > 200.0);
        assert_eq!(ntc_temp(1.0, 3950.0), None);
        assert_eq!(ntc_temp(0.0, 3950.0), None);
    }

    /// Fake MCU that answers each frame from a script.
    struct Scripted {
        replies: VecDeque<Vec<u8>>,
        inbox: VecDeque<u8>,
    }

    impl Link for Scripted {
        fn send(&mut self, _data: &[u8]) -> std::io::Result<()> {
            if let Some(r) = self.replies.pop_front() {
                self.inbox.extend(r);
            }
            Ok(())
        }
        fn recv(&mut self, buf: &mut [u8], _t: Duration) -> std::io::Result<usize> {
            let n = buf.len().min(self.inbox.len());
            for (i, b) in self.inbox.drain(..n).enumerate() {
                buf[i] = b;
            }
            Ok(n)
        }
    }

    fn frame(seq: u8, cmd: Option<&str>) -> Vec<u8> {
        let mut p = MsgBuf::new();
        if let Some(c) = cmd {
            Dictionary::centauri().encode_str(c, &mut p).unwrap();
        }
        encode_frame(seq, p.as_slice()).unwrap().to_vec()
    }

    #[test]
    fn send_retransmits_on_nak() {
        // seq 0 is NAKed with "expect 5"; the retransmit (seq 5) is acked with 6.
        let link = Scripted { replies: VecDeque::from([frame(5, None), frame(6, None)]), inbox: VecDeque::new() };
        let mut m = Mcu::new("t", Box::new(link));
        m.send("get_uptime").unwrap();
        assert_eq!(m.seq, 6);
    }

    #[test]
    fn response_is_delivered() {
        let link = Scripted { replies: VecDeque::from([frame(1, Some("clock clock=123456"))]), inbox: VecDeque::new() };
        let mut m = Mcu::new("t", Box::new(link));
        assert_eq!(m.clock().unwrap(), 123456);
    }

    #[test]
    fn identify_concatenates_chunks() {
        let link = Scripted {
            replies: VecDeque::from([
                frame(1, Some("identify_response offset=0 data=abc")),
                frame(2, Some("identify_response offset=3 data=de")),
                frame(3, Some("identify_response offset=5 data=")),
            ]),
            inbox: VecDeque::new(),
        };
        let mut m = Mcu::new("t", Box::new(link));
        assert_eq!(m.identify().unwrap(), b"abcde");
    }
}
