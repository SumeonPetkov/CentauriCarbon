//! Minimal synchronous Klipper MCU session: sequencing, ACK/NAK, retransmit,
//! request/response. Enough for bring-up, probing and simple queries; the
//! real-time motion path will need the windowed, clock-scheduled queue that
//! Klipper's `serialqueue.c` implements.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use cc_proto::{encode_frame, Dictionary, FrameDecoder, Message};
use cc_transport::Link;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

pub struct Mcu {
    pub name: String,
    link: Box<dyn Link>,
    pub dict: Dictionary,
    dec: FrameDecoder,
    seq: u8,
    pub inbox: VecDeque<Message>,
    pub trace: bool,
}

impl Mcu {
    pub fn new(name: &str, link: Box<dyn Link>) -> Self {
        Self { name: name.into(), link, dict: Dictionary::centauri(), dec: FrameDecoder::new(), seq: 0, inbox: VecDeque::new(), trace: false }
    }

    /// Read whatever arrives within `timeout`; returns the sequence numbers of
    /// frames seen (each frame from the MCU acknowledges up to `seq - 1`).
    fn pump(&mut self, timeout: Duration) -> Result<Vec<u8>> {
        let mut buf = [0u8; 4096];
        let n = self.link.recv(&mut buf, timeout)?;
        self.dec.push(&buf[..n]);
        let mut seqs = Vec::new();
        while let Some(f) = self.dec.next_frame() {
            seqs.push(f.seq);
            if f.payload.is_empty() {
                continue;
            }
            match self.dict.decode_payload(&f.payload) {
                Ok(msgs) => {
                    for m in msgs {
                        if self.trace {
                            eprintln!("[{}] <- {}", self.name, m);
                        }
                        self.inbox.push_back(m);
                    }
                }
                Err(e) => eprintln!("[{}] undecodable frame ({e}): {:02x?}", self.name, f.payload),
            }
        }
        Ok(seqs)
    }

    /// Resynchronise the MCU's frame parser.
    pub fn sync(&mut self) -> Result<()> {
        self.link.send(&[0x7e; 4])?;
        let _ = self.pump(Duration::from_millis(50))?;
        Ok(())
    }

    /// Send one command and wait for the MCU to acknowledge it.
    pub fn send(&mut self, cmd: &str) -> Result<()> {
        let payload = self.dict.encode_str(cmd)?;
        if self.trace {
            eprintln!("[{}] -> {cmd}", self.name);
        }
        for _attempt in 0..8 {
            self.link.send(&encode_frame(self.seq, &payload)?)?;
            let deadline = Instant::now() + Duration::from_millis(250);
            while Instant::now() < deadline {
                for s in self.pump(Duration::from_millis(20))? {
                    let want = (self.seq + 1) & 0x0f;
                    if s == want {
                        self.seq = want;
                        return Ok(());
                    }
                    // NAK: the MCU tells us which sequence it expects.
                    self.seq = s;
                }
            }
        }
        Err(format!("{}: no ack for '{cmd}'", self.name).into())
    }

    /// Wait for a message called `name` (others stay in the inbox).
    pub fn wait_for(&mut self, name: &str, timeout: Duration) -> Result<Message> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(i) = self.inbox.iter().position(|m| m.name == name) {
                return Ok(self.inbox.remove(i).unwrap());
            }
            if Instant::now() > deadline {
                return Err(format!("{}: timeout waiting for '{name}'", self.name).into());
            }
            self.pump(Duration::from_millis(20))?;
        }
    }

    pub fn request(&mut self, cmd: &str, response: &str) -> Result<Message> {
        self.inbox.retain(|m| m.name != response);
        self.send(cmd)?;
        self.wait_for(response, Duration::from_secs(2))
    }

    /// Download the identify blob (stale on stock firmware, but proves the link).
    pub fn identify(&mut self) -> Result<Vec<u8>> {
        let mut data = Vec::new();
        loop {
            let m = self.request(&format!("identify offset={} count=40", data.len()), "identify_response")?;
            let chunk = m.get("data").and_then(|a| a.as_bytes()).unwrap_or(&[]).to_vec();
            if m.int("offset") != Some(data.len() as i64) {
                continue;
            }
            if chunk.is_empty() {
                return Ok(data);
            }
            data.extend(chunk);
        }
    }

    /// Estimate the MCU clock frequency from two `get_clock` round trips.
    pub fn measure_clock(&mut self) -> Result<f64> {
        let t0 = Instant::now();
        let c0 = self.request("get_clock", "clock")?.int("clock").unwrap_or(0) as u32;
        std::thread::sleep(Duration::from_millis(500));
        let c1 = self.request("get_clock", "clock")?.int("clock").unwrap_or(0) as u32;
        Ok(c1.wrapping_sub(c0) as f64 / t0.elapsed().as_secs_f64())
    }
}

/// NTC thermistor with the beta model (printer.cfg: "NTC 100K beta 3950"),
/// 4.7k pull-up as in the stock config.
pub fn ntc_temp(adc_frac: f64, beta: f64) -> Option<f64> {
    let (r0, t0, pullup) = (100_000.0f64, 25.0 + 273.15, 4700.0);
    let adc = adc_frac.clamp(0.000_01, 0.999_99);
    let r = pullup * adc / (1.0 - adc);
    let inv_t = 1.0 / t0 + (r / r0).ln() / beta;
    let t = 1.0 / inv_t - 273.15;
    t.is_finite().then_some(t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cc_transport::Loopback;

    #[test]
    fn ntc_25c_at_midscale_resistance() {
        // R = 100k when adc = 100k / (100k + 4.7k)
        let adc = 100_000.0 / 104_700.0;
        assert!((ntc_temp(adc, 3950.0).unwrap() - 25.0).abs() < 0.01);
        assert!(ntc_temp(0.05, 3950.0).unwrap() > 200.0);
    }

    #[test]
    fn send_retransmits_on_nak() {
        // MCU NAKs with seq 5, then ACKs seq 6.
        let mut lb = Loopback::default();
        lb.to_receive.extend(encode_frame(5, &[]).unwrap());
        lb.to_receive.extend(encode_frame(6, &[]).unwrap());
        let mut m = Mcu::new("t", Box::new(lb));
        m.send("get_uptime").unwrap();
        assert_eq!(m.seq, 6);
    }
}
