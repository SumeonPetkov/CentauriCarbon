//! One MCU session: framing, sequence numbers, ACK/NAK and retransmission,
//! a send window, clock-scheduled messages and clock synchronisation —
//! the parts of Klipper's `serialqueue.c` and `clocksync.py` a host needs.
//!
//! Non-blocking: [`Mcu::service`] reads what arrived and sends what may be
//! sent; the printer core calls it from its `poll()` loop. Blocking helpers
//! ([`Mcu::send_wait`], [`Mcu::request`]) wrap it for bring-up and CLI tools.
//!
//! No allocation per message: messages and frames are fixed-size values in
//! pre-sized queues.

use std::collections::VecDeque;
use std::time::Duration;

use cc_proto::dict::Arg;
use cc_proto::{encode_frame, Dictionary, FrameBuf, FrameDecoder, MsgBuf, OwnedMessage};
use cc_transport::Link;

use crate::clock::now;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;

/// Messages kept for the core to pick up.
const INBOX_MAX: usize = 512;
/// Unacknowledged frames at most (printer.cfg `max_pending_blocks: 12`).
const MAX_PENDING_BLOCKS: usize = 12;
const MIN_RTO: f64 = 0.025;
const MAX_RTO: f64 = 5.0;

struct Inflight {
    seq: u8,
    frame: FrameBuf,
    sent: f64,
    retransmitted: bool,
}

/// A message held until the MCU clock reaches `min_clock` (a free move-queue
/// slot). `tag` lets the core withdraw unsent step messages after homing.
#[derive(Clone, Copy)]
struct Scheduled {
    min_clock: u64,
    /// When the MCU acts on it (diagnostics: how close to the deadline it was sent).
    req_clock: u64,
    tag: u8,
    msg: MsgBuf,
}

pub const TAG_NONE: u8 = 0xff;

/// Linear clock model fitted to `get_clock` round trips (Klipper's
/// `clocksync.py`: exponentially decaying regression, decay 1/30).
#[derive(Debug, Clone)]
pub struct ClockSync {
    pub nominal: f64,
    pub freq: f64,
    time_avg: f64,
    clock_avg: f64,
    time_variance: f64,
    clock_covariance: f64,
    min_half_rtt: f64,
    /// Best estimate: clock = clock_at + (t - time_at) * freq.
    time_at: f64,
    clock_at: f64,
    pub last_clock: u64,
    pub samples: u32,
}

impl ClockSync {
    fn new(nominal: f64) -> Self {
        Self {
            nominal,
            freq: nominal,
            time_avg: 0.0,
            clock_avg: 0.0,
            time_variance: 0.0,
            clock_covariance: 0.0,
            min_half_rtt: f64::MAX,
            time_at: 0.0,
            clock_at: 0.0,
            last_clock: 0,
            samples: 0,
        }
    }

    fn sample(&mut self, sent: f64, received: f64, clock32: u32) {
        let half_rtt = 0.5 * (received - sent);
        let aligned = sent + half_rtt;
        let clock = cc_proto::clock::clock_from_clock32(self.last_clock, clock32);
        self.last_clock = clock;
        if self.samples == 0 {
            self.min_half_rtt = half_rtt;
            self.time_avg = aligned;
            self.clock_avg = clock as f64;
            self.time_variance = 1.0;
            self.clock_covariance = self.nominal;
        } else {
            if half_rtt > 3.0 * self.min_half_rtt.max(0.0002) + 0.002 {
                // Slow round trip: the sample says little about the clock.
                self.min_half_rtt += 0.0001;
                return;
            }
            self.min_half_rtt = self.min_half_rtt.min(half_rtt);
            const DECAY: f64 = 1.0 / 30.0;
            let diff_clock = clock as f64 - self.clock_avg;
            self.clock_avg += DECAY * diff_clock;
            let diff_time = aligned - self.time_avg;
            self.time_avg += DECAY * diff_time;
            self.time_variance = (1.0 - DECAY) * (self.time_variance + diff_time * diff_time * DECAY);
            self.clock_covariance = (1.0 - DECAY) * (self.clock_covariance + diff_time * diff_clock * DECAY);
            let f = self.clock_covariance / self.time_variance;
            // Reject nonsense while the regression settles.
            if (f / self.nominal - 1.0).abs() < 0.01 {
                self.freq = f;
            }
        }
        self.samples += 1;
        self.time_at = self.time_avg;
        self.clock_at = self.clock_avg;
    }

    /// Estimated MCU clock (64-bit) at host time `t`.
    pub fn clock(&self, t: f64) -> u64 {
        (self.clock_at + (t - self.time_at) * self.freq).max(0.0) as u64
    }

    /// Host time at which the MCU clock reads `clock`.
    pub fn time(&self, clock: u64) -> f64 {
        self.time_at + (clock as f64 - self.clock_at) / self.freq
    }

    /// Extend a 32-bit clock from the MCU to 64 bits.
    pub fn extend(&self, clock32: u32) -> u64 {
        cc_proto::clock::clock_from_clock32(self.last_clock, clock32)
    }

    pub fn ticks(&self, seconds: f64) -> u32 {
        (seconds * self.freq).round().max(0.0) as u32
    }
}

pub struct Mcu {
    pub name: &'static str,
    link: Box<dyn Link>,
    pub dict: Dictionary,
    dec: FrameDecoder,
    rx: Box<[u8; 4096]>,
    next_seq: u8,
    /// An acknowledgement has confirmed the sequence numbers.
    synced: bool,
    inflight: VecDeque<Inflight>,
    inflight_bytes: usize,
    window: usize,
    urgent: VecDeque<MsgBuf>,
    sched: VecDeque<Scheduled>,
    /// Messages of another command queue (TMC UART): held until their
    /// `min_clock`, independently of the step messages in `sched`.
    held: VecDeque<(u64, MsgBuf)>,
    srtt: f64,
    rttvar: f64,
    rto: f64,
    last_retransmit: f64,
    pub inbox: VecDeque<OwnedMessage>,
    pub clock: ClockSync,
    /// Host time a `get_clock` was sent (only one outstanding).
    clock_query: Option<f64>,
    next_clock_query: f64,
    /// Set by a `shutdown`/`is_shutdown` message.
    pub shutdown: Option<&'static str>,
    pub trace: bool,
    pub retransmits: u64,
    /// Scheduled messages sent less than 25 ms before they were due.
    pub late_sends: u64,
    pub debug: bool,
    pub bytes_sent: u64,
    /// Host time of the last frame from the MCU.
    pub last_rx: f64,
}

impl Mcu {
    /// `window`: unacknowledged bytes the MCU can buffer (Klipper uses 192
    /// for serial MCUs).
    pub fn new(name: &'static str, link: Box<dyn Link>, nominal_freq: f64, window: usize) -> Self {
        Self {
            name,
            link,
            dict: Dictionary::centauri(),
            dec: FrameDecoder::new(),
            rx: Box::new([0; 4096]),
            next_seq: 0,
            synced: false,
            inflight: VecDeque::with_capacity(MAX_PENDING_BLOCKS),
            inflight_bytes: 0,
            window,
            urgent: VecDeque::with_capacity(256),
            sched: VecDeque::with_capacity(2048),
            held: VecDeque::new(),
            srtt: 0.0,
            rttvar: 0.0,
            rto: MIN_RTO * 4.0,
            last_retransmit: 0.0,
            inbox: VecDeque::with_capacity(INBOX_MAX),
            clock: ClockSync::new(nominal_freq),
            clock_query: None,
            next_clock_query: 0.0,
            shutdown: None,
            trace: false,
            retransmits: 0,
            late_sends: 0,
            debug: std::env::var_os("CCOS_DEBUG").is_some(),
            bytes_sent: 0,
            last_rx: now(),
        }
    }

    pub fn fd(&self) -> Option<i32> {
        self.link.fd()
    }

    // ---- queueing ----

    /// Encode a message from its parameters (no text parsing).
    pub fn encode(&self, name: &str, args: &[(&str, Arg<'_>)]) -> MsgBuf {
        let mut b = MsgBuf::new();
        self.dict.encode(name, args, &mut b).unwrap_or_else(|e| panic!("{name}: {e}"));
        b
    }

    /// Queue a command in Klipper text form, sent as soon as the window allows.
    pub fn queue(&mut self, cmd: &str) -> Result<()> {
        let mut b = MsgBuf::new();
        self.dict.encode_str(cmd, &mut b).map_err(|e| format!("{}: '{cmd}': {e}", self.name))?;
        if self.trace {
            eprintln!("[{}] -> {cmd}", self.name);
        }
        self.urgent.push_back(b);
        Ok(())
    }

    pub fn queue_msg(&mut self, m: MsgBuf) {
        self.urgent.push_back(m);
    }

    /// Queue a message that must not be sent before the MCU clock reaches
    /// `min_clock`. Scheduled messages keep their order.
    pub fn queue_scheduled(&mut self, min_clock: u64, req_clock: u64, tag: u8, m: MsgBuf) {
        self.sched.push_back(Scheduled { min_clock, req_clock, tag, msg: m });
    }

    /// Queue a message on a separate command queue: sent once the MCU
    /// clock reaches `min_clock`, without holding up the step messages.
    pub fn queue_held(&mut self, min_clock: u64, m: MsgBuf) {
        self.held.push_back((min_clock, m));
    }

    /// Drop unsent scheduled messages with this tag (steps of a stopped stepper).
    pub fn withdraw(&mut self, tag: u8) -> usize {
        let n = self.sched.len();
        self.sched.retain(|s| s.tag != tag);
        n - self.sched.len()
    }

    /// Held messages not yet sent.
    pub fn held_len(&self) -> usize {
        self.held.len()
    }

    /// Scheduled messages not yet sent.
    pub fn backlog(&self) -> usize {
        self.sched.len() + self.held.len()
    }

    /// Nothing waiting to be sent or acknowledged.
    pub fn idle(&self) -> bool {
        self.urgent.is_empty() && self.sched.is_empty() && self.held.is_empty() && self.inflight.is_empty()
    }

    // ---- I/O ----

    /// Read and process whatever has arrived (waiting at most `timeout`),
    /// then transmit what may be sent.
    pub fn service(&mut self, timeout: Duration) -> Result<()> {
        let n = self.link.recv(&mut self.rx[..], timeout)?;
        let t = now();
        if n > 0 {
            self.dec.push(&self.rx[..n]);
            while let Some(f) = self.dec.next_frame() {
                self.on_frame(f, t);
            }
        }
        self.transmit(t)
    }

    fn on_frame(&mut self, f: cc_proto::Frame, t: f64) {
        self.last_rx = t;
        // Every frame carries the sequence number the MCU expects next.
        let s = f.seq;
        let acked = if s == self.next_seq {
            self.inflight.len()
        } else if let Some(n) = self.inflight.iter().position(|i| i.seq == s) {
            n
        } else if !self.synced {
            // First contact: the MCU's sequence counter is wherever a previous
            // host left it. Adopt it and resend what we sent (like Klipper's
            // serialqueue does while receive_seq is still 1).
            self.resequence(s, t);
            0
        } else {
            0
        };
        if acked > 0 {
            self.synced = true;
        }
        for _ in 0..acked {
            let i = self.inflight.pop_front().unwrap();
            self.inflight_bytes -= i.frame.len();
            if !i.retransmitted {
                let rtt = t - i.sent;
                if self.srtt == 0.0 {
                    self.srtt = rtt;
                    self.rttvar = rtt / 2.0;
                } else {
                    self.rttvar = 0.75 * self.rttvar + 0.25 * (self.srtt - rtt).abs();
                    self.srtt = 0.875 * self.srtt + 0.125 * rtt;
                }
            }
            self.rto = (self.srtt + 4.0 * self.rttvar).clamp(MIN_RTO, MAX_RTO);
        }
        if acked == 0 && f.is_ack() && !self.inflight.is_empty() && self.inflight[0].seq == s && t - self.last_retransmit > self.srtt.max(0.002) {
            // NAK: the MCU wants a frame we already sent.
            self.retransmit(t);
        }
        for m in self.dict.decode(f.payload.as_slice()) {
            let Ok(m) = m else {
                eprintln!("[{}] undecodable frame: {:?}", self.name, f.payload);
                break;
            };
            if self.trace {
                eprintln!("[{}] <- {m}", self.name);
            }
            match m.name() {
                "clock" => {
                    if let Some(sent) = self.clock_query.take() {
                        self.clock.sample(sent, t, m.int("clock").unwrap_or(0) as u32);
                    }
                }
                "shutdown" | "is_shutdown" => {
                    let id = m.int("static_string_id").unwrap_or(0) as u16;
                    self.shutdown = Some(self.dict.static_string(id).unwrap_or("unknown reason"));
                }
                _ => {}
            }
            if self.inbox.len() == INBOX_MAX {
                self.inbox.pop_front();
            }
            self.inbox.push_back(m.to_owned());
        }
    }

    fn resequence(&mut self, s: u8, t: f64) {
        let mut seq = s;
        for i in self.inflight.iter_mut() {
            let payload = &i.frame[2..i.frame.len() - cc_proto::MESSAGE_TRAILER_SIZE];
            i.frame = encode_frame(seq, payload).expect("same size");
            i.seq = seq;
            seq = (seq + 1) & 0x0f;
        }
        self.next_seq = seq;
        if !self.inflight.is_empty() {
            self.retransmit(t);
        }
    }

    fn retransmit(&mut self, t: f64) {
        if self.debug {
            eprintln!("[{}] retransmit {} frames (rto {:.1} ms, srtt {:.2} ms)", self.name, self.inflight.len(), self.rto * 1e3, self.srtt * 1e3);
        }
        self.last_retransmit = t;
        let mut buf = [0u8; 64 * MAX_PENDING_BLOCKS + 1];
        buf[0] = cc_proto::MESSAGE_SYNC;
        let mut n = 1;
        for i in self.inflight.iter_mut() {
            buf[n..n + i.frame.len()].copy_from_slice(&i.frame);
            n += i.frame.len();
            i.retransmitted = true;
            i.sent = t;
        }
        self.retransmits += 1;
        let _ = self.link.send(&buf[..n]);
    }

    fn transmit(&mut self, t: f64) -> Result<()> {
        if let Some(first) = self.inflight.front() {
            if t - first.sent > self.rto {
                self.rto = (self.rto * 2.0).min(MAX_RTO);
                self.retransmit(t);
            }
        }
        // Periodic clock sync once the link is idle enough to time it well.
        if self.clock_query.is_none() && t >= self.next_clock_query && self.inflight.is_empty() && self.clock.samples > 0 {
            self.urgent.push_front(self.encode("get_clock", &[]));
            self.next_clock_query = t + 0.98;
            self.clock_query = Some(t);
        } else if let Some(q) = self.clock_query {
            if t - q > 2.0 {
                self.clock_query = None; // lost
            }
        }
        let est = self.clock.clock(t);
        while self.inflight.len() < MAX_PENDING_BLOCKS {
            // Payload room left in the MCU's receive window (one frame always fits).
            let room = if self.inflight.is_empty() {
                cc_proto::MESSAGE_PAYLOAD_MAX
            } else {
                self.window.saturating_sub(self.inflight_bytes + cc_proto::MESSAGE_MIN).min(cc_proto::MESSAGE_PAYLOAD_MAX)
            };
            let mut p = MsgBuf::new();
            loop {
                let left = room - p.len();
                if let Some(m) = self.urgent.front() {
                    if m.len() <= left {
                        p.extend(m.as_slice())?;
                        self.urgent.pop_front();
                        continue;
                    }
                    break;
                }
                match self.sched.front() {
                    Some(s) if s.min_clock <= est && s.msg.len() <= left => {
                        let margin = s.req_clock as f64 - est as f64;
                        if margin < self.clock.freq * 0.025 {
                            self.late_sends += 1;
                            if self.debug {
                                eprintln!("[{}] late send: {:.1} ms before due (min_clock {:.1} ms before), {} queued, {} in flight, rto {:.1} ms",
                                    self.name, margin / self.clock.freq * 1e3, (s.req_clock as f64 - s.min_clock as f64) / self.clock.freq * 1e3,
                                    self.sched.len(), self.inflight.len(), self.rto * 1e3);
                            }
                        }
                        p.extend(s.msg.as_slice())?;
                        self.sched.pop_front();
                        continue;
                    }
                    _ => {}
                }
                match self.held.front() {
                    Some((min, m)) if *min <= est && m.len() <= left => {
                        p.extend(m.as_slice())?;
                        self.held.pop_front();
                    }
                    _ => break,
                }
            }
            if p.is_empty() {
                break;
            }
            let frame = encode_frame(self.next_seq, p.as_slice())?;
            self.link.send(&frame)?;
            self.bytes_sent += frame.len() as u64;
            self.inflight_bytes += frame.len();
            self.inflight.push_back(Inflight { seq: self.next_seq, frame, sent: t, retransmitted: false });
            self.next_seq = (self.next_seq + 1) & 0x0f;
        }
        Ok(())
    }

    /// When the core should call [`service`](Self::service) again even if
    /// nothing arrives.
    pub fn deadline(&self) -> f64 {
        let mut d = self.next_clock_query;
        if let Some(f) = self.inflight.front() {
            d = d.min(f.sent + self.rto);
        }
        if let Some(s) = self.sched.front() {
            d = d.min(self.clock.time(s.min_clock));
        }
        if let Some((min, _)) = self.held.front() {
            d = d.min(self.clock.time(*min));
        }
        d
    }

    // ---- blocking helpers (bring-up, CLI) ----

    /// Resynchronise the MCU's frame parser and drop stale input.
    pub fn sync(&mut self) -> Result<()> {
        self.link.send(&[0x7e; 4])?;
        let deadline = now() + 0.05;
        while now() < deadline {
            self.link.recv(&mut self.rx[..], Duration::from_millis(10))?;
        }
        self.dec = FrameDecoder::new();
        Ok(())
    }

    /// Queue `cmd` and wait until everything queued is acknowledged.
    pub fn send_wait(&mut self, cmd: &str) -> Result<()> {
        self.queue(cmd)?;
        self.flush(2.0)
    }

    pub fn flush(&mut self, timeout: f64) -> Result<()> {
        let deadline = now() + timeout;
        while !(self.urgent.is_empty() && self.inflight.is_empty()) {
            if now() > deadline {
                return Err(format!("{}: no acknowledgement from the MCU", self.name).into());
            }
            self.service(Duration::from_millis(5))?;
        }
        Ok(())
    }

    /// Take the oldest queued message called `name`, waiting up to `timeout`.
    pub fn wait_for(&mut self, name: &str, timeout: Duration) -> Result<OwnedMessage> {
        let deadline = now() + timeout.as_secs_f64();
        loop {
            if let Some(i) = self.inbox.iter().position(|m| m.name() == name) {
                return Ok(self.inbox.remove(i).unwrap());
            }
            let t = now();
            if t >= deadline {
                return Err(format!("{}: timeout waiting for '{name}'", self.name).into());
            }
            self.service(Duration::from_secs_f64((deadline - t).min(0.01)))?;
        }
    }

    pub fn request(&mut self, cmd: &str, response: &str) -> Result<OwnedMessage> {
        self.inbox.retain(|m| m.name() != response);
        self.queue(cmd)?;
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

    /// Start clock synchronisation: a few quick `get_clock` round trips.
    pub fn sync_clock(&mut self) -> Result<()> {
        for _ in 0..8 {
            let sent = now();
            self.inbox.retain(|m| m.name() != "clock");
            self.clock_query = None;
            self.queue_msg(self.encode("get_clock", &[]));
            let m = self.wait_for("clock", Duration::from_secs(2))?;
            self.clock.sample(sent, now(), m.int("clock").unwrap_or(0) as u32);
            std::thread::sleep(Duration::from_millis(5));
        }
        self.next_clock_query = now() + 0.5;
        Ok(())
    }

    /// Measure the MCU clock frequency over `secs`.
    pub fn measure_clock(&mut self, secs: f64) -> Result<f64> {
        let t0 = now();
        let c0 = self.request("get_clock", "clock")?.int("clock").unwrap_or(0) as u32;
        std::thread::sleep(Duration::from_secs_f64(secs));
        let t1 = now();
        let c1 = self.request("get_clock", "clock")?.int("clock").unwrap_or(0) as u32;
        Ok(c1.wrapping_sub(c0) as f64 / (t1 - t0))
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

/// Inverse of [`ntc_temp`]: ADC fraction at `temp` °C.
pub fn ntc_frac(temp: f64, beta: f64) -> f64 {
    let (r0, t0, pullup) = (100_000.0f64, 25.0 + 273.15, 4700.0);
    let r = r0 * (beta * (1.0 / (temp + 273.15) - 1.0 / t0)).exp();
    r / (r + pullup)
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
        assert!((ntc_temp(ntc_frac(213.0, 4300.0), 4300.0).unwrap() - 213.0).abs() < 1e-6);
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

    fn mcu(replies: Vec<Vec<u8>>) -> Mcu {
        Mcu::new("t", Box::new(Scripted { replies: replies.into(), inbox: VecDeque::new() }), 84e6, 192)
    }

    #[test]
    fn send_retransmits_on_nak() {
        // The frame (seq 0) is answered by a NAK naming seq 0 again (the MCU
        // lost it); the retransmission is acked with seq 1.
        let mut m = mcu(vec![frame(0, None), frame(1, None)]);
        m.srtt = 0.0;
        m.send_wait("get_uptime").unwrap();
        assert_eq!(m.next_seq, 1);
        assert_eq!(m.retransmits, 1);
    }

    #[test]
    fn response_is_delivered() {
        let mut m = mcu(vec![frame(1, Some("clock clock=123456"))]);
        let r = m.request("get_clock", "clock").unwrap();
        assert_eq!(r.int("clock"), Some(123456));
    }

    #[test]
    fn identify_concatenates_chunks() {
        let mut m = mcu(vec![
            frame(1, Some("identify_response offset=0 data=abc")),
            frame(2, Some("identify_response offset=3 data=de")),
            frame(3, Some("identify_response offset=5 data=")),
        ]);
        assert_eq!(m.identify().unwrap(), b"abcde");
    }

    #[test]
    fn messages_are_packed_into_frames() {
        let mut m = mcu(vec![]);
        for _ in 0..10 {
            m.queue("get_uptime").unwrap(); // 1 byte each
        }
        m.service(Duration::ZERO).unwrap();
        assert_eq!(m.inflight.len(), 1, "ten one-byte messages share a frame");
    }

    #[test]
    fn scheduled_messages_wait_for_their_clock() {
        let mut m = mcu(vec![]);
        m.clock.sample(now(), now(), 0);
        m.next_clock_query = f64::MAX;
        let soon = m.clock.clock(now() + 0.05);
        m.queue_scheduled(soon, soon, 3, m.encode("get_uptime", &[]));
        m.service(Duration::ZERO).unwrap();
        assert_eq!(m.inflight.len(), 0);
        std::thread::sleep(Duration::from_millis(60));
        m.service(Duration::ZERO).unwrap();
        assert_eq!(m.inflight.len(), 1);
        m.queue_scheduled(u64::MAX, u64::MAX, 3, m.encode("get_uptime", &[]));
        assert_eq!(m.withdraw(3), 1);
    }

    #[test]
    fn clock_sync_tracks_frequency() {
        let mut c = ClockSync::new(200e6);
        let f = 200e6 * (1.0 + 30e-6);
        for i in 0..200 {
            let t = 10.0 + i as f64;
            c.sample(t - 0.0001, t + 0.0001, ((t * f) as u64) as u32);
        }
        assert!((c.freq / f - 1.0).abs() < 2e-6, "{}", c.freq);
        assert!((c.clock(250.0) as f64 - 250.0 * f).abs() < f * 1e-5);
    }
}
