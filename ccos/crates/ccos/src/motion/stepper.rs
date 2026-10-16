//! Step generation and compression for one stepper.
//!
//! The motor's position along a planned move phase is quadratic in time,
//! so step times are solved in closed form (a step is taken when the
//! commanded position crosses the middle between two step positions, as in
//! Klipper's `itersolve`). Step times are then packed into `queue_step`
//! messages — `count` steps at `last + k·interval + add·k(k-1)/2` — each
//! step within `max_error` *before* its exact time (Klipper's
//! `stepcompress.c` contract, default 25 µs).

use cc_proto::dict::Arg;
use cc_proto::{Dictionary, MsgBuf};

/// Largest gap between steps that a `queue_step` interval may span before a
/// `reset_step_clock` is used instead (Klipper's `CLOCK_DIFF_MAX`).
pub const CLOCK_DIFF_MAX: u64 = 3 << 28;

/// A generated message and when it may/must be sent.
#[derive(Clone, Copy, Debug)]
pub struct StepMsg {
    /// Clock of the first step it concerns (ordering and slot release).
    pub req_clock: u64,
    /// Not before this MCU clock (a `reset_step_clock` must wait until the
    /// previous move finished).
    pub min_clock: u64,
    /// Occupies a slot of the MCU's move queue.
    pub uses_slot: bool,
    pub oid: u8,
    pub msg: MsgBuf,
}

pub struct StepQueue {
    pub oid: u8,
    pub step_dist: f64,
    /// `!` on the dir pin: host-positive motion needs dir pin low.
    pub invert_dir: bool,
    /// Allowed early-step error in MCU ticks.
    pub max_error: u64,
    /// Commanded position in steps (host-positive).
    pub step_pos: i64,
    /// Motor position (mm) = step_pos · step_dist + mm_offset.
    pub mm_offset: f64,
    /// Actual MCU time of the last step queued.
    pub last_step_clock: u64,
    sent_dir: Option<bool>,
    /// The MCU dropped its step state (homing stop): reset the step clock first.
    pub need_reset: bool,
    pending: Vec<u64>,
    pending_dir: bool,
    pub out: Vec<StepMsg>,
    pub steps_generated: u64,
}

const DICT: Dictionary = Dictionary::centauri();

fn enc(name: &str, args: &[(&str, Arg<'_>)]) -> MsgBuf {
    let mut b = MsgBuf::new();
    DICT.encode(name, args, &mut b).expect("step message encodes");
    b
}

impl StepQueue {
    pub fn new(oid: u8, step_dist: f64, invert_dir: bool, max_error: u64) -> Self {
        Self {
            oid,
            step_dist,
            invert_dir,
            max_error,
            step_pos: 0,
            mm_offset: 0.0,
            last_step_clock: 0,
            sent_dir: None,
            need_reset: true,
            pending: Vec::with_capacity(4096),
            pending_dir: true,
            out: Vec::with_capacity(256),
            steps_generated: 0,
        }
    }

    /// Commanded motor position in mm.
    pub fn position_mm(&self) -> f64 {
        self.step_pos as f64 * self.step_dist + self.mm_offset
    }

    /// Define the current position (G92, homing) without moving.
    pub fn set_position_mm(&mut self, mm: f64) {
        self.mm_offset = mm - self.step_pos as f64 * self.step_dist;
    }

    /// The MCU reported its step counter (dir-pin-positive) after a stop.
    pub fn set_mcu_position(&mut self, mcu_steps: i64) {
        let mm = self.position_mm();
        let _ = mm;
        let host = if self.invert_dir { -mcu_steps } else { mcu_steps };
        // Keep mm_offset: the mm position follows the real step count.
        self.step_pos = host;
        self.pending.clear();
        self.sent_dir = None;
        self.need_reset = true;
    }

    /// Generate steps for one move phase.
    ///
    /// The motor position is `m0 + c·d(τ)` with `d(τ) = v0·τ + a·τ²/2` for
    /// `τ ∈ [0, dur]` from print time `t0`; only crossings inside the print
    /// time window `[ws, we]` are produced. `to_clock` maps print time to
    /// this stepper's MCU clock.
    #[allow(clippy::too_many_arguments)]
    pub fn gen_phase(&mut self, t0: f64, dur: f64, m0: f64, c: f64, v0: f64, a: f64, ws: f64, we: f64, to_clock: &dyn Fn(f64) -> u64) {
        if c == 0.0 || dur <= 0.0 {
            return;
        }
        let ts = (ws - t0).max(0.0);
        let te = (we - t0).min(dur);
        if te <= ts {
            return;
        }
        let dir = c > 0.0;
        let d_end = v0 * te + 0.5 * a * te * te;
        let sd = self.step_dist;
        loop {
            let n = self.step_pos as f64;
            let target = if dir { (n + 0.5) * sd } else { (n - 0.5) * sd } + self.mm_offset;
            let dstar = (target - m0) / c;
            if dstar > d_end + 1e-12 {
                break;
            }
            let dstar = dstar.max(0.0);
            let tau = if a == 0.0 {
                if v0 <= 0.0 {
                    break;
                }
                dstar / v0
            } else {
                let disc = (v0 * v0 + 2.0 * a * dstar).max(0.0);
                let den = v0 + disc.sqrt();
                if den <= 0.0 {
                    0.0
                } else {
                    2.0 * dstar / den
                }
            };
            self.push_step(dir, to_clock(t0 + tau));
            self.step_pos += if dir { 1 } else { -1 };
        }
    }

    fn push_step(&mut self, dir: bool, mut clock: u64) {
        if dir != self.pending_dir && !self.pending.is_empty() {
            self.flush();
        }
        self.pending_dir = dir;
        let floor = self.pending.last().copied().unwrap_or(self.last_step_clock);
        if clock <= floor && !(self.pending.is_empty() && self.need_reset) {
            clock = floor + 1;
        }
        self.pending.push(clock);
        self.steps_generated += 1;
    }

    /// Compress all generated steps into messages (appended to `out`).
    pub fn flush(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let first = self.pending[0];
        let mcu_dir = self.pending_dir != self.invert_dir;
        if self.sent_dir != Some(mcu_dir) {
            let m = enc("set_next_step_dir", &[("oid", Arg::Int(self.oid as i64)), ("dir", Arg::Int(mcu_dir as i64))]);
            self.out.push(StepMsg { req_clock: first, min_clock: 0, uses_slot: false, oid: self.oid, msg: m });
            self.sent_dir = Some(mcu_dir);
        }
        if self.need_reset || first.saturating_sub(self.last_step_clock) >= CLOCK_DIFF_MAX || first <= self.last_step_clock {
            // Only valid once the stepper finished its previous move.
            let min_clock = if self.need_reset { 0 } else { self.last_step_clock };
            // As if a step had happened one interval earlier, so the first
            // queue_step can cover the run from its start.
            let gap = self.pending.get(1).map_or(1000, |s| s - first).clamp(1, 1 << 28);
            let base = first.saturating_sub(gap);
            let m = enc("reset_step_clock", &[("oid", Arg::Int(self.oid as i64)), ("clock", Arg::Int(base as u32 as i64))]);
            self.out.push(StepMsg { req_clock: first, min_clock, uses_slot: false, oid: self.oid, msg: m });
            self.last_step_clock = base;
            self.need_reset = false;
        }
        let mut i = 0;
        while i < self.pending.len() {
            let (interval, count, add) = fit(self.last_step_clock, &self.pending[i..], self.max_error);
            let m = enc(
                "queue_step",
                &[("oid", Arg::Int(self.oid as i64)), ("interval", Arg::Int(interval as i64)), ("count", Arg::Int(count as i64)), ("add", Arg::Int(add as i64))],
            );
            self.out.push(StepMsg { req_clock: self.pending[i], min_clock: 0, uses_slot: true, oid: self.oid, msg: m });
            let k = count as u64;
            self.last_step_clock = (self.last_step_clock as i64 + (k * interval as u64) as i64 + add as i64 * (k * (k - 1) / 2) as i64) as u64;
            i += count as usize;
        }
        self.pending.clear();
    }
}

fn floor_div(a: i64, b: i64) -> i64 {
    a.div_euclid(b)
}

fn ceil_div(a: i64, b: i64) -> i64 {
    -((-a).div_euclid(b))
}

/// Longest prefix of `steps` (absolute clocks after `last`) that one
/// `queue_step` with this `add` can reproduce; returns (count, interval).
fn count_for(last: u64, steps: &[u64], max_error: u64, add: i64) -> (usize, i64) {
    let (mut lo_i, mut hi_i) = (1i64, u32::MAX as i64);
    let mut best = (0usize, 0i64);
    let mut prev = last;
    for (j, &s) in steps.iter().enumerate().take(65535) {
        let k = j as i64 + 1;
        let hi = (s - last) as i64;
        let err = max_error.min((s - prev) / 2) as i64;
        prev = s;
        let q = add * (k * (k - 1) / 2);
        lo_i = lo_i.max(ceil_div(hi - err - q, k));
        hi_i = hi_i.min(floor_div(hi - q, k));
        if lo_i > hi_i {
            break;
        }
        best = (k as usize, (lo_i + hi_i) / 2);
    }
    best
}

/// Choose (interval, count, add) covering as many steps as possible.
pub fn fit(last: u64, steps: &[u64], max_error: u64) -> (u32, u16, i16) {
    let (mut best_c, mut best_i) = count_for(last, steps, max_error, 0);
    let mut best_a = 0i64;
    let n = steps.len().min(65535);
    let mut l = 2usize.max(best_c + 1);
    let mut tried = std::collections::HashSet::new();
    tried.insert(0i64);
    while l <= n * 2 {
        let l2 = l.min(n);
        // Least-squares fit of the window midpoints to k·I + A·k(k-1)/2.
        let (mut s11, mut s12, mut s22, mut b1, mut b2) = (0f64, 0f64, 0f64, 0f64, 0f64);
        let mut prev = last;
        for (j, &s) in steps[..l2].iter().enumerate() {
            let k = j as f64 + 1.0;
            let q = k * (k - 1.0) / 2.0;
            let hi = (s - last) as f64;
            let err = max_error.min((s - prev) / 2) as f64;
            prev = s;
            let y = hi - err / 2.0;
            s11 += k * k;
            s12 += k * q;
            s22 += q * q;
            b1 += k * y;
            b2 += q * y;
        }
        let det = s11 * s22 - s12 * s12;
        if det.abs() > 1e-9 {
            let a = ((s11 * b2 - s12 * b1) / det).round() as i64;
            for cand in [a - 1, a, a + 1] {
                let cand = cand.clamp(i16::MIN as i64, i16::MAX as i64);
                if !tried.insert(cand) {
                    continue;
                }
                let (c, i) = count_for(last, steps, max_error, cand);
                if c > best_c || (c == best_c && cand.abs() < best_a.abs()) {
                    (best_c, best_i, best_a) = (c, i, cand);
                }
            }
        }
        if l2 == n {
            break;
        }
        l *= 2;
    }
    if best_c == 0 {
        // Cannot happen for strictly increasing steps; fall back to one step.
        return (((steps[0] - last) as u32).max(1), 1, 0);
    }
    (best_i as u32, best_c as u16, best_a as i16)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Expand messages back into step clocks.
    fn expand(last: u64, msgs: &[(u32, u16, i16)]) -> Vec<u64> {
        let mut v = Vec::new();
        let mut t = last as i64;
        for &(i, c, a) in msgs {
            let mut iv = i as i64;
            for _ in 0..c {
                t += iv;
                v.push(t as u64);
                iv += a as i64;
            }
        }
        v
    }

    fn compress_all(last: u64, steps: &[u64], err: u64) -> Vec<(u32, u16, i16)> {
        let mut out = Vec::new();
        let mut l = last;
        let mut i = 0;
        while i < steps.len() {
            let (iv, c, a) = fit(l, &steps[i..], err);
            out.push((iv, c, a));
            let k = c as i64;
            l = (l as i64 + k * iv as i64 + a as i64 * k * (k - 1) / 2) as u64;
            i += c as usize;
        }
        out
    }

    fn check(last: u64, steps: &[u64], err: u64) -> usize {
        let msgs = compress_all(last, steps, err);
        let got = expand(last, &msgs);
        assert_eq!(got.len(), steps.len());
        for (g, s) in got.iter().zip(steps) {
            assert!(*g <= *s && s - g <= err, "step {g} vs {s}");
        }
        msgs.len()
    }

    #[test]
    fn constant_speed_is_one_message() {
        let steps: Vec<u64> = (1..=5000).map(|k| 1000 + k * 2500).collect();
        assert_eq!(check(1000, &steps, 5000), 1);
    }

    #[test]
    fn acceleration_compresses_well() {
        // 0 -> 300 mm/s at 20000 mm/s², 0.0125 mm steps, 200 MHz.
        let (f, a, sd) = (200e6, 20000.0, 0.012460);
        let steps: Vec<u64> = (1..=(300.0f64 * 300.0 / (2.0 * a) / sd) as u64)
            .map(|n| ((2.0 * (n as f64 - 0.5) * sd / a).sqrt() * f) as u64 + 10_000)
            .collect();
        let n = check(10_000, &steps, 5000);
        assert!(n < steps.len() / 20, "{n} messages for {} steps", steps.len());
    }

    #[test]
    fn stepper_generates_exact_count_and_direction() {
        let mut q = StepQueue::new(3, 0.01, true, 5000);
        let clk = |t: f64| (t * 200e6) as u64;
        // 1 mm at constant 10 mm/s, starting at print time 1.0
        q.gen_phase(1.0, 0.1, 0.0, 1.0, 10.0, 0.0, 0.0, 10.0, &clk);
        q.flush();
        assert_eq!(q.step_pos, 100);
        assert_eq!(q.steps_generated, 100);
        let names: Vec<_> = q.out.iter().map(|m| DICT.decode(m.msg.as_slice()).next().unwrap().unwrap().to_string()).collect();
        assert_eq!(names[0], "set_next_step_dir oid=3 dir=0", "inverted dir pin");
        assert!(names[1].starts_with("reset_step_clock"));
        assert!(names[2].starts_with("queue_step oid=3 interval=") && names[2].contains("count=100"), "{names:?}");
        assert_eq!(names.len(), 3);
        // and back again, in two windows
        q.out.clear();
        q.gen_phase(2.0, 0.1, 1.0, -1.0, 10.0, 0.0, 0.0, 2.05, &clk);
        q.gen_phase(2.0, 0.1, 1.0, -1.0, 10.0, 0.0, 2.05, 10.0, &clk);
        q.flush();
        assert_eq!(q.step_pos, 0);
        let first = DICT.decode(q.out[0].msg.as_slice()).next().unwrap().unwrap().to_string();
        assert_eq!(first, "set_next_step_dir oid=3 dir=1");
    }
}
