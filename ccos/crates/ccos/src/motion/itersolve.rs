//! Step times from the move queue, as the stock host computes them
//! (chelper/itersolve.c with kin_corexy.c, kin_shaper.c, kin_extruder.c):
//! each stepper's position is a function of print time — CoreXY `x ± y` of
//! the input-shaped X/Y, Z, or the extruder with smoothed pressure advance —
//! and a secant search finds when it crosses the middle between two steps.

use super::shaper::Pulses;
use super::stepper::StepQueue;
use super::trapq::{TMove, TrapQ};

const SEEK_TIME_RESET: f64 = 0.000100;

pub const AF_X: u8 = 1;
pub const AF_Y: u8 = 2;
pub const AF_Z: u8 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kin {
    /// CoreXY A: x + y.
    CorexyPlus,
    /// CoreXY B: x − y.
    CorexyMinus,
    Z,
    Extruder,
}

/// `struct stepper_kinematics` (+ `input_shaper` / `extruder_stepper`).
#[derive(Debug, Clone)]
pub struct StepperKin {
    pub kin: Kin,
    pub active_flags: u8,
    pub last_flush_time: f64,
    pub last_move_time: f64,
    pub gen_steps_pre_active: f64,
    pub gen_steps_post_active: f64,
    /// Input shaper pulses (X, Y) — CoreXY steppers only.
    pub sx: Pulses,
    pub sy: Pulses,
    /// Pressure advance (extruder only).
    pressure_advance: f64,
    half_smooth_time: f64,
    inv_half_smooth_time2: f64,
}

fn axis_position(m: &TMove, axis: usize, t: f64) -> f64 {
    m.start_pos[axis] + m.axes_r[axis] * m.distance(t)
}

/// `get_axis_position_across_moves`.
fn across_moves(tq: &TrapQ, mut idx: usize, axis: usize, mut time: f64) -> f64 {
    let mut m = tq.get(idx);
    while time < 0.0 {
        if idx <= 1 {
            // before the oldest move kept (never reached: moves are kept
            // for kin_flush_delay)
            return axis_position(&tq.get(1), axis, 0.0);
        }
        idx -= 1;
        m = tq.get(idx);
        time += m.move_t;
    }
    while time > m.move_t {
        time -= m.move_t;
        idx += 1;
        m = tq.get(idx);
    }
    axis_position(&m, axis, time)
}

/// `calc_position` of kin_shaper.c: the shaper convolution.
fn shaped(tq: &TrapQ, idx: usize, axis: usize, move_time: f64, sp: &Pulses) -> f64 {
    (0..sp.n).map(|i| sp.a[i] * across_moves(tq, idx, axis, move_time + sp.t[i])).sum()
}

fn extruder_integrate(base: f64, start_v: f64, half_accel: f64, start: f64, end: f64) -> f64 {
    let (half_v, sixth_a) = (0.5 * start_v, (1.0 / 3.0) * half_accel);
    let si = start * (base + start * (half_v + start * sixth_a));
    let ei = end * (base + end * (half_v + end * sixth_a));
    ei - si
}

fn extruder_integrate_time(base: f64, start_v: f64, half_accel: f64, start: f64, end: f64) -> f64 {
    let (half_b, third_v, eighth_a) = (0.5 * base, (1.0 / 3.0) * start_v, 0.25 * half_accel);
    let si = start * start * (half_b + start * (third_v + start * eighth_a));
    let ei = end * end * (half_b + end * (third_v + end * eighth_a));
    ei - si
}

fn pa_move_integrate(m: &TMove, pressure_advance: f64, base: f64, start: f64, end: f64, time_offset: f64) -> f64 {
    let start = start.max(0.0);
    let end = end.min(m.move_t);
    // axes_r.y of an extruder move: "can pressure advance"
    let pa = if m.axes_r[1] != 0.0 { pressure_advance } else { 0.0 };
    let base = base + pa * m.start_v;
    let start_v = m.start_v + pa * 2.0 * m.half_accel;
    let ha = m.half_accel;
    let iext = extruder_integrate(base, start_v, ha, start, end);
    let wgt_ext = extruder_integrate_time(base, start_v, ha, start, end);
    wgt_ext - time_offset * iext
}

fn pa_range_integrate(tq: &TrapQ, idx: usize, move_time: f64, pa: f64, hst: f64) -> f64 {
    let m = tq.get(idx);
    let (mut start, mut end) = (move_time - hst, move_time + hst);
    let start_base = m.start_pos[0];
    let mut res = pa_move_integrate(&m, pa, 0.0, start, move_time, start);
    res -= pa_move_integrate(&m, pa, 0.0, move_time, end, end);
    let mut pi = idx;
    while start < 0.0 && pi > 1 {
        pi -= 1;
        let prev = tq.get(pi);
        start += prev.move_t;
        res += pa_move_integrate(&prev, pa, prev.start_pos[0] - start_base, start, prev.move_t, start);
    }
    let (mut ni, mut cur) = (idx, m);
    while end > cur.move_t {
        end -= cur.move_t;
        ni += 1;
        cur = tq.get(ni);
        res -= pa_move_integrate(&cur, pa, cur.start_pos[0] - start_base, 0.0, end, end);
    }
    res
}

impl StepperKin {
    pub fn new(kin: Kin) -> Self {
        let active_flags = match kin {
            Kin::CorexyPlus | Kin::CorexyMinus => AF_X | AF_Y,
            Kin::Z => AF_Z,
            Kin::Extruder => AF_X,
        };
        Self {
            kin,
            active_flags,
            last_flush_time: 0.0,
            last_move_time: 0.0,
            gen_steps_pre_active: 0.0,
            gen_steps_post_active: 0.0,
            sx: Pulses::default(),
            sy: Pulses::default(),
            pressure_advance: 0.0,
            half_smooth_time: 0.0,
            inv_half_smooth_time2: 0.0,
        }
    }

    /// `input_shaper_set_shaper_params` (CoreXY steppers; others unshaped).
    pub fn set_shaper(&mut self, sx: Pulses, sy: Pulses) {
        if self.kin == Kin::Z || self.kin == Kin::Extruder {
            return;
        }
        self.sx = if self.active_flags & AF_X != 0 { sx } else { Pulses::default() };
        self.sy = if self.active_flags & AF_Y != 0 { sy } else { Pulses::default() };
        // shaper_note_generation_time
        let (mut pre, mut post) = (0.0f64, 0.0f64);
        for p in [self.sx, self.sy] {
            if p.n > 0 {
                pre = pre.max(p.t[p.n - 1]);
                post = post.max(-p.t[0]);
            }
        }
        self.gen_steps_pre_active = pre;
        self.gen_steps_post_active = post;
    }

    /// `extruder_set_pressure_advance`.
    pub fn set_pressure_advance(&mut self, pressure_advance: f64, smooth_time: f64) {
        let hst = smooth_time * 0.5;
        self.half_smooth_time = hst;
        self.gen_steps_pre_active = hst;
        self.gen_steps_post_active = hst;
        if hst == 0.0 {
            return;
        }
        self.inv_half_smooth_time2 = 1.0 / (hst * hst);
        self.pressure_advance = pressure_advance;
    }

    /// The stepper's position at `move_time` into move `idx`.
    pub fn calc_position(&self, tq: &TrapQ, idx: usize, move_time: f64) -> f64 {
        let m = tq.get(idx);
        match self.kin {
            Kin::Z => m.coord(move_time)[2],
            Kin::Extruder => {
                if self.half_smooth_time == 0.0 {
                    return m.start_pos[0] + m.distance(move_time);
                }
                let area = pa_range_integrate(tq, idx, move_time, self.pressure_advance, self.half_smooth_time);
                m.start_pos[0] + area * self.inv_half_smooth_time2
            }
            Kin::CorexyPlus | Kin::CorexyMinus => {
                let c = m.coord(move_time);
                let x = if self.sx.n > 0 { shaped(tq, idx, 0, move_time, &self.sx) } else { c[0] };
                let y = if self.sy.n > 0 { shaped(tq, idx, 1, move_time, &self.sy) } else { c[1] };
                if self.kin == Kin::CorexyPlus {
                    x + y
                } else {
                    x - y
                }
            }
        }
    }

    /// `itersolve_calc_position_from_coord`.
    pub fn position_from_coord(&self, pos: [f64; 4]) -> f64 {
        match self.kin {
            Kin::CorexyPlus => pos[0] + pos[1],
            Kin::CorexyMinus => pos[0] - pos[1],
            Kin::Z => pos[2],
            Kin::Extruder => pos[3],
        }
    }

    fn check_active(&self, m: &TMove) -> bool {
        let af = self.active_flags;
        (af & AF_X != 0 && m.axes_r[0] != 0.0) || (af & AF_Y != 0 && m.axes_r[1] != 0.0) || (af & AF_Z != 0 && m.axes_r[2] != 0.0)
    }

    /// `itersolve_gen_steps_range`.
    fn gen_steps_range(&self, tq: &TrapQ, idx: usize, abs_start: f64, abs_end: f64, sq: &mut StepQueue, to_clock: &dyn Fn(f64) -> u64) {
        let m = tq.get(idx);
        let half_step = 0.5 * sq.step_dist;
        let start = (abs_start - m.print_time).max(0.0);
        let end = (abs_end - m.print_time).min(m.move_t);
        let commanded = sq.position_mm();
        let mut old_guess = (start, commanded);
        let mut guess = old_guess;
        let mut sdir = sq.last_dir();
        let (mut is_dir_change, mut have_bracket, mut check_oscillate) = (false, false, false);
        let mut target = commanded + if sdir { half_step } else { -half_step };
        let (mut last_time, mut low_time) = (start, start);
        let mut high_time = (start + SEEK_TIME_RESET).min(end);
        loop {
            // secant method
            let guess_dist = guess.1 - target;
            let og_dist = old_guess.1 - target;
            let mut next_time = (old_guess.0 * guess_dist - guess.0 * og_dist) / (guess_dist - og_dist);
            if !(next_time > low_time && next_time < high_time) {
                if have_bracket {
                    // a poor guess: bisect
                    next_time = (low_time + high_time) * 0.5;
                    check_oscillate = false;
                } else if guess.0 >= end {
                    // no more steps in this range
                    break;
                } else {
                    // exponential search
                    next_time = high_time;
                    high_time = (2.0 * high_time - last_time).min(end);
                }
            }
            old_guess = guess;
            guess = (next_time, self.calc_position(tq, idx, next_time));
            let guess_dist = guess.1 - target;
            if guess_dist.abs() > 0.000000001 {
                let rel_dist = if sdir { guess_dist } else { -guess_dist };
                if rel_dist > 0.0 {
                    // past the target: a step is here
                    if have_bracket && old_guess.0 <= low_time {
                        if check_oscillate {
                            old_guess = guess;
                        }
                        check_oscillate = true;
                    }
                    high_time = guess.0;
                    have_bracket = true;
                } else if rel_dist < -(half_step + half_step + 0.000000010) {
                    // direction change
                    sdir = !sdir;
                    target = if sdir { target + half_step + half_step } else { target - half_step - half_step };
                    low_time = last_time;
                    high_time = guess.0;
                    is_dir_change = true;
                    have_bracket = true;
                    check_oscillate = false;
                } else {
                    low_time = guess.0;
                }
                if !have_bracket || high_time - low_time > 0.000000001 {
                    continue;
                }
            }
            sq.step(sdir, to_clock(m.print_time + guess.0));
            target = if sdir { target + half_step + half_step } else { target - half_step - half_step };
            let mut seek_time_delta = (1.5 * (guess.0 - last_time)).max(0.000000001);
            if is_dir_change && seek_time_delta > SEEK_TIME_RESET {
                seek_time_delta = SEEK_TIME_RESET;
            }
            last_time = guess.0;
            low_time = guess.0;
            high_time = (guess.0 + seek_time_delta).min(end);
            is_dir_change = false;
            have_bracket = false;
            check_oscillate = false;
        }
    }

    /// `itersolve_generate_steps`: steps up to `flush_time`, including the
    /// pre/post-active windows of shaped or smoothed steppers.
    pub fn generate_steps(&mut self, tq: &TrapQ, flush_time: f64, sq: &mut StepQueue, to_clock: &dyn Fn(f64) -> u64) {
        let last_flush_time = self.last_flush_time;
        self.last_flush_time = flush_time;
        let mut i = 0usize;
        loop {
            let m = tq.get(i);
            if last_flush_time >= m.print_time + m.move_t {
                i += 1;
            } else {
                break;
            }
        }
        let mut force_steps_time = self.last_move_time + self.gen_steps_post_active;
        let mut skip_count = 0usize;
        loop {
            let m = tq.get(i);
            let (move_start, move_end) = (m.print_time, m.print_time + m.move_t);
            if self.check_active(&m) {
                if skip_count > 0 && self.gen_steps_pre_active > 0.0 {
                    // steps leading up to this stepper's activity
                    let abs_start = (move_start - self.gen_steps_pre_active).max(last_flush_time).max(force_steps_time);
                    let mut pi = i - 1;
                    loop {
                        skip_count -= 1;
                        if skip_count != 0 && tq.get(pi).print_time > abs_start {
                            pi -= 1;
                        } else {
                            break;
                        }
                    }
                    while pi != i {
                        self.gen_steps_range(tq, pi, abs_start, flush_time, sq, to_clock);
                        pi += 1;
                    }
                }
                self.gen_steps_range(tq, i, last_flush_time, flush_time, sq, to_clock);
                if move_end >= flush_time {
                    self.last_move_time = flush_time;
                    return;
                }
                skip_count = 0;
                self.last_move_time = move_end;
                force_steps_time = self.last_move_time + self.gen_steps_post_active;
            } else {
                if move_start < force_steps_time {
                    // steps just past this stepper's activity
                    let abs_end = force_steps_time.min(flush_time);
                    self.gen_steps_range(tq, i, last_flush_time, abs_end, sq, to_clock);
                    skip_count = 1;
                } else {
                    skip_count += 1;
                }
                if flush_time + self.gen_steps_pre_active <= move_end {
                    return;
                }
            }
            i += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const F: f64 = 1e6;

    fn run(sk: &mut StepperKin, tq: &TrapQ, sq: &mut StepQueue, until: f64, chunk: f64) -> Vec<u64> {
        let clk = |t: f64| (t * F) as u64;
        let mut t = 0.0;
        while t < until {
            t = (t + chunk).min(until);
            sk.generate_steps(tq, t, sq, &clk);
        }
        sq.take_pending()
    }

    #[test]
    fn constant_speed_steps_at_half_step_crossings() {
        let mut tq = TrapQ::default();
        // 1 mm of Z at 10 mm/s from print time 2.0
        tq.append(2.0, 0.0, 0.1, 0.0, [0.0; 3], [0.0, 0.0, 1.0], 10.0, 10.0, 0.0);
        let mut sk = StepperKin::new(Kin::Z);
        let mut sq = StepQueue::new(0, 0.01, false, 25);
        let steps = run(&mut sk, &tq, &mut sq, 3.0, 0.013);
        assert_eq!(steps.len(), 100);
        assert_eq!(sq.step_pos, 100);
        for (k, s) in steps.iter().enumerate() {
            let exact = (2.0 + (k as f64 + 0.5) * 0.001) * F;
            assert!((*s as f64 - exact).abs() <= 1.0, "step {k}: {s} vs {exact}");
        }
    }

    #[test]
    fn shaper_keeps_constant_speed_and_end_position() {
        let mut tq = TrapQ::default();
        // accelerate, cruise, decelerate along X: 50 mm
        tq.append(2.0, 0.1, 0.4, 0.1, [0.0; 3], [1.0, 0.0, 0.0], 0.0, 100.0, 1000.0);
        let mut plain = StepperKin::new(Kin::CorexyPlus);
        let mut shaped = StepperKin::new(Kin::CorexyPlus);
        shaped.set_shaper(Pulses::new(2, 54.8, 0.06), Pulses::new(0, 46.4, 0.082));
        assert!(shaped.gen_steps_post_active > 0.0);
        // in the cruise, far from the corners, shaping changes nothing
        let mid = 4; // the cruise piece (after the 1 s null move and accel)
        assert_eq!(tq.get(3).half_accel, 0.0);
        for t in [0.1, 0.2, 0.3] {
            let a = plain.calc_position(&tq, mid - 1, t);
            let b = shaped.calc_position(&tq, mid - 1, t);
            assert!((a - b).abs() < 1e-9, "{a} {b}");
        }
        let mut q1 = StepQueue::new(0, 0.0125, false, 25);
        let mut q2 = StepQueue::new(0, 0.0125, false, 25);
        let s1 = run(&mut plain, &tq, &mut q1, 4.0, 0.05);
        let s2 = run(&mut shaped, &tq, &mut q2, 4.0, 0.05);
        assert_eq!(s1.len(), 4000);
        assert_eq!(s2.len(), 4000, "every step, the tail included");
        // shaped motion starts a little earlier and ends a little later
        assert!(s2[0] < s1[0] && s2[3999] > s1[3999]);
    }

    #[test]
    fn pressure_advance_pushes_ahead_while_accelerating() {
        let mut tq = TrapQ::default();
        // extruding move: E 0 → 2 mm, PA allowed (axes_r.y = 1)
        tq.append(2.0, 0.1, 0.1, 0.1, [0.0; 3], [1.0, 1.0, 0.0], 0.0, 10.0, 100.0);
        let mut e = StepperKin::new(Kin::Extruder);
        e.set_pressure_advance(0.04, 0.01);
        // mid-cruise: nominal + pa·v
        let idx = 3;
        let nominal = tq.get(idx).coord(0.05)[0];
        let pa = e.calc_position(&tq, idx, 0.05);
        assert!((pa - (nominal + 0.04 * 10.0)).abs() < 1e-9, "{pa} vs {nominal}");
        // the same total, later steps come back
        let mut sq = StepQueue::new(0, 0.001, false, 25);
        run(&mut e, &tq, &mut sq, 3.0, 0.05);
        assert_eq!(sq.step_pos, 2000);
        // decelerating pulls filament back: steps in both directions
        assert!(sq.steps_generated > 2000, "{}", sq.steps_generated);
    }
}
