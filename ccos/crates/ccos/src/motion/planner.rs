//! Motion planning: trapezoidal moves with look-ahead, a port of the
//! well-tested algorithm in Klipper's `toolhead.py` (junction deviation from
//! `square_corner_velocity`, "smoothed" acceleration via `max_accel_to_decel`,
//! lazy flushing of the look-ahead queue).
//!
//! Positions are `[x, y, z, e]` in mm. Committed moves become [`Segment`]s on
//! a time line (print time = host seconds, see `clock.rs`); the step
//! generator walks that time line.

use std::collections::VecDeque;

/// Time the look-ahead may hold before it must commit moves.
pub const LOOKAHEAD_FLUSH_TIME: f64 = 0.25;

#[derive(Debug, Clone, Copy)]
pub struct Limits {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_accel_to_decel: f64,
    pub square_corner_velocity: f64,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub max_extrude_only_velocity: f64,
    pub max_extrude_only_accel: f64,
    /// Extruder `instantaneous_corner_velocity`.
    pub instant_corner_v: f64,
}

impl Limits {
    pub fn junction_deviation(&self) -> f64 {
        let scv2 = self.square_corner_velocity * self.square_corner_velocity;
        scv2 * (2f64.sqrt() - 1.0) / self.max_accel
    }
}

#[derive(Debug, Clone)]
pub struct Move {
    pub start: [f64; 4],
    pub end: [f64; 4],
    pub axes_d: [f64; 4],
    pub axes_r: [f64; 4],
    pub move_d: f64,
    pub is_kinematic: bool,
    pub accel: f64,
    junction_deviation: f64,
    pub min_move_t: f64,
    max_start_v2: f64,
    max_cruise_v2: f64,
    delta_v2: f64,
    max_smoothed_v2: f64,
    smooth_delta_v2: f64,
    pub start_v: f64,
    pub cruise_v: f64,
    pub end_v: f64,
    pub accel_t: f64,
    pub cruise_t: f64,
    pub decel_t: f64,
    /// Lookahead callbacks fired with this move's end time when it is committed.
    pub callbacks: Vec<u32>,
}

impl Move {
    pub fn new(lim: &Limits, start: [f64; 4], end: [f64; 4], speed: f64) -> Self {
        let mut axes_d = [0.0; 4];
        for i in 0..4 {
            axes_d[i] = end[i] - start[i];
        }
        let mut move_d = (axes_d[0] * axes_d[0] + axes_d[1] * axes_d[1] + axes_d[2] * axes_d[2]).sqrt();
        let mut velocity = speed.min(lim.max_velocity);
        let mut accel = lim.max_accel;
        let mut is_kinematic = true;
        let mut end = end;
        let inv;
        // Elegoo: 1e-7 (upstream 1e-9)
        if move_d < 1e-7 {
            // extrude-only move
            end = [start[0], start[1], start[2], end[3]];
            axes_d[0] = 0.0;
            axes_d[1] = 0.0;
            axes_d[2] = 0.0;
            move_d = axes_d[3].abs();
            inv = if move_d > 0.0 { 1.0 / move_d } else { 0.0 };
            accel = 99_999_999.9;
            velocity = speed;
            is_kinematic = false;
        } else {
            inv = 1.0 / move_d;
        }
        let axes_r = axes_d.map(|d| d * inv);
        let min_move_t = if velocity > 0.0 { move_d / velocity } else { 0.0 };
        Self {
            start,
            end,
            axes_d,
            axes_r,
            move_d,
            is_kinematic,
            accel,
            junction_deviation: lim.junction_deviation(),
            min_move_t,
            max_start_v2: 0.0,
            max_cruise_v2: velocity * velocity,
            delta_v2: 2.0 * move_d * accel,
            max_smoothed_v2: 0.0,
            smooth_delta_v2: 2.0 * move_d * lim.max_accel_to_decel,
            start_v: 0.0,
            cruise_v: 0.0,
            end_v: 0.0,
            accel_t: 0.0,
            cruise_t: 0.0,
            decel_t: 0.0,
            callbacks: Vec::new(),
        }
    }

    pub fn limit_speed(&mut self, speed: f64, accel: f64) {
        let speed2 = speed * speed;
        if speed2 < self.max_cruise_v2 {
            self.max_cruise_v2 = speed2;
            self.min_move_t = self.move_d / speed;
        }
        self.accel = self.accel.min(accel);
        self.delta_v2 = 2.0 * self.move_d * self.accel;
        self.smooth_delta_v2 = self.smooth_delta_v2.min(self.delta_v2);
    }

    fn calc_junction(&mut self, prev: &Move, lim: &Limits) {
        if !self.is_kinematic || !prev.is_kinematic {
            return;
        }
        // extruder: instantaneous corner velocity
        let mut extruder_v2 = self.max_cruise_v2;
        let diff_r = self.axes_r[3] - prev.axes_r[3];
        if diff_r != 0.0 {
            extruder_v2 = (lim.instant_corner_v / diff_r.abs()).powi(2);
        }
        let (a, b) = (&self.axes_r, &prev.axes_r);
        let mut cos_theta = -(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
        if cos_theta > 0.999999 {
            return; // full reversal: stop at the junction
        }
        cos_theta = cos_theta.max(-0.999999);
        let sin_theta_d2 = (0.5 * (1.0 - cos_theta)).sqrt();
        let r_jd = sin_theta_d2 / (1.0 - sin_theta_d2);
        let tan_theta_d2 = sin_theta_d2 / (0.5 * (1.0 + cos_theta)).sqrt();
        let move_centripetal_v2 = 0.5 * self.move_d * tan_theta_d2 * self.accel;
        let prev_centripetal_v2 = 0.5 * prev.move_d * tan_theta_d2 * prev.accel;
        self.max_start_v2 = (r_jd * self.junction_deviation * self.accel)
            .min(r_jd * prev.junction_deviation * prev.accel)
            .min(move_centripetal_v2)
            .min(prev_centripetal_v2)
            .min(extruder_v2)
            .min(self.max_cruise_v2)
            .min(prev.max_cruise_v2)
            .min(prev.max_start_v2 + prev.delta_v2);
        self.max_smoothed_v2 = self.max_start_v2.min(prev.max_smoothed_v2 + prev.smooth_delta_v2);
    }

    fn set_junction(&mut self, start_v2: f64, cruise_v2: f64, end_v2: f64) {
        let half_inv_accel = 0.5 / self.accel;
        let accel_d = (cruise_v2 - start_v2) * half_inv_accel;
        let decel_d = (cruise_v2 - end_v2) * half_inv_accel;
        let cruise_d = self.move_d - accel_d - decel_d;
        self.start_v = start_v2.sqrt();
        self.cruise_v = cruise_v2.sqrt();
        self.end_v = end_v2.sqrt();
        self.accel_t = accel_d / ((self.start_v + self.cruise_v) * 0.5);
        self.cruise_t = cruise_d / self.cruise_v;
        self.decel_t = decel_d / ((self.end_v + self.cruise_v) * 0.5);
        for t in [&mut self.accel_t, &mut self.cruise_t, &mut self.decel_t] {
            if !t.is_finite() || *t < 0.0 {
                *t = 0.0;
            }
        }
    }

    pub fn duration(&self) -> f64 {
        self.accel_t + self.cruise_t + self.decel_t
    }
}

/// A committed move on the time line.
#[derive(Debug, Clone, Copy)]
pub struct Segment {
    pub print_time: f64,
    pub accel_t: f64,
    pub cruise_t: f64,
    pub decel_t: f64,
    pub start: [f64; 4],
    pub axes_r: [f64; 4],
    pub start_v: f64,
    pub cruise_v: f64,
    pub accel: f64,
    /// Moves X/Y/Z (goes to the kinematic trapq).
    pub kinematic: bool,
    /// Extruding with X/Y motion: pressure advance applies.
    pub can_pa: bool,
}

impl Segment {
    pub fn end_time(&self) -> f64 {
        self.print_time + self.accel_t + self.cruise_t + self.decel_t
    }

    /// The three phases: (start time, duration, distance at start, v0, a).
    pub fn phases(&self) -> [(f64, f64, f64, f64, f64); 3] {
        let accel_d = (self.start_v + self.cruise_v) * 0.5 * self.accel_t;
        let cruise_d = self.cruise_v * self.cruise_t;
        [
            (self.print_time, self.accel_t, 0.0, self.start_v, self.accel),
            (self.print_time + self.accel_t, self.cruise_t, accel_d, self.cruise_v, 0.0),
            (self.print_time + self.accel_t + self.cruise_t, self.decel_t, accel_d + cruise_d, self.cruise_v, -self.accel),
        ]
    }

    /// Position at print time `t` (clamped to the segment).
    pub fn position(&self, t: f64) -> [f64; 4] {
        let mut d = 0.0;
        for (t0, dur, d0, v0, a) in self.phases() {
            if t >= t0 {
                let tau = (t - t0).min(dur);
                d = d0 + v0 * tau + 0.5 * a * tau * tau;
            }
        }
        let mut p = self.start;
        for i in 0..4 {
            p[i] += self.axes_r[i] * d;
        }
        p
    }
}

/// Look-ahead queue plus the committed time line.
pub struct Planner {
    pub lim: Limits,
    queue: VecDeque<Move>,
    junction_flush: f64,
    /// End of the last committed move (print time).
    pub print_time: f64,
    pub segments: VecDeque<Segment>,
    /// Committed since the core last took them (for the trapqs).
    pub committed: Vec<Segment>,
    /// (callback token, print time) of committed moves' callbacks.
    pub fired: Vec<(u32, f64)>,
    /// Steps were generated up to here assuming no more motion: a new
    /// time line starts at least `kin_flush_delay` later.
    pub min_restart: f64,
    pub kin_flush_delay: f64,
}

impl Planner {
    pub fn new(lim: Limits) -> Self {
        Self {
            lim,
            queue: VecDeque::new(),
            junction_flush: LOOKAHEAD_FLUSH_TIME,
            print_time: 0.0,
            segments: VecDeque::new(),
            committed: Vec::new(),
            fired: Vec::new(),
            min_restart: 0.0,
            kin_flush_delay: 0.0,
        }
    }

    /// `calc_print_time` when the time line restarts.
    fn restart(&mut self, now: f64, start_delay: f64) {
        if self.print_time < now + 0.05 {
            self.print_time = self.print_time.max(now + start_delay);
        }
        self.print_time = self.print_time.max(self.min_restart + self.kin_flush_delay);
    }

    pub fn queued(&self) -> usize {
        self.queue.len()
    }

    /// Add a move (already checked against machine limits). `now` is the
    /// current print time; moves that get committed start no earlier than
    /// `now + start_delay`.
    pub fn add(&mut self, mut m: Move, now: f64, start_delay: f64) {
        if let Some(prev) = self.queue.back() {
            let prev = prev.clone();
            m.calc_junction(&prev, &self.lim);
        }
        self.junction_flush -= m.min_move_t;
        self.queue.push_back(m);
        if self.junction_flush <= 0.0 {
            self.flush(true, now, start_delay);
        }
    }

    /// Commit moves. `lazy`: only those whose speeds can no longer change.
    pub fn flush(&mut self, lazy: bool, now: f64, start_delay: f64) {
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
        let mut update_flush_count = lazy;
        let n = self.queue.len();
        let mut flush_count = n;
        let mut delayed: Vec<(usize, f64, f64)> = Vec::new();
        let (mut next_end_v2, mut next_smoothed_v2, mut peak_cruise_v2) = (0.0f64, 0.0f64, 0.0f64);
        for i in (0..n).rev() {
            let (max_start_v2, delta_v2, max_smoothed_v2, smooth_delta_v2, max_cruise_v2) = {
                let m = &self.queue[i];
                (m.max_start_v2, m.delta_v2, m.max_smoothed_v2, m.smooth_delta_v2, m.max_cruise_v2)
            };
            let reachable_start_v2 = next_end_v2 + delta_v2;
            let start_v2 = max_start_v2.min(reachable_start_v2);
            let reachable_smoothed_v2 = next_smoothed_v2 + smooth_delta_v2;
            let smoothed_v2 = max_smoothed_v2.min(reachable_smoothed_v2);
            if smoothed_v2 < reachable_smoothed_v2 {
                // It's possible for this move to accelerate
                if smoothed_v2 + smooth_delta_v2 > next_smoothed_v2 || !delayed.is_empty() {
                    // This move can decelerate or this is a full accel move after a full decel move
                    if update_flush_count && peak_cruise_v2 > 0.0 {
                        flush_count = i;
                        update_flush_count = false;
                    }
                    peak_cruise_v2 = max_cruise_v2.min((smoothed_v2 + reachable_smoothed_v2) * 0.5);
                    if !delayed.is_empty() {
                        if !update_flush_count && i < flush_count {
                            let mut mc_v2 = peak_cruise_v2;
                            for &(j, ms_v2, me_v2) in delayed.iter().rev() {
                                mc_v2 = mc_v2.min(ms_v2);
                                self.queue[j].set_junction(ms_v2.min(mc_v2), mc_v2, me_v2.min(mc_v2));
                            }
                        }
                        delayed.clear();
                    }
                }
                if !update_flush_count && i < flush_count {
                    let cruise_v2 = ((start_v2 + reachable_start_v2) * 0.5).min(max_cruise_v2).min(peak_cruise_v2);
                    self.queue[i].set_junction(start_v2.min(cruise_v2), cruise_v2, next_end_v2.min(cruise_v2));
                }
            } else {
                // Delay calculating this move until peak_cruise_v2 is known
                delayed.push((i, start_v2, next_end_v2));
            }
            next_end_v2 = start_v2;
            next_smoothed_v2 = smoothed_v2;
        }
        if update_flush_count || flush_count == 0 {
            return;
        }
        // Idle (or fell behind): start a fresh buffer.
        self.restart(now, start_delay);
        for m in self.queue.drain(..flush_count) {
            let end = self.print_time + m.duration();
            for cb in &m.callbacks {
                self.fired.push((*cb, end));
            }
            if m.move_d <= 0.0 {
                continue;
            }
            let seg = Segment {
                print_time: self.print_time,
                accel_t: m.accel_t,
                cruise_t: m.cruise_t,
                decel_t: m.decel_t,
                start: m.start,
                axes_r: m.axes_r,
                start_v: m.start_v,
                cruise_v: m.cruise_v,
                accel: m.accel,
                kinematic: m.is_kinematic,
                can_pa: m.axes_r[3] > 0.0 && (m.axes_d[0].abs() > 1e-15 || m.axes_d[1] != 0.0),
            };
            self.print_time += m.duration();
            self.segments.push_back(seg);
            self.committed.push(seg);
        }
    }

    /// `register_lookahead_callback`: run `token` when the last queued move
    /// is committed (with its end time); if nothing is queued, returns the
    /// time to use now (`get_last_move_time`).
    pub fn lookahead_callback(&mut self, token: u32, now: f64, start_delay: f64) -> Option<f64> {
        match self.queue.back_mut() {
            Some(m) => {
                m.callbacks.push(token);
                None
            }
            None => Some(self.last_move_time(now, start_delay)),
        }
    }

    /// `get_last_move_time`: end of the planned time line (after committing
    /// the look-ahead), or a fresh start point when idle.
    pub fn last_move_time(&mut self, now: f64, start_delay: f64) -> f64 {
        self.flush(false, now, start_delay);
        self.restart(now, start_delay);
        self.print_time
    }

    /// `toolhead.dwell(delay)`: the time line advances by `delay`.
    pub fn dwell(&mut self, delay: f64, now: f64, start_delay: f64) {
        let t = self.last_move_time(now, start_delay);
        self.print_time = t + delay.max(0.0);
    }

    /// Drop everything not yet executed (emergency stop, homing stop).
    pub fn clear(&mut self) {
        self.queue.clear();
        self.segments.clear();
        self.committed.clear();
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
    }

    /// Forget the executed moves (`set_position`: the time line no longer
    /// says where the toolhead is).
    pub fn clear_history(&mut self) {
        self.segments.clear();
    }

    /// Position on the time line at print time `t` (None when idle).
    pub fn position_at(&self, t: f64) -> Option<[f64; 4]> {
        let s = self.segments.iter().find(|s| s.end_time() >= t).or(self.segments.back())?;
        Some(s.position(t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    pub const LIM: Limits = Limits {
        max_velocity: 500.0,
        max_accel: 20000.0,
        max_accel_to_decel: 20000.0,
        square_corner_velocity: 9.0,
        max_z_velocity: 20.0,
        max_z_accel: 500.0,
        max_extrude_only_velocity: 60.0,
        max_extrude_only_accel: 5000.0,
        instant_corner_v: 1.0,
    };

    #[test]
    fn single_move_is_a_trapezoid() {
        let mut p = Planner::new(Limits { max_accel: 1000.0, max_accel_to_decel: 1000.0, ..LIM });
        p.add(Move::new(&p.lim, [0.0; 4], [100.0, 0.0, 0.0, 0.0], 100.0), 0.0, 0.25);
        p.flush(false, 0.0, 0.25);
        let s = p.segments[0];
        assert!((s.accel_t - 0.1).abs() < 1e-9 && (s.decel_t - 0.1).abs() < 1e-9);
        assert!((s.cruise_t - 0.9).abs() < 1e-9);
        assert!((p.print_time - (0.25 + 1.1)).abs() < 1e-9);
        let end = s.position(10.0);
        assert!((end[0] - 100.0).abs() < 1e-9);
    }

    #[test]
    fn straight_line_keeps_speed_through_junctions() {
        let mut p = Planner::new(LIM);
        for i in 0..10 {
            let x = i as f64 * 10.0;
            p.add(Move::new(&p.lim, [x, 0.0, 0.0, 0.0], [x + 10.0, 0.0, 0.0, 0.0], 200.0), 0.0, 0.25);
        }
        p.flush(false, 0.0, 0.25);
        let v: Vec<f64> = p.segments.iter().map(|s| s.cruise_v).collect();
        assert!(v[1..9].iter().all(|&c| (c - 200.0).abs() < 1e-6), "{v:?}");
        // 90° corner: limited by square_corner_velocity
        let mut p = Planner::new(LIM);
        p.add(Move::new(&p.lim, [0.0; 4], [50.0, 0.0, 0.0, 0.0], 200.0), 0.0, 0.25);
        p.add(Move::new(&p.lim, [50.0, 0.0, 0.0, 0.0], [50.0, 50.0, 0.0, 0.0], 200.0), 0.0, 0.25);
        p.flush(false, 0.0, 0.25);
        let corner = p.segments[1].start_v;
        assert!(corner > 8.0 && corner < 30.0, "{corner}");
    }
}
