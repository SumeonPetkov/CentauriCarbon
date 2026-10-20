//! Trapezoidal velocity move queue (chelper/trapq.c): the committed moves on
//! the print-time line, split into accel/cruise/decel pieces, with "null"
//! moves filling time gaps, a head sentinel and a tail sentinel that holds
//! the last position forever. Indices: 0 = head sentinel, 1..=len = moves,
//! len+1 = tail sentinel.

use std::collections::VecDeque;

pub const NEVER_TIME: f64 = 9999999999999999.9;
const MAX_NULL_MOVE: f64 = 1.0;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TMove {
    pub print_time: f64,
    pub move_t: f64,
    pub start_v: f64,
    pub half_accel: f64,
    pub start_pos: [f64; 3],
    pub axes_r: [f64; 3],
}

impl TMove {
    /// `move_get_distance`.
    pub fn distance(&self, t: f64) -> f64 {
        (self.start_v + self.half_accel * t) * t
    }

    /// `move_get_coord`.
    pub fn coord(&self, t: f64) -> [f64; 3] {
        let d = self.distance(t);
        [self.start_pos[0] + self.axes_r[0] * d, self.start_pos[1] + self.axes_r[1] * d, self.start_pos[2] + self.axes_r[2] * d]
    }
}

#[derive(Debug, Clone, Default)]
pub struct TrapQ {
    moves: VecDeque<TMove>,
}

impl TrapQ {
    pub fn len(&self) -> usize {
        self.moves.len()
    }

    pub fn is_empty(&self) -> bool {
        self.moves.is_empty()
    }

    /// Move by list index, sentinels included (`trapq_check_sentinels`).
    pub fn get(&self, i: usize) -> TMove {
        if i == 0 {
            return TMove::default();
        }
        if let Some(m) = self.moves.get(i - 1) {
            return *m;
        }
        match self.moves.back() {
            Some(last) => TMove { print_time: last.print_time + last.move_t, move_t: NEVER_TIME, start_pos: last.coord(last.move_t), ..Default::default() },
            None => TMove { print_time: NEVER_TIME, move_t: NEVER_TIME, ..Default::default() },
        }
    }

    /// Index of the tail sentinel.
    pub fn tail(&self) -> usize {
        self.moves.len() + 1
    }

    /// `trapq_append`.
    #[allow(clippy::too_many_arguments)]
    pub fn append(&mut self, mut print_time: f64, accel_t: f64, cruise_t: f64, decel_t: f64, mut start_pos: [f64; 3], axes_r: [f64; 3], start_v: f64, cruise_v: f64, accel: f64) {
        if accel_t != 0.0 {
            let m = TMove { print_time, move_t: accel_t, start_v, half_accel: 0.5 * accel, start_pos, axes_r };
            self.add_move(m);
            print_time += accel_t;
            start_pos = m.coord(accel_t);
        }
        if cruise_t != 0.0 {
            let m = TMove { print_time, move_t: cruise_t, start_v: cruise_v, half_accel: 0.0, start_pos, axes_r };
            self.add_move(m);
            print_time += cruise_t;
            start_pos = m.coord(cruise_t);
        }
        if decel_t != 0.0 {
            self.add_move(TMove { print_time, move_t: decel_t, start_v: cruise_v, half_accel: -0.5 * accel, start_pos, axes_r });
        }
    }

    /// `trapq_add_move`: a null move fills a gap since the previous move.
    fn add_move(&mut self, m: TMove) {
        let prev = self.get(self.moves.len());
        if prev.print_time + prev.move_t < m.print_time {
            let print_time = if prev.print_time == 0.0 && m.print_time > MAX_NULL_MOVE { m.print_time - MAX_NULL_MOVE } else { prev.print_time + prev.move_t };
            self.moves.push_back(TMove { print_time, move_t: m.print_time - print_time, start_pos: m.start_pos, ..Default::default() });
        }
        self.moves.push_back(m);
    }

    /// `trapq_free_moves`: drop moves that ended by `print_time`.
    pub fn free_moves(&mut self, print_time: f64) {
        while self.moves.front().is_some_and(|m| m.print_time + m.move_t <= print_time) {
            self.moves.pop_front();
        }
    }

    pub fn clear(&mut self) {
        self.moves.clear();
    }

    /// Position at print time `t` (None if empty).
    pub fn position_at(&self, t: f64) -> Option<[f64; 3]> {
        let m = self.moves.iter().find(|m| m.print_time + m.move_t >= t).or(self.moves.back())?;
        Some(m.coord((t - m.print_time).clamp(0.0, m.move_t)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trapezoid_pieces_and_null_moves() {
        let mut q = TrapQ::default();
        q.append(10.0, 0.1, 0.5, 0.1, [0.0; 3], [1.0, 0.0, 0.0], 0.0, 10.0, 100.0);
        // a 1 s null move first (limited), then the three pieces
        assert_eq!(q.len(), 4);
        assert_eq!(q.get(1).print_time, 9.0);
        assert_eq!(q.get(1).move_t, 1.0);
        let end = q.get(q.tail());
        assert!((end.print_time - 10.7).abs() < 1e-12);
        assert!((end.start_pos[0] - (0.5 + 5.0 + 0.5)).abs() < 1e-12);
        // a gap: null move holding the next start
        q.append(11.0, 0.0, 1.0, 0.0, [6.0, 0.0, 0.0], [0.0, 1.0, 0.0], 0.0, 1.0, 0.0);
        let n = q.get(5);
        assert!((n.print_time - 10.7).abs() < 1e-12 && (n.move_t - 0.3).abs() < 1e-12 && n.start_v == 0.0);
        q.free_moves(10.75);
        assert!((q.get(1).print_time - 10.7).abs() < 1e-12, "the null move still runs");
    }
}
