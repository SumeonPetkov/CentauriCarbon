//! The rest of extras/strain_gauge.cpp: `PRTOUCH_READY`, `CHECK_BED_MESH`,
//! `MEASURE_GAP_TEST` and `NOZZLE_CLEAR`, on top of the probing in
//! `loadcell.rs`.
//!
//! Kept stock behaviour:
//! * `get_best_rdy_z` always interpolates over the corner probe points
//!   (`m_rdy_pos`), whatever it is given, and takes its right edge's X from
//!   a Z value.
//! * `CHECK_BED_MESH` counts a corner as bad when its probed Z is above
//!   `bed_max_err`; the difference to the mesh is worked out and unused.
//!   It compares against the mesh at the configured corners, not at the
//!   randomly moved points it probed.
//! * `_probe_times` retries a missed first probe 2 mm higher without moving
//!   there (only the start Z it reports changes).
//! * `measure_gap` reads load-cell samples the board never sends, so each
//!   measurement is 0 (the stock one returns 0 the same way).
//! * `NOZZLE_CLEAR` does nothing: its body is empty.
//!
//! Deviations (undefined behaviour in the C++):
//! * The corner and gap tables have 3 rows but 4 are read. The fourth
//!   corner is (max X, min Y), as in the commented-out stock lines; the
//!   fourth gap point is (max X − 1, min Y + 1) at 0.1.
//! * `MEASURE_GAP_TEST INDEX` outside 0..3 indexes past its table. Here it
//!   is refused.
//! * Without a `[bed_mesh]` the stock code dereferences null. Here the
//!   commands do nothing.

use cc_proto::dict::Arg;

use super::parse::{cpp_double, Abort, CmdResult, GCmd};
use super::Exec;
use crate::loadcell::StrainGaugeCfg;
use crate::machine::strain_gauge;
use crate::Target;

type Pos3 = [f64; 3];

/// `_get_linear2`: the line through p1, p2 (Z over X, or over Y), at po.
fn get_linear2(p1: Pos3, p2: Pos3, mut po: Pos3, is_base_x: bool) -> Pos3 {
    let k = if is_base_x { 0 } else { 1 };
    if (p1[k] - p2[k]).abs() < 0.001 {
        return po;
    }
    let a = (p2[2] - p1[2]) / (p2[k] - p1[k]);
    let b = p1[2] - p1[k] * a;
    po[2] = a * po[k] + b;
    po
}

/// A small generator for the random corner offsets (`std::mt19937` seeded
/// from `random_device` in the stock code).
struct Rng(u64);

impl Rng {
    fn new() -> Self {
        let t = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(1, |d| d.as_nanos() as u64);
        Rng(t | 1)
    }

    /// `uniform_real_distribution(lo, hi)`.
    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        lo + (self.0 >> 11) as f64 / (1u64 << 53) as f64 * (hi - lo)
    }
}

impl Exec {
    /// The `[strain_gauge]` commands of this file (registered with it).
    pub(super) fn bedcheck_handles(&self, key: &str) -> bool {
        matches!(key, "PRTOUCH_READY" | "CHECK_BED_MESH" | "MEASURE_GAP_TEST" | "NOZZLE_CLEAR") && self.core().m.strain_gauge.is_some()
    }

    pub(super) fn cmd_bedcheck(&mut self, key: &str, g: &GCmd) -> CmdResult {
        if let Some(l) = self.core().hw.motion_lock() {
            return Err(Abort(l.into()));
        }
        let Some(cfg) = self.sg_cfg() else { return Ok(()) };
        match key {
            "PRTOUCH_READY" => self.probe_ready(&cfg).map(|_| ()),
            "CHECK_BED_MESH" => {
                let auto = g.get_int("AUTO_G29", 0, None, None)? > 0;
                self.check_bed_mesh(&cfg, auto)
            }
            "MEASURE_GAP_TEST" => self.cmd_measure_gap_test(&cfg, g),
            _ => Ok(()),
        }
    }

    /// `m_val->m_rdy_pos` (probe corners) and `m_gap_pos`, from the mesh
    /// area as configured (`_handle_mcu_identify`).
    fn sg_corners(&self, cfg: &StrainGaugeCfg) -> Option<([Pos3; 4], [Pos3; 4], [f64; 2])> {
        let ([min_x, min_y], [max_x, max_y]) = self.mesh_area(true)?;
        let z = cfg.bed_max_err as f64 + 1.0;
        let rdy = [[min_x, min_y, z], [min_x, max_y, z], [max_x, max_y, z], [max_x, min_y, z]];
        let gap = [[max_x / 2.0, max_y - 30.0, 0.1], [max_x - 1.0, min_y - 1.0, 0.1], [min_x + 1.0, min_y + 1.0, 0.1], [max_x - 1.0, min_y + 1.0, 0.1]];
        let home = [(max_x - min_x) / 2.0 + min_x, (max_y - min_y) / 2.0 + min_y];
        Some((rdy, gap, home))
    }

    /// `_ck_g28ed`: home everything unless all axes are homed.
    fn ck_g28ed(&mut self) {
        if self.core().homed.iter().any(|h| !h) {
            self.single("G28");
        }
    }

    /// `_move(pos, speed)`: manual_move, then wait.
    fn bc_move(&mut self, pos: &[f64], speed: f64) -> CmdResult {
        let p = [pos.first().copied(), pos.get(1).copied(), pos.get(2).copied(), pos.get(3).copied()];
        self.sg_move(p, speed)
    }

    /// `_probe_times`: two probes that agree within `max_z_err` (up to
    /// `max_times` tries); their mean.
    fn probe_times(&mut self, cfg: &StrainGaugeCfg, max_times: i32, rdy: Pos3, speed: f64, min_dis: f64, max_z_err: f64) -> Result<f64, Abort> {
        let mut o_mm = 0.0;
        let mut rdy_z = rdy[2];
        let mut now = self.core().commanded;
        now[2] = rdy_z;
        self.bc_move(&now, cfg.g29_rdy_speed)?;
        self.bc_move(&rdy, cfg.g29_xy_speed)?;
        let e = self.core().commanded[3];
        for _ in 0..max_times {
            let (_, o0, ok) = self.probe_by_step([rdy[0], rdy[1], rdy_z, e], speed, min_dis, true)?;
            if !ok && rdy_z == rdy[2] {
                rdy_z += 2.0;
                continue;
            }
            let (_, o1, _) = self.probe_by_step([rdy[0], rdy[1], rdy_z, e], speed, min_dis, true)?;
            o_mm = (o0 + o1) / 2.0;
            if (o0 - o1).abs() <= max_z_err || !self.ck_sys_sta() {
                break;
            }
            self.sg.re_probe_cnt += 1;
        }
        Ok(o_mm)
    }

    /// `get_best_rdy_z`: the corners' plane at (x, y), at most `bed_max_err`.
    fn get_best_rdy_z(cfg: &StrainGaugeCfg, x: f64, y: f64, rdy: &[Pos3; 4]) -> f64 {
        let b = rdy;
        let left = get_linear2(b[0], b[1], [b[0][0], y, 0.0], false);
        let right = get_linear2(b[2], b[3], [b[0][2], y, 0.0], false);
        let mid = get_linear2(left, right, [x, y, 0.0], true);
        mid[2].min(cfg.bed_max_err as f64)
    }

    /// `probe_ready` (`PRTOUCH_READY`): probe the four mesh corners.
    fn probe_ready(&mut self, cfg: &StrainGaugeCfg) -> Result<bool, Abort> {
        if self.sg.jump_probe_ready {
            self.sg.jump_probe_ready = false;
            return Ok(false);
        }
        let (Some(([min_x, min_y], [max_x, max_y])), Some((_, gap, _))) = (self.mesh_area(false), self.sg_corners(cfg)) else {
            return Ok(false);
        };
        self.ck_g28ed();
        let z = cfg.bed_max_err as f64 + 1.0;
        let mut rdy = [[min_x, min_y, z], [min_x, max_y, z], [max_x, max_y, z], [max_x, min_y, z]];
        let mesh = self.take_mesh();
        let cells = self.core().m.hx711s.as_ref().map_or(0, |h| h.count).clamp(0, 4) as usize;
        for p in rdy.iter_mut().take(cells) {
            p[2] = self.probe_times(cfg, 3, *p, cfg.g29_speed, 10.0, 0.2)?;
        }
        if cfg.need_measure_gap {
            // in memory only, as in the stock code
            let mut c = self.core();
            for (k, i) in [("z_gap_00", 0), ("z_gap_01", 1), ("z_gap_11", 2), ("z_gap_10", 3)] {
                c.cfgs.printer.set_double("strain_gauge", k, gap[i][2]);
            }
        }
        let f = |v: f64| cpp_double(v);
        super::console(&format!("RDY_POS = [00={}, 01={}, 11={}, 10={}]", f(rdy[0][2]), f(rdy[1][2]), f(rdy[2][2]), f(rdy[3][2])));
        super::console(&format!("GAP_POS = [00={}, 01={}, 11={}, 10={}]", f(gap[0][2]), f(gap[1][2]), f(gap[2][2]), f(gap[3][2])));
        self.set_mesh(mesh);
        Ok(true)
    }

    /// `check_bed_mesh` (`CHECK_BED_MESH [AUTO_G29=1]`): probe near the four
    /// corners; re-level when most of them are far off.
    fn check_bed_mesh(&mut self, cfg: &StrainGaugeCfg, auto_g29: bool) -> CmdResult {
        let (Some(([min_x, min_y], [max_x, max_y])), Some((rdy_cfg, _, home))) = (self.mesh_area(false), self.sg_corners(cfg)) else {
            return Ok(());
        };
        let z = cfg.bed_max_err as f64 + 1.0;
        let mut r = Rng::new();
        let mut u = || r.uniform(2.0, 5.0);
        let mut rdy = [[min_x + u(), min_y + u(), z], [min_x + u(), max_y - u(), z], [max_x - u(), max_y - u(), z], [max_x - u(), min_y + u(), z]];
        self.sg.jump_probe_ready = true;
        let Some(mesh) = self.take_mesh() else {
            if auto_g29 {
                self.ck_g28ed();
                self.single("BED_MESH_CALIBRATE");
                self.single("CXSAVE_CONFIG");
            } else {
                super::console("The bed_mesh data is invalid and cannot be verified.");
            }
            return Ok(());
        };
        self.ck_g28ed();
        let e = self.core().commanded[3];
        self.bc_move(&[home[0], home[1], z, e], cfg.g29_xy_speed)?;
        for p in rdy.iter_mut() {
            p[2] = self.probe_times(cfg, 3, *p, cfg.g29_speed, 10.0, cfg.check_bed_mesh_max_err * 2.0)?;
            p[2] += if cfg.need_measure_gap { Self::get_best_rdy_z(cfg, p[0], p[1], &rdy_cfg) } else { 0.0 };
        }
        self.set_mesh(Some(mesh));
        let mut err_cnt = 0;
        for (i, p) in rdy.iter().enumerate() {
            let mesh_z = self.mesh_calc_z(rdy_cfg[i][0], rdy_cfg[i][1]).unwrap_or(0.0);
            let _err = (p[2] - mesh_z).abs();
            if p[2] > cfg.bed_max_err as f64 {
                err_cnt += 1;
            }
        }
        if err_cnt > 2 {
            if auto_g29 {
                self.sg.g29_cnt = 0;
                self.single("BED_MESH_CALIBRATE");
                self.single("CXSAVE_CONFIG");
            } else {
                super::console("The bed_mesh data is invalid and cannot be verified.");
            }
        }
        Ok(())
    }

    /// `HX711S::query_start(…, cycle_count)`: (re)start sampling.
    fn hx_query_start(&self, cycle_count: i64) -> CmdResult {
        if !self.ck_sys_sta() {
            return Ok(());
        }
        let args = [("oid", Arg::Int(strain_gauge::HX711S as i64)), ("times_read", Arg::Int(cycle_count as u16 as i64))];
        self.sg_send(Target::StrainGauge, "query_hx711s", &args)
    }

    /// `measure_gap`: Z down and up by `gap_dis_range` around the touch
    /// point while the load cells sample. The board sends no samples, so,
    /// like the stock code, this ends with "no data" and 0.
    fn measure_gap(&mut self, cfg: &StrainGaugeCfg, zero_z: f64) -> Result<f64, Abort> {
        let min_dis = cfg.gap_dis_range;
        let speed = cfg.gap_dis_range;
        let rd_cnt = (2.0 * 80.0 * (min_dis / speed)) as i64;
        let step_dist = self.core().m.steppers[2].step_dist;
        let step_cnt = (min_dis / step_dist) as i32;
        let step_us = (((min_dis / speed) * 1000.0 * 1000.0) / step_cnt as f64) as i32;
        let mut now = self.core().commanded;
        now[2] = zero_z + min_dis / 2.0;
        self.bc_move(&now, cfg.g29_rdy_speed)?;
        self.calibration_start(cfg.base_count)?;
        self.hx_query_start(rd_cnt)?;
        self.dirz_run(0, step_us as u32, step_cnt as u32, true)?;
        self.hx_query_start(rd_cnt)?;
        self.dirz_run(1, step_us as u32, step_cnt as u32, true)?;
        self.hx_query_start(0)?;
        // get_vals(): nothing came back
        Ok(0.0)
    }

    /// `_gap_times`: `max_times` gap measurements; the one past the middle.
    fn gap_times(&mut self, cfg: &StrainGaugeCfg, max_times: usize, zero: f64) -> Result<f64, Abort> {
        let mut gaps = Vec::new();
        let now = self.core().commanded;
        for _ in 0..max_times {
            self.bc_move(&[now[0], now[1], zero + 2.0, now[3]], cfg.g29_rdy_speed)?;
            self.shake_motor((cfg.shake_cnt / 2) as i32, cfg)?;
            gaps.push(self.measure_gap(cfg, zero)?);
        }
        gaps.sort_by(f64::total_cmp);
        let vals: String = gaps.iter().map(|v| format!("{} ", cpp_double(*v))).collect();
        super::console(&format!("Gap measure vals = {vals}"));
        let now = self.core().commanded;
        self.bc_move(&[now[0], now[1], cfg.bed_max_err as f64 + 1.0, now[3]], cfg.g29_rdy_speed)?;
        Ok(gaps[(max_times + 1) / 2])
    }

    /// `MEASURE_GAP_TEST [INDEX=0..3]`.
    fn cmd_measure_gap_test(&mut self, cfg: &StrainGaugeCfg, g: &GCmd) -> CmdResult {
        let Some((rdy, _, _)) = self.sg_corners(cfg) else { return Ok(()) };
        self.ck_g28ed();
        let index = g.get_int("INDEX", 0, None, None)?;
        let n_index = [0, 3, 1, 2];
        let Some(&i) = usize::try_from(index).ok().and_then(|i| n_index.get(i)) else {
            return Err(Abort(format!("MEASURE_GAP_TEST: INDEX={index} is not 0 to 3")));
        };
        let p = [rdy[i][0], rdy[i][1], cfg.bed_max_err as f64 + 1.0];
        self.bc_move(&p, cfg.g29_xy_speed)?;
        let zero_z = self.probe_times(cfg, 10, p, cfg.g29_speed, 10.0, 0.05)?;
        self.gap_times(cfg, 5, zero_z)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear2() {
        // z over y through (0, 0, 1) and (0, 10, 3): at y 5, z 2
        assert_eq!(get_linear2([0.0, 0.0, 1.0], [0.0, 10.0, 3.0], [7.0, 5.0, 0.0], false)[2], 2.0);
        // same y: unchanged
        assert_eq!(get_linear2([0.0, 1.0, 1.0], [5.0, 1.0, 3.0], [7.0, 5.0, 9.0], false)[2], 9.0);
        let mut r = Rng::new();
        for _ in 0..1000 {
            let v = r.uniform(2.0, 5.0);
            assert!((2.0..5.0).contains(&v));
        }
    }
}
