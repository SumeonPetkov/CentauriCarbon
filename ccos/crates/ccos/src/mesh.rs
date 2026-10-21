//! The bed mesh as the stock firmware computes it (extras/bed_mesh.cpp):
//! the probe grid, the interpolated `ZMesh` (bicubic / Lagrange / direct),
//! the move splitter that adds the mesh to every G-code move, and the
//! profiles stored as `[besh_profile_<material>_<name>]` sections (the stock
//! spelling) in `printer.cfg` and `user_printer.cfg`.
//!
//! The commands and the probing are in `gcode/mesh.rs`.
//!
//! Kept stock bugs:
//! * `get_z_range` starts its maximum at the smallest positive double, so a
//!   mesh that is all below zero reports a maximum of 0.
//! * A loaded profile takes `mesh_y_pps` from its `mesh_x_pps` key.
//! * With fewer than 4 points on an axis, bicubic is "forced to lagrange"
//!   only in a copy of the settings: the mesh stays bicubic.
//!
//! Deviations (undefined behaviour in the C++):
//! * The interpolated matrix is sized x_count × y_count but indexed
//!   [y][x], and a profile is read into an x_count × y_count matrix; both
//!   only work when the grid is square. Here rows are Y and columns X.
//! * Bicubic control points past the end of a row (fewer than 3 points on
//!   an axis) repeat the last point.
//! * `direct` (`ALGORITHM=direct`) with `mesh_pps` above 0 sizes the mesh
//!   for the interpolated points but stores only the probed ones, and reads
//!   past them; here a direct mesh has no points in between.

use std::cell::Cell;

use crate::config::{to_string, Config};

pub const PROFILE_VERSION: i64 = 1;
pub const PROFILE_PREFIX: &str = "besh_profile";
pub const DEFAULT_PROFILE: &str = "default";
/// `m_FADE_DISABLE`.
pub const FADE_DISABLE: f64 = 2147483647.0;

/// `isclose(a, b, rel_tol, abs_tol = 0)`.
pub fn isclose(a: f64, b: f64, rel_tol: f64) -> bool {
    (a - b).abs() <= (rel_tol * a.abs().max(b.abs())).max(0.0)
}

fn constrain(v: f64, lo: f64, hi: f64) -> f64 {
    hi.min(lo.max(v))
}

fn lerp(t: f64, v0: f64, v1: f64) -> f64 {
    (1.0 - t) * v0 + t * v1
}

/// `parse_pair`: comma-separated `atof` values; one value is doubled when
/// `check` is off.
pub fn parse_pair(c: &Config, section: &str, option: &str, def: &str, check: bool) -> Vec<f64> {
    let s = c.get_string(section, option, def);
    let s = s.trim_matches(' ');
    let mut v: Vec<f64> = if s.is_empty() { Vec::new() } else { s.split(',').map(crate::config::parse_double).collect() };
    if check && v.len() != 2 {
        crate::gcode::console("bed_mesh: malformed value");
    } else if v.len() == 1 {
        v.push(v[0]);
    }
    // the stock code indexes [0] and [1] regardless
    while v.len() < 2 {
        v.push(0.0);
    }
    v
}

/// `mesh_config` / `orig_config`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MeshParams {
    pub radius: f64,
    pub origin: Vec<f64>,
    pub rri: i64,
    pub x_count: i64,
    pub y_count: i64,
    pub mesh_min: Vec<f64>,
    pub mesh_max: Vec<f64>,
    pub mesh_x_pps: i64,
    pub mesh_y_pps: i64,
    pub algo: String,
    pub tension: f64,
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
}

/// `ZMesh`: the probed grid and its interpolation. The X/Y lookup offsets
/// (`BED_MESH_OFFSET`) belong to the mesh object, as in the stock code.
#[derive(Debug, Clone)]
pub struct ZMesh {
    pub params: MeshParams,
    pub avg_z: f64,
    pub offsets: Cell<[f64; 2]>,
    x_min: f64,
    y_min: f64,
    pub mesh_x_count: usize,
    pub mesh_y_count: usize,
    x_mult: usize,
    y_mult: usize,
    x_dist: f64,
    y_dist: f64,
    /// Probed Z, rows = Y (front first), columns = X.
    pub probed: Vec<Vec<f64>>,
    /// Interpolated Z, rows = Y.
    pub matrix: Vec<Vec<f64>>,
}

impl ZMesh {
    pub fn new(params: MeshParams) -> Self {
        let (px, py) = (params.x_count.max(0) as usize, params.y_count.max(0) as usize);
        // "direct" with mesh_pps > 0 (ALGORITHM=direct) reads past the
        // probed matrix in the stock code; direct means no points between
        let (xpps, ypps) = if params.algo == "direct" { (0, 0) } else { (params.mesh_x_pps.max(0) as usize, params.mesh_y_pps.max(0) as usize) };
        let mesh_x_count = (px.max(1) - 1) * xpps + px;
        let mesh_y_count = (py.max(1) - 1) * ypps + py;
        Self {
            avg_z: 0.0,
            offsets: Cell::new([0.0; 2]),
            x_min: params.min_x,
            y_min: params.min_y,
            mesh_x_count,
            mesh_y_count,
            x_mult: xpps + 1,
            y_mult: ypps + 1,
            x_dist: (params.max_x - params.min_x) / (mesh_x_count as f64 - 1.0),
            y_dist: (params.max_y - params.min_y) / (mesh_y_count as f64 - 1.0),
            probed: Vec::new(),
            matrix: Vec::new(),
            params,
        }
    }

    /// `build_mesh`. An unknown algorithm is an empty `std::function` in
    /// the stock code, which throws `bad_function_call`.
    pub fn build(&mut self, z: Vec<Vec<f64>>) -> Result<(), String> {
        self.probed = z.clone();
        match self.params.algo.as_str() {
            "lagrange" => self.sample_lagrange(&z),
            "bicubic" => self.sample_bicubic(&z),
            "direct" => self.matrix = z,
            _ => return Err("bad_function_call".into()),
        }
        let (sum, n) = self.matrix.iter().flatten().fold((0.0, 0usize), |(s, n), v| (s + v, n + 1));
        self.avg_z = ((sum / n as f64) * 100.0).round() / 100.0;
        Ok(())
    }

    fn x_coord(&self, i: usize) -> f64 {
        self.x_min + self.x_dist * i as f64
    }

    fn y_coord(&self, i: usize) -> f64 {
        self.y_min + self.y_dist * i as f64
    }

    /// Probed points in place, zeros in between.
    fn seed(&mut self, z: &[Vec<f64>]) {
        let (xm, ym) = (self.x_mult, self.y_mult);
        self.matrix = (0..self.mesh_y_count)
            .map(|j| (0..self.mesh_x_count).map(|i| if i % xm != 0 || j % ym != 0 { 0.0 } else { z[j / ym][i / xm] }).collect())
            .collect();
    }

    fn sample_lagrange(&mut self, z: &[Vec<f64>]) {
        self.seed(z);
        let (xm, ym) = (self.x_mult, self.y_mult);
        let xpts: Vec<f64> = (0..self.params.x_count.max(0) as usize).map(|i| self.x_coord(i * xm)).collect();
        let ypts: Vec<f64> = (0..self.params.y_count.max(0) as usize).map(|i| self.y_coord(i * ym)).collect();
        for i in (0..self.mesh_y_count).filter(|i| i % ym == 0) {
            for j in (0..self.mesh_x_count).filter(|j| j % xm != 0) {
                let x = self.x_coord(j);
                self.matrix[i][j] = self.calc_lagrange(&xpts, x, i, 0);
            }
        }
        for i in 0..self.mesh_x_count {
            for j in (0..self.mesh_y_count).filter(|j| j % ym != 0) {
                let y = self.y_coord(j);
                self.matrix[j][i] = self.calc_lagrange(&ypts, y, i, 1);
            }
        }
    }

    fn calc_lagrange(&self, lpts: &[f64], c: f64, vec: usize, axis: usize) -> f64 {
        let mut total = 0.0;
        for i in 0..lpts.len() {
            let (mut n, mut d) = (1.0, 1.0);
            for j in (0..lpts.len()).filter(|&j| j != i) {
                n *= c - lpts[j];
                d *= lpts[i] - lpts[j];
            }
            let z = if axis == 0 { self.matrix[vec][i * self.x_mult] } else { self.matrix[i * self.y_mult][vec] };
            total += z * n / d;
        }
        total
    }

    fn sample_bicubic(&mut self, z: &[Vec<f64>]) {
        self.seed(z);
        let (xm, ym) = (self.x_mult, self.y_mult);
        let c = self.params.tension;
        for y in (0..self.mesh_y_count).filter(|y| y % ym == 0) {
            for x in (0..self.mesh_x_count).filter(|x| x % xm != 0) {
                let row = &self.matrix[y];
                let p = ctl_pts(x, xm, self.mesh_x_count, |i| row[i.min(row.len() - 1)], "x");
                self.matrix[y][x] = cardinal_spline(p, c);
            }
        }
        for x in 0..self.mesh_x_count {
            for y in (0..self.mesh_y_count).filter(|y| y % ym != 0) {
                let m = &self.matrix;
                let p = ctl_pts(y, ym, self.mesh_y_count, |i| m[i.min(m.len() - 1)][x], "y");
                self.matrix[y][x] = cardinal_spline(p, c);
            }
        }
    }

    /// `_get_linear_index` → (t, index).
    fn linear_index(&self, coord: f64, axis: usize) -> (f64, usize) {
        let (min, cnt, dist) = if axis == 0 { (self.x_min, self.mesh_x_count, self.x_dist) } else { (self.y_min, self.mesh_y_count, self.y_dist) };
        let idx = ((coord - min) / dist).floor() as i32 as f64;
        let idx = constrain(idx, 0.0, cnt as f64 - 2.0);
        let base = if axis == 0 { self.x_coord(idx as usize) } else { self.y_coord(idx as usize) };
        (constrain((coord - base) / dist, 0.0, 1.0), idx as usize)
    }

    /// `calc_z`: bilinear on the interpolated matrix.
    pub fn calc_z(&self, x: f64, y: f64) -> f64 {
        if self.matrix.is_empty() {
            return 0.0;
        }
        let off = self.offsets.get();
        let (tx, xi) = self.linear_index(x + off[0], 0);
        let (ty, yi) = self.linear_index(y + off[1], 1);
        let t = &self.matrix;
        let z0 = lerp(tx, t[yi][xi], t[yi][xi + 1]);
        let z1 = lerp(tx, t[yi + 1][xi], t[yi + 1][xi + 1]);
        lerp(ty, z0, z1)
    }

    /// `get_z_range` (its maximum starts at the smallest positive double).
    pub fn z_range(&self) -> (f64, f64) {
        if self.matrix.is_empty() {
            return (0.0, 0.0);
        }
        let mut lo = f64::MAX;
        let mut hi = f64::MIN_POSITIVE;
        for row in &self.matrix {
            for &v in row {
                lo = lo.min(v);
                hi = hi.max(v);
            }
        }
        (lo, hi)
    }

    /// `get_probed_matrix`: rounded to 1e-7.
    pub fn probed_rounded(&self) -> Vec<Vec<f64>> {
        self.probed.iter().map(|r| r.iter().map(|z| (z * 10e6).round() / 10e6).collect()).collect()
    }

    /// `print_probed_matrix` / `print_mesh` text.
    pub fn report(&self, move_z: f64) -> Vec<String> {
        let rows = |m: &[Vec<f64>]| -> Vec<String> { m.iter().map(|r| r.iter().map(|z| format!("{}\t", to_string(*z))).collect()).collect() };
        let mut v = vec!["Mesh Leveling Probed Z positions:".to_string()];
        v.extend(rows(&self.probed));
        v.push(format!("Mesh X,Y: {},{}", self.mesh_x_count, self.mesh_y_count));
        if move_z != 0.0 {
            v.push(format!("Search Height: {}", to_string(move_z)));
        }
        let off = self.offsets.get();
        v.push(format!("Mesh Offsets: X={} Y={}", to_string(off[0]), to_string(off[1])));
        v.push(format!("Mesh Average: {}", to_string(self.avg_z)));
        let (lo, hi) = self.z_range();
        v.push(format!("Mesh Range: min={} max={}", to_string(lo), to_string(hi)));
        v.push(format!("Interpolation Algorithm: {}", self.params.algo));
        v.push("Measured points:".into());
        v.extend(rows(&self.probed));
        v
    }
}

/// `_get_x_ctl_pts` / `_get_y_ctl_pts`: control points and t for index
/// `i` between probed points `mult` apart.
fn ctl_pts(i: usize, mult: usize, count: usize, at: impl Fn(usize) -> f64, axis: &str) -> [f64; 5] {
    let last_pt = count as i64 - 1 - mult as i64;
    let at_i = |k: i64| at(k.max(0) as usize);
    let (i, m) = (i as i64, mult as i64);
    if i < m {
        [at_i(0), at_i(0), at_i(m), at_i(2 * m), i as f64 / m as f32 as f64]
    } else if i > last_pt {
        [at_i(last_pt - m), at_i(last_pt), at_i(last_pt + m), at_i(last_pt + m), (i - last_pt) as f64 / m as f32 as f64]
    } else {
        let mut k = m;
        while k < last_pt {
            if i > k && i < k + m {
                return [at_i(k - m), at_i(k), at_i(k + m), at_i(k + 2 * m), (i - k) as f64 / m as f32 as f64];
            }
            k += m;
        }
        crate::gcode::console(&format!("bed_mesh: Error finding {axis} control points"));
        [0.0; 5]
    }
}

fn cardinal_spline(p: [f64; 5], tension: f64) -> f64 {
    let t = p[4];
    let (t2, t3) = (t * t, t * t * t);
    let m1 = tension * (p[2] - p[0]);
    let m2 = tension * (p[3] - p[1]);
    p[1] * (2.0 * t3 - 3.0 * t2 + 1.0) + p[2] * (-2.0 * t3 + 3.0 * t2) + m1 * (t3 - 2.0 * t2 + t) + m2 * (t3 - t2)
}

/// `MoveSplitter`: cuts an XY move wherever the mesh height changes by
/// `split_delta_z`, checking every `move_check_distance` mm.
#[derive(Debug, Clone)]
pub struct MoveSplitter {
    pub split_delta_z: f64,
    pub move_check_distance: f64,
    fade_offset: f64,
    prev: [f64; 4],
    next: [f64; 4],
    current: [f64; 4],
    z_factor: f64,
    z_offset: f64,
    pub traverse_complete: bool,
    distance_checked: f64,
    total_move_length: f64,
    axis_move: [bool; 4],
}

impl MoveSplitter {
    pub fn new(c: &Config, section: &str) -> Self {
        Self {
            split_delta_z: c.get_double(section, "split_delta_z", 0.025),
            move_check_distance: c.get_double(section, "move_check_distance", 5.0),
            fade_offset: 0.0,
            prev: [0.0; 4],
            next: [0.0; 4],
            current: [0.0; 4],
            z_factor: 0.0,
            z_offset: 0.0,
            traverse_complete: true,
            distance_checked: 0.0,
            total_move_length: 0.0,
            axis_move: [false; 4],
        }
    }

    pub fn initialize(&mut self, fade_offset: f64) {
        self.fade_offset = fade_offset;
    }

    fn calc_z_offset(&self, mesh: &ZMesh, pos: &[f64; 4]) -> f64 {
        let z = mesh.calc_z(pos[0], pos[1]);
        self.z_factor * (z - self.fade_offset) + self.fade_offset
    }

    pub fn build_move(&mut self, mesh: &ZMesh, prev: [f64; 4], next: [f64; 4], factor: f64) {
        self.prev = prev;
        self.next = next;
        self.current = prev;
        self.z_factor = factor;
        self.z_offset = self.calc_z_offset(mesh, &prev);
        self.traverse_complete = false;
        self.distance_checked = 0.0;
        let d: Vec<f64> = (0..4).map(|i| next[i] - prev[i]).collect();
        self.total_move_length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        for i in 0..4 {
            self.axis_move[i] = !isclose(d[i], 0.0, 1e-10);
        }
    }

    fn set_next_move(&mut self, distance_from_prev: f64) {
        let t = distance_from_prev / self.total_move_length;
        if !(0.0..=1.0).contains(&t) {
            crate::gcode::console("bed_mesh: Slice distance is negative or greater than entire move length");
        }
        for i in 0..4 {
            if self.axis_move[i] {
                self.current[i] = lerp(t, self.prev[i], self.next[i]);
            }
        }
    }

    /// The next piece of the move, with the mesh added to Z.
    pub fn split(&mut self, mesh: &ZMesh) -> Option<[f64; 4]> {
        if self.traverse_complete {
            return None;
        }
        if self.axis_move[0] || self.axis_move[1] {
            while self.distance_checked + self.move_check_distance < self.total_move_length {
                self.distance_checked += self.move_check_distance;
                self.set_next_move(self.distance_checked);
                let next_z = self.calc_z_offset(mesh, &self.current);
                if (next_z - self.z_offset).abs() >= self.split_delta_z {
                    self.z_offset = next_z;
                    let c = self.current;
                    return Some([c[0], c[1], c[2] + self.z_offset, c[3]]);
                }
            }
        }
        self.current = self.next;
        self.z_offset = self.calc_z_offset(mesh, &self.current);
        self.current[2] += self.z_offset;
        self.traverse_complete = true;
        Some(self.current)
    }
}

/// `BedMeshCalibrate`'s grid settings and generated points.
#[derive(Debug, Clone, Default)]
pub struct Calibrate {
    pub section: String,
    pub radius: f64,
    pub origin: Vec<f64>,
    pub rri: i64,
    pub mesh_min: Vec<f64>,
    pub mesh_max: Vec<f64>,
    pub cfg: MeshParams,
    pub orig: MeshParams,
    pub points: Vec<[f64; 2]>,
    pub orig_points: Vec<[f64; 2]>,
    /// `faulty_region_N_min/max`: (min corner, max corner).
    pub faulty_regions: Vec<([f64; 2], [f64; 2])>,
    /// Point index → the points probed instead (ordered by index).
    pub substituted: std::collections::BTreeMap<usize, Vec<[f64; 2]>>,
}

fn within(c: [f64; 2], min: [f64; 2], max: [f64; 2], tol: f64) -> bool {
    max[0] + tol >= c[0] && c[0] >= min[0] - tol && max[1] + tol >= c[1] && c[1] >= min[1] - tol
}

impl Calibrate {
    /// `BedMeshCalibrate::BedMeshCalibrate` (`secondary` = a non-zero
    /// mesh index with `multi_bed_mesh_enable`).
    pub fn new(c: &Config, section: &str, secondary: bool) -> Self {
        let mut s = Self { section: section.into(), rri: c.get_int(section, "relative_reference_index", 0), ..Default::default() };
        s.orig.rri = s.rri;
        s.init_config(c, secondary);
        s.generate_points();
        s.orig_points = s.points.clone();
        s
    }

    /// `set_mesh_config`.
    pub fn reload(&mut self, c: &Config, secondary: bool) {
        self.init_config(c, secondary);
        self.generate_points();
        self.orig_points = self.points.clone();
    }

    /// `_init_mesh_config`.
    fn init_config(&mut self, c: &Config, secondary: bool) {
        let s = self.section.clone();
        let s = s.as_str();
        let (min_x, min_y, max_x, max_y, x_cnt, y_cnt);
        self.radius = c.get_double(s, "mesh_radius", 0.0);
        if self.radius != 0.0 {
            // round beds (the stock code appends to the origin each time)
            self.origin.extend(c.get_string(s, "mesh_origin", "0, 0").trim_matches(' ').split(',').map(crate::config::parse_double));
            let n = c.get_int(s, "round_probe_count", 5);
            x_cnt = n;
            y_cnt = n;
            self.radius = (self.radius * 10.0).floor() / 10.0;
            self.orig.radius = self.radius;
            self.orig.origin = self.origin.clone();
            min_x = -self.radius;
            min_y = -self.radius;
            max_x = self.radius;
            max_y = self.radius;
        } else {
            let key = if secondary { "secondary_probe_count" } else { "probe_count" };
            let cnt = parse_pair(c, s, key, "3", false);
            x_cnt = cnt[0] as i64;
            y_cnt = cnt[1] as i64;
            let lo = parse_pair(c, s, "mesh_min", "", true);
            let hi = parse_pair(c, s, "mesh_max", "", true);
            (min_x, min_y, max_x, max_y) = (lo[0], lo[1], hi[0], hi[1]);
            if max_x <= min_x || max_y <= min_y {
                crate::gcode::console("bed_mesh: invalid min/max points");
            }
        }
        self.orig.x_count = x_cnt;
        self.cfg.x_count = x_cnt;
        self.orig.y_count = y_cnt;
        self.cfg.y_count = y_cnt;
        self.orig.mesh_min = vec![min_x, min_y];
        self.mesh_min = vec![min_x, min_y];
        self.orig.mesh_max = vec![max_x, max_y];
        self.mesh_max = vec![max_x, max_y];
        let (pps, algo, tension) = if secondary {
            (parse_pair(c, s, "secondary_mesh_pps", "2", false), c.get_string(s, "secondary_algorithm", "lagrange"), c.get_double(s, "secondary_bicubic_tension", 0.2))
        } else {
            (parse_pair(c, s, "mesh_pps", "2", false), c.get_string(s, "algorithm", "lagrange"), c.get_double(s, "bicubic_tension", 0.2))
        };
        self.orig.algo = algo.clone();
        self.cfg.algo = algo;
        self.orig.tension = tension;
        self.cfg.tension = tension;
        self.orig.mesh_x_pps = pps[0] as i64;
        self.cfg.mesh_x_pps = pps[0] as i64;
        self.orig.mesh_y_pps = pps[1] as i64;
        self.cfg.mesh_y_pps = pps[1] as i64;
        for i in 1..100 {
            let (mn, mx) = (format!("faulty_region_{i}_min"), format!("faulty_region_{i}_max"));
            if c.get_string(s, &mn, "").is_empty() {
                break;
            }
            let a = parse_pair(c, s, &mn, "", true);
            let b = parse_pair(c, s, &mx, "", true);
            let c1 = [a[0].min(b[0]), a[1].min(b[1])];
            let c3 = [a[0].max(b[0]), a[1].max(b[1])];
            let corners = |lo: [f64; 2], hi: [f64; 2]| [lo, [lo[0], hi[1]], hi, [hi[0], lo[1]]];
            for &(p1, p3) in &self.faulty_regions {
                if corners(p1, p3).iter().any(|&k| within(k, c1, c3, 0.0)) || corners(c1, c3).iter().any(|&k| within(k, p1, p3, 0.0)) {
                    crate::gcode::console("bed_mesh: Existing faulty_region_ overlaps added faulty_region_");
                }
            }
            self.faulty_regions.push((c1, c3));
        }
        self.verify_algorithm();
    }

    /// `_verify_algorithm`.
    pub fn verify_algorithm(&mut self) {
        let p = &self.cfg;
        if p.algo != "lagrange" && p.algo != "bicubic" {
            crate::gcode::console("bed_mesh: Unknown algorithm");
        }
        let max_cnt = p.x_count.max(p.y_count);
        let min_cnt = p.x_count.min(p.y_count);
        if p.mesh_x_pps.max(p.mesh_y_pps) == 0 {
            self.cfg.algo = "direct".into();
        } else if p.algo == "lagrange" && max_cnt > 6 {
            crate::gcode::console("bed_mesh: cannot exceed a probe_count of 6 when using lagrange interpolation");
        } else if p.algo == "bicubic" && min_cnt < 4 {
            if max_cnt > 6 {
                crate::gcode::console(&format!(
                    "bed_mesh: invalid probe_count option when using bicubic interpolation. Combination of 3 points on one axis with more than 6 on another is not permitted. Configured PrinterProbe Count:{} {}",
                    p.x_count, p.y_count
                ));
            } else {
                // stock: changes a copy, the mesh stays bicubic
                crate::gcode::console(&format!(
                    "bed_mesh: bicubic interpolation with a probe_count of less than 4 points detected. Forcing lagrange interpolation. Configured PrinterProbe Count:{} {}",
                    p.x_count, p.y_count
                ));
            }
        }
    }

    /// `_generate_points`: zig-zag rows from the front.
    pub fn generate_points(&mut self) {
        let (x_cnt, y_cnt) = (self.cfg.x_count, self.cfg.y_count);
        let (mut min_x, mut min_y, mut max_x, mut max_y) = (self.mesh_min[0], self.mesh_min[1], self.mesh_max[0], self.mesh_max[1]);
        let mut x_dist = (max_x - min_x) / (x_cnt - 1) as f64;
        let mut y_dist = (max_y - min_y) / (y_cnt - 1) as f64;
        x_dist = (x_dist * 100.0).floor() / 100.0;
        y_dist = (y_dist * 100.0).floor() / 100.0;
        if x_dist <= 1.0 || y_dist <= 1.0 {
            crate::gcode::console("bed_mesh: min/max points too close together");
        }
        if self.radius != 0.0 {
            y_dist = x_dist;
            let new_r = ((x_cnt / 2) as f64 * x_dist) as i64 as f64;
            min_x = -new_r;
            min_y = -new_r;
            max_x = new_r;
            max_y = new_r;
        } else {
            max_x = min_x + x_dist * (x_cnt - 1) as f64;
        }
        let mut points = Vec::new();
        let mut pos_y = min_y;
        for i in 0..y_cnt.max(0) {
            for j in 0..x_cnt.max(0) {
                let pos_x = if i % 2 == 0 { min_x + j as f64 * x_dist } else { max_x - j as f64 * x_dist };
                if self.radius == 0.0 {
                    points.push([pos_x, pos_y]);
                } else if (pos_x * pos_x + pos_y * pos_y).sqrt() <= self.radius {
                    let o = |k: usize| self.origin.get(k).copied().unwrap_or(0.0);
                    points.push([o(0) + pos_x, o(1) + pos_y]);
                }
            }
            pos_y += y_dist;
        }
        self.points = points;
        if self.faulty_regions.is_empty() {
            return;
        }
        let mut last_y = self.points.first().map_or(0.0, |p| p[1]);
        let mut is_reversed = false;
        for (i, &coord) in self.points.clone().iter().enumerate() {
            if !isclose(coord[1], last_y, 1e-9) {
                is_reversed = !is_reversed;
            }
            last_y = coord[1];
            let mut adj = Vec::new();
            for &(min_c, max_c) in &self.faulty_regions {
                if within(coord, min_c, max_c, 0.00001) {
                    adj = vec![[min_c[0], coord[1]], [coord[0], min_c[1]], [coord[0], max_c[1]], [max_c[0], coord[1]]];
                    if is_reversed {
                        adj.swap(0, 3);
                    }
                    break;
                }
            }
            if adj.is_empty() {
                continue;
            }
            let valid: Vec<[f64; 2]> = adj
                .into_iter()
                .filter(|&ac| if self.radius == 0.0 { within(ac, [min_x, min_y], [max_x, max_y], 0.000001) } else { (ac[0] * ac[0] + ac[1] * ac[1]).sqrt() <= self.radius })
                .collect();
            if valid.is_empty() {
                crate::gcode::console(&format!("bed_mesh: Unable to generate coordinates for faulty region at index: {i}"));
            }
            self.substituted.insert(i, valid);
        }
    }

    /// `_get_adjusted_points`.
    pub fn adjusted_points(&self) -> Vec<[f64; 2]> {
        if self.substituted.is_empty() {
            return self.points.clone();
        }
        let mut adj = Vec::new();
        let mut last = 0;
        for (&i, sub) in &self.substituted {
            adj.extend_from_slice(&self.points[last..i]);
            adj.extend_from_slice(sub);
            last = i + 1;
        }
        adj.extend_from_slice(&self.points[last..]);
        adj
    }

    /// `relative_reference_index` as an index into the points, if it is
    /// one. A negative or too large index is undefined behaviour in the
    /// stock code; here it is ignored.
    pub fn rri_index(&self) -> Option<usize> {
        (self.rri > 0 && (self.rri as usize) < self.points.len()).then_some(self.rri as usize)
    }

    /// `print_generated_points`.
    pub fn generated_points_report(&self, offsets: [f64; 2]) -> Vec<String> {
        let mut v = vec!["bed_mesh: generated points".to_string(), "Index |  Tool Adjusted  |   Probe".to_string()];
        for (i, p) in self.points.iter().enumerate() {
            v.push(format!("  {i:<4}| ({:.1}, {:.1})  | ({:.1}, {:.1})", p[0] - offsets[0], p[1] - offsets[1], p[0], p[1]));
        }
        if let Some(r) = self.rri_index() {
            v.push(format!("bed_mesh: relative_reference_index {r} is ({:.2}, {:.2})", self.points[r][0], self.points[r][1]));
        }
        if !self.substituted.is_empty() {
            v.push("bed_mesh: faulty region points".into());
        }
        v
    }
}

/// `[besh_profile_<material>_<name>]`.
pub fn profile_section(material: &str, name: &str) -> String {
    format!("{PROFILE_PREFIX}_{material}_{name}")
}

/// `ProfileManager::save_profile`'s keys (std::to_string numbers).
pub fn write_profile(c: &mut Config, section: &str, mesh: &ZMesh) {
    let p = &mesh.params;
    let pts: Vec<String> = mesh.probed_rounded().iter().flatten().map(|z| to_string(*z)).collect();
    let pair = |v: &[f64]| v.iter().map(|x| to_string(*x)).collect::<Vec<_>>().join(", ");
    c.set_int(section, "version", PROFILE_VERSION);
    c.set_value(section, "points", &pts.join(", "));
    c.set_value(section, "mesh_max", &pair(&p.mesh_max));
    c.set_value(section, "mesh_min", &pair(&p.mesh_min));
    c.set_value(section, "algo", &p.algo);
    c.set_double(section, "max_x", p.max_x);
    c.set_double(section, "min_x", p.min_x);
    c.set_double(section, "max_y", p.max_y);
    c.set_double(section, "min_y", p.min_y);
    c.set_int(section, "mesh_x_pps", p.mesh_x_pps);
    c.set_int(section, "mesh_y_pps", p.mesh_y_pps);
    c.set_int(section, "x_count", p.x_count);
    c.set_int(section, "y_count", p.y_count);
}

/// `load_profile`'s reading of a section into `params` (which it changes
/// in place, like the stock code) and the probed grid, rows = Y.
pub fn read_profile(c: &Config, section: &str, params: &mut MeshParams) -> Vec<Vec<f64>> {
    params.mesh_max = parse_pair(c, section, "mesh_max", "", true);
    params.mesh_min = parse_pair(c, section, "mesh_min", "", true);
    let pts = c.get_doubles(section, "points");
    params.algo = c.get_string(section, "algo", "");
    params.max_x = c.get_double(section, "max_x", 0.0);
    params.min_x = c.get_double(section, "min_x", 0.0);
    params.max_y = c.get_double(section, "max_y", 0.0);
    params.min_y = c.get_double(section, "min_y", 0.0);
    params.mesh_x_pps = c.get_int(section, "mesh_x_pps", 2);
    // stock: mesh_y_pps is read from mesh_x_pps
    params.mesh_y_pps = c.get_int(section, "mesh_x_pps", 2);
    params.x_count = c.get_int(section, "x_count", 0);
    params.y_count = c.get_int(section, "y_count", 0);
    let (xc, yc) = (params.x_count.max(0) as usize, params.y_count.max(0) as usize);
    (0..yc).map(|r| (0..xc).map(|col| pts.get(r * xc + col).copied().unwrap_or(0.0)).collect()).collect()
}

/// One stored profile, for the screen.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ProfileView {
    pub section: String,
    pub material: String,
    pub name: String,
    pub x_count: usize,
    pub y_count: usize,
    /// Probed Z, rows = Y (front first).
    pub points: Vec<f64>,
    pub mesh_min: [f64; 2],
    pub mesh_max: [f64; 2],
    pub algo: String,
}

/// Every `[besh_profile_…]` section, `default` first, then by material and
/// name.
pub fn list_profiles(c: &Config) -> Vec<ProfileView> {
    let mut v: Vec<ProfileView> = c
        .sections
        .iter()
        .filter_map(|s| {
            let rest = s.name.strip_prefix(PROFILE_PREFIX)?.strip_prefix('_')?;
            let (material, name) = ["standard", "enhancement"]
                .iter()
                .find_map(|m| rest.strip_prefix(m).and_then(|r| r.strip_prefix('_')).map(|n| (m.to_string(), n.to_string())))
                .or_else(|| rest.split_once('_').map(|(m, n)| (m.to_string(), n.to_string())))?;
            let mut params = MeshParams::default();
            let rows = read_profile(c, &s.name, &mut params);
            Some(ProfileView {
                section: s.name.clone(),
                material,
                name,
                x_count: params.x_count.max(0) as usize,
                y_count: params.y_count.max(0) as usize,
                points: rows.into_iter().flatten().collect(),
                mesh_min: [params.min_x, params.min_y],
                mesh_max: [params.max_x, params.max_y],
                algo: params.algo,
            })
        })
        .collect();
    v.sort_by(|a, b| (a.name != DEFAULT_PROFILE, &a.material, &a.name).cmp(&(b.name != DEFAULT_PROFILE, &b.material, &b.name)));
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::STOCK_PRINTER_CFG;

    fn stock() -> Config {
        Config::parse(STOCK_PRINTER_CFG)
    }

    fn params(cal: &Calibrate) -> MeshParams {
        let mut p = cal.cfg.clone();
        p.mesh_min = cal.orig.mesh_min.clone();
        p.mesh_max = cal.orig.mesh_max.clone();
        let xs: Vec<f64> = cal.points.iter().map(|p| p[0]).collect();
        let ys: Vec<f64> = cal.points.iter().map(|p| p[1]).collect();
        p.min_x = xs.iter().cloned().fold(f64::MAX, f64::min);
        p.max_x = xs.iter().cloned().fold(f64::MIN, f64::max);
        p.min_y = ys.iter().cloned().fold(f64::MAX, f64::min);
        p.max_y = ys.iter().cloned().fold(f64::MIN, f64::max);
        p
    }

    #[test]
    fn stock_grid_is_11_by_11_zigzag() {
        let cal = Calibrate::new(&stock(), "bed_mesh", false);
        assert_eq!(cal.points.len(), 121);
        assert_eq!(cal.points[0], [20.0, 20.0]);
        assert_eq!(cal.points[10], [246.0, 20.0]);
        // second row runs back
        assert_eq!(cal.points[11], [246.0, 42.6]);
        assert!((cal.points[120][1] - 246.0).abs() < 1e-9);
        assert_eq!(cal.cfg.algo, "bicubic");
        let sec = Calibrate::new(&stock(), "bed_mesh", true);
        assert_eq!(sec.points.len(), 36);
        assert_eq!(sec.cfg.algo, "lagrange");
    }

    /// A plane is reproduced exactly by lagrange and direct; bicubic (a
    /// cardinal spline with tension 0.2) is exact on the probed points and
    /// close in between.
    #[test]
    fn interpolation_keeps_a_plane() {
        let cal = Calibrate::new(&stock(), "bed_mesh", false);
        let plane = |x: f64, y: f64| 0.002 * (x - 100.0) - 0.001 * (y - 50.0);
        for algo in ["bicubic", "lagrange", "direct"] {
            let mut p = params(&cal);
            p.algo = algo.into();
            if algo == "lagrange" {
                p.x_count = 5;
                p.y_count = 5;
            }
            let (xc, yc) = (p.x_count as usize, p.y_count as usize);
            let z: Vec<Vec<f64>> = (0..yc)
                .map(|r| (0..xc).map(|c| plane(p.min_x + (p.max_x - p.min_x) * c as f64 / (xc - 1) as f64, p.min_y + (p.max_y - p.min_y) * r as f64 / (yc - 1) as f64)).collect())
                .collect();
            let mut m = ZMesh::new(p);
            m.build(z).unwrap();
            for (x, y) in [(20.0, 20.0), (133.3, 77.7), (246.0, 246.0), (100.0, 200.0), (65.2, 155.6)] {
                let on_grid = [x, y].iter().all(|v: &f64| ((v - 20.0) / 22.6 - ((v - 20.0) / 22.6).round()).abs() < 1e-9);
                let tol = if algo == "bicubic" && !on_grid { 0.005 } else { 1e-9 };
                assert!((m.calc_z(x, y) - plane(x, y)).abs() < tol, "{algo} at {x},{y}: {} vs {}", m.calc_z(x, y), plane(x, y));
            }
        }
    }

    #[test]
    fn non_square_grid_works() {
        let mut cal = Calibrate::new(&stock(), "bed_mesh", false);
        cal.cfg.x_count = 5;
        cal.cfg.y_count = 3;
        cal.generate_points();
        let p = params(&cal);
        let z: Vec<Vec<f64>> = (0..3).map(|r| (0..5).map(|c| (r * 10 + c) as f64 * 0.01).collect()).collect();
        let mut m = ZMesh::new(p.clone());
        m.build(z.clone()).unwrap();
        assert_eq!(m.matrix.len(), m.mesh_y_count);
        assert_eq!(m.matrix[0].len(), m.mesh_x_count);
        assert!((m.calc_z(p.max_x, p.max_y) - 0.24).abs() < 1e-9);
        let mut c = Config::default();
        write_profile(&mut c, "besh_profile_standard_t", &m);
        let mut back = MeshParams::default();
        assert_eq!(read_profile(&c, "besh_profile_standard_t", &mut back), z);
        assert_eq!(back.x_count, 5);
    }

    #[test]
    fn splitter_follows_the_mesh() {
        let cal = Calibrate::new(&stock(), "bed_mesh", false);
        let p = params(&cal);
        let z: Vec<Vec<f64>> = (0..11).map(|_| (0..11).map(|c| c as f64 * 0.05).collect()).collect();
        let mut m = ZMesh::new(p);
        m.build(z).unwrap();
        let mut s = MoveSplitter::new(&stock(), "bed_mesh");
        s.build_move(&m, [20.0, 100.0, 0.2, 0.0], [246.0, 100.0, 0.2, 1.0], 1.0);
        let mut n = 0;
        let mut last = [0.0; 4];
        while let Some(p) = s.split(&m) {
            assert!((p[2] - 0.2 - m.calc_z(p[0], p[1])).abs() < 1e-9);
            n += 1;
            last = p;
        }
        assert!(n > 10, "split into {n}");
        assert_eq!(last[0], 246.0);
        assert!((last[2] - 0.7).abs() < 1e-9);
    }

    #[test]
    fn profiles_are_listed_default_first() {
        let cal = Calibrate::new(&stock(), "bed_mesh", false);
        let mut m = ZMesh::new(params(&cal));
        m.build(vec![vec![0.1; 11]; 11]).unwrap();
        let mut c = Config::default();
        write_profile(&mut c, "besh_profile_standard_pla", &m);
        write_profile(&mut c, "besh_profile_standard_default", &m);
        write_profile(&mut c, "besh_profile_enhancement_default", &m);
        let v = list_profiles(&c);
        let names: Vec<(&str, &str)> = v.iter().map(|p| (p.material.as_str(), p.name.as_str())).collect();
        assert_eq!(names, [("enhancement", "default"), ("standard", "default"), ("standard", "pla")]);
        assert_eq!(v[0].points.len(), 121);
        assert_eq!(c.get_string("besh_profile_standard_pla", "x_count", ""), "11");
        assert!(c.get_string("besh_profile_standard_pla", "points", "").starts_with("0.100000, 0.100000"));
    }
}
