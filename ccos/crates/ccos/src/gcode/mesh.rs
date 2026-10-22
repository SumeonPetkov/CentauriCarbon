//! Bed mesh commands and probing as the stock firmware does them
//! (extras/bed_mesh.cpp, extras/probe.cpp `ProbePointsHelper` and
//! `PrinterProbe::run_probe`, extras/auto_leveling.cpp `G29`/`G29.1`,
//! `StrainGaugeWrapper::run_G29_Z`). The mesh maths is in `crate::mesh`.
//!
//! * `BED_MESH_CALIBRATE`: reloads the grid (`BED_MESH_SET_INDEX INDEX=0`),
//!   probes every point with the load cells (`[bed_mesh_probe]`: 2 taps a
//!   point within 0.01 mm, retried up to 20 times), builds the mesh, uses
//!   it, and saves it as `default` (or `PROFILE=`/`SAVE=`).
//! * `G29`: the screen's auto-levelling: purge and wipe, heat (nozzle 140 °C,
//!   bed 60 °C), Z offset to 0, `G28 Z`, then `BED_MESH_CALIBRATE`.
//! * Every G-code move then goes through the mesh (`BedMesh::move`): split
//!   where the mesh height changes, Z raised by the mesh.
//!
//! ccos additions (not in the stock firmware):
//! * `BED_MESH_CALIBRATE S<°C>` heats the bed and waits before probing
//!   (the stock command never heats); `G29 S<°C>` uses that bed
//!   temperature instead of 60 °C.
//! * `BED_MESH_CALIBRATE <name>` saves the mesh under that name, like
//!   `PROFILE=<name>`; both can be combined: `BED_MESH_CALIBRATE S75 pla`.
//!   A word of the form `S<number>` is the temperature; names are letters,
//!   digits, `_`, `-` and `.`.
//! * PROBE_COUNT below 3 is refused before probing (it reads outside the
//!   mesh in the stock code).
//!
//! Kept stock bugs:
//! * `G29` is a "traditional" command, so `PROFILE=`/`PROBE_COUNT=` never
//!   reach it from a G-code line.
//! * `G29` sets the Z offset to 0 (`M8233 S0 P1`) and saves that.
//! * `BED_MESH_APPLICATIONS ENABLE=1` without an earlier `ENABLE=0` turns
//!   the mesh off.
//! * `BED_MESH_CALIBRATE METHOD=fast` that matches the last mesh keeps the
//!   reloaded `default` mesh and saves nothing; when it does not match it
//!   switches to the secondary 6×6 grid (`BED_MESH_SET_INDEX INDEX=1`).
//! * A probe error turns the heaters off and shuts the printer down.
//! * The round-bed extrapolation works on a copy (no effect).
//! * With a mesh in use, a move outside the limits is clamped on the first
//!   axis out of range before it is split (`check_endstops`), and the
//!   G-code position takes the clamped value.
//!
//! Deviations (undefined behaviour in the C++):
//! * `BED_MESH_OFFSET` writes into an empty vector; here it sets the mesh's
//!   X/Y lookup offsets, as Klipper does.
//! * `BED_MESH_APPLICATIONS ENABLE=1` after a new calibration uses a mesh
//!   the stock code has deleted; here the old mesh is still there.
//! * `METHOD=manual` registers ACCEPT/ABORT on an object that is gone at
//!   the end of the call; here it is refused.
//! * `G29` formats `M8233 S%.3f` with an int; here `S0.000`.
//! * `G29` without `[strain_gauge]` dereferences null; here those steps
//!   are skipped, and probing needs `[strain_gauge]`.

use std::rc::Rc;

use super::parse::{cpp_double, Abort, CmdResult, GCmd, Lim};
use super::Exec;
use crate::app::CalibEvent;
use crate::config::Config;
use crate::mesh::{self, Calibrate, MoveSplitter, ZMesh, DEFAULT_PROFILE, FADE_DISABLE};

/// `[bed_mesh_probe]` or `[probe]` (`PrinterProbe`), with
/// Define_reference.h defaults.
#[derive(Debug, Clone)]
pub(super) struct ProbeCfg {
    pub speed: f64,
    pub lift_speed: f64,
    pub x_offset: f64,
    pub y_offset: f64,
    pub samples: i64,
    pub edge_samples: i64,
    pub edge_compensation: f64,
    pub sample_retract_dist: f64,
    pub samples_result: String,
    pub samples_tolerance: f64,
    pub samples_tolerance_step: f64,
    pub edge_samples_tolerance: f64,
    pub edge_samples_tolerance_step: f64,
    pub fast_threshold: f64,
    pub samples_retries: i64,
}

impl ProbeCfg {
    pub(super) fn from_config(c: &Config, section: &str) -> Self {
        let speed = c.get_double(section, "speed", 5.0);
        Self {
            speed,
            lift_speed: c.get_double(section, "lift_speed", speed),
            x_offset: c.get_double(section, "x_offset", 0.0),
            y_offset: c.get_double(section, "y_offset", 0.0),
            samples: c.get_int(section, "samples", 2),
            edge_samples: c.get_int(section, "edge_samples", 2),
            edge_compensation: c.get_double(section, "edge_compensation", 0.0),
            sample_retract_dist: c.get_double(section, "sample_retract_dist", 5.0),
            samples_result: c.get_string(section, "samples_result", "average"),
            samples_tolerance: c.get_double(section, "samples_tolerance", 0.1),
            samples_tolerance_step: c.get_double(section, "samples_tolerance_step", 0.005),
            edge_samples_tolerance: c.get_double(section, "edge_samples_tolerance", 0.03),
            edge_samples_tolerance_step: c.get_double(section, "edge_samples_tolerance_step", 0.005),
            fast_threshold: c.get_double(section, "fast_autoleveling_threshold", 0.01),
            samples_retries: c.get_int(section, "samples_tolerance_retries", 2),
        }
    }
}

/// `BedMesh` + `BedMeshCalibrate` + `ProfileManager` + `ProbePointsHelper`
/// + the bed-mesh probe's Z offset.
pub(super) struct BedMesh {
    horizontal_move_z: f64,
    fade_start: f64,
    fade_end: f64,
    fade_dist: f64,
    base_fade_target: f64,
    fade_target: f64,
    log_fade_complete: bool,
    multi_bed_mesh_enable: bool,
    current_mesh_index: i64,
    platform_material: String,
    z_mesh: Option<Rc<ZMesh>>,
    old_z_mesh: Option<Rc<ZMesh>>,
    /// `BedMesh::m_last_position`.
    last_position: [f64; 4],
    splitter: MoveSplitter,
    bmc: Calibrate,
    pending_profile: String,
    current_profile: String,
    /// The `[besh_profile_…]` section of the mesh last loaded or saved.
    active_section: Option<String>,
    // ProbePointsHelper
    probe_points: Vec<[f64; 2]>,
    fast_points: Vec<[f64; 2]>,
    last_fast_z: Vec<f64>,
    results: Vec<[f64; 3]>,
    helper_speed: f64,
    helper_move_z: f64,
    lift_speed: f64,
    probe_offsets: [f64; 3],
    probe: ProbeCfg,
    /// `m_bed_mesh_probe->m_z_offset`.
    pub probe_z_offset: f64,
    /// `AutoLeveling::m_enable_fast_probe` (never set on this firmware).
    enable_fast_probe: bool,
    /// `[auto_leveling] enable_test_mode`.
    auto_leveling_test_mode: bool,
}

impl BedMesh {
    /// `BedMesh::BedMesh` (without the profile load, which needs the
    /// G-code position).
    pub(super) fn from_config(c: &Config) -> Option<Box<Self>> {
        const S: &str = "bed_mesh";
        if !c.has_section(S) {
            return None;
        }
        let fade_start = c.get_double(S, "fade_start", 1.0);
        let fade_end = c.get_double(S, "fade_end", 0.0);
        let fade_dist = fade_end - fade_start;
        let disabled = fade_dist <= 0.0;
        let bmc = Calibrate::new(c, S, false);
        let probe_points = bmc.adjusted_points();
        if probe_points.len() < 3 {
            super::console("Need at least %d probe points for bed_mesh");
        }
        let probe = ProbeCfg::from_config(c, "bed_mesh_probe");
        Some(Box::new(Self {
            horizontal_move_z: c.get_double(S, "horizontal_move_z", 5.0),
            fade_start: if disabled { FADE_DISABLE } else { fade_start },
            fade_end: if disabled { FADE_DISABLE } else { fade_end },
            fade_dist,
            base_fade_target: c.get_double(S, "fade_target", f64::MIN_POSITIVE),
            fade_target: 0.0,
            log_fade_complete: false,
            multi_bed_mesh_enable: c.get_bool(S, "multi_bed_mesh_enable", false),
            current_mesh_index: 0,
            platform_material: c.get_string(S, "platform_material", "standard"),
            z_mesh: None,
            old_z_mesh: None,
            last_position: [0.0; 4],
            splitter: MoveSplitter::new(c, S),
            bmc,
            pending_profile: String::new(),
            current_profile: String::new(),
            active_section: None,
            probe_points,
            fast_points: Vec::new(),
            last_fast_z: Vec::new(),
            results: Vec::new(),
            helper_speed: c.get_double(S, "speed", 50.0),
            helper_move_z: c.get_double(S, "horizontal_move_z", 5.0),
            lift_speed: c.get_double(S, "speed", 50.0),
            probe_offsets: [0.0; 3],
            probe_z_offset: -c.get_double("probe", "z_offset", 1.0),
            probe,
            enable_fast_probe: false,
            auto_leveling_test_mode: c.get_bool("auto_leveling", "enable_test_mode", false),
        }))
    }

    fn secondary(&self) -> bool {
        self.current_mesh_index != 0 && self.multi_bed_mesh_enable
    }

    /// `get_z_factor`.
    fn z_factor(&self, z: f64) -> f64 {
        if z >= self.fade_end {
            0.0
        } else if z >= self.fade_start {
            (self.fade_end - z) / self.fade_dist
        } else {
            1.0
        }
    }

    /// The `[besh_profile_…]` section `load_profile`/`save_profile` use.
    fn cfg_name(&self, name: &str, force_name: bool) -> String {
        if !force_name && self.current_mesh_index != 0 {
            mesh::profile_section(&self.platform_material, &self.current_mesh_index.to_string())
        } else {
            mesh::profile_section(&self.platform_material, name)
        }
    }
}

/// The ccos extras of `BED_MESH_CALIBRATE`: words without `=` after the
/// command. `S<number>` is a bed temperature, anything else a profile name.
fn calibrate_extras(g: &GCmd) -> Result<(Option<f64>, Option<String>), Abort> {
    let mut temp = None;
    let mut name: Option<String> = None;
    let mut quoted = false;
    for w in g.commandline.split(' ').skip(1).filter(|w| !w.is_empty()) {
        // an odd number of quotes opens or closes a quoted value
        let toggles = w.matches('"').count() % 2 == 1;
        if quoted || toggles {
            quoted ^= toggles;
            continue;
        }
        if w.starts_with([';', '#', '*']) {
            break;
        }
        if w.contains('=') {
            continue;
        }
        let bytes = w.as_bytes();
        if bytes.len() > 1 && (bytes[0] == b'S' || bytes[0] == b's') && w[1..].parse::<f64>().is_ok_and(|v| v.is_finite()) {
            if temp.is_some() {
                return Err(Abort(format!("BED_MESH_CALIBRATE: more than one bed temperature in '{}'", g.commandline)));
            }
            temp = Some(w[1..].parse::<f64>().unwrap_or(0.0));
            continue;
        }
        if !w.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.')) {
            return Err(Abort(format!("BED_MESH_CALIBRATE: '{w}' is not a valid profile name (letters, digits, _ - .)")));
        }
        if let Some(n) = &name {
            return Err(Abort(format!("BED_MESH_CALIBRATE: more than one profile name ('{n}', '{w}')")));
        }
        name = Some(w.to_string());
    }
    if g.has("S") {
        if temp.is_some() {
            return Err(Abort(format!("BED_MESH_CALIBRATE: more than one bed temperature in '{}'", g.commandline)));
        }
        temp = Some(g.f("S", 0.0)?);
    }
    Ok((temp, name))
}

/// `parse_cmd_int_pair(gcmd, option, check=false, minval=3)`.
fn cmd_int_pair(g: &GCmd, option: &str) -> Vec<i64> {
    let s = g.get_string(option, "");
    let mut v: Vec<i64> = s.trim_matches(' ').split(',').map(|t| t.trim_matches(' ')).filter(|t| !t.is_empty()).map(|t| super::parse::stoi(t).unwrap_or(0)).collect();
    if v.len() == 1 {
        v.push(v[0]);
    }
    if v.len() >= 2 && (v[0] < 3 || v[1] < 3) {
        super::console("bed_mesh must have a minimum");
    }
    v
}

impl Exec {
    /// `BedMesh` at start-up: the profile manager loads `default` of the
    /// configured material.
    pub(super) fn mesh_init(&mut self) {
        if self.bm.is_none() {
            return;
        }
        if let Err(e) = self.load_profile(DEFAULT_PROFILE, false) {
            self.log(format!("bed_mesh: {e}"));
        }
        self.mesh_publish();
    }

    pub(super) fn mesh_handles(&self, key: &str) -> bool {
        match key {
            "BED_MESH_OUTPUT" | "BED_MESH_MAP" | "BED_MESH_CLEAR" | "BED_MESH_OFFSET" | "BED_MESH_APPLICATIONS" | "BED_MESH_SET_INDEX" | "BED_MESH_CALIBRATE" => self.bm.is_some(),
            "G29" | "G29.1" => self.core().cfgs.printer.has_section("auto_leveling"),
            _ => false,
        }
    }

    pub(super) fn cmd_mesh(&mut self, key: &str, g: &GCmd) -> CmdResult {
        match key {
            "BED_MESH_OUTPUT" => self.cmd_bed_mesh_output(g),
            "BED_MESH_MAP" => Ok(()),
            "BED_MESH_CLEAR" => {
                self.set_mesh(None);
                Ok(())
            }
            "BED_MESH_OFFSET" => self.cmd_bed_mesh_offset(g),
            "BED_MESH_APPLICATIONS" => self.cmd_bed_mesh_applications(g),
            "BED_MESH_SET_INDEX" => self.cmd_bed_mesh_set_index(g),
            "BED_MESH_CALIBRATE" => {
                if let Some(l) = self.core().hw.motion_lock() {
                    return Err(Abort(l.into()));
                }
                self.cmd_bed_mesh_calibrate(g, None)
            }
            "G29" => {
                if let Some(l) = self.core().hw.motion_lock() {
                    return Err(Abort(l.into()));
                }
                self.cmd_g29(g)
            }
            _ => self.cmd_g29_1(g),
        }
    }

    fn bm(&mut self) -> &mut BedMesh {
        self.bm.as_mut().expect("bed mesh")
    }

    // ---- BedMesh: the move transform ----

    /// `BedMesh::set_mesh`.
    pub(super) fn set_mesh(&mut self, mesh: Option<Rc<ZMesh>>) {
        let Some(bm) = self.bm.as_mut() else { return };
        match &mesh {
            Some(m) if bm.fade_end != FADE_DISABLE => {
                bm.log_fade_complete = true;
                let (lo, hi) = m.z_range();
                if bm.base_fade_target == 0.0 {
                    bm.fade_target = m.avg_z;
                } else {
                    bm.fade_target = bm.base_fade_target;
                    if !(lo <= bm.fade_target && bm.fade_target <= hi) && bm.fade_target != 0.0 {
                        let err = bm.fade_target;
                        bm.fade_target = 0.0;
                        super::console(&format!("bed_mesh: ERROR, fade_target lies outside of mesh z range min: {lo:.4}, max: {hi:.4}, fade_target: {err:.4}"));
                    }
                }
                if bm.fade_dist <= lo.abs().max(hi.abs()) {
                    bm.fade_target = 0.0;
                    super::console(&format!(
                        "bed_mesh:  Mesh extends outside of the fade range, please see the fade_start and fade_end options in example-extras.cfg. fade distance: {:.2} mesh min: {lo:.4} mesh max: {hi:.4}",
                        bm.fade_dist
                    ));
                }
            }
            _ => bm.fade_target = 0.0,
        }
        // (the stock code's "disable the mesh" above is overwritten here)
        bm.z_mesh = mesh;
        let ft = bm.fade_target;
        bm.splitter.initialize(ft);
        self.reset_last_position();
        self.mesh_publish();
    }

    /// `BedMesh::get_position`: the toolhead position minus the mesh.
    pub(super) fn mesh_get_position(&mut self) -> [f64; 4] {
        let pos = self.core().commanded;
        let Some(bm) = self.bm.as_mut() else { return pos };
        match &bm.z_mesh {
            None => {
                bm.last_position = pos;
                bm.last_position[2] -= bm.fade_target;
            }
            Some(m) => {
                let max_adj = m.calc_z(pos[0], pos[1]);
                let z_adj = max_adj - bm.fade_target;
                let mut factor = 1.0;
                if pos[2].min(pos[2] - max_adj) >= bm.fade_end {
                    factor = 0.0;
                } else if pos[2].max(pos[2] - max_adj) >= bm.fade_start {
                    factor = ((bm.fade_end + bm.fade_target - pos[2]) / (bm.fade_dist - z_adj)).clamp(0.0, 1.0);
                }
                let final_z_adj = factor * z_adj + bm.fade_target;
                bm.last_position = [pos[0], pos[1], pos[2] - final_z_adj, pos[3]];
            }
        }
        bm.last_position
    }

    /// `ToolHead::move1` from the G-code position.
    fn move1(&mut self, pos: [f64; 4], speed: f64) -> Result<bool, Abort> {
        self.wait_buffer()?;
        Ok(self.core().add_move(pos, speed, true)?)
    }

    /// `BedMesh::move`.
    pub(super) fn mesh_move(&mut self, newpos: [f64; 4], speed: f64) -> Result<bool, Abort> {
        let bm = self.bm.as_mut().expect("bed mesh");
        let factor = bm.z_factor(newpos[2]);
        let Some(m) = bm.z_mesh.clone().filter(|_| factor != 0.0) else {
            if bm.log_fade_complete {
                bm.log_fade_complete = false;
                super::console(&format!("bed_mesh fade complete: Current Z: {:.4} fade_target: {:.4} ", newpos[2], bm.fade_target));
            }
            let ft = bm.fade_target;
            let r = self.move1([newpos[0], newpos[1], newpos[2] + ft, newpos[3]], speed);
            self.bm().last_position = newpos;
            return r;
        };
        let mut newpos = newpos;
        let last = bm.last_position;
        self.check_endstops(last, &mut newpos);
        self.last = newpos;
        self.bm().splitter.build_move(&m, last, newpos, factor);
        let mut ret = Ok(false);
        while !self.bm().splitter.traverse_complete {
            match self.bm().splitter.split(&m) {
                Some(p) => ret = self.move1(p, speed),
                None => super::console("Mesh Leveling: Error splitting move "),
            }
            if ret.is_err() {
                break;
            }
        }
        self.bm().last_position = newpos;
        ret
    }

    /// `BedMesh::check_move` → `CoreXYKinematics::check_endstops`: clamp the
    /// first axis outside the limits.
    fn check_endstops(&self, last: [f64; 4], newpos: &mut [f64; 4]) {
        let d: f64 = (0..3).map(|i| (newpos[i] - last[i]).powi(2)).sum::<f64>().sqrt();
        if d <= 1e-15 {
            return;
        }
        let c = self.core();
        for i in 0..3 {
            let (lo, hi) = if c.homed[i] { (c.m.axes[i].min, c.m.axes[i].max) } else { (1.0, -1.0) };
            let lim = if newpos[i] < lo {
                lo
            } else if newpos[i] > hi {
                hi
            } else {
                continue;
            };
            super::console(&format!("soft endstop hit on axis {i}, end_pos: {:.6}, limit: {:.6}", newpos[i], lim));
            newpos[i] = lim;
            return;
        }
    }

    // ---- ProfileManager ----

    /// `ProfileManager::load_profile`.
    fn load_profile(&mut self, name: &str, force_name: bool) -> CmdResult {
        let cfg_name = self.bm().cfg_name(name, force_name);
        super::console(&format!("bed mesh cfg_name:{cfg_name}"));
        let rows = {
            let c = self.p.lock();
            if !c.cfgs.printer.has_section(&cfg_name) {
                None
            } else {
                // parse_pair(points) and parse_pair(bed_mesh, offset_points) complain
                if c.cfgs.printer.get_doubles(&cfg_name, "points").len() != 2 {
                    super::console("bed_mesh: malformed value");
                }
                super::console("bed_mesh: malformed value");
                let bm = self.bm.as_mut().expect("bed mesh");
                Some(mesh::read_profile(&c.cfgs.printer, &cfg_name, &mut bm.bmc.cfg))
            }
        };
        let Some(rows) = rows else {
            self.set_mesh(None);
            return Ok(());
        };
        let mut m = ZMesh::new(self.bm().bmc.cfg.clone());
        m.build(rows)?;
        let bm = self.bm();
        bm.current_profile = name.to_string();
        bm.active_section = Some(cfg_name);
        self.set_mesh(Some(Rc::new(m)));
        Ok(())
    }

    /// `ProfileManager::save_profile`.
    fn save_profile(&mut self, name: &str, force_name: bool) {
        let Some(m) = self.bm().z_mesh.clone() else {
            super::console(&format!("Unable to save to profile [ {name} ], the bed has not been probed"));
            return;
        };
        let cfg_name = self.bm().cfg_name(name, force_name);
        {
            let mut c = self.core();
            mesh::write_profile(&mut c.cfgs.printer, &cfg_name, &m);
            c.cfgs.save_printer();
            c.cfgs.save_user(&cfg_name, &[]);
        }
        let bm = self.bm();
        bm.current_profile = name.to_string();
        bm.active_section = Some(cfg_name.clone());
        super::console(&format!(
            "Bed Mesh state has been saved to profile [ {name} ]\nfor the current session. The SAVE_CONFIG command will\nupdate the printer config file and restart the printer."
        ));
        self.mesh_publish();
        let c = self.core();
        let mut s = c.ui.lock();
        if let Some(i) = s.meshes.iter().position(|p| p.section == cfg_name) {
            s.mesh_sel = i;
        }
    }

    /// Hand the stored profiles to the screen.
    pub(super) fn mesh_publish(&self) {
        let Some(bm) = &self.bm else { return };
        let active = bm.z_mesh.as_ref().and(bm.active_section.clone());
        let c = self.core();
        let list = mesh::list_profiles(&c.cfgs.printer);
        let mut s = c.ui.lock();
        let shown = s.shown_mesh().map(|m| m.section.clone());
        s.mesh_sel = shown.and_then(|sec| list.iter().position(|p| p.section == sec)).unwrap_or(0);
        s.meshes = list;
        s.mesh_active = active;
    }

    // ---- commands ----

    fn cmd_bed_mesh_output(&mut self, g: &GCmd) -> CmdResult {
        let bm = self.bm.as_ref().expect("bed mesh");
        if g.i("PGP", 0)? != 0 {
            for l in bm.bmc.generated_points_report([bm.probe.x_offset, bm.probe.y_offset]) {
                super::console(&l);
            }
        } else if let Some(m) = &bm.z_mesh {
            for l in m.report(bm.horizontal_move_z) {
                super::console(&l);
            }
        } else {
            super::console("Bed has not been probed ");
        }
        Ok(())
    }

    fn cmd_bed_mesh_offset(&mut self, g: &GCmd) -> CmdResult {
        let Some(m) = self.bm().z_mesh.clone() else {
            super::console("No mesh loaded to offset");
            return Ok(());
        };
        let mut off = m.offsets.get();
        if g.has("X") {
            off[0] = g.f("X", 0.0)?;
        }
        if g.has("Y") {
            off[1] = g.f("Y", 0.0)?;
        }
        m.offsets.set(off);
        self.reset_last_position();
        Ok(())
    }

    fn cmd_bed_mesh_applications(&mut self, g: &GCmd) -> CmdResult {
        let bm = self.bm();
        if bm.old_z_mesh.is_none() && bm.z_mesh.is_none() {
            return Ok(());
        }
        if g.i("ENABLE", 1)? != 0 {
            let old = bm.old_z_mesh.clone();
            self.set_mesh(old);
        } else {
            bm.old_z_mesh = bm.z_mesh.clone();
            self.set_mesh(None);
        }
        Ok(())
    }

    /// `BED_MESH_SET_INDEX [INDEX] [TYPE]`: choose the grid and the
    /// print-plate material, apply its Z offset, load its `default`.
    fn cmd_bed_mesh_set_index(&mut self, g: &GCmd) -> CmdResult {
        let index = g.i("INDEX", 0)?;
        let material = g.get_string("TYPE", &self.bm().platform_material);
        let bm = self.bm();
        bm.platform_material = material.clone();
        bm.current_mesh_index = if bm.multi_bed_mesh_enable && index != 0 { index } else { 0 };
        if material == "standard" || material == "enhancement" {
            self.single("SET_GCODE_OFFSET Z=0 MOVE=0 MOVE_SPEED=5.0");
            let zo = {
                let c = self.core();
                let key = format!("{material}_z_offset");
                c.cfgs.printer.get_double("bed_mesh", &key, self.bm.as_ref().map_or(0.0, |b| b.probe_z_offset))
            };
            self.single(&format!("M8233 S{zo:.3} P1"));
            let mut c = self.core();
            c.cfgs.printer.set_value("bed_mesh", "platform_material", &material);
            c.cfgs.save_printer();
        }
        // set_mesh_config
        let secondary = self.bm().secondary();
        {
            let c = self.p.lock();
            let bm = self.bm.as_mut().expect("bed mesh");
            bm.bmc.reload(&c.cfgs.printer, secondary);
            bm.probe_points = bm.bmc.adjusted_points();
        }
        if self.bm().probe_points.len() < 3 {
            super::console("Need at least %d probe points for bed_mesh");
        }
        self.load_profile(DEFAULT_PROFILE, false)
    }

    /// `BED_MESH_CALIBRATE [PROFILE|SAVE=] [METHOD=] [PROBE_COUNT=]
    /// [ALGORITHM=] [RELATIVE_REFERENCE_INDEX=] [SAMPLES=…]`, plus the ccos
    /// `S<°C>` and `<name>` words. `bed_temp` comes from `G29 S`.
    fn cmd_bed_mesh_calibrate(&mut self, g: &GCmd, bed_temp: Option<f64>) -> CmdResult {
        let (temp, name) = calibrate_extras(g)?;
        let temp = temp.or(bed_temp);
        let mut pending = g.get_string("PROFILE", "");
        if pending.is_empty() {
            pending = g.get_string("SAVE", "");
        }
        if let Some(n) = &name {
            if !pending.is_empty() && pending != *n {
                return Err(Abort(format!("BED_MESH_CALIBRATE: two profile names ('{pending}' and '{n}')")));
            }
        }
        if !g.get_string("PROBE_COUNT", "").is_empty() {
            let cnt = cmd_int_pair(g, "PROBE_COUNT");
            if cnt.len() >= 2 && (cnt[0] < 3 || cnt[1] < 3) {
                return Err(Abort(format!("BED_MESH_CALIBRATE: PROBE_COUNT={} needs at least 3 points per axis", g.get_string("PROBE_COUNT", ""))));
            }
        }
        if let Some(t) = temp {
            let (lo, hi) = {
                let c = self.core();
                (c.heaters[1].cfg.min_temp, c.heaters[1].cfg.max_temp)
            };
            if !(lo..=hi).contains(&t) {
                return Err(Abort(format!("BED_MESH_CALIBRATE: bed temperature {t:.1} out of range ({lo:.1}:{hi:.1})")));
            }
        }
        self.single("BED_MESH_SET_INDEX INDEX=0");
        let pending = if pending.is_empty() { name.unwrap_or_default() } else { pending };
        self.bm().pending_profile = pending;
        if g.get_string("METHOD", "automatic") == "fast" {
            self.fast_points_from_last_mesh();
        }
        if let Some(t) = temp {
            super::console(&format!("BED_MESH_CALIBRATE: heating the bed to {t:.1} °C"));
            self.core().ui.lock().status = format!("Bed mesh: heating the bed to {t:.0} °C");
            self.set_temperature(1, t, true)?;
        }
        self.update_config(g)?;
        self.start_probe(g)?;
        self.bm().enable_fast_probe = false;
        Ok(())
    }

    /// The `METHOD=fast` set-up: the four corners of the current mesh.
    fn fast_points_from_last_mesh(&mut self) {
        let bm = self.bm();
        let Some(m) = bm.z_mesh.clone() else {
            bm.fast_points.clear();
            bm.last_fast_z.clear();
            super::console(" last auto leveling result is null, continue auto leveling step.....................................");
            return;
        };
        let pm = m.probed_rounded();
        if pm.is_empty() || pm[0].is_empty() {
            return;
        }
        let (cols, rows) = (pm[0].len(), pm.len());
        let last_row = &pm[rows - 1];
        bm.last_fast_z = vec![pm[0][0], pm[0][cols - 1], last_row[last_row.len() - 1], last_row[0]];
        let p = &bm.probe_points;
        let at = |i: usize| p.get(i).copied().unwrap_or([0.0; 2]);
        let n = p.len();
        bm.fast_points = if cols % 2 == 0 {
            vec![at(0), at(cols - 1), at(n.saturating_sub(cols)), at(n.saturating_sub(1))]
        } else {
            vec![at(0), at(cols - 1), at(n.saturating_sub(1)), at(n.saturating_sub(cols))]
        };
    }

    /// `BedMeshCalibrate::update_config`.
    fn update_config(&mut self, g: &GCmd) -> CmdResult {
        let bm = self.bm();
        let b = &mut bm.bmc;
        b.radius = b.orig.radius;
        b.origin = b.orig.origin.clone();
        b.rri = b.orig.rri;
        b.mesh_min = b.orig.mesh_min.clone();
        b.mesh_max = b.orig.mesh_max.clone();
        let o = b.orig.clone();
        b.cfg.radius = o.radius;
        b.cfg.origin = o.origin;
        b.cfg.rri = o.rri;
        b.cfg.x_count = o.x_count;
        b.cfg.y_count = o.y_count;
        b.cfg.mesh_min = o.mesh_min;
        b.cfg.mesh_max = o.mesh_max;
        b.cfg.mesh_x_pps = o.mesh_x_pps;
        b.cfg.mesh_y_pps = o.mesh_y_pps;
        b.cfg.algo = o.algo;
        b.cfg.tension = o.tension;
        let mut need = false;
        if g.has("RELATIVE_REFERENCE_INDEX") {
            let r = g.i("RELATIVE_REFERENCE_INDEX", i32::MIN as i64)?;
            self.bm().bmc.rri = if r < 0 { i32::MIN as i64 } else { r };
            need = true;
        }
        let b = &mut self.bm.as_mut().expect("bed mesh").bmc;
        if b.radius != 0.0 {
            if g.has("MESH_RADIUS") {
                b.radius = (g.f("MESH_RADIUS", 0.0)? * 10.0).floor() / 10.0;
                b.mesh_min = vec![-b.radius, -b.radius];
                b.mesh_max = vec![b.radius, b.radius];
                need = true;
            }
            if g.has("MESH_ORIGIN") {
                need = true;
            }
            if g.has("ROUND_PROBE_COUNT") {
                let n = g.get_int("ROUND_PROBE_COUNT", i32::MIN as i64, Some(3), None)?;
                b.cfg.x_count = n;
                b.cfg.y_count = n;
                need = true;
            }
        } else {
            if g.has("MESH_MIN") || g.has("MESH_MAX") {
                need = true;
            }
            if g.has("PROBE_COUNT") {
                let cnt = cmd_int_pair(g, "PROBE_COUNT");
                if cnt.len() >= 2 {
                    b.cfg.x_count = cnt[0];
                    b.cfg.y_count = cnt[1];
                    need = true;
                }
            }
        }
        if g.has("ALGORITHM") {
            b.cfg.algo = g.get_string("ALGORITHM", "");
            need = true;
        }
        if need {
            b.verify_algorithm();
            b.generate_points();
            super::console("Generating new points...");
            let bm = self.bm.as_ref().expect("bed mesh");
            for l in bm.bmc.generated_points_report([bm.probe.x_offset, bm.probe.y_offset]) {
                super::console(&l);
            }
        } else {
            b.points = b.orig_points.clone();
        }
        let bm = self.bm();
        bm.probe_points = bm.bmc.adjusted_points();
        if bm.probe_points.len() < 3 {
            super::console("Need at least %d probe points for bed_mesh");
        }
        Ok(())
    }

    // ---- ProbePointsHelper ----

    /// `ProbePointsHelper::start_probe` (the bed-mesh probe).
    fn start_probe(&mut self, g: &GCmd) -> CmdResult {
        super::console("start_probe");
        let method = g.get_string("METHOD", "automatic");
        self.bm().results.clear();
        let bm = self.bm();
        if method == "fast" && !bm.last_fast_z.is_empty() && !bm.fast_points.is_empty() {
            super::console("cmd_BED_MESH_CALIBRATE fast");
            bm.lift_speed = bm.helper_speed;
            bm.probe_offsets = [0.0; 3];
            if self.fast_probe_start(g)? {
                self.log("fast auto leveling succssful，Use the previous leveling result");
                return Ok(());
            }
            self.log("fast auto leveling failed, continue auto leveling step");
            self.single("BED_MESH_SET_INDEX INDEX=1");
        }
        if method == "manual" {
            return Err(Abort("BED_MESH_CALIBRATE METHOD=manual is not available in ccos".into()));
        }
        super::console("cmd_BED_MESH_CALIBRATE automatic");
        let bm = self.bm();
        bm.lift_speed = bm.probe.lift_speed;
        bm.probe_offsets = [bm.probe.x_offset, bm.probe.y_offset, bm.probe_z_offset];
        if bm.helper_move_z < bm.probe_offsets[2] {
            super::console("horizontal_move_z can't be less than probe's z_offset");
        }
        loop {
            if self.move_next(0)? {
                break;
            }
            match { let p = self.bm().probe.clone(); self.probe_run(g, &p) } {
                Ok(pos) => {
                    let bm = self.bm();
                    bm.results.push(pos);
                    let (n, total) = (bm.results.len(), bm.probe_points.len());
                    self.core().ui.lock().status = format!("Bed mesh: point {n} of {total} (z {:.3})", pos[2]);
                    self.calib(CalibEvent::LevelPoint);
                }
                Err(e) => {
                    self.calib(CalibEvent::LevelError);
                    // the stock code shuts the printer down on any probe error
                    if self.core().shutdown.is_none() {
                        self.invoke_shutdown(&format!("bed mesh probe error: {e}"));
                    }
                    super::console("bed mesh probe error");
                    return Err(e);
                }
            }
        }
        Ok(())
    }

    /// `_fast_probe_start`: true when the four corners match the last mesh.
    fn fast_probe_start(&mut self, g: &GCmd) -> Result<bool, Abort> {
        super::console("_fast_probe_start");
        let mut i = 0;
        loop {
            if self.move_next(2)? {
                super::console("_fast_probe_start move done.............");
                return Ok(true);
            }
            match { let p = self.bm().probe.clone(); self.probe_run(g, &p) } {
                Ok(pos) => {
                    self.calib(CalibEvent::LevelPoint);
                    let bm = self.bm();
                    bm.results.push(pos);
                    let last = bm.last_fast_z.get(i).copied().unwrap_or(0.0);
                    let delta = pos[2] - last;
                    if delta.abs() > bm.probe.fast_threshold {
                        bm.results.clear();
                        super::console(&format!(
                            "fast auto leveling now:{} last: {} delta:{} fast_autoleveling_threshold:{}",
                            cpp_double(pos[2]),
                            cpp_double(last),
                            cpp_double(delta),
                            cpp_double(bm.probe.fast_threshold)
                        ));
                        return Ok(false);
                    }
                }
                Err(e) => {
                    if self.core().shutdown.is_some() {
                        return Err(e);
                    }
                    super::console("bed mesh probe error");
                    self.calib(CalibEvent::LevelError);
                    return Ok(false);
                }
            }
            i += 1;
        }
    }

    /// `_move_next`: up to the travel height, finish when every point is
    /// done, else over the next point. true = done.
    fn move_next(&mut self, move_type: i32) -> Result<bool, Abort> {
        let (lift, z) = (self.bm().lift_speed, self.bm().helper_move_z);
        self.manual_move([None, None, Some(z), None], lift)?;
        let bm = self.bm();
        let n = if move_type == 2 { bm.fast_points.len() } else { bm.probe_points.len() };
        if bm.results.len() >= n {
            self.core().flush_moves();
            let (off, results) = (self.bm().probe_offsets, self.bm().results.clone());
            self.probe_finalize(off, results)?;
            return Ok(true);
        }
        let bm = self.bm();
        let p = if move_type == 2 { bm.fast_points[bm.results.len()] } else { bm.probe_points[bm.results.len()] };
        let (x, y) = (p[0] - bm.probe_offsets[0], p[1] - bm.probe_offsets[1]);
        let speed = bm.helper_speed;
        self.manual_move([Some(x), Some(y), Some(z), None], speed)?;
        Ok(false)
    }

    /// `BedMeshCalibrate::probe_finalize`: the probed points → mesh, in use
    /// and saved.
    fn probe_finalize(&mut self, offsets: [f64; 3], positions: Vec<[f64; 3]>) -> CmdResult {
        let bm = self.bm();
        let b = &mut bm.bmc;
        let fold = |f: fn(f64, f64) -> f64, k: usize, init: f64| positions.iter().map(|p| p[k]).fold(init, f);
        b.cfg.min_x = fold(f64::min, 0, f64::MAX) + offsets[0];
        b.cfg.max_x = fold(f64::max, 0, f64::MIN) + offsets[0];
        b.cfg.min_y = fold(f64::min, 1, f64::MAX) + offsets[1];
        b.cfg.max_y = fold(f64::max, 1, f64::MIN) + offsets[1];
        let (x_cnt, y_cnt) = (b.cfg.x_count, b.cfg.y_count);
        let mut positions = positions;
        if !b.substituted.is_empty() {
            let mut corrected: Vec<[f64; 3]> = Vec::new();
            let (mut idx_offset, mut start) = (0i64, 0usize);
            for (&i, pts) in &b.substituted {
                let fpt = [b.points[i][0] - offsets[0], b.points[i][1] - offsets[1]];
                let idx = (i as i64 + idx_offset).max(0) as usize;
                corrected.extend_from_slice(&positions[start.min(positions.len())..idx.min(positions.len())]);
                let zs: Vec<f64> = positions.iter().skip(idx).take(pts.len()).map(|p| p[2]).collect();
                let avg = zs.iter().sum::<f64>() / pts.len() as f64;
                idx_offset += pts.len() as i64 - 1;
                start = idx + pts.len();
                corrected.push([fpt[0], fpt[1], avg]);
            }
            corrected.extend_from_slice(&positions[start.min(positions.len())..]);
            if b.points.len() != corrected.len() {
                super::console(&format!("bed_mesh: invalid position list size, generated count: {} probed count: {}", b.points.len(), corrected.len()));
            }
            for (i, gen) in b.points.iter().enumerate() {
                let off = [gen[0] - offsets[0], gen[1] - offsets[1]];
                if let Some(p) = corrected.get(i) {
                    if !mesh::isclose(off[0], p[0], 0.1) || !mesh::isclose(off[1], p[1], 0.1) {
                        super::console("bed_mesh: point mismatch");
                    }
                }
            }
            positions = corrected;
        }
        let z_offset = match b.rri_index() {
            Some(r) if r < positions.len() => positions[r][2],
            _ => offsets[2],
        };
        let mut matrix: Vec<Vec<f64>> = Vec::new();
        let mut row: Vec<f64> = Vec::new();
        let mut prev = positions.first().copied().unwrap_or_default();
        for pos in &positions {
            if !mesh::isclose(pos[1], prev[1], 0.01) {
                matrix.push(std::mem::take(&mut row));
            }
            if pos[0] > prev[0] {
                row.push(pos[2] - z_offset);
            } else {
                row.insert(0, pos[2] - z_offset);
            }
            prev = *pos;
        }
        matrix.push(row);
        if matrix.len() as i64 != y_cnt {
            super::console(&format!("bed_mesh: Invalid y-axis table length\nProbed table length: {} Probed Table:\nprobed_matrix", matrix.len()));
        }
        for r in &matrix {
            if r.len() as i64 != x_cnt {
                super::console(&format!("bed_mesh: invalid x-axis table length\nProbed table length: {} Probed Table:\nprobed_matrix", matrix.len()));
                return Ok(());
            }
        }
        let mut m = ZMesh::new(self.bm().bmc.cfg.clone());
        m.build(matrix)?;
        self.set_mesh(Some(Rc::new(m)));
        super::console("Bed Mesh Leveling Complete");
        self.log("Bed mesh leveling complete");
        let bm = self.bm();
        let pending = std::mem::take(&mut bm.pending_profile);
        if !pending.is_empty() {
            self.save_profile(&pending, true);
        } else if bm.current_mesh_index != 0 {
            let i = bm.current_mesh_index.to_string();
            self.save_profile(&i, false);
        } else {
            self.save_profile(DEFAULT_PROFILE, false);
        }
        Ok(())
    }

    // ---- PrinterProbe (bed_mesh_probe) ----

    /// `PrinterProbe::run_probe`: tap until `samples` taps agree.
    pub(super) fn probe_run(&mut self, g: &GCmd, p: &ProbeCfg) -> Result<[f64; 3], Abort> {
        g.get_double("LIFT_SPEED", p.lift_speed, Lim::above(0.0))?;
        let mut sample_count = g.get_int("SAMPLES", p.samples, Some(1), None)?;
        g.get_double("SAMPLE_RETRACT_DIST", p.sample_retract_dist, Lim::above(0.0))?;
        let mut tolerance = g.get_double("SAMPLES_TOLERANCE", p.samples_tolerance, Lim::min(0.0))?;
        let mut step = p.samples_tolerance_step;
        let max_retries = g.get_int("SAMPLES_TOLERANCE_RETRIES", p.samples_retries, Some(0), None)?;
        let samples_result = g.get_string("SAMPLES_RESULT", &p.samples_result);
        let xy = self.core().commanded;
        let edge = xy[0] <= 20.0 || xy[0] >= 230.0 || xy[1] <= 20.0 || xy[1] >= 230.0;
        if edge {
            sample_count = p.edge_samples;
            tolerance = p.edge_samples_tolerance;
            step = p.edge_samples_tolerance_step;
        }
        let mut retries = 0;
        let mut positions: Vec<[f64; 3]> = Vec::new();
        while (positions.len() as i64) < sample_count {
            if positions.len() as i64 != sample_count - 1 {
                g.get_double("PROBE_SPEED", p.speed, Lim::above(0.0))?;
            }
            let pos = self.run_g29_z()?;
            positions.push([pos[0], pos[1], pos[2]]);
            let lo = positions.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
            let hi = positions.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
            if hi - lo > tolerance {
                if retries >= max_retries {
                    self.log(format!("PrinterProbe samples exceed samples_tolerance MAX:{hi:.6} MIN:{lo:.6} samples_tolerance:{tolerance:.6}"));
                    return Err(Abort("Probe samples exceed samples_tolerance".into()));
                }
                retries += 1;
                if retries >= 2 {
                    tolerance += step;
                }
                positions.clear();
            }
        }
        let n = positions.len() as f64;
        let mean = |ps: &[[f64; 3]]| {
            let k = ps.len() as f64;
            [ps.iter().map(|p| p[0]).sum::<f64>() / k, ps.iter().map(|p| p[1]).sum::<f64>() / k, ps.iter().map(|p| p[2]).sum::<f64>() / k]
        };
        let mut r = match samples_result.as_str() {
            "median" => {
                let mut s = positions.clone();
                s.sort_by(|a, b| a[2].total_cmp(&b[2]));
                let mid = s.len() / 2;
                if s.len() % 2 == 1 {
                    s[mid]
                } else {
                    mean(&s[mid - 1..=mid])
                }
            }
            "weighted" if positions.len() == 2 => {
                let m = mean(&positions);
                // float arithmetic in the stock code
                [m[0], m[1], (positions[1][2] * 3.0 + positions[0][2] * 2.0) * 0.2f32 as f64]
            }
            _ => mean(&positions),
        };
        let _ = n;
        if edge {
            r[2] += p.edge_compensation;
        }
        Ok(r)
    }

    /// `StrainGaugeWrapper::run_G29_Z` (`probing_move`): one tap down from
    /// here with the load cells; Z comes back up. [x, y, z at the press,
    /// where Z stopped].
    pub(super) fn run_g29_z(&mut self) -> Result<[f64; 4], Abort> {
        let Some(cfg) = self.core().m.strain_gauge else {
            return Err(Abort("bed mesh probing needs the load cells ([strain_gauge])".into()));
        };
        if !cfg.use_probe_by_step {
            return Err(Abort("bed mesh probing with [strain_gauge] use_probe_by_step: 0 is not available in ccos".into()));
        }
        super::console("-----------------run G29 Z use probe_by_step-----------------");
        self.wait_moves()?;
        let now = self.core().commanded;
        if self.sg.g29_cnt == 0 || self.sg.g29_cnt % 12 == 0 {
            self.shake_motor(20, &cfg)?;
        }
        self.calibration_start(30)?;
        self.sg.g29_cnt += 1;
        let max_z_detection = now[2] - cfg.g29_max_detection;
        super::console(&format!("max_z_detection:{}", cpp_double(max_z_detection)));
        let (_, z, ok) = self.probe_by_step(now, cfg.g29_speed, max_z_detection, true)?;
        self.wait_moves()?;
        let mut pos = self.core().commanded;
        if !ok {
            pos[2] -= 10.0;
            self.core().add_move(pos, 360.0, true)?;
            return Err(Abort("G29 probe_by_step failed".into()));
        }
        pos[2] = z;
        pos[3] = self.sg.end_z_mm;
        super::console(&format!("***************now_pos[2] : {}*****************", cpp_double(pos[2])));
        Ok(pos)
    }

    // ---- auto_leveling ----

    /// `G29 [S<bed °C>]`: the screen's auto-levelling.
    fn cmd_g29(&mut self, g: &GCmd) -> CmdResult {
        // traditional parse: PROFILE/SAVE/PROBE_COUNT only exist when called
        // from code
        let mut params = std::collections::BTreeMap::new();
        for k in ["PROFILE", "PROBE_COUNT"] {
            let v = if k == "PROFILE" {
                let p = g.get_string("PROFILE", "");
                if p.is_empty() {
                    g.get_string("SAVE", "")
                } else {
                    p
                }
            } else {
                g.get_string(k, "")
            };
            if !v.is_empty() {
                params.insert(k.to_string(), v);
            }
        }
        let bed = if g.has("S") { g.f("S", 60.0)? } else { 60.0 };
        let enable_z_home = self.core().m.strain_gauge.is_some_and(|s| s.enable_z_home);
        let test_mode = self.bm.as_ref().is_some_and(|b| b.auto_leveling_test_mode);
        self.calib(CalibEvent::LevelStart);
        if !test_mode {
            if enable_z_home {
                self.single("G29.1 P0");
            }
            self.single("M211 S0");
            self.calib(CalibEvent::LevelPreheat);
            self.single("M104 S140");
            self.single(&format!("M140 S{bed:.2}"));
            self.single("M106 P1 S0");
            self.single("M106 P2 S0");
            self.single("M106 P3 S0");
            self.cmd_wipe_nozzle()?;
            self.single(&format!("M140 S{bed:.2}"));
            self.single("RESET_PRINTER_PARAM");
            self.single("M109 S140");
        }
        self.single("SET_GCODE_OFFSET Z=0 MOVE=0 MOVE_SPEED=5.0");
        {
            let mut c = self.core();
            c.m.axes[2].position_endstop = c.m.axes[2].position_endstop_extra;
        }
        self.single("M8233 S0.000 P1");
        self.core().cfgs.save_printer();
        self.single("G28 Z");
        self.single("M109 S140");
        self.single(&format!("M190 S{bed:.2}"));
        self.calib(CalibEvent::LevelProbe);
        if self.bm.is_none() {
            return Err(Abort("G29 needs [bed_mesh]".into()));
        }
        let fast = self.bm().enable_fast_probe;
        if fast {
            params.insert("METHOD".into(), "fast".into());
        }
        let line = std::iter::once("BED_MESH_CALIBRATE".to_string()).chain(params.iter().map(|(k, v)| format!("{k}={v}"))).collect::<Vec<_>>().join(" ");
        let cal = GCmd { command: "BED_MESH_CALIBRATE".into(), commandline: line, params };
        if let Err(e) = self.cmd_bed_mesh_calibrate(&cal, None) {
            self.log("BED_MESH_CALIBRATE error");
            return Err(e);
        }
        self.calib(CalibEvent::LevelFinish);
        self.single("G90");
        self.single("G1 X10 Y10 F4500");
        self.single("M106 S0");
        self.single("M104 S0");
        self.single("M140 S0");
        if enable_z_home {
            self.single("G29.1 P1");
        }
        self.single("M211 S1");
        Ok(())
    }

    /// `G29.1 P1` re-applies the configured Z endstop, the probes' Z offset
    /// and the current profile; `P0` zeroes them and turns the mesh off.
    pub(super) fn cmd_g29_1(&mut self, g: &GCmd) -> CmdResult {
        if self.bm.is_none() {
            return Ok(());
        }
        if g.i("P", 1)? == 1 {
            super::console("----------------------------------------G29 P1--------------------------------------------------------");
            let zo = {
                let mut c = self.core();
                let e = c.cfgs.printer.get_double("stepper_z", "position_endstop", 4.5);
                c.m.axes[2].position_endstop = e;
                c.cfgs.printer.get_double("probe", "z_offset", 1.0)
            };
            self.bm().probe_z_offset = zo;
            let name = self.bm().current_profile.clone();
            self.load_profile(&name, false)?;
        } else {
            super::console("----------------------------------------G29 P0--------------------------------------------------------");
            self.base[2] = 0.0;
            self.homing_position[2] = 0.0;
            self.core().m.axes[2].position_endstop = 0.0;
            self.bm().probe_z_offset = 0.0;
            self.set_mesh(None);
        }
        Ok(())
    }

    /// The mesh in use, for `WIPE_NOZZLE` (which turns it off meanwhile).
    /// `m_bmc->m_mesh_min` / `m_mesh_max`; `orig`: as configured (what
    /// `StrainGaugeWrapper::_handle_mcu_identify` saw).
    pub(super) fn mesh_area(&self, orig: bool) -> Option<([f64; 2], [f64; 2])> {
        let b = &self.bm.as_ref()?.bmc;
        let (lo, hi) = if orig { (&b.orig.mesh_min, &b.orig.mesh_max) } else { (&b.mesh_min, &b.mesh_max) };
        Some(([*lo.first()?, *lo.get(1)?], [*hi.first()?, *hi.get(1)?]))
    }

    /// `m_z_mesh->calc_z` with the mesh in use.
    pub(super) fn mesh_calc_z(&self, x: f64, y: f64) -> Option<f64> {
        self.bm.as_ref()?.z_mesh.as_ref().map(|m| m.calc_z(x, y))
    }

    pub(super) fn take_mesh(&mut self) -> Option<Rc<ZMesh>> {
        let m = self.bm.as_ref().and_then(|b| b.z_mesh.clone());
        self.set_mesh(None);
        m
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gcode::parse::parse;

    #[test]
    fn calibrate_words() {
        let g = parse("BED_MESH_CALIBRATE S75 pla_0.2").unwrap();
        assert_eq!(calibrate_extras(&g).unwrap(), (Some(75.0), Some("pla_0.2".into())));
        let g = parse("BED_MESH_CALIBRATE petg").unwrap();
        assert_eq!(calibrate_extras(&g).unwrap(), (None, Some("petg".into())));
        let g = parse("BED_MESH_CALIBRATE s60.5 PROBE_COUNT=3,3").unwrap();
        assert_eq!(calibrate_extras(&g).unwrap(), (Some(60.5), None));
        let g = parse("BED_MESH_CALIBRATE S=80").unwrap();
        assert_eq!(calibrate_extras(&g).unwrap(), (Some(80.0), None));
        assert!(calibrate_extras(&parse("BED_MESH_CALIBRATE a b").unwrap()).is_err());
        assert!(calibrate_extras(&parse("BED_MESH_CALIBRATE S60 S70").unwrap()).is_err());
        assert!(calibrate_extras(&parse("BED_MESH_CALIBRATE bad[name]").unwrap()).is_err());
        // a bare S is a name, S followed by a non-number too
        assert_eq!(calibrate_extras(&parse("BED_MESH_CALIBRATE Sx").unwrap()).unwrap(), (None, Some("Sx".into())));
        // words inside a quoted value are not names
        assert_eq!(calibrate_extras(&parse("BED_MESH_CALIBRATE S70 PROFILE=\"a b c\"").unwrap()).unwrap(), (Some(70.0), None));
        assert_eq!(calibrate_extras(&parse("BED_MESH_CALIBRATE pla ; comment words").unwrap()).unwrap(), (None, Some("pla".into())));
    }
}
