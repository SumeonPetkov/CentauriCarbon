//! The G-code thread: connects the MCUs, runs commands from the screen and
//! prints files. Moves are handed to the core; waits (M109, M400, homing)
//! block this thread only.
//!
//! Supported: G0 G1 G2 G3 (I/J arcs) G4 G28 G90 G91 G92, M17 M18 M82 M83
//! M84 M104 M105 M106 M107 M109 M112 M114 M117 M140 M190 M220 M221 M400.
//! Anything else is reported once and skipped (slicer extras like M73,
//! EXCLUDE_OBJECT, SET_* …).

use std::collections::HashSet;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::Arc;
use std::time::{Duration, Instant};

use cc_proto::dict::Arg;

use crate::app::{LinkState, LinkStatus, Phase};
use crate::clock::now;
use crate::core::{Output, Shared};
use crate::machine::{self, AXES};
use crate::mcu::Result;
use crate::Target;

/// Work for the G-code thread.
#[derive(Debug, Clone)]
pub enum Job {
    /// One or more lines (`\n`-separated).
    Gcode(String),
    Print(PathBuf),
    /// Re-attach every MCU (after a shutdown).
    Restart,
    Reset(Target),
    Probe,
    RefreshFiles,
}

/// Flags the screen sets while a print runs.
#[derive(Default)]
pub struct Control {
    pub pause: AtomicBool,
    pub cancel: AtomicBool,
}

/// G-code modal state (Klipper's `gcode_move`).
struct Exec {
    p: Shared,
    ctl: Arc<Control>,
    absolute: bool,
    absolute_e: bool,
    /// Toolhead-space target of the last move.
    last: [f64; 4],
    base: [f64; 4],
    speed: f64,
    speed_factor: f64,
    extrude_factor: f64,
    unknown: HashSet<String>,
}

fn param(words: &[(char, f64)], c: char) -> Option<f64> {
    words.iter().find(|(k, _)| *k == c).map(|(_, v)| *v)
}

impl Exec {
    fn new(p: Shared, ctl: Arc<Control>) -> Self {
        Self {
            p,
            ctl,
            absolute: true,
            absolute_e: true,
            last: [0.0; 4],
            base: [0.0; 4],
            speed: 25.0,
            speed_factor: 1.0,
            extrude_factor: 1.0,
            unknown: HashSet::new(),
        }
    }

    fn sync_position(&mut self) {
        self.last = self.p.lock().commanded;
    }

    /// Wait like Printer::wait, but also stop on cancel.
    fn wait<R>(&self, timeout: f64, what: &str, mut f: impl FnMut(&mut crate::core::Core) -> Option<R>) -> Result<R> {
        let ctl = self.ctl.clone();
        let mut cancelled = false;
        let r = self.p.wait(timeout, what, |c| {
            if ctl.cancel.load(Ordering::Relaxed) {
                cancelled = true;
                return Some(None);
            }
            f(c).map(Some)
        })?;
        match r {
            Some(r) => Ok(r),
            None if cancelled => Err("cancelled".into()),
            None => unreachable!(),
        }
    }

    /// Queue a move, waiting while the planned buffer is full.
    fn move_to(&mut self, target: [f64; 4], speed: f64) -> Result<()> {
        self.wait(f64::INFINITY, "buffer space", |c| {
            (c.buffered_time() < machine::BUFFER_TIME_HIGH && c.mcus.iter().flatten().all(|m| m.backlog() < 600)).then_some(())
        })?;
        self.p.lock().add_move(target, speed, None, true)
    }

    /// M400: everything planned has executed.
    fn wait_moves(&mut self) -> Result<()> {
        self.p.lock().flush_moves();
        self.wait(f64::INFINITY, "moves to finish", |c| c.idle().then_some(()))
    }

    pub fn run_line(&mut self, raw: &str) -> Result<()> {
        let line = raw.split(';').next().unwrap_or("").trim();
        if line.is_empty() {
            return Ok(());
        }
        let line = match line.split_once('*') {
            Some((l, _)) => l.trim(), // checksum
            None => line,
        };
        let mut parts = line.split_whitespace();
        let mut cmd = parts.next().unwrap_or("").to_ascii_uppercase();
        if cmd.starts_with('N') && cmd[1..].chars().all(|c| c.is_ascii_digit()) {
            cmd = parts.next().unwrap_or("").to_ascii_uppercase(); // line number
        }
        if cmd == "M117" {
            let msg = line.splitn(2, char::is_whitespace).nth(1).unwrap_or("").trim().to_string();
            self.p.lock().ui.set_status(msg);
            return Ok(());
        }
        let (mut wbuf, mut nw) = ([(' ', 0.0f64); 16], 0usize);
        let (mut fbuf, mut nf) = ([' '; 8], 0usize);
        for w in parts {
            let mut ch = w.chars();
            let Some(k) = ch.next() else { continue };
            let k = k.to_ascii_uppercase();
            let v = ch.as_str();
            if v.is_empty() {
                if nf < fbuf.len() {
                    fbuf[nf] = k;
                    nf += 1;
                }
            } else if let Ok(v) = v.parse::<f64>() {
                if nw < wbuf.len() {
                    wbuf[nw] = (k, v);
                    nw += 1;
                }
            }
        }
        let (words, flags) = (&wbuf[..nw], &fbuf[..nf]);
        let has = |c: char| words.iter().any(|(k, _)| *k == c) || flags.contains(&c);
        match cmd.as_str() {
            "G0" | "G1" => self.g1(words),
            "G2" | "G3" => self.arc(words, cmd == "G2"),
            "G4" => {
                let secs = param(words, 'P').map(|p| p / 1000.0).or(param(words, 'S')).unwrap_or(0.0);
                self.wait_moves()?;
                let until = now() + secs;
                self.wait(secs + 1.0, "dwell", |_| (now() >= until).then_some(()))
            }
            "G28" => {
                let mut axes: Vec<usize> = (0..3).filter(|&i| has(['X', 'Y', 'Z'][i])).collect();
                if axes.is_empty() {
                    axes = vec![0, 1, 2];
                }
                self.home(&axes)
            }
            "G90" => {
                self.absolute = true;
                self.absolute_e = true;
                Ok(())
            }
            "G91" => {
                self.absolute = false;
                self.absolute_e = false;
                Ok(())
            }
            "M82" => {
                self.absolute_e = true;
                Ok(())
            }
            "M83" => {
                self.absolute_e = false;
                Ok(())
            }
            "G92" => {
                self.sync_position();
                if words.is_empty() && flags.is_empty() {
                    for i in 0..4 {
                        self.base[i] = self.last[i];
                    }
                }
                for (i, c) in ['X', 'Y', 'Z', 'E'].iter().enumerate() {
                    if let Some(v) = param(words, *c) {
                        let v = if i == 3 { v * self.extrude_factor } else { v };
                        self.base[i] = self.last[i] - v;
                    }
                }
                Ok(())
            }
            "M17" => self.p.lock().set_motors(true),
            "M18" | "M84" => {
                self.wait_moves()?;
                self.p.lock().set_motors(false)
            }
            "M104" | "M109" | "M140" | "M190" => {
                let h = if cmd == "M104" || cmd == "M109" { 0 } else { 1 };
                let t = param(words, 'S').or(param(words, 'R')).unwrap_or(0.0);
                self.p.lock().set_heater(h, t)?;
                if cmd == "M109" || cmd == "M190" {
                    let name = if h == 0 { "nozzle" } else { "bed" };
                    self.p.lock().ui.set_status(format!("Heating {name} to {t:.0}°…"));
                    self.wait(f64::INFINITY, "temperature", |c| (c.heaters[h].pending.is_none() && c.heaters[h].at_target()).then_some(()))?;
                    let name = if h == 0 { "Nozzle" } else { "Bed" };
                    self.p.lock().ui.set_status(format!("{name} at {t:.0}°"));
                }
                Ok(())
            }
            "M105" => {
                let c = self.p.lock();
                let f = |t: Option<f64>| t.map_or("--".to_string(), |t| format!("{t:.1}"));
                let s = format!("T:{} /{:.0} B:{} /{:.0}", f(c.heaters[0].temp), c.heaters[0].target, f(c.heaters[1].temp), c.heaters[1].target);
                c.log(s);
                Ok(())
            }
            "M106" => {
                let s = param(words, 'S').unwrap_or(255.0) / 255.0;
                self.wait_moves_soft();
                self.p.lock().set_output(Output::PartFan, s)
            }
            "M107" => {
                self.wait_moves_soft();
                self.p.lock().set_output(Output::PartFan, 0.0)
            }
            "M112" => {
                self.p.lock().emergency("M112 emergency stop".into());
                Ok(())
            }
            "M114" => {
                let c = self.p.lock();
                let p = c.live_position();
                c.log(format!("X:{:.3} Y:{:.3} Z:{:.3} E:{:.3}", p[0], p[1], p[2], p[3]));
                Ok(())
            }
            "M220" => {
                if let Some(s) = param(words, 'S') {
                    self.speed = self.speed / self.speed_factor * (s / 100.0);
                    self.speed_factor = s / 100.0;
                }
                Ok(())
            }
            "M221" => {
                if let Some(s) = param(words, 'S') {
                    self.sync_position();
                    let f = s / 100.0;
                    let e_value = (self.last[3] - self.base[3]) / self.extrude_factor;
                    self.base[3] = self.last[3] - e_value * f;
                    self.extrude_factor = f;
                }
                Ok(())
            }
            "M400" => self.wait_moves(),
            _ => {
                if self.unknown.insert(cmd.clone()) {
                    self.p.lock().log(format!("ignored unsupported command {cmd}"));
                }
                Ok(())
            }
        }
    }

    /// Fan changes take effect "now"; in a print they should follow the
    /// moves before them. Commit the look-ahead so they are close.
    fn wait_moves_soft(&self) {
        self.p.lock().flush_moves();
    }

    fn g1(&mut self, w: &[(char, f64)]) -> Result<()> {
        self.sync_position();
        let mut t = self.last;
        for (i, c) in ['X', 'Y', 'Z'].iter().enumerate() {
            if let Some(v) = param(w, *c) {
                t[i] = if self.absolute { v + self.base[i] } else { t[i] + v };
            }
        }
        if let Some(v) = param(w, 'E') {
            let v = v * self.extrude_factor;
            t[3] = if self.absolute && self.absolute_e { v + self.base[3] } else { t[3] + v };
        }
        if let Some(f) = param(w, 'F') {
            if f <= 0.0 {
                return Err(format!("Invalid speed F{f}").into());
            }
            self.speed = f / 60.0 * self.speed_factor;
        }
        self.move_to(t, self.speed)?;
        self.last = t;
        Ok(())
    }

    /// G2/G3 with I/J centre offsets, split into ≤ 0.5 mm chords.
    fn arc(&mut self, w: &[(char, f64)], clockwise: bool) -> Result<()> {
        self.sync_position();
        let start = self.last;
        let mut end = start;
        for (i, c) in ['X', 'Y', 'Z'].iter().enumerate() {
            if let Some(v) = param(w, *c) {
                end[i] = if self.absolute { v + self.base[i] } else { start[i] + v };
            }
        }
        if let Some(v) = param(w, 'E') {
            let v = v * self.extrude_factor;
            end[3] = if self.absolute && self.absolute_e { v + self.base[3] } else { start[3] + v };
        }
        if let Some(f) = param(w, 'F') {
            self.speed = f / 60.0 * self.speed_factor;
        }
        let (i, j) = (param(w, 'I').unwrap_or(0.0), param(w, 'J').unwrap_or(0.0));
        if i == 0.0 && j == 0.0 {
            return Err("G2/G3 needs I or J (R is not supported)".into());
        }
        let (cx, cy) = (start[0] + i, start[1] + j);
        let r = (i * i + j * j).sqrt();
        let a0 = (start[1] - cy).atan2(start[0] - cx);
        let a1 = (end[1] - cy).atan2(end[0] - cx);
        let mut sweep = a1 - a0;
        if clockwise {
            if sweep >= 0.0 {
                sweep -= std::f64::consts::TAU;
            }
        } else if sweep <= 0.0 {
            sweep += std::f64::consts::TAU;
        }
        let n = ((sweep.abs() * r) / 0.5).ceil().max(1.0) as usize;
        for k in 1..=n {
            let f = k as f64 / n as f64;
            let a = a0 + sweep * f;
            let mut p = [cx + r * a.cos(), cy + r * a.sin(), start[2] + (end[2] - start[2]) * f, start[3] + (end[3] - start[3]) * f];
            if k == n {
                p = end;
            }
            self.move_to(p, self.speed)?;
        }
        self.last = end;
        Ok(())
    }

    // ---- homing ----

    fn home(&mut self, axes: &[usize]) -> Result<()> {
        self.p.lock().motion_lock_check()?;
        for &a in axes.iter().filter(|&&a| a < 2) {
            self.home_axis(a, AXES[a].homing_speed, 1.5 * (AXES[a].max - AXES[a].min))?;
        }
        if axes.contains(&2) {
            {
                let c = self.p.lock();
                if !(c.homed[0] && c.homed[1]) {
                    drop(c);
                    return Err("Home X and Y before Z (Z homes at the bed centre)".into());
                }
            }
            self.sync_position();
            let mut t = self.last;
            // Lift first if Z is known, then go to the bed centre.
            if self.p.lock().homed[2] && t[2] < 5.0 {
                t[2] = 5.0;
                self.move_to(t, 10.0)?;
            }
            t[0] = machine::Z_HOME_XY[0];
            t[1] = machine::Z_HOME_XY[1];
            self.move_to(t, 200.0)?;
            let ax = AXES[2];
            self.home_axis(2, ax.homing_speed, 1.5 * (ax.max - ax.min))?;
            // Retract, then a slow second touch for accuracy.
            self.sync_position();
            let mut up = self.last;
            up[2] = ax.position_endstop + ax.retract;
            self.move_to(up, 6.0)?;
            self.home_axis(2, ax.second_speed, 2.0 * ax.retract)?;
            self.sync_position();
            let mut up = self.last;
            up[2] = ax.position_endstop + 5.0;
            self.move_to(up, 10.0)?;
        }
        self.wait_moves()?;
        self.sync_position();
        self.p.lock().ui.set_status("Homed");
        Ok(())
    }

    /// One homing move towards the endstop, stopped by the MCU (trsync).
    fn home_axis(&mut self, a: usize, speed: f64, travel: f64) -> Result<()> {
        let ax = AXES[a];
        self.wait_moves()?;
        {
            let mut c = self.p.lock();
            c.ui.set_status(format!("Homing {}…", ax.name));
            c.responses.clear();
            let mut start = c.commanded;
            start[a] = ax.position_endstop + travel;
            c.set_position(start);
            c.homing = true;
            let t = now();
            let m = c.mcu(Target::Dsp)?;
            let clock = |s: f64| m.clock.clock(s) as u32 as i64;
            let (c0, sample, rest) = (clock(t + 0.05), m.clock.ticks(15e-6), m.clock.ticks(1e-4));
            let ts = crate::machine::dsp::TRSYNC as i64;
            let msgs = [
                m.encode("trsync_start", &[("oid", Arg::Int(ts)), ("report_clock", Arg::Int(0)), ("report_ticks", Arg::Int(0)), ("expire_reason", Arg::Int(4))]),
            ];
            for msg in msgs {
                m.queue_msg(msg);
            }
            for &s in ax.steppers {
                let oid = machine::STEPPERS[s].oid as i64;
                let msg = m.encode("stepper_stop_on_trigger", &[("oid", Arg::Int(oid)), ("trsync_oid", Arg::Int(ts))]);
                m.queue_msg(msg);
            }
            let msg = m.encode(
                "endstop_home",
                &[
                    ("oid", Arg::Int(ax.endstop_oid as i64)),
                    ("clock", Arg::Int(c0)),
                    ("sample_ticks", Arg::Int(sample as i64)),
                    ("sample_count", Arg::Int(4)),
                    ("rest_ticks", Arg::Int(rest as i64)),
                    ("pin_value", Arg::Int(ax.triggered_level as i64)),
                    ("trsync_oid", Arg::Int(ts)),
                    ("trigger_reason", Arg::Int(1)),
                ],
            );
            m.queue_msg(msg);
            let mut end = start;
            end[a] = ax.position_endstop;
            if let Err(e) = c.add_move(end, speed, Some(ax.homing_accel), false) {
                c.homing = false;
                return Err(e);
            }
            c.flush_moves();
            let end_t = c.planner.print_time + 0.1;
            let m = c.mcu(Target::Dsp)?;
            let msg = m.encode("trsync_set_timeout", &[("oid", Arg::Int(ts)), ("clock", Arg::Int(m.clock.clock(end_t) as u32 as i64))]);
            m.queue_msg(msg);
            c.wake();
        }
        let travel_t = travel / speed + 2.0;
        let reason = self.wait(travel_t + 2.0, "homing trigger", |c| {
            let i = c.responses.iter().position(|(t, m)| *t == Target::Dsp && m.name() == "trsync_state" && m.int("can_trigger") == Some(0))?;
            let (_, m) = c.responses.remove(i).unwrap();
            Some(m.int("trigger_reason").unwrap_or(0))
        });
        // Stop: forget the rest of the move, stop sampling, read positions.
        {
            let mut c = self.p.lock();
            c.homing = false;
            c.stop_motion();
            let m = c.mcu(Target::Dsp)?;
            for &s in ax.steppers {
                m.withdraw(machine::STEPPERS[s].oid);
            }
            let msg = m.encode(
                "endstop_home",
                &[
                    ("oid", Arg::Int(ax.endstop_oid as i64)),
                    ("clock", Arg::Int(0)),
                    ("sample_ticks", Arg::Int(0)),
                    ("sample_count", Arg::Int(0)),
                    ("rest_ticks", Arg::Int(0)),
                    ("pin_value", Arg::Int(0)),
                    ("trsync_oid", Arg::Int(0)),
                    ("trigger_reason", Arg::Int(0)),
                ],
            );
            m.queue_msg(msg);
            for &s in ax.steppers {
                let msg = m.encode("stepper_get_position", &[("oid", Arg::Int(machine::STEPPERS[s].oid as i64))]);
                m.queue_msg(msg);
            }
            c.wake();
        }
        let positions: Vec<(usize, i64)> = ax
            .steppers
            .iter()
            .map(|&s| {
                let oid = machine::STEPPERS[s].oid as i64;
                self.p
                    .wait(2.0, "stepper position", |c| {
                        let i = c.responses.iter().position(|(_, m)| m.name() == "stepper_position" && m.int("oid") == Some(oid))?;
                        let (_, m) = c.responses.remove(i).unwrap();
                        m.int("pos")
                    })
                    .map(|p| (s, p))
            })
            .collect::<Result<_>>()?;
        let reason = reason?;
        let mut c = self.p.lock();
        for (s, pos) in positions {
            c.steppers[s].set_mcu_position(pos);
        }
        if reason != 1 {
            c.homed[a] = false;
            let p = c.stepper_position();
            c.set_position(p);
            return Err(format!("No trigger on {} after full movement", ax.name.to_ascii_lowercase()).into());
        }
        let mut p = c.stepper_position();
        p[a] = ax.position_endstop;
        p[3] = c.commanded[3];
        c.set_position(p);
        c.homed[a] = true;
        drop(c);
        self.sync_position();
        Ok(())
    }
}

/// List printable files, newest first.
pub fn list_files(dir: &Path) -> Vec<String> {
    let mut v: Vec<(std::time::SystemTime, String)> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().to_string();
            let lower = name.to_ascii_lowercase();
            (lower.ends_with(".gcode") || lower.ends_with(".gco") || lower.ends_with(".g")).then(|| (e.metadata().and_then(|m| m.modified()).unwrap_or(std::time::UNIX_EPOCH), name))
        })
        .collect();
    v.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
    v.into_iter().map(|(_, n)| n).collect()
}

/// The G-code thread body.
#[cfg(target_os = "linux")]
pub fn run(p: Shared, ctl: Arc<Control>, jobs: Receiver<Job>) {
    let mut ex = Exec::new(p.clone(), ctl.clone());
    refresh_files(&p);
    connect_all(&p);
    for job in jobs {
        match job {
            Job::Gcode(text) => {
                ctl.cancel.store(false, Ordering::Relaxed);
                for line in text.lines() {
                    if let Err(e) = ex.run_line(line) {
                        p.lock().ui.set_status(format!("{}: {e}", line.trim()));
                        break;
                    }
                }
                p.lock().flush_moves();
            }
            Job::Print(path) => print_file(&mut ex, &p, &ctl, &path),
            Job::Restart => {
                ctl.cancel.store(false, Ordering::Relaxed);
                {
                    let mut c = p.lock();
                    c.shutdown = None;
                    for t in Target::ALL {
                        c.detach(t, "restarting");
                    }
                }
                connect_all(&p);
            }
            Job::Reset(t) => {
                p.lock().detach(t, "resetting");
                connect(&p, t);
            }
            Job::Probe => {
                let c = p.lock();
                for t in Target::ALL {
                    let msg = match &c.mcus[t as usize] {
                        Some(m) => format!("{}: {:.3} MHz, {} resends, {} KiB sent", t.title(), m.clock.freq / 1e6, m.retransmits, m.bytes_sent / 1024),
                        None => format!("{}: not connected ({})", t.title(), c.links[t as usize].detail),
                    };
                    c.log(msg);
                }
            }
            Job::RefreshFiles => refresh_files(&p),
        }
        let c = p.lock();
        if c.exit {
            return;
        }
    }
}

pub fn refresh_files(p: &Shared) {
    let c = p.lock();
    let files = list_files(&c.hw.gcode_dir);
    let mut s = c.ui.lock();
    s.files = files;
    s.gcode_dir = c.hw.gcode_dir.display().to_string();
}

#[cfg(target_os = "linux")]
fn connect(p: &Shared, t: Target) {
    let hw = p.lock().hw.clone();
    p.lock().links[t as usize] = LinkStatus { state: LinkState::Unknown, detail: "connecting".into() };
    match crate::core::attach(&hw, t) {
        Ok((m, detail)) => {
            let mut c = p.lock();
            c.log(format!("{}: connected ({detail})", t.title()));
            c.install(t, m, detail);
        }
        Err(e) => {
            let mut c = p.lock();
            c.links[t as usize] = LinkStatus { state: LinkState::Failed, detail: e.to_string() };
            c.log(format!("{}: {e}", t.title()));
        }
    }
}

#[cfg(target_os = "linux")]
fn connect_all(p: &Shared) {
    if crate::sys::stock_app_running() && !p.lock().hw.simulated {
        p.lock().ui.set_status("Stock app is running — stop it to use the MCUs");
        return;
    }
    p.lock().ui.set_status("Connecting to MCUs…");
    for t in Target::ALL {
        connect(p, t);
    }
    let c = p.lock();
    let ok = c.mcus.iter().filter(|m| m.is_some()).count();
    let lock = if c.hw.motion_lock().is_some() { " · motion locked" } else { "" };
    c.ui.set_status(format!("{ok} of 3 MCUs connected{lock}"));
}

#[cfg(target_os = "linux")]
fn print_file(ex: &mut Exec, p: &Shared, ctl: &Control, path: &Path) {
    let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
    let started = Instant::now();
    let set = |phase: Phase, progress: f32, err: &str| {
        let c = p.lock();
        let mut s = c.ui.lock();
        s.print.phase = phase;
        s.print.file = name.clone();
        s.print.progress = progress;
        s.print.elapsed = started.elapsed().as_secs_f64();
        s.print.error = err.to_string();
        if phase == Phase::Printing && s.print.started.is_none() {
            s.print.started = Some(started);
        }
    };
    ctl.cancel.store(false, Ordering::Relaxed);
    ctl.pause.store(false, Ordering::Relaxed);
    let file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(e) => {
            set(Phase::Failed, 0.0, &format!("{name}: {e}"));
            return;
        }
    };
    let total = file.metadata().map(|m| m.len()).unwrap_or(1).max(1);
    let mut reader = BufReader::with_capacity(64 * 1024, file);
    set(Phase::Printing, 0.0, "");
    p.lock().ui.set_status(format!("Printing {name}"));
    *ex = Exec::new(p.clone(), ex.ctl.clone());
    let mut done = 0u64;
    let mut line = String::new();
    let mut lines = 0u64;
    let result: Result<()> = (|| {
        loop {
            line.clear();
            let n = reader.read_line(&mut line)?;
            if n == 0 {
                return Ok(());
            }
            done += n as u64;
            lines += 1;
            if ctl.cancel.load(Ordering::Relaxed) {
                return Err("cancelled".into());
            }
            if ctl.pause.load(Ordering::Relaxed) {
                ex.wait_moves()?;
                set(Phase::Paused, done as f32 / total as f32, "");
                p.lock().ui.set_status("Paused");
                while ctl.pause.load(Ordering::Relaxed) {
                    if ctl.cancel.load(Ordering::Relaxed) {
                        return Err("cancelled".into());
                    }
                    std::thread::sleep(Duration::from_millis(50));
                }
                set(Phase::Printing, done as f32 / total as f32, "");
                p.lock().ui.set_status(format!("Printing {name}"));
            }
            ex.run_line(&line).map_err(|e| format!("line {lines}: {}: {e}", line.trim()))?;
            if lines % 16 == 0 {
                set(Phase::Printing, done as f32 / total as f32, "");
            }
        }
    })();
    let result = result.and_then(|_| ex.wait_moves());
    match result {
        Ok(()) => {
            set(Phase::Done, 1.0, "");
            p.lock().ui.set_status(format!("Finished {name} in {:.0} s", started.elapsed().as_secs_f64()));
        }
        Err(e) if e.to_string() == "cancelled" => {
            let mut c = p.lock();
            c.stop_motion();
            let _ = c.set_heater(0, 0.0);
            let _ = c.set_heater(1, 0.0);
            let _ = c.set_output(Output::PartFan, 0.0);
            drop(c);
            set(Phase::Cancelled, done as f32 / total as f32, "");
            p.lock().ui.set_status(format!("Cancelled {name}"));
        }
        Err(e) => {
            let mut c = p.lock();
            let _ = c.set_heater(0, 0.0);
            let _ = c.set_heater(1, 0.0);
            c.flush_moves();
            drop(c);
            set(Phase::Failed, done as f32 / total as f32, &e.to_string());
            p.lock().ui.set_status(format!("Print failed: {e}"));
        }
    }
}

impl crate::core::Core {
    pub fn motion_lock_check(&self) -> Result<()> {
        if let Some(l) = self.hw.motion_lock() {
            return Err(l.into());
        }
        if let Some(s) = &self.shutdown {
            return Err(format!("Printer is shut down: {s}").into());
        }
        Ok(())
    }
}
