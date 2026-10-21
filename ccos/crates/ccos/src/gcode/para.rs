//! `ROOT`/`UNROOT` and the M8000–M8826 configuration commands
//! (firmware/core/klippy/printer_para.cpp).
//!
//! Most of them set one `printer.cfg` key in memory (optionally applying
//! it at once); they are generated into `para_table.rs` by
//! `ccos/tools/m8xxx.py`. The rest are below. All are ignored when they
//! come from a print file unless `ROOT` ran earlier in that file — except
//! M8803/M8807 (never gated) and M8016 (gate inverted: only from a file).
//! Like the stock firmware, edits reach the disk only through M8800,
//! M8801, M8233, M8823, M8824 or a later save.

use super::parse::{cpp_double, CmdResult, GCmd};
use super::Exec;
use crate::config::to_string;

pub enum Kind {
    /// `S1` = default pin, `S-1` = inverted default pin.
    Pin(&'static str),
    /// `S` (double), default from the given config key, else the value.
    Double(f64, &'static str, &'static str),
    Int(i64, &'static str, &'static str),
    /// Read as double, stored as int.
    DoubleAsInt(f64, &'static str, &'static str),
    Str(&'static str, &'static str, &'static str),
}

pub enum Effect {
    None,
    RailEndstop(usize),
    RailMax(usize),
    RailHomingSpeed(usize),
    RailSecondSpeed(usize),
    RailRetractSpeed(usize),
    RailRetractDist(usize),
    HeaterMinTemp(usize),
    HeaterMaxTemp(usize),
    MaxZVelocity,
    MaxZAccel,
    /// Run this command with the value appended (`to_string`).
    Script(&'static str),
}

#[derive(PartialEq, Eq)]
pub enum Gate {
    Gated,
    Inverted,
    Ungated,
}

include!("para_table.rs");

const SPECIAL: &[&str] = &[
    "M8052", "M8062", "M8095", "M8096", "M8212", "M8213", "M8233", "M8253", "M8800", "M8801", "M8802", "M8803", "M8807", "M8810", "M8819", "M8822", "M8823", "M8824", "M8825", "M8826",
];

pub fn handles(key: &str) -> bool {
    SPECIAL.contains(&key) || TABLE.iter().any(|r| r.0 == key)
}

impl Exec {
    fn gate_blocks(&self, gate: &Gate) -> bool {
        match gate {
            Gate::Gated => !self.is_root && self.from_file,
            Gate::Inverted => !self.is_root && !self.from_file,
            Gate::Ungated => false,
        }
    }

    pub(super) fn cmd_para(&mut self, key: &str, g: &GCmd) -> CmdResult {
        if let Some((_, kind, sec, k, effect, gate)) = TABLE.iter().find(|r| r.0 == key) {
            if self.gate_blocks(gate) {
                return Ok(());
            }
            return self.para_row(kind, sec, k, effect, g);
        }
        let gate = if matches!(key, "M8803" | "M8807") { Gate::Ungated } else { Gate::Gated };
        if self.gate_blocks(&gate) {
            return Ok(());
        }
        self.para_special(key, g)
    }

    fn para_row(&mut self, kind: &Kind, sec: &str, key: &str, effect: &Effect, g: &GCmd) -> CmdResult {
        let value: f64 = match kind {
            Kind::Pin(pin) => {
                match g.get_int("S", i32::MIN as i64, None, None)? {
                    1 => self.core().cfgs.printer.set_value(sec, key, pin),
                    -1 => self.core().cfgs.printer.set_value(sec, key, &format!("!{pin}")),
                    _ => {}
                }
                return Ok(());
            }
            Kind::Str(def, rs, rk) => {
                let cur = self.core().cfgs.printer.get_string(rs, rk, def);
                let v = g.get_string("S", &cur);
                self.core().cfgs.printer.set_value(sec, key, &v);
                return Ok(());
            }
            Kind::Double(def, rs, rk) => {
                let cur = self.core().cfgs.printer.get_double(rs, rk, *def);
                let v = g.f("S", cur)?;
                self.core().cfgs.printer.set_double(sec, key, v);
                v
            }
            Kind::Int(def, rs, rk) => {
                let cur = self.core().cfgs.printer.get_int(rs, rk, *def);
                let v = g.i("S", cur)?;
                self.core().cfgs.printer.set_int(sec, key, v);
                v as f64
            }
            Kind::DoubleAsInt(def, rs, rk) => {
                let cur = self.core().cfgs.printer.get_double(rs, rk, *def);
                let v = g.f("S", cur)? as i64;
                self.core().cfgs.printer.set_int(sec, key, v);
                v as f64
            }
        };
        let mut c = self.core();
        match effect {
            Effect::None => {}
            Effect::RailEndstop(i) => c.m.axes[*i].position_endstop = value,
            Effect::RailMax(i) => c.m.axes[*i].max = value,
            Effect::RailHomingSpeed(i) => c.m.axes[*i].homing_speed = value,
            Effect::RailSecondSpeed(i) => c.m.axes[*i].second_homing_speed = value,
            Effect::RailRetractSpeed(i) => c.m.axes[*i].retract_speed = value,
            Effect::RailRetractDist(i) => c.m.axes[*i].retract_dist = value,
            Effect::HeaterMinTemp(i) => c.heaters[*i].cfg.min_temp = value,
            Effect::HeaterMaxTemp(i) => c.heaters[*i].cfg.max_temp = value,
            Effect::MaxZVelocity => c.planner.lim.max_z_velocity = value,
            Effect::MaxZAccel => c.planner.lim.max_z_accel = value,
            Effect::Script(prefix) => {
                drop(c);
                self.single(&format!("{prefix}{}", to_string(value)));
            }
        }
        Ok(())
    }

    fn para_special(&mut self, key: &str, g: &GCmd) -> CmdResult {
        match key {
            "M8052" | "M8062" => {
                let (sec, h) = if key == "M8052" { ("extruder", 0) } else { ("heater_bed", 1) };
                // M8062's defaults are the extruder's constants in the firmware
                let (dp, di, dd) = (24.860427, 2.702253, 57.178292);
                let (cp, ci, cd) = {
                    let c = self.core();
                    (c.cfgs.printer.get_double(sec, "pid_Kp", dp), c.cfgs.printer.get_double(sec, "pid_Ki", di), c.cfgs.printer.get_double(sec, "pid_Kd", dd))
                };
                let (p, i, d) = (g.f("P", cp)?, g.f("I", ci)?, g.f("D", cd)?);
                let mut c = self.core();
                c.cfgs.printer.set_double(sec, "pid_Kp", p);
                c.cfgs.printer.set_double(sec, "pid_Ki", i);
                c.cfgs.printer.set_double(sec, "pid_Kd", d);
                c.heaters[h].set_gains(p, i, d);
            }
            "M8095" => {
                let (e, b) = (g.i("E", 1)?, g.i("B", 1)?);
                let mut s = String::new();
                if e != 0 {
                    s.push_str("extruder,");
                }
                if b != 0 {
                    s.push_str("heater_bed,");
                }
                self.core().cfgs.printer.set_value("controller_fan board_cooling_fan", "heater", &s);
            }
            "M8096" => {
                let mut s = String::new();
                for (a, name) in [("X", "stepper_x"), ("Y", "stepper_y"), ("Z", "stepper_z"), ("E", "extruder")] {
                    if g.i(a, 1)? != 0 {
                        s.push_str(name);
                        s.push(',');
                    }
                }
                self.core().cfgs.printer.set_value("controller_fan board_cooling_fan", "stepper", &s);
            }
            // empty in the firmware
            "M8212" | "M8213" | "M8819" | "M8822" => {}
            "M8253" => {
                g.i("R", 0)?;
            }
            "M8233" => {
                let cur = self.core().cfgs.printer.get_double("probe", "z_offset", 0.0);
                let zo = g.f("S", cur)?;
                let apply = g.i("P", 1)? == 1;
                self.apply_z_offset(zo, apply, true);
            }
            "M8800" => self.core().cfgs.save_printer(),
            "M8801" => {
                let mut c = self.core();
                for (s, k, v) in DEFAULT_CONFIG {
                    c.cfgs.printer.set_value(s, k, v);
                }
                c.cfgs.save_printer();
            }
            "M8802" => {
                let sim = self.core().hw.simulated;
                if sim {
                    super::console("M8802: reboot (not done in the simulator)");
                } else {
                    let _ = std::process::Command::new("reboot").status();
                }
            }
            "M8803" | "M8807" => self.copy_configs_usb(key == "M8803"),
            "M8810" => super::console("M8810: log export is not available in ccos"),
            "M8823" => {
                let (fix, zo, material, extra) = {
                    let c = self.core();
                    let material = c.cfgs.printer.get_string("bed_mesh", "platform_material", "standard");
                    let fk = if material == "standard" { "standard_fix_z_offset" } else { "enhancement_fix_z_offset" };
                    (-c.cfgs.unmodifiable.get_double("strain_gauge", fk, 0.0), c.cfgs.printer.get_double("probe", "z_offset", 0.0), material, c.m.axes[2].position_endstop_extra)
                };
                super::console(&format!("cur_fix_z_offset: {} cur_z_offset: {}", cpp_double(fix), cpp_double(zo)));
                let mut c = self.core();
                match material.as_str() {
                    "standard" => c.cfgs.unmodifiable.set_double("strain_gauge", "standard_fix_z_offset", -(fix - zo)),
                    "enhancement" => c.cfgs.unmodifiable.set_double("strain_gauge", "enhancement_fix_z_offset", -(fix - zo)),
                    _ => {}
                }
                c.cfgs.printer.set_int("probe", "z_offset", 0);
                c.cfgs.printer.set_double("stepper_z", "position_endstop", extra);
                let k = if material == "standard" { "standard_z_offset" } else { "enhancement_z_offset" };
                c.cfgs.printer.set_int("bed_mesh", k, 0);
                c.m.axes[2].position_endstop = extra;
                c.cfgs.save_printer();
                if let Some(p) = c.cfgs.path("unmodifiable.cfg") {
                    let _ = c.cfgs.unmodifiable.write(&p);
                }
                c.cfgs.save_user("probe", &["z_offset"]);
                c.cfgs.save_user("stepper_z", &["position_endstop", "position_endstop_extra"]);
                // the firmware also rewrites [bed_mesh] in user_printer.cfg with
                // the stepper_z key list, which leaves it empty
                c.cfgs.save_user("bed_mesh", &["position_endstop", "position_endstop_extra"]);
                drop(c);
                if let Some(bm) = self.bm.as_mut() {
                    bm.probe_z_offset = 0.0;
                }
                self.single(&format!("BED_MESH_SET_INDEX TYPE={material} INDEX=0"));
            }
            "M8824" => {
                let mut c = self.core();
                c.cfgs.unmodifiable.set_double("strain_gauge", "standard_fix_z_offset", 0.15);
                c.cfgs.unmodifiable.set_double("strain_gauge", "enhancement_fix_z_offset", 0.17);
                if let Some(p) = c.cfgs.path("unmodifiable.cfg") {
                    let _ = c.cfgs.unmodifiable.write(&p);
                }
            }
            "M8825" => self.core().cfgs.sysconf.set_int("system", "cumulative_time", 0),
            "M8826" => {
                let usb = self.usb_dir();
                let id = std::fs::read_to_string("/sys/class/sunxi_info/sys_info").ok().and_then(|s| s.lines().find(|l| l.contains("chipid")).map(|l| l.rsplit(':').next().unwrap_or("").trim().to_string())).unwrap_or_default();
                let path = usb.join("chipid.txt");
                use std::io::Write;
                match std::fs::OpenOptions::new().append(true).create(true).open(&path) {
                    Ok(mut f) => {
                        let _ = writeln!(f, "{id}");
                        super::console(&format!("chipid writen to file: {}", path.display()));
                    }
                    Err(_) => super::console(&format!("Failed to open file: {}", path.display())),
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// The USB stick mount point (`/mnt/exUDISK`; `<sim dir>/usb` in ccsim).
    fn usb_dir(&self) -> std::path::PathBuf {
        let c = self.core();
        if c.hw.simulated {
            c.cfgs.dir.as_ref().map(|d| d.parent().unwrap_or(d).join("usb")).unwrap_or_else(|| "/mnt/exUDISK".into())
        } else {
            "/mnt/exUDISK".into()
        }
    }

    /// M8803 (configs → USB) / M8807 (USB → configs).
    fn copy_configs_usb(&mut self, to_usb: bool) {
        let usb = self.usb_dir();
        let Some(dir) = self.core().cfgs.dir.clone() else { return };
        let _ = std::fs::create_dir_all(&usb);
        let files: &[&str] = if to_usb { &["printer.cfg", "user_printer.cfg", "unmodifiable.cfg"] } else { &["printer.cfg", "user_printer.cfg"] };
        for f in files {
            let (from, to) = if to_usb { (dir.join(f), usb.join(f)) } else { (usb.join(f), dir.join(f)) };
            if *f == "printer.cfg" || from.exists() {
                let _ = std::fs::copy(&from, &to);
            }
        }
    }
}
