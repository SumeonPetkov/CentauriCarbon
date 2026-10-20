//! TMC2209 commands and the UART transport (driver logic: `crate::tmc`).
//!
//! `SET_TMC_CURRENT_<stepper>` per driver; `INIT_TMC`, `SET_TMC_FIELD` and
//! `DUMP_TMC` are mux commands in Klipper, but the stock `register_mux_command`
//! ignores the key and re-registers the plain command, so the last driver
//! created — `[tmc2209 extruder]` — handles them whatever `STEPPER=` says.

use cc_proto::dict::Arg;

use super::parse::{cpp_to_string, CmdResult, GCmd, Lim};
use super::Exec;
use crate::core::Shared;
use crate::machine::{dsp, toolhead};
use crate::tmc::{Tmc, TmcBus};
use crate::Target;

/// `DBL_MIN`, the firmware's "parameter not given" marker.
const DBL_MIN: f64 = f64::MIN_POSITIVE;

/// `send_with_response("tmcuart_send …", "tmcuart_response")`: a copy resent
/// while the UART is still busy is dropped by the MCU.
pub(super) struct UartBus<'a> {
    pub p: &'a Shared,
}

impl TmcBus for UartBus<'_> {
    fn transfer(&mut self, target: Target, write: &[u8], read: u8, print_time: Option<f64>) -> Result<Vec<u8>, String> {
        let oid = match target {
            Target::Dsp => dsp::TMCUART,
            Target::Toolhead => toolhead::TMCUART,
            Target::StrainGauge => return Err("no TMC UART on the load-cell board".into()),
        } as i64;
        let args = [("oid", Arg::Int(oid)), ("write", Arg::Bytes(write)), ("read", Arg::Int(read as i64))];
        let m = super::send_with_response_at(self.p, target, "tmcuart_send", &args, "tmcuart_response", oid, print_time)?;
        Ok(m.message().bytes("read").unwrap_or(&[]).to_vec())
    }

    fn log(&mut self, msg: &str) {
        super::console(msg);
    }

    fn debug(&mut self, msg: &str) {
        if std::env::var_os("CCOS_DEBUG").is_some() {
            eprintln!("ccos: {msg}");
        }
    }
}

impl Exec {
    /// The `[tmc2209 …]` drivers from printer.cfg, in file order.
    pub(super) fn tmc_load(p: &Shared) -> Vec<Tmc> {
        let c = p.lock();
        let cfg = &c.cfgs.printer;
        crate::tmc::sections(cfg)
            .iter()
            .filter_map(|sec| {
                let stepper = sec.split(' ').nth(1)?;
                let def = c.m.steppers.iter().find(|s| s.name == stepper)?;
                Some(Tmc::from_config(cfg, sec, def.target, def.step_dist))
            })
            .collect()
    }

    /// `klippy:connect` of every driver on a connected MCU (or only those of
    /// `only`), in event-name order: extruder, stepper_x, stepper_y,
    /// stepper_z. A failure is logged and the next driver initialised.
    pub(super) fn tmc_connect(&mut self, only: Option<Target>) {
        let (motion, connected) = {
            let c = self.core();
            (c.hw.motion, [0, 1, 2].map(|i| c.mcus[i].is_some()))
        };
        if !motion {
            return;
        }
        let mut order: Vec<usize> = (0..self.tmc.len()).collect();
        order.sort_by(|a, b| self.tmc[*a].name.cmp(&self.tmc[*b].name));
        let fresh = Self::tmc_load(&self.p);
        for i in order {
            let t = self.tmc[i].target;
            if only.is_some_and(|o| o != t) || !connected[t as usize] {
                continue;
            }
            // a reconnected MCU is a fresh firmware start: the driver object
            // (register image, IFCNT) is created again from the config
            if let Some(f) = fresh.iter().find(|f| f.name == self.tmc[i].name) {
                self.tmc[i] = f.clone();
            }
            let mut bus = UartBus { p: &self.p };
            if let Err(e) = self.tmc[i].init_registers(&mut bus, None) {
                super::console(&format!("TMC {} failed to init: {e}", self.tmc[i].name));
            }
        }
    }

    fn tmc_index(&self, name: &str) -> Option<usize> {
        self.tmc.iter().position(|t| t.name == name)
    }

    fn tmc_ready(&self, i: usize) -> Result<(), super::parse::Abort> {
        let c = self.core();
        if let Some(l) = c.hw.motion_lock() {
            return Err(l.into());
        }
        c.mcus[self.tmc[i].target as usize].as_ref().map(|_| ()).ok_or_else(|| format!("{} is not connected", self.tmc[i].target.title()).into())
    }

    /// Commands this module handles (`SET_TMC_CURRENT_*` only for drivers
    /// in the config).
    pub(super) fn tmc_handles(&self, key: &str) -> bool {
        match key.strip_prefix("SET_TMC_CURRENT_") {
            Some(n) => self.tmc_index(n).is_some(),
            None => matches!(key, "INIT_TMC" | "SET_TMC_FIELD" | "DUMP_TMC") && !self.tmc.is_empty(),
        }
    }

    pub(super) fn cmd_tmc(&mut self, key: &str, g: &GCmd) -> CmdResult {
        // the mux commands belong to the driver created last
        let i = match key.strip_prefix("SET_TMC_CURRENT_") {
            Some(n) => self.tmc_index(n).unwrap(),
            None => self.tmc.len() - 1,
        };
        self.tmc_ready(i)?;
        match key {
            "INIT_TMC" => {
                let pt = self.core().last_move_time();
                let mut bus = UartBus { p: &self.p };
                self.tmc[i].init_registers(&mut bus, Some(pt))?;
            }
            "SET_TMC_FIELD" => {
                let field = g.get_string("FIELD", "");
                let value = g.i("VALUE", 0)?;
                let pt = self.core().last_move_time();
                let mut bus = UartBus { p: &self.p };
                self.tmc[i].set_tmc_field(&mut bus, &field, value, pt);
            }
            "DUMP_TMC" => {
                let _ = self.core().last_move_time();
                let mut bus = UartBus { p: &self.p };
                self.tmc[i].dump(&mut bus)?;
            }
            _ => self.cmd_set_tmc_current(i, g)?,
        }
        Ok(())
    }

    /// `SET_TMC_CURRENT_<stepper> [CURRENT=] [HOLDCURRENT=]`; without
    /// parameters the configured currents are restored. Reports the
    /// currents before a restore, after a change.
    fn cmd_set_tmc_current(&mut self, i: usize, g: &GCmd) -> CmdResult {
        let max = crate::tmc::MAX_CURRENT;
        let (mut prev_run, mut prev_hold) = self.tmc[i].get_current();
        let mut run = g.get_double("CURRENT", DBL_MIN, Lim::min_max(0.0, max))?;
        let mut hold = g.get_double("HOLDCURRENT", DBL_MIN, Lim { min: Some(0.0), max: Some(max), above: Some(0.0), below: None })?;
        if run != DBL_MIN || hold != DBL_MIN {
            if run == DBL_MIN {
                run = prev_run;
            }
            if hold == DBL_MIN {
                hold = prev_hold;
            }
            let pt = self.core().last_move_time();
            let mut bus = UartBus { p: &self.p };
            self.tmc[i].set_current(&mut bus, run, hold, Some(pt));
            (prev_run, prev_hold) = self.tmc[i].get_current();
        }
        if run == DBL_MIN && hold == DBL_MIN {
            let pt = self.core().last_move_time();
            let (r, h) = (self.tmc[i].run_current, self.tmc[i].hold_current);
            let mut bus = UartBus { p: &self.p };
            self.tmc[i].set_current(&mut bus, r, h, Some(pt));
        }
        let name = &self.tmc[i].name;
        if prev_hold.abs() < 1e-15 {
            self.respond(&format!("{name} Run Current: {}A", cpp_to_string(prev_run)));
        } else {
            self.respond(&format!("{name} Run Current: {}A Hold Current: {}A", cpp_to_string(prev_run), cpp_to_string(prev_hold)));
        }
        Ok(())
    }

    /// The driver whose `virtual_endstop` is the endstop of this axis.
    pub(super) fn tmc_for_axis(&self, axis: usize) -> Option<usize> {
        let name = ["stepper_x", "stepper_y", "stepper_z"].get(axis)?;
        self.tmc.iter().position(|t| t.name == *name && t.virtual_endstop)
    }

    /// `homing:homing_move_begin` / `homing:homing_move_end` for the driver
    /// of this axis' virtual endstop.
    pub(super) fn tmc_homing(&mut self, axis: usize, begin: bool) {
        let Some(i) = self.tmc_for_axis(axis) else { return };
        if self.core().hw.motion_lock().is_some() {
            return;
        }
        let mut bus = UartBus { p: &self.p };
        if begin {
            self.tmc[i].homing_begin(&mut bus);
        } else {
            self.tmc[i].homing_end(&mut bus);
        }
    }
}
