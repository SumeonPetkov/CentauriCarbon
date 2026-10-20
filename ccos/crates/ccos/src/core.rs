//! The printer core: owns the MCU sessions, the planner, the step queues,
//! heaters, fans and LEDs. One I/O thread runs [`Core::service`] from a
//! `poll()` loop (reads MCU replies, runs heater control and the
//! heater/controller fan and idle timers, generates and sends steps); the
//! G-code thread calls into the core under its mutex and waits on the
//! condition variable.
//!
//! Behaviour follows Elegoo's firmware where it differs from upstream
//! Klipper: moves past a soft limit are clamped, moves on unhomed axes and
//! cold or over-extrusions are silently dropped, fan changes apply at the
//! end of the queued moves with the latest commanded value, heater targets
//! apply immediately.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::time::Duration;

use cc_proto::dict::{pin_number, Arg};
use cc_proto::OwnedMessage;

use crate::app::{LinkState, LinkStatus, Ui};
use crate::clock::now;
use crate::config::Configs;
use crate::heater::{Heater, PwmOut, MAX_HEAT_TIME};
use crate::hw::Hardware;
use crate::machine::{self, dsp, toolhead, FanCfg, Machine};
use crate::mcu::{ntc_frac, ntc_temp, Mcu, Result};
use crate::motion::itersolve::{Kin, StepperKin};
use crate::motion::planner::{Move, Planner};
use crate::motion::shaper::{self, Pulses};
use crate::motion::stepper::{StepMsg, StepQueue};
use crate::motion::trapq::TrapQ;
use crate::Target;

/// `SDS_CHECK_TIME`: the minimum `kin_flush_delay`.
pub const SDS_CHECK_TIME: f64 = 0.008576076596340965;

/// `[input_shaper]` state (input_shaper.cpp).
#[derive(Debug, Clone, Copy)]
pub struct ShaperState {
    pub type_x: i32,
    pub type_y: i32,
    pub freq_x: f64,
    pub freq_y: f64,
    pub damping_x: f64,
    pub damping_y: f64,
    pub saved_freq_x: f64,
    pub saved_freq_y: f64,
    old_delay: f64,
}

/// Free move-queue slots on one MCU, tracked from the host side
/// (Klipper's `steppersync`): a message may only be sent once the MCU has
/// consumed an older one.
struct Slots {
    release: BinaryHeap<Reverse<u64>>,
    cap: usize,
}

impl Slots {
    fn new() -> Self {
        Self { release: BinaryHeap::new(), cap: machine::MOVE_COUNT - machine::RESERVED_SLOTS }
    }
    fn take(&mut self, release: u64) -> u64 {
        let mut min = 0;
        if self.release.len() >= self.cap {
            min = self.release.pop().map_or(0, |r| r.0);
        }
        self.release.push(Reverse(release));
        min
    }
}

/// Fans: 0 model (part), 1 model_helper_fan (aux), 2 box_fan (chamber),
/// 3 hotend (heater_fan), 4 board (controller_fan).
pub const FAN_MODEL: usize = 0;
pub const FAN_HELPER: usize = 1;
pub const FAN_BOX: usize = 2;
pub const FAN_HOTEND: usize = 3;
pub const FAN_BOARD: usize = 4;

/// `trdispatch` (chelper/trdispatch.c) for one homing on one MCU: while the
/// MCU reports `can_trigger=1` the timeout is pushed out to the report
/// clock + `expire_ticks` (sent when it moves by `min_extend_ticks`); when
/// it reports a trigger, `trsync_trigger` with the dispatch reason is sent
/// back. Runs on the I/O thread like the firmware's serial reader.
#[derive(Debug, Clone)]
pub struct TrDispatch {
    pub target: Target,
    pub oid: u8,
    pub can_trigger: bool,
    pub reason: u8,
    pub expire_clock: u64,
    pub expire_ticks: u64,
    pub min_extend_ticks: u64,
    /// `trigger_reason` of the MCU's trigger report (`is_trigger`).
    pub triggered: Option<u8>,
}

/// `PrinterFan` state (fan.cpp).
#[derive(Debug, Clone, Copy)]
pub struct Fan {
    pub cfg: FanCfg,
    /// Last commanded speed 0..1 (what the UI shows).
    pub current_speed: f64,
    /// Applied PWM after startup_voltage/max_power mapping.
    pub last_fan_value: f64,
    pub last_fan_time: f64,
}

impl Fan {
    fn new(cfg: FanCfg) -> Self {
        Self { cfg, current_speed: 0.0, last_fan_value: 0.0, last_fan_time: 0.0 }
    }
}

pub struct Core {
    pub hw: Hardware,
    pub cfgs: Configs,
    pub m: Machine,
    pub mcus: [Option<Mcu>; 3],
    pub links: [LinkStatus; 3],
    pub steppers: [StepQueue; 4],
    pub enabled: [bool; 4],
    pub planner: Planner,
    /// Committed X/Y/Z motion and extruder motion (`trapq`s).
    pub trapq: TrapQ,
    pub etrapq: TrapQ,
    /// Step solvers: A (x+y), B (x−y), Z, extruder.
    pub kins: [StepperKin; 4],
    /// Steps are generated this long behind the planned moves (the largest
    /// shaper/pressure-advance window + SDS_CHECK_TIME).
    pub kin_flush_delay: f64,
    kin_flush_times: Vec<f64>,
    pub shaper: ShaperState,
    /// (pressure_advance, pressure_advance_smooth_time) as last set.
    pub pressure_advance: (f64, f64),
    /// 0 = extruder, 1 = bed.
    pub heaters: [Heater; 2],
    pub chamber: Option<f64>,
    pub fans: [Fan; 5],
    /// LED red channel (led1 = toolhead PC9, led2 = PG15 chamber light).
    pub leds: [f64; 2],
    led_time: f64,
    /// Axis limits set (homed).
    pub homed: [bool; 3],
    /// `M211`: soft limits enforced.
    pub soft_limits: bool,
    /// Toolhead position after the last queued move.
    pub commanded: [f64; 4],
    pub shutdown: Option<String>,
    slots: [Slots; 3],
    batch: [Vec<StepMsg>; 3],
    gen_time: f64,
    /// Homing in progress: generate only a little ahead.
    pub homing: bool,
    pub trdispatch: Option<TrDispatch>,
    last_move_add: f64,
    /// Replies the G-code thread waits for (trsync_state, stepper_position…).
    pub responses: VecDeque<(Target, OwnedMessage)>,
    /// Pending fan callbacks: token → fan index.
    fan_tokens: Vec<(u32, usize)>,
    next_token: u32,
    /// The G-code thread is printing or paused (idle timeout does nothing).
    pub busy_printing: bool,
    /// Last time something happened (idle timeout).
    pub last_activity: f64,
    timers_at: f64,
    pub ui: Ui,
    ui_at: f64,
    pub exit: bool,
    wake_fd: i32,
}

pub struct Printer {
    pub core: Mutex<Core>,
    pub cv: Condvar,
    wake: (i32, i32),
}

pub type Shared = Arc<Printer>;

impl Printer {
    pub fn new(hw: Hardware, cfgs: Configs, ui: Ui) -> Shared {
        let mut fds = [0i32; 2];
        unsafe { libc::pipe2(fds.as_mut_ptr(), libc::O_NONBLOCK | libc::O_CLOEXEC) };
        let core = Core::new(hw, cfgs, ui, fds[1]);
        Arc::new(Printer { core: Mutex::new(core), cv: Condvar::new(), wake: (fds[0], fds[1]) })
    }

    pub fn lock(&self) -> MutexGuard<'_, Core> {
        self.core.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// The I/O loop. Returns when `Core::exit` is set.
    pub fn run_io(self: &Arc<Self>) {
        let mut pfds: Vec<libc::pollfd> = Vec::with_capacity(4);
        loop {
            let timeout_ms = {
                let c = self.lock();
                if c.exit {
                    return;
                }
                pfds.clear();
                pfds.push(libc::pollfd { fd: self.wake.0, events: libc::POLLIN, revents: 0 });
                for m in c.mcus.iter().flatten() {
                    if let Some(fd) = m.fd() {
                        pfds.push(libc::pollfd { fd, events: libc::POLLIN, revents: 0 });
                    }
                }
                ((c.deadline() - now()) * 1000.0).clamp(0.0, 100.0).ceil() as i32
            };
            unsafe { libc::poll(pfds.as_mut_ptr(), pfds.len() as _, timeout_ms) };
            let mut drain = [0u8; 64];
            while unsafe { libc::read(self.wake.0, drain.as_mut_ptr() as _, drain.len()) } > 0 {}
            let mut c = self.lock();
            c.service();
            drop(c);
            self.cv.notify_all();
        }
    }

    /// Wait until `f` returns a value; fails on shutdown or timeout.
    pub fn wait<R>(&self, timeout: f64, what: &str, mut f: impl FnMut(&mut Core) -> Option<R>) -> Result<R> {
        let deadline = now() + timeout;
        let mut c = self.lock();
        loop {
            if let Some(r) = f(&mut c) {
                return Ok(r);
            }
            if let Some(s) = &c.shutdown {
                return Err(s.clone().into());
            }
            if c.exit {
                return Err("exiting".into());
            }
            let t = now();
            if t >= deadline {
                return Err(format!("timeout waiting for {what}").into());
            }
            c = self.cv.wait_timeout(c, Duration::from_secs_f64((deadline - t).min(0.1))).unwrap_or_else(|p| p.into_inner()).0;
        }
    }
}

impl Drop for Printer {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.wake.0);
            libc::close(self.wake.1);
        }
    }
}

fn steppers_new(m: &Machine) -> [StepQueue; 4] {
    std::array::from_fn(|i| {
        let s = &m.steppers[i];
        StepQueue::new(s.oid, s.step_dist, s.dir.invert, (machine::MAX_STEP_ERROR * s.target.nominal_clock()) as u64)
    })
}

impl Core {
    fn new(hw: Hardware, cfgs: Configs, ui: Ui, wake_fd: i32) -> Self {
        let m = Machine::from_config(&cfgs.printer);
        // InputShaper::InputShaper (an unknown type name maps to 0, "zv")
        let c = &cfgs.printer;
        let ty = |k: &str| shaper::parse(&c.get_string("input_shaper", k, &c.get_string("input_shaper", "shaper_type", "mzv"))).unwrap_or(0);
        let shaper = ShaperState {
            type_x: ty("shaper_type_x"),
            type_y: ty("shaper_type_y"),
            freq_x: c.get_double("input_shaper", "shaper_freq_x", 0.0),
            freq_y: c.get_double("input_shaper", "shaper_freq_y", 0.0),
            damping_x: c.get_double("input_shaper", "damping_ratio_x", 0.1),
            damping_y: c.get_double("input_shaper", "damping_ratio_y", 0.1),
            saved_freq_x: 0.0,
            saved_freq_y: 0.0,
            old_delay: 0.0,
        };
        let has_shaper = c.has_section("input_shaper");
        let pa = (m.extruder.pressure_advance, m.extruder.pressure_advance_smooth_time);
        let mut core = Self {
            steppers: steppers_new(&m),
            planner: Planner::new(m.limits),
            trapq: TrapQ::default(),
            etrapq: TrapQ::default(),
            kins: [StepperKin::new(Kin::CorexyPlus), StepperKin::new(Kin::CorexyMinus), StepperKin::new(Kin::Z), StepperKin::new(Kin::Extruder)],
            kin_flush_delay: SDS_CHECK_TIME,
            kin_flush_times: Vec::new(),
            shaper,
            pressure_advance: (0.0, 0.0),
            heaters: [Heater::new(m.heaters[0]), Heater::new(m.heaters[1])],
            fans: [Fan::new(m.fans[0]), Fan::new(m.fans[1]), Fan::new(m.fans[2]), Fan::new(m.hotend_fan), Fan::new(m.board_fan)],
            hw,
            cfgs,
            m,
            mcus: [None, None, None],
            links: Default::default(),
            enabled: [false; 4],
            chamber: None,
            leds: [0.0; 2],
            led_time: 0.0,
            homed: [false; 3],
            soft_limits: true,
            commanded: [0.0; 4],
            shutdown: None,
            slots: [Slots::new(), Slots::new(), Slots::new()],
            batch: Default::default(),
            gen_time: 0.0,
            homing: false,
            trdispatch: None,
            last_move_add: 0.0,
            responses: VecDeque::with_capacity(64),
            fan_tokens: Vec::new(),
            next_token: 1,
            busy_printing: false,
            last_activity: now(),
            timers_at: 0.0,
            ui,
            ui_at: 0.0,
            exit: false,
            wake_fd,
        };
        // PrinterExtruder::PrinterExtruder, then InputShaper::connect
        core.set_pressure_advance(pa.0, pa.1);
        if has_shaper {
            let s = core.shaper;
            core.set_input_shaper(s.type_x, s.type_y, s.freq_x, s.freq_y, s.damping_x, s.damping_y);
        }
        core
    }

    // ---- step generation set-up ----

    /// `ToolHead::note_step_generation_scan_time`.
    pub fn note_step_generation_scan_time(&mut self, delay: f64, old_delay: f64) {
        self.flush_step_generation();
        if old_delay != 0.0 {
            if let Some(i) = self.kin_flush_times.iter().position(|d| *d == old_delay) {
                self.kin_flush_times.remove(i);
            }
        }
        if delay != 0.0 {
            self.kin_flush_times.push(delay);
        }
        if let Some(max) = self.kin_flush_times.iter().copied().reduce(f64::max) {
            self.kin_flush_delay = max + SDS_CHECK_TIME;
        }
        self.planner.kin_flush_delay = self.kin_flush_delay;
    }

    /// `PrinterExtruder::set_pressure_advance`.
    pub fn set_pressure_advance(&mut self, pressure_advance: f64, smooth_time: f64) {
        let old = if self.pressure_advance.0 <= 1e-15 { 0.0 } else { self.pressure_advance.1 };
        let new = if pressure_advance <= 1e-15 { 0.0 } else { smooth_time };
        self.note_step_generation_scan_time(new * 0.5, old * 0.5);
        self.kins[3].set_pressure_advance(pressure_advance, new);
        self.pressure_advance = (pressure_advance, smooth_time);
    }

    /// `InputShaper::set_input_shaper`. (The Y window is computed with the
    /// X damping ratio, as the firmware does.)
    pub fn set_input_shaper(&mut self, type_x: i32, type_y: i32, freq_x: f64, freq_y: f64, damping_x: f64, damping_y: f64) {
        if type_x != self.shaper.type_x || type_y != self.shaper.type_y {
            self.flush_step_generation();
        }
        let delay = Pulses::new(type_x, freq_x, damping_x).window().max(Pulses::new(type_y, freq_y, damping_x).window());
        let old = self.shaper.old_delay;
        self.note_step_generation_scan_time(delay, old);
        let s = &mut self.shaper;
        (s.old_delay, s.type_x, s.type_y, s.freq_x, s.freq_y, s.damping_x, s.damping_y) = (delay, type_x, type_y, freq_x, freq_y, damping_x, damping_y);
        let (sx, sy) = (Pulses::new(type_x, freq_x, damping_x), Pulses::new(type_y, freq_y, damping_y));
        for k in &mut self.kins {
            k.set_shaper(sx, sy);
        }
    }

    /// Committed planner moves into the trapqs (`trapq_append`,
    /// `PrinterExtruder::move`).
    fn drain_committed(&mut self) {
        for s in std::mem::take(&mut self.planner.committed) {
            if s.kinematic {
                self.trapq.append(s.print_time, s.accel_t, s.cruise_t, s.decel_t, [s.start[0], s.start[1], s.start[2]], [s.axes_r[0], s.axes_r[1], s.axes_r[2]], s.start_v, s.cruise_v, s.accel);
            }
            let r = s.axes_r[3];
            if r != 0.0 {
                let pa = (s.can_pa as u8) as f64;
                self.etrapq.append(s.print_time, s.accel_t, s.cruise_t, s.decel_t, [s.start[3], 0.0, 0.0], [1.0, pa, 0.0], s.start_v * r, s.cruise_v * r, s.accel * r);
            }
        }
    }

    /// itersolve every stepper up to `until`, then free what no solver needs.
    fn gen_steps(&mut self, until: f64) {
        self.drain_committed();
        let t = now();
        for (si, def) in self.m.steppers.iter().enumerate() {
            let Some(mcu) = self.mcus[def.target as usize].as_ref() else {
                self.kins[si].last_flush_time = until;
                continue;
            };
            let est = cc_proto::clock::ClockEstimate { conv_time: t, conv_clock: mcu.clock.clock(t), freq: mcu.clock.freq };
            let to_clock = move |pt: f64| est.clock_from_time(pt);
            let tq = if si == 3 { &self.etrapq } else { &self.trapq };
            self.kins[si].generate_steps(tq, until, &mut self.steppers[si], &to_clock);
        }
        self.gen_time = self.gen_time.max(until);
        // (1 ns: the planner's end time and the sum of a move's pieces can
        // differ in the last bit)
        let free = until - self.kin_flush_delay + 1e-9;
        self.trapq.free_moves(free);
        self.etrapq.free_moves(free);
    }

    /// `ToolHead::flush_step_generation`: commit the look-ahead and generate
    /// every step, the shaped/smoothed tail of the last move included; the
    /// next move starts `kin_flush_delay` after that.
    pub fn flush_step_generation(&mut self) {
        self.planner.flush(false, now(), machine::START_DELAY);
        let until = self.planner.print_time + self.kin_flush_delay;
        if until > self.gen_time {
            self.gen_steps(until);
        }
        self.planner.min_restart = self.planner.min_restart.max(self.gen_time);
        self.wake();
    }

    /// Make the I/O thread run `service` now (after queueing messages).
    pub fn wake(&self) {
        unsafe { libc::write(self.wake_fd, b"w".as_ptr() as _, 1) };
    }

    pub fn log(&self, s: impl Into<String>) {
        self.ui.log(s);
    }

    pub fn mcu(&mut self, t: Target) -> Result<&mut Mcu> {
        self.mcus[t as usize].as_mut().ok_or_else(|| format!("{} is not connected", t.title()).into())
    }

    // ---- attach / detach ----

    pub fn install(&mut self, t: Target, m: Mcu, detail: String) {
        self.mcus[t as usize] = Some(m);
        self.links[t as usize] = LinkStatus { state: LinkState::Ok, detail };
        self.slots[t as usize] = Slots::new();
        let fresh = steppers_new(&self.m);
        for (i, s) in fresh.into_iter().enumerate() {
            if self.m.steppers[i].target == t {
                self.steppers[i] = s;
                self.enabled[i] = false;
            }
        }
        match t {
            Target::Dsp => {
                self.homed = [false; 3];
                self.heaters[1] = Heater::new(self.m.heaters[1]);
                self.leds[1] = 0.0;
            }
            Target::Toolhead => {
                self.heaters[0] = Heater::new(self.m.heaters[0]);
                self.leds[0] = 0.0;
            }
            Target::StrainGauge => {}
        }
        for f in self.fans.iter_mut().filter(|f| f.cfg.pin_oid.0 == t) {
            *f = Fan::new(f.cfg);
        }
        self.set_position(self.commanded);
        self.wake();
    }

    pub fn detach(&mut self, t: Target, why: &str) {
        if self.mcus[t as usize].take().is_some() {
            self.log(format!("{}: disconnected ({why})", t.title()));
        }
        self.links[t as usize] = LinkStatus { state: LinkState::Failed, detail: why.to_string() };
        match t {
            Target::Dsp => {
                self.heaters[1].temp = None;
                self.chamber = None;
            }
            Target::Toolhead => self.heaters[0].temp = None,
            Target::StrainGauge => {}
        }
    }

    // ---- safety ----

    /// `invoke_shutdown`: heaters and part fan off, every MCU shut down.
    pub fn emergency(&mut self, reason: String) {
        if self.shutdown.is_some() {
            return;
        }
        self.log(format!("SHUTDOWN: {reason}"));
        self.ui.set_status(format!("Shutdown: {reason} — tap Restart"));
        for h in &mut self.heaters {
            h.target = 0.0;
        }
        self.fans[FAN_MODEL].current_speed = 0.0;
        for m in self.mcus.iter_mut().flatten() {
            let _ = m.queue("emergency_stop");
        }
        self.stop_motion();
        self.homed = [false; 3];
        self.homing = false;
        self.trdispatch = None;
        self.shutdown = Some(reason);
        self.wake();
    }

    // ---- positions ----

    /// Define the toolhead position without moving (G92, homing).
    pub fn set_position(&mut self, pos: [f64; 4]) {
        // ToolHead::set_position: every step of the moves so far first
        self.flush_step_generation();
        self.trapq.clear();
        self.etrapq.clear();
        for i in 0..4 {
            let mm = self.kins[i].position_from_coord(pos);
            self.steppers[i].set_position_mm(mm);
        }
        self.commanded = pos;
    }

    /// `set_position(pos, homing_axes)`: also mark axes homed.
    pub fn set_position_homed(&mut self, pos: [f64; 4], axes: &[usize]) {
        self.set_position(pos);
        for &a in axes {
            self.homed[a] = true;
        }
    }

    /// Position from the steppers' (commanded) step counts.
    pub fn stepper_position(&self) -> [f64; 4] {
        let a = self.steppers[0].position_mm();
        let b = self.steppers[1].position_mm();
        [(a + b) * 0.5, (a - b) * 0.5, self.steppers[2].position_mm(), self.steppers[3].position_mm()]
    }

    pub fn live_position(&self) -> [f64; 4] {
        self.planner.position_at(now()).unwrap_or(self.commanded)
    }

    /// Forget planned motion; the position becomes where the generated
    /// steps end (steps already sent are stopped by trsync or shutdown).
    pub fn stop_motion(&mut self) {
        let t = now();
        self.planner.clear();
        self.trapq.clear();
        self.etrapq.clear();
        self.gen_time = self.gen_time.max(t);
        self.planner.print_time = self.gen_time;
        self.planner.min_restart = self.planner.min_restart.max(self.gen_time);
        for s in &mut self.steppers {
            s.out.clear();
        }
        self.commanded = self.stepper_position();
    }

    pub fn buffered_time(&self) -> f64 {
        (self.planner.print_time - now()).max(0.0)
    }

    /// All motion executed and every scheduled message sent.
    pub fn idle(&self) -> bool {
        self.planner.queued() == 0
            && now() > self.planner.print_time + 0.02
            && self.trapq.is_empty()
            && self.etrapq.is_empty()
            && self.steppers.iter().all(|s| s.out.is_empty())
            && self.mcus.iter().flatten().all(|m| m.backlog() == 0)
    }

    // ---- motion ----

    fn motion_allowed(&self) -> Result<()> {
        if let Some(s) = &self.shutdown {
            return Err(format!("Printer is shut down: {s}").into());
        }
        if let Some(l) = self.hw.motion_lock() {
            return Err(l.into());
        }
        if self.mcus[Target::Dsp as usize].is_none() {
            return Err("DSP is not connected".into());
        }
        Ok(())
    }

    /// `ToolHead::move1`: queue a move. Ok(false) when the move is dropped
    /// the way the stock firmware drops it (unhomed axis, cold or
    /// over-extrusion). `kin_checks` = soft limit / homed checks (homing
    /// moves bypass them).
    pub fn add_move(&mut self, target: [f64; 4], speed: f64, kin_checks: bool) -> Result<bool> {
        self.motion_allowed()?;
        let lim = self.planner.lim;
        let mut target = target;
        let mut m = Move::new(&lim, self.commanded, target, speed);
        if m.move_d <= 0.0 {
            return Ok(true);
        }
        if m.is_kinematic {
            // corexy check_move: clamp to the limits, drop unhomed
            if kin_checks && self.soft_limits {
                let xy_out = (0..2).any(|i| self.outside(i, target[i]));
                let z_moves = m.axes_d[2] != 0.0;
                let mut changed = false;
                if xy_out || z_moves {
                    for i in 0..3 {
                        if m.axes_d[i] == 0.0 || !self.outside(i, target[i]) {
                            continue;
                        }
                        if !self.homed[i] {
                            return Ok(false);
                        }
                        let ax = &self.m.axes[i];
                        let clamped = target[i].clamp(ax.min, ax.max);
                        crate::gcode::console(&format!("soft endstop hit on axis {i}, end_pos: {:.6}, limit: {:.6}", target[i], clamped));
                        target[i] = clamped;
                        changed = true;
                    }
                }
                if changed {
                    m = Move::new(&lim, self.commanded, target, speed);
                    if m.move_d <= 0.0 {
                        return Ok(true);
                    }
                }
            }
            if m.axes_d[2] != 0.0 {
                let z_ratio = m.move_d / m.axes_d[2].abs();
                m.limit_speed(lim.max_z_velocity * z_ratio, lim.max_z_accel * z_ratio);
            }
        }
        if m.axes_d[3] != 0.0 {
            // PrinterExtruder::check_move (Elegoo)
            if self.mcus[Target::Toolhead as usize].is_none() {
                return Err("Toolhead is not connected".into());
            }
            if !self.heaters[0].can_extrude() {
                return Ok(false);
            }
            let ex = self.m.extruder;
            if (m.axes_d[0] == 0.0 && m.axes_d[1] == 0.0) || m.axes_r[3] < 0.0 {
                if m.axes_d[3].abs() <= ex.max_extrude_only_distance {
                    let inv = 1.0 / m.axes_r[3].abs();
                    m.limit_speed(ex.max_extrude_only_velocity * inv, ex.max_extrude_only_accel * inv);
                }
            } else if m.axes_r[3] > ex.max_extrude_ratio && m.axes_d[3] > ex.nozzle_diameter * ex.max_extrude_ratio {
                crate::gcode::console(&format!(
                    "Overextrude: {:.6} vs {:.6} (area {:.3} vs {:.3})",
                    m.axes_r[3],
                    ex.max_extrude_ratio,
                    m.axes_r[3] * ex.filament_area,
                    ex.max_extrude_ratio * ex.filament_area
                ));
                return Ok(false);
            }
        }
        let t = now();
        if self.enabled.iter().any(|e| !e) {
            self.motor_enable_all();
        }
        self.planner.add(m, t, machine::START_DELAY);
        self.commanded = target;
        self.last_move_add = t;
        self.last_activity = t;
        self.wake();
        Ok(true)
    }

    fn outside(&self, i: usize, v: f64) -> bool {
        // unhomed limits are (1, -1): everything is outside
        if !self.homed[i] {
            return true;
        }
        let ax = &self.m.axes[i];
        v < ax.min || v > ax.max
    }

    /// Commit everything in the look-ahead (`flush_lookahead`).
    pub fn flush_moves(&mut self) {
        self.planner.flush(false, now(), machine::START_DELAY);
        self.wake();
    }

    /// `get_last_move_time`.
    pub fn last_move_time(&mut self) -> f64 {
        self.planner.last_move_time(now(), machine::START_DELAY)
    }

    /// `toolhead.dwell(delay)`.
    pub fn dwell(&mut self, delay: f64) {
        self.planner.dwell(delay, now(), machine::START_DELAY);
        self.wake();
    }

    fn set_enable(&mut self, i: usize, on: bool, at: f64) {
        let s = self.m.steppers[i].clone();
        let Some(m) = self.mcus[s.target as usize].as_mut() else { return };
        let clock = m.clock.clock(at.max(now() + 0.02));
        // enable pins are active-low ("!PE10"): pin level = on xor invert
        let level = on != s.enable.invert;
        let msg = m.encode("queue_digital_out", &[("oid", Arg::Int(s.enable_oid as i64)), ("clock", Arg::Int(clock as u32 as i64)), ("on_ticks", Arg::Int(level as i64))]);
        m.queue_msg(msg);
        self.enabled[i] = on;
    }

    fn motor_enable_all(&mut self) {
        let at = now() + 0.05;
        for i in 0..4 {
            if !self.enabled[i] {
                self.set_enable(i, true, at);
            }
        }
    }

    /// `M18`/`M84` (`motor_off`): disable every stepper at the end of the
    /// queued moves; all axes become unhomed.
    pub fn motor_off(&mut self) -> Result<()> {
        self.motion_allowed()?;
        self.dwell(0.1);
        let at = self.last_move_time();
        for i in 0..4 {
            self.set_enable(i, false, at);
        }
        self.homed = [false; 3];
        self.dwell(0.1);
        Ok(())
    }

    /// `SET_STEPPER_ENABLE` for one stepper (does not unhome).
    pub fn stepper_enable(&mut self, i: usize, on: bool) -> Result<()> {
        self.motion_allowed()?;
        self.dwell(0.1);
        let at = self.last_move_time();
        self.set_enable(i, on, at);
        self.dwell(0.1);
        Ok(())
    }

    /// Generate and queue steps ahead of their time.
    fn generate(&mut self, t: f64) {
        // Generate in chunks: each chunk boundary splits the step stream
        // into an extra queue_step per stepper, so refill only when the
        // buffer runs low (Klipper flushes in batches for the same reason).
        // Steps lag the planned moves by kin_flush_delay (a shaped position
        // needs the moves after it); the last move's tail is generated once
        // nothing is queued and the buffer runs out, and the next move then
        // starts kin_flush_delay later (min_restart_time).
        self.drain_committed();
        let kfd = self.kin_flush_delay;
        let (low, high) = if self.homing { (machine::DRIP_DELAY + kfd, machine::DRIP_DELAY + kfd + machine::DRIP_SEGMENT_TIME) } else { machine::GEN_AHEAD };
        let pt = self.planner.print_time;
        let tail = self.planner.queued() == 0 && pt < t + low + kfd;
        let end = if tail { pt + kfd } else { pt - kfd };
        let start = self.gen_time.max(t - 1.0);
        let until = (t + high).min(end.max(start));
        let behind_end = end <= t + high;
        if until > start && (start - t < low || behind_end && until - start > 0.005) {
            self.gen_steps(until);
            if until > pt - kfd {
                self.planner.min_restart = self.planner.min_restart.max(until);
            }
        }
        let mut batch = std::mem::take(&mut self.batch);
        for (si, def) in self.m.steppers.iter().enumerate() {
            let s = &mut self.steppers[si];
            s.flush();
            batch[def.target as usize].append(&mut s.out);
        }
        for (i, msgs) in batch.iter_mut().enumerate() {
            if msgs.is_empty() {
                continue;
            }
            msgs.sort_by_key(|m| m.req_clock);
            let Some(mcu) = self.mcus[i].as_mut() else { continue };
            for m in msgs.iter() {
                let mut min = m.min_clock;
                if m.uses_slot {
                    min = min.max(self.slots[i].take(m.req_clock));
                }
                mcu.queue_scheduled(min, m.req_clock, m.oid, m.msg);
            }
        }
        for b in batch.iter_mut() {
            b.clear();
        }
        self.batch = batch;
        while self.planner.segments.front().is_some_and(|s| s.end_time() < t - 0.5 && s.end_time() <= self.gen_time) {
            self.planner.segments.pop_front();
        }
    }

    // ---- outputs ----

    fn queue_pwm(&mut self, place: (Target, u8), cycle_time: f64, at: f64, value: f64) {
        let (t, oid) = place;
        let Some(m) = self.mcus[t as usize].as_mut() else { return };
        let clock = m.clock.clock(at.max(now() + 0.02));
        // the cycle configured on the MCU (nominal clock, as in `configure`)
        let cycle = (cycle_time * t.nominal_clock()).round();
        let on = if value >= 1.0 { cycle.max(1.0) } else { (value * cycle).round() } as u32;
        let msg = m.encode("queue_digital_out", &[("oid", Arg::Int(oid as i64)), ("clock", Arg::Int(clock as u32 as i64)), ("on_ticks", Arg::Int(on as i64))]);
        m.queue_msg(msg);
    }

    /// `PrinterFan::set_speed_from_command`: the speed becomes current now;
    /// the PWM changes when the moves queued before it are planned, using
    /// the then-current speed.
    pub fn fan_from_command(&mut self, fan: usize, speed: f64) {
        self.fans[fan].current_speed = speed;
        let token = self.next_token;
        self.next_token += 1;
        match self.planner.lookahead_callback(token, now(), machine::START_DELAY) {
            Some(t) => self.fan_apply(fan, t),
            None => self.fan_tokens.push((token, fan)),
        }
        self.wake();
    }

    /// `PrinterFan::set_speed` (value = current speed).
    fn fan_apply(&mut self, i: usize, print_time: f64) {
        let f = self.fans[i];
        let c = f.cfg;
        let mut value = f.current_speed;
        if c.startup_voltage != 0.0 {
            value = if value == 0.0 { 0.0 } else { (1.0 - c.startup_voltage) * value + c.startup_voltage };
        }
        if value < c.off_below {
            value = 0.0;
        }
        value = (value * c.max_power).clamp(0.0, c.max_power);
        if value == f.last_fan_value {
            return;
        }
        let mut pt = (f.last_fan_time + 0.1).max(print_time);
        if value > 0.0 && value < c.max_power && c.kick_start_time > 0.0 && (f.last_fan_value == 0.0 || value - f.last_fan_value > 0.5) {
            self.queue_pwm(c.pin_oid, c.cycle_time, pt, c.max_power);
            pt += c.kick_start_time;
        }
        self.queue_pwm(c.pin_oid, c.cycle_time, pt, value);
        self.fans[i].last_fan_value = value;
        self.fans[i].last_fan_time = pt;
    }

    /// `SET_LED_led<n>` transmit: only the red channel has a pin, at now + 0.1 s.
    pub fn set_led(&mut self, i: usize, red: f64) {
        if self.leds[i] == red {
            return;
        }
        self.leds[i] = red;
        let place = if i == 0 { (Target::Toolhead, toolhead::LED) } else { (Target::Dsp, dsp::LIGHT) };
        let pt = (now() + 0.1).max(self.led_time + 0.1);
        self.led_time = pt;
        self.queue_pwm(place, machine::LED_CYCLE_TIME, pt, red);
        self.wake();
    }

    /// Heater target, applied immediately (`set_temperature`).
    pub fn set_heater(&mut self, i: usize, target: f64) -> Result<()> {
        if target > 0.0 {
            self.motion_allowed()?;
        }
        self.flush_moves();
        self.heaters[i].set_target(target);
        self.last_activity = now();
        self.wake();
        Ok(())
    }

    fn send_heater(&mut self, i: usize, at: f64, value: f64) {
        if !self.hw.motion {
            return;
        }
        let place = if i == 0 { (Target::Toolhead, toolhead::HOTEND_HEATER) } else { (Target::Dsp, dsp::BED_HEATER) };
        let cycle_time = self.heaters[i].cfg.pwm_cycle_time;
        self.queue_pwm(place, cycle_time, at, value);
    }

    // ---- the I/O loop body ----

    pub fn deadline(&self) -> f64 {
        let mut d = self.ui_at.min(self.timers_at);
        for m in self.mcus.iter().flatten() {
            d = d.min(m.deadline());
        }
        if !self.planner.segments.is_empty() || self.planner.queued() > 0 {
            d = d.min(now() + 0.02);
        }
        d
    }

    pub fn service(&mut self) {
        let t = now();
        for i in 0..3 {
            let Some(m) = self.mcus[i].as_mut() else { continue };
            // clock queries go out every second, so silence means trouble
            let r = m.service(Duration::ZERO).and_then(|_| {
                if t - m.last_rx > 5.0 {
                    Err(format!("no response from the {} MCU for 5 s", Target::ALL[i].title()).into())
                } else {
                    Ok(())
                }
            });
            if let Err(e) = r {
                let busy = self.enabled.iter().any(|e| *e) || self.heaters.iter().any(|h| h.target > 0.0);
                self.detach(Target::ALL[i], &e.to_string());
                if busy && self.hw.motion {
                    self.emergency(format!("{} link lost: {e}", Target::ALL[i].title()));
                }
                continue;
            }
            if let Some(why) = m.shutdown {
                if self.shutdown.is_none() {
                    let name = Target::ALL[i].title();
                    self.emergency(format!("MCU '{name}' shutdown: {why}"));
                }
            }
            self.handle_messages(i);
        }
        for i in 0..2 {
            if let Err(e) = self.heaters[i].verify(t) {
                self.emergency(e);
            }
        }
        if self.shutdown.is_none() {
            if self.planner.queued() > 0 && t - self.last_move_add > 0.1 {
                self.planner.flush(false, t, machine::START_DELAY);
            }
            let fired = std::mem::take(&mut self.planner.fired);
            for (token, time) in fired {
                if let Some(p) = self.fan_tokens.iter().position(|(k, _)| *k == token) {
                    let (_, fan) = self.fan_tokens.remove(p);
                    self.fan_apply(fan, time);
                }
            }
            self.generate(t);
            if t >= self.timers_at {
                self.timers_at = t + 1.0;
                self.timers(t);
            }
        }
        for i in 0..3 {
            if let Some(m) = self.mcus[i].as_mut() {
                let _ = m.service(Duration::ZERO);
            }
        }
        if t >= self.ui_at {
            self.ui_at = t + 0.1;
            self.publish();
        }
    }

    /// 1 Hz timers: heater_fan, controller_fan, idle_timeout.
    fn timers(&mut self, t: f64) {
        let est = t + 0.1;
        if self.mcus[Target::Toolhead as usize].is_some() {
            // heater_fan: on while heating or above heater_temp (no hysteresis)
            let h = &self.heaters[0];
            let speed = if h.target != 0.0 || h.smoothed > self.m.hotend_fan_temp { 1.0 } else { 0.0 };
            if self.fans[FAN_HOTEND].current_speed != speed {
                self.fans[FAN_HOTEND].current_speed = speed;
                self.fan_apply(FAN_HOTEND, est);
            }
        }
        if self.hw.motion && self.mcus[Target::Dsp as usize].is_some() {
            // controller_fan: on while a stepper is enabled or a heater is on
            let active = self.enabled.iter().any(|e| *e) || self.heaters.iter().any(|h| h.target != 0.0);
            let speed = if active { 1.0 } else { 0.0 };
            if self.fans[FAN_BOARD].current_speed != speed {
                self.fans[FAN_BOARD].current_speed = speed;
                self.fan_apply(FAN_BOARD, est);
            }
            // idle_timeout: heaters off after `timeout` idle (not while printing/paused)
            let heating = self.heaters.iter().any(|h| h.target != 0.0);
            if heating && !self.busy_printing && t - self.last_activity > self.m.idle_timeout {
                self.log("Idle timeout: heaters off");
                for h in &mut self.heaters {
                    h.set_target(0.0);
                }
                self.last_activity = t;
            }
        }
    }

    fn handle_messages(&mut self, i: usize) {
        let target = Target::ALL[i];
        loop {
            let Some(m) = self.mcus[i].as_mut().and_then(|m| m.inbox.pop_front()) else { break };
            match m.name() {
                "analog_in_state" => {
                    let oid = m.int("oid").unwrap_or(-1) as u8;
                    let full = machine::ADC_SAMPLES as f64 * cc_proto::clock::ADC_MAX as f64;
                    // calc_temp clamps the ADC fraction instead of failing
                    let frac = (m.int("value").unwrap_or(0) as f64 / full).clamp(0.00001, 0.99999);
                    let mcu = self.mcus[i].as_ref().unwrap();
                    let next = mcu.clock.extend(m.int("next_clock").unwrap_or(0) as u32);
                    // read time = start of this sample burst + the burst length
                    let read_time = mcu.clock.time(next) - machine::ADC_REPORT_TIME + machine::ADC_SAMPLES as f64 * machine::ADC_SAMPLE_TIME;
                    match (target, oid) {
                        (Target::Dsp, dsp::ADC_CHAMBER) => self.chamber = ntc_temp(frac, 3950.0),
                        (Target::Dsp, dsp::ADC_BED) | (Target::Toolhead, toolhead::ADC_HOTEND) => {
                            let h = if target == Target::Dsp { 1 } else { 0 };
                            let temp = ntc_temp(frac, self.heaters[h].cfg.beta).unwrap_or(-273.15);
                            if let PwmOut::Send(at, v) = self.heaters[h].update(read_time, temp) {
                                self.send_heater(h, at, v);
                            }
                        }
                        _ => {}
                    }
                }
                "trsync_state" | "stepper_position" | "endstop_state" | "pong" | "tmcuart_response" => {
                    if m.name() == "trsync_state" {
                        self.trdispatch_state(target, &m);
                    }
                    if self.responses.len() == 64 {
                        self.responses.pop_front();
                    }
                    self.responses.push_back((target, m));
                }
                _ => {}
            }
        }
    }

    /// `handle_trsync_state` of trdispatch.c (and `MCU_trsync`'s
    /// `is_trigger`).
    fn trdispatch_state(&mut self, target: Target, m: &OwnedMessage) {
        let oid = m.int("oid").unwrap_or(-1);
        let Some(td) = self.trdispatch.as_mut() else { return };
        if td.target != target || td.oid as i64 != oid || !td.can_trigger {
            return;
        }
        let Some(mcu) = self.mcus[target as usize].as_mut() else { return };
        if m.int("can_trigger") == Some(0) {
            td.can_trigger = false;
            td.triggered = Some(m.int("trigger_reason").unwrap_or(0) as u8);
            let msg = mcu.encode("trsync_trigger", &[("oid", Arg::Int(oid)), ("reason", Arg::Int(td.reason as i64))]);
            mcu.queue_msg(msg);
            return;
        }
        let status = mcu.clock.extend(m.int("clock").unwrap_or(0) as u32);
        let expire = status + td.expire_ticks;
        if expire as i64 - td.expire_clock as i64 >= td.min_extend_ticks as i64 {
            td.expire_clock = expire;
            let msg = mcu.encode("trsync_set_timeout", &[("oid", Arg::Int(oid)), ("clock", Arg::Int(expire as u32 as i64))]);
            mcu.queue_msg(msg);
        }
    }

    /// Copy what the screen shows.
    pub fn publish(&self) {
        let pos = self.live_position();
        let mut s = self.ui.lock();
        s.nozzle = self.heaters[0].temp.map(|_| self.heaters[0].smoothed);
        s.nozzle_target = self.heaters[0].target;
        s.bed = self.heaters[1].temp.map(|_| self.heaters[1].smoothed);
        s.bed_target = self.heaters[1].target;
        s.chamber = self.chamber;
        s.light = self.leds[1] > 0.0;
        s.fan = self.fans[FAN_MODEL].current_speed;
        s.links = self.links.clone();
        s.pos = pos;
        s.homed = self.homed;
        s.shutdown = self.shutdown.clone();
        s.motion_lock = self.hw.motion_lock();
    }
}

/// Open, reset and configure one MCU. Blocking; call without the core lock.
#[cfg(target_os = "linux")]
pub fn attach(hw: &Hardware, m: &Machine, t: Target) -> Result<(Mcu, String)> {
    use crate::hw::Endpoint;
    if t != Target::Dsp || matches!(hw.endpoint(t), Endpoint::Serial { .. }) {
        hw.restart(t)?;
    }
    let window = if t == Target::Dsp { 1024 } else { 192 };
    let mut mcu = Mcu::new(t.label(), hw.open(t)?, t.nominal_clock(), window);
    mcu.sync()?;
    let cfg = mcu.request("get_config", "config")?;
    if cfg.int("is_config") == Some(1) {
        return Err(format!("{} is already configured by another host; restart it first", t.title()).into());
    }
    mcu.sync_clock()?;
    let hz = mcu.clock.freq;
    configure(&mut mcu, m, t, hw.motion)?;
    Ok((mcu, format!("{:.1} MHz", hz / 1e6)))
}

fn configure(mcu: &mut Mcu, mc: &Machine, t: Target, motion: bool) -> Result<()> {
    let ppb = t.pins_per_bank();
    let pin = |p: &machine::Pin| pin_number(&p.name, ppb).ok_or_else(|| format!("bad pin {}", p.name));
    let pn = |p: &str| pin_number(p, ppb).unwrap();
    let ticks = |s: f64| (s * t.nominal_clock()).round() as u64;
    let mut cmds: Vec<String> = Vec::new();
    // setup_start_value(start, shutdown) + soft PWM cycle
    let dout = |cmds: &mut Vec<String>, oid: u8, pinno: u32, value: u8, default: u8, max: f64, cycle: f64| {
        cmds.push(format!("config_digital_out oid={oid} pin={pinno} value={value} default_value={default} max_duration={}", ticks(max)));
        if cycle > 0.0 {
            cmds.push(format!("set_digital_out_pwm_cycle oid={oid} cycle_ticks={}", ticks(cycle)));
        }
    };
    let mut adcs: Vec<(u8, u32, f64, f64, f64)> = Vec::new(); // oid, pin, beta, min_temp, max_temp
    let count = match t {
        Target::Dsp => dsp::COUNT,
        Target::Toolhead => toolhead::COUNT,
        Target::StrainGauge => 0,
    };
    cmds.push(format!("allocate_oids count={count}"));
    if motion {
        for s in mc.steppers.iter().filter(|s| s.target == t) {
            cmds.push(format!("config_stepper oid={} step_pin={} dir_pin={} invert_step=0 step_pulse_ticks={}", s.oid, pin(&s.step)?, pin(&s.dir)?, ticks(machine::STEP_PULSE)));
            let disabled = s.enable.invert as u8; // pin level while disabled
            dout(&mut cmds, s.enable_oid, pin(&s.enable)?, disabled, disabled, 0.0, 0.0);
        }
    }
    for f in mc.fans.iter().chain([&mc.hotend_fan, &mc.board_fan]) {
        if f.pin_oid.0 != t || (!motion && f.pin_oid.1 != toolhead::HOTEND_FAN) {
            continue;
        }
        let pinname = match f.pin_oid {
            (Target::Toolhead, toolhead::PART_FAN) => "PB5",
            (Target::Toolhead, _) => "PC8",
            (_, dsp::AUX_FAN) => "PG17",
            (_, dsp::CHAMBER_FAN) => "PG18",
            _ => "PG16",
        };
        let shutdown = (f.shutdown_speed.clamp(0.0, f.max_power) >= 0.5) as u8;
        dout(&mut cmds, f.pin_oid.1, pn(pinname), 0, shutdown, 0.0, f.cycle_time);
    }
    match t {
        Target::Dsp => {
            if motion {
                for a in &mc.axes {
                    cmds.push(format!("config_endstop oid={} pin={} pull_up=0", a.endstop_oid, pin(&a.endstop)?));
                }
                cmds.push(format!("config_trsync oid={}", dsp::TRSYNC));
                dout(&mut cmds, dsp::BED_HEATER, pn("PG13"), 0, 0, MAX_HEAT_TIME, mc.heaters[1].pwm_cycle_time);
            }
            dout(&mut cmds, dsp::LIGHT, pin(&mc.led_pins[1])?, 0, 0, 0.0, machine::LED_CYCLE_TIME);
            adcs.push((dsp::ADC_BED, pn("PB14"), mc.heaters[1].beta, mc.heaters[1].min_temp, mc.heaters[1].max_temp));
            adcs.push((dsp::ADC_CHAMBER, pn("PB13"), 3950.0, -200.0, 125.0));
        }
        Target::Toolhead => {
            if motion {
                dout(&mut cmds, toolhead::HOTEND_HEATER, pn("PB6"), 0, 0, MAX_HEAT_TIME, mc.heaters[0].pwm_cycle_time);
            }
            dout(&mut cmds, toolhead::LED, pin(&mc.led_pins[0])?, 0, 0, 0.0, machine::LED_CYCLE_TIME);
            adcs.push((toolhead::ADC_HOTEND, pn("PA3"), mc.heaters[0].beta, mc.heaters[0].min_temp, mc.heaters[0].max_temp));
        }
        Target::StrainGauge => {}
    }
    for (oid, p, _, _, _) in &adcs {
        cmds.push(format!("config_analog_in oid={oid} pin={p}"));
    }
    if motion {
        // MCU_TMC_uart_bitbang::build_config, 20 kbaud
        for u in mc.tmc_uarts.iter().filter(|u| u.target == t) {
            let oid = if t == Target::Dsp { dsp::TMCUART } else { toolhead::TMCUART };
            let bit_time = (t.nominal_clock() / crate::tmc::TMC_BAUD_RATE) as u64;
            cmds.push(format!("config_tmcuart oid={oid} rx_pin={} pull_up={} tx_pin={} bit_time={bit_time}", pin(&u.rx)?, u.pull_up as u8, pin(&u.tx)?));
        }
    }
    cmds.push("finalize_config crc=0".into());
    for c in &cmds {
        mcu.queue(c)?;
    }
    mcu.flush(3.0)?;
    let full = (machine::ADC_SAMPLES * cc_proto::clock::ADC_MAX) as f64;
    let start = now() + 0.1;
    for (i, (oid, _, beta, min_temp, max_temp)) in adcs.iter().enumerate() {
        // MCU range check (adc_temperature.cpp): min from max_temp, max from min_temp
        let min_v = (ntc_frac(*max_temp, *beta) * full) as u32;
        let max_v = ((ntc_frac(*min_temp, *beta) * full).ceil() as u32).min(65535);
        let clock = mcu.clock.clock(start + i as f64 * 0.01);
        mcu.queue(&format!(
            "query_analog_in oid={oid} clock={} sample_ticks={} sample_count={} rest_ticks={} min_value={min_v} max_value={max_v} range_check_count={}",
            clock as u32,
            ticks(machine::ADC_SAMPLE_TIME),
            machine::ADC_SAMPLES,
            ticks(machine::ADC_REPORT_TIME),
            machine::ADC_RANGE_CHECKS
        ))?;
    }
    mcu.flush(2.0)?;
    Ok(())
}
