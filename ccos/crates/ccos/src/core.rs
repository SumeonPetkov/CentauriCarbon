//! The printer core: owns the MCU sessions, the planner, the step queues and
//! the heaters. One I/O thread runs [`Core::service`] from a `poll()` loop
//! (reads MCU replies, runs heater control and safety checks, generates and
//! sends steps); the G-code thread calls into the core under its mutex and
//! waits on the condition variable.

use std::collections::{BinaryHeap, VecDeque};
use std::cmp::Reverse;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::time::Duration;

use cc_proto::dict::{pin_number, Arg};
use cc_proto::OwnedMessage;

use crate::app::{LinkState, LinkStatus, Ui};
use crate::clock::now;
use crate::heater::{Heater, BED, EXTRUDER, MAX_HEAT_TIME};
use crate::hw::Hardware;
use crate::machine::{self, dsp, toolhead, AXES, STEPPERS};
use crate::mcu::{ntc_frac, ntc_temp, Mcu, Result};
use crate::motion::planner::{Move, Planner};
use crate::motion::stepper::{StepMsg, StepQueue};
use crate::Target;

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
    /// Take a slot released at `release`; returns the clock before which the
    /// message must not be sent.
    fn take(&mut self, release: u64) -> u64 {
        let mut min = 0;
        if self.release.len() >= self.cap {
            min = self.release.pop().map_or(0, |r| r.0);
        }
        self.release.push(Reverse(release));
        min
    }
}

/// Configured outputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Output {
    PartFan,
    AuxFan,
    ChamberFan,
    BoardFan,
    HotendFan,
    Light,
}

impl Output {
    fn place(self) -> (Target, u8) {
        match self {
            Output::PartFan => (Target::Toolhead, toolhead::PART_FAN),
            Output::HotendFan => (Target::Toolhead, toolhead::HOTEND_FAN),
            Output::AuxFan => (Target::Dsp, dsp::AUX_FAN),
            Output::ChamberFan => (Target::Dsp, dsp::CHAMBER_FAN),
            Output::BoardFan => (Target::Dsp, dsp::BOARD_FAN),
            Output::Light => (Target::Dsp, dsp::LIGHT),
        }
    }
}

pub struct Core {
    pub hw: Hardware,
    pub mcus: [Option<Mcu>; 3],
    pub links: [LinkStatus; 3],
    pub steppers: [StepQueue; 4],
    pub motors_enabled: bool,
    pub planner: Planner,
    /// 0 = extruder, 1 = bed.
    pub heaters: [Heater; 2],
    pub chamber: Option<f64>,
    pub outputs: [(Output, f64); 6],
    pub homed: [bool; 3],
    /// Toolhead position after the last queued move.
    pub commanded: [f64; 4],
    pub shutdown: Option<String>,
    slots: [Slots; 3],
    /// Reused per-MCU message batch (no allocation per chunk).
    batch: [Vec<StepMsg>; 3],
    gen_time: f64,
    /// Homing in progress: generate only a little ahead.
    pub homing: bool,
    last_move_add: f64,
    /// Replies the G-code thread waits for (trsync_state, stepper_position…).
    pub responses: VecDeque<(Target, OwnedMessage)>,
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
    pub fn new(hw: Hardware, ui: Ui) -> Shared {
        let mut fds = [0i32; 2];
        unsafe { libc::pipe2(fds.as_mut_ptr(), libc::O_NONBLOCK | libc::O_CLOEXEC) };
        let core = Core::new(hw, ui, fds[1]);
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

fn steppers_new() -> [StepQueue; 4] {
    STEPPERS.map(|s| {
        let freq = s.target.nominal_clock();
        StepQueue::new(s.oid, s.step_dist, s.invert_dir, (machine::MAX_STEP_ERROR * freq) as u64)
    })
}

impl Core {
    fn new(hw: Hardware, ui: Ui, wake_fd: i32) -> Self {
        Self {
            hw,
            mcus: [None, None, None],
            links: Default::default(),
            steppers: steppers_new(),
            motors_enabled: false,
            planner: Planner::new(machine::LIMITS),
            heaters: [Heater::new(EXTRUDER), Heater::new(BED)],
            chamber: None,
            outputs: [
                (Output::PartFan, 0.0),
                (Output::AuxFan, 0.0),
                (Output::ChamberFan, 0.0),
                (Output::BoardFan, 0.0),
                (Output::HotendFan, 0.0),
                (Output::Light, 0.0),
            ],
            homed: [false; 3],
            commanded: [0.0; 4],
            shutdown: None,
            slots: [Slots::new(), Slots::new(), Slots::new()],
            batch: Default::default(),
            gen_time: 0.0,
            homing: false,
            last_move_add: 0.0,
            responses: VecDeque::with_capacity(64),
            ui,
            ui_at: 0.0,
            exit: false,
            wake_fd,
        }
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

    /// Install a configured MCU session (see [`attach`]).
    pub fn install(&mut self, t: Target, m: Mcu, detail: String) {
        self.mcus[t as usize] = Some(m);
        self.links[t as usize] = LinkStatus { state: LinkState::Ok, detail };
        self.slots[t as usize] = Slots::new();
        match t {
            Target::Dsp => {
                for i in 0..3 {
                    self.steppers[i] = StepQueue::new(STEPPERS[i].oid, STEPPERS[i].step_dist, STEPPERS[i].invert_dir, self.steppers[i].max_error);
                }
                self.homed = [false; 3];
                self.heaters[1] = Heater::new(BED);
            }
            Target::Toolhead => {
                self.steppers[3] = StepQueue::new(STEPPERS[3].oid, STEPPERS[3].step_dist, STEPPERS[3].invert_dir, self.steppers[3].max_error);
                self.heaters[0] = Heater::new(EXTRUDER);
            }
            Target::StrainGauge => {}
        }
        self.motors_enabled = false;
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

    /// Stop everything: every MCU shuts down (outputs off, steppers stop).
    pub fn emergency(&mut self, reason: String) {
        if self.shutdown.is_some() {
            return;
        }
        self.log(format!("SHUTDOWN: {reason}"));
        self.ui.set_status(format!("Shutdown: {reason} — tap Restart"));
        for m in self.mcus.iter_mut().flatten() {
            let _ = m.queue("emergency_stop");
        }
        self.stop_motion();
        for h in &mut self.heaters {
            h.target = 0.0;
            h.pending = None;
        }
        self.homed = [false; 3];
        self.homing = false;
        self.shutdown = Some(reason);
        self.wake();
    }

    // ---- positions ----

    /// Define the toolhead position without moving (G92, homing).
    pub fn set_position(&mut self, pos: [f64; 4]) {
        for (i, s) in STEPPERS.iter().enumerate() {
            let mm: f64 = (0..4).map(|k| s.kin[k] * pos[k]).sum();
            self.steppers[i].set_position_mm(mm);
        }
        self.commanded = pos;
    }

    /// Position from the steppers' (commanded) step counts.
    pub fn stepper_position(&self) -> [f64; 4] {
        let a = self.steppers[0].position_mm();
        let b = self.steppers[1].position_mm();
        [(a + b) * 0.5, (a - b) * 0.5, self.steppers[2].position_mm(), self.steppers[3].position_mm()]
    }

    /// Where the toolhead is right now (planned time line), else commanded.
    pub fn live_position(&self) -> [f64; 4] {
        self.planner.position_at(now()).unwrap_or(self.commanded)
    }

    /// Forget all planned motion (homing stop, emergency). Steps already
    /// sent are the MCU's business (stopped by trsync or shutdown).
    /// The position becomes where the generated steps end.
    pub fn stop_motion(&mut self) {
        let t = now();
        self.planner.clear();
        self.gen_time = self.gen_time.max(t);
        self.planner.print_time = self.gen_time;
        for s in &mut self.steppers {
            s.out.clear();
        }
        self.commanded = self.stepper_position();
    }

    /// Planned motion not yet executed (seconds).
    pub fn buffered_time(&self) -> f64 {
        (self.planner.print_time - now()).max(0.0)
    }

    /// All motion executed and every scheduled message sent.
    pub fn idle(&self) -> bool {
        self.planner.queued() == 0
            && now() > self.planner.print_time + 0.02
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

    /// Queue a move to `target` at `speed` mm/s. `check`: enforce homing
    /// and travel limits (off while homing).
    pub fn add_move(&mut self, target: [f64; 4], speed: f64, accel: Option<f64>, check: bool) -> Result<()> {
        self.motion_allowed()?;
        let lim = self.planner.lim;
        let mut m = Move::new(&lim, self.commanded, target, speed);
        if m.move_d <= 0.0 {
            return Ok(());
        }
        if m.is_kinematic {
            for (i, ax) in AXES.iter().enumerate() {
                if m.axes_d[i] == 0.0 || !check {
                    continue;
                }
                if !self.homed[i] {
                    return Err(format!("Must home axis first: {} {:.3} {:.3} {:.3}", ax.name, target[0], target[1], target[2]).into());
                }
                if target[i] < ax.min || target[i] > ax.max {
                    return Err(format!("Move out of range: {:.3} {:.3} {:.3}", target[0], target[1], target[2]).into());
                }
            }
            if m.axes_d[2] != 0.0 {
                let z_ratio = m.move_d / m.axes_d[2].abs();
                m.limit_speed(lim.max_z_velocity * z_ratio, lim.max_z_accel * z_ratio);
            }
        }
        if m.axes_d[3] != 0.0 {
            if self.mcus[Target::Toolhead as usize].is_none() {
                return Err("Toolhead is not connected".into());
            }
            match self.heaters[0].temp {
                Some(t) if t >= machine::MIN_EXTRUDE_TEMP => {}
                _ => return Err("Extrude below minimum temp".into()),
            }
            if (m.axes_d[0] == 0.0 && m.axes_d[1] == 0.0) || m.axes_r[3] < 0.0 {
                let inv = 1.0 / m.axes_r[3].abs();
                m.limit_speed(lim.max_extrude_only_velocity * inv, lim.max_extrude_only_accel * inv);
            }
        }
        if let Some(a) = accel {
            m.limit_speed(speed, a);
        }
        let t = now();
        if !self.motors_enabled {
            self.set_motors(true)?;
        }
        self.planner.add(m, t, machine::START_DELAY);
        self.commanded = target;
        self.last_move_add = t;
        self.wake();
        Ok(())
    }

    /// Commit everything in the look-ahead (end of a command batch).
    pub fn flush_moves(&mut self) {
        self.planner.flush(false, now(), machine::START_DELAY);
        self.wake();
    }

    pub fn set_motors(&mut self, on: bool) -> Result<()> {
        self.motion_allowed()?;
        let t = now() + 0.05;
        for s in STEPPERS {
            let Some(m) = self.mcus[s.target as usize].as_mut() else { continue };
            let clock = m.clock.clock(t);
            // Active-low: 0 = enabled.
            let msg = m.encode("queue_digital_out", &[("oid", Arg::Int(s.enable_oid as i64)), ("clock", Arg::Int(clock as u32 as i64)), ("on_ticks", Arg::Int((!on) as i64))]);
            m.queue_msg(msg);
        }
        self.motors_enabled = on;
        if !on {
            self.homed = [false; 3];
        }
        self.wake();
        Ok(())
    }

    /// Generate and queue steps up to `GEN_AHEAD` from now.
    fn generate(&mut self, t: f64) {
        // Generate in chunks: each chunk boundary splits the step stream
        // into an extra queue_step per stepper, so refill only when the
        // buffer runs low (Klipper flushes in batches for the same reason).
        let (low, high) = if self.homing { machine::GEN_AHEAD_HOMING } else { machine::GEN_AHEAD };
        let start = self.gen_time.max(t - 1.0);
        let until = (t + high).min(self.planner.print_time.max(start));
        let behind_end = self.planner.print_time <= t + high;
        if until > start && (start - t < low || behind_end && until - start > 0.005) {
            let clocks: [Option<cc_proto::clock::ClockEstimate>; 3] = std::array::from_fn(|i| {
                self.mcus[i].as_ref().map(|m| {
                    let c = &m.clock;
                    cc_proto::clock::ClockEstimate { conv_time: t, conv_clock: c.clock(t), freq: c.freq }
                })
            });
            for seg in self.planner.segments.iter() {
                if seg.end_time() <= start || seg.print_time >= until {
                    continue;
                }
                for (si, def) in STEPPERS.iter().enumerate() {
                    let Some(est) = clocks[def.target as usize] else { continue };
                    let c: f64 = (0..4).map(|k| def.kin[k] * seg.axes_r[k]).sum();
                    if c == 0.0 {
                        continue;
                    }
                    let m0: f64 = (0..4).map(|k| def.kin[k] * seg.start[k]).sum();
                    let to_clock = move |pt: f64| est.clock_from_time(pt);
                    for (t0, dur, d0, v0, a) in seg.phases() {
                        self.steppers[si].gen_phase(t0, dur, m0 + c * d0, c, v0, a, start, until, &to_clock);
                    }
                }
            }
            self.gen_time = until;
        }
        // Compress and hand to the MCUs in step-time order, with move-queue slots.
        let mut batch = std::mem::take(&mut self.batch);
        for (si, def) in STEPPERS.iter().enumerate() {
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

    /// Set a fan or the light (0..1) shortly from now.
    pub fn set_output(&mut self, o: Output, value: f64) -> Result<()> {
        let value = value.clamp(0.0, 1.0);
        let (t, oid) = o.place();
        let m = self.mcu(t)?;
        let at = now() + 0.05;
        let clock = m.clock.clock(at);
        // The cycle configured on the MCU (nominal clock, as in `configure`).
        let cycle = (machine::FAN_CYCLE_TIME * t.nominal_clock()).round() as u32;
        let on = if o == Output::Light { (value > 0.0) as u32 } else { (value * cycle as f64).round() as u32 };
        let msg = m.encode("queue_digital_out", &[("oid", Arg::Int(oid as i64)), ("clock", Arg::Int(clock as u32 as i64)), ("on_ticks", Arg::Int(on as i64))]);
        m.queue_msg(msg);
        if let Some(e) = self.outputs.iter_mut().find(|e| e.0 == o) {
            e.1 = value;
        }
        self.wake();
        Ok(())
    }

    pub fn output(&self, o: Output) -> f64 {
        self.outputs.iter().find(|e| e.0 == o).map_or(0.0, |e| e.1)
    }

    pub fn set_heater(&mut self, i: usize, target: f64) -> Result<()> {
        if target > 0.0 {
            self.motion_allowed()?;
            let t = if i == 0 { Target::Toolhead } else { Target::Dsp };
            self.mcu(t)?;
        }
        // Validate now, apply when the moves queued before it have run
        // (M104 S0 at the end of a print must not cool the last layer).
        let mut check = self.heaters[i].clone();
        check.set_target(target)?;
        self.flush_moves();
        let at = self.planner.print_time;
        if at > now() {
            self.heaters[i].pending = Some((at, target));
        } else {
            self.heaters[i].pending = None;
            self.heaters[i].target = target;
        }
        self.wake();
        Ok(())
    }

    /// Target including a change queued with the motion.
    pub fn heater_target(&self, i: usize) -> f64 {
        self.heaters[i].pending.map_or(self.heaters[i].target, |p| p.1)
    }

    fn send_heater(&mut self, i: usize, t: f64) {
        if !self.hw.motion {
            return;
        }
        let (target, oid) = if i == 0 { (Target::Toolhead, toolhead::HOTEND_HEATER) } else { (Target::Dsp, dsp::BED_HEATER) };
        let cycle_time = self.heaters[i].cfg.pwm_cycle_time;
        let pwm = self.heaters[i].pwm;
        let Some(m) = self.mcus[target as usize].as_mut() else { return };
        let clock = m.clock.clock(t + 0.1);
        let cycle = (cycle_time * target.nominal_clock()).round() as u32;
        let on = (pwm * cycle as f64).round() as u32;
        let msg = m.encode("queue_digital_out", &[("oid", Arg::Int(oid as i64)), ("clock", Arg::Int(clock as u32 as i64)), ("on_ticks", Arg::Int(on as i64))]);
        m.queue_msg(msg);
        self.heaters[i].sent = Some((pwm, t));
    }

    // ---- the I/O loop body ----

    pub fn deadline(&self) -> f64 {
        let mut d = self.ui_at;
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
            // Clock queries go out every second, so silence means trouble.
            let r = m.service(Duration::ZERO).and_then(|_| {
                if t - m.last_rx > 5.0 {
                    Err(format!("no response from the {} MCU for 5 s", Target::ALL[i].title()).into())
                } else {
                    Ok(())
                }
            });
            if let Err(e) = r {
                let busy = self.motors_enabled || self.heaters.iter().any(|h| h.target > 0.0);
                self.detach(Target::ALL[i], &e.to_string());
                if busy && self.hw.motion {
                    self.emergency(format!("{} link lost: {e}", Target::ALL[i].title()));
                }
                continue;
            }
            if let Some(why) = m.shutdown {
                if self.shutdown.is_none() {
                    let name = Target::ALL[i].title();
                    self.emergency(format!("{name} MCU shutdown: {why}"));
                }
            }
            self.handle_messages(i, t);
        }
        for i in 0..2 {
            if let Some((at, target)) = self.heaters[i].pending {
                if t >= at {
                    self.heaters[i].target = target;
                    self.heaters[i].pending = None;
                }
            }
            if let Err(e) = self.heaters[i].verify(t) {
                self.emergency(e);
            }
        }
        if self.shutdown.is_none() {
            if self.planner.queued() > 0 && t - self.last_move_add > 0.1 {
                self.planner.flush(false, t, machine::START_DELAY);
            }
            self.generate(t);
            self.hotend_fan(t);
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

    fn handle_messages(&mut self, i: usize, t: f64) {
        let target = Target::ALL[i];
        loop {
            let Some(m) = self.mcus[i].as_mut().and_then(|m| m.inbox.pop_front()) else { break };
            match m.name() {
                "analog_in_state" => {
                    let oid = m.int("oid").unwrap_or(-1) as u8;
                    let frac = m.int("value").unwrap_or(0) as f64 / (machine::ADC_SAMPLES as f64 * cc_proto::clock::ADC_MAX as f64);
                    match (target, oid) {
                        (Target::Dsp, dsp::ADC_CHAMBER) => self.chamber = ntc_temp(frac, 3950.0),
                        (Target::Dsp, dsp::ADC_BED) | (Target::Toolhead, toolhead::ADC_HOTEND) => {
                            let h = if target == Target::Dsp { 1 } else { 0 };
                            match ntc_temp(frac, self.heaters[h].cfg.beta) {
                                Some(temp) => {
                                    self.heaters[h].update(t, temp);
                                    if self.heaters[h].needs_send(t) {
                                        self.send_heater(h, t);
                                    }
                                }
                                None => self.heaters[h].temp = None,
                            }
                        }
                        _ => {}
                    }
                }
                "trsync_state" | "stepper_position" | "endstop_state" | "pong" => {
                    if self.responses.len() == 64 {
                        self.responses.pop_front();
                    }
                    self.responses.push_back((target, m));
                }
                _ => {}
            }
        }
    }

    /// Like the stock `heater_fan`: on from 50 °C (or while heating), off
    /// below 45 °C. An unreadable sensor counts as hot.
    fn hotend_fan(&mut self, _t: f64) {
        if self.mcus[Target::Toolhead as usize].is_none() {
            return;
        }
        let h = &self.heaters[0];
        let on = self.output(Output::HotendFan) > 0.0;
        let want = match h.temp {
            _ if h.target > 0.0 => true,
            Some(c) if c >= 50.0 => true,
            Some(c) if c < 45.0 => false,
            Some(_) => on,
            None => true,
        };
        if want != on {
            let _ = self.set_output(Output::HotendFan, want as u8 as f64);
        }
    }

    /// Copy what the screen shows.
    pub fn publish(&self) {
        let pos = self.live_position();
        let mut s = self.ui.lock();
        s.nozzle = self.heaters[0].temp;
        s.nozzle_target = self.heater_target(0);
        s.bed = self.heaters[1].temp;
        s.bed_target = self.heater_target(1);
        s.chamber = self.chamber;
        s.light = self.output(Output::Light) > 0.0;
        s.fan = self.output(Output::PartFan);
        s.links = self.links.clone();
        s.pos = pos;
        s.homed = self.homed;
        s.shutdown = self.shutdown.clone();
        s.motion_lock = self.hw.motion_lock();
    }
}

/// Open, reset and configure one MCU. Blocking; call without the core lock.
#[cfg(target_os = "linux")]
pub fn attach(hw: &Hardware, t: Target) -> Result<(Mcu, String)> {
    use crate::hw::Endpoint;
    if t != Target::Dsp || matches!(hw.endpoint(t), Endpoint::Serial { .. }) {
        hw.restart(t)?;
    }
    let window = if t == Target::Dsp { 1024 } else { 192 };
    let mut m = Mcu::new(t.label(), hw.open(t)?, t.nominal_clock(), window);
    m.sync()?;
    let cfg = m.request("get_config", "config")?;
    if cfg.int("is_config") == Some(1) {
        return Err(format!("{} is already configured by another host; restart it first", t.title()).into());
    }
    m.sync_clock()?;
    let hz = m.clock.freq;
    configure(&mut m, t, hw.motion)?;
    Ok((m, format!("{:.1} MHz", hz / 1e6)))
}

fn configure(m: &mut Mcu, t: Target, motion: bool) -> Result<()> {
    let ppb = t.pins_per_bank();
    let pin = |p: &str| pin_number(p, ppb).unwrap();
    let ticks = |s: f64| (s * t.nominal_clock()).round() as u64;
    let mut cmds: Vec<String> = Vec::new();
    let dout = |cmds: &mut Vec<String>, oid: u8, p: &str, value: u8, max: f64, cycle: f64| {
        cmds.push(format!("config_digital_out oid={oid} pin={} value={value} default_value={value} max_duration={}", pin(p), ticks(max)));
        if cycle > 0.0 {
            cmds.push(format!("set_digital_out_pwm_cycle oid={oid} cycle_ticks={}", ticks(cycle)));
        }
    };
    let mut adcs: Vec<(u8, &str, f64, f64)> = Vec::new();
    match t {
        Target::Dsp => {
            cmds.push(format!("allocate_oids count={}", dsp::COUNT));
            if motion {
                for s in STEPPERS.iter().filter(|s| s.target == t) {
                    cmds.push(format!(
                        "config_stepper oid={} step_pin={} dir_pin={} invert_step=0 step_pulse_ticks={}",
                        s.oid,
                        pin(s.step_pin),
                        pin(s.dir_pin),
                        ticks(machine::STEP_PULSE)
                    ));
                    dout(&mut cmds, s.enable_oid, s.enable_pin, 1, 0.0, 0.0);
                }
                for a in AXES {
                    cmds.push(format!("config_endstop oid={} pin={} pull_up=0", a.endstop_oid, pin(a.endstop_pin)));
                }
                cmds.push(format!("config_trsync oid={}", dsp::TRSYNC));
                dout(&mut cmds, dsp::BED_HEATER, "PG13", 0, MAX_HEAT_TIME, BED.pwm_cycle_time);
                dout(&mut cmds, dsp::AUX_FAN, "PG17", 0, 0.0, machine::FAN_CYCLE_TIME);
                dout(&mut cmds, dsp::CHAMBER_FAN, "PG18", 0, 0.0, machine::FAN_CYCLE_TIME);
                dout(&mut cmds, dsp::BOARD_FAN, "PG16", 0, 0.0, machine::FAN_CYCLE_TIME);
            }
            dout(&mut cmds, dsp::LIGHT, "PG15", 0, 0.0, 0.0);
            adcs.push((dsp::ADC_BED, "PB14", BED.beta, BED.max_temp));
            adcs.push((dsp::ADC_CHAMBER, "PB13", 3950.0, 125.0));
        }
        Target::Toolhead => {
            cmds.push(format!("allocate_oids count={}", toolhead::COUNT));
            if motion {
                let s = STEPPERS[3];
                cmds.push(format!(
                    "config_stepper oid={} step_pin={} dir_pin={} invert_step=0 step_pulse_ticks={}",
                    s.oid,
                    pin(s.step_pin),
                    pin(s.dir_pin),
                    ticks(machine::STEP_PULSE)
                ));
                dout(&mut cmds, s.enable_oid, s.enable_pin, 1, 0.0, 0.0);
                dout(&mut cmds, toolhead::HOTEND_HEATER, "PB6", 0, MAX_HEAT_TIME, EXTRUDER.pwm_cycle_time);
                dout(&mut cmds, toolhead::PART_FAN, "PB5", 0, 0.0, machine::FAN_CYCLE_TIME);
            }
            dout(&mut cmds, toolhead::HOTEND_FAN, "PC8", 0, 0.0, machine::FAN_CYCLE_TIME);
            adcs.push((toolhead::ADC_HOTEND, "PA3", EXTRUDER.beta, EXTRUDER.max_temp));
        }
        Target::StrainGauge => {
            // Load cells: link only for now.
            cmds.push("allocate_oids count=0".into());
        }
    }
    for (oid, p, _, _) in &adcs {
        cmds.push(format!("config_analog_in oid={oid} pin={}", pin(p)));
    }
    cmds.push("finalize_config crc=0".into());
    for c in &cmds {
        m.queue(c)?;
    }
    m.flush(3.0)?;
    let full = (machine::ADC_SAMPLES * cc_proto::clock::ADC_MAX) as f64;
    let start = now() + 0.1;
    for (i, (oid, _, beta, max_temp)) in adcs.iter().enumerate() {
        // Range check: an open/shorted thermistor shuts the MCU down.
        let (min_v, max_v) = if t == Target::Dsp && *oid == dsp::ADC_CHAMBER || !motion {
            (0, 65535)
        } else {
            ((ntc_frac(*max_temp, *beta) * full) as u32, (ntc_frac(0.0, *beta) * full) as u32)
        };
        let clock = m.clock.clock(start + i as f64 * 0.01);
        m.queue(&format!(
            "query_analog_in oid={oid} clock={} sample_ticks={} sample_count={} rest_ticks={} min_value={min_v} max_value={max_v} range_check_count={}",
            clock as u32,
            ticks(machine::ADC_SAMPLE_TIME),
            machine::ADC_SAMPLES,
            ticks(machine::ADC_REPORT_TIME),
            machine::ADC_RANGE_CHECKS
        ))?;
    }
    m.flush(2.0)?;
    Ok(())
}
