//! The running application, independent of where pixels go: starts the
//! printer core (I/O thread) and the G-code thread, turns touches into
//! actions, and renders the screen into a [`Canvas`] reporting dirty
//! rectangles. The device binary presents them on `/dev/fb0`; `ccsim` shows
//! them in a window or drives them from a test script.

use std::sync::atomic::Ordering;
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread::JoinHandle;

use cc_ui::{Action, Canvas, Engine, Rect, Touch};

use crate::app::{Phase, State, Ui};
use crate::core::{Printer, Shared};
use crate::gcode::{Control, Job};
use crate::hw::Hardware;
use crate::ui::{self, Cmd, Flag, Value};
use crate::Target;

/// Things only the host of the runtime can do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    Exit,
    /// Turn the backlight off until the next tap.
    ScreenOff,
    Backlight(u8),
}

pub struct Runtime {
    pub ui: Ui,
    pub printer: Shared,
    pub ctl: Arc<Control>,
    jobs: Option<Sender<Job>>,
    threads: Vec<JoinHandle<()>>,
    pub engine: Engine<Cmd, Value, Flag>,
    pub canvas: Canvas,
    dirty: Vec<Rect>,
    last_screen: u8,
}

impl Runtime {
    /// Start the core and G-code threads (they connect to the MCUs).
    #[cfg(target_os = "linux")]
    pub fn start(hw: Hardware) -> Runtime {
        let ui = Ui::default();
        {
            let mut s = ui.lock();
            s.motion_lock = hw.motion_lock();
            s.gcode_dir = hw.gcode_dir.display().to_string();
        }
        let cfgs = match &hw.config_dir {
            Some(d) => crate::config::Configs::load(d),
            None => crate::config::Configs::stock(),
        };
        let printer = Printer::new(hw, cfgs, ui.clone());
        let ctl = Arc::new(Control::default());
        let (tx, rx) = std::sync::mpsc::channel();
        let p1 = printer.clone();
        let io = std::thread::Builder::new().name("ccos-io".into()).stack_size(256 * 1024).spawn(move || p1.run_io()).expect("spawn io thread");
        let (p2, c2) = (printer.clone(), ctl.clone());
        let gc = std::thread::Builder::new()
            .name("ccos-gcode".into())
            .stack_size(512 * 1024)
            .spawn(move || crate::gcode::run(p2, c2, rx))
            .expect("spawn gcode thread");
        Runtime {
            ui,
            printer,
            ctl,
            jobs: Some(tx),
            threads: vec![io, gc],
            engine: Engine::new(&ui::SCREENS, ui::START),
            canvas: Canvas::new(480, 272),
            dirty: Vec::with_capacity(16),
            last_screen: ui::START,
        }
    }

    pub fn send(&self, job: Job) {
        if let Some(j) = &self.jobs {
            let _ = j.send(job);
        }
    }

    /// Rectangles redrawn by the last [`frame`](Self::frame).
    pub fn dirty(&self) -> &[Rect] {
        &self.dirty
    }

    /// One UI step: apply a touch sample, redraw what changed (see
    /// [`dirty`](Self::dirty)). Returns an event for the host.
    pub fn frame(&mut self, touch: Touch) -> Option<Event> {
        let action = {
            let s = self.ui.lock();
            let (a, d) = self.engine.update(&*s, touch, &mut self.canvas);
            self.dirty.clear();
            self.dirty.extend_from_slice(d.rects());
            a
        };
        if self.engine.current() != self.last_screen {
            self.last_screen = self.engine.current();
            if self.engine.screen().name == "print" {
                self.send(Job::RefreshFiles);
            }
        }
        action.and_then(|a| self.act(a))
    }

    fn act(&mut self, a: Action<Cmd>) -> Option<Event> {
        match a {
            Action::Gcode(g) => {
                if self.busy_printing() {
                    self.ui.set_status("Busy printing");
                } else {
                    self.send(Job::Gcode(g.to_string()));
                }
            }
            Action::Cmd(c) => return self.command(c),
            Action::Goto(_) | Action::Back => {}
        }
        None
    }

    fn busy_printing(&self) -> bool {
        matches!(self.ui.lock().print.phase, Phase::Printing | Phase::Paused)
    }

    /// Run a screen command (also used by test scripts).
    pub fn command(&mut self, c: Cmd) -> Option<Event> {
        match c {
            Cmd::Probe => self.send(Job::Probe),
            Cmd::ResetToolhead => self.send(Job::Reset(Target::Toolhead)),
            Cmd::Restart => {
                self.ctl.cancel.store(true, Ordering::Relaxed);
                {
                    let mut s = self.ui.lock();
                    s.links = Default::default();
                    s.shutdown = None;
                    s.set_status("Restarting…");
                }
                self.send(Job::Restart);
            }
            Cmd::ToggleLight => {
                // the stock UI's "illumination light" switch
                let on = self.printer.lock().leds[1] > 0.0;
                let v = if on { 0 } else { 1 };
                self.send(Job::Gcode(format!("SET_LED_led2 RED={v} GREEN={v} BLUE={v} WHITE={v} TRANSMIT=1")));
            }
            Cmd::BacklightUp | Cmd::BacklightDown => {
                let mut s = self.ui.lock();
                s.backlight = if c == Cmd::BacklightUp { s.backlight.saturating_add(32) } else { s.backlight.saturating_sub(32).max(16) };
                return Some(Event::Backlight(s.backlight));
            }
            Cmd::ScreenOff => return Some(Event::ScreenOff),
            Cmd::Exit => return Some(Event::Exit),
            Cmd::PrintFile1 | Cmd::PrintFile2 | Cmd::PrintFile3 | Cmd::PrintFile4 | Cmd::PrintFile5 => {
                let i = match c {
                    Cmd::PrintFile1 => 0,
                    Cmd::PrintFile2 => 1,
                    Cmd::PrintFile3 => 2,
                    Cmd::PrintFile4 => 3,
                    _ => 4,
                };
                self.print_index(i);
            }
            // the stock UI's commands
            Cmd::Pause => self.send(Job::Gcode("PAUSE".into())),
            Cmd::Resume => self.send(Job::Gcode("RESUME".into())),
            Cmd::Cancel => {
                // highest priority: ends waits (M109…), then CANCEL_PRINT runs
                self.ctl.cancel.store(true, Ordering::Relaxed);
                self.send(Job::Gcode("CANCEL_PRINT".into()));
            }
            Cmd::RefreshFiles => self.send(Job::RefreshFiles),
        }
        None
    }

    /// Start printing the `i`-th listed file.
    pub fn print_index(&mut self, i: usize) {
        if self.busy_printing() {
            return;
        }
        let file = self.ui.lock().files.get(i).cloned();
        if let Some(f) = file {
            let dir = self.printer.lock().hw.gcode_dir.clone();
            self.print_path(dir.join(f));
        }
    }

    pub fn print_path(&mut self, path: std::path::PathBuf) {
        {
            let mut s = self.ui.lock();
            s.print.phase = Phase::Printing;
            s.print.file = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
            s.print.progress = 0.0;
            s.print.started = None;
            s.print.layer = 0;
            s.print.total_layers = 0;
        }
        self.send(Job::Print(path));
    }

    /// Snapshot of the screen state.
    pub fn state(&self) -> std::sync::MutexGuard<'_, State> {
        self.ui.lock()
    }

    /// Stop the threads (closes MCU links; frees the DSP ring buffer).
    pub fn stop(&mut self) {
        self.ctl.cancel.store(true, Ordering::Relaxed);
        self.jobs = None;
        {
            let mut c = self.printer.lock();
            c.exit = true;
            for h in 0..2 {
                let _ = c.set_heater(h, 0.0);
            }
            c.wake();
        }
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
        let mut c = self.printer.lock();
        for t in Target::ALL {
            c.mcus[t as usize] = None;
        }
    }
}

impl Drop for Runtime {
    fn drop(&mut self) {
        self.stop();
    }
}
