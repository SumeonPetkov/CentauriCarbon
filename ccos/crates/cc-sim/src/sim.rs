//! Real-time runner: the virtual MCUs on pseudo-terminals, a fake sysfs GPIO
//! tree for the STM32 power pins, and the world, advanced together.
//!
//! Layout of the simulation directory (default `/tmp/ccsim`):
//!
//! ```text
//! dsp            -> /dev/pts/N   (the DSP; on the printer this is a shared-memory ring)
//! ttyACM0        -> /dev/pts/N   (toolhead, "1 Mbaud")
//! ttyS4          -> /dev/pts/N   (load-cell board, 115200 baud, throttled like the UART)
//! gpio/export
//! gpio/gpio140/{direction,value} (PE12, toolhead power)
//! gpio/gpio201/{direction,value} (PG9, load-cell board power)
//! ```

use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use crate::mcu::{McuKind, VirtualMcu};
use crate::world::{Fault, World, WorldConfig};

#[derive(Debug, Clone)]
pub struct SimConfig {
    pub dir: PathBuf,
    pub world: WorldConfig,
    /// STM32 firmware already running at start (as after `killall app`);
    /// otherwise they wait in the bootloader like after power-on.
    pub mcus_running: bool,
    /// Probability that a byte from the host is corrupted on the wire.
    pub corrupt_rate: f64,
    /// Print MCU/world events to stderr.
    pub verbose: bool,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self { dir: PathBuf::from("/tmp/ccsim"), world: WorldConfig::default(), mcus_running: true, corrupt_rate: 0.0, verbose: false }
    }
}

#[derive(Debug, Clone)]
pub struct SimPaths {
    pub dir: PathBuf,
    pub dsp: PathBuf,
    pub toolhead: PathBuf,
    pub loadcell: PathBuf,
    pub gpio_root: PathBuf,
}

/// Everything simulated; lock it to inspect or inject faults.
pub struct Printer {
    pub world: World,
    pub mcus: [VirtualMcu; 3],
    t0: Instant,
    /// Index into `world.log` already printed (verbose mode).
    printed: usize,
}

impl Printer {
    /// Simulation time (seconds since start).
    pub fn now(&self) -> f64 {
        self.t0.elapsed().as_secs_f64()
    }

    pub fn mcu(&self, k: McuKind) -> &VirtualMcu {
        &self.mcus[k as usize]
    }

    pub fn inject(&mut self, f: Fault, on: bool) {
        let t = self.now();
        if on {
            self.world.faults.insert(f);
        } else {
            self.world.faults.remove(&f);
        }
        self.world.note_event(t, format!("fault {} {}", f.name(), if on { "injected" } else { "cleared" }));
    }

    /// First MCU shutdown reason, if any.
    pub fn shutdown(&self) -> Option<(McuKind, &'static str)> {
        self.mcus.iter().find_map(|m| m.is_shutdown().map(|s| (m.kind, s)))
    }
}

struct Pty {
    master: OwnedFd,
    _slave: OwnedFd,
}

fn open_pty(link: &Path) -> io::Result<Pty> {
    unsafe {
        let m = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC);
        if m < 0 {
            return Err(io::Error::last_os_error());
        }
        let master = OwnedFd::from_raw_fd(m);
        if libc::grantpt(m) != 0 || libc::unlockpt(m) != 0 {
            return Err(io::Error::last_os_error());
        }
        let name = std::ffi::CStr::from_ptr(libc::ptsname(m)).to_owned();
        let s = libc::open(name.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC);
        if s < 0 {
            return Err(io::Error::last_os_error());
        }
        // Keep a slave fd open so the master never sees a hang-up between
        // host connections; raw mode so nothing is echoed or translated.
        let slave = OwnedFd::from_raw_fd(s);
        let mut t: libc::termios = std::mem::zeroed();
        libc::tcgetattr(s, &mut t);
        libc::cfmakeraw(&mut t);
        libc::tcsetattr(s, libc::TCSANOW, &t);
        let _ = std::fs::remove_file(link);
        std::os::unix::fs::symlink(name.to_str().unwrap(), link)?;
        Ok(Pty { master, _slave: slave })
    }
}

/// A link with the byte rate of the real one.
struct Wire {
    pty: Pty,
    bytes_per_s: f64,
    credit: f64,
}

/// sysfs GPIO number → board it powers.
const POWER: [(u32, McuKind); 2] = [(4 * 32 + 12, McuKind::Toolhead), (6 * 32 + 9, McuKind::LoadCell)];

pub struct Sim {
    pub printer: Arc<Mutex<Printer>>,
    pub paths: SimPaths,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Sim {
    /// Create the ptys and GPIO tree and start the simulation thread.
    pub fn start(cfg: SimConfig) -> io::Result<Sim> {
        let dir = cfg.dir.clone();
        std::fs::create_dir_all(&dir)?;
        let paths = SimPaths {
            dsp: dir.join("dsp"),
            toolhead: dir.join("ttyACM0"),
            loadcell: dir.join("ttyS4"),
            gpio_root: dir.join("gpio"),
            dir: dir.clone(),
        };
        std::fs::create_dir_all(&paths.gpio_root)?;
        std::fs::write(paths.gpio_root.join("export"), "")?;
        std::fs::write(paths.gpio_root.join("unexport"), "")?;
        for (n, _) in POWER {
            let g = paths.gpio_root.join(format!("gpio{n}"));
            std::fs::create_dir_all(&g)?;
            std::fs::write(g.join("direction"), "out\n")?;
            std::fs::write(g.join("value"), "1\n")?;
        }
        let wires = [
            Wire { pty: open_pty(&paths.dsp)?, bytes_per_s: f64::INFINITY, credit: 0.0 },
            Wire { pty: open_pty(&paths.toolhead)?, bytes_per_s: 1_000_000.0 / 10.0, credit: 0.0 },
            Wire { pty: open_pty(&paths.loadcell)?, bytes_per_s: 115_200.0 / 10.0, credit: 0.0 },
        ];
        let t0 = Instant::now();
        let mut world = World::new(cfg.world.clone());
        let mut mcus = McuKind::ALL.map(|k| VirtualMcu::new(k, 0.0));
        if !cfg.mcus_running {
            for m in mcus.iter_mut().filter(|m| m.kind.has_bootloader()) {
                m.set_power(false, 0.0, &mut world);
                m.set_power(true, 0.0, &mut world);
            }
        }
        let printer = Arc::new(Mutex::new(Printer { world, mcus, t0, printed: 0 }));
        let stop = Arc::new(AtomicBool::new(false));
        let (p2, s2, gpio, c2) = (printer.clone(), stop.clone(), paths.gpio_root.clone(), cfg.clone());
        let thread = std::thread::Builder::new().name("ccsim".into()).spawn(move || run(p2, s2, wires, gpio, c2))?;
        Ok(Sim { printer, paths, stop, thread: Some(thread) })
    }

    pub fn lock(&self) -> MutexGuard<'_, Printer> {
        self.printer.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Environment that points `ccos` at this simulation.
    pub fn env(&self) -> Vec<(String, String)> {
        let p = &self.paths;
        vec![
            ("CCOS_DSP_PORT".into(), p.dsp.display().to_string()),
            ("CCOS_TOOLHEAD_PORT".into(), p.toolhead.display().to_string()),
            ("CCOS_LOADCELL_PORT".into(), p.loadcell.display().to_string()),
            ("CCOS_GPIO_ROOT".into(), p.gpio_root.display().to_string()),
        ]
    }
}

impl Drop for Sim {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
        for p in [&self.paths.dsp, &self.paths.toolhead, &self.paths.loadcell] {
            let _ = std::fs::remove_file(p);
        }
    }
}

fn read_power(gpio: &Path, n: u32) -> Option<bool> {
    let s = std::fs::read_to_string(gpio.join(format!("gpio{n}/value"))).ok()?;
    match s.trim() {
        "0" => Some(false),
        "1" => Some(true),
        _ => None,
    }
}

fn run(printer: Arc<Mutex<Printer>>, stop: Arc<AtomicBool>, mut wires: [Wire; 3], gpio: PathBuf, cfg: SimConfig) {
    let mut buf = vec![0u8; 8192];
    let mut last_t = 0.0f64;
    let mut gpio_at = 0.0f64;
    let mut power = [true; 2];
    let mut rng = 0x2545_f491_4f6c_dd1du64;
    while !stop.load(Ordering::Relaxed) {
        let mut pfds: Vec<libc::pollfd> =
            wires.iter().map(|w| libc::pollfd { fd: w.pty.master.as_raw_fd(), events: libc::POLLIN, revents: 0 }).collect();
        unsafe { libc::poll(pfds.as_mut_ptr(), pfds.len() as _, 1) };

        let mut p = printer.lock().unwrap_or_else(|e| e.into_inner());
        let t = p.now();
        let dt = t - last_t;
        last_t = t;
        let Printer { world, mcus, .. } = &mut *p;

        if t >= gpio_at {
            gpio_at = t + 0.01;
            for (i, (n, k)) in POWER.iter().enumerate() {
                if let Some(on) = read_power(&gpio, *n) {
                    if on != power[i] {
                        power[i] = on;
                        mcus[*k as usize].set_power(on, t, world);
                    }
                }
            }
        }

        for (i, w) in wires.iter_mut().enumerate() {
            let unplugged = i == McuKind::Toolhead as usize && world.faults.contains(&Fault::ToolheadUnplugged);
            w.credit = (w.credit + dt * w.bytes_per_s).min(buf.len() as f64);
            let want = if w.bytes_per_s.is_finite() { w.credit as usize } else { buf.len() };
            if want > 0 {
                let n = unsafe { libc::read(w.pty.master.as_raw_fd(), buf.as_mut_ptr() as _, want) };
                if n > 0 {
                    let n = n as usize;
                    if w.bytes_per_s.is_finite() {
                        w.credit -= n as f64;
                    }
                    if cfg.corrupt_rate > 0.0 {
                        for b in &mut buf[..n] {
                            rng ^= rng << 13;
                            rng ^= rng >> 7;
                            rng ^= rng << 17;
                            if (rng >> 11) as f64 / (1u64 << 53) as f64 <= cfg.corrupt_rate {
                                *b ^= 0x20;
                            }
                        }
                    }
                    if !unplugged {
                        mcus[i].receive(&buf[..n], t, world);
                    }
                }
            }
            mcus[i].advance(t, world);
            let out = mcus[i].take_output();
            if !out.is_empty() && !unplugged {
                // A full pty buffer means the host is not reading: drop, like a UART overrun.
                unsafe { libc::write(w.pty.master.as_raw_fd(), out.as_ptr() as _, out.len()) };
            }
        }
        world.advance_thermal(t);

        if cfg.verbose {
            let start = p.printed.min(p.world.log.len());
            for (t, m) in &p.world.log[start..] {
                eprintln!("[sim {t:9.3}] {m}");
            }
            p.printed = p.world.log.len();
        }
        drop(p);
        if dt > 0.05 {
            // The process was descheduled; MCU timers caught up in one go.
            std::thread::sleep(Duration::from_micros(100));
        }
    }
    // Leave the printer cold.
    let mut p = printer.lock().unwrap_or_else(|e| e.into_inner());
    p.world.hotend.duty = 0.0;
    p.world.bed.duty = 0.0;
}
