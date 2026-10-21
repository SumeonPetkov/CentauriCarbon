//! `cc-sim` — a virtual Centauri Carbon for developing the host on a PC.
//!
//! * [`mcu`]: the three MCUs (DSP, toolhead, load-cell board) speaking the
//!   printer's exact protocol with the firmware's timing rules and shutdowns.
//! * [`world`]: the machine they drive — CoreXY carriage, Z, extruder,
//!   heaters, thermistors, fans, light, stall/endstop inputs, print record.
//! * [`tmc`]: the four TMC2209 drivers on the MCUs' UARTs (microstepping and
//!   StallGuard depend on what the host configures).
//! * [`sim`]: runs it in real time on pseudo-terminals with a fake sysfs GPIO
//!   tree, so an unmodified host (ccos, or anything that opens a serial port)
//!   can connect.

pub mod hx711s;
pub mod mcu;
pub mod sim;
pub mod tmc;
pub mod world;

pub use mcu::{McuKind, VirtualMcu};
pub use sim::{Printer, Sim, SimConfig, SimPaths};
pub use world::{Fault, World, WorldConfig};
