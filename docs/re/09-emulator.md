# 09 — `ccsim`: a dev emulator of the printer

`ccsim` runs the real `ccos` — UI, printer core, G-code thread, the same
protocol code and step generator that ship to the printer — against a virtual
Centauri Carbon on your PC. Use it to design screens, develop printing
features and run regression tests without risking the machine.

```sh
cd ccos
cargo run --release -p ccsim                    # window: LCD (click = touch) + bed view
cargo run --release -p ccsim -- --speed 10      # heaters 10x faster
cargo run --release -p ccsim -- sim/scripts/03_print_cube.ccsim   # a test script, headless
cargo test --release -p ccsim                   # every script in sim/scripts/ (~3 min)
```

## What is simulated

```
           ccos (unchanged)                               cc-sim
 ┌──────────────────────────────────────┐   ┌─────────────────────────────────────────┐
 │ UI ── Runtime ── G-code thread        │   │  VirtualMcu "dsp"      ┐                │
 │            │           │              │   │  VirtualMcu "toolhead" ├─ World         │
 │            └── Core (I/O thread) ─────┼pty┤  VirtualMcu "loadcell" ┘  (physics)     │
 │                  │  planner, steps,   │   │                                         │
 │                  │  heaters, homing   │   │  fake sysfs GPIO (power pins)           │
 └──────────────────┼───────────────────┘   └─────────────────────────────────────────┘
        window / script replaces /dev/fb0 + touch
```

**MCUs** (`crates/cc-sim/src/mcu.rs`). Three virtual Klipper MCUs on
pseudo-terminals (`<dir>/dsp`, `<dir>/ttyACM0`, `<dir>/ttyS4`) that speak the
printer's fixed 115-message table and follow Elegoo's firmware sources in this
repository line by line for everything a host uses to print:

| Feature | Behaviour taken from |
|---|---|
| Framing, sequence numbers, NAK on wrong sequence, ack per block | `mcu/src/command.c` |
| `queue_step` (`interval`/`count`/`add`), `set_next_step_dir`, `reset_step_clock`, `stepper_get_position` (biased position, sign flips), step pulse spacing | `dsp/.../printer/stepper.c` |
| `trsync_*`, `stepper_stop_on_trigger`, `endstop_home` oversampling | `trsync.c`, `endstop.c` |
| Soft-PWM `queue_digital_out`/`update_digital_out` with `max_duration` | `hal_call/gpiocmds.c` |
| `query_analog_in` sums, range check | `hal_call/adccmds.c` (DSP: only PB13–PB15 are ADC pins) |
| 1024-entry move pool shared by steps and output events | `basecmd.c` |
| `get_config`, `allocate_oids`, `finalize_config`, `emergency_stop`, `clear_shutdown`, `reset` | `basecmd.c`, `armcm_reset.c` |
| STM32 bootloader after power-up: prints `C`, waits for `a` | `mcu/bootloader/.../ymodem.c` |

and the same shutdowns: *Timer too close*, *Stepper too far in past*, *Move
queue overflow*, *Missed scheduling of next digital out event*, *Scheduled
digital out event will exceed max_duration*, *ADC out of range*, *Can't reset
time when stepper active*, … After a shutdown the MCU answers other commands
with `is_shutdown` (the STM32 rule; Elegoo's DSP build skips that check —
`command.c` "gg-7.4" — the simulator applies it to all three).

The DSP is a serial port in the simulator; on the printer it is the
shared-memory ring (`cc-transport/src/dsp.rs`), which the emulator does not
cover. The toolhead link is throttled to 100 kB/s and the load-cell board to
115 200 baud.

**Machine** (`crates/cc-sim/src/world.rs`), wired exactly as `printer.cfg`:

* CoreXY belts from the A/B step pins (`a = x + y`, `b = x − y`), Z and the
  extruder, with the stock step distances and `!` direction inversions;
  steps to a disabled driver are lost (and counted).
* Frame limits: pushing past them slips the belt (position is lost, as on the
  real machine); the Z endstop `PG8` (a separate sensor, not the load
  cells) closes 4.5 mm before the nozzle reaches the bed, which is
  `position_endstop_extra` in `printer.cfg`.
* Four TMC2209 drivers (`crates/cc-sim/src/tmc.rs`) on the MCUs' UARTs
  (`tmcuart.c`: one transfer at a time, a busy UART drops the request,
  a read that gets no start bit within 64 bits returns nothing).
  Each driver has these traits:
  - its node address from the MS1/MS2 pins (X 0, Y 3, Z 1, E 3);
  - the datasheet power-on registers, access types, IFCNT and SENDDELAY;
  - microstepping from those same pins (1/8, 1/32 or 1/16) until the
    host sets `mstep_reg_select`, so an unconfigured X moves twice as
    far per step;
  - a DIAG output (`!PG4`, `!PF6`) that asserts on a stall only in
    StealthChop, with SGTHRS > 0, TOFF > 0 and TCOOLTHRS ≥ TSTEP >
    TPWMTHRS. TSTEP comes from the actual step rate.
* The load cells (`crates/cc-sim/src/hx711s.rs`).
  - The bed sits on them and gives 0.5 mm (`BED_GIVE`) before Z slips.
    `push_z` counts the press.
  - The bed is flat unless a script sets `option bed_shape <tilt x>
    <tilt y> <dome>`: the surface rises by `tilt x` mm from X0 to X256 and
    by `tilt y` from Y0 to Y256, and the centre sits `dome` mm above the
    edges. The Z limits and PG8 are measured at the bed centre. `gap` is
    the gap under the nozzle, which is what the load cells feel and where
    filament is laid.
  - The bridge reads 251000 counts unloaded, minus 60000 counts per mm
    pressed, with 40-count noise.
  - The board firmware (`hx711s.c` and its helpers) is ported for the
    board as strapped: an HX717 half bridge on line 2 at 320 SPS. The
    task runs every 1.5 ms.
  - The port covers tare and the bad-cell bits, the sliding-window
    filter, the trigger rules, the knee search, and `sg_resp` with its
    heartbeat and repeat limits. The firmware's never-reset statics are
    kept.
  - `dirzctl.c` on the DSP toggles the Z step pin from its own timer.
    Its reports carry 16-bit step counts, as on the printer.
* Heaters as lumped thermal masses (hotend ≈ 60 W, bed ≈ 350 W, chamber
  coupled to the bed; the part fan cools the hotend), NTC 100K thermistors
  (β 4300 / 3950) on 4.7 kΩ pull-ups into 12-bit ADCs with noise.
* Outputs: part/hotend/aux/chamber/board fans, chamber light.
* A 2 px/mm height map of everything extruded through a hot nozzle, layers,
  bounding box, cold extrusion, retraction. Filament pushed into the hot
  nozzle leaves it with a 40 ms lag (first-order melt flow,
  `MELT_TAU`), so a pressure-advance pulse is not deposited the instant it
  is fed, and a line ends a little after the feed stops, as on a real
  nozzle. Each virtual MCU applies its steps in bursts, so the extruder
  and the carriage arrive out of order; melt is laid down 50 ms behind,
  in time order, where the nozzle was at that moment.

**Faults** to inject: `hotend_heater_dead`, `bed_heater_dead`,
`hotend_thermistor_open`, `hotend_thermistor_short`, `bed_thermistor_open`,
`hotend_thermistor_loose`, `toolhead_unplugged`, `x_diag_dead`; and
`--corrupt <p>` flips bits in bytes the host sends.

**Power cuts** (`Emu::power_cut` / `power_on`; script `power_cut`,
`power_on`):
- A cut raises the A7's power-fail pin (gpio192) and takes power from the
  three MCUs. Motors stop where they are, heaters go cold, and what was
  printed stays on the bed.
- ccos runs on until it has written its power-loss save (at most 1.5 s),
  then stops, as the A7 does when its capacitor runs out.
- Power on brings the MCUs back in their bootloaders and boots a fresh
  ccos.

## ccos changes that made this possible

The emulator needed — and drove — the printing core:

* `hw.rs`: where the MCUs are (`CCOS_SIM`, `CCOS_*_PORT`, `CCOS_GPIO_ROOT`),
  power-cycling through any sysfs GPIO root, and the **motion lock**: on real
  hardware heaters and motors stay off unless `CCOS_ENABLE_MOTION=1`.
* `mcu.rs`: non-blocking session à la `serialqueue.c` — frames packed with
  several messages, 12-frame/192-byte window, RTT-based retransmit, NAK
  handling, adopting the MCU's sequence number on first contact, messages
  held until a move-queue slot frees (`min_clock`), clock sync à la
  `clocksync.py`.
* `motion/planner.rs`: Klipper's look-ahead (junction deviation, smoothed
  acceleration); `motion/itersolve.rs`: the stock step-time solver with
  input shaping and pressure advance; `motion/stepper.rs`: `queue_step`
  compression within 25 µs.
* `heater.rs`: the stock PID (1 s derivative smoothing), `verify_heater`
  (including the < −20 °C thermistor check), autotune; heater outputs with
  `max_duration` 5 s; ADC range limits from `min_temp`/`max_temp`.
* `core.rs`, `gcode/`: I/O thread + G-code thread, homing through
  trsync/endstop/`stepper_stop_on_trigger`. The G-code interpreter is a
  port of the stock one: parser, commands, scripts, file pipeline, M8xxx.
  See [10](10-gcode.md).
* `config.rs`: the stock ConfigParser; `printer.cfg`, `user_printer.cfg`
  and `unmodifiable.cfg` are read from `CCOS_CONFIG_DIR` (the emulator
  seeds `<dir>/board-resource` with the stock files).
* `runtime.rs`: the UI loop independent of `/dev/fb0`, shared by the device
  and the emulator. New Print screen (`ui/screens/print.toml`).

## In Docker

`ccos/sim/docker/` runs ccsim in a container and serves its window to a web
browser: Xvfb, then x11vnc, then noVNC on port 6080. It needs nothing on the
host but Docker; see its README:
`docker compose -f ccos/sim/docker/compose.yaml up --build`, then open
<http://localhost:6080>.

The page also has a G-code console and a file panel. Both use
`ccsim --control <addr>` (`crates/ccsim/src/control.rs`), a small HTTP server
in the emulator's interactive mode:
- `GET /console` returns ccos's console ring.
- `POST /gcode` queues lines as a `Job::Gcode`; each line is echoed into the
  console as `> line`.
- `GET /files` lists the G-code files.
- `POST /upload|print|delete?name=` manage files. A print starts the way the
  Print screen starts one, and is refused while a print runs.
- `GET /power` and `POST /power?on=0|1` cut or restore mains (the page's
  **Power cut / Power on** button). G-code and prints are refused while
  the printer is off.

It is emulator-only and needs no changes to ccos.

## Test scripts

`ccos/sim/scripts/*.ccsim` (syntax in `crates/ccsim/src/script.rs`):

| Script | Checks |
|---|---|
| `01_connect_ui` | power-cycle + bootloader release, live temperatures, taps, light |
| `02_heat_home` | PID heating, hotend fan at 50 °C, the stock G28: X/Y twice on stall, Z twice on PG8 at (130, 130), ends at Z 14.35; `G28 X Y` ends at X18 Y17.5; machine and host agree; moves past a limit are clamped |
| `03_print_cube` | prints `cube_20mm.gcode` from the Print screen: CUSTOM_ZERO instead of the first G28, `;LAYER:` counting, 6 layers 0.15 mm above commanded (no bed mesh), bounding box, no cold extrusion, no lost steps, end G-code turns everything off |
| `04_faults` | open thermistor → host *thermistor fault* (stock `min_temp` is −200, so the MCU range check cannot fire); dead heater → *not heating at expected rate*; USB unplugged → MCU *Missed scheduling…* then host *no response*; restart after each; unhomed move silently dropped; out-of-range target only warned |
| `05_stress` | G2 arcs and a 300 mm/s zig-zag while 1 in 2000 bytes are corrupted: retransmits happen, nothing is lost or late |
| `07_tmc` | TMC2209 register images after connect, SET_TMC_CURRENT, INIT_TMC/SET_TMC_FIELD answered by the extruder driver, StallGuard on/off around homing, a broken X DIAG: X fails, G28 goes on with Y |
| `08_loadcell` | `STRAINGAUGE_Z_HOME` (tare flags 0x8B, Z0 with the bed pressed ~0.1 mm), `STRAINGAUGE_TEST` back where it started, `DIRZCTL` moves Z without changing ccos's position, `READ_HX711` returns |
| `09_loadcell_home` | G28 with `option config strain_gauge enable_z_home 1`: Z homed on the load cells, Z10 is 5.65 mm above the bed |
| `10_bed_mesh` | a tilted, domed bed (`option bed_shape`) on a 3×3 grid: bad `BED_MESH_CALIBRATE` words refused before moving; the stock calibration saved as `default` and in use; Z1 at the same gap at five places; `BED_MESH_CALIBRATE S60 pla` heats first and saves `pla`; the Bed mesh screen and its arrows; `BED_MESH_CLEAR` shows the slope again; `G29 S65` |
| `12_probe` | PROBE, PROBE_ACCURACY/HOME_ACCURACY (Z0 first, σ 0 as stock), PRTOUCH_READY's corners reading 0 (taps from Z11 never reach the bed, as on the printer), CHECK_BED_MESH without and with a mesh, MEASURE_GAP_TEST (refused INDEX, gaps 0), LIFT_HOT_BED stopping 5 mm back after the press, PROBE_CALIBRATE, CALIBRATE_Z_OFFSET |
| `11_power_loss` | a power cut at layer 3: the save is written; at power on the "Kind Reminder" prompt; Resume: STOP_RESUME's lift, X/Y homing and purge at the pause position (~20 mm up, as stock), the save deleted, the cube finished; again with Cancel: the save deleted; a cut with no print saves nothing |
| `06_gcode_parity` | stock parser/console behaviour (unknown commands, M21 silent, range error text, upper-casing), M106 P/S rules, SET_LED TRANSMIT, SET_PRESSURE_ADVANCE/M900 and `kin_flush_delay`, SET_INPUT_SHAPER (SHAPER_TYPE_X ignored as in the stock code), the ROOT gate in a file, PAUSE/RESUME/CANCEL_PRINT during a print |

Example:

```text
option speed 20
wait_until links == 3 timeout 20
goto print
tap "cube_20mm.gcode"
wait_until print_state == "Finished" timeout 180
expect layers == 6
expect cold_extrusion == 0
bedshot shots/cube_bed.png
```

## Limits

* Timing is real time on a PC: it shows protocol/scheduling bugs, not whether
  the A7 is fast enough — measure that on the printer.
* The TMC2209 model has no current or load physics: StallGuard is ideal
  once enabled (no false or late stall detection), and the run current
  changes nothing.
* The bed is rigid apart from its linear give, and its shape is a plane
  plus a dome. Moving X/Y at a small gap does not check for a crash into a
  raised part of the bed. The load cells see
  no vibration, heat drift or nozzle ooze, so every tap is clean. The
  board's shake filter and the "other trigger" rules are therefore never
  tested by real noise.
* No resonance model: input shaping changes the step times ccos sends,
  but the emulated machine has no ringing to cancel.
* `layers` counts a height once 0.05 mm of filament went there: pressure
  advance and the melt lag let a trace out while Z is still moving.
* The melt model is one time constant, not a pressure model: it will not
  show whether a pressure-advance value is right for a filament.
* The `console` probe reads a process-wide ring of the last 256 console
  lines; when scripts run in parallel (`cargo test`), use text unique to
  the script.
* The DSP's shared-memory transport is not emulated (the DSP is a pty).
