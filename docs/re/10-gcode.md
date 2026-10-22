# 10 — The ccos G-code interpreter: parity with the stock firmware

ccos's G-code interpreter (`ccos/crates/ccos/src/gcode/`) is a port of the
stock firmware's C++ klippy (`firmware/core/klippy/`). It aims to behave like
it exactly: the same parser, the same command set as the Centauri Carbon
actually loads, the same parameters, defaults and range checks, and the same
G-code scripts that Elegoo's commands run internally. Where the C++ has
undefined behaviour, ccos does what was evidently intended; every such case is
listed under [Deliberate differences](#deliberate-differences).

| File | Ported from |
|---|---|
| `gcode/parse.rs` | `gcode.cpp` (`process_command`, `get_int/get_double` and their errors) |
| `gcode/mod.rs` | `gcode.cpp` dispatch, `idle_timeout.cpp`, `heaters.cpp` M105/TURN_OFF_HEATERS |
| `gcode/cmds.rs` | `gcode_move.cpp`, `toolhead.cpp`, `extruder.cpp`, `heaters.cpp`, `pid_calibrate.cpp`, `fan*.cpp`, `led.cpp`, `stepper_enable.cpp`, `arcs.cpp` |
| `gcode/home.rs` | `homing.cpp`, `safe_z_home.cpp`, `kinematics/corexy.cpp` (homing), `mcu_io.cpp` (trsync/endstop), `chelper/trdispatch.c` (in `core.rs`) |
| `motion/trapq.rs`, `itersolve.rs`, `shaper.rs`; `core.rs` (flush timing) | `chelper/trapq.c`, `itersolve.c`, `kin_corexy.c`, `kin_shaper.c`, `kin_extruder.c`; `toolhead.cpp`, `extras/input_shaper.cpp`, `kinematics/extruder.cpp` |
| `tmc.rs` + `gcode/tmc.rs` | `tmc.cpp`, `tmc2208.cpp`, `tmc2209.cpp`, `tmc_uart.cpp`, `serialhdl.cpp` (`send_with_response`) |
| `gcode/print.rs` | `virtual_sdcard.cpp`, `print_stats.cpp`, `pause_resume.cpp`, `change_filament.cpp`, `auto_leveling.cpp` (M729, WIPE_NOZZLE) |
| `mesh.rs` + `gcode/mesh.rs` | `extras/bed_mesh.cpp`, `probe.cpp` (`ProbePointsHelper`, `PrinterProbe::run_probe`), `auto_leveling.cpp` (G29, G29.1), `strain_gauge.cpp` (`run_G29_Z`) |
| `gcode/para.rs` + `para_table.rs` | `printer_para.cpp` (ROOT, UNROOT, M8000–M8826) |
| `config.rs` | `configfile.cpp` (ConfigParser, `WriteIni`, `WriteI_specified_Ini`) |

`para_table.rs` is generated: `python ccos/tools/m8xxx.py
firmware/core/klippy/printer_para.cpp firmware/core/allwinner/Define_reference.h
ccos/crates/ccos/src/gcode/para_table.rs <other.txt>`. Defaults come from
`Define_reference.h`, the header `printer_para.cpp` really includes, not
`Define_default.h`.

## Parser

Exactly `gcode.cpp`:

- Only spaces are stripped. Tabs stay in the line.
- **Traditional lines** (a letter followed by a digit, e.g. `G1 X10`): the
  whole line is upper-cased and split into letter/value pairs. Each letter,
  `_`, `*` and `/` starts a new word. Comments are **not** removed: in
  `G1 X10 ;move`, `;MOVE` is parsed as more words. A trailing letter without
  a value gets the value `" "`. A leading `N<n>` is skipped.
- **Extended lines** (`SET_FAN_SPEED I=1 S=0.5`): case-sensitive. They are
  split on spaces but respect quotes. A token that starts with `;`, `#` or
  `*` ends the line. A bare token (`FOO` without `=`) gets itself as its
  value. A quoted value is only parsed correctly when it is the last
  parameter.
- The command name used for lookup has all whitespace and NULs removed.
- Number parsing follows `std::stod`/`std::stoi`: leading numeric prefix,
  `0x` accepted for doubles, trailing junk ignored, and nothing numeric is an
  error.
- A value outside its range is not an error. ccos logs
  `error on <line>: <NAME>must have minimum of <x>` (sic, no space) and the
  default is used, like the firmware.
- An exception in a handler is logged as `error` and the next line runs. If
  a bad number is found in `G1`, the axes before it have already been
  updated.
- An unknown command logs `unknown command: X`, except M21 and M105, which
  are silent. Nothing replies `ok`.

## Command coverage

**Implemented**, same parameters/defaults/scripts as the stock firmware:

| Group | Commands |
|---|---|
| Moves | G0 G1 G2 G3 G4 G17 G18 G19 G20 G21 G90 G91 G92 M82 M83 M114 M220 M221 GET_POSITION SET_GCODE_OFFSET SAVE_GCODE_STATE RESTORE_GCODE_STATE MANUAL_MOVE |
| Toolhead | M204 M211 M222 M223 M400 SET_VELOCITY_LIMIT RESET_PRINTER_PARAM M18 M84 SET_STEPPER_ENABLE STEPPER_BUZZ |
| Homing | G28 CUSTOM_ZERO Z_AXIS_OFF_LIMIT_ACTION |
| Load cells | STRAINGAUGE_Z_HOME STRAINGAUGE_TEST DIRZCTL READ_HX711 PRTOUCH_READY CHECK_BED_MESH MEASURE_GAP_TEST NOZZLE_CLEAR |
| Probe | PROBE QUERY_PROBE PROBE_ACCURACY HOME_ACCURACY PROBE_CALIBRATE CALIBRATE_Z_OFFSET LIFT_HOT_BED |
| TMC2209 | SET_TMC_CURRENT_stepper_x/_y/_z/_extruder INIT_TMC SET_TMC_FIELD DUMP_TMC |
| Extruder | SET_PRESSURE_ADVANCE M900 SET_EXTRUDER_STEP_DISTANCE SET_MIN_EXTRUDE_TEMP FLOW_CALIBRATION ACTIVATE_EXTRUDER RESET_EXTRUDER |
| Input shaper | SET_INPUT_SHAPER M593 |
| Heaters | M104 M109 M140 M190 M105 SET_HEATER_TEMPERATURE TEMPERATURE_WAIT TURN_OFF_HEATERS M301 M130 M131 M132 PID_CALIBRATE PID_CALIBRATE_ALL PID_CALIBRATE_HOTBED PID_CALIBRATE_NOZZLE |
| Fans, LEDs | M106 M107 SET_FAN_SPEED SET_LED_led1 SET_LED_led2 |
| Print job | SDCARD_PRINT_FILE SDCARD_RESET_FILE SET_PRINT_STATS_INFO M117 PAUSE RESUME CLEAR_PAUSE CANCEL_PRINT STOP_PRINT M75 M76 M77 M601 M602 M603 |
| Power-loss resume | BREAK_SAVE_STATUS SDCARD_PRINT_FILE_BREAK_SAVE STOP_RESUME |
| Filament | M600 LOAD_FILAMENT MOVE_TO_EXTRUDE CUT_OFF_FILAMENT EXTRUDE_FILAMENT CHANGE_FILAMENT_SET_ACTIVE CHANGE_FILAMENT_SET_BUSY CHANGE_FILAMENT_SET_CHECK_MOVE_IGNORE M729 WIPE_NOZZLE |
| Bed mesh | G29 G29.1 BED_MESH_CALIBRATE BED_MESH_SET_INDEX BED_MESH_OUTPUT BED_MESH_MAP BED_MESH_OFFSET BED_MESH_APPLICATIONS BED_MESH_CLEAR |
| System | ROOT UNROOT M8000–M8826 (all 165), SET_IDLE_TIMEOUT UPDATE_IDLE_TIMER M112 RESTART FIRMWARE_RESTART ECHO STATUS HELP G M M110 M115 |

**Phase 2**: these exist on the printer, but ccos does not yet have the
subsystem behind them. They are accepted, do nothing, and log
`<CMD>: not available in ccos yet (needs …)` the first time they are used.

| Subsystem | Commands |
|---|---|
| Accelerometer | SHAPER_CALIBRATE SHAPER_CALIBRATE_SCRIPT TEST_RESONANCES MEASURE_AXES_NOISE ACCELEROMETER_MEASURE_X ACCELEROMETER_QUERY_X ADXL345_DEBUG_READ_X ADXL345_DEBUG_WRITE_X |

**Not on the Centauri Carbon.** Their klippy modules exist in the source
but are not loaded by the stock `printer.cfg`, so the printer answers
`unknown command` and ccos does the same: M73, G10/G11 (firmware retract),
EXCLUDE_OBJECT*, SAVE_VARIABLE, M119, M20–M30 (SD card codes), the Palette
O-codes, FORCE_MOVE, SET_KINEMATIC_POSITION, TUNING_TOWER, G30,
BED_MESH_PROFILE.

## Behaviour worth knowing (all as in the stock firmware)

- **Defaults.** Speed is 25 mm/s until the first F. `F0` or a negative F
  becomes 40 mm/s.
- **G92.** Without axes it does nothing; it does not scale E.
- **G4.** Only `P` (ms) is read.
- **Soft limits.** A move past an axis limit is *clamped* to the limit. A
  move on an unhomed axis is *dropped* without an error.
- **Cold extrusion** (below `min_extrude_temp` − 3 °C) is dropped, not
  aborted. E-only moves longer than 60 mm are not speed limited. Moves that
  extrude more than ratio 0.26608 are dropped.
- **M204.** An `S` above the limit is ignored. Without a valid S, P or T it
  logs `Invalid M204 command` and the line. M222, M223 and M211 are ignored
  when they come from a file.
- **Fans.** `M106 P1` (or no P) is the part fan, P2 the aux fan, P3 the box
  fan. P0 and P4+ do nothing. A missing or out-of-range S means 255. M107
  turns off the part fan only. Fan changes apply when the moves queued before
  them have finished.
- **Heaters.** A target outside min/max only warns. "Reached" means within
  2 °C. `SET_HEATER_TEMPERATURE` always sets the extruder (the firmware's
  heater lookup ignores `HEATER=`). M301/M130–M132 write the misspelt
  `[exturder]` section, then reload the extruder gains. `PID_CALIBRATE`
  writes the new gains to the config and leaves the running gains and the
  heater off.
- **G28.**
  1. If the nozzle is below 138 °C, run `M109 S140`.
  2. `Z_AXIS_OFF_LIMIT_ACTION`: if PG8 is already triggered, lift Z 10 mm.
  3. Home X, Y, X, Y (stall detection, 20 mm back-off each) with
     `M204 S5000.00` and the homing currents.
  4. Move to (130, 130) at 300 mm/s and home Z twice on PG8 (8 mm/s, then
     3 mm/s), backing off 10 mm.
  5. Z then reads `position_endstop_extra − z_offset − fix_z_offset + 10`
     (stock: 4.5 − 0 − 0.15 + 10 = 14.35).
  6. Restore the earlier extruder target.

  `G28 X Y` ends at X18 Y17.5.
- **Homing protocol** (`homing_move`). The move starts at
  `get_last_move_time()`.
  - `endstop_home` samples once per motor step: `rest_ticks` is one step's
    time at the homing speed (`calc_endstop_rate`), 4 samples of 15 µs.
  - `trsync_start` asks for a report every 0.1 s and expires 0.25 s after
    the start (`expire_reason` 2). trdispatch pushes the expiry to the
    report clock + 0.25 s, and sends `trsync_trigger` when the MCU
    reports a trigger.
  - The move begins after a 1 ms `HOMING_START_DELAY` plus the 8.6 ms
    `kin_flush_delay`. Steps are dripped 0.16–0.21 s ahead.
  - The outcome comes from `trsync_trigger reason=3`, then
    `stepper_get_position`.
- **Step generation** (`motion/trapq.rs`, `itersolve.rs`, `shaper.rs`)
  is the stock C helper code ported: committed moves go into a trapq;
  each stepper's position is a function of print time; a secant search
  finds each crossing of the middle between two steps.
  - The CoreXY A and B motors run x ± y of the **input-shaped** X and Y.
    Stock: MZV at 54.8 Hz with damping 0.06 on X, ZV at 46.4 Hz with
    damping 0.082 on Y.
  - The extruder runs smoothed **pressure advance**: 0.04 over 0.01 s.
    It applies only to moves that extrude while moving X/Y.
    `pressure_advance_smooth_time: 0.01` is below the firmware's own
    minimum of 0.02. The config reader only logs that, so 0.01 is used.
  - Steps are generated `kin_flush_delay` behind the planned moves:
    the larger shaper window, or half the smooth time, plus 8.6 ms.
    Once nothing more is queued, the last move's tail is generated and
    the next move starts at least that much later (`min_restart_time`).
- **SET_PRESSURE_ADVANCE / M900 / SET_INPUT_SHAPER.**
  - A change that alters `kin_flush_delay` flushes step generation
    first, so it causes a short pause in a print, as on the stock
    firmware.
  - `M900 K` defaults to printer.cfg's `pressure_advance`, or 0.03 if
    there is none.
  - `SET_INPUT_SHAPER` and `M593` are one command. The firmware tests
    `if (shaper_type_x = -1)`, an assignment, so `SHAPER_TYPE_X` and
    `SHAPER_TYPE_Y` never take effect; only `SHAPER_TYPE` changes the
    type.
  - The Y step-generation window is computed with the X damping ratio.
  - Neither command replies.
- **Failed homing.**
  - The axes become unhomed (`CoreXYKinematics::motor_off`), but the
    motors stay enabled.
  - It is not an error: the retract and the second pass are dropped
    (unhomed), and G28 goes on with the next axis.
  - Only a failed Z shuts the printer down ("Z-axis zeroing failed").
  - With `[strain_gauge] enable_z_home` set, G28 homes Z on the load
    cells instead (`run_G28_Z`, below). The stock config has it off.
- **Load cells** (`strain_gauge.cpp`, `hx711s.cpp`, `dirzctl.cpp`).
  - A tap (`probe_by_step`) goes like this:
    1. Tare the cells (`calibration_sample`, 30 samples) and arm the
       board (`sg_probe_check cmd=1`).
    2. The DSP steps Z down on its own (`run_dirzctl`) at the probe
       speed. This bypasses the motion queue, so ccos's Z does not
       change.
    3. The board reports the press (`sg_resp`) with the tick of its
       knee.
    4. Z at that tick is interpolated between the two `result_dirzctl`
       reports (first step, stop).
    5. With "up after", the same number of steps go back up, at 3× the
       speed.
  - `STRAINGAUGE_Z_HOME` taps:
    1. Once at 6 mm/s from where Z is; that point becomes Z0 (homed).
    2. It goes up to Z2 and taps twice at 3 mm/s (each followed by going
       back up).
    3. A third tap at 3 mm/s stays down, and that point becomes Z0.
    The nozzle is left pressing on the bed.
  - G28 with `enable_z_home` (`run_G28_Z`):
    1. If the nozzle is above 145 °C, cool it to 140 °C with the part
       fan at 255.
    2. Shake Z 20 times by `shake_range` (0 in the stock config).
    3. Tare, then one fast tap that becomes Z0, then go up to Z2.
    4. Tap up to 4 mm at 3 mm/s until two taps in a row agree within
       `g28_max_err` and are within 1 mm. The speed halves (down to
       1 mm/s) after each miss. There are 10 taps per round and 3 rounds.
    5. The touch point becomes `position_endstop` (so stock Z10 is
       5.65 mm above the bed).
    6. Go to Z10, then `M106 S0.00` and `M104`/`M109` back to the
       earlier target. This turns the part fan off.
    G28 ignores whether it worked: after 3 failed rounds it only logs
    "Homing Z failure …".
  - `STRAINGAUGE_TEST [X Y Z SPEED]`: move there, then tap up to 50 mm at
    SPEED (1 mm/s) and go back up. It uses the last tare.
    MIN_HOLD/MAX_HOLD are read but not used.
  - `DIRZCTL [DIRECT=1] [STEP_US=1500] [STEP_CNT=256]` steps Z directly
    and returns at once. DIRECT 1 is up.
  - The step counts in `result_dirzctl` are 16 bits. A first tap more
    than ~94 mm (37664 steps) above the bed wraps the arithmetic, looks
    like a 3 mm miss, and shuts down with "No response from MCU" (stock
    bug, kept).
  - Only `[strain_gauge]`, `[hx711s]` and `[dirzctl]` register these
    commands. Without them, ccos answers `unknown command`, as the
    printer does.
  - `PRTOUCH_READY` taps the four mesh corners (two taps each, up to
    three tries) and prints `RDY_POS`/`GAP_POS`. Every tap starts at Z
    `bed_max_err` + 1 and goes at most 10 mm down. The stock `bed_max_err:
    20` is clamped to 10, so the taps start at Z11, never reach the bed and
    read 0 (stock behaviour).
  - `CHECK_BED_MESH [AUTO_G29=1]` taps 2–5 mm (random) inside each mesh
    corner. With no mesh, it prints "The bed_mesh data is invalid and
    cannot be verified."; with `AUTO_G29` it calibrates instead.
    - A corner counts as bad when its *probed* Z is above `bed_max_err`.
      The difference to the mesh is worked out and never used. More than
      two bad corners re-level (or print the message).
    - It makes the next `PRTOUCH_READY` do nothing.
    - It sends `CXSAVE_CONFIG`, which nothing registers: `unknown command`.
  - `MEASURE_GAP_TEST [INDEX=0..3]` taps a corner, then measures the gap
    5 times: shake, then Z ±`gap_dis_range` while the cells sample. The
    board sends no samples, so every gap is 0 and the result is
    `Gap measure vals = 0 0 0 0 0` (as on the printer).
  - `NOZZLE_CLEAR` does nothing: its body is empty.
- **Probe** (`[probe]`, `[bed_mesh_probe]`, `probe.cpp`,
  `auto_leveling.cpp`). Each tap is a `run_G29_Z` load-cell tap.
  - `PROBE` samples with the `[probe]` settings (2 samples, 4 at the edges,
    tolerance 0.05, 3 retries). The result is only logged.
  - `QUERY_PROBE` does nothing.
  - `PROBE_ACCURACY [SAMPLES=10]` (`[probe]`) and `HOME_ACCURACY`
    (`[bed_mesh_probe]`):
    - first Z goes to 0, then the taps run without lifting in between;
    - then "probe accuracy results: maximum …, minimum …, range …,
      average …, median …, standard deviation …";
    - the deviation is summed into an `int` and divided as integers, so it
      is nearly always 0.
  - `LIFT_SPEED` is read and ignored everywhere.
  - `PROBE_CALIBRATE` has no manual step:
    - it takes the probed Z as the new offset at once and shifts the
      current Z by it;
    - it sets `[probe] z_offset` and `[stepper_z] position_endstop`, then
      writes printer.cfg.
  - `CALIBRATE_Z_OFFSET` homes Z with `position_endstop` 0, goes to Z5 and
    runs `PROBE`. It saves −(result + 1) as `position_endstop` and
    `position_endstop_extra` in printer.cfg and user_printer.cfg. If the
    probe fails, `position_endstop` stays 0.
  - `LIFT_HOT_BED [S=50]`:
    - Z is set to 0 (homed), then it steps down S mm at `z_hop_speed`;
    - when the load cells are pressed, it goes 5 mm back;
    - afterwards Z is unhomed.
- **Bed mesh** (`[bed_mesh]`, `[bed_mesh_probe]`).
  - The grid is 11×11 from (20, 20) to (246, 246), probed in a zig-zag
    from the front. It is interpolated bicubically with 3 points between
    probes. `BED_MESH_SET_INDEX INDEX=1` switches to the secondary 6×6 grid
    (Lagrange, 6 points between).
  - At boot the stock UI runs `BED_MESH_SET_INDEX TYPE=standard|enhancement
    INDEX=0`, from sysconf `print_platform_type`. That applies the plate's
    Z offset (`SET_GCODE_OFFSET Z=0`, then `M8233 S<offset> P1`, which
    saves), stores the plate in `platform_material`, rebuilds the grid and
    loads the plate's `default` mesh. M8823 runs it too.
  - Profiles are `[besh_profile_<plate>_<name>]` sections (the stock
    spelling) in `printer.cfg` and `user_printer.cfg`: points, min/max,
    algorithm and counts. Loading one logs "bed_mesh: malformed value"
    twice, and takes `mesh_y_pps` from `mesh_x_pps` (stock bugs, kept).
  - `BED_MESH_PROFILE` is never registered: the profile manager reads the
    version from a section that does not exist. ccos leaves it out too, so
    the screen only *shows* the saved meshes.
  - `BED_MESH_CALIBRATE [PROFILE=|SAVE=] [PROBE_COUNT=x,y] [ALGORITHM=]
    [RELATIVE_REFERENCE_INDEX=] [METHOD=fast] [SAMPLES=…]`:
    1. It runs `BED_MESH_SET_INDEX INDEX=0` (so the `default` mesh is in use
       while probing; the probe moves bypass it).
    2. For each point: up to Z2 at 6 mm/s, over the point at 150 mm/s,
       then taps. Each tap is `run_G29_Z`: shake Z every 12th tap since the
       last load-cell Z homing, tare 30 samples, probe down up to 32 mm at
       3 mm/s, come back up.
    3. Two taps must agree within 0.01 mm. After the 2nd retry the
       tolerance grows by 0.005 per retry. After 20 retries "Probe samples
       exceed samples_tolerance" ends the run. Points within 20 mm of an
       edge use the `edge_*` settings (the same values in the stock
       config).
    4. The mean of the taps, minus the bed-mesh probe's Z offset
       (−z_offset + fix_z_offset after M8233), is the point's height.
    5. The mesh is built, used at once, logged "Bed Mesh Leveling
       Complete", and saved: to the PROFILE/SAVE name, else to the grid
       index (1), else to `default`.
  - **Any probe error** turns the heaters and part fan off and shuts the
    printer down, as on the printer.
  - `METHOD=fast` re-taps the four corners of the mesh in use. If all are
    within 0.03 mm of it, the run ends and nothing is saved. Otherwise
    `BED_MESH_SET_INDEX INDEX=1` and a full run on the 6×6 grid.
  - **G29** (the screen's auto-levelling):
    1. `M211 S0`, nozzle 140 °C, bed 60 °C, fans off.
    2. `WIPE_NOZZLE` (G28 and the purge) with the mesh off.
    3. `RESET_PRINTER_PARAM`, `M109 S140`.
    4. Z offset to 0: `SET_GCODE_OFFSET Z=0`, `M8233 S0.000 P1`, which is
       saved (stock behaviour).
    5. `G28 Z`, `M109 S140`, `M190 S60`, `BED_MESH_CALIBRATE`.
    6. `G1 X10 Y10 F4500`, heaters and fan off, `M211 S1`.

    G29 is a "traditional" command, so `PROFILE=` and `PROBE_COUNT=` on a
    G29 line are garbage letters and never reach it.
  - `G29.1 P0` zeroes the Z endstop, the G-code Z offset and the bed-mesh
    probe offset and turns the mesh off. `P1` restores them from the config
    and reloads the current profile. G29 only uses them with
    `enable_z_home`, which the stock config has off.
  - **Moves.** Every G-code move goes through the mesh. It is split where
    the mesh height changes (`move_check_distance` 5 mm, `split_delta_z`
    0.025 mm) and Z is raised by the mesh. Before splitting, a move past a
    limit is clamped on the first axis out of range only, and the G-code
    position takes the clamped value (`check_endstops`). There is no fade
    (`fade_end` is 0).
  - `BED_MESH_OUTPUT` logs the probed and interpolated mesh. `PGP=1` logs
    the grid points instead. `BED_MESH_MAP` does nothing.
  - `BED_MESH_APPLICATIONS ENABLE=0` turns the mesh off and keeps it.
    `ENABLE=1` puts the kept one back. Without an earlier `ENABLE=0`, that
    is no mesh: `ENABLE=1` turns the mesh off (stock bug, kept).
    `BED_MESH_CLEAR` turns it off.
- **TMC2209 drivers.**
  - At connect, in name order (extruder, stepper_x, stepper_y,
    stepper_z): read DRV_STATUS; write SLAVECONF and GCONF; then write
    every register that has a value, in name order; read DRV_STATUS
    again. The stock values:

    | | CHOPCONF | IHOLD_IRUN | TPWMTHRS | SGTHRS |
    |---|---|---|---|---|
    | X, Y | `0x14010053` | `0x081010` | `0xfffff` | 120 |
    | Z | `0x14030053` | `0x081717` | 0 | 120 |
    | E | `0x14030053` | `0x08170e` | `0xfffff` | 10 |

    Common to all four: GCONF `0xc0`, PWMCONF `0xc80d0e24`, SLAVECONF
    `0x200`, TPOWERDOWN 20.
  - `stealthchop_threshold: 0` gives 12 MHz × step / 0 = +∞. The
    printer's ARM float conversion saturates that to `0xfffff`, so
    StealthChop applies only at standstill.
  - Every write is verified by IFCNT going up by one, with five tries.
    The check doesn't wrap (as in the firmware), so a write that takes
    IFCNT from 255 to 0 is sent twice.
  - Each homing move on a virtual endstop (X, Y) switches StallGuard on:
    TPWMTHRS 0, StealthChop, TCOOLTHRS for 1 mm/s, SGTHRS =
    `driver_sgthrs`. Afterwards TPWMTHRS goes back, and TCOOLTHRS and
    SGTHRS go to 0. G28 sets the homing current with
    `SET_TMC_CURRENT_<stepper> CURRENT=…`.
  - Stepper enable/disable never touches the drivers: the stock
    reactor's `register_callback` is empty.
  - The UI's "silent mode" event is never sent: all its callers are
    commented out.
- **TMC commands.**
  - `SET_TMC_CURRENT_<stepper>` without parameters restores the
    configured currents and reports the currents from before. With
    parameters it reports the new ones, e.g. `stepper_x Run Current:
    0.795495A Hold Current: 0.795495A`.
  - `INIT_TMC`, `SET_TMC_FIELD` and `DUMP_TMC` are mux commands in
    Klipper. The stock `register_mux_command` drops the key, so the last
    driver created, **the extruder**, handles them whatever `STEPPER=`
    says.
  - `DUMP_TMC` reads the registers and prints nothing.
  - `SET_TMC_FIELD` with an unknown field logs an error, then writes 0
    to register "", i.e. address 0, the extruder's GCONF. The empty
    register then stays in the image that `INIT_TMC` rewrites.
  - The UART transport resends a request every 7 ms until it is
    answered; the MCU drops copies that arrive while busy. Writes
    scheduled with a print time wait on a queue of their own, so they
    don't hold up the step messages.
- **CUSTOM_ZERO** homes Z at X202 Y264.5 (the purge corner). The file
  pipeline replaces the first line containing `G28` in a print file with it.
- **Print files.** Text after `;` is cut. Layers come from `;LAYER:<n>`
  (FILESLICE 2 or 4) or from `;AFTER_LAYER_CHANGE` plus the next line's
  number. UI commands run between lines. M117 resets the print timer.
- **PAUSE / RESUME / CANCEL_PRINT / STOP_PRINT / M600** run the stock
  scripts from `pause_resume.cpp` and `change_filament.cpp` word for word.
  Five minutes after a pause, the nozzle is cooled to
  min(saved target, 45 °C) and the part fan is turned off.
- **Power-loss resume** (`break_save.cpp`, `src/breaksave.rs`,
  `src/gcode/breaksave.rs`).
  - A thread polls GPIOG(0) (gpio192, high = mains lost) every
    millisecond. It turns on capacitor charging, GPIOC(1), at start.
    On an outage it stops charging, switches the capacitor onto the
    rail (GPIOG(11)), waits 200 ms and checks again. A glitch logs
    "power outages interference signal detected." and charging resumes.
  - A real outage while printing stops the file reader (`do_pause`) and
    writes `/board-resource/break_save0.gcode`, an INI file. It holds the
    `virtual_sd`, `print_stats`, `gcode_move`, toolhead, fan and heater
    status: the same sections and keys as stock, with values as
    `%f`/`1`/`0`.
  - The thread then ends, so there is no detection until the next boot.
    It also ends after an outage with nothing to save (not printing), as
    stock does.
  - Only with sysconf `power_off_resume_print` on (the stock default).
    The System screen has the switch.
  - At boot, if a save names a file, the "Kind Reminder" prompt offers
    to resume. **Cancel** adds the saved print time to `cumulative_time`
    and deletes the saves. **Resume** prints the file again.
  - `SDCARD_PRINT_FILE` of the file a save names resumes it:
    1. Layer counters and print time are restored, the file seeks to the
       saved line, and Z is set from the saved stepper position.
    2. M140/M104/M190/M109 to the saved targets (truncated to whole
       degrees), then the fans.
    3. `STOP_RESUME` lifts Z 10 mm, homes X/Y and purges 8×15 mm at
       the pause position. It wipes (M729), goes back over the part and
       down to the saved height, then the file goes on.
    4. G90/G91 and M82/M83 are set as saved.
    5. The print runs at 50 % speed for 3 minutes.
  - Any `SDCARD_PRINT_FILE` deletes all saves, even with the switch off.
    A save left over with the switch off still resumes when its file is
    printed again.
  - The save keeps the file line the G-code thread was on and each
    stepper's *commanded* position. The moves still buffered in the
    toolhead are skipped on resume, and Z is off by whatever the MCU
    had not stepped yet.
  - The X/Y "stepper" positions are CoreXY A/B (x+y, x−y), and
    `STOP_RESUME`'s G28 X Y makes them irrelevant.
  - `G28 X Y` inside `STOP_RESUME` queries the Z endstop before the
    10 mm lift has run. Near the bed it reads triggered, so
    `Z_AXIS_OFF_LIMIT_ACTION` lifts another 10 mm and the purge happens
    ~20 mm up.
  - `STOP_RESUME` writes `is_z_homing` into the hard-coded
    `/board-resource/break_save0.gcode`.
  - `BREAK_SAVE_STATUS` does nothing: its body is commented out.
    `gcode_cmds_before/after` are read and never used.
  - `filament_used` is saved as 0, because ccos does not count it yet.
    Fan `rpm` is 0, because ccos has no tachometer. Print history is not
    in ccos, so Cancel doesn't mark the job failed.
- **ROOT gate.** M8xxx lines in a print file are ignored unless `ROOT` ran
  earlier in that file. Exceptions: M8016 is the inverse (it only works from
  a file), and M8803/M8807 are never gated. The flag resets when the file
  ends.
- **Config saves.** M8xxx change `printer.cfg` in memory only. M8800 saves
  it. M8801 restores the factory list (including the `extruer` typo key) and
  saves. M8233, M8823 and M8824 save the files they touch. Files are written
  as `key : value` with 6 decimals; `user_printer.cfg` is overlaid on
  `printer.cfg` at load.

## ccos additions

These are not in the stock firmware. Lines that work there do the same
here; the additions only give meaning to lines the stock firmware ignores.

- `BED_MESH_CALIBRATE S<°C>` heats the bed and waits for it before
  probing. The bed is left at that temperature. Plain `BED_MESH_CALIBRATE`
  does not heat, as in the stock firmware. `S=75` works too.
- `BED_MESH_CALIBRATE <name>` saves the mesh as `<name>` instead of
  `default`, like `PROFILE=<name>`. Both can be combined, in either order:
  `BED_MESH_CALIBRATE S75 pla`.
  - A bare word `S<number>` (or `s<number>`) is the temperature; any other
    bare word is the name.
  - Names may use letters, digits, `_`, `-` and `.`.
  - Two names, two temperatures, a name that differs from `PROFILE=`, or a
    temperature outside the bed's `min_temp`..`max_temp` are refused
    before anything moves.
  - In the stock firmware a bare word is a parameter nobody reads.
- `G29 S<°C>` uses that bed temperature instead of 60 °C.
- The **Bed mesh** screen (Control → Bed mesh) shows the saved profiles
  one at a time, `default` first: a heatmap seen from above (front at the
  bottom; blue low, red high), the plate, the grid size, the algorithm,
  whether it is the mesh in use, and the highest point, lowest point, range
  and mean. The arrows step through the profiles. It is view-only, because
  the stock firmware has no command to load a profile
  (`BED_MESH_PROFILE` is never registered).
- While probing, the status line reads `Bed mesh: point <n> of <total>`.

## Deliberate differences

| Where | Stock firmware | ccos | Why |
|---|---|---|---|
| G28 heater restore | `M109 S%d` is passed a `double`: undefined behaviour, prints garbage | restores the previous extruder target | intended behaviour |
| M8802 | `reboot` | in the emulator: only logs | do not reboot the PC |
| M8803/M8807/M8826 | `/mnt/exUDISK` | in the emulator: `<sim dir>/usb` | no USB stick |
| M8810 | exports logs | logs "not available" | the log files are those of the stock app |
| TMC register read failing 5 times | logs, then returns an undefined value | an error: `INIT_TMC`/`DUMP_TMC` fail; at connect "TMC <name> failed to init" is logged and the next driver is set up | undefined behaviour |
| TMC drivers with motion locked | — | not configured (`CCOS_ENABLE_MOTION` unset: ccos drives no motor outputs) | safety on the real printer |
| Load-cell waits | `MCUException("No response from MCU")` after 50 resends (2345 polls each) or a 100 s tare; nothing catches it | an emergency shutdown with "strain_gauge_mcu: No response from MCU" | the uncaught exception ends the stock app |
| `READ_HX711` | waits forever for samples nothing collects | sends `query_hx711s`, logs that no samples come back, returns | a hang |
| `cal_min_z` | spins for the two Z reports without checking for a shutdown | a shutdown ends the wait | a hang |
| `run_G28_Z` (probe path) | no `return` statement | returns whether Z homed; G28 ignores it as before | undefined behaviour |
| `use_probe_by_step: 0` | G28 homes Z with `homing_z_move` on the endstop, sampling the load cells | refused ("not available in ccos") | not used by the stock config |
| `BED_MESH_OFFSET` | writes X/Y into an empty vector | sets the mesh's X/Y lookup offsets (as Klipper does); "No mesh loaded to offset" without a mesh | undefined behaviour |
| `BED_MESH_APPLICATIONS ENABLE=1` after a calibration | uses the mesh that `set_mesh` has deleted | the kept mesh is still valid and is used | undefined behaviour |
| `BED_MESH_CALIBRATE METHOD=manual` | registers ACCEPT/ABORT on a helper that is destroyed when the command returns | refused ("not available in ccos") | undefined behaviour |
| `PROBE_COUNT` below 3 | only logs "bed_mesh must have a minimum", then reads outside the mesh | refused before probing | undefined behaviour |
| Grid that isn't square (`PROBE_COUNT=5,3`) | the interpolated mesh is sized x × y but indexed [y][x]; a saved profile is read the same way | rows are Y, columns X | undefined behaviour |
| Bicubic with fewer than 3 points on an axis | control points past the end of a row | the last point repeats | undefined behaviour |
| `ALGORITHM=direct` with `mesh_pps` > 0 | the mesh is sized for the interpolated points but holds only the probed ones | no points between probes | undefined behaviour |
| `RELATIVE_REFERENCE_INDEX` outside the grid | indexes past the probed points | ignored (the probe's Z offset is used) | undefined behaviour |
| `G29` Z offset | `M8233 S%.3f` given an `int` | `M8233 S0.000 P1` | undefined behaviour |
| `G29`/probing without `[strain_gauge]` | dereferences null | probing refused with an error | undefined behaviour |
| Shutdown after a probe error | reason "" | the error text, so the screen says why | display only |
| Load-cell corner tables | 3 rows, 4 read | the 4th corner is (max X, min Y), as the commented-out stock lines; the 4th gap point (max X − 1, min Y + 1) at 0.1 | undefined behaviour |
| `MEASURE_GAP_TEST INDEX` outside 0–3 | reads past its table | refused | undefined behaviour |
| Load-cell commands without `[bed_mesh]` | null dereference | do nothing | undefined behaviour |
| Resume `set_position` | given X, Y, Z only; the toolhead reads E past the end | E keeps its current value | undefined behaviour |
| After a power-loss save | `while(1){}` until the capacitor runs out | the watcher thread ends | undefined behaviour |
| Power-loss save path | `/board-resource/` | the config folder (`<sim dir>/board-resource` in the emulator) | same folder on the printer |

## Tests

- Unit tests: `cargo test -p ccos`. They cover the parser examples from
  `gcode.cpp` comments, out-of-range fallbacks, number parsing, ConfigParser
  round trips, arcs, the heater rules and the stock machine read from
  `printer.cfg`.
- Emulator scenarios in `ccos/sim/scripts/` (see [09](09-emulator.md)):
  - `02_heat_home` checks the stock G28 end positions and that PG8 agrees
    with the physical gap.
  - `03_print_cube` checks CUSTOM_ZERO, layer counting and the 0.15 mm
    offset.
  - `04_faults` checks verify_heater, the MCU max_duration and dropped
    unhomed moves.
  - `07_tmc` checks the driver register images after connect,
    `SET_TMC_CURRENT`, the extruder answering `INIT_TMC`/`SET_TMC_FIELD`,
    StallGuard on and off around homing, and a failed X homing (broken
    DIAG wire) with G28 carrying on with Y. The emulated drivers
    microstep at their pin default (1/8, 1/32 or 1/16 by address) and
    never stall-detect unless configured like this.
  - `12_probe` runs the probe commands: PROBE, PROBE_ACCURACY,
    HOME_ACCURACY, PRTOUCH_READY, CHECK_BED_MESH with and without a mesh,
    MEASURE_GAP_TEST, LIFT_HOT_BED, PROBE_CALIBRATE and
    CALIBRATE_Z_OFFSET.
  - `11_power_loss` cuts the power mid-print, then checks the save file,
    the boot prompt, `STOP_RESUME`, the finished part and the Cancel
    path.
