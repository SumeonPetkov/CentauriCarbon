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
| `gcode/print.rs` | `virtual_sdcard.cpp`, `print_stats.cpp`, `pause_resume.cpp`, `change_filament.cpp`, `auto_leveling.cpp` (M729, WIPE_NOZZLE, G29.1) |
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
| Load cells | STRAINGAUGE_Z_HOME STRAINGAUGE_TEST DIRZCTL READ_HX711 |
| TMC2209 | SET_TMC_CURRENT_stepper_x/_y/_z/_extruder INIT_TMC SET_TMC_FIELD DUMP_TMC |
| Extruder | SET_PRESSURE_ADVANCE M900 SET_EXTRUDER_STEP_DISTANCE SET_MIN_EXTRUDE_TEMP FLOW_CALIBRATION ACTIVATE_EXTRUDER RESET_EXTRUDER |
| Input shaper | SET_INPUT_SHAPER M593 |
| Heaters | M104 M109 M140 M190 M105 SET_HEATER_TEMPERATURE TEMPERATURE_WAIT TURN_OFF_HEATERS M301 M130 M131 M132 PID_CALIBRATE PID_CALIBRATE_ALL PID_CALIBRATE_HOTBED PID_CALIBRATE_NOZZLE |
| Fans, LEDs | M106 M107 SET_FAN_SPEED SET_LED_led1 SET_LED_led2 |
| Print job | SDCARD_PRINT_FILE SDCARD_RESET_FILE SET_PRINT_STATS_INFO M117 PAUSE RESUME CLEAR_PAUSE CANCEL_PRINT STOP_PRINT M75 M76 M77 M601 M602 M603 |
| Filament | M600 LOAD_FILAMENT MOVE_TO_EXTRUDE CUT_OFF_FILAMENT EXTRUDE_FILAMENT CHANGE_FILAMENT_SET_ACTIVE CHANGE_FILAMENT_SET_BUSY CHANGE_FILAMENT_SET_CHECK_MOVE_IGNORE M729 WIPE_NOZZLE G29.1 |
| System | ROOT UNROOT M8000–M8826 (all 165), SET_IDLE_TIMEOUT UPDATE_IDLE_TIMER M112 RESTART FIRMWARE_RESTART ECHO STATUS HELP G M M110 M115 BED_MESH_APPLICATIONS BED_MESH_CLEAR |

**Phase 2**: these exist on the printer, but ccos does not yet have the
subsystem behind them. They are accepted, do nothing, and log
`<CMD>: not available in ccos yet (needs …)` the first time they are used.

| Subsystem | Commands |
|---|---|
| Load-cell probing | G29 PROBE PROBE_ACCURACY PROBE_CALIBRATE QUERY_PROBE CALIBRATE_Z_OFFSET CHECK_BED_MESH HOME_ACCURACY LIFT_HOT_BED MEASURE_GAP_TEST NOZZLE_CLEAR PRTOUCH_READY BED_MESH_CALIBRATE |
| Bed mesh | BED_MESH_MAP BED_MESH_OFFSET BED_MESH_OUTPUT BED_MESH_SET_INDEX |
| Accelerometer | SHAPER_CALIBRATE SHAPER_CALIBRATE_SCRIPT TEST_RESONANCES MEASURE_AXES_NOISE ACCELEROMETER_MEASURE_X ACCELEROMETER_QUERY_X ADXL345_DEBUG_READ_X ADXL345_DEBUG_WRITE_X |
| Power-loss resume | BREAK_SAVE_STATUS SDCARD_PRINT_FILE_BREAK_SAVE STOP_RESUME |

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
- **ROOT gate.** M8xxx lines in a print file are ignored unless `ROOT` ran
  earlier in that file. Exceptions: M8016 is the inverse (it only works from
  a file), and M8803/M8807 are never gated. The flag resets when the file
  ends.
- **Config saves.** M8xxx change `printer.cfg` in memory only. M8800 saves
  it. M8801 restores the factory list (including the `extruer` typo key) and
  saves. M8233, M8823 and M8824 save the files they touch. Files are written
  as `key : value` with 6 decimals; `user_printer.cfg` is overlaid on
  `printer.cfg` at load.

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
| Bed mesh | Z compensation from the probed mesh | none, so the nozzle sits `fix_z_offset` (0.15 mm) higher than the stock firmware with a mesh | phase 2 |

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
