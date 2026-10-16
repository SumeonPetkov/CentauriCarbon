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
| `gcode/home.rs` | `homing.cpp`, `safe_z_home.cpp`, `homing_override`-style G28 of `printer.cfg` |
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
| Extruder | SET_PRESSURE_ADVANCE M900 SET_EXTRUDER_STEP_DISTANCE SET_MIN_EXTRUDE_TEMP FLOW_CALIBRATION ACTIVATE_EXTRUDER RESET_EXTRUDER |
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
| Load-cell probing | G29 PROBE PROBE_ACCURACY PROBE_CALIBRATE QUERY_PROBE CALIBRATE_Z_OFFSET CHECK_BED_MESH HOME_ACCURACY LIFT_HOT_BED MEASURE_GAP_TEST NOZZLE_CLEAR PRTOUCH_READY READ_HX711 STRAINGAUGE_TEST STRAINGAUGE_Z_HOME BED_MESH_CALIBRATE |
| Bed mesh | BED_MESH_MAP BED_MESH_OFFSET BED_MESH_OUTPUT BED_MESH_SET_INDEX |
| TMC2209 UART | INIT_TMC DUMP_TMC SET_TMC_FIELD SET_TMC_CURRENT_stepper_x/_y/_z/_extruder |
| Accelerometer / input shaper | SET_INPUT_SHAPER M593 SHAPER_CALIBRATE SHAPER_CALIBRATE_SCRIPT TEST_RESONANCES MEASURE_AXES_NOISE ACCELEROMETER_MEASURE_X ACCELEROMETER_QUERY_X ADXL345_DEBUG_READ_X ADXL345_DEBUG_WRITE_X |
| Power-loss resume | BREAK_SAVE_STATUS SDCARD_PRINT_FILE_BREAK_SAVE STOP_RESUME |
| Other | DIRZCTL |

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
| G28 homing current | `SET_TMC_CURRENT_*` changes the TMC current | sent, logged as phase 2 | no TMC UART yet; the stock current from power-on is used |
| SET_PRESSURE_ADVANCE / M900 | applied by the extruder kinematics | value stored, not applied to motion | phase 2 |
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
