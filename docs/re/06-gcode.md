# 06 — G-code extensions

The C++ host implements standard Klipper/Marlin commands (G0–G4, G28, G29,
M104/M109/M140/M190, M106/M107, M204, M220/M221 equivalents, M600, PAUSE,
RESUME, CANCEL_PRINT, SET_*, BED_MESH_*, SHAPER_CALIBRATE, PID_CALIBRATE,
EXCLUDE_OBJECT…) plus a large set of Elegoo additions. Full list of
registered names: `grep -rhoE 'register_command\("[A-Z_0-9.]+"' firmware/core/klippy`.

## Printer-specific commands

| Command | Where | What it does |
|---|---|---|
| `G29` (with profile / probe-count params in this fork), `G29.1` | `extras/auto_leveling.cpp` | Load-cell bed mesh |
| `M729` | `auto_leveling.cpp` | Nozzle wipe / clean sequence used in auto-leveling |
| `WIPE_NOZZLE`, `CALIBRATE_Z_OFFSET`, `LIFT_HOT_BED` | `auto_leveling.cpp` | Calibration helpers |
| `STRAINGAUGE_Z_HOME`, `STRAINGAUGE_TEST`, `PRTOUCH_READY`, `READ_HX711` | `strain_gauge.cpp`, `hx711s.cpp` | Load-cell Z homing and diagnostics |
| `MEASURE_GAP_TEST`, `HOME_ACCURACY`, `TESTZ`, `DIRZCTL` | various | Z / probe accuracy tests |
| `CUT_OFF_FILAMENT`, `LOAD_FILAMENT`, `EXTRUDE_FILAMENT`, `MOVE_TO_EXTRUDE`, `NOZZLE_CLEAR` | `change_filament.cpp` | Filament change flow |
| `BREAK_SAVE_STATUS`, `SDCARD_PRINT_FILE_BREAK_SAVE` | `break_save.cpp`, `virtual_sdcard.cpp` | Power-loss resume |
| `FLOW_CALIBRATION` | | Flow test pattern (`[flow_calibration]`) |
| `M222 S<mm/s>` / `M223 S<mm/s>` | `toolhead.cpp` | Set max XY / Z velocity at runtime (ignored from print files) |
| Speed modes (UI) | | Silent / Balanced / Sport / Ludicrous = 50 / 100 / 130 / 160 % speed (measured by OpenCentauri) |
| `O1`, `O9`, `O21`–`O32` | `palette2.cpp` | Upstream Klipper Palette 2 support, left compiled in |

## `M8000`–`M8826`: config editing from G-code

`printer_para.cpp` registers ~165 commands that rewrite `printer.cfg`,
`user_printer.cfg`, `unmodifiable.cfg` or `sysconf.cfg` keys, grouped by
section (from the source comments):

| Range | Section |
|---|---|
| M8000–M8044 | Stepper step/dir/enable pins and directions (`S1` = default, `S-1` = inverted), steps, endstops |
| M8045–M8047 | Virtual endstop |
| M8050–M8059 | `[extruder]` |
| M8060–M8069 | `[heater_bed]` |
| M8070–M8074 | `[resonance_tester]` |
| M8080–M8099 | `[printer]` limits |
| M8100–M8122 | `[tmc2209 *]` |
| M8200–M8204 | `[bed_mesh]` |
| M8210–M8220 | `[auto_leveling]` |
| M8230–M8240 | `[probe]` |
| M8250–M8259 | `[bed_mesh_probe]` |
| M8800 | Save |
| M8802 | Reboot |
| M8803 | Copy `printer.cfg`, `user_printer.cfg`, `unmodifiable.cfg` to USB |
| M8804 | Set UI language |
| M8805 | Beeper on/off |
| M8806 | Filament runout detection on/off |
| M8807 | Copy `printer.cfg` and (if present) `user_printer.cfg` from USB to the printer |
| M8808 | First-boot flag |
| M8809 | Wi-Fi on/off |
| M8811 | Region |
| M8815 | Run mode |
| M8823 | Strain-gauge Z-offset refresh (forced at boot when `unmodifiable.cfg [strain_gauge] update` is 0) |

**Gate:** when one of the config *setters* (M8000–M8259, M8800, M8804, M8823, …)
arrives from a print file (`is_cmd_from_sd()`), it returns immediately unless a
preceding `ROOT` command set `is_root`; `UNROOT` clears it. From the UI/console
they always run. `M8803` and `M8807` are not gated. So printing a G-code file
containing

```gcode
M8803        ; dump printer.cfg / user_printer.cfg / unmodifiable.cfg to the USB stick
```

is a no-shell way to pull the live configs off the printer, and `M8807` the way
to push edited configs back (read at the next start; an invalid config can stop
the printer from booting). Treat stepper/pin commands with care:
they change the hardware mapping.
