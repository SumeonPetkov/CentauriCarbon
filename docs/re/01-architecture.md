# 01 — Software architecture

## Processors

| Processor | Runs | Source in repo |
|---|---|---|
| R528-S3, 2x Cortex-A7, 128 MB DDR3 in package, 8 GB eMMC | Tina Linux (OpenWrt-based), kernel 5.4.61, glibc 2.23; the `app` binary | `firmware/` |
| R528 HiFi4 DSP (Xtensa) | FreeRTOS + Klipper MCU firmware (main board I/O); image in the `dsp0A`/`dsp0B` partitions | `dsp/projects/r528/dsp0/` (project files only, no SDK) |
| STM32F402RCT6 "extruder" (toolhead) | Klipper MCU firmware behind Elegoo's 32 KiB YMODEM bootloader | `mcu/` with `board/extruder` |
| STM32F402RCT6 "sg" (bed / load-cell board) | Klipper MCU firmware behind the same bootloader | `mcu/` with `board/sg` |

The STM32F402 is a China-market variant; build for STM32F401 (Elegoo's
configs and OpenCentauri do the same). Hardware photos and connector pinouts:
[OpenCentauri hardware docs](https://github.com/OpenCentauri/OpenCentauri/tree/main/docs/hardware/CC1).

## eMMC partitions (from OpenCentauri)

| Partition | Content |
|---|---|
| `boot-resource` (p1) | boot logos |
| `env`, `env-redund` (p2, p3) | u-boot environment, incl. the active A/B slot (`boot_partition`) |
| `bootA` / `rootfsA` / `dsp0A` (p4–p6) | slot A: kernel, root filesystem, DSP firmware |
| `bootB` / `rootfsB` / `dsp0B` (p7–p9) | slot B |
| `rootfs_data` (p10) | overlay data |
| `user` (p11) | logs |
| `private` (p12) | empty |
| `UDISK` (p13) | user files (`/user-resource`) |

The repo also contains an `e100_lite` variant (`app/e100_lite`,
`resources/e100_lite`, `CMakeLists.txt.e100_lite`) for a cheaper model.

## The `app` binary

One C++14 executable (`firmware/CMakeLists.txt`, target `app`, built `-O3`),
installed as `/app/app` with data in `/app/resources/`. It contains:

| Directory | Role |
|---|---|
| `main.cpp` | Process entry (below) |
| `core/klippy/` | C++ port of Klipper's Python host: `klippy.cpp` (Printer singleton), `reactor`, `gcode`, `toolhead`, `mcu`, `serialhdl`, `clocksync`, `msgproto` |
| `core/klippy/extras/` | ~85 modules: heaters, PID, fans, TMC drivers, bed_mesh, probe, input_shaper, resonance_tester, virtual_sdcard, pause_resume, **strain_gauge**, **hx711s**, **auto_leveling**, **break_save** (power-loss resume), change_filament, … |
| `core/klippy/kinematics/` | cartesian, corexy (used), corexz, delta, extruder |
| `core/klippy/chelper/` | Klipper's C helpers: `serialqueue.c`, `stepcompress.c`, `itersolve.c`, `trapq.c`, `kin_*.c`, `kin_shaper.c` |
| `core/allwinner/`, `core/aw/` | DSP link (kbuf, sharespace, rpmsg msgbox), "combus" transport abstraction |
| `gui/` | LVGL 8.3 + fbdev/evdev drivers |
| `ui/loader/` | Loads `widget.bin` / `translation.bin`, builds LVGL objects |
| `app/e100/` | Screen logic (~18 k lines): main, print, control, settings, calibration, camera, network, file explorer, device self-test |
| `service/` | `srv_state` / `srv_control` façade between UI and printer (over simplebus) |
| `core/simplebus.c` | In-process pub/sub + request/response bus |
| `hl/` | "Hardware layer": Wi-Fi (wpa_supplicant), ethernet, USB disk hotplug (netlink uevents), camera (V4L2), boot env, thread pool, ring buffers, MD5 |
| `devices/` | GPIO, beeper, serial, OTA/MCU update, params |
| `api/` | OTA (`swupdate` invocation), u-boot env |
| `utils/` | cJSON, mongoose (linked but no server in this drop), AES/SM4, ymodem, tiny EKF, unzip |
| `lib/`, `include/` | Prebuilt ARM libs: ffmpeg/x264 (timelapse), curl, openssl/gnutls, freetype, png, alsa, wpa_client… |

### Startup sequence (`main.cpp`)

1. `hl_tpool_init(24)`, logging, read chip id, `simple_bus_init()`.
2. Spawn **`process_task_printer`** with `SCHED_RR` at max priority:
   `Printer::connect()` → `read_config()` → load objects → events
   `klippy:mcu_identify`, `klippy:connect`, `klippy:ready` → `sem_post` →
   forever: run reactor timers (`_check_timers`) and the manual-control queue.
3. Meanwhile on the main thread: `gui_init()` (fb0 + evdev, LVGL),
   netlink uevents, USB device modules (`/lib/modules/5.4.61`), disks,
   `param_init`, `sysconf_init`, `FileManager`, camera, timelapse, Wi-Fi.
4. `sem_wait` until the printer thread is up, set backlight to 191/255 via
   `/sys/kernel/debug/dispdbg`, then `service_init()`, `register_api()`,
   `ui_init()`, `app_init()`.
5. Delete `*.tmp` on USB and eMMC (interrupted copies), start the
   **break_save** thread (power-loss recovery).
6. Main loop: `ui_loop(); lv_task_handler(); usleep(20000);` — the UI runs at
   ~50 Hz on the main thread.

Other threads: one `serialqueue` I/O thread per MCU (`serialhdl.cpp`), a
clock-sync thread, OTA workers, beeper, file explorer, thread-pool workers.

### Data flow

```
touch → LVGL event → app/e100/*.cpp → manual_control_sq / gcode queue
      → printer thread (reactor) → gcode handlers → toolhead → trapq/itersolve
      → stepcompress → serialqueue → MCU link
MCU responses → serialqueue thread → registered callbacks → printer objects
      → srv_state (simplebus publish) → app_* update callbacks → LVGL
```

## On-device paths

| Path | Content |
|---|---|
| `/app/app` | the binary |
| `/app/resources/` | `widget.bin`, `translation.bin`, `images/<n>.png`, `fonts/font.ttc`, `configs/*.cfg` (factory defaults), sample `gcode/`, `file_info/` |
| `/board-resource/` | persistent per-machine data: `printer.cfg`, `user_printer.cfg`, `unmodifiable.cfg`, `sysconf.cfg`, `variables.cfg`, `history.txt`, `break_save*.gcode`, logs, `machine_info.bin`, `wlan_entery` |
| `/user-resource/` | user files (uploaded gcode), timelapse videos `aic_tlp/`, OTA staging (`update.bin`, `update_tmp/`) |
| `/mnt/exUDISK/` | USB stick mount point |
| `/lib/firmware/upgrade_extruder.bin`, `upgrade_sg.bin` | STM32 images flashed by the host when versions differ |
| `/sys/class/remoteproc/remoteproc0/state` | DSP start/stop |
| `/dev/dsp_debug`, `/dev/kbuf-mgr-0`, `/dev/rpmsg_ctrl0` | DSP link devices |
| `/dev/fb0`, `/dev/input/event1` | display, touch |
