# 04 — Storage, configuration and updates

## Configuration files

| File (on device) | Factory copy | Purpose |
|---|---|---|
| `/board-resource/printer.cfg` | `/app/resources/configs/printer.cfg` | Klipper config (pins, kinematics, PID, …). Copied from the factory version if missing. |
| `/board-resource/user_printer.cfg` | `/app/resources/configs/user_printer.cfg` | User overrides |
| `/board-resource/unmodifiable.cfg` | `…/unmodifiable.cfg` | Factory calibration (e.g. strain-gauge fixed Z offsets) |
| `/board-resource/sysconf.cfg` | `…/sysconf.cfg` | UI/system settings: language, Wi-Fi on/off, timezone, silent mode, timelapse, … |
| `/board-resource/variables.cfg` | — | `[save_variables]` storage |

Paths are defined in `core/klippy/Define_config_path.h`; the copy-on-missing
logic is in `configfile.cpp`. Deleting a file in `/board-resource/` restores
the factory default on the next boot.

`sysconf.cfg` keys of note: `language` (index into the translation columns),
`screen_off_time`, `run_mode`, `silent_mode`, `power_off_resume_print`,
`tlp_switch` (timelapse), `engineering_mode` (see below).

## Firmware update formats

### Main system: `update.bin` → `update.swu` (signed)

1. USB: `/mnt/exUDISK/update.bin`, or network download to
   `/user-resource/update.bin`.
2. Header (`api/ota_update.cpp: update_firmware_check`): 4-byte magic
   `14 17 0b 17`, 4 bytes info (`info[3]` must equal `CONFIG_BOARD` = 0 for
   E100), 4 bytes custom, `u32` length, 16-byte MD5 of everything after 32 bytes.
3. "Decrypt" (`ota_dec_routine`) — in this source drop it only verifies the MD5;
   the step that produces `/user-resource/update_firmware.zip` is absent, so the
   shipping binary presumably has an AES stage that was removed.
4. Unzip to `/user-resource/update_tmp/`, then
   `swupdate -i update/update.swu -e stable,now_A_next_B -k /etc/swupdate_public.pem`
   (or `now_B_next_A`, based on `fw_printenv boot_partition`).

`-k` means the `.swu` must be signed with Elegoo's private key. **You cannot
install a custom rootfs through the official updater.** A/B slots mean a
failed update falls back to the other slot.

A USB stick with `update/update.swu` directly is also handled (`ota_update2`),
with the same signature check.

### STM32 MCUs: unsigned

`mcu/scripts/add_magic.c`, verified on both shipped images:

```
0x00  14 18 01 1a                magic (decimal 20 24 01 26)
0x04  major minor patch board    board: 1 = e100, 255 = generic
0x08  01 00 00 00
0x0c  u32 payload length
0x10  MD5(payload)
0x20  0xff … up to 0x4000
0x4000 payload = klipper.bin linked at 0x0800C000
```

Shipped: `upgrade_extruder.bin` v1.2.3, `upgrade_sg.bin` v1.6.9 (board 255).
On every connect the host power-cycles the board, then sends
`/lib/firmware/upgrade_*.bin` over YMODEM to the bootloader, which flashes it
only if major/minor/patch differ from what it has (`mcu/bootloader/Utils/Src/ymodem.c`).
Only MD5 is checked — rebuild `mcu/` and repack with
`ccres mcu pack klipper.bin upgrade_extruder.bin --version 1.2.4`.

`build.sh` also produces `*_full_pack.bin` = bootloader padded to 32 KiB + the
image above, i.e. a complete flash image starting at `0x08000000` for
SWD/DFU programming.

### DSP firmware

Loaded by the kernel via remoteproc from the root filesystem (exact file name
is not in this repo). Rebuilding it needs the Allwinner Tina RTOS SDK and
Cadence Xtensa tools (see [07](07-building.md)).

## Engineering / debug hooks

* `/mnt/exUDISK/elegoo_engineering_mode` — if present on a USB stick when the
  first-boot device inspection runs, sets `engineering_mode=1` in `sysconf.cfg`.
* `ROOT` / `UNROOT` G-codes — not Linux root; they allow `M8000…M8826`
  config-editing commands to run from a G-code file (otherwise those are
  ignored when coming from the SD/print path). See [06](06-gcode.md).
* `/mnt/exUDISK/user_printer.cfg` — referenced by the settings screen for
  importing a user config from USB.

## Getting a shell

Nothing in this repo opens a shell (no sshd/telnetd/adb configuration), and the
official update path is signed. Community projects have documented ways to get
root on the Centauri Carbon; check their current status before attempting
anything, since methods change between firmware versions. Once you have a
shell, deploying `ccos` or edited UI resources is a file copy.
