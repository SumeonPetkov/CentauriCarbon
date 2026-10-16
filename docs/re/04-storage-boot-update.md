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
3. Decrypt (`ota_dec_routine`) — in this source drop it only verifies the MD5;
   the step that produces `/user-resource/update_firmware.zip` is missing. The
   shipped firmware AES-decrypts the payload; OpenCentauri found the key and IV
   and publishes an unpacker (`docs/assets/unpack.py` and an online version in
   their docs).
4. Unzip to `/user-resource/update_tmp/`, then
   `swupdate -i update/update.swu -e stable,now_A_next_B -k /etc/swupdate_public.pem`
   (or `now_B_next_A`, based on `fw_printenv boot_partition`).

`-k` means the `.swu` must be signed with Elegoo's private key. **You cannot
install a custom rootfs through the official updater.** A/B slots mean a
failed update falls back to the other slot.

A USB stick with `update/update.swu` directly is also handled (`ota_update2`),
with the same signature check. Inside the `.swu` are the partition images that
get written (kernel, rootfs, DSP image). The printer polls
`mms.chituiot.com/mainboardVersionUpdate/getInfo.do7` for updates; OpenCentauri
keeps an archive of every released `update.bin`.

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

Shipped in this repo: `upgrade_extruder.bin` v1.2.3, `upgrade_sg.bin` v1.6.9
(board 255). Names and locations changed between releases: older firmware
used `/lib/firmware/upgrade-hotend.bin` / `upgrade-bed.bin`, and from 1.1.29
they live in `/app/resources/` (per OpenCentauri). The header layout is the same.
On every connect the host power-cycles the board, then sends
`/lib/firmware/upgrade_*.bin` over YMODEM to the bootloader, which flashes it
only if major/minor/patch differ from what it has (`mcu/bootloader/Utils/Src/ymodem.c`).
Only MD5 is checked — rebuild `mcu/` and repack with
`ccres mcu pack klipper.bin upgrade_extruder.bin --version 1.2.4`.

`build.sh` also produces `*_full_pack.bin` = bootloader padded to 32 KiB + the
image above, i.e. a complete flash image starting at `0x08000000`: the 16 KiB
header sector lands at `0x08008000`, the app at `0x0800C000`. OpenCentauri's
`mcu-flasher` pushes images through the stock bootloader without touching it;
prefer that over SWD/DFU, which requires disabling read-out protection and
erases the bootloader.

### DSP firmware

Stored in the `dsp0A`/`dsp0B` eMMC partitions (one per A/B slot) and started
through remoteproc. Rebuilding it needs the Allwinner Tina RTOS SDK and
Cadence Xtensa tools (see [07](07-building.md)).

## Engineering / debug hooks

* `/mnt/exUDISK/elegoo_engineering_mode` — if present on a USB stick when the
  first-boot device inspection runs, sets `engineering_mode=1` in `sysconf.cfg`.
* `ROOT` / `UNROOT` G-codes — not Linux root; they allow `M8000…M8826`
  config-editing commands to run from a G-code file (otherwise those are
  ignored when coming from the SD/print path). See [06](06-gcode.md).
* `M8803` copies `printer.cfg`, `user_printer.cfg` and `unmodifiable.cfg` to
  the USB stick; `M8807` copies `printer.cfg` and (if present)
  `user_printer.cfg` back. Neither is gated by `ROOT`, so a one-line G-code
  file can export or import the configuration. An invalid config can stop the
  printer from booting.

## Getting a shell

Nothing in Elegoo's firmware opens a shell. The supported route is
**OpenCentauri patched firmware** (based on official 1.4.46 as of their v0.4.0):
install it from a USB stick with their `oc-installer` (touch-and-hold the
`IMPORT_ME_DO_NOT_PRINT` file in the printer's file browser), after which SSH
is open with user `root`, password `OpenCentauri`. It is uninstallable from the
same installer. Hardware routes also exist: UART0 on the mainboard and FEL
mode / eMMC restore (OpenCentauri `docs/software/FEL-mode.md`).

Deploying `ccos` from there:

```sh
scp target/armv7-unknown-linux-musleabihf/release/ccos root@<printer>:/user-resource/
ssh root@<printer>
killall app            # the stock UI/host; a reboot starts it again
/user-resource/ccos
```

If something respawns `app`, stop it through its init script in
`/etc/init.d/` instead (not verified here), or use the patched firmware's
developer option that prevents the stock host from starting at boot; that is
also the clean way to give the DSP to `ccos` without a remoteproc restart.

Edited stock UI resources go to `/app/resources/` the same way (back up the
originals first).
