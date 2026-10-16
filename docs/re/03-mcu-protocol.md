# 03 — MCU protocol and links

Implemented and unit-tested in `ccos/crates/cc-proto` and `cc-transport`.

## Framing (same as upstream Klipper)

```
| len | seq | payload (0..59 B) | crc16 hi | crc16 lo | 0x7e |
```

* `len` = whole frame, 5..64. `seq` = `0x10 | (n & 0x0f)`.
* CRC-16/CCITT over `len..payload` (Klipper's `crc16_ccitt`, init 0xffff).
* A frame with an empty payload is an ACK/NAK; the MCU's frames carry the next
  sequence number it expects from the host.
* Payload = one or more messages: `id` (VLQ) then parameters. Integers
  (`%u %i %hu %hi %c`) are VLQ-encoded (1–5 bytes, sign-aware); buffers
  (`%s %*s %.*s`) are a length byte plus bytes.

Example, `identify offset=0 count=40`, seq 0:
`08 10 01 00 28 5e 9f 7e` (verified against Elegoo's C).

## The Centauri difference: everything is hard-coded

Upstream Klipper downloads a zlib-compressed JSON "data dictionary" with
`identify` and learns message ids, constants and pin enumerations from it.
Elegoo's port does not:

* **Message ids** — `MessageParser::init_id_to_messages_format_map()` in
  `msgproto.cpp` is a fixed table of 115 formats (ids 0–114). Every MCU firmware
  is generated to match (`compile_time_request.c` files are checked in, not
  generated at build time). Full list: [mcu-command-table.md](mcu-command-table.md).
* **Stale identify blobs** — the dictionaries compiled into both STM32 firmwares
  decode to `v0.9.1-616-g28f60f7e-dirty-20220408 … fluiddpi`, `MCU: stm32f446xx`,
  `CLOCK_FREQ: 180000000`, 65 commands. They are leftovers; the host fetches but
  ignores them. 27 ids differ from the real table.
* **Constants** — `Serialhdl::connect_uart()`:

  | | DSP (`mcu_type 1`) | STM32 (`mcu_type 2`) |
  |---|---|---|
  | `clock_freq` | 200 000 000 | 84 000 000 |
  | `pwm_max` | 255 | 2 000 000 |
  | `adc_max` | 4095 | 4095 |
  | `receive_window` | 192 | 192 |
  | `stats_sumsq_base` | 256 | 256 |

* **Pins** — `init_pin_map()`: `P<bank><n>` → `bank * pins_per_bank + n`,
  32/bank on the DSP, 16/bank on the STM32s. `ADC_TEMPERATURE` = 254.
* **Shutdown strings** — `init_shutdown_info_map()` maps `static_string_id`
  to text (e.g. 3 = "Timer too close").

Consequence for modders: if you add a command to an MCU firmware you must add
it to the same id slot in the host table, or switch the host to dynamic
dictionaries (`cc_proto::Dictionary::from_identify` supports both).

Notable Elegoo-specific messages: `config_stepper … step_pulse_ticks=%u`,
`hx711s_*` / load-cell streaming, `dirzctl` (direct Z control during probing),
`fan_check` tach reporting, `lis2dw_*` accelerometer, and
`extruder_bootup_info` (crash dump registers R0–R3, R12, LR, PC, xPSR).

## DSP link (live path: `GetDspKbufOps()`)

`serialqueue.c` treats the DSP as fd type `'m'` (SQT_MEM). The active
implementation is `core/allwinner/kbuf.c` + `SharespaceInit.c` +
`msgbox.c` (`Sharespace.c` and `DSP_mem.c` are older alternatives).

### When the DSP accepts a host

The DSP firmware runs `share_space_clear()` then `share_space_init()` **once,
at DSP boot**: it fills the mailbox head with `0xA5` bytes and spins until a
host writes ring addresses into it. After that it never looks at the mailbox
again. Consequences:

* Whoever attaches first after a DSP (re)start owns it — normally Elegoo's
  `app` at boot.
* A second host (e.g. `ccos` after you kill `app`) must restart the DSP
  through remoteproc first. A mailbox head that still reads `0xA5A5A5A5` means
  the DSP is waiting; anything else means it is attached. `ccos` checks this
  and restarts the DSP only when needed (`--keep-dsp` disables that).

### Setup

1. `open("/dev/dsp_debug")`, `ioctl(1 = CMD_READ_DEBUG_MSG, &dsp_sharespace_t)`.
   The 52-byte struct lists the physical addresses/sizes of the DSP-write,
   ARM-write and log regions defined in the DSP image's device-tree header.
2. Set `mmap_phy_addr`/`mmap_phy_size` to the ARM-write region,
   `ioctl(3 = CMD_WRITE_DEBUG_MSG)`, `mmap` 4 KiB → the **mailbox page**.
3. `open("/dev/kbuf-mgr-0")`, `ioctl(0x100 = CREATE_BUF, {name "test",
   len 16384, type NONCACHE, pa = arm_write_addr})` (the stock host pre-sets
   `pa`) → returns `minor` and the buffer's physical address `pa`.
   `mmap("/dev/kbuf-map-<minor>-test")`. Page 0 = ARM→DSP ring, page 1 =
   DSP→ARM ring. `ioctl(0x200 = DESTROY_BUF)` frees it (the stock host never
   does — "have bug" — but OpenCentauri's tool and `ccos` do on exit).
4. Write into the mailbox head `{read_addr: pa+4096, write_addr: pa,
   init_state: 1 (or 2 if it was already 1/2)}`. The DSP
   (`sharespace_reinit()` in `share_space.c`) adopts the addresses.
5. Write `{12, 12, 1}` into ring page 0's head; poll ring page 1's head until
   the DSP sets `init_state == 1`.
6. `open("/dev/rpmsg_ctrl0")`, `ioctl(RPMSG_CREATE_EPT_IOCTL = 0x4028b501,
   {name "msgbox_demo", src 3, dst 0xffffffff})`, then find the
   `/sys/class/rpmsg/rpmsgN/name == "msgbox_demo"` and open `/dev/rpmsgN`.

### Ring format (each 4 KiB page)

```
0          12                                4084      4096
| unused   | data (4072 B ring)               | head    |
head = { u32 read_addr, u32 write_addr, u32 init_state }  (little-endian)
```

* Positions are byte offsets in the page, wrap 4084 → 12.
* Writer: wait until `free > len` (never fill completely), copy, update own
  head, then send a 4-byte **doorbell** on the rpmsg endpoint:
  `(write_pos << 16) | read_pos`.
* The DSP keeps its consumer position current in **its own page head**
  (`read_addr`); OpenCentauri's working bridge computes free space from that,
  and so does `ccos`. The DSP's doorbells also carry
  `(dsp_write_pos << 16) | dsp_read_pos` and serve as the "data available"
  wake-up; values ≥ 5000 are ignored.
* The host reads new data up to the DSP head's `write_addr`. Its own read
  position reaches the DSP through the low half of its next doorbell.
* The buffer is non-cached, so no cache maintenance is needed per access (the
  stock `kbuf_mem_sync()` is empty).

### DSP log

Selecting the log region (`CHOOSE_DSP_LOG_SPACE`) and mapping
`dsp_log_size` bytes gives the DSP's `printf` output: a `u32` write offset at
0, NUL-separated strings from offset 4, wrapping. `ccos dsp-log` follows it.

### DSP lifecycle

`/sys/class/remoteproc/remoteproc0/state` accepts `start` / `stop`
(`core/aw/aw_dsp.c`; the stock `main.cpp` has a `dsp_restart()` call commented
out). A restart re-runs the DSP's `share_space_init()`. Where remoteproc loads
the image from on a restart has not been verified on hardware.

## STM32 links

Plain termios raw serial. Toolhead: `/dev/ttyACM0` (USB CDC; the configured
1 000 000 baud is irrelevant over USB). Bed board: `/dev/ttyS4` 115200 8N1.
After a power cycle both sit in Elegoo's bootloader until they receive a
YMODEM upload or an `a` (see [02](02-hardware.md)).

## Talking to the MCUs yourself

With the stock `app` stopped (`ccos` refuses to touch the MCUs while it runs):

```sh
ccos probe all              # identify, get_config, measure clock on all three
ccos console toolhead       # type raw commands, e.g. get_uptime / get_clock
ccos power-cycle toolhead   # reset + boot an STM32 (power pin, then 'a' to the bootloader)
ccos dsp-log                # follow the DSP's printf output
ccos                        # the touchscreen UI (live bed/chamber/nozzle temperatures, light)
```

An MCU accepts `config_*` commands only until `finalize_config`. After the
stock app has configured them, the STM32s need a power cycle and the DSP a
remoteproc restart before another host can configure them — `ccos` does both
automatically.
