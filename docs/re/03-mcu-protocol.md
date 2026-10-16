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

### Setup

1. `open("/dev/dsp_debug")`, `ioctl(1 = CMD_READ_DEBUG_MSG, &dsp_sharespace_t)`.
   The 52-byte struct lists the physical addresses/sizes of the DSP-write,
   ARM-write and log regions defined in the DSP image's device-tree header.
2. Set `mmap_phy_addr = arm_write_addr`, `ioctl(3 = CMD_WRITE_DEBUG_MSG)`,
   `mmap` 4 KiB → the **mailbox page**.
3. `open("/dev/kbuf-mgr-0")`, `ioctl(0x100 = CREATE_BUF, {name "test",
   len 16384, type NONCACHE})` → returns `minor` and physical address `pa`.
   `mmap("/dev/kbuf-map-<minor>-test")`. Page 0 = ARM→DSP ring, page 1 =
   DSP→ARM ring.
4. Write into the mailbox head `{read_addr: pa+4096, write_addr: pa,
   init_state: 1 (or 2 if it was already 1/2)}`. The DSP
   (`sharespace_reinit()` in `share_space.c`) polls this and adopts the
   addresses.
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
* The DSP's doorbells carry `(dsp_write_pos << 16) | dsp_read_pos`; the host
  takes `dsp_read_pos` from the low half to compute free space, and reads new
  data up to the DSP head's `write_addr`. Values ≥ 5000 are ignored.
* The host's read position reaches the DSP only through the low half of the
  host's next doorbell.

### DSP lifecycle

`/sys/class/remoteproc/remoteproc0/state` accepts `start` / `stop`
(`core/aw/aw_dsp.c`). A restart re-runs the DSP's `share_space_init()`.

## STM32 links

Plain termios raw serial. Toolhead: `/dev/ttyACM0` (USB CDC; the configured
1 000 000 baud is irrelevant over USB). Load-cell board: `/dev/ttyS4` 115200 8N1.

## Talking to the MCUs yourself

With the stock `app` stopped:

```sh
ccos probe all          # identify, get_config, get_uptime, measure clock
ccos console toolhead   # type raw commands, e.g. get_uptime / get_clock
ccos temps              # configure 2 ADC channels on a fresh DSP, print bed & chamber temps
```

An MCU accepts `config_*` commands only until `finalize_config`. After the
stock app has configured them, return the STM32s to a fresh state with
`ccos power-cycle toolhead|sg` (same GPIO sequence as the stock host), and the
DSP with a remoteproc stop/start or a reboot.
