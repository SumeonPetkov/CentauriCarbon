# Editing the ccos UI

Everything the screen shows is defined in this folder, in plain English text
files. `cargo build` compiles it into the `ccos` binary — fonts become
pre-rendered bitmaps, icons become packed pixel data, screens become static
tables. Nothing in this folder is copied to the printer, and nothing is parsed
at runtime.

```
ui/theme.toml          colours, fonts, icons, defaults
ui/screens/<name>.toml one file per screen; <name> is what `goto <name>` uses
ui/parts/<name>.toml   widget groups shared by screens (`include = ["<name>"]`)
ui/fonts/              .ttf files (DejaVu Sans; any TTF/OTF works)
ui/icons/              .png files
```

Workflow:

```sh
# edit a .toml file, then
cargo run --release -p ccsim                          # try it live in the emulator
cargo run --release -p ccos -- preview /tmp/screens   # PNG of every screen, on your PC
cargo build --release --target armv7-unknown-linux-musleabihf -p ccos   # printer binary
```

Mistakes are build errors that name the file, widget and problem, e.g.
`ui/screens/home.toml widget #3 (button): 'goto setings': no screen named 'setings'`.

## Screen files

```toml
include = ["sidebar"]      # optional: widgets from ui/parts/sidebar.toml first
background = "bg"          # optional: colour name or "#RRGGBB"

[[widget]]
type = "button"
at = [209, 206, 128, 60]   # x, y, width, height (screen is 480 x 272)
text = "Home all"
tap = "gcode G28"
```

Widgets are drawn in order, later ones on top.

| Field | Meaning |
|---|---|
| `type` | `panel`, `label`, `button`, `toggle`, `icon`, `temp`, `bar` |
| `at` | `[x, y, width, height]` |
| `text` | Fixed text. Use `\n` for a line break; long label text wraps. For `temp` it is the title. |
| `value` | Live text (or level, for `bar`) supplied by the app — any `lower_snake_case` name |
| `target` | Second live value (`temp`: the target temperature line) |
| `icon`, `icon_on` | Icon names from `theme.toml`; `icon_on` is shown while `on_if` holds |
| `tap` | `goto <screen>`, `back`, `gcode <line>` (`\n` for several lines), or a command name |
| `show_if` | Flag name, `!flag`, or `@screen` (only on that screen, resolved at build time) |
| `on_if` | Same syntax; "on" draws `icon_on` and, for `toggle`/`temp`, an accent outline |
| `id` | Optional; becomes `widget_id::<screen>::<ID>` in Rust |
| `font`, `color`, `bg`, `pressed`, `accent`, `radius`, `align` | Override the defaults (`bg = "none"` = transparent, `align` = left/center/right) |

### Adding live data or commands

Using a new name in `value`, `target`, `show_if`/`on_if` or `tap` creates a new
variant in the generated `Value`, `Flag` or `Cmd` enum. The build then fails
until you handle it in Rust — which is the point: nothing on screen can be
silently unconnected.

* `Value::YourName` → add a case to `Model::text` (and `Model::level` for bars)
  in `crates/ccos/src/app.rs` (fill it from the core in `Core::publish`,
  `crates/ccos/src/core.rs`, or from the G-code thread).
* `Flag::YourName` → add a case to `Model::flag` in the same file.
* `Cmd::YourName` → handle it in `Runtime::command` in `crates/ccos/src/runtime.rs`.

## theme.toml

* `[colors]`: names usable everywhere a colour is expected.
* `[fonts]`: `name = { file, size, chars }`. Every font gets ASCII,
  `° ± × · — – − → ← ↑ ↓ … • ✓ ✗` and every character used in any screen's
  fixed text; add more with `chars = "..."`. Glyphs the TTF lacks are reported
  as build warnings.
* `[icons]`: `name = "icons/x.png"` or `{ file = "...", tint = "colour" }`.
  Single-colour icons are stored as 4-bit masks (8x smaller than RGBA) and can
  be re-tinted; others are stored as RGBA. Unused icons are not baked.
* `[defaults]`: start screen, default colours, fonts and corner radius.

## Cost

The current UI — 4 screens, 64 widgets, 4 fonts × 110 glyphs, 18 icons — adds
about 46 KB to the binary.
