//! Build-time UI compiler.
//!
//! Reads an application's `ui/` folder:
//!
//! ```text
//! ui/theme.toml          colours, fonts, icons, defaults
//! ui/screens/<name>.toml one file per screen
//! ui/parts/<name>.toml   widget groups that screens `include`
//! ui/fonts/*.ttf         rasterised here, never shipped
//! ui/icons/*.png         converted here, never shipped
//! ```
//!
//! and writes `ui.rs` (plus raw bitmap blobs) into `OUT_DIR`. The generated
//! code is plain `static` data for `cc-ui`; see that crate for the runtime.
//!
//! Call from `build.rs`:
//!
//! ```ignore
//! fn main() { cc_ui_codegen::build("ui").unwrap_or_else(|e| panic!("{e}")); }
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use serde::Deserialize;

pub const SCREEN_W: i16 = 480;
pub const SCREEN_H: i16 = 272;
/// Must match `cc_ui::screen::MAX_WIDGETS`.
pub const MAX_WIDGETS: usize = 96;
/// Always baked into every font so live values can use them.
pub const DEFAULT_CHARSET: &str =
    " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~°±×·—–−→←↑↓…•✓✗";

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Theme {
    #[serde(default)]
    colors: BTreeMap<String, String>,
    fonts: BTreeMap<String, FontSpec>,
    #[serde(default)]
    icons: BTreeMap<String, IconSpec>,
    defaults: Defaults,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct FontSpec {
    file: String,
    size: f32,
    /// Extra characters to bake (besides DEFAULT_CHARSET and all static text).
    #[serde(default)]
    chars: String,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum IconSpec {
    Path(String),
    Full {
        file: String,
        #[serde(default)]
        tint: Option<String>,
    },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Defaults {
    start: String,
    background: String,
    text: String,
    dim: String,
    panel: String,
    pressed: String,
    accent: String,
    font: String,
    small_font: String,
    big_font: String,
    #[serde(default = "default_radius")]
    radius: i16,
}

fn default_radius() -> i16 {
    10
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct ScreenFile {
    #[serde(default)]
    background: Option<String>,
    #[serde(default)]
    include: Vec<String>,
    #[serde(default, rename = "widget")]
    widgets: Vec<WidgetSpec>,
}

#[derive(Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct WidgetSpec {
    #[serde(rename = "type")]
    kind: String,
    at: [i16; 4],
    #[serde(default)]
    id: Option<String>,
    #[serde(default)]
    text: Option<String>,
    #[serde(default)]
    value: Option<String>,
    #[serde(default)]
    target: Option<String>,
    #[serde(default)]
    icon: Option<String>,
    #[serde(default)]
    icon_on: Option<String>,
    #[serde(default)]
    font: Option<String>,
    #[serde(default)]
    color: Option<String>,
    #[serde(default)]
    bg: Option<String>,
    #[serde(default)]
    pressed: Option<String>,
    #[serde(default)]
    accent: Option<String>,
    #[serde(default)]
    radius: Option<i16>,
    #[serde(default)]
    align: Option<String>,
    #[serde(default)]
    tap: Option<String>,
    #[serde(default)]
    show_if: Option<String>,
    #[serde(default)]
    on_if: Option<String>,
}

/// Entry point for `build.rs`: compiles `ui_dir` (relative to the crate) into
/// `$OUT_DIR/ui.rs` and prints `cargo:rerun-if-changed` lines.
pub fn build(ui_dir: &str) -> Result<(), String> {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").map_err(|e| e.to_string())?;
    let out = std::env::var("OUT_DIR").map_err(|e| e.to_string())?;
    let ui = Path::new(&manifest).join(ui_dir);
    let gen = generate(&ui, Path::new(&out))?;
    for f in &gen.inputs {
        println!("cargo:rerun-if-changed={}", f.display());
    }
    for w in &gen.warnings {
        println!("cargo:warning={w}");
    }
    Ok(())
}

#[derive(Debug)]
pub struct Generated {
    pub inputs: Vec<PathBuf>,
    pub warnings: Vec<String>,
    pub code: String,
}

fn camel(s: &str) -> String {
    s.split('_').filter(|p| !p.is_empty()).map(|p| p[..1].to_uppercase() + &p[1..]).collect()
}

fn ident_ok(s: &str) -> bool {
    let mut c = s.chars();
    matches!(c.next(), Some('a'..='z')) && c.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
}

struct Ctx<'a> {
    theme: &'a Theme,
    colors: BTreeMap<String, u32>,
    values: BTreeSet<String>,
    flags: BTreeSet<String>,
    cmds: BTreeSet<String>,
    screens: Vec<String>,
    used_icons: BTreeSet<String>,
    used_fonts: BTreeSet<String>,
}

impl Ctx<'_> {
    fn color(&self, s: &str, at: &str) -> Result<u32, String> {
        if let Some(h) = s.strip_prefix('#') {
            if h.len() == 6 {
                if let Ok(v) = u32::from_str_radix(h, 16) {
                    return Ok(v);
                }
            }
            return Err(format!("{at}: bad colour '{s}' (use #RRGGBB or a name from [colors])"));
        }
        self.colors.get(s).copied().ok_or_else(|| format!("{at}: unknown colour '{s}' (defined: {:?})", self.colors.keys().collect::<Vec<_>>()))
    }

    fn font(&mut self, name: &str, at: &str) -> Result<String, String> {
        if !self.theme.fonts.contains_key(name) {
            return Err(format!("{at}: unknown font '{name}' (defined: {:?})", self.theme.fonts.keys().collect::<Vec<_>>()));
        }
        self.used_fonts.insert(name.to_string());
        Ok(format!("&FONT_{}", name.to_uppercase()))
    }

    fn icon(&mut self, name: &str, at: &str) -> Result<String, String> {
        if !self.theme.icons.contains_key(name) {
            return Err(format!("{at}: unknown icon '{name}' (defined: {:?})", self.theme.icons.keys().collect::<Vec<_>>()));
        }
        self.used_icons.insert(name.to_string());
        Ok(format!("Some(&ICON_{})", name.to_uppercase()))
    }

    fn name(&self, kind: &str, s: &str, at: &str) -> Result<String, String> {
        if ident_ok(s) {
            Ok(s.to_string())
        } else {
            Err(format!("{at}: {kind} '{s}' must be lower_snake_case"))
        }
    }

    fn cond(&mut self, s: &str, at: &str, screen: &str) -> Result<(String, bool), String> {
        // "@screen" is resolved at build time (true when on that screen).
        if let Some(sc) = s.strip_prefix('@') {
            if !self.screens.iter().any(|x| x == sc) {
                return Err(format!("{at}: unknown screen '@{sc}'"));
            }
            return Ok(("None".into(), sc == screen));
        }
        let (neg, f) = match s.strip_prefix('!') {
            Some(f) => (true, f.trim()),
            None => (false, s.trim()),
        };
        let f = self.name("flag", f, at)?;
        self.flags.insert(f.clone());
        Ok((format!("Some(Cond {{ flag: Flag::{}, negate: {neg} }})", camel(&f)), false))
    }

    fn action(&mut self, s: &str, at: &str) -> Result<String, String> {
        let s = s.trim();
        if s == "back" {
            return Ok("Some(Action::Back)".into());
        }
        if let Some(t) = s.strip_prefix("goto ") {
            let t = t.trim();
            let i = self.screens.iter().position(|x| x == t).ok_or_else(|| format!("{at}: 'goto {t}': no screen named '{t}' (screens: {:?})", self.screens))?;
            return Ok(format!("Some(Action::Goto({i}))"));
        }
        if let Some(g) = s.strip_prefix("gcode ") {
            return Ok(format!("Some(Action::Gcode({:?}))", g.trim()));
        }
        let c = self.name("command", s, at).map_err(|e| format!("{e}; actions are 'goto <screen>', 'back', 'gcode <line>' or a command name"))?;
        self.cmds.insert(c.clone());
        Ok(format!("Some(Action::Cmd(Cmd::{}))", camel(&c)))
    }
}

/// Compile `ui` into `out/ui.rs`. Returns the generated code and the files read.
pub fn generate(ui: &Path, out: &Path) -> Result<Generated, String> {
    let mut inputs = vec![ui.join("theme.toml"), ui.join("screens"), ui.join("parts")];
    let mut warnings = Vec::new();
    let read = |p: &Path| std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()));
    let theme: Theme = toml::from_str(&read(&ui.join("theme.toml"))?).map_err(|e| format!("ui/theme.toml: {e}"))?;

    // Screens, sorted by file name for stable indices.
    let mut screen_files: Vec<(String, ScreenFile)> = Vec::new();
    let mut entries: Vec<_> = std::fs::read_dir(ui.join("screens"))
        .map_err(|e| format!("{}: {e}", ui.join("screens").display()))?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "toml"))
        .collect();
    entries.sort();
    for p in entries {
        let name = p.file_stem().unwrap().to_string_lossy().to_string();
        if !ident_ok(&name) {
            return Err(format!("{}: screen file names must be lower_snake_case", p.display()));
        }
        let sf: ScreenFile = toml::from_str(&read(&p)?).map_err(|e| format!("ui/screens/{name}.toml: {e}"))?;
        inputs.push(p);
        screen_files.push((name, sf));
    }
    if screen_files.is_empty() {
        return Err("ui/screens/ has no .toml files".into());
    }

    let mut ctx = Ctx {
        theme: &theme,
        colors: BTreeMap::new(),
        values: BTreeSet::new(),
        flags: BTreeSet::new(),
        cmds: BTreeSet::new(),
        screens: screen_files.iter().map(|(n, _)| n.clone()).collect(),
        used_icons: BTreeSet::new(),
        used_fonts: BTreeSet::new(),
    };
    for (k, v) in &theme.colors {
        let c = ctx.color(v, &format!("[colors] {k}"))?;
        ctx.colors.insert(k.clone(), c);
    }
    let d = &theme.defaults;
    let at = "[defaults]";
    let (c_bg, c_text, c_dim, c_panel, c_pressed, c_accent) =
        (ctx.color(&d.background, at)?, ctx.color(&d.text, at)?, ctx.color(&d.dim, at)?, ctx.color(&d.panel, at)?, ctx.color(&d.pressed, at)?, ctx.color(&d.accent, at)?);
    let start = ctx.screens.iter().position(|s| *s == d.start).ok_or_else(|| format!("[defaults] start = '{}': no such screen", d.start))?;
    let default_font = ctx.font(&d.font, at)?;
    let small_font = ctx.font(&d.small_font, at)?;
    let big_font = ctx.font(&d.big_font, at)?;

    let mut static_chars: BTreeSet<char> = BTreeSet::new();
    let mut screens_code = String::new();
    let mut ids_code = String::new();
    for (sname, sf) in &screen_files {
        let mut widgets: Vec<(String, WidgetSpec)> = Vec::new();
        for inc in &sf.include {
            let p = ui.join("parts").join(format!("{inc}.toml"));
            let part: ScreenFile = toml::from_str(&read(&p).map_err(|e| format!("ui/screens/{sname}.toml include '{inc}': {e}"))?)
                .map_err(|e| format!("ui/parts/{inc}.toml: {e}"))?;
            inputs.push(p);
            widgets.extend(part.widgets.into_iter().map(|w| (format!("ui/parts/{inc}.toml"), w)));
        }
        widgets.extend(sf.widgets.iter().cloned().map(|w| (format!("ui/screens/{sname}.toml"), w)));
        if widgets.len() > MAX_WIDGETS {
            return Err(format!("screen '{sname}' has {} widgets, max {MAX_WIDGETS}", widgets.len()));
        }
        let bg = match &sf.background {
            Some(b) => ctx.color(b, &format!("ui/screens/{sname}.toml background"))?,
            None => c_bg,
        };
        let mut wcode = String::new();
        let mut ids = BTreeMap::new();
        let mut emitted = 0usize;
        for (n, (file, w)) in widgets.iter().enumerate() {
            let at = format!("{file} widget #{} ({})", n + 1, w.id.as_deref().unwrap_or(&w.kind));
            let show_if = match &w.show_if {
                Some(s) => {
                    let (c, stat) = ctx.cond(s, &at, sname)?;
                    if c == "None" && !stat {
                        continue; // statically hidden on this screen
                    }
                    c
                }
                None => "None".into(),
            };
            let kind = match w.kind.as_str() {
                "panel" => "Panel",
                "label" => "Label",
                "button" => "Button",
                "icon" => "Icon",
                "temp" => "Temp",
                "bar" => "Bar",
                "toggle" => "Toggle",
                "heatmap" => "Heatmap",
                k => return Err(format!("{at}: unknown type '{k}' (panel, label, button, icon, temp, bar, toggle, heatmap)")),
            };
            let [x, y, ww, hh] = w.at;
            if ww <= 0 || hh <= 0 || x < 0 || y < 0 || x + ww > SCREEN_W || y + hh > SCREEN_H {
                return Err(format!("{at}: at = [{x}, {y}, {ww}, {hh}] is outside the {SCREEN_W}x{SCREEN_H} screen"));
            }
            if let Some(id) = &w.id {
                let id = ctx.name("id", id, &at)?;
                if ids.insert(id.clone(), emitted).is_some() {
                    return Err(format!("{at}: duplicate id '{id}' on screen '{sname}'"));
                }
            }
            if matches!(kind, "Temp" | "Bar" | "Heatmap") && w.value.is_none() {
                return Err(format!("{at}: type '{}' needs value = \"...\"", w.kind));
            }
            let text = w.text.clone().unwrap_or_default();
            static_chars.extend(text.chars());
            let value = |v: &Option<String>, ctx: &mut Ctx| -> Result<String, String> {
                Ok(match v {
                    Some(v) => {
                        let v = ctx.name("value", v, &at)?;
                        ctx.values.insert(v.clone());
                        format!("Some(Value::{})", camel(&v))
                    }
                    None => "None".into(),
                })
            };
            let v1 = value(&w.value, &mut ctx)?;
            let v2 = value(&w.target, &mut ctx)?;
            let icon = match &w.icon {
                Some(i) => ctx.icon(i, &at)?,
                None => "None".into(),
            };
            let icon_on = match &w.icon_on {
                Some(i) => ctx.icon(i, &at)?,
                None => "None".into(),
            };
            let font = match &w.font {
                Some(f) => ctx.font(f, &at)?,
                None if kind == "Temp" => big_font.clone(),
                None => default_font.clone(),
            };
            let fg = match &w.color {
                Some(c) => ctx.color(c, &at)?,
                None => c_text,
            };
            let bg_w = match w.bg.as_deref() {
                Some("none") => "None".to_string(),
                Some(c) => format!("Some(0x{:06x})", ctx.color(c, &at)?),
                None if matches!(kind, "Panel" | "Button" | "Toggle" | "Temp" | "Heatmap") => format!("Some(0x{c_panel:06x})"),
                None => "None".into(),
            };
            let pressed = match &w.pressed {
                Some(c) => ctx.color(c, &at)?,
                None => c_pressed,
            };
            let accent = match &w.accent {
                Some(c) => ctx.color(c, &at)?,
                None => c_accent,
            };
            let align = match w.align.as_deref() {
                None if matches!(kind, "Button" | "Toggle") => "Center",
                None | Some("left") => "Left",
                Some("center") => "Center",
                Some("right") => "Right",
                Some(a) => return Err(format!("{at}: align = '{a}' (left, center, right)")),
            };
            let tap = match &w.tap {
                Some(t) => ctx.action(t, &at)?,
                None => "None".into(),
            };
            let (on_if, on) = match &w.on_if {
                Some(s) => ctx.cond(s, &at, sname)?,
                None => ("None".into(), false),
            };
            writeln!(
                wcode,
                "        Widget {{ id: {:?}, kind: Kind::{kind}, rect: Rect::new({x}, {y}, {ww}, {hh}), text: {text:?}, value: {v1}, value2: {v2}, \
icon: {icon}, icon_on: {icon_on}, font: {font}, font_small: {small_font}, fg: 0x{fg:06x}, fg_dim: 0x{c_dim:06x}, bg: {bg_w}, \
bg_pressed: 0x{pressed:06x}, accent: 0x{accent:06x}, radius: {}, align: Align::{align}, tap: {tap}, show_if: {show_if}, on_if: {on_if}, on: {on} }},",
                w.id.as_deref().unwrap_or(""),
                w.radius.unwrap_or(d.radius),
            )
            .unwrap();
            emitted += 1;
        }
        writeln!(screens_code, "    Screen {{ name: {sname:?}, bg: 0x{bg:06x}, widgets: &[\n{wcode}    ] }},").unwrap();
        if !ids.is_empty() {
            writeln!(ids_code, "    pub mod {sname} {{").unwrap();
            for (id, i) in ids {
                writeln!(ids_code, "        pub const {}: usize = {i};", id.to_uppercase()).unwrap();
            }
            writeln!(ids_code, "    }}").unwrap();
        }
    }

    // ---------- code ----------
    let mut code = String::new();
    code += "// @generated by cc-ui-codegen from ui/*.toml — edit those, not this.\n";
    code += "#[allow(unused_imports)]\nuse cc_ui::{Action, Align, Cond, Font, Glyph, Icon, IconData, Kind, Rect, Screen, Widget};\n\n";
    code += "/// Colours from `[colors]`.\n#[allow(dead_code)]\npub mod color {\n";
    for (k, v) in &ctx.colors {
        writeln!(code, "    pub const {}: u32 = 0x{v:06x};", k.to_uppercase()).unwrap();
    }
    code += "}\n\n";

    let gen_enum = |code: &mut String, name: &str, doc: &str, items: &BTreeSet<String>| {
        writeln!(code, "/// {doc}\n#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]\npub enum {name} {{").unwrap();
        for i in items {
            writeln!(code, "    {},", camel(i)).unwrap();
        }
        writeln!(code, "}}\n\nimpl {name} {{\n    pub const ALL: &'static [{name}] = &[{}];", items.iter().map(|i| format!("{name}::{}", camel(i))).collect::<Vec<_>>().join(", ")).unwrap();
        writeln!(code, "    pub fn name(self) -> &'static str {{\n        match self {{").unwrap();
        for i in items {
            writeln!(code, "            {name}::{} => {i:?},", camel(i)).unwrap();
        }
        if items.is_empty() {
            code.push_str("            _ => unreachable!(),\n");
        }
        code.push_str("        }\n    }\n}\n\n");
    };
    gen_enum(&mut code, "Value", "Live values used by `value =` / `target =`. Implement them in `Model::text`.", &ctx.values);
    gen_enum(&mut code, "Flag", "Conditions used by `show_if` / `on_if`. Implement them in `Model::flag`.", &ctx.flags);
    gen_enum(&mut code, "Cmd", "App commands used by `tap =`.", &ctx.cmds);

    code += "/// Screen indices.\n#[allow(dead_code)]\npub mod screen {\n";
    for (i, s) in ctx.screens.iter().enumerate() {
        writeln!(code, "    pub const {}: u8 = {i};", s.to_uppercase()).unwrap();
    }
    code += "}\n\n/// Widget indices for widgets with an `id`, per screen.\n#[allow(dead_code)]\npub mod widget_id {\n";
    code += &ids_code;
    code += "}\n\n";
    writeln!(code, "pub const START: u8 = {start};\n").unwrap();

    // Fonts
    let mut chars: BTreeSet<char> = DEFAULT_CHARSET.chars().collect();
    chars.extend(static_chars.iter().filter(|c| !c.is_control()));
    let mut font_bytes = 0;
    for name in &ctx.used_fonts {
        let spec = &theme.fonts[name];
        let path = ui.join(&spec.file);
        inputs.push(path.clone());
        let data = std::fs::read(&path).map_err(|e| format!("font '{name}': {}: {e}", path.display()))?;
        let mut cs = chars.clone();
        cs.extend(spec.chars.chars());
        let (rs, blob, missing) = bake_font(name, &data, spec.size, &cs)?;
        if !missing.is_empty() {
            let used_missing: String = missing.iter().filter(|c| static_chars.contains(c)).collect();
            if !used_missing.is_empty() {
                warnings.push(format!("font '{name}' ({}) lacks characters used in screen text: {used_missing:?}", spec.file));
            }
        }
        let bin = out.join(format!("ui_font_{name}.bin"));
        std::fs::write(&bin, &blob).map_err(|e| e.to_string())?;
        font_bytes += blob.len();
        code += &rs.replace("@DATA@", &format!("include_bytes!({:?})", bin.display().to_string()));
    }

    // Icons
    let mut icon_bytes = 0;
    for name in &ctx.used_icons {
        let (file, tint) = match &theme.icons[name] {
            IconSpec::Path(p) => (p.clone(), None),
            IconSpec::Full { file, tint } => (file.clone(), tint.clone()),
        };
        let tint = tint.map(|t| ctx.color(&t, &format!("[icons] {name}"))).transpose()?;
        let path = ui.join(&file);
        inputs.push(path.clone());
        let (w, h, rgba) = load_png(&path)?;
        let (kind, blob) = bake_icon(w, h, &rgba, tint);
        let bin = out.join(format!("ui_icon_{name}.bin"));
        std::fs::write(&bin, &blob).map_err(|e| e.to_string())?;
        icon_bytes += blob.len();
        let inc = format!("include_bytes!({:?})", bin.display().to_string());
        let data = match kind {
            Some(color) => format!("IconData::Mask {{ color: 0x{color:06x}, data: {inc} }}"),
            None => format!("IconData::Rgba({inc})"),
        };
        writeln!(code, "pub static ICON_{}: Icon = Icon {{ w: {w}, h: {h}, data: {data} }};", name.to_uppercase()).unwrap();
    }
    for name in theme.icons.keys().filter(|k| !ctx.used_icons.contains(*k)) {
        warnings.push(format!("icon '{name}' is defined but not used (not baked)"));
    }

    writeln!(code, "\n/// All screens, indexed by `screen::*`.\npub static SCREENS: [Screen<Cmd, Value, Flag>; {}] = [\n{screens_code}];", ctx.screens.len()).unwrap();
    writeln!(code, "\n/// Size of baked UI assets in bytes (fonts, icons).\npub const ASSET_BYTES: (usize, usize) = ({font_bytes}, {icon_bytes});").unwrap();

    std::fs::write(out.join("ui.rs"), &code).map_err(|e| e.to_string())?;
    Ok(Generated { inputs, warnings, code })
}

/// Rasterise `chars` to 4-bit glyphs. Returns (rust code with `@DATA@`
/// placeholder, bitmap blob, characters the font lacks).
fn bake_font(name: &str, ttf: &[u8], px: f32, chars: &BTreeSet<char>) -> Result<(String, Vec<u8>, Vec<char>), String> {
    let font = fontdue::Font::from_bytes(ttf, fontdue::FontSettings { scale: px, ..Default::default() }).map_err(|e| format!("font '{name}': {e}"))?;
    let lm = font.horizontal_line_metrics(px).ok_or_else(|| format!("font '{name}': no horizontal metrics"))?;
    let ascent = lm.ascent.round() as i32;
    let line_height = lm.new_line_size.ceil() as i32;
    if line_height > 255 {
        return Err(format!("font '{name}': size {px} too large"));
    }
    let mut blob = Vec::new();
    let mut glyphs = String::new();
    let mut ascii = [u16::MAX; 96];
    let mut missing = Vec::new();
    let mut n = 0u16;
    for &ch in chars {
        if ch != ' ' && font.lookup_glyph_index(ch) == 0 {
            missing.push(ch);
            continue;
        }
        let (m, bmp) = font.rasterize(ch, px);
        let x = m.xmin;
        let y = ascent - (m.height as i32 + m.ymin);
        let adv = m.advance_width.round() as i32;
        if !(-128..128).contains(&x) || !(-128..128).contains(&y) || m.width > 255 || m.height > 255 || !(0..256).contains(&adv) {
            return Err(format!("font '{name}': glyph {ch:?} too large for size {px}"));
        }
        let off = blob.len();
        let stride = m.width.div_ceil(2);
        for row in 0..m.height {
            for b in 0..stride {
                let a0 = bmp[row * m.width + 2 * b];
                let a1 = if 2 * b + 1 < m.width { bmp[row * m.width + 2 * b + 1] } else { 0 };
                let q = |a: u8| ((a as u32 + 8) / 17).min(15) as u8;
                blob.push(q(a0) << 4 | q(a1));
            }
        }
        writeln!(glyphs, "    Glyph {{ ch: {}, x: {x}, y: {y}, w: {}, h: {}, adv: {adv}, off: {off} }},", ch as u32, m.width, m.height).unwrap();
        if (0x20..0x80).contains(&(ch as u32)) {
            ascii[ch as usize - 0x20] = n;
        }
        n += 1;
    }
    let up = name.to_uppercase();
    let mut rs = String::new();
    writeln!(rs, "static GLYPHS_{up}: [Glyph; {n}] = [\n{glyphs}];").unwrap();
    writeln!(
        rs,
        "/// Font `{name}`: {px}px, {n} glyphs, {} bytes of 4-bit bitmaps.\npub static FONT_{up}: Font = Font {{ line_height: {line_height}, glyphs: &GLYPHS_{up}, ascii: {ascii:?}, data: @DATA@ }};\n",
        blob.len()
    )
    .unwrap();
    Ok((rs, blob, missing))
}

fn load_png(path: &Path) -> Result<(u16, u16, Vec<u8>), String> {
    let f = std::fs::File::open(path).map_err(|e| format!("icon {}: {e}", path.display()))?;
    let mut dec = png::Decoder::new(std::io::BufReader::new(f));
    dec.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
    let mut r = dec.read_info().map_err(|e| format!("icon {}: {e}", path.display()))?;
    let mut buf = vec![0; r.output_buffer_size()];
    let info = r.next_frame(&mut buf).map_err(|e| format!("icon {}: {e}", path.display()))?;
    let px = (info.width * info.height) as usize;
    let rgba: Vec<u8> = match info.color_type {
        png::ColorType::Rgba => buf[..px * 4].to_vec(),
        png::ColorType::Rgb => buf[..px * 3].chunks(3).flat_map(|c| [c[0], c[1], c[2], 255]).collect(),
        png::ColorType::GrayscaleAlpha => buf[..px * 2].chunks(2).flat_map(|c| [c[0], c[0], c[0], c[1]]).collect(),
        png::ColorType::Grayscale => buf[..px].iter().flat_map(|&g| [g, g, g, 255]).collect(),
        png::ColorType::Indexed => return Err(format!("icon {}: indexed PNG", path.display())),
    };
    if info.width > 480 || info.height > 272 {
        return Err(format!("icon {}: {}x{} is larger than the screen", path.display(), info.width, info.height));
    }
    Ok((info.width as u16, info.height as u16, rgba))
}

/// Single-colour icons become a 4-bit mask (returns `Some(color)`),
/// others stay RGBA8.
fn bake_icon(w: u16, h: u16, rgba: &[u8], tint: Option<u32>) -> (Option<u32>, Vec<u8>) {
    let (mut sr, mut sg, mut sb, mut sa) = (0u64, 0u64, 0u64, 0u64);
    for p in rgba.chunks(4) {
        let a = p[3] as u64;
        sr += p[0] as u64 * a;
        sg += p[1] as u64 * a;
        sb += p[2] as u64 * a;
        sa += a;
    }
    let avg = if sa > 0 { [(sr / sa) as i32, (sg / sa) as i32, (sb / sa) as i32] } else { [255, 255, 255] };
    let mono = tint.is_some()
        || rgba.chunks(4).all(|p| p[3] < 64 || (0..3).all(|i| (p[i] as i32 - avg[i]).abs() <= 40));
    if mono {
        let color = tint.unwrap_or(((avg[0] as u32) << 16) | ((avg[1] as u32) << 8) | avg[2] as u32);
        let stride = (w as usize).div_ceil(2);
        let mut out = vec![0u8; stride * h as usize];
        for y in 0..h as usize {
            for x in 0..w as usize {
                let a = rgba[(y * w as usize + x) * 4 + 3];
                let q = ((a as u32 + 8) / 17).min(15) as u8;
                out[y * stride + x / 2] |= if x % 2 == 0 { q << 4 } else { q };
            }
        }
        (Some(color), out)
    } else {
        (None, rgba.to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names() {
        assert_eq!(camel("nozzle_temp"), "NozzleTemp");
        assert!(ident_ok("home_2"));
        assert!(!ident_ok("Home"));
        assert!(!ident_ok("2x"));
    }

    fn try_ui(screen: &str) -> Result<Generated, String> {
        static N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = N.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        let dir = std::env::temp_dir().join(format!("ccui-test-{}-{n}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(dir.join("screens")).unwrap();
        std::fs::create_dir_all(dir.join("fonts")).unwrap();
        let font = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../ui/fonts/DejaVuSans.ttf");
        std::fs::copy(font, dir.join("fonts/f.ttf")).unwrap();
        std::fs::write(
            dir.join("theme.toml"),
            r##"
[colors]
bg = "#000000"
[fonts]
f = { file = "fonts/f.ttf", size = 12 }
[defaults]
start = "main"
background = "bg"
text = "#ffffff"
dim = "#888888"
panel = "#333333"
pressed = "#444444"
accent = "#00ff00"
font = "f"
small_font = "f"
big_font = "f"
"##,
        )
        .unwrap();
        std::fs::write(dir.join("screens/main.toml"), screen).unwrap();
        let r = generate(&dir, &dir);
        let _ = std::fs::remove_dir_all(&dir);
        r
    }

    #[test]
    fn good_screen_generates() {
        let g = try_ui("[[widget]]\ntype = \"button\"\nat = [0, 0, 100, 40]\ntext = \"Hi\"\ntap = \"do_it\"\n").unwrap();
        assert!(g.code.contains("Cmd::DoIt"));
        assert!(g.code.contains("pub static SCREENS"));
    }

    #[test]
    fn helpful_errors() {
        let e = try_ui("[[widget]]\ntype = \"button\"\nat = [0, 0, 100, 40]\ntap = \"goto nowhere\"\n").unwrap_err();
        assert!(e.contains("no screen named 'nowhere'"), "{e}");
        let e = try_ui("[[widget]]\ntype = \"buton\"\nat = [0, 0, 10, 10]\n").unwrap_err();
        assert!(e.contains("unknown type 'buton'"), "{e}");
        let e = try_ui("[[widget]]\ntype = \"label\"\nat = [400, 0, 100, 10]\n").unwrap_err();
        assert!(e.contains("outside the 480x272 screen"), "{e}");
        let e = try_ui("[[widget]]\ntype = \"label\"\nat = [0, 0, 10, 10]\ncolour = \"red\"\n").unwrap_err();
        assert!(e.contains("unknown field"), "{e}");
        let e = try_ui("[[widget]]\ntype = \"temp\"\nat = [0, 0, 10, 10]\n").unwrap_err();
        assert!(e.contains("needs value"), "{e}");
    }

    #[test]
    fn mono_detection() {
        let white = [255u8, 255, 255, 255, 255, 255, 255, 0];
        let (c, data) = bake_icon(2, 1, &white, None);
        assert_eq!(c, Some(0xffffff));
        assert_eq!(data, vec![0xf0]);
        let multi = [255u8, 0, 0, 255, 0, 0, 255, 255];
        assert_eq!(bake_icon(2, 1, &multi, None).0, None);
    }
}
