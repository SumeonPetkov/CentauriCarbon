//! `ccres` — PC-side tool for Centauri Carbon resources and firmware images.

use std::path::{Path, PathBuf};

use cc_gui::image::ImageStore;
use cc_gui::preview::{PreviewOptions, Previewer};
use cc_gui::text::Font;
use cc_res::mcu_image::McuImage;
use cc_res::translation::TranslationDb;
use cc_res::widget::{WidgetDb, WidgetType};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

const USAGE: &str = "\
ccres — Centauri Carbon resource tool

UI layout (widget.bin):
  ccres widget decompile <widget.bin> <out.csv|out.json>
  ccres widget compile   <in.csv|in.json> <widget.bin>
  ccres widget list      <widget.bin|widget.csv>
  ccres widget show      <widget.bin|widget.csv> <window>

UI text (translation.bin):
  ccres tr decompile <translation.bin> <out.csv|out.json>
  ccres tr compile   <in.csv|in.json> <translation.bin>
  ccres tr get       <translation.bin> <id> [lang]

Preview screens as PNG (resource dir holds widget.* translation.* images/ fonts/):
  ccres preview <resource-dir> <window|all> <out.png|out-dir> [--lang N] [--annotate] [--no-sys]

STM32 MCU update images (upgrade_extruder.bin / upgrade_sg.bin):
  ccres mcu info   <image>
  ccres mcu unpack <image> <klipper.bin>
  ccres mcu pack   <klipper.bin> <image> [--version 1.2.3] [--board 255]

Klipper protocol (fixed Centauri id table):
  ccres proto encode \"<command line>\" [seq]
  ccres proto decode <hex bytes>
  ccres proto list";

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let s: Vec<&str> = a.iter().map(String::as_str).collect();
    let r = match s.as_slice() {
        ["widget", "decompile", i, o] => widget_decompile(i, o),
        ["widget", "compile", i, o] => widget_compile(i, o),
        ["widget", "list", i] => widget_list(i),
        ["widget", "show", i, w] => widget_show(i, w.parse().unwrap_or(0)),
        ["tr", "decompile", i, o] => tr_decompile(i, o),
        ["tr", "compile", i, o] => tr_compile(i, o),
        ["tr", "get", i, id, rest @ ..] => tr_get(i, id.parse().unwrap_or(0), rest.first().and_then(|l| l.parse().ok()).unwrap_or(1)),
        ["preview", dir, win, out, rest @ ..] => preview(dir, win, out, rest),
        ["mcu", "info", i] => mcu_info(i),
        ["mcu", "unpack", i, o] => mcu_unpack(i, o),
        ["mcu", "pack", i, o, rest @ ..] => mcu_pack(i, o, rest),
        ["proto", "encode", cmd, rest @ ..] => proto_encode(cmd, rest.first().and_then(|v| v.parse().ok()).unwrap_or(0)),
        ["proto", "decode", hex @ ..] => proto_decode(&hex.join("")),
        ["proto", "list"] => proto_list(),
        _ => {
            println!("{USAGE}");
            Ok(())
        }
    };
    if let Err(e) = r {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn ext(p: &str) -> String {
    Path::new(p).extension().map(|e| e.to_string_lossy().to_lowercase()).unwrap_or_default()
}

fn load_widgets(p: &str) -> Result<WidgetDb> {
    Ok(match ext(p).as_str() {
        "csv" | "txt" | "tsv" => WidgetDb::parse_csv(&std::fs::read_to_string(p)?)?,
        "json" => serde_json::from_str(&std::fs::read_to_string(p)?)?,
        _ => WidgetDb::parse_bin(&std::fs::read(p)?)?,
    })
}

fn load_tr(p: &str) -> Result<TranslationDb> {
    Ok(match ext(p).as_str() {
        "csv" | "txt" | "tsv" => TranslationDb::parse_csv(&std::fs::read_to_string(p)?)?,
        "json" => serde_json::from_str(&std::fs::read_to_string(p)?)?,
        _ => TranslationDb::parse_bin(&std::fs::read(p)?)?,
    })
}

fn widget_decompile(i: &str, o: &str) -> Result<()> {
    let db = load_widgets(i)?;
    let out = if ext(o) == "json" { serde_json::to_string_pretty(&db)? } else { db.to_csv() };
    std::fs::write(o, out)?;
    println!("{} windows, {} widgets -> {o}", db.windows.len(), db.windows.iter().map(|w| w.widgets.len()).sum::<usize>());
    Ok(())
}

fn widget_compile(i: &str, o: &str) -> Result<()> {
    let db = load_widgets(i)?;
    let bin = db.to_bin();
    std::fs::write(o, &bin)?;
    println!("{} windows -> {o} ({} bytes)", db.windows.len(), bin.len());
    Ok(())
}

fn widget_list(i: &str) -> Result<()> {
    let db = load_widgets(i)?;
    for w in &db.windows {
        let first = w.widgets.first().map(|x| x.comment.as_str()).unwrap_or("");
        println!("window {:3}: {:3} widgets  {first}", w.index, w.widgets.len());
    }
    Ok(())
}

fn widget_show(i: &str, win: u8) -> Result<()> {
    let db = load_widgets(i)?;
    let w = db.window(win).ok_or("no such window")?;
    for x in &w.widgets {
        let kind = WidgetType::from_u16(x.kind).map(|k| format!("{k:?}")).unwrap_or(format!("type{}", x.kind));
        println!("#{:<3} {:9} x={:<3} y={:<3} w={:<3} h={:<3} cb={} ev={}  {}", x.index, kind, x.x, x.y, x.w, x.h, x.callback, x.event, x.comment);
        for f in &x.flags {
            println!("       flag {:08x} {}", f.flag, if f.mask != 0 { "add" } else { "clear" });
        }
        for s in &x.styles {
            let v = if s.is_color() { format!("#{:06X}", s.value as u32 & 0xffffff) } else { s.value.to_string() };
            let sel = if s.selector != 0 { format!(" [selector {:#x}]", s.selector) } else { String::new() };
            println!("       {} = {v}{sel}", s.name());
        }
    }
    Ok(())
}

fn tr_decompile(i: &str, o: &str) -> Result<()> {
    let db = load_tr(i)?;
    let out = if ext(o) == "json" { serde_json::to_string_pretty(&db)? } else { db.to_csv() };
    std::fs::write(o, out)?;
    println!("{} languages x {} strings -> {o}", db.langs.len(), db.langs.first().map_or(0, Vec::len));
    Ok(())
}

fn tr_compile(i: &str, o: &str) -> Result<()> {
    let db = load_tr(i)?;
    let bin = db.to_bin();
    std::fs::write(o, &bin)?;
    println!("-> {o} ({} bytes)", bin.len());
    Ok(())
}

fn tr_get(i: &str, id: u16, lang: usize) -> Result<()> {
    println!("{}", load_tr(i)?.get(lang, id).ok_or("no such string")?);
    Ok(())
}

fn preview(dir: &str, win: &str, out: &str, rest: &[&str]) -> Result<()> {
    let d = PathBuf::from(dir);
    let pick = |a: &str, b: &str| if d.join(a).exists() { d.join(a) } else { d.join(b) };
    let db = load_widgets(pick("widget.bin", "widget.csv").to_str().unwrap())?;
    let tr = load_tr(pick("translation.bin", "translation.csv").to_str().unwrap()).ok();
    let mut images = ImageStore::new(d.join("images"));
    let mut font = Font::load(d.join("fonts/font.ttc").to_str().unwrap(), 0)?;
    let lang = rest.iter().position(|a| *a == "--lang").and_then(|i| rest.get(i + 1)).and_then(|v| v.parse().ok()).unwrap_or(1);
    let opt = PreviewOptions { lang, with_sys_layer: !rest.contains(&"--no-sys"), annotate: rest.contains(&"--annotate") };
    let mut pv = Previewer { db: &db, tr: tr.as_ref(), images: &mut images, font: &mut font };
    let windows: Vec<u8> = if win == "all" { db.windows.iter().map(|w| w.index).collect() } else { vec![win.parse()?] };
    if windows.len() > 1 {
        std::fs::create_dir_all(out)?;
    }
    for w in windows {
        let c = pv.render(w, &opt)?;
        let path = if win == "all" { format!("{out}/window_{w:02}.png") } else { out.to_string() };
        std::fs::write(&path, c.to_png())?;
        println!("window {w} -> {path}");
    }
    Ok(())
}

fn mcu_info(i: &str) -> Result<()> {
    let img = McuImage::parse(&std::fs::read(i)?)?;
    let (sp, rst) = img.vector_table().unwrap_or((0, 0));
    println!("version  {}.{}.{}", img.version.0, img.version.1, img.version.2);
    println!("board    {}", img.board);
    println!("payload  {} bytes, MD5 ok", img.payload.len());
    println!("vectors  SP={sp:#010x} Reset={rst:#010x} ({})", if img.looks_valid() { "plausible STM32F401 app @0x0800C000" } else { "unexpected" });
    Ok(())
}

fn mcu_unpack(i: &str, o: &str) -> Result<()> {
    let img = McuImage::parse(&std::fs::read(i)?)?;
    std::fs::write(o, &img.payload)?;
    println!("{} bytes -> {o}", img.payload.len());
    Ok(())
}

fn mcu_pack(i: &str, o: &str, rest: &[&str]) -> Result<()> {
    let get = |k: &str| rest.iter().position(|a| *a == k).and_then(|p| rest.get(p + 1)).copied();
    let v: Vec<u8> = get("--version").unwrap_or("1.0.0").split('.').map(|x| x.parse().unwrap_or(0)).collect();
    let board = get("--board").map(|b| b.parse()).transpose()?.unwrap_or(255);
    let img = McuImage::new(std::fs::read(i)?, (v[0], *v.get(1).unwrap_or(&0), *v.get(2).unwrap_or(&0)), board);
    if !img.looks_valid() {
        eprintln!("warning: payload does not look like an STM32 app linked at 0x0800C000");
    }
    std::fs::write(o, img.to_bytes())?;
    println!("-> {o}");
    Ok(())
}

fn proto_encode(cmd: &str, seq: u8) -> Result<()> {
    let d = cc_proto::Dictionary::centauri();
    let f = cc_proto::encode_frame(seq, &d.encode_str(cmd)?)?;
    println!("{}", f.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" "));
    Ok(())
}

fn proto_decode(hex: &str) -> Result<()> {
    let clean: String = hex.chars().filter(|c| c.is_ascii_hexdigit()).collect();
    let bytes: Vec<u8> = (0..clean.len() / 2).map(|i| u8::from_str_radix(&clean[2 * i..2 * i + 2], 16)).collect::<std::result::Result<_, _>>()?;
    let d = cc_proto::Dictionary::centauri();
    let mut dec = cc_proto::FrameDecoder::new();
    dec.push(&bytes);
    let mut any = false;
    while let Some(f) = dec.next_frame() {
        any = true;
        if f.is_ack() {
            println!("seq {:2}: ack", f.seq);
        }
        for m in d.decode_payload(&f.payload)? {
            println!("seq {:2}: {m}", f.seq);
        }
    }
    if !any {
        // Bare payload without framing
        for m in d.decode_payload(&bytes)? {
            println!("{m}");
        }
    }
    Ok(())
}

fn proto_list() -> Result<()> {
    let d = cc_proto::Dictionary::centauri();
    let mut f: Vec<_> = d.formats().map(|f| (f.id, f.format.clone())).collect();
    f.sort();
    for (id, s) in f {
        println!("{id:3} {s}");
    }
    Ok(())
}
