//! `widget.bin`: the stock UI layout database.
//!
//! ```text
//! | bin_header (16) | window_header (8) x N | per window: widget_header (20), flags (8) x f, styles (12) x s, ... |
//! ```
//! All little-endian, packed. The screen is 480x272.
//!
//! A style `prop` word packs three things:
//! * bits 0..15  — an LVGL 8 `lv_style_prop_t` (id in bits 0..7, flags above),
//! * bit 16      — "custom" property interpreted by Elegoo's loader
//!   (image index, text id, font, parent, relative alignment),
//! * bits 17..19 — which sub-object of the widget it applies to (a button is
//!   `0` = button, `1` = its image, `2` = its label).

use serde::{Deserialize, Serialize};

use crate::csvutil::{fgets_lines, scan_dec, scan_hex, strtok_tabs};
use crate::{fmt_err, Error, Rd};

pub const MAGIC: u32 = 0x12fd_1001;
pub const SCREEN_W: u16 = 480;
pub const SCREEN_H: u16 = 272;
const BIN_HEADER: usize = 16;
const WINDOW_HEADER: usize = 8;

pub const CUSTOM_BIT: u32 = 1 << 16;
pub const INNER_OBJ_MASK: u32 = 0x000e_0000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u16)]
pub enum WidgetType {
    Container = 0,
    Button = 1,
    Image = 2,
    Label = 3,
    Bar = 4,
    Slider = 5,
    Template = 6,
    TextArea = 7,
    Arc = 8,
}

impl WidgetType {
    pub fn from_u16(v: u16) -> Option<Self> {
        use WidgetType::*;
        Some(match v {
            0 => Container,
            1 => Button,
            2 => Image,
            3 => Label,
            4 => Bar,
            5 => Slider,
            6 => Template,
            7 => TextArea,
            8 => Arc,
            _ => return None,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Flag {
    /// `lv_obj_flag_t` bits.
    pub flag: u32,
    /// Non-zero: `lv_obj_add_flag`; zero: `lv_obj_clear_flag`.
    pub mask: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Style {
    pub prop: u32,
    pub value: i32,
    /// LVGL selector: part (bits 16..23) | state (bits 0..15).
    pub selector: u32,
}

impl Style {
    pub fn is_custom(&self) -> bool {
        self.prop & CUSTOM_BIT != 0
    }
    pub fn obj_index(&self) -> u32 {
        (self.prop & INNER_OBJ_MASK) >> 17
    }
    /// The bare LVGL property id (custom index if [`is_custom`](Self::is_custom)).
    pub fn id(&self) -> u32 {
        if self.is_custom() { self.prop & 0xffff } else { self.prop & 0xff }
    }
    /// Human readable name such as `label.text_color` or `custom.tr_index`.
    pub fn name(&self) -> String {
        let base = if self.is_custom() {
            custom_prop_name(self.id()).map(|n| format!("custom.{n}"))
        } else {
            lv_prop_name(self.id()).map(str::to_string)
        }
        .unwrap_or_else(|| format!("prop{}", self.prop & 0xffff));
        match self.obj_index() {
            0 => base,
            i => format!("obj{i}.{base}"),
        }
    }
    pub fn is_color(&self) -> bool {
        !self.is_custom() && COLOR_PROPS.contains(&self.id())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Widget {
    pub index: u16,
    #[serde(rename = "type")]
    pub kind: u16,
    pub x: u16,
    pub y: u16,
    pub w: u16,
    pub h: u16,
    pub callback: u16,
    pub event: u16,
    pub flags: Vec<Flag>,
    pub styles: Vec<Style>,
    /// First CSV column (designer comment). Not stored in the .bin.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub comment: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Window {
    pub index: u8,
    #[serde(default)]
    pub attr: u16,
    pub widgets: Vec<Widget>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct WidgetDb {
    pub windows: Vec<Window>,
    #[serde(default)]
    pub header_size: u32,
    #[serde(default)]
    pub header_crc: u16,
}

impl WidgetDb {
    pub fn window(&self, index: u8) -> Option<&Window> {
        self.windows.iter().find(|w| w.index == index)
    }

    pub fn parse_bin(b: &[u8]) -> Result<Self, Error> {
        let mut r = Rd::at(b, 0);
        if r.u32()? != MAGIC {
            return fmt_err("widget.bin: bad magic");
        }
        let header_size = r.u32()?;
        let header_crc = r.u16()?;
        let nwin = r.u16()? as usize;
        let _total = r.u16()?;
        let _max = r.u16()?;
        let mut windows = Vec::with_capacity(nwin);
        for i in 0..nwin {
            let mut h = Rd::at(b, BIN_HEADER + i * WINDOW_HEADER);
            let off = h.u32()? as usize;
            let attr = h.u16()?;
            let n = h.u8()? as usize;
            let index = h.u8()?;
            let mut r = Rd::at(b, off);
            let mut widgets = Vec::with_capacity(n);
            for _ in 0..n {
                let index = r.u16()?;
                let kind = r.u16()?;
                let (x, y, w, hh) = (r.u16()?, r.u16()?, r.u16()?, r.u16()?);
                let callback = r.u16()?;
                let event = r.u16()?;
                let nf = r.u16()? as usize;
                let ns = r.u16()? as usize;
                let flags = (0..nf).map(|_| Ok(Flag { flag: r.u32()?, mask: r.u32()? })).collect::<Result<_, Error>>()?;
                let styles = (0..ns)
                    .map(|_| Ok(Style { prop: r.u32()?, value: r.i32()?, selector: r.u32()? }))
                    .collect::<Result<_, Error>>()?;
                widgets.push(Widget { index, kind, x, y, w, h: hh, callback, event, flags, styles, comment: String::new() });
            }
            windows.push(Window { index, attr, widgets });
        }
        Ok(Self { windows, header_size, header_crc })
    }

    /// Serialise exactly like `firmware/tools/widget`.
    pub fn to_bin(&self) -> Vec<u8> {
        let total: usize = self.windows.iter().map(|w| w.widgets.len()).sum();
        let max = self.windows.iter().map(|w| w.widgets.len()).max().unwrap_or(0);
        let mut body = Vec::new();
        let mut headers = Vec::new();
        let mut off = BIN_HEADER + self.windows.len() * WINDOW_HEADER;
        for win in &self.windows {
            headers.extend_from_slice(&(off as u32).to_le_bytes());
            headers.extend_from_slice(&win.attr.to_le_bytes());
            headers.push(win.widgets.len() as u8);
            headers.push(win.index);
            let start = body.len();
            for w in &win.widgets {
                for v in [w.index, w.kind, w.x, w.y, w.w, w.h, w.callback, w.event, w.flags.len() as u16, w.styles.len() as u16] {
                    body.extend_from_slice(&v.to_le_bytes());
                }
                for f in &w.flags {
                    body.extend_from_slice(&f.flag.to_le_bytes());
                    body.extend_from_slice(&f.mask.to_le_bytes());
                }
                for s in &w.styles {
                    body.extend_from_slice(&s.prop.to_le_bytes());
                    body.extend_from_slice(&s.value.to_le_bytes());
                    body.extend_from_slice(&s.selector.to_le_bytes());
                }
            }
            off += body.len() - start;
        }
        let mut out = Vec::with_capacity(off);
        out.extend_from_slice(&MAGIC.to_le_bytes());
        out.extend_from_slice(&self.header_size.to_le_bytes());
        out.extend_from_slice(&self.header_crc.to_le_bytes());
        out.extend_from_slice(&(self.windows.len() as u16).to_le_bytes());
        out.extend_from_slice(&(total as u16).to_le_bytes());
        out.extend_from_slice(&(max as u16).to_le_bytes());
        out.extend_from_slice(&headers);
        out.extend_from_slice(&body);
        out
    }

    /// Parse Elegoo's tab-separated `widget.csv` (same rules as the C tool).
    pub fn parse_csv(text: &str) -> Result<Self, Error> {
        let mut db = WidgetDb::default();
        for (lineno, line) in fgets_lines(text).enumerate().skip(1) {
            let toks = strtok_tabs(line);
            if toks.len() < 2 {
                continue;
            }
            let mut w = Widget { index: 0, kind: 0, x: 0, y: 0, w: 0, h: 0, callback: 0, event: 0, flags: vec![], styles: vec![], comment: toks[0].trim_end().to_string() };
            let win = scan_dec(toks[1]).map(|v| v.0 as u8).ok_or_else(|| Error::Format(format!("line {}: bad window", lineno + 1)))?;
            let field = |i: usize| toks.get(i).and_then(|t| scan_dec(t)).map(|v| v.0 as u16).unwrap_or(0);
            w.index = field(2);
            w.kind = field(3);
            w.x = field(4);
            w.y = field(5);
            w.w = field(6);
            w.h = field(7);
            w.callback = field(8);
            w.event = field(9);
            if let Some(t) = toks.get(10) {
                for p in t.split(':').filter(|p| !p.is_empty()) {
                    if let Some((flag, rest)) = scan_hex(p) {
                        let mask = rest.strip_prefix('|').and_then(scan_hex).map(|v| v.0).unwrap_or(0);
                        w.flags.push(Flag { flag, mask });
                    }
                }
            }
            for t in toks.iter().skip(11) {
                if let Some(s) = parse_style(t) {
                    w.styles.push(s);
                }
            }
            match db.windows.iter_mut().find(|x| x.index == win) {
                Some(x) => x.widgets.push(w),
                None => db.windows.push(Window { index: win, attr: 0, widgets: vec![w] }),
            }
        }
        Ok(db)
    }

    /// Write a CSV that [`parse_csv`](Self::parse_csv) (and Elegoo's tool) read
    /// back to the same binary.
    pub fn to_csv(&self) -> String {
        let mut out = String::from("备注\t界面序号\t控件序号\t控件类型\t控件坐标X\t控件坐标Y\t控件宽度W\t控件长度H\t控件回调号码\t控件事件号码\t属性\n");
        for win in &self.windows {
            for w in &win.widgets {
                let comment = if w.comment.is_empty() { format!("w{}_{}", win.index, w.index) } else { w.comment.replace('\t', " ") };
                out += &format!("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t", comment, win.index, w.index, w.kind, w.x, w.y, w.w, w.h, w.callback, w.event);
                let flags: Vec<String> = w.flags.iter().map(|f| format!("{:08x}|{:08x}", f.flag, f.mask)).collect();
                // An empty flag column would be swallowed by strtok and shift
                // the styles left; the C format cannot express "no flags".
                out += &flags.join(":");
                for s in &w.styles {
                    if s.is_color() {
                        out += &format!("\t{}|#{:08X}|{:x}", s.prop, s.value as u32, s.selector);
                    } else {
                        out += &format!("\t{}|{}|{:x}", s.prop, s.value, s.selector);
                    }
                }
                out.push('\n');
            }
        }
        out
    }
}

fn parse_style(t: &str) -> Option<Style> {
    let (prop, rest) = scan_dec(t)?;
    let rest = rest.strip_prefix('|')?;
    let (value, rest) = if let Some(h) = rest.strip_prefix('#') {
        let (v, r) = scan_hex(h)?;
        (v as i32, r)
    } else {
        let (v, r) = scan_dec(rest)?;
        (v as i32, r)
    };
    let (selector, _) = scan_hex(rest.strip_prefix('|')?)?;
    Some(Style { prop: prop as u32, value, selector })
}

/// LVGL 8.3 `lv_style_prop_t` ids that hold an `lv_color_t`.
const COLOR_PROPS: &[u32] = &[32, 34, 42, 48, 59, 68, 71, 77, 82, 87];

pub fn lv_prop_name(id: u32) -> Option<&'static str> {
    Some(match id {
        1 => "width",
        2 => "min_width",
        3 => "max_width",
        4 => "height",
        5 => "min_height",
        6 => "max_height",
        7 => "x",
        8 => "y",
        9 => "align",
        10 => "transform_width",
        11 => "transform_height",
        12 => "translate_x",
        13 => "translate_y",
        14 => "transform_zoom",
        15 => "transform_angle",
        16 => "pad_top",
        17 => "pad_bottom",
        18 => "pad_left",
        19 => "pad_right",
        20 => "pad_row",
        21 => "pad_column",
        32 => "bg_color",
        33 => "bg_opa",
        34 => "bg_grad_color",
        35 => "bg_grad_dir",
        36 => "bg_main_stop",
        37 => "bg_grad_stop",
        40 => "bg_img_src",
        41 => "bg_img_opa",
        42 => "bg_img_recolor",
        43 => "bg_img_recolor_opa",
        44 => "bg_img_tiled",
        48 => "border_color",
        49 => "border_opa",
        50 => "border_width",
        51 => "border_side",
        52 => "border_post",
        58 => "outline_width",
        59 => "outline_color",
        60 => "outline_opa",
        61 => "outline_pad",
        64 => "shadow_width",
        65 => "shadow_ofs_x",
        66 => "shadow_ofs_y",
        67 => "shadow_spread",
        68 => "shadow_color",
        69 => "shadow_opa",
        70 => "img_opa",
        71 => "img_recolor",
        72 => "img_recolor_opa",
        73 => "line_width",
        77 => "line_color",
        78 => "line_opa",
        80 => "arc_width",
        81 => "arc_rounded",
        82 => "arc_color",
        83 => "arc_opa",
        87 => "text_color",
        88 => "text_opa",
        89 => "text_font",
        90 => "text_letter_space",
        91 => "text_line_space",
        92 => "text_decor",
        93 => "text_align",
        96 => "radius",
        97 => "clip_corner",
        98 => "opa",
        101 => "anim_time",
        103 => "transition",
        104 => "blend_mode",
        105 => "layout",
        _ => return None,
    })
}

/// Elegoo's custom properties (`ui/loader/widget.c`).
pub fn custom_prop_name(id: u32) -> Option<&'static str> {
    Some(match id {
        1 => "image_index",  // /app/resources/images/<value>.png
        2 => "tr_index",     // translation string id
        3 => "font_weight",  // low byte: px size, high byte: 1 italic | 2 bold
        4 => "parent_index", // re-parent onto widget <value> of the same window
        5 => "align_out_top_left",
        6 => "align_out_top_mid",
        7 => "align_out_top_right",
        8 => "align_out_bottom_left",
        9 => "align_out_bottom_mid",
        10 => "align_out_bottom_right",
        11 => "align_out_left_top",
        12 => "align_out_left_mid",
        13 => "align_out_left_bottom",
        14 => "align_out_right_top",
        15 => "align_out_right_mid",
        16 => "align_out_right_bottom",
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn res(p: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../firmware/resources/e100").join(p)
    }

    #[test]
    fn shipped_bin_roundtrips() {
        let bin = std::fs::read(res("widget.bin")).unwrap();
        let db = WidgetDb::parse_bin(&bin).unwrap();
        assert_eq!(db.windows.len(), 42);
        assert_eq!(db.windows.iter().map(|w| w.widgets.len()).sum::<usize>(), 631);
        assert_eq!(db.to_bin(), bin);
    }

    #[test]
    fn shipped_csv_compiles_to_shipped_bin() {
        let csv = std::fs::read_to_string(res("widget.csv")).unwrap();
        let bin = std::fs::read(res("widget.bin")).unwrap();
        assert_eq!(WidgetDb::parse_csv(&csv).unwrap().to_bin(), bin);
    }

    #[test]
    fn exported_csv_compiles_back() {
        let bin = std::fs::read(res("widget.bin")).unwrap();
        let csv = WidgetDb::parse_bin(&bin).unwrap().to_csv();
        assert_eq!(WidgetDb::parse_csv(&csv).unwrap().to_bin(), bin);
    }

    #[test]
    fn json_roundtrip() {
        let bin = std::fs::read(res("widget.bin")).unwrap();
        let db = WidgetDb::parse_bin(&bin).unwrap();
        let j = serde_json::to_string(&db).unwrap();
        assert_eq!(serde_json::from_str::<WidgetDb>(&j).unwrap().to_bin(), bin);
    }

    #[test]
    fn prop_names() {
        assert_eq!(Style { prop: 1111, value: 0, selector: 0 }.name(), "text_color");
        assert_eq!(Style { prop: 266249, value: 0, selector: 0 }.name(), "obj2.align");
        assert_eq!(Style { prop: 65538, value: 0, selector: 0 }.name(), "custom.tr_index");
        assert_eq!(Style { prop: 135177, value: 0, selector: 0 }.name(), "obj1.align");
    }
}
