//! Approximate renderer for stock `widget.bin` windows.
//!
//! It interprets the same data Elegoo's `ui/loader/widget.c` feeds to LVGL:
//! positions, parent links, LVGL alignment, default-state background/text
//! styles and the custom image / text-id / font-size properties. It does not
//! run the app logic, so dynamic text (temperatures, file names) is absent —
//! use `annotate` to label every widget with its CSV comment / index instead.

use cc_res::translation::TranslationDb;
use cc_res::widget::{Style, WidgetDb, WidgetType, Window};

use crate::canvas::{Canvas, Rect};
use crate::image::ImageStore;
use crate::text::{Align, Font};
use crate::{SCREEN_H, SCREEN_W};

// LVGL 8 dark default theme approximations.
const SCREEN_BG: u32 = 0x15171a;
const CARD_BG: u32 = 0x282b30;
const PRIMARY: u32 = 0x2196f3;
const TEXT: u32 = 0xffffff;

pub struct PreviewOptions {
    pub lang: usize,
    /// Draw window 0 (status bar / side bar layer) underneath.
    pub with_sys_layer: bool,
    /// Outline every widget and print its index / comment.
    pub annotate: bool,
}

impl Default for PreviewOptions {
    fn default() -> Self {
        Self { lang: 1, with_sys_layer: true, annotate: false }
    }
}

#[derive(Default, Clone, Copy)]
struct Props {
    bg_color: Option<u32>,
    bg_opa: Option<u8>,
    radius: Option<i32>,
    text_color: Option<u32>,
    text_align: Option<i32>,
    align: Option<i32>,
    x: Option<i32>,
    y: Option<i32>,
    w: Option<i32>,
    h: Option<i32>,
    image: Option<i32>,
    tr: Option<i32>,
    font_px: Option<i32>,
    parent: Option<i32>,
    align_to: Option<(u32, i32)>,
}

fn collect(styles: &[Style], obj: u32) -> Props {
    let mut p = Props::default();
    for s in styles.iter().filter(|s| s.obj_index() == obj && s.selector == 0) {
        let v = s.value;
        if s.is_custom() {
            match s.id() {
                1 => p.image = Some(v),
                2 => p.tr = Some(v),
                3 => p.font_px = Some(v & 0xff),
                4 => p.parent = Some(v),
                id @ 5..=16 => p.align_to = Some((id, v)),
                _ => {}
            }
        } else {
            match s.id() {
                32 => p.bg_color = Some(v as u32 & 0xffffff),
                33 => p.bg_opa = Some(v.clamp(0, 255) as u8),
                96 => p.radius = Some(v),
                87 => p.text_color = Some(v as u32 & 0xffffff),
                93 => p.text_align = Some(v),
                9 => p.align = Some(v),
                7 => p.x = Some(v),
                8 => p.y = Some(v),
                1 => p.w = Some(v),
                4 => p.h = Some(v),
                _ => {}
            }
        }
    }
    p
}

/// Position a `cw x ch` child inside `parent` per LVGL `lv_align_t`.
fn align_in(parent: Rect, cw: i32, ch: i32, align: i32, ox: i32, oy: i32) -> Rect {
    let (pw, ph) = (parent.w, parent.h);
    let (x, y) = match align {
        2 => ((pw - cw) / 2, 0),
        3 => (pw - cw, 0),
        4 => (0, ph - ch),
        5 => ((pw - cw) / 2, ph - ch),
        6 => (pw - cw, ph - ch),
        7 => (0, (ph - ch) / 2),
        8 => (pw - cw, (ph - ch) / 2),
        9 => ((pw - cw) / 2, (ph - ch) / 2),
        _ => (0, 0),
    };
    Rect::new(parent.x + x + ox, parent.y + y + oy, cw, ch)
}

/// Elegoo custom `align_out_*` (relative to a sibling widget).
fn align_out(base: Rect, cw: i32, ch: i32, kind: u32, ox: i32, oy: i32) -> Rect {
    let (bx, by, bw, bh) = (base.x, base.y, base.w, base.h);
    let (x, y) = match kind {
        5 => (bx, by - ch),
        6 => (bx + (bw - cw) / 2, by - ch),
        7 => (bx + bw - cw, by - ch),
        8 => (bx, by + bh),
        9 => (bx + (bw - cw) / 2, by + bh),
        10 => (bx + bw - cw, by + bh),
        11 => (bx - cw, by),
        12 => (bx - cw, by + (bh - ch) / 2),
        13 => (bx - cw, by + bh - ch),
        14 => (bx + bw, by),
        15 => (bx + bw, by + (bh - ch) / 2),
        _ => (bx + bw, by + bh - ch),
    };
    Rect::new(x + ox, y + oy, cw, ch)
}

pub struct Previewer<'a> {
    pub db: &'a WidgetDb,
    pub tr: Option<&'a TranslationDb>,
    pub images: &'a mut ImageStore,
    pub font: &'a mut Font,
}

impl Previewer<'_> {
    pub fn render(&mut self, window: u8, opt: &PreviewOptions) -> Result<Canvas, String> {
        let mut c = Canvas::new(SCREEN_W, SCREEN_H);
        c.clear(SCREEN_BG);
        let win = self.db.window(window).ok_or_else(|| format!("no window {window}"))?;
        if opt.with_sys_layer && window != 0 {
            if let Some(sys) = self.db.window(0) {
                self.draw_window(&mut c, sys, opt, true);
            }
        }
        self.draw_window(&mut c, win, opt, false);
        Ok(c)
    }

    fn text_of(&self, p: &Props, lang: usize) -> Option<String> {
        let id = p.tr.filter(|&t| t > 0)?;
        self.tr.and_then(|t| t.get(lang, id as u16)).map(|s| s.trim_end_matches(['\r', '\n']).to_string())
    }

    fn draw_window(&mut self, c: &mut Canvas, win: &Window, opt: &PreviewOptions, sys: bool) {
        let screen = Rect::new(0, 0, SCREEN_W as i32, SCREEN_H as i32);
        let mut rects: Vec<Option<Rect>> = vec![None; 256];
        for w in &win.widgets {
            let hidden = w.flags.iter().any(|f| f.flag & 1 != 0 && f.mask != 0);
            let p0 = collect(&w.styles, 0);
            let parent = p0.parent.and_then(|i| rects.get(i as usize).copied().flatten()).unwrap_or(screen);
            let kind = WidgetType::from_u16(w.kind);
            let text = self.text_of(&p0, opt.lang);
            let font_px = p0.font_px.unwrap_or(16) as f32;
            let (mut cw, mut ch) = (w.w as i32, w.h as i32);
            if let Some(v) = p0.w {
                cw = v;
            }
            if let Some(v) = p0.h {
                ch = v;
            }
            // LV_SIZE_CONTENT stand-ins
            if kind == Some(WidgetType::Image) && (cw == 0 || ch == 0) {
                if let Some(img) = p0.image.and_then(|i| self.images.get(i)) {
                    cw = img.width as i32;
                    ch = img.height as i32;
                }
            }
            let text = match (kind, text) {
                (Some(WidgetType::Label), Some(t)) if cw > 0 => Some(self.font.wrap(&t, font_px, cw)),
                (_, t) => t,
            };
            if kind == Some(WidgetType::Label) && (cw == 0 || ch == 0) {
                let t = text.clone().unwrap_or_default();
                cw = t.split('\n').map(|l| self.font.measure(l, font_px)).max().unwrap_or(0);
                ch = self.font.line_height(font_px) * t.split('\n').count() as i32;
            }
            let (ox, oy) = (p0.x.unwrap_or(w.x as i32), p0.y.unwrap_or(w.y as i32));
            let r = match p0.align_to {
                Some((kind, base)) => {
                    let b = rects.get(base as usize).copied().flatten().unwrap_or(screen);
                    align_out(b, cw, ch, kind, w.x as i32, w.y as i32)
                }
                None => align_in(parent, cw, ch, p0.align.unwrap_or(1), ox, oy),
            };
            if (w.index as usize) < rects.len() {
                rects[w.index as usize] = Some(r);
            }
            if hidden {
                continue;
            }
            let default_bg = match kind {
                Some(WidgetType::Button) => Some(PRIMARY),
                Some(WidgetType::Container) => Some(CARD_BG),
                Some(WidgetType::Bar | WidgetType::Slider) => Some(0x404040),
                _ => None,
            };
            let opa = p0.bg_opa.unwrap_or(if default_bg.is_some() { 255 } else { 0 });
            if let Some(bg) = p0.bg_color.or(default_bg) {
                if opa > 0 {
                    c.fill_round_rect(r, p0.radius.unwrap_or(0), bg, opa);
                }
            }
            if kind == Some(WidgetType::Image) {
                if let Some(img) = p0.image.and_then(|i| self.images.get(i)) {
                    c.blit_rgba(r.x, r.y, img.width, img.height, &img.rgba, 255);
                }
            }
            let color = p0.text_color.unwrap_or(TEXT);
            if kind == Some(WidgetType::Label) {
                if let Some(t) = &text {
                    let al = match p0.text_align {
                        Some(2) => Align::Center,
                        Some(3) => Align::Right,
                        _ => Align::Left,
                    };
                    self.font.draw_in(c, r, t, font_px, color, al);
                }
            }
            if kind == Some(WidgetType::Button) {
                // obj1 = image, obj2 = label (see widget_alloc)
                let p1 = collect(&w.styles, 1);
                if let Some(img) = p1.image.or(p0.image).and_then(|i| self.images.get(i)) {
                    let ir = align_in(r, img.width as i32, img.height as i32, p1.align.unwrap_or(1), p1.x.unwrap_or(0), p1.y.unwrap_or(0));
                    let (iw, ih, data) = (img.width, img.height, img.rgba.clone());
                    c.blit_rgba(ir.x, ir.y, iw, ih, &data, 255);
                }
                let p2 = collect(&w.styles, 2);
                let label = self.text_of(&p2, opt.lang).or(text);
                if let Some(t) = label {
                    let px = p2.font_px.or(p0.font_px).unwrap_or(16) as f32;
                    let lw = p2.w.unwrap_or_else(|| t.split('\n').map(|l| self.font.measure(l, px)).max().unwrap_or(0));
                    let lh = self.font.line_height(px) * t.split('\n').count() as i32;
                    let lr = align_in(r, lw, lh, p2.align.unwrap_or(1), p2.x.unwrap_or(0), p2.y.unwrap_or(0));
                    let al = match p2.text_align {
                        Some(2) => Align::Center,
                        Some(3) => Align::Right,
                        _ => Align::Left,
                    };
                    self.font.draw_in(c, lr, &t, px, p2.text_color.or(p0.text_color).unwrap_or(TEXT), al);
                }
            }
            if opt.annotate && !sys {
                c.stroke_round_rect(r, 0, 1, 0xff00ff);
                let tag = if w.comment.is_empty() { format!("{}", w.index) } else { format!("{} {}", w.index, w.comment) };
                self.font.draw_line(c, r.x + 1, r.y + 1, &tag, 10.0, 0xffff00);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn res(p: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../firmware/resources/e100").join(p)
    }

    #[test]
    fn renders_every_stock_window() {
        let db = WidgetDb::parse_bin(&std::fs::read(res("widget.bin")).unwrap()).unwrap();
        let tr = TranslationDb::parse_bin(&std::fs::read(res("translation.bin")).unwrap()).unwrap();
        let mut images = ImageStore::new(res("images"));
        let mut font = Font::load(res("fonts/font.ttc").to_str().unwrap(), 0).unwrap();
        let mut pv = Previewer { db: &db, tr: Some(&tr), images: &mut images, font: &mut font };
        for w in &db.windows {
            let c = pv.render(w.index, &PreviewOptions::default()).unwrap();
            assert_eq!(c.pixels.len(), 480 * 272);
        }
        // The home screen must not be blank.
        let home = pv.render(1, &PreviewOptions::default()).unwrap();
        let distinct: std::collections::HashSet<u32> = home.pixels.iter().copied().collect();
        assert!(distinct.len() > 20, "home screen looks empty");
    }
}
