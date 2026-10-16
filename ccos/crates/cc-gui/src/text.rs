//! TrueType text rendering with a glyph cache.

use std::collections::HashMap;

use crate::canvas::{Canvas, Rect};

pub struct Font {
    font: fontdue::Font,
    cache: HashMap<(char, u32), (fontdue::Metrics, Vec<u8>)>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Align {
    Left,
    Center,
    Right,
}

impl Font {
    /// Load a .ttf/.otf, or face `index` of a .ttc collection.
    pub fn from_bytes(data: &[u8], index: u32) -> Result<Self, String> {
        let settings = fontdue::FontSettings { collection_index: index, scale: 40.0, ..Default::default() };
        Ok(Self { font: fontdue::Font::from_bytes(data, settings).map_err(|e| e.to_string())?, cache: HashMap::new() })
    }

    pub fn load(path: &str, index: u32) -> Result<Self, String> {
        Self::from_bytes(&std::fs::read(path).map_err(|e| format!("{path}: {e}"))?, index)
    }

    fn glyph(&mut self, c: char, px: f32) -> &(fontdue::Metrics, Vec<u8>) {
        let key = (c, px.to_bits());
        let font = &self.font;
        self.cache.entry(key).or_insert_with(|| font.rasterize(c, px))
    }

    /// Width in pixels of a single line.
    pub fn measure(&mut self, s: &str, px: f32) -> i32 {
        s.chars().map(|c| self.glyph(c, px).0.advance_width).sum::<f32>().round() as i32
    }

    pub fn line_height(&self, px: f32) -> i32 {
        self.font.horizontal_line_metrics(px).map(|m| m.new_line_size.ceil() as i32).unwrap_or(px as i32 + 2)
    }

    fn ascent(&self, px: f32) -> f32 {
        self.font.horizontal_line_metrics(px).map(|m| m.ascent).unwrap_or(px * 0.8)
    }

    /// Draw one line with its top-left at (x, y).
    pub fn draw_line(&mut self, canvas: &mut Canvas, x: i32, y: i32, s: &str, px: f32, color: u32) -> i32 {
        let baseline = y as f32 + self.ascent(px);
        let mut pen = x as f32;
        for c in s.chars() {
            let (m, bmp) = self.glyph(c, px).clone();
            let gx = (pen + m.xmin as f32).round() as i32;
            let gy = (baseline - m.height as f32 - m.ymin as f32).round() as i32;
            for row in 0..m.height {
                for col in 0..m.width {
                    canvas.blend(gx + col as i32, gy + row as i32, color, bmp[row * m.width + col]);
                }
            }
            pen += m.advance_width;
        }
        (pen - x as f32).round() as i32
    }

    /// Break `s` into lines no wider than `max_w` (LVGL `LV_LABEL_LONG_WRAP`).
    /// Breaks at spaces when possible, otherwise between characters (CJK).
    pub fn wrap(&mut self, s: &str, px: f32, max_w: i32) -> String {
        let mut out = String::new();
        for (i, para) in s.split('\n').enumerate() {
            if i > 0 {
                out.push('\n');
            }
            let mut line = String::new();
            let mut last_space: Option<usize> = None;
            for ch in para.chars() {
                line.push(ch);
                if ch == ' ' {
                    last_space = Some(line.len() - 1);
                }
                if self.measure(&line, px) > max_w && line.chars().count() > 1 {
                    let cut = last_space.unwrap_or(line.len() - ch.len_utf8());
                    let rest = line[cut..].trim_start().to_string();
                    out.push_str(line[..cut].trim_end());
                    out.push('\n');
                    line = rest;
                    last_space = None;
                }
            }
            out.push_str(&line);
        }
        out
    }

    /// Draw multi-line text inside `r`, vertically centred, clipped to `r`.
    pub fn draw_in(&mut self, canvas: &mut Canvas, r: Rect, s: &str, px: f32, color: u32, align: Align) {
        let lines: Vec<&str> = s.split('\n').collect();
        let lh = self.line_height(px);
        let total = lh * lines.len() as i32;
        let mut y = r.y + (r.h - total) / 2;
        canvas.set_clip(Some(r));
        for l in lines {
            let w = self.measure(l, px);
            let x = match align {
                Align::Left => r.x,
                Align::Center => r.x + (r.w - w) / 2,
                Align::Right => r.x + r.w - w,
            };
            self.draw_line(canvas, x, y, l, px, color);
            y += lh;
        }
        canvas.set_clip(None);
    }
}
