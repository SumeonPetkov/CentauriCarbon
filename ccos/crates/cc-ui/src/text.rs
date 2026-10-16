//! Text with fonts pre-rasterised at build time (4-bit alpha). No font
//! engine, no glyph cache, no allocation at runtime.

use crate::canvas::{Canvas, Rect};

/// One baked glyph. `x`/`y` place the bitmap relative to the pen position
/// and the top of the line.
#[derive(Debug, Clone, Copy)]
pub struct Glyph {
    pub ch: u32,
    pub x: i8,
    pub y: i8,
    pub w: u8,
    pub h: u8,
    pub adv: u8,
    /// Byte offset of the bitmap in `Font::data` (rows padded to bytes).
    pub off: u32,
}

#[derive(Debug)]
pub struct Font {
    pub line_height: u8,
    /// Sorted by `ch`.
    pub glyphs: &'static [Glyph],
    /// Index into `glyphs` for U+0020..U+007F (`u16::MAX` = missing).
    pub ascii: [u16; 96],
    pub data: &'static [u8],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Align {
    Left,
    Center,
    Right,
}

impl Font {
    #[inline]
    pub fn glyph(&self, ch: char) -> Option<&Glyph> {
        let c = ch as u32;
        if (0x20..0x80).contains(&c) {
            let i = self.ascii[(c - 0x20) as usize];
            return (i != u16::MAX).then(|| &self.glyphs[i as usize]);
        }
        self.glyphs.binary_search_by_key(&c, |g| g.ch).ok().map(|i| &self.glyphs[i])
    }

    fn glyph_or_fallback(&self, ch: char) -> Option<&Glyph> {
        self.glyph(ch).or_else(|| self.glyph('?'))
    }

    /// Width in pixels of one line.
    pub fn measure(&self, s: &str) -> i32 {
        s.chars().map(|c| self.glyph_or_fallback(c).map_or(0, |g| g.adv as i32)).sum()
    }

    /// Draw one line with its top-left at (x, y); returns the width.
    pub fn draw(&self, c: &mut Canvas, x: i32, y: i32, s: &str, color: u32) -> i32 {
        let mut pen = x;
        for ch in s.chars() {
            if let Some(g) = self.glyph_or_fallback(ch) {
                if g.w > 0 {
                    let stride = (g.w as usize).div_ceil(2);
                    let end = g.off as usize + stride * g.h as usize;
                    c.draw_mask4(pen + g.x as i32, y + g.y as i32, g.w as u16, g.h as u16, &self.data[g.off as usize..end], color);
                }
                pen += g.adv as i32;
            }
        }
        pen - x
    }

    /// Lines of `s` wrapped to `max_w` (at spaces, or anywhere if a word is
    /// too long); calls `f(line)` for each. No allocation.
    pub fn wrap(&self, s: &str, max_w: i32, mut f: impl FnMut(&str)) {
        for para in s.split('\n') {
            let mut start = 0;
            let mut last_space: Option<usize> = None;
            let mut width = 0;
            for (i, ch) in para.char_indices() {
                let end = i + ch.len_utf8();
                let adv = self.glyph_or_fallback(ch).map_or(0, |g| g.adv as i32);
                if ch == ' ' {
                    last_space = Some(i);
                }
                width += adv;
                if width > max_w && i > start {
                    let cut = match last_space {
                        Some(sp) if sp > start => sp,
                        _ => i,
                    };
                    f(para[start..cut].trim_end());
                    start = if para[cut..].starts_with(' ') { cut + 1 } else { cut };
                    last_space = None;
                    width = self.measure(&para[start..end]);
                }
            }
            f(&para[start..]);
        }
    }

    /// Draw (wrapped) text inside `r`, vertically centred, clipped to `r`.
    pub fn draw_in(&self, c: &mut Canvas, r: Rect, s: &str, color: u32, align: Align) {
        let mut lines = 0;
        self.wrap(s, r.w as i32, |_| lines += 1);
        let lh = self.line_height as i32;
        let mut y = r.y as i32 + (r.h as i32 - lh * lines) / 2;
        let saved = c.clip();
        c.set_clip(saved.intersect(&r));
        self.wrap(s, r.w as i32, |line| {
            let w = self.measure(line);
            let x = match align {
                Align::Left => r.x as i32,
                Align::Center => r.x as i32 + (r.w as i32 - w) / 2,
                Align::Right => r.right() as i32 - w,
            };
            self.draw(c, x, y, line, color);
            y += lh;
        });
        c.set_clip(saved);
    }
}

/// Fixed-capacity string for formatting live values without allocating.
pub struct TextBuf {
    buf: [u8; 64],
    len: usize,
}

impl Default for TextBuf {
    fn default() -> Self {
        Self { buf: [0; 64], len: 0 }
    }
}

impl TextBuf {
    pub fn as_str(&self) -> &str {
        // Only whole UTF-8 sequences are ever written.
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
    pub fn clear(&mut self) {
        self.len = 0;
    }
    pub fn push_str(&mut self, s: &str) {
        let _ = core::fmt::Write::write_str(self, s);
    }
}

impl core::fmt::Write for TextBuf {
    /// Truncates at a character boundary when full.
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for ch in s.chars() {
            let n = ch.len_utf8();
            if self.len + n > self.buf.len() {
                break;
            }
            ch.encode_utf8(&mut self.buf[self.len..]);
            self.len += n;
        }
        Ok(())
    }
}
