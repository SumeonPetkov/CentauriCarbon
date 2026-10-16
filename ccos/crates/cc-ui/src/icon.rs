//! Icons converted at build time.

use crate::canvas::Canvas;

#[derive(Debug)]
pub enum IconData {
    /// Single-colour icon: 4-bit alpha mask, drawn in `color` (or any colour
    /// passed at draw time). Most UI glyph icons end up here — 8x smaller
    /// than RGBA.
    Mask { color: u32, data: &'static [u8] },
    /// Multi-colour icon, straight-alpha RGBA8.
    Rgba(&'static [u8]),
}

#[derive(Debug)]
pub struct Icon {
    pub w: u16,
    pub h: u16,
    pub data: IconData,
}

impl Icon {
    /// Draw with its top-left at (x, y). `tint` overrides a mask's colour.
    pub fn draw(&self, c: &mut Canvas, x: i32, y: i32, tint: Option<u32>) {
        match self.data {
            IconData::Mask { color, data } => c.draw_mask4(x, y, self.w, self.h, data, tint.unwrap_or(color)),
            IconData::Rgba(data) => c.draw_rgba(x, y, self.w, self.h, data),
        }
    }
}
