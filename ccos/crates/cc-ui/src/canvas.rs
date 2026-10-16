//! XRGB8888 back buffer with clipping and the few primitives the UI needs.

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Rect {
    pub x: i16,
    pub y: i16,
    pub w: i16,
    pub h: i16,
}

impl Rect {
    pub const fn new(x: i16, y: i16, w: i16, h: i16) -> Self {
        Self { x, y, w, h }
    }
    pub const fn right(&self) -> i16 {
        self.x + self.w
    }
    pub const fn bottom(&self) -> i16 {
        self.y + self.h
    }
    pub fn is_empty(&self) -> bool {
        self.w <= 0 || self.h <= 0
    }
    pub fn contains(&self, px: i32, py: i32) -> bool {
        px >= self.x as i32 && py >= self.y as i32 && px < self.right() as i32 && py < self.bottom() as i32
    }
    pub fn intersect(&self, o: &Rect) -> Rect {
        let x = self.x.max(o.x);
        let y = self.y.max(o.y);
        let r = self.right().min(o.right());
        let b = self.bottom().min(o.bottom());
        Rect::new(x, y, (r - x).max(0), (b - y).max(0))
    }
    pub fn intersects(&self, o: &Rect) -> bool {
        !self.intersect(o).is_empty()
    }
    pub fn union(&self, o: &Rect) -> Rect {
        if self.is_empty() {
            return *o;
        }
        if o.is_empty() {
            return *self;
        }
        let x = self.x.min(o.x);
        let y = self.y.min(o.y);
        Rect::new(x, y, self.right().max(o.right()) - x, self.bottom().max(o.bottom()) - y)
    }
    pub fn inset(&self, d: i16) -> Rect {
        Rect::new(self.x + d, self.y + d, (self.w - 2 * d).max(0), (self.h - 2 * d).max(0))
    }
    pub fn area(&self) -> i32 {
        if self.is_empty() { 0 } else { self.w as i32 * self.h as i32 }
    }
}

pub struct Canvas {
    pub width: u16,
    pub height: u16,
    pub pixels: Vec<u32>,
    clip: Rect,
}

#[inline(always)]
fn mix(c: u32, d: u32, a: u32) -> u32 {
    // (c*a + d*(255-a)) / 255 per channel, two channels at a time.
    let inv = 255 - a;
    let rb = ((c & 0x00ff_00ff) * a + (d & 0x00ff_00ff) * inv + 0x0080_0080) >> 8 & 0x00ff_00ff;
    let g = ((c & 0x0000_ff00) * a + (d & 0x0000_ff00) * inv + 0x0000_8000) >> 8 & 0x0000_ff00;
    rb | g
}

impl Canvas {
    /// The only allocation of the UI: one frame buffer, done once.
    pub fn new(width: u16, height: u16) -> Self {
        Self { width, height, pixels: vec![0; width as usize * height as usize], clip: Rect::new(0, 0, width as i16, height as i16) }
    }

    pub fn bounds(&self) -> Rect {
        Rect::new(0, 0, self.width as i16, self.height as i16)
    }

    pub fn set_clip(&mut self, r: Rect) {
        self.clip = r.intersect(&self.bounds());
    }

    pub fn clip(&self) -> Rect {
        self.clip
    }

    #[inline(always)]
    pub fn blend(&mut self, x: i32, y: i32, c: u32, a: u32) {
        if a == 0 || !self.clip.contains(x, y) {
            return;
        }
        let i = y as usize * self.width as usize + x as usize;
        self.pixels[i] = if a >= 255 { c } else { mix(c, self.pixels[i], a) };
    }

    pub fn fill_rect(&mut self, r: Rect, c: u32) {
        let r = r.intersect(&self.clip);
        if r.is_empty() {
            return;
        }
        let w = self.width as usize;
        for y in r.y as usize..r.bottom() as usize {
            self.pixels[y * w + r.x as usize..y * w + r.right() as usize].fill(c);
        }
    }

    /// Filled rectangle with anti-aliased rounded corners; `opa` 0..=255.
    pub fn fill_round_rect(&mut self, r: Rect, radius: i16, c: u32, opa: u8) {
        let rad = radius.min(r.w / 2).min(r.h / 2).max(0) as i32;
        if opa == 255 {
            // Fast path: solid middle band, AA only in the corner squares.
            self.fill_rect(Rect::new(r.x, r.y + rad as i16, r.w, r.h - 2 * rad as i16), c);
            self.fill_rect(Rect::new(r.x + rad as i16, r.y, r.w - 2 * rad as i16, rad as i16), c);
            self.fill_rect(Rect::new(r.x + rad as i16, r.bottom() - rad as i16, r.w - 2 * rad as i16, rad as i16), c);
            if rad > 0 {
                self.corners(r, rad, c, 255);
            }
            return;
        }
        let clip = r.intersect(&self.clip);
        for y in clip.y as i32..clip.bottom() as i32 {
            for x in clip.x as i32..clip.right() as i32 {
                let cov = coverage(x - r.x as i32, y - r.y as i32, r.w as i32, r.h as i32, rad);
                self.blend(x, y, c, cov * opa as u32 / 255);
            }
        }
    }

    fn corners(&mut self, r: Rect, rad: i32, c: u32, opa: u32) {
        let (x0, y0, w, h) = (r.x as i32, r.y as i32, r.w as i32, r.h as i32);
        for dy in 0..rad {
            for dx in 0..rad {
                let cov = coverage(dx, dy, w, h, rad) * opa / 255;
                self.blend(x0 + dx, y0 + dy, c, cov);
                self.blend(x0 + w - 1 - dx, y0 + dy, c, cov);
                self.blend(x0 + dx, y0 + h - 1 - dy, c, cov);
                self.blend(x0 + w - 1 - dx, y0 + h - 1 - dy, c, cov);
            }
        }
    }

    /// Outline of a rounded rectangle, `width` pixels thick.
    pub fn stroke_round_rect(&mut self, r: Rect, radius: i16, width: i16, c: u32) {
        let rad = radius.min(r.w / 2).min(r.h / 2).max(0) as i32;
        let inner = r.inset(width);
        let irad = (rad - width as i32).max(0);
        let clip = r.intersect(&self.clip);
        for y in clip.y as i32..clip.bottom() as i32 {
            for x in clip.x as i32..clip.right() as i32 {
                let o = coverage(x - r.x as i32, y - r.y as i32, r.w as i32, r.h as i32, rad);
                let i = if inner.contains(x, y) {
                    coverage(x - inner.x as i32, y - inner.y as i32, inner.w as i32, inner.h as i32, irad)
                } else {
                    0
                };
                self.blend(x, y, c, o.saturating_sub(i));
            }
        }
    }

    /// Draw a 4-bit alpha mask (two pixels per byte, high nibble first,
    /// rows padded to whole bytes) in colour `c`.
    pub fn draw_mask4(&mut self, x0: i32, y0: i32, w: u16, h: u16, data: &[u8], c: u32) {
        let stride = (w as usize).div_ceil(2);
        let clip = self.clip;
        let ys = (clip.y as i32 - y0).max(0) as usize;
        let ye = ((clip.bottom() as i32 - y0).max(0) as usize).min(h as usize);
        let xs = (clip.x as i32 - x0).max(0) as usize;
        let xe = ((clip.right() as i32 - x0).max(0) as usize).min(w as usize);
        let width = self.width as usize;
        for row in ys..ye {
            let src = &data[row * stride..];
            let base = (y0 + row as i32) as usize * width;
            for col in xs..xe {
                let byte = src[col / 2];
                let n = if col % 2 == 0 { byte >> 4 } else { byte & 0x0f } as u32;
                if n == 0 {
                    continue;
                }
                let i = base + (x0 + col as i32) as usize;
                self.pixels[i] = if n == 15 { c } else { mix(c, self.pixels[i], n * 17) };
            }
        }
    }

    /// Draw straight-alpha RGBA8 pixels.
    pub fn draw_rgba(&mut self, x0: i32, y0: i32, w: u16, h: u16, data: &[u8]) {
        for row in 0..h as i32 {
            for col in 0..w as i32 {
                let i = (row as usize * w as usize + col as usize) * 4;
                let c = (data[i] as u32) << 16 | (data[i + 1] as u32) << 8 | data[i + 2] as u32;
                self.blend(x0 + col, y0 + row, c, data[i + 3] as u32);
            }
        }
    }
}

/// 0..=255 coverage of pixel (x, y) in a w x h rectangle with corner radius
/// `rad` (4x4 supersampling, integer only).
fn coverage(x: i32, y: i32, w: i32, h: i32, rad: i32) -> u32 {
    if rad == 0 {
        return 255;
    }
    let cx = if x < rad { rad } else if x >= w - rad { w - rad } else { return 255 };
    let cy = if y < rad { rad } else if y >= h - rad { h - rad } else { return 255 };
    // Work in 1/8 pixel units; sample centres at 1,3,5,7.
    let r2 = (rad * 8) * (rad * 8);
    let mut n = 0;
    for sy in 0..4 {
        for sx in 0..4 {
            let px = x * 8 + sx * 2 + 1 - cx * 8;
            let py = y * 8 + sy * 2 + 1 - cy * 8;
            if px * px + py * py <= r2 {
                n += 1;
            }
        }
    }
    (n * 255 / 16) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rect_ops() {
        let a = Rect::new(0, 0, 10, 10);
        let b = Rect::new(5, 5, 10, 10);
        assert_eq!(a.intersect(&b), Rect::new(5, 5, 5, 5));
        assert_eq!(a.union(&b), Rect::new(0, 0, 15, 15));
        assert!(!a.intersects(&Rect::new(10, 0, 5, 5)));
    }

    #[test]
    fn mix_matches_float() {
        for (c, d, a) in [(0xffffff, 0, 128), (0x123456, 0xabcdef, 77), (0xff0000, 0x00ff00, 255), (0x0000ff, 0xffffff, 0)] {
            let got = mix(c, d, a);
            for sh in [0, 8, 16] {
                let want = (((c >> sh) & 0xff) * a + ((d >> sh) & 0xff) * (255 - a)) as f32 / 255.0;
                assert!((((got >> sh) & 0xff) as f32 - want).abs() <= 1.0, "{c:x} {d:x} {a}");
            }
        }
    }

    #[test]
    fn round_rect_and_clip() {
        let mut c = Canvas::new(40, 40);
        c.fill_round_rect(Rect::new(0, 0, 40, 40), 10, 0xffffff, 255);
        assert_eq!(c.pixels[0], 0);
        assert_eq!(c.pixels[20 * 40 + 20], 0xffffff);
        assert_eq!(c.pixels[39], 0);
        c.set_clip(Rect::new(0, 0, 5, 5));
        c.fill_rect(Rect::new(0, 0, 40, 40), 0x111111);
        assert_eq!(c.pixels[6 * 40 + 6], 0xffffff);
    }

    #[test]
    fn mask4() {
        let mut c = Canvas::new(4, 1);
        c.draw_mask4(0, 0, 3, 1, &[0xf0, 0x80], 0xffffff);
        assert_eq!(c.pixels[0], 0xffffff);
        assert_eq!(c.pixels[1], 0);
        assert_eq!(c.pixels[2], mix(0xffffff, 0, 8 * 17));
    }
}
