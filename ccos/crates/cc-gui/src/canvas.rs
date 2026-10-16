use embedded_graphics::pixelcolor::{Rgb888, RgbColor};
use embedded_graphics::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Rect {
    pub x: i32,
    pub y: i32,
    pub w: i32,
    pub h: i32,
}

impl Rect {
    pub const fn new(x: i32, y: i32, w: i32, h: i32) -> Self {
        Self { x, y, w, h }
    }
    pub fn contains(&self, px: i32, py: i32) -> bool {
        px >= self.x && py >= self.y && px < self.x + self.w && py < self.y + self.h
    }
    pub fn inset(&self, d: i32) -> Self {
        Self::new(self.x + d, self.y + d, (self.w - 2 * d).max(0), (self.h - 2 * d).max(0))
    }
    pub fn center(&self) -> (i32, i32) {
        (self.x + self.w / 2, self.y + self.h / 2)
    }
}

/// XRGB8888 back buffer. Pixels are `0x00RRGGBB`.
#[derive(Clone)]
pub struct Canvas {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u32>,
    clip: Rect,
}

impl Canvas {
    pub fn new(width: u32, height: u32) -> Self {
        Self { width, height, pixels: vec![0; (width * height) as usize], clip: Rect::new(0, 0, width as i32, height as i32) }
    }

    pub fn clear(&mut self, color: u32) {
        self.pixels.fill(color);
    }

    pub fn set_clip(&mut self, r: Option<Rect>) {
        let full = Rect::new(0, 0, self.width as i32, self.height as i32);
        self.clip = match r {
            None => full,
            Some(r) => {
                let x0 = r.x.max(0);
                let y0 = r.y.max(0);
                let x1 = (r.x + r.w).min(full.w);
                let y1 = (r.y + r.h).min(full.h);
                Rect::new(x0, y0, (x1 - x0).max(0), (y1 - y0).max(0))
            }
        };
    }

    #[inline]
    pub fn put(&mut self, x: i32, y: i32, c: u32) {
        if self.clip.contains(x, y) {
            self.pixels[(y as u32 * self.width + x as u32) as usize] = c;
        }
    }

    /// Alpha-blend `c` with coverage `a` (0..=255).
    #[inline]
    pub fn blend(&mut self, x: i32, y: i32, c: u32, a: u8) {
        if a == 0 || !self.clip.contains(x, y) {
            return;
        }
        let i = (y as u32 * self.width + x as u32) as usize;
        if a == 255 {
            self.pixels[i] = c;
            return;
        }
        let d = self.pixels[i];
        let a = a as u32;
        let mix = |s: u32, d: u32| (s * a + d * (255 - a) + 127) / 255;
        let r = mix((c >> 16) & 0xff, (d >> 16) & 0xff);
        let g = mix((c >> 8) & 0xff, (d >> 8) & 0xff);
        let b = mix(c & 0xff, d & 0xff);
        self.pixels[i] = (r << 16) | (g << 8) | b;
    }

    pub fn fill_rect(&mut self, r: Rect, c: u32) {
        let x0 = r.x.max(self.clip.x);
        let y0 = r.y.max(self.clip.y);
        let x1 = (r.x + r.w).min(self.clip.x + self.clip.w);
        let y1 = (r.y + r.h).min(self.clip.y + self.clip.h);
        for y in y0..y1 {
            let row = (y as u32 * self.width) as usize;
            self.pixels[row + x0 as usize..row + x1.max(x0) as usize].fill(c);
        }
    }

    /// Filled rectangle with anti-aliased rounded corners and optional opacity.
    pub fn fill_round_rect(&mut self, r: Rect, radius: i32, c: u32, opa: u8) {
        let rad = radius.min(r.w / 2).min(r.h / 2).max(0);
        if rad == 0 && opa == 255 {
            return self.fill_rect(r, c);
        }
        for y in r.y..r.y + r.h {
            for x in r.x..r.x + r.w {
                let cov = corner_coverage(x - r.x, y - r.y, r.w, r.h, rad);
                if cov > 0 {
                    self.blend(x, y, c, ((cov as u32 * opa as u32) / 255) as u8);
                }
            }
        }
    }

    pub fn stroke_round_rect(&mut self, r: Rect, radius: i32, width: i32, c: u32) {
        let inner = r.inset(width);
        let rad = radius.min(r.w / 2).min(r.h / 2).max(0);
        let irad = (rad - width).max(0);
        for y in r.y..r.y + r.h {
            for x in r.x..r.x + r.w {
                let o = corner_coverage(x - r.x, y - r.y, r.w, r.h, rad);
                let i = if inner.contains(x, y) { corner_coverage(x - inner.x, y - inner.y, inner.w, inner.h, irad) } else { 0 };
                let cov = o.saturating_sub(i);
                self.blend(x, y, c, cov);
            }
        }
    }

    /// Blit straight-alpha RGBA pixels.
    pub fn blit_rgba(&mut self, x0: i32, y0: i32, w: u32, h: u32, rgba: &[u8], opa: u8) {
        for y in 0..h as i32 {
            for x in 0..w as i32 {
                let i = ((y as u32 * w + x as u32) * 4) as usize;
                let a = (rgba[i + 3] as u32 * opa as u32 / 255) as u8;
                let c = ((rgba[i] as u32) << 16) | ((rgba[i + 1] as u32) << 8) | rgba[i + 2] as u32;
                self.blend(x0 + x, y0 + y, c, a);
            }
        }
    }

    pub fn to_png(&self) -> Vec<u8> {
        let mut out = Vec::new();
        {
            let mut enc = png::Encoder::new(&mut out, self.width, self.height);
            enc.set_color(png::ColorType::Rgb);
            enc.set_depth(png::BitDepth::Eight);
            let mut w = enc.write_header().expect("png header");
            let data: Vec<u8> = self.pixels.iter().flat_map(|p| [(p >> 16) as u8, (p >> 8) as u8, *p as u8]).collect();
            w.write_image_data(&data).expect("png data");
        }
        out
    }
}

/// 0..=255 coverage of pixel (x, y) inside a w x h rounded rectangle.
fn corner_coverage(x: i32, y: i32, w: i32, h: i32, rad: i32) -> u8 {
    if rad == 0 {
        return 255;
    }
    let cx = if x < rad { rad } else if x >= w - rad { w - rad - 1 } else { return 255 };
    let cy = if y < rad { rad } else if y >= h - rad { h - rad - 1 } else { return 255 };
    // 4x4 supersampling around the pixel centre
    let mut n = 0;
    for sy in 0..4 {
        for sx in 0..4 {
            let dx = x as f32 + (sx as f32 + 0.5) / 4.0 - (cx as f32 + 0.5);
            let dy = y as f32 + (sy as f32 + 0.5) / 4.0 - (cy as f32 + 0.5);
            if dx * dx + dy * dy <= (rad as f32) * (rad as f32) {
                n += 1;
            }
        }
    }
    (n * 255 / 16) as u8
}

impl OriginDimensions for Canvas {
    fn size(&self) -> Size {
        Size::new(self.width, self.height)
    }
}

impl DrawTarget for Canvas {
    type Color = Rgb888;
    type Error = core::convert::Infallible;

    fn draw_iter<I: IntoIterator<Item = Pixel<Rgb888>>>(&mut self, pixels: I) -> Result<(), Self::Error> {
        for Pixel(p, c) in pixels {
            self.put(p.x, p.y, ((c.r() as u32) << 16) | ((c.g() as u32) << 8) | c.b() as u32);
        }
        Ok(())
    }

    fn clear(&mut self, c: Rgb888) -> Result<(), Self::Error> {
        Canvas::clear(self, ((c.r() as u32) << 16) | ((c.g() as u32) << 8) | c.b() as u32);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_graphics::primitives::{Circle, PrimitiveStyle};

    #[test]
    fn embedded_graphics_draws() {
        let mut c = Canvas::new(32, 32);
        Circle::new(Point::new(4, 4), 24).into_styled(PrimitiveStyle::with_fill(Rgb888::RED)).draw(&mut c).unwrap();
        assert_eq!(c.pixels[16 * 32 + 16], 0xff0000);
        assert_eq!(c.pixels[0], 0);
    }

    #[test]
    fn rounded_rect_corners_are_soft() {
        let mut c = Canvas::new(40, 40);
        c.fill_round_rect(Rect::new(0, 0, 40, 40), 10, 0xffffff, 255);
        assert_eq!(c.pixels[0], 0);
        assert_eq!(c.pixels[20 * 40 + 20], 0xffffff);
        assert!(c.to_png().starts_with(b"\x89PNG"));
    }
}
