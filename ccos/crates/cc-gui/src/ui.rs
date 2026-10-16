//! Immediate-mode widgets. Each frame you describe the screen; widgets return
//! whether they were tapped. No retained tree, no allocation per frame.
//!
//! ```ignore
//! let mut ui = Ui::new(&mut canvas, &mut font, touch_state, &mut input, &theme);
//! ui.clear();
//! if ui.button(Rect::new(10, 10, 120, 50), "Home all") { /* send G28 */ }
//! ```

use crate::canvas::{Canvas, Rect};
use crate::text::{Align, Font};

#[derive(Debug, Clone, Copy)]
pub struct Theme {
    pub bg: u32,
    pub panel: u32,
    pub panel_pressed: u32,
    pub accent: u32,
    pub text: u32,
    pub text_dim: u32,
    pub danger: u32,
    pub radius: i32,
    pub font_px: f32,
}

impl Default for Theme {
    /// Close to the stock dark theme (`#1E1E20` background, `#383837` tiles).
    fn default() -> Self {
        Self {
            bg: 0x1e1e20,
            panel: 0x383837,
            panel_pressed: 0x505050,
            accent: 0x22d000,
            text: 0xffffff,
            text_dim: 0xa0a0a0,
            danger: 0xe03030,
            radius: 10,
            font_px: 18.0,
        }
    }
}

/// Edge-detection state that survives between frames.
#[derive(Debug, Default, Clone, Copy)]
pub struct InputState {
    pub x: i32,
    pub y: i32,
    pub pressed: bool,
    was_pressed: bool,
    press_x: i32,
    press_y: i32,
}

impl InputState {
    /// Call once per frame with the latest touch sample.
    pub fn update(&mut self, x: i32, y: i32, pressed: bool) {
        self.was_pressed = self.pressed;
        if pressed && !self.pressed {
            self.press_x = x;
            self.press_y = y;
        }
        self.x = x;
        self.y = y;
        self.pressed = pressed;
    }
    fn released_in(&self, r: Rect) -> bool {
        self.was_pressed && !self.pressed && r.contains(self.press_x, self.press_y) && r.contains(self.x, self.y)
    }
    fn held_in(&self, r: Rect) -> bool {
        self.pressed && r.contains(self.press_x, self.press_y) && r.contains(self.x, self.y)
    }
}

pub struct Ui<'a> {
    pub canvas: &'a mut Canvas,
    pub font: &'a mut Font,
    pub input: InputState,
    pub theme: Theme,
}

impl<'a> Ui<'a> {
    pub fn new(canvas: &'a mut Canvas, font: &'a mut Font, input: InputState, theme: Theme) -> Self {
        Self { canvas, font, input, theme }
    }

    pub fn clear(&mut self) {
        self.canvas.clear(self.theme.bg);
    }

    pub fn panel(&mut self, r: Rect) {
        self.canvas.fill_round_rect(r, self.theme.radius, self.theme.panel, 255);
    }

    pub fn label(&mut self, r: Rect, text: &str, px: f32, color: u32, align: Align) {
        self.font.draw_in(self.canvas, r, text, px, color, align);
    }

    pub fn text(&mut self, r: Rect, text: &str) {
        let (px, c) = (self.theme.font_px, self.theme.text);
        self.label(r, text, px, c, Align::Left);
    }

    /// A tappable tile. Returns true on the frame the finger lifts inside it.
    pub fn button(&mut self, r: Rect, text: &str) -> bool {
        let held = self.input.held_in(r);
        let bg = if held { self.theme.panel_pressed } else { self.theme.panel };
        self.canvas.fill_round_rect(r, self.theme.radius, bg, 255);
        let (px, c) = (self.theme.font_px, self.theme.text);
        self.label(r.inset(4), text, px, c, Align::Center);
        self.input.released_in(r)
    }

    /// Toggle-style button with an accent outline when `on`.
    pub fn toggle(&mut self, r: Rect, text: &str, on: bool) -> bool {
        let clicked = self.button(r, text);
        if on {
            self.canvas.stroke_round_rect(r, self.theme.radius, 2, self.theme.accent);
        }
        clicked
    }

    /// Horizontal progress bar, `frac` in 0..=1.
    pub fn progress(&mut self, r: Rect, frac: f32) {
        let rad = r.h / 2;
        self.canvas.fill_round_rect(r, rad, self.theme.panel, 255);
        let w = ((r.w as f32) * frac.clamp(0.0, 1.0)).round() as i32;
        if w > 0 {
            self.canvas.fill_round_rect(Rect::new(r.x, r.y, w.max(r.h), r.h), rad, self.theme.accent, 255);
        }
    }

    /// Temperature tile: big current value, small target.
    pub fn temp_tile(&mut self, r: Rect, title: &str, current: f64, target: f64) -> bool {
        let clicked = self.button(r, "");
        let dim = self.theme.text_dim;
        let txt = self.theme.text;
        self.label(Rect::new(r.x + 8, r.y + 4, r.w - 16, 18), title, 14.0, dim, Align::Left);
        self.label(Rect::new(r.x + 8, r.y + 20, r.w - 16, r.h - 40), &format!("{current:.0}°"), 28.0, txt, Align::Left);
        let t = if target > 0.0 { format!("→ {target:.0}°") } else { "off".to_string() };
        self.label(Rect::new(r.x + 8, r.y + r.h - 22, r.w - 16, 18), &t, 14.0, dim, Align::Left);
        clicked
    }
}
