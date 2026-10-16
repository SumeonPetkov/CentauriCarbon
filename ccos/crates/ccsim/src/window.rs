//! Desktop window: the 480×272 LCD at 2× (click = touch), a top view of the
//! bed with what has been printed, and the simulated machine's state.

use cc_ui::{Canvas, Rect, Touch};
use minifb::{Key, MouseButton, MouseMode, Window, WindowOptions};

use crate::Emu;

const SCALE: usize = 2;
const LCD_W: usize = 480 * SCALE;
const LCD_H: usize = 272 * SCALE;
const SIDE: usize = 420;
pub const W: usize = LCD_W + SIDE;
pub const H: usize = LCD_H + 120;

pub struct View {
    win: Window,
    buf: Vec<u32>,
    bed: Vec<u32>,
    info: Canvas,
    frames: u64,
}

impl View {
    pub fn open(title: &str) -> Result<View, String> {
        let mut win = Window::new(title, W, H, WindowOptions::default()).map_err(|e| format!("cannot open a window ({e}); use --headless"))?;
        win.set_target_fps(60);
        Ok(View { win, buf: vec![0x101012; W * H], bed: vec![0; SIDE * SIDE], info: Canvas::new(W as u16, 120), frames: 0 })
    }

    pub fn is_open(&self) -> bool {
        self.win.is_open() && !self.win.is_key_down(Key::Escape)
    }

    /// Mouse on the LCD area as a touch.
    pub fn touch(&self) -> Touch {
        let pressed = self.win.get_mouse_down(MouseButton::Left);
        match self.win.get_mouse_pos(MouseMode::Discard) {
            Some((x, y)) if (x as usize) < LCD_W && (y as usize) < LCD_H => Touch { x: (x as usize / SCALE) as i32, y: (y as usize / SCALE) as i32, pressed },
            _ => Touch { x: 0, y: 0, pressed: false },
        }
    }

    pub fn draw(&mut self, emu: &Emu) -> bool {
        self.frames += 1;
        // LCD, scaled
        let lcd = &emu.rt.canvas;
        for y in 0..LCD_H {
            let src = &lcd.pixels[(y / SCALE) * 480..(y / SCALE) * 480 + 480];
            let row = &mut self.buf[y * W..y * W + LCD_W];
            for (x, p) in row.iter_mut().enumerate() {
                *p = src[x / SCALE];
            }
        }
        // bed view (every few frames: rendering it is the costly part)
        if self.frames % 4 == 1 {
            emu.sim.lock().world.render_bed(&mut self.bed, SIDE);
        }
        let top = (LCD_H - SIDE) / 2;
        for y in 0..SIDE {
            self.buf[(top + y) * W + LCD_W..(top + y) * W + W].copy_from_slice(&self.bed[y * SIDE..(y + 1) * SIDE]);
        }
        // info lines
        self.info.fill_rect(Rect::new(0, 0, W as i16, 120), 0x18181a);
        let font = &ccos::ui::FONT_SMALL;
        let mut y = 6;
        let mut lines = emu.summary();
        {
            let s = emu.rt.state();
            lines.push(format!("ccos: {}", s.status));
        }
        for l in lines.iter().take(6) {
            font.draw(&mut self.info, 10, y, l, 0xd0d0d0);
            y += font.line_height as i32;
        }
        font.draw(&mut self.info, LCD_W as i32 + 10, 6, "Bed (top view) — red cross: nozzle", 0x808080);
        self.buf[LCD_H * W..].copy_from_slice(&self.info.pixels);
        self.win.update_with_buffer(&self.buf, W, H).is_ok() && self.is_open()
    }
}
