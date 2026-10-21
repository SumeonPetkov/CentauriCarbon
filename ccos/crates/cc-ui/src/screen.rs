//! The screen engine. Screens and widgets are `static` data generated from
//! `ui/*.toml` at build time; the app supplies live values through
//! [`Model`]. Each update hashes every live widget's content and redraws only
//! the rectangles that changed.

use crate::canvas::{Canvas, Rect};
use crate::icon::Icon;
use crate::text::{Align, Font, TextBuf};

/// Live data the UI shows. `V` and `F` are enums generated from the names
/// used in `ui/screens/*.toml`, so a missing case is a compile error.
pub trait Model<V: Copy, F: Copy> {
    /// Text for a `value = "..."` binding.
    fn text(&self, v: V, out: &mut TextBuf);
    /// 0.0..=1.0 for bars (`type = "bar"`).
    fn level(&self, _v: V) -> f32 {
        0.0
    }
    /// Condition for `show_if` / `on_if`.
    fn flag(&self, f: F) -> bool;
    /// Cells for heatmaps (`type = "heatmap"`); leave `out` empty for none.
    fn matrix(&self, _v: V, _out: &mut Matrix) {}
}

/// Most cells a heatmap shows (32 × 32).
pub const MATRIX_MAX: usize = 32 * 32;

/// A grid of values for `type = "heatmap"`, row 0 first (drawn at the
/// bottom). Fixed size, so filling it every frame allocates nothing.
#[derive(Clone)]
pub struct Matrix {
    pub cols: u8,
    pub rows: u8,
    pub z: [f32; MATRIX_MAX],
}

impl Default for Matrix {
    fn default() -> Self {
        Self { cols: 0, rows: 0, z: [0.0; MATRIX_MAX] }
    }
}

impl Matrix {
    pub fn clear(&mut self) {
        self.cols = 0;
        self.rows = 0;
    }

    /// Set the size; false (and empty) if it does not fit.
    pub fn resize(&mut self, cols: usize, rows: usize) -> bool {
        if cols == 0 || rows == 0 || cols > 255 || rows > 255 || cols * rows > MATRIX_MAX {
            self.clear();
            return false;
        }
        self.cols = cols as u8;
        self.rows = rows as u8;
        true
    }

    pub fn cells(&self) -> &[f32] {
        &self.z[..self.cols as usize * self.rows as usize]
    }

    pub fn get(&self, col: usize, row: usize) -> f32 {
        self.z[row * self.cols as usize + col]
    }

    pub fn range(&self) -> (f32, f32) {
        self.cells().iter().fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), &v| (lo.min(v), hi.max(v)))
    }
}

/// Heatmap colour for `t` in 0..=1: blue (low) → green → red (high).
pub fn heat_color(t: f32) -> u32 {
    const STOPS: [(f32, [f32; 3]); 5] =
        [(0.0, [40.0, 90.0, 230.0]), (0.25, [40.0, 190.0, 230.0]), (0.5, [60.0, 200.0, 90.0]), (0.75, [240.0, 200.0, 40.0]), (1.0, [230.0, 60.0, 50.0])];
    let t = if t.is_nan() { 0.5 } else { t.clamp(0.0, 1.0) };
    let i = STOPS.iter().rposition(|(s, _)| *s <= t).unwrap_or(0).min(STOPS.len() - 2);
    let (s0, c0) = STOPS[i];
    let (s1, c1) = STOPS[i + 1];
    let f = (t - s0) / (s1 - s0);
    let ch = |k: usize| (c0[k] + (c1[k] - c0[k]) * f).round() as u32;
    (ch(0) << 16) | (ch(1) << 8) | ch(2)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action<C> {
    /// Switch to screen index (handled by the engine, also reported).
    Goto(u8),
    /// Return to the previous screen (handled by the engine, also reported).
    Back,
    /// A G-code line for the printer core.
    Gcode(&'static str),
    /// An app command (generated enum).
    Cmd(C),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Panel,
    Label,
    Button,
    Icon,
    Temp,
    Bar,
    Toggle,
    /// A grid of coloured cells from [`Model::matrix`].
    Heatmap,
}

#[derive(Debug, Clone, Copy)]
pub struct Cond<F> {
    pub flag: F,
    pub negate: bool,
}

#[derive(Debug)]
pub struct Widget<C: 'static, V: 'static, F: 'static> {
    pub id: &'static str,
    pub kind: Kind,
    pub rect: Rect,
    /// Static text (label/button) or title (temp tile).
    pub text: &'static str,
    /// Live text / main value.
    pub value: Option<V>,
    /// Secondary value (temp tile target).
    pub value2: Option<V>,
    pub icon: Option<&'static Icon>,
    /// Icon used while `on_if` holds.
    pub icon_on: Option<&'static Icon>,
    pub font: &'static Font,
    pub font_small: &'static Font,
    pub fg: u32,
    pub fg_dim: u32,
    pub bg: Option<u32>,
    pub bg_pressed: u32,
    pub accent: u32,
    pub radius: i16,
    pub align: Align,
    pub tap: Option<Action<C>>,
    pub show_if: Option<Cond<F>>,
    pub on_if: Option<Cond<F>>,
    /// Statically "on" (e.g. the sidebar entry of the current screen,
    /// resolved at build time).
    pub on: bool,
}

#[derive(Debug)]
pub struct Screen<C: 'static, V: 'static, F: 'static> {
    pub name: &'static str,
    pub bg: u32,
    pub widgets: &'static [Widget<C, V, F>],
}

/// Most widgets a screen may have (checked at build time).
pub const MAX_WIDGETS: usize = 96;
const MAX_DIRTY: usize = 8;

#[derive(Debug, Clone, Copy, Default)]
pub struct Touch {
    pub x: i32,
    pub y: i32,
    pub pressed: bool,
}

/// Rectangles redrawn by the last [`Engine::update`]; present these.
pub struct Dirty {
    rects: [Rect; MAX_DIRTY],
    n: usize,
}

impl Dirty {
    fn add(&mut self, r: Rect) {
        if r.is_empty() {
            return;
        }
        for i in 0..self.n {
            if self.rects[i].intersects(&r) {
                self.rects[i] = self.rects[i].union(&r);
                return;
            }
        }
        if self.n < MAX_DIRTY {
            self.rects[self.n] = r;
            self.n += 1;
        } else {
            // Out of slots: merge everything.
            let mut u = r;
            for x in &self.rects[..self.n] {
                u = u.union(x);
            }
            self.rects[0] = u;
            self.n = 1;
        }
    }
    pub fn rects(&self) -> &[Rect] {
        &self.rects[..self.n]
    }
    pub fn is_empty(&self) -> bool {
        self.n == 0
    }
}

pub struct Engine<C: 'static, V: 'static, F: 'static> {
    screens: &'static [Screen<C, V, F>],
    cur: u8,
    stack: [u8; 8],
    depth: usize,
    pressed: Option<usize>,
    was_down: bool,
    sig: [u64; MAX_WIDGETS],
    full: bool,
    dirty: Dirty,
    tmp: TextBuf,
    mat: Box<Matrix>,
}

fn fnv(h: &mut u64, bytes: &[u8]) {
    for &b in bytes {
        *h ^= b as u64;
        *h = h.wrapping_mul(0x100_0000_01b3);
    }
}

impl<C: Copy, V: Copy, F: Copy> Engine<C, V, F> {
    pub fn new(screens: &'static [Screen<C, V, F>], start: u8) -> Self {
        Self {
            screens,
            cur: start,
            stack: [0; 8],
            depth: 0,
            pressed: None,
            was_down: false,
            sig: [0; MAX_WIDGETS],
            full: true,
            dirty: Dirty { rects: [Rect::default(); MAX_DIRTY], n: 0 },
            tmp: TextBuf::default(),
            mat: Box::default(),
        }
    }

    pub fn current(&self) -> u8 {
        self.cur
    }

    pub fn screen(&self) -> &'static Screen<C, V, F> {
        &self.screens[self.cur as usize]
    }

    /// Force a full redraw on the next update.
    pub fn invalidate(&mut self) {
        self.full = true;
    }

    pub fn goto(&mut self, s: u8) {
        if s != self.cur && (s as usize) < self.screens.len() {
            if self.depth == self.stack.len() {
                self.stack.copy_within(1.., 0);
                self.depth -= 1;
            }
            self.stack[self.depth] = self.cur;
            self.depth += 1;
            self.cur = s;
            self.pressed = None;
            self.full = true;
        }
    }

    pub fn back(&mut self) {
        if self.depth > 0 {
            self.depth -= 1;
            self.cur = self.stack[self.depth];
            self.pressed = None;
            self.full = true;
        }
    }

    fn visible(&self, w: &Widget<C, V, F>, m: &impl Model<V, F>) -> bool {
        w.show_if.map_or(true, |c| m.flag(c.flag) != c.negate)
    }

    fn hit(&self, x: i32, y: i32, m: &impl Model<V, F>) -> Option<usize> {
        let ws = self.screen().widgets;
        (0..ws.len()).rev().find(|&i| ws[i].tap.is_some() && ws[i].rect.contains(x, y) && self.visible(&ws[i], m))
    }

    /// Feed one touch sample and the model; redraws what changed into
    /// `canvas`. Returns the triggered action (navigation is already
    /// applied) and the rectangles to present.
    pub fn update(&mut self, m: &impl Model<V, F>, touch: Touch, canvas: &mut Canvas) -> (Option<Action<C>>, &Dirty) {
        self.dirty.n = 0;
        let mut action = None;
        let ws = self.screen().widgets;

        // Touch edges
        if touch.pressed && !self.was_down {
            self.pressed = self.hit(touch.x, touch.y, m);
            if let Some(i) = self.pressed {
                self.dirty.add(ws[i].rect);
            }
        } else if !touch.pressed && self.was_down {
            if let Some(i) = self.pressed.take() {
                self.dirty.add(ws[i].rect);
                if ws[i].rect.contains(touch.x, touch.y) {
                    action = ws[i].tap;
                }
            }
        }
        self.was_down = touch.pressed;

        match action {
            Some(Action::Goto(s)) => self.goto(s),
            Some(Action::Back) => self.back(),
            _ => {}
        }

        // Content changes
        let ws = self.screen().widgets;
        for (i, w) in ws.iter().enumerate().take(MAX_WIDGETS) {
            let mut h: u64 = 0xcbf2_9ce4_8422_2325;
            let vis = self.visible(w, m);
            fnv(&mut h, &[vis as u8, (self.pressed == Some(i)) as u8]);
            if let Some(c) = w.on_if {
                fnv(&mut h, &[(m.flag(c.flag) != c.negate) as u8]);
            }
            for v in [w.value, w.value2].into_iter().flatten() {
                self.tmp.clear();
                m.text(v, &mut self.tmp);
                fnv(&mut h, self.tmp.as_str().as_bytes());
                if w.kind == Kind::Bar {
                    fnv(&mut h, &((m.level(v).clamp(0.0, 1.0) * 1000.0) as u32).to_le_bytes());
                }
            }
            if let (Kind::Heatmap, Some(v)) = (w.kind, w.value) {
                self.mat.clear();
                m.matrix(v, &mut self.mat);
                fnv(&mut h, &[self.mat.cols, self.mat.rows]);
                for z in self.mat.cells() {
                    fnv(&mut h, &z.to_le_bytes());
                }
            }
            if self.sig[i] != h {
                self.sig[i] = h;
                if !self.full {
                    self.dirty.add(w.rect);
                }
            }
        }

        if self.full {
            self.full = false;
            self.dirty.n = 0;
            self.dirty.add(canvas.bounds());
        }

        // Redraw
        let screen = self.screen();
        for k in 0..self.dirty.n {
            let r = self.dirty.rects[k];
            canvas.set_clip(r);
            canvas.fill_rect(r, screen.bg);
            for (i, w) in screen.widgets.iter().enumerate() {
                if w.rect.intersects(&r) && self.visible(w, m) {
                    if let (Kind::Heatmap, Some(v)) = (w.kind, w.value) {
                        self.mat.clear();
                        m.matrix(v, &mut self.mat);
                    }
                    draw_widget(canvas, w, m, self.pressed == Some(i), &mut self.tmp, &self.mat);
                }
            }
        }
        canvas.set_clip(canvas.bounds());
        (action, &self.dirty)
    }
}

fn draw_widget<C, V: Copy, F: Copy>(c: &mut Canvas, w: &Widget<C, V, F>, m: &impl Model<V, F>, pressed: bool, tmp: &mut TextBuf, mat: &Matrix) {
    let on = w.on || w.on_if.is_some_and(|k| m.flag(k.flag) != k.negate);
    let r = w.rect;
    let bg = if pressed { Some(w.bg_pressed) } else { w.bg };
    if let Some(bg) = bg {
        c.fill_round_rect(r, w.radius, bg, 255);
    }
    match w.kind {
        Kind::Panel => {}
        Kind::Label => {
            let text = match w.value {
                Some(v) => {
                    tmp.clear();
                    m.text(v, tmp);
                    tmp.as_str()
                }
                None => w.text,
            };
            w.font.draw_in(c, r, text, w.fg, w.align);
        }
        Kind::Icon => {
            if let Some(icon) = if on { w.icon_on.or(w.icon) } else { w.icon } {
                icon.draw(c, r.x as i32 + (r.w as i32 - icon.w as i32) / 2, r.y as i32 + (r.h as i32 - icon.h as i32) / 2, None);
            }
        }
        Kind::Button | Kind::Toggle => {
            let icon = if on { w.icon_on.or(w.icon) } else { w.icon };
            let text = match w.value {
                Some(v) => {
                    tmp.clear();
                    m.text(v, tmp);
                    tmp.as_str()
                }
                None => w.text,
            };
            let lh = w.font.line_height as i32;
            match icon {
                Some(icon) if !text.is_empty() => {
                    let total = icon.h as i32 + 2 + lh;
                    let y0 = r.y as i32 + (r.h as i32 - total) / 2;
                    icon.draw(c, r.x as i32 + (r.w as i32 - icon.w as i32) / 2, y0, None);
                    w.font.draw_in(c, Rect::new(r.x, (y0 + icon.h as i32 + 2) as i16, r.w, lh as i16), text, w.fg, Align::Center);
                }
                Some(icon) => {
                    icon.draw(c, r.x as i32 + (r.w as i32 - icon.w as i32) / 2, r.y as i32 + (r.h as i32 - icon.h as i32) / 2, None)
                }
                None => w.font.draw_in(c, r.inset(4), text, w.fg, w.align),
            }
            if on && w.kind == Kind::Toggle {
                c.stroke_round_rect(r, w.radius, 2, w.accent);
            }
        }
        Kind::Temp => {
            let small = w.font_small;
            let pad = 8;
            let mut x = r.x as i32 + pad;
            if let Some(icon) = if on { w.icon_on.or(w.icon) } else { w.icon } {
                icon.draw(c, x, r.y as i32 + 6, None);
                x += icon.w as i32 + 4;
            }
            small.draw_in(c, Rect::new(x as i16, r.y + 4, r.right() - x as i16 - pad as i16, small.line_height as i16), w.text, w.fg_dim, Align::Left);
            if let Some(v) = w.value {
                tmp.clear();
                m.text(v, tmp);
                let y = r.y + (r.h - w.font.line_height as i16) / 2 + 2;
                w.font.draw_in(c, Rect::new(r.x + pad as i16, y, r.w - 2 * pad as i16, w.font.line_height as i16), tmp.as_str(), w.fg, Align::Left);
            }
            if let Some(v) = w.value2 {
                tmp.clear();
                m.text(v, tmp);
                let y = r.bottom() - small.line_height as i16 - 4;
                small.draw_in(c, Rect::new(r.x + pad as i16, y, r.w - 2 * pad as i16, small.line_height as i16), tmp.as_str(), w.fg_dim, Align::Left);
            }
            if on {
                c.stroke_round_rect(r, w.radius, 2, w.accent);
            }
        }
        Kind::Bar => {
            let level = w.value.map_or(0.0, |v| m.level(v)).clamp(0.0, 1.0);
            let rad = r.h / 2;
            if bg.is_none() {
                c.fill_round_rect(r, rad, w.bg_pressed, 255);
            }
            let fw = (r.w as f32 * level).round() as i16;
            if fw > 0 {
                c.fill_round_rect(Rect::new(r.x, r.y, fw.max(r.h), r.h), rad, w.accent, 255);
            }
        }
        Kind::Heatmap => draw_heatmap(c, w, mat, tmp),
    }
}

/// Square cells, as large as fit, centred; row 0 at the bottom. Colours
/// span the matrix's own range. Cells wide enough also show their value.
fn draw_heatmap<C, V, F>(c: &mut Canvas, w: &Widget<C, V, F>, mat: &Matrix, tmp: &mut TextBuf) {
    let (cols, rows) = (mat.cols as i32, mat.rows as i32);
    if cols == 0 || rows == 0 {
        return;
    }
    let r = w.rect.inset(4);
    let cell = (r.w as i32 / cols).min(r.h as i32 / rows);
    if cell < 2 {
        return;
    }
    let x0 = r.x as i32 + (r.w as i32 - cell * cols) / 2;
    let y0 = r.y as i32 + (r.h as i32 - cell * rows) / 2;
    let (lo, hi) = mat.range();
    let span = hi - lo;
    let gap = if cell >= 8 { 1 } else { 0 };
    let small = w.font_small;
    let label = cell >= 44 && cell >= small.line_height as i32 + 4;
    for row in 0..rows {
        for col in 0..cols {
            let v = mat.get(col as usize, row as usize);
            let t = if span > 1e-6 { (v - lo) / span } else { 0.5 };
            let x = x0 + col * cell;
            let y = y0 + (rows - 1 - row) * cell;
            let cr = Rect::new(x as i16, y as i16, (cell - gap) as i16, (cell - gap) as i16);
            c.fill_rect(cr, heat_color(t));
            if label {
                use core::fmt::Write;
                tmp.clear();
                let _ = write!(tmp, "{v:.2}");
                small.draw_in(c, cr, tmp.as_str(), 0x101010, Align::Center);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static FONT: Font = Font { line_height: 10, glyphs: &[], ascii: [u16::MAX; 96], data: &[] };

    #[derive(Clone, Copy, Debug)]
    enum V {
        Temp,
    }
    #[derive(Clone, Copy, Debug)]
    enum F {
        Hot,
    }
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum C {
        Ping,
    }

    struct M {
        t: i32,
        hot: bool,
    }
    impl Model<V, F> for M {
        fn text(&self, _v: V, out: &mut TextBuf) {
            use core::fmt::Write;
            let _ = write!(out, "{}", self.t);
        }
        fn flag(&self, _f: F) -> bool {
            self.hot
        }
    }

    const fn w(kind: Kind, rect: Rect, value: Option<V>, tap: Option<Action<C>>, show_if: Option<Cond<F>>) -> Widget<C, V, F> {
        Widget {
            id: "",
            kind,
            rect,
            text: "",
            value,
            value2: None,
            icon: None,
            icon_on: None,
            font: &FONT,
            font_small: &FONT,
            fg: 0xffffff,
            fg_dim: 0x888888,
            bg: Some(0x333333),
            bg_pressed: 0x555555,
            accent: 0x00ff00,
            radius: 4,
            align: Align::Left,
            tap,
            show_if,
            on_if: None,
            on: false,
        }
    }

    static W0: [Widget<C, V, F>; 3] = [
        w(Kind::Label, Rect::new(0, 0, 50, 20), Some(V::Temp), None, None),
        w(Kind::Button, Rect::new(0, 30, 50, 20), None, Some(Action::Cmd(C::Ping)), None),
        w(Kind::Button, Rect::new(60, 30, 50, 20), None, Some(Action::Goto(1)), Some(Cond { flag: F::Hot, negate: true })),
    ];
    static W1: [Widget<C, V, F>; 1] = [w(Kind::Button, Rect::new(0, 0, 100, 100), None, Some(Action::Back), None)];
    static SCREENS: [Screen<C, V, F>; 2] = [Screen { name: "a", bg: 0x101010, widgets: &W0 }, Screen { name: "b", bg: 0x202020, widgets: &W1 }];

    fn tap(e: &mut Engine<C, V, F>, m: &M, c: &mut Canvas, x: i32, y: i32) -> Option<Action<C>> {
        e.update(m, Touch { x, y, pressed: true }, c);
        e.update(m, Touch { x, y, pressed: false }, c).0
    }

    #[test]
    fn first_frame_is_full_then_idle_is_free() {
        let mut c = Canvas::new(120, 100);
        let mut e = Engine::new(&SCREENS, 0);
        let m = M { t: 20, hot: false };
        let (_, d) = e.update(&m, Touch::default(), &mut c);
        assert_eq!(d.rects(), &[c.bounds()]);
        let (_, d) = e.update(&m, Touch::default(), &mut c);
        assert!(d.is_empty(), "nothing changed -> nothing redrawn");
    }

    #[test]
    fn value_change_redraws_only_that_widget() {
        let mut c = Canvas::new(120, 100);
        let mut e = Engine::new(&SCREENS, 0);
        e.update(&M { t: 20, hot: false }, Touch::default(), &mut c);
        let (_, d) = e.update(&M { t: 21, hot: false }, Touch::default(), &mut c);
        assert_eq!(d.rects(), &[Rect::new(0, 0, 50, 20)]);
    }

    #[test]
    fn taps_actions_and_navigation() {
        let mut c = Canvas::new(120, 100);
        let mut e = Engine::new(&SCREENS, 0);
        let m = M { t: 20, hot: false };
        e.update(&m, Touch::default(), &mut c);
        assert_eq!(tap(&mut e, &m, &mut c, 5, 35), Some(Action::Cmd(C::Ping)));
        assert_eq!(tap(&mut e, &m, &mut c, 65, 35), Some(Action::Goto(1)));
        assert_eq!(e.current(), 1);
        assert_eq!(tap(&mut e, &m, &mut c, 5, 5), Some(Action::Back));
        assert_eq!(e.current(), 0);
        // Hidden widgets cannot be tapped.
        let hot = M { t: 20, hot: true };
        assert_eq!(tap(&mut e, &hot, &mut c, 65, 35), None);
        // Releasing outside cancels.
        e.update(&m, Touch { x: 5, y: 35, pressed: true }, &mut c);
        assert_eq!(e.update(&m, Touch { x: 100, y: 90, pressed: false }, &mut c).0, None);
    }

    #[test]
    fn dirty_merges_when_full() {
        let mut d = Dirty { rects: [Rect::default(); MAX_DIRTY], n: 0 };
        for i in 0..20 {
            d.add(Rect::new(i * 20, 0, 5, 5));
        }
        assert!(d.n <= MAX_DIRTY);
        let u = d.rects().iter().fold(Rect::default(), |a, r| a.union(r));
        assert!(u.contains(380, 0));
    }
}
