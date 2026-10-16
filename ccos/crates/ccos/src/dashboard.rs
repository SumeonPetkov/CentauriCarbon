//! Example home screen built with `cc_gui::ui`. This is the starting point for
//! your own interface: everything here is plain Rust drawn every frame.

use cc_gui::text::Align;
use cc_gui::ui::Ui;
use cc_gui::Rect;

#[derive(Debug, Clone, Default)]
pub struct McuStatus {
    pub name: &'static str,
    pub detail: String,
    pub ok: Option<bool>,
}

#[derive(Debug, Clone)]
pub struct State {
    pub page: Page,
    pub nozzle: (f64, f64),
    pub bed: (f64, f64),
    pub chamber: f64,
    pub mcus: Vec<McuStatus>,
    pub backlight: u8,
    pub log: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Page {
    Home,
    System,
}

impl Default for State {
    fn default() -> Self {
        Self {
            page: Page::Home,
            nozzle: (0.0, 0.0),
            bed: (0.0, 0.0),
            chamber: 0.0,
            mcus: vec![
                McuStatus { name: "DSP", ..Default::default() },
                McuStatus { name: "Toolhead", ..Default::default() },
                McuStatus { name: "Load cell", ..Default::default() },
            ],
            backlight: 191,
            log: vec!["ccos started".into()],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    None,
    Probe,
    Backlight(u8),
    ScreenOff,
    Exit,
}

pub fn frame(ui: &mut Ui, s: &mut State) -> Action {
    let mut action = Action::None;
    ui.clear();

    // Side bar
    let dim = ui.theme.text_dim;
    if ui.toggle(Rect::new(6, 6, 64, 126), "Home", s.page == Page::Home) {
        s.page = Page::Home;
    }
    if ui.toggle(Rect::new(6, 140, 64, 126), "Sys", s.page == Page::System) {
        s.page = Page::System;
    }

    match s.page {
        Page::Home => {
            ui.temp_tile(Rect::new(80, 6, 126, 90), "Nozzle", s.nozzle.0, s.nozzle.1);
            ui.temp_tile(Rect::new(214, 6, 126, 90), "Bed", s.bed.0, s.bed.1);
            ui.temp_tile(Rect::new(348, 6, 126, 90), "Chamber", s.chamber, 0.0);
            ui.panel(Rect::new(80, 104, 394, 162));
            ui.label(Rect::new(92, 110, 300, 20), "MCU links", 14.0, dim, Align::Left);
            for (i, m) in s.mcus.iter().enumerate() {
                let y = 134 + i as i32 * 30;
                let (dot, col) = match m.ok {
                    Some(true) => ("●", ui.theme.accent),
                    Some(false) => ("●", ui.theme.danger),
                    None => ("○", dim),
                };
                ui.label(Rect::new(92, y, 20, 24), dot, 16.0, col, Align::Left);
                let txt = ui.theme.text;
                ui.label(Rect::new(114, y, 100, 24), m.name, 16.0, txt, Align::Left);
                ui.label(Rect::new(214, y, 250, 24), &m.detail, 13.0, dim, Align::Left);
            }
            if ui.button(Rect::new(362, 222, 104, 36), "Probe") {
                action = Action::Probe;
            }
        }
        Page::System => {
            ui.panel(Rect::new(80, 6, 394, 120));
            ui.label(Rect::new(92, 12, 300, 20), "Backlight", 14.0, dim, Align::Left);
            ui.progress(Rect::new(92, 44, 370, 14), s.backlight as f32 / 255.0);
            if ui.button(Rect::new(92, 70, 80, 44), "−") {
                s.backlight = s.backlight.saturating_sub(32);
                action = Action::Backlight(s.backlight);
            }
            if ui.button(Rect::new(182, 70, 80, 44), "+") {
                s.backlight = s.backlight.saturating_add(32);
                action = Action::Backlight(s.backlight);
            }
            if ui.button(Rect::new(272, 70, 90, 44), "Off") {
                action = Action::ScreenOff;
            }
            if ui.button(Rect::new(372, 70, 90, 44), "Exit") {
                action = Action::Exit;
            }
            ui.panel(Rect::new(80, 134, 394, 132));
            let txt = ui.theme.text;
            for (i, l) in s.log.iter().rev().take(6).enumerate() {
                ui.label(Rect::new(92, 140 + i as i32 * 20, 370, 20), l, 13.0, txt, Align::Left);
            }
        }
    }
    action
}
