//! A small, fast GUI stack for the Centauri Carbon's 480x272 touchscreen.
//!
//! * [`Canvas`] — an XRGB8888 back buffer that implements
//!   `embedded_graphics::DrawTarget`, so the whole embedded-graphics ecosystem
//!   (shapes, charts, image decoders) works on it. Saves to PNG for host-side
//!   previews.
//! * [`text::Font`] — TrueType/TTC text via fontdue (the stock
//!   `/app/resources/fonts/font.ttc` covers all 11 UI languages incl. CJK).
//! * [`fb::Framebuffer`] / [`touch::Touch`] — Linux `/dev/fb0` and evdev
//!   `/dev/input/event1`, the same devices the stock LVGL app drives.
//! * [`ui`] — a tiny immediate-mode widget layer (buttons, labels, panels).
//! * [`preview`] — renders windows from the stock `widget.bin` so you can see
//!   and edit Elegoo's screens on a PC.

pub mod canvas;
#[cfg(target_os = "linux")]
pub mod fb;
pub mod image;
pub mod preview;
pub mod text;
#[cfg(target_os = "linux")]
pub mod touch;
pub mod ui;

pub use canvas::{Canvas, Rect};

pub const SCREEN_W: u32 = 480;
pub const SCREEN_H: u32 = 272;

/// `0xRRGGBB` helper.
pub const fn rgb(v: u32) -> u32 {
    v & 0x00ff_ffff
}
