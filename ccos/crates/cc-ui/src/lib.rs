//! Ahead-of-time compiled UI runtime for the Centauri Carbon's 480x272
//! touchscreen.
//!
//! Everything a screen needs — layout, English text, fonts (pre-rasterised
//! to 4-bit glyphs), icons (pre-converted, single-colour ones as 4-bit
//! masks) — is generated into `static` data by `cc-ui-codegen` from the
//! application's `ui/` folder. At runtime there is no parsing, no font
//! engine, no image decoder and no per-frame allocation; the [`Engine`]
//! redraws only rectangles whose content changed.
//!
//! [`Engine`]: screen::Engine

pub mod canvas;
#[cfg(target_os = "linux")]
pub mod fb;
pub mod icon;
pub mod png;
pub mod screen;
pub mod text;
#[cfg(target_os = "linux")]
pub mod touch;

pub use canvas::{Canvas, Rect};
pub use icon::{Icon, IconData};
pub use screen::{heat_color, Action, Cond, Dirty, Engine, Kind, Matrix, Model, Screen, Touch, Widget, MATRIX_MAX};
pub use text::{Align, Font, Glyph, TextBuf};
