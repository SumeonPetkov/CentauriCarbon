//! PNG loading (the stock UI ships 400+ PNGs in `/app/resources/images/`).

use std::collections::HashMap;
use std::path::{Path, PathBuf};

pub struct Image {
    pub width: u32,
    pub height: u32,
    /// Straight-alpha RGBA8.
    pub rgba: Vec<u8>,
}

impl Image {
    pub fn decode_png(data: &[u8]) -> Result<Self, String> {
        let mut dec = png::Decoder::new(data);
        dec.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
        let mut reader = dec.read_info().map_err(|e| e.to_string())?;
        let mut buf = vec![0; reader.output_buffer_size()];
        let info = reader.next_frame(&mut buf).map_err(|e| e.to_string())?;
        let px = (info.width * info.height) as usize;
        let rgba = match info.color_type {
            png::ColorType::Rgba => buf[..px * 4].to_vec(),
            png::ColorType::Rgb => buf[..px * 3].chunks(3).flat_map(|c| [c[0], c[1], c[2], 255]).collect(),
            png::ColorType::GrayscaleAlpha => buf[..px * 2].chunks(2).flat_map(|c| [c[0], c[0], c[0], c[1]]).collect(),
            png::ColorType::Grayscale => buf[..px].iter().flat_map(|&g| [g, g, g, 255]).collect(),
            png::ColorType::Indexed => return Err("indexed PNG not expanded".into()),
        };
        Ok(Self { width: info.width, height: info.height, rgba })
    }

    pub fn load(path: &Path) -> Result<Self, String> {
        Self::decode_png(&std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?)
    }
}

/// Lazily loads `<dir>/<index>.png`, like the stock `IMAGE_RESOURCES_PATH`.
pub struct ImageStore {
    dir: PathBuf,
    cache: HashMap<i32, Option<Image>>,
}

impl ImageStore {
    pub fn new(dir: impl Into<PathBuf>) -> Self {
        Self { dir: dir.into(), cache: HashMap::new() }
    }
    pub fn get(&mut self, index: i32) -> Option<&Image> {
        let dir = &self.dir;
        self.cache.entry(index).or_insert_with(|| Image::load(&dir.join(format!("{index}.png"))).ok()).as_ref()
    }
}
