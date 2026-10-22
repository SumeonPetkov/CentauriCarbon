//! The printer configuration, read and written like Elegoo's `ConfigParser`
//! (firmware/core/klippy/configfile.cpp), so files written by ccos and by the
//! stock firmware are interchangeable:
//!
//! * `key : value` (also `key: value`), `[section]` headers;
//! * a line is cut at the first `;`, `#`, `\r`, `\n` or NUL — comment lines
//!   are kept only as blank lines before the next entry;
//! * duplicate sections merge and the **first** value of a key wins;
//! * `user_printer.cfg` overrides values (or adds sections) on top;
//! * numbers are written with `std::to_string` (six decimals);
//! * `write_specified` rewrites one section of a file in place, the same
//!   (non-truncating) way the firmware does.
//!
//! On the printer: `/board-resource/printer.cfg` (copied from
//! `/app/resources/configs/printer.cfg` if missing), `user_printer.cfg` and
//! `unmodifiable.cfg` in the same folder.

use std::path::{Path, PathBuf};

/// The stock configuration shipped with the firmware (default when no file
/// is found, and for the emulator).
pub const STOCK_PRINTER_CFG: &str = include_str!("../../../../firmware/resources/e100/configs/printer.cfg");
pub const STOCK_UNMODIFIABLE_CFG: &str = include_str!("../../../../firmware/resources/e100/configs/unmodifiable.cfg");
pub const STOCK_SYSCONF_CFG: &str = include_str!("../../../../firmware/resources/e100/configs/sysconf.cfg");

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Opt {
    pub name: String,
    pub value: String,
    pub comment: String,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Section {
    pub name: String,
    pub comment: String,
    pub options: Vec<Opt>,
}

#[derive(Debug, Clone, Default)]
pub struct Config {
    pub sections: Vec<Section>,
    /// Where it was read from (for saving).
    pub path: Option<PathBuf>,
}

fn trim_line_end(s: &str) -> &str {
    match s.find(['\0', '\r', '\n', ';', '#']) {
        Some(p) => &s[..p],
        None => s,
    }
}

/// `TrimLeadTrailSpecStr`: spaces and tabs.
fn trim_ws(s: &str) -> &str {
    s.trim_matches([' ', '\t'])
}

/// `istringstream >> double`: longest numeric prefix after whitespace; 0 on failure.
pub fn parse_double(s: &str) -> f64 {
    crate::gcode::parse::stod(s).unwrap_or(0.0)
}

/// `istringstream >> int`.
pub fn parse_int(s: &str) -> i64 {
    crate::gcode::parse::stoi(s).unwrap_or(0)
}

/// `std::to_string(double)`.
pub fn to_string(v: f64) -> String {
    format!("{v:.6}")
}

impl Config {
    pub fn parse(text: &str) -> Config {
        let mut c = Config::default();
        let mut notes = String::new();
        let mut section: Option<String> = None;
        for raw in text.split('\n') {
            let line = trim_line_end(raw);
            if line.trim_start_matches(' ').is_empty() {
                notes.push('\n');
                continue;
            }
            let line = trim_ws(line);
            if line.starts_with('[') && line.ends_with(']') {
                let name = line.trim_start_matches('[').trim_end_matches(']').to_string();
                if c.section(&name).is_none() {
                    c.sections.push(Section { name: name.clone(), comment: std::mem::take(&mut notes), options: Vec::new() });
                }
                notes.clear();
                section = Some(name);
                continue;
            }
            let Some(sec) = &section else { continue };
            if line.len() == 1 {
                continue;
            }
            let Some(p) = line.find(':').filter(|&p| p > 0) else {
                // The firmware refuses the whole file here; ignore the line instead.
                continue;
            };
            let name = trim_ws(&line[..p]).to_string();
            let value = trim_ws(&line[p + 1..]).to_string();
            let s = c.section_mut(sec).unwrap();
            if !s.options.iter().any(|o| o.name == name) {
                s.options.push(Opt { name, value, comment: std::mem::take(&mut notes) });
            }
            notes.clear();
        }
        c
    }

    pub fn read(path: &Path) -> std::io::Result<Config> {
        let mut c = Config::parse(&std::fs::read_to_string(path)?);
        c.path = Some(path.to_path_buf());
        Ok(c)
    }

    pub fn section(&self, name: &str) -> Option<&Section> {
        self.sections.iter().find(|s| s.name == name)
    }

    fn section_mut(&mut self, name: &str) -> Option<&mut Section> {
        self.sections.iter_mut().find(|s| s.name == name)
    }

    /// Section names in file order.
    pub fn section_names(&self) -> Vec<String> {
        self.sections.iter().map(|s| s.name.clone()).collect()
    }

    pub fn has_section(&self, name: &str) -> bool {
        self.section(name).is_some()
    }

    pub fn get(&self, section: &str, key: &str) -> Option<&str> {
        self.section(section)?.options.iter().find(|o| o.name == key).map(|o| o.value.as_str())
    }

    pub fn get_string(&self, section: &str, key: &str, def: &str) -> String {
        self.get(section, key).map_or_else(|| def.to_string(), str::to_string)
    }

    pub fn get_double(&self, section: &str, key: &str, def: f64) -> f64 {
        match self.get(section, key) {
            Some(v) if !v.is_empty() => parse_double(v),
            _ => def,
        }
    }

    pub fn get_int(&self, section: &str, key: &str, def: i64) -> i64 {
        match self.get(section, key) {
            Some(v) if !v.is_empty() => parse_int(v),
            _ => def,
        }
    }

    /// `GetInt(…, minval, maxval)`: an out-of-range value is reported and
    /// clamped.
    pub fn get_int_clamped(&self, section: &str, key: &str, def: i64, min: i64, max: i64) -> i64 {
        let v = self.get_int(section, key, def);
        if v < min {
            crate::gcode::console(&format!("error on {section}: {key} must have minimum of {min}"));
            min
        } else if v > max {
            crate::gcode::console(&format!("error on {section}: {key} must have maximum of {max}"));
            max
        } else {
            v
        }
    }

    /// `GetDouble(…, minval)`: an out-of-range value is only reported.
    pub fn get_double_min(&self, section: &str, key: &str, def: f64, min: f64) -> f64 {
        let v = self.get_double(section, key, def);
        if v < min {
            crate::gcode::console(&format!("error on {section}: {key} must have minimum of {}", crate::gcode::parse::cpp_double(min)));
        }
        v
    }

    /// `GetDouble(…, minval, maxval)`: an out-of-range value is only reported.
    pub fn get_double_range(&self, section: &str, key: &str, def: f64, min: f64, max: f64) -> f64 {
        let v = self.get_double_min(section, key, def, min);
        if v > max {
            crate::gcode::console(&format!("error on {section}: {key} must have maximum of {}", crate::gcode::parse::cpp_double(max)));
        }
        v
    }

    pub fn get_bool(&self, section: &str, key: &str, def: bool) -> bool {
        match self.get(section, key) {
            Some(v) if !v.is_empty() => matches!(v, "1" | "true" | "TRUE" | "True"),
            _ => def,
        }
    }

    /// Comma-separated numbers (`atof` each).
    pub fn get_doubles(&self, section: &str, key: &str) -> Vec<f64> {
        self.get(section, key).map_or_else(Vec::new, |v| v.trim().split(',').map(|t| parse_double(t.trim())).collect())
    }

    pub fn set_value(&mut self, section: &str, key: &str, value: &str) {
        if self.section(section).is_none() {
            self.sections.push(Section { name: section.into(), ..Default::default() });
        }
        let s = self.section_mut(section).unwrap();
        match s.options.iter_mut().find(|o| o.name == key) {
            Some(o) => o.value = value.to_string(),
            None => s.options.push(Opt { name: key.into(), value: value.into(), comment: String::new() }),
        }
    }

    pub fn set_double(&mut self, section: &str, key: &str, v: f64) {
        self.set_value(section, key, &to_string(v));
    }

    pub fn set_int(&mut self, section: &str, key: &str, v: i64) {
        self.set_value(section, key, &v.to_string());
    }

    /// Overlay `user_printer.cfg` (existing keys replaced, new sections added;
    /// new keys in existing sections are ignored, as in `Setuservalue`).
    pub fn overlay(&mut self, user: &Config) {
        for us in &user.sections {
            match self.section_mut(&us.name) {
                Some(s) => {
                    for uo in &us.options {
                        if let Some(o) = s.options.iter_mut().find(|o| o.name == uo.name) {
                            o.value = uo.value.clone();
                        }
                    }
                }
                None => self.sections.push(us.clone()),
            }
        }
    }

    /// Whole file, like `WriteIni`.
    pub fn to_text(&self) -> String {
        let mut out = String::new();
        for s in &self.sections {
            out.push_str(&s.comment);
            out.push_str(&format!("[{}]\n", s.name));
            for o in &s.options {
                out.push_str(&o.comment);
                out.push_str(&format!("{} : {}\n", o.name, o.value));
            }
        }
        out
    }

    pub fn write(&self, path: &Path) -> std::io::Result<()> {
        std::fs::write(path, self.to_text())?;
        crate::gcode::console(&format!("write to  {}  finish ", path.display()));
        Ok(())
    }

    /// `WriteI_specified_Ini`: write `[root]` (only `keys`, or all if empty)
    /// at the start of `path`, then the old file content with any `[root]`
    /// block removed. Like the firmware, the file is not truncated first, so
    /// a shorter result leaves the tail of the old content in place.
    pub fn write_specified(&self, path: &Path, root: &str, keys: &[&str]) -> std::io::Result<()> {
        let old = std::fs::read(path).unwrap_or_default();
        let mut content = old.clone();
        let mut out = Vec::new();
        if let Some(s) = self.section(root) {
            let header = format!("[{}]", s.name);
            if let Some(pos) = find(&content, header.as_bytes()) {
                let end = find(&content[pos + 1..], b"[").map_or(content.len(), |e| pos + 1 + e);
                content.drain(pos..end);
            }
            out.extend_from_slice(s.comment.as_bytes());
            out.extend_from_slice(format!("{header}\n").as_bytes());
            for o in &s.options {
                if keys.is_empty() || keys.contains(&o.name.as_str()) {
                    out.extend_from_slice(o.comment.as_bytes());
                    out.extend_from_slice(format!("{} : {}\n", o.name, o.value).as_bytes());
                }
            }
        }
        out.extend_from_slice(&content);
        if out.len() < old.len() {
            // fstream in|out without truncation: old bytes past our end remain
            out.extend_from_slice(&old[out.len()..]);
        }
        std::fs::write(path, out)?;
        crate::gcode::console(&format!("write to  {}  finish ", path.display()));
        Ok(())
    }
}

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// The three configuration files and where they live.
#[derive(Debug, Clone)]
pub struct Configs {
    pub printer: Config,
    pub unmodifiable: Config,
    pub sysconf: Config,
    pub dir: Option<PathBuf>,
}

impl Configs {
    /// Stock files only (nothing on disk).
    pub fn stock() -> Configs {
        Configs {
            printer: Config::parse(STOCK_PRINTER_CFG),
            unmodifiable: Config::parse(STOCK_UNMODIFIABLE_CFG),
            sysconf: Config::parse(STOCK_SYSCONF_CFG),
            dir: None,
        }
    }

    /// Load from a `/board-resource`-style folder, creating missing files
    /// from the stock copies (as `configfile.cpp` does).
    pub fn load(dir: &Path) -> Configs {
        let _ = std::fs::create_dir_all(dir);
        let load = |name: &str, stock: &str| -> Config {
            let p = dir.join(name);
            if !p.exists() {
                let _ = std::fs::write(&p, stock);
            }
            let mut c = Config::read(&p).unwrap_or_else(|_| Config::parse(stock));
            c.path = Some(p);
            c
        };
        let mut printer = load("printer.cfg", STOCK_PRINTER_CFG);
        if let Ok(user) = Config::read(&dir.join("user_printer.cfg")) {
            printer.overlay(&user);
        }
        Configs {
            printer,
            unmodifiable: load("unmodifiable.cfg", STOCK_UNMODIFIABLE_CFG),
            sysconf: load("sysconf.cfg", STOCK_SYSCONF_CFG),
            dir: Some(dir.to_path_buf()),
        }
    }

    pub fn path(&self, name: &str) -> Option<PathBuf> {
        self.dir.as_ref().map(|d| d.join(name))
    }

    /// `WriteIni(CONFIG_PATH)`.
    pub fn save_printer(&self) {
        if let Some(p) = self.path("printer.cfg") {
            if let Err(e) = self.printer.write(&p) {
                crate::gcode::console(&format!("Cannot open this file: {}: {e}", p.display()));
            }
        }
    }

    /// `WriteI_specified_Ini(USER_CONFIG_PATH, root, keys)`.
    pub fn save_user(&self, root: &str, keys: &[&str]) {
        if let Some(p) = self.path("user_printer.cfg") {
            let _ = self.printer.write_specified(&p, root, keys);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stock_config_reads_like_the_firmware() {
        let c = Config::parse(STOCK_PRINTER_CFG);
        assert_eq!(c.get_string("printer", "kinematics", ""), "corexy");
        assert_eq!(c.get_double("stepper_x", "position_endstop", 0.0), -2.0);
        assert_eq!(c.get_double("stepper_z", "position_endstop", 0.0), 4.5);
        assert_eq!(c.get_double("extruder", "pid_Kp", 0.0), 24.860427);
        assert_eq!(c.get_string("fan model", "pin", ""), "stm32:PB5");
        // duplicate [gcode_arcs]: first value wins
        assert_eq!(c.get_double("gcode_arcs", "resolution", 1.0), 0.1);
        assert_eq!(c.get_double("safe_z_home", "temp", 0.0), 140.0);
    }

    #[test]
    fn overlay_and_write_roundtrip() {
        let mut c = Config::parse("[a]\nx : 1\ny: 2 ; note\n\n[b]\nz : 3\n");
        assert_eq!(c.get_int("a", "y", 0), 2);
        c.overlay(&Config::parse("[a]\nx : 5\nnew : 9\n[c]\nw : 1\n"));
        assert_eq!(c.get_int("a", "x", 0), 5);
        assert_eq!(c.get("a", "new"), None, "new keys of existing sections are ignored");
        assert_eq!(c.get_int("c", "w", 0), 1);
        c.set_double("b", "z", 0.5);
        let back = Config::parse(&c.to_text());
        assert_eq!(back.get_string("b", "z", ""), "0.500000");
    }

    #[test]
    fn write_specified_replaces_one_section() {
        let dir = std::env::temp_dir().join(format!("ccos-cfg-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("user_printer.cfg");
        std::fs::write(&p, "[probe]\nz_offset : 1.000000\n[other]\nk : v\n").unwrap();
        let mut c = Config::parse("[probe]\nz_offset : 2\nspeed : 5\n");
        c.set_double("probe", "z_offset", -0.25);
        c.write_specified(&p, "probe", &["z_offset"]).unwrap();
        let back = Config::read(&p).unwrap();
        assert_eq!(back.get_double("probe", "z_offset", 0.0), -0.25);
        assert_eq!(back.get("probe", "speed"), None);
        assert_eq!(back.get("other", "k"), Some("v"));
        let _ = std::fs::remove_dir_all(dir);
    }
}
