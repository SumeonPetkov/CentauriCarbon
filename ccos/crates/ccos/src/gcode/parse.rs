//! G-code line parsing, ported from Elegoo's `GCodeDispatch::process_command`
//! and `GCodeCommand` (firmware/core/klippy/gcode.cpp) so that every line is
//! understood exactly as the stock firmware understands it — quirks
//! included:
//!
//! * only ASCII spaces are stripped (a tab-indented `G1` is an extended
//!   command with no axes);
//! * "traditional" lines (letter + digit) are upper-cased entirely and every
//!   letter, `_`, `*` or `/` becomes its own key — comments are *not*
//!   removed here (the file reader cuts at `;` before dispatch);
//! * extended commands keep their case, a bare token is its own value
//!   (`RESET` → `RESET=RESET`), and a quoted value only parses cleanly when
//!   it is the last parameter;
//! * numbers are read like C++ `std::stod`/`std::stoi` (leading whitespace,
//!   longest numeric prefix, error when there is none);
//! * out-of-range values print `error on <line>: <NAME>must ...` and fall
//!   back to the default instead of failing the command.

use std::collections::BTreeMap;

/// A command could not be executed (the stock firmware catches the C++
/// exception, logs "error" and moves on to the next line).
#[derive(Debug, Clone, PartialEq)]
pub struct Abort(pub String);

impl std::fmt::Display for Abort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Abort {}

impl From<String> for Abort {
    fn from(s: String) -> Self {
        Abort(s)
    }
}

impl From<&str> for Abort {
    fn from(s: &str) -> Self {
        Abort(s.to_string())
    }
}

impl From<Box<dyn std::error::Error + Send + Sync>> for Abort {
    fn from(e: Box<dyn std::error::Error + Send + Sync>) -> Self {
        Abort(e.to_string())
    }
}

pub type CmdResult = Result<(), Abort>;

/// One parsed line.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GCmd {
    /// Command name as parsed (may still contain `\r` etc.; see [`key`](Self::key)).
    pub command: String,
    /// The stripped line (upper-cased for traditional commands).
    pub commandline: String,
    pub params: BTreeMap<String, String>,
}

/// `strip()` from helper/my_string.h: spaces only.
pub fn strip(s: &str) -> &str {
    s.trim_matches(' ')
}

fn is_traditional(line: &str) -> bool {
    let b = line.as_bytes();
    b.len() >= 2 && b[0].to_ascii_uppercase().is_ascii_uppercase() && b[1].is_ascii_digit()
}

/// Parse one line. `None` for an empty line (skipped, like the firmware).
pub fn parse(raw: &str) -> Option<GCmd> {
    let line = strip(raw);
    if line.is_empty() {
        return None;
    }
    let mut params = BTreeMap::new();
    let mut cmd = String::new();
    let commandline;
    if !is_traditional(line) {
        commandline = line.to_string();
        let mut parts: Vec<String> = Vec::new();
        let mut in_quotes = false;
        let mut cur = String::new();
        for ch in line.chars() {
            if ch == '"' {
                in_quotes = !in_quotes;
                cur.push(ch);
            } else if ch == ' ' && !in_quotes {
                if !cur.is_empty() {
                    parts.push(std::mem::take(&mut cur));
                }
            } else {
                cur.push(ch);
            }
        }
        if !cur.is_empty() {
            parts.push(cur);
        }
        cmd = parts[0].clone();
        let mut i = 1;
        while i < parts.len() {
            let p = &parts[i];
            if p.starts_with(';') || p.starts_with('#') || p.starts_with('*') {
                break;
            }
            match p.find('=') {
                // `=` first: the C++ reads an empty vector (undefined
                // behaviour); treat the token as absent.
                Some(0) => {}
                found => {
                    let (key, mut value) = match found {
                        Some(pos) => (p[..pos].to_string(), p[pos + 1..].to_string()),
                        // no '=': npos + 1 wraps to 0, so the value is the token
                        None => (p.clone(), p.clone()),
                    };
                    if value.starts_with('"') {
                        let mut j = i + 1;
                        while j < parts.len() {
                            value.push(' ');
                            value.push_str(&parts[j]);
                            if parts[j].ends_with('"') {
                                break;
                            }
                            j += 1;
                        }
                        i = j;
                        // drop the first and last character
                        let mut chars: Vec<char> = value.chars().collect();
                        if !chars.is_empty() {
                            chars.remove(0);
                        }
                        chars.pop();
                        value = chars.into_iter().collect();
                    }
                    params.insert(key, value);
                }
            }
            i += 1;
        }
    } else {
        let up = line.to_ascii_uppercase();
        let mut parts: Vec<String> = Vec::new();
        let mut temp = String::new();
        for c in up.chars() {
            if c.is_ascii_uppercase() || c == '_' || c == '*' || c == '/' {
                if !temp.is_empty() {
                    parts.push(std::mem::take(&mut temp));
                }
                parts.push(c.to_string());
            } else {
                temp.push(c);
            }
        }
        if !temp.is_empty() {
            parts.push(temp);
        }
        let n = parts.len();
        if n >= 2 && parts[0] != "N" {
            cmd = format!("{}{}", parts[0], strip(&parts[1]));
        } else if n >= 4 && parts[0] == "N" {
            cmd = format!("{}{}", parts[2], strip(&parts[3]));
        }
        let mut i = 0;
        while i < n {
            let v = if i + 1 >= n { " ".to_string() } else { parts[i + 1].clone() };
            params.insert(parts[i].clone(), v);
            i += 2;
        }
        commandline = up;
    }
    Some(GCmd { command: cmd, commandline, params })
}

impl GCmd {
    /// Name used to look the handler up: whitespace and NULs removed.
    pub fn key(&self) -> String {
        self.command.chars().filter(|c| !c.is_whitespace() && *c != '\0').collect()
    }

    pub fn has(&self, name: &str) -> bool {
        self.params.contains_key(name)
    }

    pub fn get_string(&self, name: &str, default: &str) -> String {
        self.params.get(name).cloned().unwrap_or_else(|| default.to_string())
    }

    fn range_error(&self, name: &str, what: &str, bound: f64) {
        crate::gcode::console(&format!("error on {}: {name}must {what} {}", self.commandline, cpp_double(bound)));
    }

    /// `get_int(name, default, min, max)`; `None` = no check.
    pub fn get_int(&self, name: &str, default: i64, min: Option<i64>, max: Option<i64>) -> Result<i64, Abort> {
        let Some(v) = self.params.get(name) else { return Ok(default) };
        let value = stoi(v).map_err(|e| Abort(format!("{}: {name}: {e}", self.commandline)))?;
        if let Some(m) = min {
            if value < m {
                self.range_error(name, "have minimum of", m as f64);
                return Ok(default);
            }
        }
        if let Some(m) = max {
            if value > m {
                self.range_error(name, "have maximum of", m as f64);
                return Ok(default);
            }
        }
        Ok(value)
    }

    /// `get_double(name, default, minval, maxval, above, below)`.
    pub fn get_double(&self, name: &str, default: f64, lim: Lim) -> Result<f64, Abort> {
        let Some(v) = self.params.get(name) else { return Ok(default) };
        let value = stod(v).map_err(|e| Abort(format!("{}: {name}: {e}", self.commandline)))?;
        if let Some(m) = lim.min {
            if value < m {
                self.range_error(name, "have minimum of", m);
                return Ok(default);
            }
        }
        if let Some(m) = lim.max {
            if value > m {
                self.range_error(name, "have maximum of", m);
                return Ok(default);
            }
        }
        if let Some(m) = lim.above {
            if value <= m {
                self.range_error(name, "be above", m);
                return Ok(default);
            }
        }
        if let Some(m) = lim.below {
            if value >= m {
                self.range_error(name, "be below", m);
                return Ok(default);
            }
        }
        Ok(value)
    }

    /// Shorthand: `get_double(name, default)` without checks.
    pub fn f(&self, name: &str, default: f64) -> Result<f64, Abort> {
        self.get_double(name, default, Lim::NONE)
    }

    pub fn i(&self, name: &str, default: i64) -> Result<i64, Abort> {
        self.get_int(name, default, None, None)
    }
}

/// Validation bounds for [`GCmd::get_double`].
#[derive(Debug, Clone, Copy, Default)]
pub struct Lim {
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub above: Option<f64>,
    pub below: Option<f64>,
}

impl Lim {
    pub const NONE: Lim = Lim { min: None, max: None, above: None, below: None };
    pub fn min(m: f64) -> Lim {
        Lim { min: Some(m), ..Lim::NONE }
    }
    pub fn above(a: f64) -> Lim {
        Lim { above: Some(a), ..Lim::NONE }
    }
    pub fn min_max(a: f64, b: f64) -> Lim {
        Lim { min: Some(a), max: Some(b), ..Lim::NONE }
    }
}

/// `std::stod`: skip leading whitespace, take the longest numeric prefix
/// (`strtod` rules: sign, digits, `.`, exponent, `inf`, `nan`, hex).
pub fn stod(s: &str) -> Result<f64, &'static str> {
    let t = s.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    let b = t.as_bytes();
    let mut i = 0;
    let neg = match b.first() {
        Some(b'-') => {
            i = 1;
            true
        }
        Some(b'+') => {
            i = 1;
            false
        }
        _ => false,
    };
    let rest = &t[i..];
    let lower = rest.to_ascii_lowercase();
    if lower.starts_with("inf") {
        return Ok(if neg { f64::NEG_INFINITY } else { f64::INFINITY });
    }
    if lower.starts_with("nan") {
        return Ok(f64::NAN);
    }
    if lower.starts_with("0x") {
        let h: String = rest[2..].chars().take_while(|c| c.is_ascii_hexdigit()).collect();
        if h.is_empty() {
            return Ok(if neg { -0.0 } else { 0.0 });
        }
        let v = u64::from_str_radix(&h[..h.len().min(15)], 16).unwrap_or(0) as f64;
        return Ok(if neg { -v } else { v });
    }
    let rb = rest.as_bytes();
    let mut j = 0;
    let mut digits = 0;
    while j < rb.len() && rb[j].is_ascii_digit() {
        j += 1;
        digits += 1;
    }
    if j < rb.len() && rb[j] == b'.' {
        j += 1;
        while j < rb.len() && rb[j].is_ascii_digit() {
            j += 1;
            digits += 1;
        }
    }
    if digits == 0 {
        return Err("stod: no conversion");
    }
    if j < rb.len() && (rb[j] == b'e' || rb[j] == b'E') {
        let mut k = j + 1;
        if k < rb.len() && (rb[k] == b'+' || rb[k] == b'-') {
            k += 1;
        }
        let start = k;
        while k < rb.len() && rb[k].is_ascii_digit() {
            k += 1;
        }
        if k > start {
            j = k;
        }
    }
    let v: f64 = rest[..j].parse().map_err(|_| "stod: no conversion")?;
    if v.is_infinite() {
        return Err("stod: out of range");
    }
    Ok(if neg { -v } else { v })
}

/// `std::stoi` (base 10, range of a 32-bit int).
pub fn stoi(s: &str) -> Result<i64, &'static str> {
    let t = s.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    let b = t.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'-' || b[i] == b'+') {
        i += 1;
    }
    let start = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    if i == start {
        return Err("stoi: no conversion");
    }
    let v: i64 = t[..i].parse().map_err(|_| "stoi: out of range")?;
    if v < i32::MIN as i64 || v > i32::MAX as i64 {
        return Err("stoi: out of range");
    }
    Ok(v)
}

/// `std::ostream << double` with the default precision (`%g`, 6 digits).
pub fn cpp_double(x: f64) -> String {
    if x.is_nan() {
        return "nan".into();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf".into() } else { "-inf".into() };
    }
    if x == 0.0 {
        return "0".into();
    }
    let e = format!("{:.5e}", x);
    let (mant, exp) = e.split_once('e').unwrap();
    let exp: i32 = exp.parse().unwrap();
    if !(-5..6).contains(&exp) {
        let m = mant.trim_end_matches('0').trim_end_matches('.');
        format!("{m}e{}{:02}", if exp < 0 { '-' } else { '+' }, exp.abs())
    } else {
        let decimals = (5 - exp).max(0) as usize;
        let s = format!("{:.*}", decimals, x);
        if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            s
        }
    }
}

/// `std::to_string(double)`: `%f`, six decimals.
pub fn cpp_to_string(x: f64) -> String {
    format!("{x:.6}")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(line: &str) -> GCmd {
        parse(line).unwrap()
    }

    fn params(g: &GCmd) -> Vec<(String, String)> {
        g.params.iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }

    #[test]
    fn traditional_examples_from_the_firmware() {
        let g = p("G1 X10 Y-2.5 E.4 F3000");
        assert_eq!(g.key(), "G1");
        assert_eq!(g.params["X"], "10 ");
        assert_eq!(g.params["Y"], "-2.5 ");
        assert_eq!(g.params["E"], ".4 ");
        assert_eq!(g.params["F"], "3000");
        assert_eq!(g.params["G"], "1 ");
        let g = p("G28 X Y");
        assert_eq!((g.params["X"].as_str(), g.params["Y"].as_str()), (" ", " "));
        let g = p("g1x1e-5");
        assert_eq!((g.key().as_str(), g.params["X"].as_str(), g.params["E"].as_str()), ("G1", "1", "-5"));
        assert_eq!(p("G29.1").key(), "G29.1");
        let g = p("N5 G1 X1*71");
        assert_eq!(g.key(), "G1");
        assert_eq!(g.params["*"], "71");
        // comments are not stripped by the dispatcher: S is overwritten
        let g = p("M104 S200 ; set temp");
        assert_eq!(g.params["S"], "E");
        assert!(g.f("S", 0.0).is_err());
        // whole line upper-cased
        assert_eq!(p("M117 hello").commandline, "M117 HELLO");
        // a CRLF file line still resolves
        assert_eq!(p("G28\r").key(), "G28");
    }

    #[test]
    fn extended_examples_from_the_firmware() {
        let g = p("SET_MIN_EXTRUDE_TEMP RESET");
        assert_eq!(g.params["RESET"], "RESET");
        let g = p("SDCARD_PRINT_FILE FILESLICE=4 FILENAME=\"/a b.gcode\"");
        assert_eq!(g.params["FILENAME"], "/a b.gcode");
        let g = p("X A=\"x y\" B=1");
        assert_eq!(g.params["A"], "x y\" B=");
        let g = p("set_velocity_limit velocity=1 ; comment VELOCITY=2");
        assert_eq!(g.key(), "set_velocity_limit");
        assert_eq!(params(&g), vec![("velocity".to_string(), "1".to_string())]);
        // leading tab: extended, the axis becomes a bare key
        let g = p("\tG1 X10");
        assert_eq!(g.key(), "G1");
        assert_eq!(params(&g), vec![("X10".to_string(), "X10".to_string())]);
        assert!(parse("   ").is_none());
    }

    #[test]
    fn numbers_like_cpp() {
        assert_eq!(stod(" 10 ").unwrap(), 10.0);
        assert_eq!(stod("1.5e3x").unwrap(), 1500.0);
        assert_eq!(stod("-.5").unwrap(), -0.5);
        assert_eq!(stod("1e").unwrap(), 1.0);
        assert!(stod(" ").is_err());
        assert!(stod("E").is_err());
        assert_eq!(stoi("1.9").unwrap(), 1);
        assert!(stoi("x").is_err());
        assert_eq!(cpp_double(5000.0), "5000");
        assert_eq!(cpp_double(0.001), "0.001");
        assert_eq!(cpp_double(1e-7), "1e-07");
        assert_eq!(cpp_double(1234567.0), "1.23457e+06");
        assert_eq!(cpp_double(0.2), "0.2");
        assert_eq!(cpp_to_string(202.0), "202.000000");
    }

    #[test]
    fn out_of_range_falls_back_to_default() {
        let g = p("M220 S0");
        assert_eq!(g.get_double("S", 100.0, Lim::above(0.0)).unwrap(), 100.0);
        let g = p("G4 P-5");
        assert_eq!(g.get_double("P", 0.0, Lim::min(0.0)).unwrap(), 0.0);
    }
}
