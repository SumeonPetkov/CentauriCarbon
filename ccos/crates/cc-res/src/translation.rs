//! `translation.bin`: every UI string, per language.
//!
//! ```text
//! | header (14) | font sizes: u8 x font_len | lang offsets: u32 x lang_len |
//! | entry (8) x tr_len, for each language | NUL-terminated UTF-8 strings |
//! entry = { offset: u32, len: u16 (incl. NUL), weight: u8, weight_index: u8 }
//! ```
//! String id `n` in the UI (`tr(n)`, widget custom prop `tr_index`) is row
//! `n` of the CSV (1-based). Column order is the language index used by
//! `sysconf.cfg [system] language`.
//!
//! The per-string font weight column is `\t` (unparseable) in the shipped
//! CSV, so the stock tool writes uninitialised stack bytes there; it is
//! unused in practice — fonts come from the widget `font_weight` style.

use serde::{Deserialize, Serialize};

use crate::csvutil::{fgets_lines, scan_dec, strtok_tabs};
use crate::{fmt_err, Error, Rd};

pub const MAGIC: u32 = 0x12fd_1000;
const HEADER: usize = 14;

pub const LANGUAGES: [&str; 11] =
    ["中文", "English", "Español", "Français", "Italiano", "Русский", "Deutsch", "日本語", "한국어", "Türkçe", "Українська"];

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Entry {
    pub text: String,
    /// No string at all (len 0, no NUL): the CSV row had no cell for this
    /// language. The stock loader treats it as an empty string.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub missing: bool,
    #[serde(default)]
    pub weight: u8,
    #[serde(default)]
    pub weight_index: u8,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct TranslationDb {
    pub crc: u16,
    pub fonts: Vec<u8>,
    /// `langs[lang][id - 1]`
    pub langs: Vec<Vec<Entry>>,
}

impl TranslationDb {
    pub fn get(&self, lang: usize, id: u16) -> Option<&str> {
        self.langs.get(lang)?.get((id as usize).checked_sub(1)?).map(|e| e.text.as_str())
    }

    pub fn parse_bin(b: &[u8]) -> Result<Self, Error> {
        let mut r = Rd::at(b, 0);
        if r.u32()? != MAGIC {
            return fmt_err("translation.bin: bad magic");
        }
        let _size = r.u32()?;
        let crc = r.u16()?;
        let font_len = r.u8()? as usize;
        let lang_len = r.u8()? as usize;
        let tr_len = r.u16()? as usize;
        let fonts = r.bytes(font_len)?.to_vec();
        let offs: Vec<u32> = (0..lang_len).map(|_| r.u32()).collect::<Result<_, _>>()?;
        let mut langs = Vec::with_capacity(lang_len);
        for off in offs {
            let mut h = Rd::at(b, off as usize);
            let mut v = Vec::with_capacity(tr_len);
            for _ in 0..tr_len {
                let so = h.u32()? as usize;
                let len = h.u16()? as usize;
                let weight = h.u8()?;
                let weight_index = h.u8()?;
                let raw = Rd::at(b, so).bytes(len)?;
                let raw = raw.strip_suffix(&[0]).unwrap_or(raw);
                let text = String::from_utf8_lossy(raw).into_owned();
                v.push(Entry { text, missing: len == 0, weight, weight_index });
            }
            langs.push(v);
        }
        Ok(Self { crc, fonts, langs })
    }

    /// Serialise exactly like `firmware/tools/tr`.
    pub fn to_bin(&self) -> Vec<u8> {
        let lang_len = self.langs.len();
        let tr_len = self.langs.first().map_or(0, Vec::len);
        let headers_at = HEADER + self.fonts.len() + 4 * lang_len;
        let data_at = headers_at + 8 * lang_len * tr_len;
        let mut headers = Vec::with_capacity(data_at - headers_at);
        let mut data = Vec::new();
        for lang in &self.langs {
            for e in lang {
                let off = data_at + data.len();
                let len = if e.missing { 0 } else { e.text.len() + 1 };
                if !e.missing {
                    data.extend_from_slice(e.text.as_bytes());
                    data.push(0);
                }
                headers.extend_from_slice(&(off as u32).to_le_bytes());
                headers.extend_from_slice(&(len as u16).to_le_bytes());
                headers.push(e.weight);
                headers.push(e.weight_index);
            }
        }
        let total = data_at + data.len();
        let mut out = Vec::with_capacity(total);
        out.extend_from_slice(&MAGIC.to_le_bytes());
        out.extend_from_slice(&((total - HEADER) as u32).to_le_bytes());
        out.extend_from_slice(&self.crc.to_le_bytes());
        out.push(self.fonts.len() as u8);
        out.push(lang_len as u8);
        out.extend_from_slice(&(tr_len as u16).to_le_bytes());
        out.extend_from_slice(&self.fonts);
        for i in 0..lang_len {
            out.extend_from_slice(&((headers_at + i * 8 * tr_len) as u32).to_le_bytes());
        }
        out.extend_from_slice(&headers);
        out.extend_from_slice(&data);
        out
    }

    /// Parse Elegoo's tab-separated `translation.csv` like the C tool
    /// (unparseable weights become 0 instead of stack garbage).
    pub fn parse_csv(text: &str) -> Result<Self, Error> {
        let mut lines = fgets_lines(text);
        let header = lines.next().ok_or_else(|| Error::Format("empty csv".into()))?;
        let lang_len = strtok_tabs(header).len().saturating_sub(2);
        let rows: Vec<&str> = lines.collect();
        let tr_len = rows.len();
        let mut langs = vec![vec![Entry { text: String::new(), missing: true, weight: 0, weight_index: 0 }; tr_len]; lang_len];
        let mut fonts: Vec<u8> = Vec::new();
        for (n, row) in rows.iter().enumerate() {
            let toks = strtok_tabs(row);
            let Some(id) = toks.first().and_then(|t| scan_dec(t)) else { continue };
            let idx = (id.0 - 1) as usize;
            if idx >= tr_len {
                return fmt_err(format!("row {}: id {} outside 1..={tr_len}", n + 2, id.0));
            }
            let mut weights: Vec<u8> =
                toks.get(1).map(|t| t.split('-').map(|w| scan_dec(w).map_or(0, |v| v.0 as u8)).collect()).unwrap_or_default();
            if weights.is_empty() {
                weights.push(0);
            }
            for &w in &weights {
                if !fonts.contains(&w) {
                    fonts.push(w);
                }
            }
            while weights.len() < lang_len {
                weights.push(*weights.last().unwrap());
            }
            for (col, t) in toks.iter().skip(2).enumerate().take(lang_len) {
                langs[col][idx] = Entry { text: t.replace("\\n", "\n"), missing: false, weight: weights[col], weight_index: 0 };
            }
        }
        fonts.sort_unstable();
        for lang in &mut langs {
            for e in lang.iter_mut() {
                e.weight_index = fonts.iter().position(|&f| f == e.weight).map_or(0xff, |p| p as u8);
            }
        }
        Ok(Self { crc: 0, fonts, langs })
    }

    pub fn to_csv(&self) -> String {
        let mut out = String::from("字符串序号\t字号大小");
        for (i, _) in self.langs.iter().enumerate() {
            out.push('\t');
            out += LANGUAGES.get(i).copied().unwrap_or("结束列");
        }
        out.push('\n');
        let tr_len = self.langs.first().map_or(0, Vec::len);
        for id in 0..tr_len {
            let weights: Vec<String> = self.langs.iter().map(|l| l[id].weight.to_string()).collect();
            let w = if weights.iter().all(|w| w == &weights[0]) { weights[0].clone() } else { weights.join("-") };
            out += &format!("{}\t{}", id + 1, w);
            for l in &self.langs {
                // Strings containing a real newline become the "\n" escape;
                // a trailing line terminator (from the last CSV column) is dropped.
                let t = l[id].text.trim_end_matches(['\r', '\n']).replace('\n', "\\n");
                if l[id].missing {
                    break; // trailing cells absent, as in the source CSV
                }
                out.push('\t');
                out += if t.is_empty() { "-" } else { &t };
            }
            out.push('\n');
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn res(p: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../firmware/resources/e100").join(p)
    }

    #[test]
    fn shipped_bin_roundtrips() {
        let bin = std::fs::read(res("translation.bin")).unwrap();
        let db = TranslationDb::parse_bin(&bin).unwrap();
        assert_eq!(db.langs.len(), 12); // 11 languages + the "end" column
        assert_eq!(db.langs[0].len(), 466);
        assert_eq!(db.get(1, 1), Some("Prepare"));
        assert_eq!(db.get(1, 2), Some("Fan"));
        assert_eq!(db.to_bin(), bin);
    }

    #[test]
    fn shipped_csv_matches_shipped_text() {
        // The shipped .bin was built from a CRLF copy of the CSV; compare text
        // modulo line endings.
        let csv = std::fs::read_to_string(res("translation.csv")).unwrap();
        let ours = TranslationDb::parse_csv(&csv).unwrap();
        let shipped = TranslationDb::parse_bin(&std::fs::read(res("translation.bin")).unwrap()).unwrap();
        assert_eq!(ours.langs.len(), shipped.langs.len());
        for (a, b) in ours.langs.iter().zip(&shipped.langs) {
            for (x, y) in a.iter().zip(b) {
                assert_eq!(x.missing, y.missing);
                assert_eq!(x.text, y.text.replace("\r\n", "\n"));
            }
        }
    }

    #[test]
    fn csv_export_reimports() {
        let shipped = TranslationDb::parse_bin(&std::fs::read(res("translation.bin")).unwrap()).unwrap();
        let again = TranslationDb::parse_csv(&shipped.to_csv()).unwrap();
        for lang in 0..11 {
            for id in 0..466 {
                let e = &shipped.langs[lang][id];
                assert_eq!(again.langs[lang][id].missing, e.missing, "lang {lang} id {}", id + 1);
                let a = e.text.trim_end_matches(['\r', '\n']);
                let a = if a.is_empty() && !e.missing { "-" } else { a };
                assert_eq!(again.langs[lang][id].text.trim_end_matches('\n'), a, "lang {lang} id {}", id + 1);
            }
        }
    }
}
