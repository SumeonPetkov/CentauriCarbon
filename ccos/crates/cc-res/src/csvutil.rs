//! Helpers that reproduce the C `strtok`/`sscanf` behaviour of Elegoo's tools.

/// `strtok_r(line, "\t")`: split on tabs, dropping empty tokens.
pub fn strtok_tabs(line: &str) -> Vec<&str> {
    line.split('\t').filter(|t| !t.is_empty()).collect()
}

/// `fgets` semantics: lines keep their terminator, except the last one.
pub fn fgets_lines(text: &str) -> impl Iterator<Item = &str> {
    text.split_inclusive('\n')
}

/// `sscanf("%d")`-style signed decimal prefix (leading whitespace allowed).
pub fn scan_dec(s: &str) -> Option<(i64, &str)> {
    let s = s.trim_start();
    let neg = s.starts_with('-');
    let body = s.strip_prefix(['-', '+']).unwrap_or(s);
    let end = body.find(|c: char| !c.is_ascii_digit()).unwrap_or(body.len());
    if end == 0 {
        return None;
    }
    let v: i64 = body[..end].parse().ok()?;
    Some((if neg { -v } else { v }, &body[end..]))
}

/// `sscanf("%x")`-style hex prefix (optional `0x`).
pub fn scan_hex(s: &str) -> Option<(u32, &str)> {
    let s = s.trim_start();
    let body = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")).unwrap_or(s);
    let end = body.find(|c: char| !c.is_ascii_hexdigit()).unwrap_or(body.len());
    if end == 0 {
        return None;
    }
    // sscanf %x into unsigned int wraps on overflow; keep the low 32 bits.
    let v = u64::from_str_radix(&body[..end.min(16)], 16).ok()? as u32;
    Some((v, &body[end..]))
}
