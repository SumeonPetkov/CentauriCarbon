//! Compile `data/command_table.json` and `data/static_strings.json` into
//! static Rust tables, so the device binary carries no JSON parser and does
//! no work at startup.

use std::fmt::Write;
use std::path::Path;

/// Minimal parser for the flat `{"<int>": "<string>", ...}` files produced
/// by `tools/extract_tables.py` (no escapes occur in them).
fn parse_flat(text: &str) -> Vec<(u32, String)> {
    let mut out = Vec::new();
    let mut rest = text;
    while let Some(i) = rest.find('"') {
        rest = &rest[i + 1..];
        let j = rest.find('"').expect("unterminated key");
        let key = &rest[..j];
        rest = &rest[j + 1..];
        let k = rest.find('"').expect("missing value");
        rest = &rest[k + 1..];
        let mut v = String::new();
        let mut chars = rest.char_indices();
        let end = loop {
            let (p, c) = chars.next().expect("unterminated value");
            match c {
                '\\' => {
                    let (_, e) = chars.next().unwrap();
                    v.push(match e {
                        'n' => '\n',
                        't' => '\t',
                        other => other,
                    });
                }
                '"' => break p,
                c => v.push(c),
            }
        };
        rest = &rest[end + 1..];
        out.push((key.parse().expect("numeric key"), v));
    }
    out
}

fn ptype(spec: &str) -> &'static str {
    match spec {
        "%u" => "U32",
        "%i" => "I32",
        "%hu" => "U16",
        "%hi" => "I16",
        "%c" => "Byte",
        "%s" | "%*s" | "%.*s" => "Buffer",
        s => panic!("unknown parameter type {s}"),
    }
}

fn main() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("data");
    println!("cargo:rerun-if-changed=data/command_table.json");
    println!("cargo:rerun-if-changed=data/static_strings.json");
    let mut table = parse_flat(&std::fs::read_to_string(dir.join("command_table.json")).unwrap());
    table.sort_by_key(|(id, _)| *id);
    let strings = parse_flat(&std::fs::read_to_string(dir.join("static_strings.json")).unwrap());

    let mut code = String::new();
    writeln!(code, "/// The {} fixed message formats, indexed by id.", table.len()).unwrap();
    writeln!(code, "pub static CENTAURI_FORMATS: [MessageFormat; {}] = [", table.len()).unwrap();
    let mut max_params = 0;
    for (id, fmt) in &table {
        let mut parts = fmt.split_whitespace();
        let name = parts.next().unwrap();
        let params: Vec<(String, &str)> = parts
            .map(|p| {
                let (k, s) = p.split_once('=').expect("param without '='");
                (k.to_string(), ptype(s))
            })
            .collect();
        max_params = max_params.max(params.len());
        let plist: Vec<String> = params.iter().map(|(k, t)| format!("({k:?}, ParamType::{t})")).collect();
        writeln!(
            code,
            "    MessageFormat {{ id: {id}, name: {name:?}, params: &[{}], format: {:?} }},",
            plist.join(", "),
            fmt.trim()
        )
        .unwrap();
    }
    writeln!(code, "];").unwrap();
    assert!(max_params <= 12, "MAX_ARGS must be raised to {max_params}");

    let mut by_name: Vec<(String, u32)> = table.iter().map(|(id, f)| (f.split_whitespace().next().unwrap().to_string(), *id)).collect();
    by_name.sort();
    for w in by_name.windows(2) {
        assert_ne!(w[0].0, w[1].0, "duplicate message name");
    }
    writeln!(code, "/// Message name -> index into `CENTAURI_FORMATS`, sorted by name.").unwrap();
    writeln!(code, "pub static CENTAURI_BY_NAME: [(&str, u16); {}] = [", by_name.len()).unwrap();
    for (n, id) in &by_name {
        let idx = table.iter().position(|(i, _)| i == id).unwrap();
        writeln!(code, "    ({n:?}, {idx}),").unwrap();
    }
    writeln!(code, "];").unwrap();

    writeln!(code, "/// `static_string_id` -> text (shutdown reasons).").unwrap();
    writeln!(code, "pub static CENTAURI_STATIC_STRINGS: [(u16, &str); {}] = [", strings.len()).unwrap();
    let mut strings = strings;
    strings.sort_by_key(|(k, _)| *k);
    for (k, v) in &strings {
        writeln!(code, "    ({k}, {v:?}),").unwrap();
    }
    writeln!(code, "];").unwrap();

    let out = Path::new(&std::env::var("OUT_DIR").unwrap()).join("centauri_table.rs");
    std::fs::write(out, code).unwrap();
}
