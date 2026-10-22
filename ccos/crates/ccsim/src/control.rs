//! Emulator-only remote control over HTTP (`ccsim --control <addr>`): send
//! G-code, read ccos's console, upload/print/delete G-code files. The Docker
//! page (sim/docker/index.html) uses it; ccos itself has nothing like it.
//!
//!   GET  /console              the last 256 console lines
//!   POST /gcode                body: G-code lines, queued like the UI's
//!   GET  /files                "name\tsize" per file, newest first
//!   POST /upload?name=<file>   body: the file
//!   POST /print?name=<file>
//!   POST /delete?name=<file>
//!
//! Commands go to the main loop over a channel (the runtime lives there);
//! a print in progress holds them until it ends, as on the printer.

use std::io::{self, BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{channel, Receiver, Sender};

use ccos::gcode::Job;

/// Largest upload accepted.
const MAX_BODY: usize = 1 << 30;

/// Listen on `addr` ("6081" = 127.0.0.1:6081); jobs arrive on the receiver.
pub fn start(addr: &str, gcode_dir: PathBuf) -> io::Result<Receiver<Job>> {
    let addr = if addr.contains(':') { addr.to_string() } else { format!("127.0.0.1:{addr}") };
    let listener = TcpListener::bind(&addr)?;
    let (tx, rx) = channel();
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let (tx, dir) = (tx.clone(), gcode_dir.clone());
            std::thread::spawn(move || {
                let _ = serve(stream, &tx, &dir);
            });
        }
    });
    Ok(rx)
}

struct Reply(u16, String);

fn serve(stream: TcpStream, tx: &Sender<Job>, dir: &Path) -> io::Result<()> {
    let mut r = BufReader::new(stream.try_clone()?);
    let mut line = String::new();
    r.read_line(&mut line)?;
    let mut parts = line.split_whitespace();
    let (method, target) = (parts.next().unwrap_or("").to_string(), parts.next().unwrap_or("").to_string());
    let mut len = 0usize;
    loop {
        let mut h = String::new();
        if r.read_line(&mut h)? == 0 || h.trim().is_empty() {
            break;
        }
        if let Some((k, v)) = h.split_once(':') {
            if k.trim().eq_ignore_ascii_case("content-length") {
                len = v.trim().parse().unwrap_or(0);
            }
        }
    }
    let reply = if len > MAX_BODY {
        Reply(413, "too large\n".into())
    } else {
        let mut body = vec![0; len];
        r.read_exact(&mut body)?;
        let (path, query) = target.split_once('?').unwrap_or((&target, ""));
        let name = query.split('&').find_map(|kv| kv.strip_prefix("name=")).map(decode);
        handle(&method, path, name, body, tx, dir)
    };
    let Reply(code, text) = reply;
    let status = match code {
        200 => "OK",
        204 => "No Content",
        400 => "Bad Request",
        404 => "Not Found",
        413 => "Payload Too Large",
        _ => "Internal Server Error",
    };
    let mut w = stream;
    write!(
        w,
        "HTTP/1.1 {code} {status}\r\nContent-Type: text/plain; charset=utf-8\r\nContent-Length: {}\r\n\
         Access-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\n\
         Access-Control-Allow-Headers: Content-Type\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n{text}",
        text.len()
    )
}

fn handle(method: &str, path: &str, name: Option<String>, body: Vec<u8>, tx: &Sender<Job>, dir: &Path) -> Reply {
    let ok = || Reply(200, "ok\n".into());
    let send = |j: Job| match tx.send(j) {
        Ok(()) => ok(),
        Err(_) => Reply(500, "emulator stopped\n".into()),
    };
    if method == "OPTIONS" {
        return Reply(204, String::new());
    }
    match (method, path) {
        ("GET", "/console") => Reply(200, ccos::gcode::console_lines().join("\n")),
        ("GET", "/files") => {
            let mut s = String::new();
            for n in ccos::gcode::list_files(dir) {
                let size = std::fs::metadata(dir.join(&n)).map(|m| m.len()).unwrap_or(0);
                s.push_str(&format!("{n}\t{size}\n"));
            }
            Reply(200, s)
        }
        ("POST", "/gcode") => {
            let text = String::from_utf8_lossy(&body).replace("\r\n", "\n");
            if text.trim().is_empty() {
                return Reply(400, "no G-code\n".into());
            }
            // echo into the console so replies read in context
            for l in text.lines().filter(|l| !l.trim().is_empty()) {
                ccos::gcode::console(&format!("> {}", l.trim()));
            }
            send(Job::Gcode(text))
        }
        ("POST", "/upload" | "/print" | "/delete") => {
            let Some(n) = name.filter(|n| valid_name(n)) else {
                return Reply(400, "need name=<file>.gcode (no folders)\n".into());
            };
            let p = dir.join(&n);
            match path {
                "/upload" => match std::fs::write(&p, &body) {
                    Ok(()) => send(Job::RefreshFiles),
                    Err(e) => Reply(500, format!("{e}\n")),
                },
                "/print" if !p.is_file() => Reply(404, "no such file\n".into()),
                "/print" => send(Job::Print(p)),
                _ => match std::fs::remove_file(&p) {
                    Ok(()) => send(Job::RefreshFiles),
                    Err(e) => Reply(404, format!("{e}\n")),
                },
            }
        }
        _ => Reply(404, "not found\n".into()),
    }
}

/// A plain file name the Print screen lists.
fn valid_name(n: &str) -> bool {
    let lower = n.to_ascii_lowercase();
    !n.is_empty()
        && !n.starts_with('.')
        && !n.contains(['/', '\\', '\0'])
        && (lower.ends_with(".gcode") || lower.ends_with(".gco") || lower.ends_with(".g"))
}

/// Percent-decoding of a query value (`+` = space).
fn decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        let hex = |c: u8| (c as char).to_digit(16);
        match b[i] {
            b'%' if i + 2 < b.len() => match (hex(b[i + 1]), hex(b[i + 2])) {
                (Some(h), Some(l)) => {
                    out.push((h * 16 + l) as u8);
                    i += 3;
                    continue;
                }
                _ => out.push(b'%'),
            },
            b'+' => out.push(b' '),
            c => out.push(c),
        }
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names() {
        assert_eq!(decode("my%20cube+2.gcode"), "my cube 2.gcode");
        assert_eq!(decode("a%2"), "a%2");
        assert!(valid_name("cube.gcode") && valid_name("A.G"));
        assert!(!valid_name("../x.gcode") && !valid_name("x.txt") && !valid_name(".hidden.gcode") && !valid_name("d\\x.gcode"));
    }
}
