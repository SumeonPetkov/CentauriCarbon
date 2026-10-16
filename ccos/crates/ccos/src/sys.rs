//! Small Linux helpers (process stats, stock app detection). No allocation in
//! the periodic paths beyond a stack buffer.

use std::io::Read;

fn read_small(path: &str, buf: &mut [u8]) -> Option<usize> {
    std::fs::File::open(path).ok()?.read(buf).ok()
}

/// Resident memory of this process in KiB (`/proc/self/statm`).
pub fn rss_kib() -> Option<u64> {
    let mut b = [0u8; 128];
    let n = read_small("/proc/self/statm", &mut b)?;
    let s = std::str::from_utf8(&b[..n]).ok()?;
    let pages: u64 = s.split_whitespace().nth(1)?.parse().ok()?;
    Some(pages * page_size() / 1024)
}

/// CPU time used by this process so far, in seconds (`/proc/self/stat`).
pub fn cpu_seconds() -> Option<f64> {
    let mut b = [0u8; 512];
    let n = read_small("/proc/self/stat", &mut b)?;
    let s = std::str::from_utf8(&b[..n]).ok()?;
    // Fields after the ")" of the command name; utime/stime are 14/15.
    let rest = &s[s.rfind(')')? + 2..];
    let mut f = rest.split_whitespace().skip(11);
    let ut: u64 = f.next()?.parse().ok()?;
    let st: u64 = f.next()?.parse().ok()?;
    Some((ut + st) as f64 / clk_tck())
}

fn page_size() -> u64 {
    #[cfg(unix)]
    {
        (unsafe { libc::sysconf(libc::_SC_PAGESIZE) }).max(1) as u64
    }
    #[cfg(not(unix))]
    {
        4096
    }
}

fn clk_tck() -> f64 {
    #[cfg(unix)]
    {
        (unsafe { libc::sysconf(libc::_SC_CLK_TCK) }).max(1) as f64
    }
    #[cfg(not(unix))]
    {
        100.0
    }
}

/// Is Elegoo's `/app/app` running? It owns the MCU links, the framebuffer
/// and the touchscreen, so ccos must not run alongside it.
pub fn stock_app_running() -> bool {
    let Ok(dir) = std::fs::read_dir("/proc") else { return false };
    let me = std::process::id().to_string();
    for e in dir.flatten() {
        let name = e.file_name();
        let name = name.to_string_lossy();
        if !name.bytes().all(|b| b.is_ascii_digit()) || name == me {
            continue;
        }
        if std::fs::read_link(format!("/proc/{name}/exe")).is_ok_and(|p| p == std::path::Path::new("/app/app")) {
            return true;
        }
        let mut b = [0u8; 256];
        if let Some(n) = read_small(&format!("/proc/{name}/cmdline"), &mut b) {
            let argv0 = b[..n].split(|&c| c == 0).next().unwrap_or(&[]);
            if argv0 == b"/app/app" {
                return true;
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    #[cfg(target_os = "linux")]
    #[test]
    fn proc_stats_work_here() {
        assert!(super::rss_kib().unwrap() > 0);
        assert!(super::cpu_seconds().is_some());
        assert!(!super::stock_app_running());
    }
}
