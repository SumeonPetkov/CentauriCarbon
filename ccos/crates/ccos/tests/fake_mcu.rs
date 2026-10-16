//! End-to-end test of the host stack (termios serial link -> framing ->
//! sequencing -> Mcu session) against a fake Klipper MCU on a pseudo-terminal.

#![cfg(target_os = "linux")]

use std::time::Instant;

use cc_proto::{encode_frame, Dictionary, FrameDecoder, MsgBuf};
use cc_transport::serial::SerialLink;
use ccos::mcu::Mcu;

const BLOB: &[u8] = b"0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ-identify-blob";

/// Minimal MCU: acks every frame, answers identify/get_config/get_clock, and
/// NAKs the very first frame to exercise the retransmit path.
fn fake_mcu(master: libc::c_int, clock_hz: f64) {
    let d = Dictionary::centauri();
    let mut dec = FrameDecoder::new();
    let mut expect: u8 = 3; // host starts at 0 -> first frame is NAKed
    let t0 = Instant::now();
    let mut buf = [0u8; 256];
    loop {
        let n = unsafe { libc::read(master, buf.as_mut_ptr() as _, buf.len()) };
        if n <= 0 {
            return;
        }
        dec.push(&buf[..n as usize]);
        while let Some(f) = dec.next_frame() {
            if f.seq != expect {
                let nak = encode_frame(expect, &[]).unwrap();
                unsafe { libc::write(master, nak.as_ptr() as _, nak.len()) };
                continue;
            }
            expect = (expect + 1) & 0x0f;
            let mut out = MsgBuf::new();
            for m in d.decode(f.payload.as_slice()) {
                let m = m.unwrap();
                let reply = match m.name() {
                    "identify" => {
                        let off = m.int("offset").unwrap() as usize;
                        let cnt = m.int("count").unwrap() as usize;
                        let chunk = &BLOB[off.min(BLOB.len())..(off + cnt).min(BLOB.len())];
                        let hex: String = chunk.iter().map(|b| format!("{b:02x}")).collect();
                        if hex.is_empty() { format!("identify_response offset={off} data=") } else { format!("identify_response offset={off} data=0x{hex}") }
                    }
                    "get_config" => "config is_config=0 crc=0 move_count=0 is_shutdown=0".to_string(),
                    "get_clock" => format!("clock clock={}", (t0.elapsed().as_secs_f64() * clock_hz) as u64 as u32),
                    _ => continue,
                };
                d.encode_str(&reply, &mut out).unwrap();
            }
            let fr = encode_frame(expect, out.as_slice()).unwrap();
            unsafe { libc::write(master, fr.as_ptr() as _, fr.len()) };
        }
    }
}

#[test]
fn host_stack_against_fake_mcu_over_pty() {
    let (master, name) = unsafe {
        let m = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY);
        assert!(m >= 0);
        assert_eq!(libc::grantpt(m), 0);
        assert_eq!(libc::unlockpt(m), 0);
        // raw mode on the master side too, so bytes pass untouched
        let mut t: libc::termios = std::mem::zeroed();
        libc::tcgetattr(m, &mut t);
        libc::cfmakeraw(&mut t);
        libc::tcsetattr(m, libc::TCSANOW, &t);
        (m, std::ffi::CStr::from_ptr(libc::ptsname(m)).to_str().unwrap().to_string())
    };
    let hz = 84_000_000.0;
    std::thread::spawn(move || fake_mcu(master, hz));

    let link = SerialLink::open(&name, 115200).unwrap();
    let mut m = Mcu::new("fake", Box::new(link), hz, 192);
    m.sync().unwrap();
    assert_eq!(m.identify().unwrap(), BLOB);
    let cfg = m.request("get_config", "config").unwrap();
    assert_eq!(cfg.int("is_config"), Some(0));
    let measured = m.measure_clock(0.5).unwrap();
    assert!((measured - hz).abs() / hz < 0.05, "clock {measured}");
}
