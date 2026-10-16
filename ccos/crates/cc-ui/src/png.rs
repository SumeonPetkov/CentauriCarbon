//! Minimal dependency-free PNG writer (uncompressed deflate blocks) used to
//! render screens on a PC. A few hundred bytes of code; not used on the
//! printer unless you ask for a screenshot.

use crate::canvas::Canvas;

fn crc32(data: &[u8], mut crc: u32) -> u32 {
    crc = !crc;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xedb8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let crc = crc32(data, crc32(kind, 0));
    out.extend_from_slice(&crc.to_be_bytes());
}

/// Encode the canvas as an RGB PNG.
pub fn encode(c: &Canvas) -> Vec<u8> {
    let (w, h) = (c.width as usize, c.height as usize);
    let mut raw = Vec::with_capacity(h * (1 + 3 * w));
    for y in 0..h {
        raw.push(0); // filter: none
        for p in &c.pixels[y * w..(y + 1) * w] {
            raw.extend_from_slice(&[(p >> 16) as u8, (p >> 8) as u8, *p as u8]);
        }
    }
    // zlib stream with stored blocks
    let mut z = vec![0x78, 0x01];
    for (i, block) in raw.chunks(65535).enumerate() {
        let last = (i + 1) * 65535 >= raw.len();
        z.push(last as u8);
        z.extend_from_slice(&(block.len() as u16).to_le_bytes());
        z.extend_from_slice(&(!(block.len() as u16)).to_le_bytes());
        z.extend_from_slice(block);
    }
    let (mut a, mut b) = (1u32, 0u32);
    for &x in &raw {
        a = (a + x as u32) % 65521;
        b = (b + a) % 65521;
    }
    z.extend_from_slice(&((b << 16) | a).to_be_bytes());

    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&(w as u32).to_be_bytes());
    ihdr.extend_from_slice(&(h as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &z);
    chunk(&mut out, b"IEND", &[]);
    out
}

#[cfg(test)]
mod tests {
    #[test]
    fn crc_known_value() {
        assert_eq!(super::crc32(b"IEND", 0), 0xae42_6082);
    }
}
