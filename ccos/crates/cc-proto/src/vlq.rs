//! Klipper "variable length quantity" integer encoding.
//!
//! Signed-aware: small negative numbers are as short as small positive ones.
//! Every 32-bit value fits in at most 5 bytes.

/// Append `v` to `out`. Signed and unsigned values share the same encoding
/// (pass `x as u32` for an `i32`).
pub fn encode(out: &mut Vec<u8>, v: u32) {
    let mut tmp = [0u8; 5];
    let n = encode_into(&mut tmp, v);
    out.extend_from_slice(&tmp[..n]);
}

/// Encode `v` into `out`, returning the number of bytes used (1..=5).
pub fn encode_into(out: &mut [u8; 5], v: u32) -> usize {
    let sv = v as i32;
    let len = if (-(1 << 5)..(3 << 5)).contains(&sv) {
        1
    } else if (-(1 << 12)..(3 << 12)).contains(&sv) {
        2
    } else if (-(1 << 19)..(3 << 19)).contains(&sv) {
        3
    } else if (-(1 << 26)..(3 << 26)).contains(&sv) {
        4
    } else {
        5
    };
    for (k, i) in (1..len).rev().enumerate() {
        let shift = 7 * i;
        let chunk = if shift == 28 { v >> 28 } else { (v >> shift) & 0x7f };
        out[k] = chunk as u8 | 0x80;
    }
    out[len - 1] = (v & 0x7f) as u8;
    len
}

/// Decode one value from the front of `buf`, returning it and the number of
/// bytes consumed.
pub fn decode(buf: &[u8]) -> Option<(u32, usize)> {
    let mut it = buf.iter();
    let mut c = *it.next()?;
    let mut v = (c & 0x7f) as u32;
    if c & 0x60 == 0x60 {
        v |= (-0x20i32) as u32;
    }
    let mut n = 1;
    while c & 0x80 != 0 {
        c = *it.next()?;
        v = (v << 7) | (c & 0x7f) as u32;
        n += 1;
        if n > 5 {
            return None;
        }
    }
    Some((v, n))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Generated from Elegoo's `encode_int` / `parse_int` (chelper/msgblock.c).
    const VECTORS: &[(i32, &[u8])] = &[
        (0, &[0x00]),
        (1, &[0x01]),
        (-1, &[0x7f]),
        (95, &[0x5f]),
        (96, &[0x80, 0x60]),
        (-32, &[0x60]),
        (-33, &[0xff, 0x5f]),
        (3000, &[0x97, 0x38]),
        (12287, &[0xdf, 0x7f]),
        (12288, &[0x80, 0xe0, 0x00]),
        (-4096, &[0xe0, 0x00]),
        (-4097, &[0xff, 0xdf, 0x7f]),
        (393215, &[0x97, 0xff, 0x7f]),
        (393216, &[0x98, 0x80, 0x00]),
        (-524288, &[0xe0, 0x80, 0x00]),
        (-524289, &[0xff, 0xdf, 0xff, 0x7f]),
        (100663295, &[0xaf, 0xff, 0xff, 0x7f]),
        (100663296, &[0xb0, 0x80, 0x80, 0x00]),
        (-67108864, &[0xe0, 0x80, 0x80, 0x00]),
        (-67108865, &[0x8f, 0xdf, 0xff, 0xff, 0x7f]),
        (2147483647, &[0x87, 0xff, 0xff, 0xff, 0x7f]),
        (i32::MIN, &[0x88, 0x80, 0x80, 0x80, 0x00]),
        (84000000, &[0xa8, 0x86, 0xfa, 0x00]),
        (200000000, &[0xdf, 0xaf, 0x84, 0x00]),
        (-123456, &[0xf8, 0xbb, 0x40]),
    ];

    #[test]
    fn encode_matches_c() {
        for &(v, want) in VECTORS {
            let mut out = Vec::new();
            encode(&mut out, v as u32);
            assert_eq!(out, want, "encode({v})");
        }
    }

    #[test]
    fn decode_matches_c() {
        for &(v, bytes) in VECTORS {
            assert_eq!(decode(bytes), Some((v as u32, bytes.len())), "decode({v})");
        }
    }

    #[test]
    fn roundtrip_sweep() {
        let mut x: u32 = 1;
        for _ in 0..2000 {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            for v in [x, x >> 7, x >> 14, x >> 21, (x as i32 >> 20) as u32] {
                let mut b = Vec::new();
                encode(&mut b, v);
                assert_eq!(decode(&b), Some((v, b.len())));
            }
        }
    }

    #[test]
    fn truncated() {
        assert_eq!(decode(&[]), None);
        assert_eq!(decode(&[0x80]), None);
    }
}
