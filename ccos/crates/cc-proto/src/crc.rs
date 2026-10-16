/// CRC-16/CCITT as implemented in Klipper (`msgblock_crc16_ccitt`).
pub fn crc16_ccitt(buf: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &b in buf {
        let mut data = b ^ (crc as u8);
        data ^= data << 4;
        crc = (((data as u16) << 8) | (crc >> 8)) ^ ((data >> 4) as u16) ^ ((data as u16) << 3);
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    // Reference values produced by compiling Elegoo's chelper/msgblock.c.
    #[test]
    fn matches_c_reference() {
        assert_eq!(crc16_ccitt(&[1, 2, 3, 4, 5]), 0xdd13);
        assert_eq!(crc16_ccitt(&[0x10]), 0x1f06);
        assert_eq!(crc16_ccitt(&[]), 0xffff);
    }
}
