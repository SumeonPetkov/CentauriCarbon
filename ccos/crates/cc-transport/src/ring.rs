//! The ARM<->DSP ring-buffer protocol, independent of how the memory is
//! mapped (so it can be unit-tested against a simulated DSP).
//!
//! Each direction is one 4 KiB page:
//!
//! ```text
//! 0          12                                   4084       4096
//! | unused   | data ring (4072 bytes)              | msg_head |
//! ```
//! `msg_head = { read_addr: u32, write_addr: u32, init_state: u32 }` (LE),
//! stored at the *end* of the page (`SHARE_SPACE_HEAD_END`). Positions are
//! byte offsets inside the page and wrap from 4084 back to 12.
//!
//! The writer publishes its position in its own page head **and** by ringing
//! the rpmsg "msgbox" doorbell with `(write_pos << 16) | read_pos`. The
//! reader's consumed position only travels via the doorbell's low half.

pub const PAGE: usize = 4096;
pub const HEAD_SIZE: usize = 12;
pub const HEAD_OFFSET: usize = PAGE - HEAD_SIZE; // 4084
pub const MIN_ADDR: u32 = HEAD_SIZE as u32; // 12
pub const MAX_ADDR: u32 = HEAD_OFFSET as u32; // 4084
pub const CAPACITY: u32 = MAX_ADDR - MIN_ADDR; // 4072
/// Stock host refuses single writes larger than this.
pub const MAX_WRITE: usize = 4000;

/// Something that behaves like one shared page.
pub trait Page {
    fn read(&self, off: usize, out: &mut [u8]);
    fn write(&mut self, off: usize, data: &[u8]);
}

impl Page for [u8; PAGE] {
    fn read(&self, off: usize, out: &mut [u8]) {
        out.copy_from_slice(&self[off..off + out.len()]);
    }
    fn write(&mut self, off: usize, data: &[u8]) {
        self[off..off + data.len()].copy_from_slice(data);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Head {
    pub read_addr: u32,
    pub write_addr: u32,
    pub init_state: u32,
}

impl Head {
    pub fn load(p: &impl Page) -> Self {
        let mut b = [0u8; HEAD_SIZE];
        p.read(HEAD_OFFSET, &mut b);
        let w = |i: usize| u32::from_le_bytes(b[i..i + 4].try_into().unwrap());
        Self { read_addr: w(0), write_addr: w(4), init_state: w(8) }
    }
    pub fn store(&self, p: &mut impl Page) {
        let mut b = [0u8; HEAD_SIZE];
        b[0..4].copy_from_slice(&self.read_addr.to_le_bytes());
        b[4..8].copy_from_slice(&self.write_addr.to_le_bytes());
        b[8..12].copy_from_slice(&self.init_state.to_le_bytes());
        p.write(HEAD_OFFSET, &b);
    }
}

/// Bytes used between `read` and `write` positions.
pub fn used(read: u32, write: u32) -> u32 {
    if write >= read { write - read } else { CAPACITY - (read - write) }
}

/// Free space as computed by the stock host (`DSP_mem_write`). A write of
/// `len` bytes is allowed only when `free > len`, so the ring never fills
/// completely (full and empty would be indistinguishable).
pub fn free(read: u32, write: u32) -> u32 {
    CAPACITY - used(read, write)
}

/// Copy `data` into the ring at `pos`, wrapping; returns the new position.
pub fn ring_write(p: &mut impl Page, pos: u32, data: &[u8]) -> u32 {
    let first = ((MAX_ADDR - pos) as usize).min(data.len());
    p.write(pos as usize, &data[..first]);
    if first < data.len() {
        p.write(MIN_ADDR as usize, &data[first..]);
        MIN_ADDR + (data.len() - first) as u32
    } else {
        let np = pos + first as u32;
        if np >= MAX_ADDR { MIN_ADDR } else { np }
    }
}

/// Copy everything between `pos` and `end` out of the ring; returns the new
/// position (== `end`).
pub fn ring_read(p: &impl Page, pos: u32, end: u32, out: &mut Vec<u8>) -> u32 {
    let n = used(pos, end) as usize;
    let start = out.len();
    out.resize(start + n, 0);
    let first = ((MAX_ADDR - pos) as usize).min(n);
    p.read(pos as usize, &mut out[start..start + first]);
    if first < n {
        p.read(MIN_ADDR as usize, &mut out[start + first..]);
    }
    end
}

/// Doorbell word: `(write_pos << 16) | read_pos`.
pub fn doorbell(write_pos: u32, read_pos: u32) -> u32 {
    ((write_pos & 0xffff) << 16) | (read_pos & 0xffff)
}

/// Split a doorbell word into `(read_pos, write_pos)`.
pub fn split_doorbell(v: u32) -> (u32, u32) {
    (v & 0xffff, v >> 16)
}

/// Host-side state machine for the two rings. Holds no memory itself.
#[derive(Debug, Clone)]
pub struct HostRings {
    /// Where we write next in the ARM->DSP page.
    pub tx_write: u32,
    /// Where we read next in the DSP->ARM page.
    pub rx_read: u32,
    /// Last DSP consumer position seen on the doorbell.
    pub dsp_read: u32,
}

impl Default for HostRings {
    fn default() -> Self {
        Self { tx_write: MIN_ADDR, rx_read: MIN_ADDR, dsp_read: MIN_ADDR }
    }
}

impl HostRings {
    /// Initialise our page head the way `set_arm_ops_addr` does.
    pub fn init(&mut self, tx: &mut impl Page) {
        *self = Self::default();
        Head { read_addr: MIN_ADDR, write_addr: MIN_ADDR, init_state: 1 }.store(tx);
    }

    /// Feed a doorbell received from the DSP.
    pub fn on_doorbell(&mut self, word: u32) {
        let (r, _w) = split_doorbell(word);
        // msgbox_read_signal ignores positions >= 5000 (garbage / reset).
        if r < 5000 && (MIN_ADDR..MAX_ADDR).contains(&r) {
            self.dsp_read = r;
        }
    }

    pub fn can_write(&self, len: usize) -> bool {
        len <= MAX_WRITE && free(self.dsp_read, self.tx_write) as usize > len
    }

    /// Write `data` and update our head. Returns the doorbell word to send.
    /// Caller must check [`can_write`](Self::can_write) first.
    pub fn write(&mut self, tx: &mut impl Page, data: &[u8]) -> u32 {
        debug_assert!(self.can_write(data.len()));
        self.tx_write = ring_write(tx, self.tx_write, data);
        Head { read_addr: self.rx_read, write_addr: self.tx_write, init_state: 1 }.store(tx);
        doorbell(self.tx_write, self.rx_read)
    }

    /// Drain everything the DSP has published.
    pub fn read(&mut self, rx: &impl Page, out: &mut Vec<u8>) -> usize {
        let h = Head::load(rx);
        if !(MIN_ADDR..MAX_ADDR).contains(&h.write_addr) || h.write_addr == self.rx_read {
            return 0;
        }
        let before = out.len();
        self.rx_read = ring_read(rx, self.rx_read, h.write_addr, out);
        out.len() - before
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal model of the DSP side (share_space.c) for testing.
    struct FakeDsp {
        rx_read: u32,  // DSP consumer pos in ARM->DSP page
        tx_write: u32, // DSP producer pos in DSP->ARM page
    }

    impl FakeDsp {
        fn consume(&mut self, arm_page: &[u8; PAGE], out: &mut Vec<u8>) -> u32 {
            let h = Head::load(arm_page);
            self.rx_read = ring_read(arm_page, self.rx_read, h.write_addr, out);
            doorbell(self.tx_write, self.rx_read)
        }
        fn produce(&mut self, dsp_page: &mut [u8; PAGE], data: &[u8]) -> u32 {
            self.tx_write = ring_write(dsp_page, self.tx_write, data);
            Head { read_addr: self.rx_read, write_addr: self.tx_write, init_state: 1 }.store(dsp_page);
            doorbell(self.tx_write, self.rx_read)
        }
    }

    #[test]
    fn head_layout_is_le_at_end_of_page() {
        let mut p = [0u8; PAGE];
        Head { read_addr: 0x11223344, write_addr: 12, init_state: 1 }.store(&mut p);
        assert_eq!(&p[4084..4088], &[0x44, 0x33, 0x22, 0x11]);
        assert_eq!(&p[4088..4092], &[12, 0, 0, 0]);
        assert_eq!(&p[4092..4096], &[1, 0, 0, 0]);
    }

    #[test]
    fn wraparound_both_directions() {
        let mut arm = Box::new([0u8; PAGE]);
        let mut dspp = Box::new([0u8; PAGE]);
        let mut host = HostRings::default();
        host.init(&mut *arm);
        let mut dsp = FakeDsp { rx_read: MIN_ADDR, tx_write: MIN_ADDR };
        Head { read_addr: MIN_ADDR, write_addr: MIN_ADDR, init_state: 1 }.store(&mut *dspp);

        let mut sent_all = Vec::new();
        let mut got_dsp = Vec::new();
        let mut got_host = Vec::new();
        let mut produced = Vec::new();
        // 3000 frames of varying size = many wraps of a 4072-byte ring.
        for i in 0..3000u32 {
            let frame: Vec<u8> = (0..(5 + i % 60)).map(|j| (i * 7 + j) as u8).collect();
            while !host.can_write(frame.len()) {
                let bell = dsp.consume(&arm, &mut got_dsp);
                host.on_doorbell(bell);
            }
            let bell = host.write(&mut *arm, &frame);
            assert_eq!(split_doorbell(bell).1, host.tx_write);
            sent_all.extend_from_slice(&frame);

            if i % 3 == 0 {
                let reply: Vec<u8> = (0..(5 + i % 50)).map(|j| (i + j) as u8).collect();
                dsp.produce(&mut dspp, &reply);
                produced.extend_from_slice(&reply);
                host.read(&*dspp, &mut got_host);
            }
        }
        dsp.consume(&arm, &mut got_dsp);
        assert_eq!(got_dsp, sent_all);
        assert_eq!(got_host, produced);
    }

    #[test]
    fn never_fills_completely() {
        let mut arm = Box::new([0u8; PAGE]);
        let mut host = HostRings::default();
        host.init(&mut *arm);
        let mut written = 0;
        while host.can_write(64) {
            host.write(&mut *arm, &[0xaa; 64]);
            written += 64;
        }
        assert!(written < CAPACITY as usize);
        assert!(free(host.dsp_read, host.tx_write) <= 64);
    }

    #[test]
    fn ignores_garbage_doorbell() {
        let mut h = HostRings::default();
        h.on_doorbell(doorbell(100, 6000));
        assert_eq!(h.dsp_read, MIN_ADDR);
        h.on_doorbell(doorbell(100, 200));
        assert_eq!(h.dsp_read, 200);
    }
}
