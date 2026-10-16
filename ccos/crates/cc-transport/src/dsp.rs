//! Link to the Klipper firmware running on the R528's HiFi4 DSP.
//!
//! Reproduces the stock host's `GetDspKbufOps()` path
//! (`firmware/core/allwinner/kbuf.c` + `SharespaceInit.c` + `msgbox.c`),
//! cross-checked against OpenCentauri's on-device `dsp-to-serial` tool:
//!
//! 1. `/dev/dsp_debug`: read the `dsp_sharespace_t` descriptor (ioctl 1),
//!    select the *arm_write* region (ioctl 3) and mmap it. This page is only a
//!    mailbox: its head tells the DSP where the real rings live.
//! 2. `/dev/kbuf-mgr-0`: allocate a 16 KiB non-cached buffer (ioctl 0x100,
//!    with `pa` pre-set to `arm_write_addr` exactly like the stock host), mmap
//!    `/dev/kbuf-map-<minor>-<name>`. Page 0 = ARM->DSP ring, page 1 =
//!    DSP->ARM ring. Freed again (ioctl 0x200) when the link is dropped.
//! 3. Write `{read_addr: pa+4096, write_addr: pa, init_state: 1|2}` into the
//!    mailbox head, then `{12, 12, 1}` into ring page 0's head.
//! 4. Spin until the DSP writes `init_state == 1` into page 1's head.
//! 5. `/dev/rpmsg_ctrl0`: create endpoint `msgbox_demo` (src 3, dst ANY),
//!    open the matching `/dev/rpmsgN`. 4-byte doorbells travel over it.
//!
//! **The DSP reads the mailbox once per DSP boot** (`share_space_clear()` +
//! `share_space_init()` in its init functions). At boot it fills the mailbox
//! head with `0xA5` bytes and waits. If another host (Elegoo's `app`) has
//! already attached, the DSP must be restarted through remoteproc before a
//! new host can attach — see [`DspLink::open`]'s `restart` flag.
//!
//! **Untested on hardware** — derived from source and OpenCentauri's tool.

use std::ffi::CString;
use std::fs;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::{Duration, Instant};

use crate::ring::{self, Head, HostRings, Page, PAGE};
use crate::Link;

const DSP_DEBUG_DEV: &str = "/dev/dsp_debug";
const KBUF_MGR_DEV: &str = "/dev/kbuf-mgr-0";
const RPMSG_CTRL_DEV: &str = "/dev/rpmsg_ctrl0";
pub const ENDPOINT_NAME: &str = "msgbox_demo";

const CMD_READ_DEBUG_MSG: libc::c_ulong = 0x01;
const CMD_WRITE_DEBUG_MSG: libc::c_ulong = 0x03;
const KBUF_MGR_DEV_IOCTL_CREATE_BUF: libc::c_ulong = 0x100;
const KBUF_MGR_DEV_IOCTL_DESTROY_BUF: libc::c_ulong = 0x200;
const KBUF_TYPE_NONCACHE: u32 = 1;
const KBUF_LEN: u32 = 4 * 4096;
/// `_IOW(0xb5, 0x1, struct rpmsg_endpoint_info)` (40 bytes).
const RPMSG_CREATE_EPT_IOCTL: libc::c_ulong = (1 << 30) | (40 << 16) | (0xb5 << 8) | 1;
/// What `share_space_clear()` writes into the mailbox head at DSP boot.
const MAILBOX_CLEARED: u32 = 0xa5a5_a5a5;

#[repr(C)]
#[derive(Default, Debug, Clone, Copy)]
pub struct DspSharespace {
    pub dsp_write_addr: u32,
    pub dsp_write_size: u32,
    pub arm_write_addr: u32,
    pub arm_write_size: u32,
    pub dsp_log_addr: u32,
    pub dsp_log_size: u32,
    pub mmap_phy_addr: u32,
    pub mmap_phy_size: u32,
    pub arm_read_dsp_log_addr: u32,
    pub debug_msg: [u32; 4],
}

/// Which region the next `mmap` of `/dev/dsp_debug` maps.
#[derive(Debug, Clone, Copy)]
pub enum Region {
    DspWrite,
    ArmWrite,
    Log,
}

#[repr(C)]
struct KbufBufData {
    name: [u8; 32],
    len: u32,
    ty: u32,
    minor: i32,
    va: libc::c_ulong,
    pa: libc::c_ulong,
}

#[repr(C)]
struct RpmsgEndpointInfo {
    name: [u8; 32],
    src: u32,
    dst: u32,
}

/// One mmapped page accessed with volatile reads/writes.
pub struct MmapPage {
    ptr: *mut u8,
}

unsafe impl Send for MmapPage {}

impl Page for MmapPage {
    fn read(&self, off: usize, out: &mut [u8]) {
        assert!(off + out.len() <= PAGE);
        for (i, b) in out.iter_mut().enumerate() {
            *b = unsafe { std::ptr::read_volatile(self.ptr.add(off + i)) };
        }
    }
    fn write(&mut self, off: usize, data: &[u8]) {
        assert!(off + data.len() <= PAGE);
        for (i, b) in data.iter().enumerate() {
            unsafe { std::ptr::write_volatile(self.ptr.add(off + i), *b) };
        }
        std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
    }
}

fn cvt(r: libc::c_int) -> io::Result<libc::c_int> {
    if r < 0 { Err(io::Error::last_os_error()) } else { Ok(r) }
}

fn open(path: &str, flags: libc::c_int) -> io::Result<OwnedFd> {
    let c = CString::new(path).unwrap();
    let fd = unsafe { libc::open(c.as_ptr(), flags | libc::O_CLOEXEC) };
    if fd < 0 {
        return Err(io::Error::new(io::Error::last_os_error().kind(), format!("{path}: {}", io::Error::last_os_error())));
    }
    Ok(unsafe { OwnedFd::from_raw_fd(fd) })
}

fn map(fd: &OwnedFd, len: usize, prot: libc::c_int) -> io::Result<*mut u8> {
    let p = unsafe { libc::mmap(std::ptr::null_mut(), len, prot, libc::MAP_SHARED, fd.as_raw_fd(), 0) };
    if p == libc::MAP_FAILED { Err(io::Error::last_os_error()) } else { Ok(p as *mut u8) }
}

/// Point the next mmap of `/dev/dsp_debug` at `region` (`choose_sharespace`).
pub fn choose_region(fd: &OwnedFd, region: Region) -> io::Result<DspSharespace> {
    let mut d = DspSharespace::default();
    cvt(unsafe { libc::ioctl(fd.as_raw_fd(), CMD_READ_DEBUG_MSG as _, &mut d) })?;
    (d.mmap_phy_addr, d.mmap_phy_size) = match region {
        Region::DspWrite => (d.dsp_write_addr, d.dsp_write_size),
        Region::ArmWrite => (d.arm_write_addr, d.arm_write_size),
        Region::Log => (d.dsp_log_addr, d.dsp_log_size),
    };
    cvt(unsafe { libc::ioctl(fd.as_raw_fd(), CMD_WRITE_DEBUG_MSG as _, &mut d) })?;
    Ok(d)
}

pub struct DspLink {
    _debug_fd: OwnedFd,
    kbuf_mgr: OwnedFd,
    kbuf: KbufBufData,
    _kbuf_map_fd: OwnedFd,
    base: *mut u8,
    _ctrl_fd: OwnedFd,
    ept_fd: OwnedFd,
    tx: MmapPage,
    rx: MmapPage,
    rings: HostRings,
    pending: Vec<u8>,
    pub descriptor: DspSharespace,
}

unsafe impl Send for DspLink {}

impl DspLink {
    /// Attach to the DSP. With `restart`, the DSP firmware is first restarted
    /// via remoteproc when it is not already waiting for a host (i.e. when
    /// Elegoo's `app` attached earlier in this boot).
    pub fn open(restart: bool, init_timeout: Duration) -> io::Result<Self> {
        // 1. mailbox page
        let debug_fd = open(DSP_DEBUG_DEV, libc::O_RDWR | libc::O_SYNC | libc::O_NONBLOCK)?;
        let desc = choose_region(&debug_fd, Region::ArmWrite)?;
        let mut mailbox = MmapPage { ptr: map(&debug_fd, PAGE, libc::PROT_READ | libc::PROT_WRITE)? };

        if Head::load(&mailbox).read_addr != MAILBOX_CLEARED {
            if !restart {
                unsafe { libc::munmap(mailbox.ptr as _, PAGE) };
                return Err(io::Error::other(
                    "DSP is attached to another host (stock app ran this boot); retry with DSP restart enabled",
                ));
            }
            remoteproc::restart()?;
            let deadline = Instant::now() + init_timeout;
            while Head::load(&mailbox).read_addr != MAILBOX_CLEARED {
                if Instant::now() > deadline {
                    unsafe { libc::munmap(mailbox.ptr as _, PAGE) };
                    return Err(io::Error::new(io::ErrorKind::TimedOut, "DSP did not come back after remoteproc restart"));
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        }

        // 2. ring buffers
        let mgr = open(KBUF_MGR_DEV, libc::O_RDWR)?;
        let mut kb = KbufBufData { name: [0; 32], len: KBUF_LEN, ty: KBUF_TYPE_NONCACHE, minor: 0, va: 0, pa: desc.arm_write_addr as _ };
        kb.name[..4].copy_from_slice(b"test");
        cvt(unsafe { libc::ioctl(mgr.as_raw_fd(), KBUF_MGR_DEV_IOCTL_CREATE_BUF as _, &mut kb) })?;
        let map_fd = match open(&format!("/dev/kbuf-map-{}-test", kb.minor), libc::O_RDWR) {
            Ok(f) => f,
            Err(e) => {
                unsafe { libc::ioctl(mgr.as_raw_fd(), KBUF_MGR_DEV_IOCTL_DESTROY_BUF as _, &mut kb) };
                return Err(e);
            }
        };
        let base = map(&map_fd, KBUF_LEN as usize, libc::PROT_READ | libc::PROT_WRITE)?;
        let mut tx = MmapPage { ptr: base };
        let mut rx = MmapPage { ptr: unsafe { base.add(PAGE) } };
        let pa = kb.pa as u32;

        // A recycled buffer could still hold a stale "init_state = 1".
        Head::default().store(&mut rx);

        // 3. tell the DSP where the rings are
        let prev = Head::load(&mailbox);
        let state = if prev.init_state == 1 || prev.init_state == 2 { 2 } else { 1 };
        Head { read_addr: pa + PAGE as u32, write_addr: pa, init_state: state }.store(&mut mailbox);
        let mut rings = HostRings::default();
        rings.init(&mut tx);

        // 4. wait for the DSP (wait_dsp_set_init)
        let deadline = Instant::now() + init_timeout;
        loop {
            Head { read_addr: ring::MIN_ADDR, write_addr: ring::MIN_ADDR, init_state: 1 }.store(&mut tx);
            if Head::load(&rx).init_state == 1 {
                break;
            }
            if Instant::now() > deadline {
                unsafe { libc::ioctl(mgr.as_raw_fd(), KBUF_MGR_DEV_IOCTL_DESTROY_BUF as _, &mut kb) };
                return Err(io::Error::new(io::ErrorKind::TimedOut, "DSP did not set init_state"));
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        unsafe { libc::munmap(mailbox.ptr as _, PAGE) };

        // 5. doorbell endpoint
        let ctrl = open(RPMSG_CTRL_DEV, libc::O_RDWR)?;
        let mut info = RpmsgEndpointInfo { name: [0; 32], src: 3, dst: 0xffff_ffff };
        info.name[..ENDPOINT_NAME.len()].copy_from_slice(ENDPOINT_NAME.as_bytes());
        cvt(unsafe { libc::ioctl(ctrl.as_raw_fd(), RPMSG_CREATE_EPT_IOCTL as _, &mut info) })?;
        let ept = find_endpoint(ENDPOINT_NAME)?;
        let ept_fd = open(&ept, libc::O_RDWR)?;

        Ok(Self {
            _debug_fd: debug_fd,
            kbuf_mgr: mgr,
            kbuf: kb,
            _kbuf_map_fd: map_fd,
            base,
            _ctrl_fd: ctrl,
            ept_fd,
            tx,
            rx,
            rings,
            pending: Vec::with_capacity(PAGE),
            descriptor: desc,
        })
    }

    /// Physical address of the ring buffer (for diagnostics).
    pub fn kbuf_phys(&self) -> u32 {
        self.kbuf.pa as u32
    }

    /// Wait for a doorbell; returns false on timeout.
    fn wait_doorbell(&mut self, timeout: Duration) -> io::Result<bool> {
        let mut pfd = libc::pollfd { fd: self.ept_fd.as_raw_fd(), events: libc::POLLIN, revents: 0 };
        let ms = timeout.as_millis().min(i32::MAX as u128) as i32;
        if cvt(unsafe { libc::poll(&mut pfd, 1, ms) })? == 0 {
            return Ok(false);
        }
        let mut word = [0u8; 4];
        let n = unsafe { libc::read(self.ept_fd.as_raw_fd(), word.as_mut_ptr() as _, 4) };
        if n == 4 {
            self.rings.on_doorbell(u32::from_le_bytes(word));
        }
        Ok(true)
    }
}

impl Drop for DspLink {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.base as _, KBUF_LEN as usize);
            libc::ioctl(self.kbuf_mgr.as_raw_fd(), KBUF_MGR_DEV_IOCTL_DESTROY_BUF as _, &mut self.kbuf);
        }
    }
}

impl Link for DspLink {
    fn fd(&self) -> Option<i32> {
        Some(self.ept_fd.as_raw_fd())
    }

    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        if data.is_empty() || data.len() > ring::MAX_WRITE {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "bad write size"));
        }
        let deadline = Instant::now() + Duration::from_secs(1);
        loop {
            // The DSP publishes its consumer position in its own page head.
            self.rings.refresh_dsp_read(&self.rx);
            if self.rings.can_write(data.len()) {
                break;
            }
            // While waiting for space, anything the DSP sends must be kept.
            self.wait_doorbell(Duration::from_millis(2))?;
            self.rings.read(&self.rx, &mut self.pending);
            if Instant::now() > deadline {
                return Err(io::Error::new(io::ErrorKind::WouldBlock, "DSP ring full"));
            }
        }
        let bell = self.rings.write(&mut self.tx, data);
        let n = unsafe { libc::write(self.ept_fd.as_raw_fd(), bell.to_le_bytes().as_ptr() as _, 4) };
        if n != 4 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        if self.pending.is_empty() {
            self.rings.read(&self.rx, &mut self.pending);
        }
        if self.pending.is_empty() && self.wait_doorbell(timeout)? {
            self.rings.read(&self.rx, &mut self.pending);
        }
        let n = buf.len().min(self.pending.len());
        buf[..n].copy_from_slice(&self.pending[..n]);
        self.pending.drain(..n);
        Ok(n)
    }
}

/// `/sys/class/rpmsg/rpmsgN/name == name` -> `/dev/rpmsgN`.
pub fn find_endpoint(name: &str) -> io::Result<String> {
    for e in fs::read_dir("/sys/class/rpmsg")? {
        let e = e?;
        let fname = e.file_name();
        let fname = fname.to_string_lossy();
        if !fname.starts_with("rpmsg") || fname.starts_with("rpmsg_ctrl") {
            continue;
        }
        if let Ok(n) = fs::read_to_string(e.path().join("name")) {
            if n.trim_end_matches(['\0', '\n']) == name {
                return Ok(format!("/dev/{fname}"));
            }
        }
    }
    Err(io::Error::new(io::ErrorKind::NotFound, format!("rpmsg endpoint {name} not found")))
}

/// Read new DSP `printf` output from its log region (format as used by
/// OpenCentauri: `u32` write offset at 0, NUL-separated strings from 4).
pub struct DspLog {
    _fd: OwnedFd,
    mem: *const u8,
    len: usize,
    read: usize,
}

impl DspLog {
    pub fn open() -> io::Result<Self> {
        let fd = open(DSP_DEBUG_DEV, libc::O_RDWR | libc::O_SYNC | libc::O_NONBLOCK)?;
        let d = choose_region(&fd, Region::Log)?;
        let len = d.dsp_log_size as usize;
        if len < 8 {
            return Err(io::Error::other("DSP log region is empty"));
        }
        let mem = map(&fd, len, libc::PROT_READ)? as *const u8;
        let mut l = Self { _fd: fd, mem, len, read: 4 };
        l.read = l.write_pos();
        Ok(l)
    }

    fn write_pos(&self) -> usize {
        let mut b = [0u8; 4];
        for (i, x) in b.iter_mut().enumerate() {
            *x = unsafe { std::ptr::read_volatile(self.mem.add(i)) };
        }
        (u32::from_le_bytes(b) as usize).clamp(4, self.len)
    }

    /// Append any new log lines to `out`.
    pub fn poll(&mut self, out: &mut Vec<String>) {
        let w = self.write_pos();
        if w == self.read {
            return;
        }
        let byte = |i: usize| unsafe { std::ptr::read_volatile(self.mem.add(i)) };
        let mut cur = Vec::new();
        let mut i = self.read;
        while i != w {
            let b = byte(i);
            if b == 0 {
                if !cur.is_empty() {
                    out.push(String::from_utf8_lossy(&cur).trim_end().to_string());
                    cur.clear();
                }
            } else {
                cur.push(b);
            }
            i += 1;
            if i >= self.len {
                i = 4;
            }
        }
        if !cur.is_empty() {
            out.push(String::from_utf8_lossy(&cur).trim_end().to_string());
        }
        self.read = w;
    }
}

impl Drop for DspLog {
    fn drop(&mut self) {
        unsafe { libc::munmap(self.mem as _, self.len) };
    }
}

/// DSP firmware lifecycle via remoteproc (`aw_dsp.c`).
pub mod remoteproc {
    use std::fs;
    use std::io;
    const STATE: &str = "/sys/class/remoteproc/remoteproc0/state";

    pub fn state() -> io::Result<String> {
        Ok(fs::read_to_string(STATE)?.trim().to_string())
    }
    pub fn start() -> io::Result<()> {
        fs::write(STATE, "start\n")
    }
    pub fn stop() -> io::Result<()> {
        fs::write(STATE, "stop\n")
    }
    pub fn restart() -> io::Result<()> {
        if state()? == "running" {
            stop()?;
        }
        start()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn ioctl_numbers_and_layouts() {
        assert_eq!(super::RPMSG_CREATE_EPT_IOCTL, 0x4028_b501);
        assert_eq!(std::mem::size_of::<super::RpmsgEndpointInfo>(), 40);
        assert_eq!(std::mem::size_of::<super::DspSharespace>(), 52);
        let ul = std::mem::size_of::<libc::c_ulong>();
        assert_eq!(std::mem::size_of::<super::KbufBufData>(), if ul == 4 { 52 } else { 64 });
    }
}
