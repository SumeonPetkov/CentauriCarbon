//! Link to the Klipper firmware running on the R528's HiFi4 DSP.
//!
//! Reproduces the stock host's `GetDspKbufOps()` path
//! (`firmware/core/allwinner/kbuf.c` + `SharespaceInit.c` + `msgbox.c`):
//!
//! 1. `/dev/dsp_debug`: read the `dsp_sharespace_t` descriptor (ioctl 1),
//!    select the *arm_write* region (ioctl 3) and mmap it. This page is only a
//!    mailbox: its head tells the DSP where the real rings live.
//! 2. `/dev/kbuf-mgr-0`: allocate a 16 KiB non-cached buffer (ioctl 0x100),
//!    mmap `/dev/kbuf-map-<minor>-<name>`. Page 0 = ARM->DSP ring, page 1 =
//!    DSP->ARM ring.
//! 3. Write `{read_addr: pa+4096, write_addr: pa, init_state: 1|2}` into the
//!    mailbox head, then `{12, 12, 1}` into ring page 0's head.
//! 4. Spin until the DSP writes `init_state == 1` into page 1's head.
//! 5. `/dev/rpmsg_ctrl0`: create endpoint `msgbox_demo` (src 3, dst ANY),
//!    open the matching `/dev/rpmsgN`. 4-byte doorbells travel over it.
//!
//! **Untested on hardware** — derived from source. Stop the stock `app`
//! process first: only one host may own the DSP link.

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
const CHOOSE_ARM_WRITE_SPACE: u32 = 1;
const KBUF_MGR_DEV_IOCTL_CREATE_BUF: libc::c_ulong = 0x100;
const KBUF_TYPE_NONCACHE: u32 = 1;
const KBUF_LEN: u32 = 4 * 4096;
/// `_IOW(0xb5, 0x1, struct rpmsg_endpoint_info)` (40 bytes).
const RPMSG_CREATE_EPT_IOCTL: libc::c_ulong = (1 << 30) | (40 << 16) | (0xb5 << 8) | 1;

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
    let fd = cvt(unsafe { libc::open(c.as_ptr(), flags | libc::O_CLOEXEC) })?;
    Ok(unsafe { OwnedFd::from_raw_fd(fd) })
}

fn map(fd: &OwnedFd, len: usize, prot: libc::c_int) -> io::Result<*mut u8> {
    let p = unsafe { libc::mmap(std::ptr::null_mut(), len, prot, libc::MAP_SHARED, fd.as_raw_fd(), 0) };
    if p == libc::MAP_FAILED { Err(io::Error::last_os_error()) } else { Ok(p as *mut u8) }
}

pub struct DspLink {
    _debug_fd: OwnedFd,
    _kbuf_fds: (OwnedFd, OwnedFd),
    _ctrl_fd: OwnedFd,
    ept_fd: OwnedFd,
    tx: MmapPage,
    rx: MmapPage,
    rings: HostRings,
    pending: Vec<u8>,
    pub descriptor: DspSharespace,
    pub kbuf_phys: u64,
}

impl DspLink {
    pub fn open(init_timeout: Duration) -> io::Result<Self> {
        // 1. mailbox page
        let debug_fd = open(DSP_DEBUG_DEV, libc::O_RDWR | libc::O_SYNC | libc::O_NONBLOCK)?;
        let mut desc = DspSharespace::default();
        cvt(unsafe { libc::ioctl(debug_fd.as_raw_fd(), CMD_READ_DEBUG_MSG as _, &mut desc) })?;
        let _ = CHOOSE_ARM_WRITE_SPACE;
        desc.mmap_phy_addr = desc.arm_write_addr;
        cvt(unsafe { libc::ioctl(debug_fd.as_raw_fd(), CMD_WRITE_DEBUG_MSG as _, &mut desc) })?;
        let mut mailbox = MmapPage { ptr: map(&debug_fd, PAGE, libc::PROT_READ | libc::PROT_WRITE)? };

        // 2. ring buffers
        let mgr = open(KBUF_MGR_DEV, libc::O_RDWR)?;
        let mut kb = KbufBufData { name: [0; 32], len: KBUF_LEN, ty: KBUF_TYPE_NONCACHE, minor: 0, va: 0, pa: 0 };
        kb.name[..4].copy_from_slice(b"test");
        cvt(unsafe { libc::ioctl(mgr.as_raw_fd(), KBUF_MGR_DEV_IOCTL_CREATE_BUF as _, &mut kb) })?;
        let map_fd = open(&format!("/dev/kbuf-map-{}-test", kb.minor), libc::O_RDWR)?;
        let base = map(&map_fd, KBUF_LEN as usize, libc::PROT_READ | libc::PROT_WRITE)?;
        let mut tx = MmapPage { ptr: base };
        let rx = MmapPage { ptr: unsafe { base.add(PAGE) } };
        let pa = kb.pa as u32;

        // 3. tell the DSP where the rings are
        let prev = Head::load(&mailbox);
        let state = if prev.init_state == 1 || prev.init_state == 2 { 2 } else { 1 };
        Head { read_addr: pa + PAGE as u32, write_addr: pa, init_state: state }.store(&mut mailbox);
        let mut rings = HostRings::default();
        rings.init(&mut tx);

        // 4. wait for the DSP
        let deadline = Instant::now() + init_timeout;
        loop {
            let h = Head::load(&rx);
            Head { read_addr: ring::MIN_ADDR, write_addr: ring::MIN_ADDR, init_state: 1 }.store(&mut tx);
            if h.init_state == 1 {
                break;
            }
            if Instant::now() > deadline {
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
            _kbuf_fds: (mgr, map_fd),
            _ctrl_fd: ctrl,
            ept_fd,
            tx,
            rx,
            rings,
            pending: Vec::new(),
            descriptor: desc,
            kbuf_phys: kb.pa as u64,
        })
    }

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

impl Link for DspLink {
    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        if data.len() > ring::MAX_WRITE {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "write too large"));
        }
        let deadline = Instant::now() + Duration::from_secs(1);
        while !self.rings.can_write(data.len()) {
            // While waiting for space, anything the DSP sends must be kept.
            self.wait_doorbell(Duration::from_millis(5))?;
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
        if self.pending.is_empty() {
            self.wait_doorbell(timeout)?;
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
            if n.trim_end() == name {
                return Ok(format!("/dev/{fname}"));
            }
        }
    }
    Err(io::Error::new(io::ErrorKind::NotFound, format!("rpmsg endpoint {name} not found")))
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
    fn ioctl_numbers() {
        assert_eq!(super::RPMSG_CREATE_EPT_IOCTL, 0x4028_b501);
        assert_eq!(std::mem::size_of::<super::RpmsgEndpointInfo>(), 40);
        assert_eq!(std::mem::size_of::<super::DspSharespace>(), 52);
    }
}
