//! Linux framebuffer output (`/dev/fb0`, sunxi DE2, 480x272 ARGB8888 on the stock image).
//! The mapping is written directly; no extra copy of the screen is kept.

use std::ffi::CString;
use std::io;

use crate::canvas::{Canvas, Rect};

const FBIOGET_VSCREENINFO: libc::c_ulong = 0x4600;
const FBIOGET_FSCREENINFO: libc::c_ulong = 0x4602;
const FBIOBLANK: libc::c_ulong = 0x4611;

#[repr(C)]
#[derive(Default, Clone, Copy, Debug)]
pub struct FbVarScreeninfo {
    pub xres: u32,
    pub yres: u32,
    pub xres_virtual: u32,
    pub yres_virtual: u32,
    pub xoffset: u32,
    pub yoffset: u32,
    pub bits_per_pixel: u32,
    pub grayscale: u32,
    pub red: [u32; 3],
    pub green: [u32; 3],
    pub blue: [u32; 3],
    pub transp: [u32; 3],
    pub nonstd: u32,
    pub activate: u32,
    pub height: u32,
    pub width: u32,
    pub accel_flags: u32,
    /// pixclock, margins (4), hsync_len, vsync_len, sync, vmode, rotate, colorspace
    pub timing: [u32; 11],
    pub reserved: [u32; 4],
}

#[repr(C)]
#[derive(Clone, Copy)]
struct FbFixScreeninfo {
    id: [u8; 16],
    smem_start: libc::c_ulong,
    smem_len: u32,
    type_: u32,
    type_aux: u32,
    visual: u32,
    xpanstep: u16,
    ypanstep: u16,
    ywrapstep: u16,
    line_length: u32,
    mmio_start: libc::c_ulong,
    mmio_len: u32,
    accel: u32,
    capabilities: u16,
    reserved: [u16; 2],
}

pub struct Framebuffer {
    fd: libc::c_int,
    mem: *mut u8,
    len: usize,
    pub var: FbVarScreeninfo,
    pub line_length: usize,
}

unsafe impl Send for Framebuffer {}

impl Framebuffer {
    pub fn open(path: &str) -> io::Result<Self> {
        let c = CString::new(path).unwrap();
        let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_CLOEXEC) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        let mut var = FbVarScreeninfo::default();
        let mut fix: FbFixScreeninfo = unsafe { std::mem::zeroed() };
        unsafe {
            if libc::ioctl(fd, FBIOGET_FSCREENINFO as _, &mut fix) < 0 || libc::ioctl(fd, FBIOGET_VSCREENINFO as _, &mut var) < 0 {
                let e = io::Error::last_os_error();
                libc::close(fd);
                return Err(e);
            }
            libc::ioctl(fd, FBIOBLANK as _, 0 /* FB_BLANK_UNBLANK */);
        }
        let len = fix.smem_len as usize;
        let mem = unsafe { libc::mmap(std::ptr::null_mut(), len, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, fd, 0) };
        if mem == libc::MAP_FAILED {
            let e = io::Error::last_os_error();
            unsafe { libc::close(fd) };
            return Err(e);
        }
        Ok(Self { fd, mem: mem as *mut u8, len, var, line_length: fix.line_length as usize })
    }

    pub fn width(&self) -> u32 {
        self.var.xres
    }
    pub fn height(&self) -> u32 {
        self.var.yres
    }

    /// Copy the whole canvas to the visible page.
    pub fn present(&mut self, c: &Canvas) {
        self.present_rect(c, c.bounds());
    }

    /// Copy one rectangle of the canvas to the visible page (only what
    /// changed needs to cross the bus).
    pub fn present_rect(&mut self, c: &Canvas, r: Rect) {
        let r = r.intersect(&Rect::new(0, 0, self.var.xres.min(c.width as u32) as i16, self.var.yres.min(c.height as u32) as i16));
        if r.is_empty() {
            return;
        }
        let bpp = self.var.bits_per_pixel as usize / 8;
        let base = self.var.yoffset as usize * self.line_length + self.var.xoffset as usize * bpp;
        let (x0, x1) = (r.x as usize, r.right() as usize);
        for y in r.y as usize..r.bottom() as usize {
            let src = &c.pixels[y * c.width as usize + x0..y * c.width as usize + x1];
            let off = base + y * self.line_length + x0 * bpp;
            if off + src.len() * bpp > self.len {
                return;
            }
            match bpp {
                4 => {
                    let dst = unsafe { std::slice::from_raw_parts_mut(self.mem.add(off) as *mut u32, src.len()) };
                    for (d, s) in dst.iter_mut().zip(src) {
                        *d = 0xff00_0000 | s;
                    }
                }
                2 => {
                    let dst = unsafe { std::slice::from_raw_parts_mut(self.mem.add(off) as *mut u16, src.len()) };
                    for (d, s) in dst.iter_mut().zip(src) {
                        *d = (((s >> 8) & 0xf800) | ((s >> 5) & 0x07e0) | ((s >> 3) & 0x1f)) as u16;
                    }
                }
                _ => return,
            }
        }
    }

    pub fn blank(&self, off: bool) {
        unsafe { libc::ioctl(self.fd, FBIOBLANK as _, if off { 4 } else { 0 }) };
    }
}

impl Drop for Framebuffer {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.mem as _, self.len);
            libc::close(self.fd);
        }
    }
}

/// Backlight 0..=255 via the sunxi display debugfs, exactly like `main.cpp`
/// (`echo setbl > /sys/kernel/debug/dispdbg/command` ...).
pub fn set_backlight(level: u8) -> io::Result<()> {
    let d = "/sys/kernel/debug/dispdbg";
    std::fs::write(format!("{d}/command"), "setbl")?;
    std::fs::write(format!("{d}/name"), "lcd0")?;
    std::fs::write(format!("{d}/param"), level.to_string())?;
    std::fs::write(format!("{d}/start"), "1")
}

#[cfg(test)]
mod tests {
    #[test]
    fn struct_sizes_match_linux_uapi() {
        assert_eq!(std::mem::size_of::<super::FbVarScreeninfo>(), 160);
        let ul = std::mem::size_of::<libc::c_ulong>();
        // id(16) + smem_start + smem_len..ywrapstep + line_length + mmio_start + mmio_len, accel, caps, reserved
        assert_eq!(std::mem::size_of::<super::FbFixScreeninfo>(), if ul == 8 { 80 } else { 68 });
    }
}
