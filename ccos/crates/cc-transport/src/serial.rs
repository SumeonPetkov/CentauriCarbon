//! Raw termios serial link (toolhead `/dev/ttyACM0`, strain gauge `/dev/ttyS4`).

use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::Duration;

use crate::Link;

pub const TOOLHEAD_PORT: &str = "/dev/ttyACM0";
pub const TOOLHEAD_BAUD: u32 = 1_000_000;
pub const STRAIN_GAUGE_PORT: &str = "/dev/ttyS4";
pub const STRAIN_GAUGE_BAUD: u32 = 115_200;

pub struct SerialLink {
    fd: OwnedFd,
}

fn baud_const(baud: u32) -> io::Result<libc::speed_t> {
    Ok(match baud {
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        460800 => libc::B460800,
        500000 => libc::B500000,
        921600 => libc::B921600,
        1000000 => libc::B1000000,
        1500000 => libc::B1500000,
        2000000 => libc::B2000000,
        _ => return Err(io::Error::new(io::ErrorKind::InvalidInput, format!("unsupported baud {baud}"))),
    })
}

impl SerialLink {
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let c = CString::new(path).unwrap();
        let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        let fd = unsafe { OwnedFd::from_raw_fd(fd) };
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd.as_raw_fd(), &mut t) != 0 {
                return Err(io::Error::last_os_error());
            }
            libc::cfmakeraw(&mut t);
            t.c_cflag |= libc::CLOCAL | libc::CREAD;
            t.c_cflag &= !libc::CRTSCTS;
            t.c_cc[libc::VMIN] = 0;
            t.c_cc[libc::VTIME] = 0;
            let sp = baud_const(baud)?;
            libc::cfsetispeed(&mut t, sp);
            libc::cfsetospeed(&mut t, sp);
            if libc::tcsetattr(fd.as_raw_fd(), libc::TCSANOW, &t) != 0 {
                return Err(io::Error::last_os_error());
            }
            libc::tcflush(fd.as_raw_fd(), libc::TCIOFLUSH);
        }
        Ok(Self { fd })
    }
}

impl Link for SerialLink {
    fn send(&mut self, mut data: &[u8]) -> io::Result<()> {
        while !data.is_empty() {
            let n = unsafe { libc::write(self.fd.as_raw_fd(), data.as_ptr() as _, data.len()) };
            if n < 0 {
                let e = io::Error::last_os_error();
                if e.kind() == io::ErrorKind::WouldBlock {
                    let mut pfd = libc::pollfd { fd: self.fd.as_raw_fd(), events: libc::POLLOUT, revents: 0 };
                    unsafe { libc::poll(&mut pfd, 1, 100) };
                    continue;
                }
                return Err(e);
            }
            data = &data[n as usize..];
        }
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        let mut pfd = libc::pollfd { fd: self.fd.as_raw_fd(), events: libc::POLLIN, revents: 0 };
        let ms = timeout.as_millis().min(i32::MAX as u128) as i32;
        let r = unsafe { libc::poll(&mut pfd, 1, ms) };
        if r < 0 {
            return Err(io::Error::last_os_error());
        }
        if r == 0 {
            return Ok(0);
        }
        let n = unsafe { libc::read(self.fd.as_raw_fd(), buf.as_mut_ptr() as _, buf.len()) };
        if n < 0 {
            let e = io::Error::last_os_error();
            return if e.kind() == io::ErrorKind::WouldBlock { Ok(0) } else { Err(e) };
        }
        Ok(n as usize)
    }
}
