//! Resistive/capacitive touch via evdev (`/dev/input/event1` on the stock image).
//!
//! The stock driver feeds raw ABS_X/ABS_Y straight to LVGL (no calibration),
//! so the panel already reports screen coordinates. Use [`Touch::set_transform`]
//! if a replacement panel needs scaling or axis swapping.

use std::ffi::CString;
use std::io;

const EV_KEY: u16 = 0x01;
const EV_ABS: u16 = 0x03;
const ABS_X: u16 = 0x00;
const ABS_Y: u16 = 0x01;
const ABS_MT_POSITION_X: u16 = 0x35;
const ABS_MT_POSITION_Y: u16 = 0x36;
const ABS_MT_TRACKING_ID: u16 = 0x39;
const BTN_TOUCH: u16 = 0x14a;
const BTN_MOUSE: u16 = 0x110;

/// Kernel `struct input_event` (on 32-bit ARM the time fields are 32-bit even
/// with 64-bit `time_t` userspace).
#[repr(C)]
#[derive(Default, Clone, Copy)]
struct InputEvent {
    sec: libc::c_ulong,
    usec: libc::c_ulong,
    type_: u16,
    code: u16,
    value: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TouchState {
    pub x: i32,
    pub y: i32,
    pub pressed: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct Transform {
    pub swap_xy: bool,
    pub invert_x: bool,
    pub invert_y: bool,
    pub raw: (i32, i32, i32, i32), // x_min, x_max, y_min, y_max
    pub screen: (i32, i32),
}

pub struct Touch {
    fd: libc::c_int,
    state: TouchState,
    transform: Option<Transform>,
}

impl Touch {
    pub fn open(path: &str) -> io::Result<Self> {
        let c = CString::new(path).unwrap();
        let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDONLY | libc::O_NONBLOCK | libc::O_CLOEXEC) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(Self { fd, state: TouchState { x: 0, y: 0, pressed: false }, transform: None })
    }

    pub fn set_transform(&mut self, t: Option<Transform>) {
        self.transform = t;
    }

    pub fn fd(&self) -> libc::c_int {
        self.fd
    }

    /// Drain pending events; returns the latest state.
    pub fn poll(&mut self) -> TouchState {
        let mut ev = InputEvent::default();
        let sz = std::mem::size_of::<InputEvent>();
        loop {
            let n = unsafe { libc::read(self.fd, &mut ev as *mut _ as *mut libc::c_void, sz) };
            if n != sz as isize {
                break;
            }
            match (ev.type_, ev.code) {
                (EV_ABS, ABS_X | ABS_MT_POSITION_X) => self.state.x = ev.value,
                (EV_ABS, ABS_Y | ABS_MT_POSITION_Y) => self.state.y = ev.value,
                (EV_ABS, ABS_MT_TRACKING_ID) => self.state.pressed = ev.value != -1,
                (EV_KEY, BTN_TOUCH | BTN_MOUSE) => self.state.pressed = ev.value != 0,
                _ => {}
            }
        }
        self.mapped()
    }

    fn mapped(&self) -> TouchState {
        let Some(t) = self.transform else { return self.state };
        let (mut x, mut y) = (self.state.x, self.state.y);
        if t.swap_xy {
            std::mem::swap(&mut x, &mut y);
        }
        let map = |v: i32, lo: i32, hi: i32, out: i32| ((v - lo) * out / (hi - lo).max(1)).clamp(0, out - 1);
        let mut x = map(x, t.raw.0, t.raw.1, t.screen.0);
        let mut y = map(y, t.raw.2, t.raw.3, t.screen.1);
        if t.invert_x {
            x = t.screen.0 - 1 - x;
        }
        if t.invert_y {
            y = t.screen.1 - 1 - y;
        }
        TouchState { x, y, pressed: self.state.pressed }
    }
}

impl Drop for Touch {
    fn drop(&mut self) {
        unsafe { libc::close(self.fd) };
    }
}
