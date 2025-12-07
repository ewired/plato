use std::mem::{self, MaybeUninit};
use std::ptr;
use std::slice;
use std::thread;
use std::io::Read;
use std::fs::File;
use std::sync::mpsc::{self, Sender, Receiver};
use std::os::unix::io::AsRawFd;
use std::ffi::CString;
use std::time::{Duration, Instant};
use std::collections::HashSet;
use fxhash::FxHashMap;
use serde::Serialize;
use nix::{ioctl_read, ioctl_read_buf};
use crate::framebuffer::Display;
use crate::settings::ButtonScheme;
use crate::device::CURRENT_DEVICE;
use crate::geom::{Point, LinearDir};
use anyhow::{Error, Context};

// Event types
pub const EV_SYN: u16 = 0x00;
pub const EV_KEY: u16 = 0x01;
pub const EV_ABS: u16 = 0x03;
pub const EV_MSC: u16 = 0x04;

// Event codes
pub const ABS_MT_TRACKING_ID: u16 = 0x39;
pub const ABS_MT_POSITION_X: u16 = 0x35;
pub const ABS_MT_POSITION_Y: u16 = 0x36;
pub const ABS_MT_PRESSURE: u16 = 0x3a;
pub const ABS_MT_TOUCH_MAJOR: u16 = 0x30;
pub const ABS_X: u16 = 0x00;
pub const ABS_Y: u16 = 0x01;
pub const ABS_PRESSURE: u16 = 0x18;
pub const MSC_RAW: u16 = 0x03;
pub const SYN_REPORT: u16 = 0x00;

// Event values
pub const MSC_RAW_GSENSOR_PORTRAIT_DOWN: i32 = 0x17;
pub const MSC_RAW_GSENSOR_PORTRAIT_UP: i32 = 0x18;
pub const MSC_RAW_GSENSOR_LANDSCAPE_RIGHT: i32 = 0x19;
pub const MSC_RAW_GSENSOR_LANDSCAPE_LEFT: i32 = 0x1a;
// pub const MSC_RAW_GSENSOR_BACK: i32 = 0x1b;
// pub const MSC_RAW_GSENSOR_FRONT: i32 = 0x1c;

// The indices of this clockwise ordering of the sensor values match the Forma's rotation values.
pub const GYROSCOPE_ROTATIONS: [i32; 4] = [MSC_RAW_GSENSOR_LANDSCAPE_LEFT, MSC_RAW_GSENSOR_PORTRAIT_UP,
                                           MSC_RAW_GSENSOR_LANDSCAPE_RIGHT, MSC_RAW_GSENSOR_PORTRAIT_DOWN];

pub const VAL_RELEASE: i32 = 0;
pub const VAL_PRESS: i32 = 1;
pub const VAL_REPEAT: i32 = 2;

// Key codes
pub const KEY_POWER: u16 = 116;
pub const KEY_HOME: u16 = 102;
pub const KEY_LIGHT: u16 = 90;
pub const KEY_BACKWARD: u16 = 193;
pub const KEY_FORWARD: u16 = 194;
pub const KEY_SPACE: u16 = 57;
pub const PEN_ERASE: u16 = 331;
pub const PEN_HIGHLIGHT: u16 = 332;
pub const SLEEP_COVER: [u16; 2] = [59, 35];
// Synthetic touch button
pub const BTN_TOUCH: u16 = 330;
// The following key codes are fake, and are used to support
// software toggles within this design
pub const KEY_ROTATE_DISPLAY: u16 = 0xffff;
pub const KEY_BUTTON_SCHEME: u16 = 0xfffe;

pub const SINGLE_TOUCH_CODES: TouchCodes = TouchCodes {
    pressure: ABS_PRESSURE,
    x: ABS_X,
    y: ABS_Y,
};

pub const MULTI_TOUCH_CODES_A: TouchCodes = TouchCodes {
    pressure: ABS_MT_TOUCH_MAJOR,
    x: ABS_MT_POSITION_X,
    y: ABS_MT_POSITION_Y,
};

pub const MULTI_TOUCH_CODES_B: TouchCodes = TouchCodes {
    pressure: ABS_MT_PRESSURE,
    .. MULTI_TOUCH_CODES_A
};

#[repr(C)]
#[derive(Debug, Clone)]
pub struct InputEvent {
    pub time: libc::timeval,
    pub kind: u16, // type
    pub code: u16,
    pub value: i32,
}

#[derive(Debug, Clone)]
pub struct TaggedInputEvent {
    pub event: InputEvent,
    pub is_dynamic: bool, // true for hotplugged devices, false for static devices
}

// Handle different touch protocols
#[derive(Debug)]
pub struct TouchCodes {
    pressure: u16,
    x: u16,
    y: u16,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum TouchProto {
    Single,
    MultiA,
    MultiB, // Pressure won't indicate a finger release.
    MultiC,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum FingerStatus {
    Down,
    Motion,
    Up,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum ButtonStatus {
    Pressed,
    Released,
    Repeated,
}

impl ButtonStatus {
    pub fn try_from_raw(value: i32) -> Option<ButtonStatus> {
        match value {
            VAL_RELEASE => Some(ButtonStatus::Released),
            VAL_PRESS => Some(ButtonStatus::Pressed),
            VAL_REPEAT => Some(ButtonStatus::Repeated),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize)]
#[serde(rename_all = "camelCase")]
pub enum ButtonCode {
    Power,
    Home,
    Light,
    Backward,
    Forward,
    Erase,
    Highlight,
    Raw(u16),
}

impl ButtonCode {
    fn from_raw(code: u16, rotation: i8, button_scheme: ButtonScheme, is_dynamic: bool) -> ButtonCode {
        // For dynamic devices, treat special keys as raw to avoid conflicts with sleep cover
        if is_dynamic {
            return ButtonCode::Raw(code)
        }
        match code {
            KEY_POWER => ButtonCode::Power,
            KEY_HOME => ButtonCode::Home,
            KEY_LIGHT => ButtonCode::Light,
            KEY_BACKWARD => resolve_button_direction(LinearDir::Backward, rotation, button_scheme),
            KEY_FORWARD => resolve_button_direction(LinearDir::Forward, rotation, button_scheme),
            KEY_SPACE => ButtonCode::Forward,
            PEN_ERASE => ButtonCode::Erase,
            PEN_HIGHLIGHT => ButtonCode::Highlight,
            _ => ButtonCode::Raw(code)
        }
    }
}

fn resolve_button_direction(mut direction: LinearDir, rotation: i8, button_scheme: ButtonScheme) -> ButtonCode {
    if (CURRENT_DEVICE.should_invert_buttons(rotation)) ^ (button_scheme == ButtonScheme::Inverted) {
        direction = direction.opposite();
    }

    if direction == LinearDir::Forward {
        return ButtonCode::Forward;
    }

    ButtonCode::Backward
}

pub fn display_rotate_event(n: i8) -> TaggedInputEvent {
    let mut tp = libc::timeval { tv_sec: 0, tv_usec: 0 };
    unsafe { libc::gettimeofday(&mut tp, ptr::null_mut()); }
    TaggedInputEvent {
        event: InputEvent {
            time: tp,
            kind: EV_KEY,
            code: KEY_ROTATE_DISPLAY,
            value: n as i32,
        },
        is_dynamic: false,
    }
}

pub fn button_scheme_event(v: i32) -> TaggedInputEvent {
    let mut tp = libc::timeval { tv_sec: 0, tv_usec: 0 };
    unsafe { libc::gettimeofday(&mut tp, ptr::null_mut()); }
    TaggedInputEvent {
        event: InputEvent {
            time: tp,
            kind: EV_KEY,
            code: KEY_BUTTON_SCHEME,
            value: v,
        },
        is_dynamic: false,
    }
}

#[derive(Debug, Copy, Clone)]
pub enum DeviceEvent {
    Finger {
        id: i32,
        time: f64,
        status: FingerStatus,
        position: Point,
    },
    Button {
        time: f64,
        code: ButtonCode,
        status: ButtonStatus,
    },
    Plug(PowerSource),
    Unplug(PowerSource),
    RotateScreen(i8),
    CoverOn,
    CoverOff,
    NetUp,
    UserActivity,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PowerSource {
    Host,
    Wall,
}

pub fn seconds(time: libc::timeval) -> f64 {
    time.tv_sec as f64 + time.tv_usec as f64 / 1e6
}

// Use nix crate for proper ioctl definitions
ioctl_read!(eviocgid, b'E', 0x02, InputId);

#[repr(C)]
#[derive(Debug, Clone)]
struct InputId {
    bustype: u16,
    vendor: u16,
    product: u16,
    version: u16,
}

fn get_device_identity(path: &str) -> Option<String> {
    if let Ok(file) = File::open(path) {
        let fd = file.as_raw_fd();
        
        let mut id = InputId {
            bustype: 0,
            vendor: 0,
            product: 0,
            version: 0,
        };
        
        match unsafe { eviocgid(fd, &mut id) } {
            Ok(_) => {
                if id.bustype != 0 || id.vendor != 0 || id.product != 0 {
                    return Some(format!("id:{:04x}:{:04x}:{:04x}:{:04x}", 
                        id.bustype, id.vendor, id.product, id.version));
                }
            }
            Err(_) => {}
        }
    }
    
    None
}

// Use nix crate for proper ioctl definitions
ioctl_read_buf!(eviocgbit_ev, b'E', 0x20, u8);

fn is_keyboard_device(path: &str) -> bool {
    if let Ok(file) = File::open(path) {
        let fd = file.as_raw_fd();
        let mut evbits = [0u8; 4];
        match unsafe { eviocgbit_ev(fd, &mut evbits) } {
            Ok(len) if len > 0 => {
                // Check if EV_KEY (bit 1) is set in the first byte
                return (evbits[0] & (1 << 1)) != 0;
            }
            _ => {}
        }
    }
    false
}

/// Scan for keyboard devices in /dev/input/eventN, excluding paths in `static_paths`
/// and deduplicating based on device identity.
fn scan_dynamic_keyboards(static_paths: &[String], existing_dynamic: &[String], blacklisted_paths: &mut HashSet<String>) -> Vec<String> {
    let mut keyboards = Vec::new();
    let mut seen_identities = HashSet::new();
    
    // First, collect identities of all static devices to avoid duplicates
    for path in static_paths {
        if let Some(identity) = get_device_identity(path) {
            seen_identities.insert(identity);
        }
    }
    
    // Also collect identities of existing dynamic devices
    for path in existing_dynamic {
        if let Some(identity) = get_device_identity(path) {
            seen_identities.insert(identity);
        }
    }
    
    let entries = match std::fs::read_dir("/dev/input") {
        Ok(e) => e,
        Err(_) => return keyboards,
    };
    
    for entry in entries.flatten() {
        let path = entry.path();
        let path_str = match path.to_str() {
            Some(s) => s,
            None => continue,
        };
        
        // Only look at /dev/input/eventN
        if !path_str.starts_with("/dev/input/event") {
            continue;
        }
        
        // Skip hard-coded static devices
        if static_paths.iter().any(|p| p == path_str) {
            continue;
        }
        
        // Skip if already tracked as dynamic
        if existing_dynamic.iter().any(|p| p == path_str) {
            continue;
        }
        
        // Skip if blacklisted (previously identified as duplicate)
        if blacklisted_paths.contains(path_str) {
            continue;
        }
        
        // Check if it's a keyboard-like device (has EV_KEY)
        if is_keyboard_device(path_str) {
            // Get device identity to check for duplicates
            if let Some(identity) = get_device_identity(path_str) {
                if !seen_identities.contains(&identity) {
                    seen_identities.insert(identity.clone());
                    keyboards.push(path_str.to_string());
                    eprintln!("Hotplug: found new keyboard device {} (identity: {})", path_str, identity);
                } else {
                    eprintln!("Hotplug: blacklisting duplicate keyboard device {} (identity: {})", path_str, identity);
                    blacklisted_paths.insert(path_str.to_string());
                }
            }
        }
    }
    
    keyboards
}

pub fn raw_events(paths: Vec<String>) -> (Sender<TaggedInputEvent>, Receiver<TaggedInputEvent>) {
    let (tx, rx) = mpsc::channel();
    let tx_for_thread = tx.clone();
    thread::spawn(move || parse_raw_events(&paths, &tx_for_thread).ok());
    (tx, rx)
}

fn parse_raw_events(paths: &[String], tx: &Sender<TaggedInputEvent>) -> Result<(), Error> {
    // Static devices: always open, never close
    let mut static_files = Vec::new();
    let mut static_pfds = Vec::new();
    let static_paths: Vec<String> = paths.to_vec();
    
    for path in paths.iter() {
        let file = File::open(path)
            .with_context(|| format!("can't open input file {}", path))?;
        let fd = file.as_raw_fd();
        static_files.push(file);
        static_pfds.push(libc::pollfd {
            fd,
            events: libc::POLLIN,
            revents: 0,
        });
    }
    
    // Dynamic devices: hotplugged keyboards
    let mut dynamic_files: Vec<File> = Vec::new();
    let mut dynamic_pfds: Vec<libc::pollfd> = Vec::new();
    let mut dynamic_paths: Vec<String> = Vec::new();
    
    // Persistent blacklist of duplicate device paths
    let mut blacklisted_paths: HashSet<String> = HashSet::new();
    
    // Set to epoch to trigger immediate scan on first iteration
    let mut last_scan = Instant::now() - Duration::from_secs(10);
    const SCAN_INTERVAL: Duration = Duration::from_secs(5);
    
    loop {
        // Scan for new devices periodically
        if last_scan.elapsed() >= SCAN_INTERVAL {
            last_scan = Instant::now();
            
            let new_keyboards = scan_dynamic_keyboards(&static_paths, &dynamic_paths, &mut blacklisted_paths);
            for path in new_keyboards {
                if let Ok(file) = File::open(&path) {
                    let fd = file.as_raw_fd();
                    dynamic_pfds.push(libc::pollfd {
                        fd,
                        events: libc::POLLIN,
                        revents: 0,
                    });
                    dynamic_files.push(file);
                    dynamic_paths.push(path.clone());
                    eprintln!("Hotplug: opened keyboard device {}", path);
                }
            }
        }
        
        // Build combined poll array
        let mut all_pfds: Vec<libc::pollfd> = static_pfds.clone();
        all_pfds.extend(dynamic_pfds.iter().cloned());
        
        // Calculate timeout until next scan
        let elapsed = last_scan.elapsed();
        let timeout_ms = if elapsed >= SCAN_INTERVAL {
            0
        } else {
            SCAN_INTERVAL.saturating_sub(elapsed).as_millis() as i32
        };
        
        let ret = unsafe { libc::poll(all_pfds.as_mut_ptr(), all_pfds.len() as libc::nfds_t, timeout_ms) };
        if ret < 0 {
            break;
        }
        
        // Process static device events (indices 0..static_pfds.len())
        for (i, pfd) in all_pfds[..static_pfds.len()].iter().enumerate() {
            if pfd.revents & libc::POLLIN != 0 {
                let mut input_event = MaybeUninit::<InputEvent>::uninit();
                unsafe {
                    let event_slice = slice::from_raw_parts_mut(
                        input_event.as_mut_ptr() as *mut u8,
                        mem::size_of::<InputEvent>()
                    );
                    if static_files[i].read_exact(event_slice).is_err() {
                        continue;
                    }
                    tx.send(TaggedInputEvent {
                        event: input_event.assume_init(),
                        is_dynamic: false,
                    }).ok();
                }
            }
        }
        
        // Process dynamic device events and track removals
        let mut to_remove = Vec::new();
        let dynamic_start = static_pfds.len();
        
        for (i, pfd) in all_pfds[dynamic_start..].iter().enumerate() {
            if pfd.revents & libc::POLLIN != 0 {
                let mut input_event = MaybeUninit::<InputEvent>::uninit();
                unsafe {
                    let event_slice = slice::from_raw_parts_mut(
                        input_event.as_mut_ptr() as *mut u8,
                        mem::size_of::<InputEvent>()
                    );
                    if dynamic_files[i].read_exact(event_slice).is_err() {
                        to_remove.push(i);
                        continue;
                    }
                    tx.send(TaggedInputEvent {
                        event: input_event.assume_init(),
                        is_dynamic: true,
                    }).ok();
                }
            } else if pfd.revents & (libc::POLLERR | libc::POLLHUP) != 0 {
                to_remove.push(i);
            }
        }
        
        // Remove disconnected dynamic devices (reverse order)
        for &i in to_remove.iter().rev() {
            eprintln!("Hotplug: closed keyboard device {}", dynamic_paths[i]);
            dynamic_files.remove(i);
            dynamic_pfds.remove(i);
            dynamic_paths.remove(i);
        }
    }
    
    Ok(())
}

pub fn usb_events() -> Receiver<DeviceEvent> {
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || parse_usb_events(&tx));
    rx
}

fn parse_usb_events(tx: &Sender<DeviceEvent>) {
    let path = CString::new("/tmp/nickel-hardware-status").unwrap();
    let fd = unsafe { libc::open(path.as_ptr(), libc::O_NONBLOCK | libc::O_RDWR) };

    if fd < 0 {
        return;
    }

    let mut pfd = libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    };

    const BUF_LEN: usize = 256;

    loop {
        let ret = unsafe { libc::poll(&mut pfd as *mut libc::pollfd, 1, -1) };

        if ret < 0 {
            break;
        }

        let buf = CString::new(vec![1; BUF_LEN]).unwrap();
        let c_buf = buf.into_raw();

        if pfd.revents & libc::POLLIN != 0 {
            let n = unsafe { libc::read(fd, c_buf as *mut libc::c_void, BUF_LEN as libc::size_t) };
            let buf = unsafe { CString::from_raw(c_buf) };
            if n > 0 {
                if let Ok(s) = buf.to_str() {
                    for msg in s[..n as usize].lines() {
                        if msg == "usb plug add" {
                            tx.send(DeviceEvent::Plug(PowerSource::Host)).ok();
                        } else if msg == "usb plug remove" {
                            tx.send(DeviceEvent::Unplug(PowerSource::Host)).ok();
                        } else if msg == "usb ac add" {
                            tx.send(DeviceEvent::Plug(PowerSource::Wall)).ok();
                        } else if msg == "usb ac remove" {
                            tx.send(DeviceEvent::Unplug(PowerSource::Wall)).ok();
                        } else if msg.starts_with("network bound") {
                            tx.send(DeviceEvent::NetUp).ok();
                        }
                    }
                }
            } else {
                break;
            }
        }
    }
}

pub fn device_events(rx: Receiver<TaggedInputEvent>, display: Display, button_scheme: ButtonScheme) -> Receiver<DeviceEvent> {
    let (ty, ry) = mpsc::channel();
    thread::spawn(move || parse_device_events(&rx, &ty, display, button_scheme));
    ry
}

struct TouchState {
    position: Point,
    pressure: i32,
}

impl Default for TouchState {
    fn default() -> Self {
        TouchState {
            position: Point::default(),
            pressure: 0,
        }
    }
}

pub fn parse_device_events(rx: &Receiver<TaggedInputEvent>, ty: &Sender<DeviceEvent>, display: Display, button_scheme: ButtonScheme) {
    let mut id = 0;
    let mut last_activity = -60;
    let Display { mut dims, mut rotation } = display;
    let mut fingers: FxHashMap<i32, Point> = FxHashMap::default();
    let mut packets: FxHashMap<i32, TouchState> = FxHashMap::default();
    let proto = CURRENT_DEVICE.proto;

    let mut tc = match proto {
        TouchProto::Single => SINGLE_TOUCH_CODES,
        TouchProto::MultiA => MULTI_TOUCH_CODES_A,
        TouchProto::MultiB => MULTI_TOUCH_CODES_B,
        TouchProto::MultiC => MULTI_TOUCH_CODES_B,
    };

    if proto == TouchProto::Single {
        packets.insert(id, TouchState::default());
    }

    let (mut mirror_x, mut mirror_y) = CURRENT_DEVICE.should_mirror_axes(rotation);
    if CURRENT_DEVICE.should_swap_axes(rotation) {
        mem::swap(&mut tc.x, &mut tc.y);
    }

    let mut button_scheme = button_scheme;

    while let Ok(tagged_evt) = rx.recv() {
        let evt = tagged_evt.event;
        if evt.kind == EV_ABS {
            if evt.code == ABS_MT_TRACKING_ID {
                if evt.value >= 0 {
                    id = evt.value;
                    packets.insert(id, TouchState::default());
                }
            } else if evt.code == tc.x {
                if let Some(state) = packets.get_mut(&id) {
                    state.position.x = if mirror_x {
                        dims.0 as i32 - 1 - evt.value
                    } else {
                        evt.value
                    };
                }
            } else if evt.code == tc.y {
                if let Some(state) = packets.get_mut(&id) {
                    state.position.y = if mirror_y {
                        dims.1 as i32 - 1 - evt.value
                    } else {
                        evt.value
                    };
                }
            } else if evt.code == tc.pressure {
                if let Some(state) = packets.get_mut(&id) {
                    state.pressure = evt.value;
                    if proto == TouchProto::Single && CURRENT_DEVICE.mark() == 3 && state.pressure == 0 {
                        state.position.x = dims.0 as i32 - 1 - state.position.x;
                        mem::swap(&mut state.position.x, &mut state.position.y);
                    }
                }
            }
        } else if evt.kind == EV_SYN && evt.code == SYN_REPORT {
            // The absolute value accounts for the wrapping around that might occur,
            // since `tv_sec` can't grow forever.
            if (evt.time.tv_sec - last_activity).abs() >= 60 {
                last_activity = evt.time.tv_sec;
                ty.send(DeviceEvent::UserActivity).ok();
            }

            if proto == TouchProto::MultiB {
                fingers.retain(|other_id, other_position| {
                    packets.contains_key(&other_id) ||
                    ty.send(DeviceEvent::Finger {
                        id: *other_id,
                        time: seconds(evt.time),
                        status: FingerStatus::Up,
                        position: *other_position,
                    }).is_err()
                });
            }

            for (&id, state) in &packets {
                if let Some(&pos) = fingers.get(&id) {
                    if state.pressure > 0 {
                        if state.position != pos {
                            ty.send(DeviceEvent::Finger {
                                id,
                                time: seconds(evt.time),
                                status: FingerStatus::Motion,
                                position: state.position,
                            }).unwrap();
                            fingers.insert(id, state.position);
                        }
                    } else {
                        ty.send(DeviceEvent::Finger {
                            id,
                            time: seconds(evt.time),
                            status: FingerStatus::Up,
                            position: state.position,
                        }).unwrap();
                        fingers.remove(&id);
                    }
                } else if state.pressure > 0 {
                    ty.send(DeviceEvent::Finger {
                        id,
                        time: seconds(evt.time),
                        status: FingerStatus::Down,
                        position: state.position,
                    }).unwrap();
                    fingers.insert(id, state.position);
                }
            }

            if proto != TouchProto::Single {
                packets.clear();
            }
        } else if evt.kind == EV_KEY {
            if SLEEP_COVER.contains(&evt.code) {
                if evt.value == VAL_PRESS {
                    ty.send(DeviceEvent::CoverOn).ok();
                } else if evt.value == VAL_RELEASE {
                    ty.send(DeviceEvent::CoverOff).ok();
                } else if evt.value == VAL_REPEAT {
                    ty.send(DeviceEvent::CoverOn).ok();
                }
            } else if evt.code == KEY_BUTTON_SCHEME {
                if evt.value == VAL_PRESS {
                    button_scheme = ButtonScheme::Inverted;
                } else {
                    button_scheme = ButtonScheme::Natural;
                }
            } else if evt.code == KEY_ROTATE_DISPLAY {
                let next_rotation = evt.value as i8;
                if next_rotation != rotation {
                    let delta = (rotation - next_rotation).abs();
                    if delta % 2 == 1 {
                        mem::swap(&mut tc.x, &mut tc.y);
                        mem::swap(&mut dims.0, &mut dims.1);
                    }
                    rotation = next_rotation;
                    let should_mirror = CURRENT_DEVICE.should_mirror_axes(rotation);
                    mirror_x = should_mirror.0;
                    mirror_y = should_mirror.1;
                }
            } else if evt.code != BTN_TOUCH {
                if let Some(button_status) = ButtonStatus::try_from_raw(evt.value) {
                    ty.send(DeviceEvent::Button {
                        time: seconds(evt.time),
                        code: ButtonCode::from_raw(evt.code, rotation, button_scheme, tagged_evt.is_dynamic),
                        status: button_status,
                    }).unwrap();
                }
            }
        } else if evt.kind == EV_MSC && evt.code == MSC_RAW {
            if evt.value >= MSC_RAW_GSENSOR_PORTRAIT_DOWN && evt.value <= MSC_RAW_GSENSOR_LANDSCAPE_LEFT {
                let next_rotation = GYROSCOPE_ROTATIONS.iter().position(|&v| v == evt.value)
                                                       .map(|i| CURRENT_DEVICE.transformed_gyroscope_rotation(i as i8));
                if let Some(next_rotation) = next_rotation {
                    ty.send(DeviceEvent::RotateScreen(next_rotation)).ok();
                }
            }
        }
    }
}
