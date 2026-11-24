use std::fs::{self, File, OpenOptions};
use std::io::{self, Read};
use std::os::unix::fs::OpenOptionsExt;
use std::path::Path;
use std::time::{Duration, Instant};

use evdev::{AbsoluteAxisCode, raw_stream::RawDevice};
use gilrs::LinuxGamepadExt;

const SIXAXIS_REPORT_LEN: usize = 49;
const R2_PRESSURE_BYTE: usize = 19;
const REPORT_TIMEOUT: Duration = Duration::from_millis(300);

pub enum Ps3Trigger {
    Evdev(RawDevice),
    Hidraw {
        device: File,
        sample: Option<(u8, Instant)>,
    },
}

impl Ps3Trigger {
    pub fn open(pad: &gilrs::Gamepad<'_>) -> Option<Self> {
        if pad.vendor_id() != Some(0x054c) || pad.product_id() != Some(0x0268) {
            return None;
        }
        if let Ok(device) = RawDevice::open(pad.devpath())
            && device.supported_absolute_axes().is_some_and(|axes| {
                [
                    AbsoluteAxisCode::ABS_RX,
                    AbsoluteAxisCode::ABS_RY,
                    AbsoluteAxisCode::ABS_RZ,
                ]
                .into_iter()
                .all(|axis| axes.contains(axis))
            })
        {
            // hid-sony has separate RX/RY stick axes and puts R2 on RZ.
            return Some(Self::Evdev(device));
        }

        // The generic Bluetooth HID driver exposes the right stick on Z/RZ.
        // Sixaxis report 1 still includes the two trigger pressure bytes.
        let event = pad.devpath().file_name()?;
        let hid = Path::new("/sys/class/input")
            .join(event)
            .join("device/device/hidraw");
        let node = fs::read_dir(hid).ok()?.find_map(|entry| {
            let entry = entry.ok()?;
            let name = entry.file_name();
            name.to_str()?.starts_with("hidraw").then_some(name)
        })?;
        let device = OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(Path::new("/dev").join(node))
            .ok()?;
        Some(Self::Hidraw {
            device,
            sample: None,
        })
    }

    pub fn pressure(&mut self, now: Instant) -> Option<f32> {
        match self {
            Self::Evdev(device) => {
                let axes = device.get_abs_state().ok()?;
                let axis = axes[usize::from(AbsoluteAxisCode::ABS_RZ.0)];
                let span = axis.maximum - axis.minimum;
                (span > 0).then(|| (axis.value - axis.minimum) as f32 / span as f32)
            }
            Self::Hidraw { device, sample } => {
                let mut report = [0; 64];
                for _ in 0..32 {
                    match device.read(&mut report) {
                        Ok(0) => {
                            *sample = None;
                            break;
                        }
                        Ok(SIXAXIS_REPORT_LEN) if report[0] == 1 => {
                            *sample = Some((report[R2_PRESSURE_BYTE], now));
                        }
                        Ok(_) => {}
                        Err(error) if error.kind() == io::ErrorKind::WouldBlock => break,
                        Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                        Err(_) => {
                            *sample = None;
                            break;
                        }
                    }
                }
                sample
                    .filter(|(_, at)| now.saturating_duration_since(*at) <= REPORT_TIMEOUT)
                    .map(|(pressure, _)| f32::from(pressure) / 255.0)
            }
        }
    }
}
