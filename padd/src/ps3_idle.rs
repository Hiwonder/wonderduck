//! Disconnect idle Bluetooth PS3 controllers without blocking the control loop.

use std::collections::HashMap;
use std::path::Path;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use evdev::{BusType, raw_stream::RawDevice};
use gilrs::{Axis, Button, Event, EventType, Gamepad, GamepadId, Gilrs, LinuxGamepadExt};

pub(crate) fn load_timeout(path: &Path) -> std::io::Result<Duration> {
    let settings = robotd_params::gamepad_idle::Settings::load(path)?;
    Ok(Duration::from_secs(u64::from(settings.minutes) * 60))
}

const DISCONNECT_RETRY: Duration = Duration::from_secs(10);
const TRIGGER_DEADZONE: f32 = 0.02;
const STICKS: [Axis; 4] = [
    Axis::LeftStickX,
    Axis::LeftStickY,
    Axis::RightStickX,
    Axis::RightStickY,
];
const BUTTONS: [Button; 17] = [
    Button::South,
    Button::East,
    Button::North,
    Button::West,
    Button::LeftTrigger,
    Button::RightTrigger,
    Button::LeftTrigger2,
    Button::RightTrigger2,
    Button::Select,
    Button::Start,
    Button::Mode,
    Button::LeftThumb,
    Button::RightThumb,
    Button::DPadUp,
    Button::DPadDown,
    Button::DPadLeft,
    Button::DPadRight,
];

#[derive(Default)]
pub(crate) struct Ps3Idle {
    devices: HashMap<GamepadId, Option<Connection>>,
}

struct Connection {
    address: String,
    last_activity: Instant,
    last_attempt: Option<Instant>,
}

impl Connection {
    fn open(pad: &Gamepad<'_>, now: Instant) -> Option<Self> {
        if pad.vendor_id() != Some(0x054c) || pad.product_id() != Some(0x0268) {
            return None;
        }
        let evdev = RawDevice::open(pad.devpath()).ok()?;
        if evdev.input_id().bus_type() != BusType::BUS_BLUETOOTH {
            return None;
        }
        let address = evdev.unique_name()?.trim();
        valid_bluetooth_address(address).then(|| Self {
            address: address.to_ascii_uppercase(),
            last_activity: now,
            last_attempt: None,
        })
    }

    fn record_activity(&mut self, now: Instant) {
        self.last_activity = now;
        self.last_attempt = None;
    }

    fn disconnect_due(&mut self, now: Instant, timeout: Duration) -> bool {
        if now.saturating_duration_since(self.last_activity) < timeout
            || self
                .last_attempt
                .is_some_and(|at| now.saturating_duration_since(at) < DISCONNECT_RETRY)
        {
            return false;
        }
        self.last_attempt = Some(now);
        true
    }

    fn disconnect(&self, timeout: Duration) {
        let address = self.address.clone();
        if let Err(error) = std::thread::Builder::new().name("ps3-disconnect".into()).spawn(move || {
            match Command::new("bluetoothctl")
                .args(["--timeout", "5", "disconnect", &address])
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .status()
            {
                Ok(status) if status.success() => tracing::warn!(%address, idle_minutes = timeout.as_secs() / 60, "PS3 controller disconnected after idle timeout"),
                Ok(status) => tracing::warn!(%address, %status, "could not disconnect idle PS3 controller"),
                Err(error) => tracing::warn!(%address, %error, "could not run bluetoothctl"),
            }
        }) {
            tracing::warn!(address = %self.address, %error, "could not start PS3 disconnect worker");
        }
    }
}

fn valid_bluetooth_address(address: &str) -> bool {
    address.len() == 17
        && address.bytes().enumerate().all(|(index, byte)| {
            if index % 3 == 2 {
                byte == b':'
            } else {
                byte.is_ascii_hexdigit()
            }
        })
}

impl Ps3Idle {
    pub(crate) fn event(&mut self, event: Event, now: Instant, deadzone: f64) {
        let active = match event.event {
            EventType::Connected | EventType::Disconnected => {
                self.devices.remove(&event.id);
                return;
            }
            EventType::ButtonPressed(..) | EventType::ButtonReleased(..) => true,
            EventType::ButtonChanged(Button::LeftTrigger2 | Button::RightTrigger2, value, _) => {
                value > TRIGGER_DEADZONE
            }
            EventType::AxisChanged(axis, value, _) => {
                STICKS.contains(&axis) && f64::from(value.abs()) > deadzone
            }
            _ => false,
        };
        if active && let Some(Some(device)) = self.devices.get_mut(&event.id) {
            device.record_activity(now);
        }
    }

    pub(crate) fn poll(
        &mut self,
        gilrs: &Gilrs,
        now: Instant,
        deadzone: f64,
        active: Option<GamepadId>,
        timeout: Duration,
    ) {
        for (id, pad) in gilrs.gamepads() {
            let Some(device) = self
                .devices
                .entry(id)
                .or_insert_with(|| Connection::open(&pad, now))
            else {
                continue;
            };
            let held = active == Some(id)
                || STICKS
                    .iter()
                    .any(|axis| f64::from(pad.value(*axis).abs()) > deadzone)
                || BUTTONS.iter().any(|button| pad.is_pressed(*button))
                || [Button::LeftTrigger2, Button::RightTrigger2]
                    .iter()
                    .any(|button| {
                        pad.button_data(*button)
                            .is_some_and(|data| data.value() > TRIGGER_DEADZONE)
                    });
            if held {
                device.record_activity(now);
            } else if device.disconnect_due(now, timeout) {
                device.disconnect(timeout);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn activity_starts_a_full_new_idle_period() {
        let timeout = Duration::from_secs(180);
        let start = Instant::now();
        let mut device = Connection {
            address: String::new(),
            last_activity: start,
            last_attempt: None,
        };
        let near_timeout = start + timeout - Duration::from_secs(1);
        assert!(!device.disconnect_due(near_timeout, timeout));
        device.record_activity(near_timeout);
        assert!(!device.disconnect_due(start + timeout, timeout));
        assert!(device.disconnect_due(near_timeout + timeout, timeout));
        assert!(!device.disconnect_due(near_timeout + timeout, timeout));
    }

    #[test]
    fn changing_the_timeout_uses_the_original_activity_time() {
        let start = Instant::now();
        let mut device = Connection {
            address: String::new(),
            last_activity: start,
            last_attempt: None,
        };
        let now = start + Duration::from_secs(120);
        assert!(!device.disconnect_due(now, Duration::from_secs(180)));
        assert!(!device.disconnect_due(now, Duration::from_secs(3600)));
        assert!(device.disconnect_due(now, Duration::from_secs(60)));
    }

    #[test]
    fn bluetooth_address_requires_six_hex_octets() {
        assert!(valid_bluetooth_address("a0:5a:5f:33:ab:83"));
        assert!(!valid_bluetooth_address("a0:5a:5f:33:ab"));
        assert!(!valid_bluetooth_address("a0-5a:5f:33:ab:83"));
        assert!(!valid_bluetooth_address("a0:5a:5f:33:ab:zz"));
    }
}
