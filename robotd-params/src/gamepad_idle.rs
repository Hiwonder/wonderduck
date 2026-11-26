//! Persistent Bluetooth gamepad idle settings shared by Studio and padd.

use std::io;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Settings {
    pub minutes: u32,
}

impl Default for Settings {
    fn default() -> Self {
        Self { minutes: 3 }
    }
}

impl Settings {
    pub const MIN: u32 = 1;
    pub const MAX: u32 = 60;

    pub fn valid(self) -> bool {
        (Self::MIN..=Self::MAX).contains(&self.minutes)
    }

    pub fn path(robot_config: &Path) -> PathBuf {
        robot_config.with_extension("gamepad-idle.json")
    }

    pub fn load(path: &Path) -> io::Result<Self> {
        let settings: Self = match std::fs::read(path) {
            Ok(data) => serde_json::from_slice(&data)?,
            Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Self::default()),
            Err(error) => return Err(error),
        };
        if !settings.valid() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "gamepad idle timeout outside range",
            ));
        }
        Ok(settings)
    }
}
