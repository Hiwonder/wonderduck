//! Sensitivity parameters from a successful manual calibration.

use serde::Deserialize;
use crate::io::{IoError, Result};

pub const ADDRESS: u8 = 117;
pub const STATUS_LENGTH: u8 = 40;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GyroGains {
    schema_version: u16,
    pub device_uid: [u8; 6],
    pub gains: [u16; 3],
    cod_status: u8,
    created_utc: String,
}

impl GyroGains {
    pub fn validate(&self) -> Result<()> {
        if self.schema_version != 1 || self.cod_status != 0 || self.created_utc.is_empty()
            || self.device_uid == [0; 6] || self.device_uid == [255; 6] {
            return Err(IoError::Bus("gyro gain calibration is invalid or not successful".into()));
        }
        Ok(())
    }

    pub fn wire_configuration(&self, device_uid: [u8; 6]) -> [u8; 20] {
        let mut bytes = [0; 20];
        bytes[..4].copy_from_slice(&[1, 0, 2, 0]);
        bytes[4..10].copy_from_slice(&device_uid);
        for (index, gain) in self.gains.iter().enumerate() {
            bytes[10 + index*2..12 + index*2].copy_from_slice(&gain.to_le_bytes());
        }
        bytes
    }
}

#[derive(Debug)]
pub struct GainState {
    pub supported: bool,
    pub installed: bool,
    pub busy: bool,
    pub phase: u8,
    pub error: u32,
    pub device_uid: [u8; 6],
    pub gains: [u16; 3],
    pub generation: u32,
}

impl GainState {
    pub fn decode(bytes: &[u8]) -> Result<Self> {
        if bytes.len() != usize::from(STATUS_LENGTH) || &bytes[..8] != b"GYGC\x01\x00\x28\x00"
            || bytes[8] & !15 != 0 || bytes[9] != 0 || bytes[10] > 4 || bytes[36..] != [0; 4] {
            return Err(IoError::Bus("invalid gyro gain status header".into()));
        }
        let state = Self {
            supported: bytes[8] & 1 != 0, installed: bytes[8] & 2 != 0,
            busy: bytes[8] & 4 != 0, phase: bytes[10],
            error: u32::from_le_bytes(bytes[12..16].try_into().unwrap()),
            device_uid: bytes[16..22].try_into().unwrap(),
            gains: std::array::from_fn(|i| u16::from_le_bytes(bytes[22+i*2..24+i*2].try_into().unwrap())),
            generation: u32::from_le_bytes(bytes[28..32].try_into().unwrap()),
        };
        if state.error > 5 || (state.error != 0) != (bytes[8] & 8 != 0)
            || (state.phase == 4) != (state.error != 0) || state.busy != matches!(state.phase, 1 | 2)
            || (state.supported && (state.device_uid == [0; 6] || state.device_uid == [255; 6])) {
            return Err(IoError::Bus("inconsistent gyro gain state".into()));
        }
        Ok(state)
    }

    pub fn matches(&self, calibration: Option<&GyroGains>) -> bool {
        self.supported && !self.busy && self.error == 0 && match calibration {
            Some(value) => self.installed && self.gains == value.gains,
            None => !self.installed,
        }
    }
}
