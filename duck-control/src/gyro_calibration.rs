//! Validated host backup of a manually measured sensor-frame gyro bias.

use std::path::Path;

use serde::Deserialize;

use crate::io::{IoError, Result};

pub const ACTIVE_PATH: &str = "/etc/robot/imu-gyro-fixed.json";
// Permit only decimal-to-f64 parsing roundoff, far below MCU float32 precision.
const JSON_ROUNDING_TOLERANCE: f64 = 1e-14;

#[derive(Debug, Deserialize)]
pub struct FixedGyroCalibration {
    schema_version: u16,
    mode: String,
    frame: String,
    units: String,
    created_utc: String,
    source: Source,
    pub gyro_bias_rad_s: [f32; 3],
    #[serde(skip)]
    recorded_bias_rad_s: [f64; 3],
    pub gyro_gains: Option<crate::gyro_gains::GyroGains>,
    measurement: GyroStats,
    verification: GyroStats,
    verification_corrected_mean_rad_s: [f64; 3],
}

#[derive(Deserialize)]
struct RecordedBias {
    gyro_bias_rad_s: [f64; 3],
}

#[derive(Debug, Deserialize)]
struct Source {
    port: String,
    imu_id: u8,
}

#[derive(Debug, Deserialize)]
struct GyroStats {
    samples: u64,
    mean_rad_s: [f64; 3],
    std_rad_s: [f64; 3],
    peak_deviation_rad_s: [f64; 3],
    half_mean_drift_rad_s: [f64; 3],
    gravity_mean: [f64; 3],
    gravity_max_deviation_deg: f64,
}

impl GyroStats {
    fn valid(&self) -> bool {
        let vectors = [
            self.mean_rad_s,
            self.std_rad_s,
            self.peak_deviation_rad_s,
            self.half_mean_drift_rad_s,
            self.gravity_mean,
        ];
        let gravity_norm = self
            .gravity_mean
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        self.samples >= 2
            && vectors.iter().flatten().all(|value| value.is_finite())
            && self
                .std_rad_s
                .iter()
                .all(|value| (0.0..=0.02).contains(value))
            && self
                .peak_deviation_rad_s
                .iter()
                .all(|value| (0.0..=0.10).contains(value))
            && self
                .half_mean_drift_rad_s
                .iter()
                .all(|value| value.abs() <= 0.002)
            && gravity_norm.is_finite()
            && (0.9..=1.01 + 1e-12).contains(&gravity_norm)
            && self.gravity_max_deviation_deg.is_finite()
            && (0.0..=2.0).contains(&self.gravity_max_deviation_deg)
    }
}

impl FixedGyroCalibration {
    pub fn load(path: &Path, port: &str) -> Result<Self> {
        let bytes = std::fs::read(path).map_err(|error| {
            IoError::Bus(format!(
                "manual gyro calibration required at {}: {error}",
                path.display()
            ))
        })?;
        Self::decode(&bytes, port)
    }

    pub fn decode(bytes: &[u8], port: &str) -> Result<Self> {
        let mut value: Self = serde_json::from_slice(bytes)
            .map_err(|error| IoError::Bus(format!("invalid fixed gyro calibration: {error}")))?;
        let recorded: RecordedBias = serde_json::from_slice(bytes).map_err(|error| {
            IoError::Bus(format!("invalid fixed gyro calibration bias: {error}"))
        })?;
        value.recorded_bias_rad_s = recorded.gyro_bias_rad_s;
        if value.schema_version != 1
            || value.mode != "fixed"
            || value.frame != "sensor"
            || value.units != "rad/s"
            || value.created_utc.is_empty()
            || value.source.imu_id != 200
            || value.source.port != port
            || value.recorded_bias_rad_s.iter().any(|x| !x.is_finite())
            || value
                .recorded_bias_rad_s
                .iter()
                .map(|x| x * x)
                .sum::<f64>()
                .sqrt()
                > f64::from(0.30_f32)
            || value
                .verification_corrected_mean_rad_s
                .iter()
                .any(|x| !x.is_finite() || x.abs() > 0.002)
            || !value.measurement.valid()
            || !value.verification.valid()
            || (0..3).any(|axis| {
                value.gyro_bias_rad_s[axis] != value.measurement.mean_rad_s[axis] as f32
                    || (value.recorded_bias_rad_s[axis]
                        - f64::from(value.measurement.mean_rad_s[axis] as f32))
                    .abs()
                        > JSON_ROUNDING_TOLERANCE
            })
            || (0..3).any(|axis| {
                let residual =
                    value.verification.mean_rad_s[axis] - value.recorded_bias_rad_s[axis];
                !residual.is_finite()
                    || residual.abs() > 0.002
                    || (residual - value.verification_corrected_mean_rad_s[axis]).abs() > 1e-12
            })
        {
            return Err(IoError::Bus(
                "fixed gyro calibration frame, source, bias or independent verification mismatch"
                    .into(),
            ));
        }
        if let Some(gains) = &value.gyro_gains {
            gains.validate()?;
        }
        Ok(value)
    }

    pub fn wire_configuration(&self) -> [u8; 16] {
        let mut bytes = [0; 16];
        bytes[..4].copy_from_slice(&[1, 0, 1, 0]);
        for (index, value) in self.gyro_bias_rad_s.iter().enumerate() {
            bytes[4 + index * 4..8 + index * 4].copy_from_slice(&value.to_le_bytes());
        }
        bytes
    }
}
