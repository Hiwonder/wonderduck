//! Validated host backup of an accelerometer calibration.

use std::collections::BTreeMap;
use std::path::Path;

use serde::Deserialize;

use crate::io::{IoError, Result};

pub const ACTIVE_PATH: &str = "/etc/robot/imu-accel.json";
const FACES: [&str; 6] = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"];
// JSON decimal parsing and the Python writer can differ by an f64 ULP. This is
// far below the precision of the f32 coefficients sent to the MCU.
const JSON_ROUNDING_TOLERANCE: f64 = 1e-14;

#[derive(Debug, Deserialize)]
pub struct FixedAccelCalibration {
    schema_version: u16,
    sensor: String,
    model: String,
    frame: String,
    units: String,
    created_utc: String,
    source: Source,
    pub bias_g: [f32; 3],
    pub scale: [f32; 3],
    #[serde(skip)]
    recorded_bias_g: [f64; 3],
    #[serde(skip)]
    recorded_scale: [f64; 3],
    pending_faces: Vec<String>,
    measurement: BTreeMap<String, FaceStats>,
    verification: BTreeMap<String, FaceStats>,
    verification_residual_g: BTreeMap<String, f64>,
    installed_configuration: Installed,
    limits: Option<Limits>,
    accepted_residual_limit_g: Option<f64>,
}

#[derive(Deserialize)]
struct RecordedCoefficients {
    bias_g: [f64; 3],
    scale: [f64; 3],
}

#[derive(Debug, Deserialize)]
struct Source {
    port: String,
    imu_id: u8,
}

#[derive(Debug, Deserialize)]
struct Installed {
    flags: u16,
    bias_g: [f64; 3],
    scale: [f64; 3],
}

#[derive(Debug, Deserialize)]
struct FaceStats {
    samples: u64,
    accel_mean: [f64; 3],
    accel_std: [f64; 3],
    accel_peak: [f64; 3],
    accel_drift: [f64; 3],
    gyro_mean: [f64; 3],
    gyro_std: [f64; 3],
    gyro_peak: [f64; 3],
    gyro_drift: [f64; 3],
}

impl FaceStats {
    fn valid(&self, expected_axis: usize, positive: bool) -> bool {
        let vectors = [
            self.accel_mean,
            self.accel_std,
            self.accel_peak,
            self.accel_drift,
            self.gyro_mean,
            self.gyro_std,
            self.gyro_peak,
            self.gyro_drift,
        ];
        if self.samples < 20 || vectors.iter().flatten().any(|value| !value.is_finite()) {
            return false;
        }
        for (values, limit, signed) in [
            (self.accel_std, 0.01, false),
            (self.accel_peak, 0.06, false),
            (self.accel_drift, 0.01, true),
            (self.gyro_std, 0.02, false),
            (self.gyro_peak, 0.10, false),
            (self.gyro_drift, 0.01, true),
        ] {
            if values
                .iter()
                .any(|value| (!signed && *value < 0.0) || value.abs() > limit)
            {
                return false;
            }
        }
        if self
            .gyro_mean
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt()
            > 0.5
            || self.accel_mean.iter().any(|value| value.abs() >= 1.98)
        {
            return false;
        }
        // Python's max() chooses the first axis on a tie.
        let mut axis = 0;
        for index in 1..3 {
            if self.accel_mean[index].abs() > self.accel_mean[axis].abs() {
                axis = index;
            }
        }
        let other = (0..3)
            .filter(|index| *index != axis)
            .map(|index| self.accel_mean[index].abs())
            .fold(0.0_f64, f64::max);
        axis == expected_axis
            && (self.accel_mean[axis] > 0.0) == positive
            && self.accel_mean[axis].abs() >= 0.45
            && self.accel_mean[axis].abs() - other >= 0.3
    }
}

#[derive(Debug, Deserialize)]
struct Limits {
    residual_g: f64,
}

impl FixedAccelCalibration {
    pub fn load(path: &Path, port: &str) -> Result<Self> {
        let bytes = std::fs::read(path).map_err(|error| {
            IoError::Bus(format!(
                "manual accelerometer calibration required at {}: {error}",
                path.display()
            ))
        })?;
        Self::decode(&bytes, port)
    }

    pub fn decode(bytes: &[u8], port: &str) -> Result<Self> {
        let mut value: Self = serde_json::from_slice(bytes)
            .map_err(|error| IoError::Bus(format!("invalid accelerometer calibration: {error}")))?;
        let recorded: RecordedCoefficients = serde_json::from_slice(bytes).map_err(|error| {
            IoError::Bus(format!(
                "invalid accelerometer calibration coefficients: {error}"
            ))
        })?;
        value.recorded_bias_g = recorded.bias_g;
        value.recorded_scale = recorded.scale;
        // Older accepted records predate the saved limits field.
        let residual_limit = value
            .limits
            .as_ref()
            .map_or(0.065, |limits| limits.residual_g);
        let complete = |faces: &BTreeMap<String, FaceStats>| {
            faces.len() == FACES.len() && FACES.iter().all(|face| faces.contains_key(*face))
        };
        if value.schema_version != 1
            || value.sensor != "accel"
            || value.model != "QMI8658"
            || value.frame != "sensor"
            || value.units != "g"
            || value.created_utc.is_empty()
            || value.source.imu_id != 200
            || value.source.port != port
            || value
                .recorded_bias_g
                .iter()
                .any(|x| !x.is_finite() || x.abs() > 0.5)
            || value
                .recorded_scale
                .iter()
                .any(|x| !x.is_finite() || !(0.5..=1.5).contains(x))
            || !value.pending_faces.is_empty()
            || !complete(&value.measurement)
            || !complete(&value.verification)
            || value.installed_configuration.flags != 1
            || (0..3).any(|axis| {
                value.installed_configuration.bias_g[axis] as f32 != value.bias_g[axis]
                    || value.installed_configuration.scale[axis] as f32 != value.scale[axis]
                    || (value.installed_configuration.bias_g[axis] - f64::from(value.bias_g[axis]))
                        .abs()
                        > JSON_ROUNDING_TOLERANCE
                    || (value.installed_configuration.scale[axis] - f64::from(value.scale[axis]))
                        .abs()
                        > JSON_ROUNDING_TOLERANCE
            })
            || !residual_limit.is_finite()
            || residual_limit < 0.0
            || value
                .accepted_residual_limit_g
                .is_some_and(|accepted| !accepted.is_finite() || accepted != residual_limit)
            || (residual_limit > 0.065 && value.accepted_residual_limit_g != Some(residual_limit))
            || value.verification_residual_g.len() != FACES.len()
            || FACES.iter().any(|face| {
                value
                    .verification_residual_g
                    .get(*face)
                    .is_none_or(|x| !x.is_finite() || x.abs() > residual_limit)
            })
        {
            return Err(IoError::Bus(
                "accelerometer calibration source, coefficients or accepted verification mismatch"
                    .into(),
            ));
        }
        // Recompute the fit and both independent windows, matching the calibration tool.
        for axis in 0..3 {
            let positive = &value.measurement[FACES[2 * axis]].accel_mean;
            let negative = &value.measurement[FACES[2 * axis + 1]].accel_mean;
            let difference = positive[axis] - negative[axis];
            if difference < 1.0
                || ((positive[axis] + negative[axis]) / 2.0 - value.recorded_bias_g[axis]).abs()
                    > JSON_ROUNDING_TOLERANCE
                || (2.0 / difference - value.recorded_scale[axis]).abs() > JSON_ROUNDING_TOLERANCE
            {
                return Err(IoError::Bus(
                    "accelerometer calibration does not match six-face fit".into(),
                ));
            }
        }
        for (index, face) in FACES.iter().enumerate() {
            let measurement = &value.measurement[*face];
            let verification = &value.verification[*face];
            if !measurement.valid(index / 2, index % 2 == 0)
                || !verification.valid(index / 2, index % 2 == 0)
                || (0..3).any(|axis| {
                    (measurement.accel_mean[axis] - verification.accel_mean[axis]).abs() > 0.015
                })
            {
                return Err(IoError::Bus(format!(
                    "accelerometer calibration face {face} is invalid or moved"
                )));
            }
            let mut verification_residual = 0.0;
            for stats in [measurement, verification] {
                let residual = (0..3)
                    .map(|axis| {
                        let target = if axis == index / 2 {
                            if index % 2 == 0 { 1.0 } else { -1.0 }
                        } else {
                            0.0
                        };
                        let corrected = (stats.accel_mean[axis] - value.recorded_bias_g[axis])
                            * value.recorded_scale[axis];
                        (corrected - target).powi(2)
                    })
                    .sum::<f64>()
                    .sqrt();
                if !residual.is_finite() || residual > residual_limit {
                    return Err(IoError::Bus(format!(
                        "accelerometer calibration face {face} residual exceeds its accepted limit"
                    )));
                }
                verification_residual = residual;
            }
            if (value.verification_residual_g[*face] - verification_residual).abs() > 1e-12 {
                return Err(IoError::Bus(format!(
                    "accelerometer calibration face {face} residual does not match verification"
                )));
            }
        }
        Ok(value)
    }

    pub fn wire_configuration(&self, generation: u32) -> [u8; 32] {
        let mut bytes = [0; 32];
        bytes[..4].copy_from_slice(&[2, 0, 1, 0]);
        bytes[4..8].copy_from_slice(&generation.to_le_bytes());
        for (index, value) in self.bias_g.iter().chain(self.scale.iter()).enumerate() {
            bytes[8 + index * 4..12 + index * 4].copy_from_slice(&value.to_le_bytes());
        }
        bytes
    }
}
