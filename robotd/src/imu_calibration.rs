//! Paired tilt alignment; historical gyro records are validated only as provenance.

use std::path::Path;

use duck_control::Sensors;
use serde::{Deserialize, Serialize};

pub const ACTIVE_PATH: &str = "/etc/robot/imu-gyro.json";
pub const PAIR_PATH: &str = "/etc/robot/imu-pair.json";
// Compensation bound; stationarity and drift are checked during calibration.
// Keep in sync with scripts/imu-live.py.
const MAX_GYRO_BIAS_NORM_RAD_S: f64 = 0.30;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GyroCalibration {
    schema_version: u32,
    created_utc: String,
    frame: String,
    axes: [String; 3],
    units: String,
    sensor_to_trunk_wxyz: [f64; 4],
    pub gyro_bias_rad_s: [f64; 3],
    source: Source,
    verification_corrected_mean_rad_s: [f64; 3],
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Source {
    port: String,
    imu_id: u8,
}

impl GyroCalibration {
    pub fn load(path: &Path, port: &str, enabled: bool) -> Result<Option<Self>, String> {
        if !enabled {
            return Ok(None);
        }
        let bytes = match std::fs::read(path) {
            Ok(bytes) => bytes,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(error) => return Err(format!("reading {}: {error}", path.display())),
        };
        let calibration: Self = serde_json::from_slice(&bytes)
            .map_err(|error| format!("parsing {}: {error}", path.display()))?;
        if calibration.schema_version != 1
            || calibration.frame != "trunk"
            || calibration.axes != ["forward", "left", "up"]
            || calibration.units != "rad/s"
            || calibration.sensor_to_trunk_wxyz != [0.5; 4]
            || calibration.source.imu_id != 200
            || calibration.source.port != port
            || calibration.created_utc.is_empty()
        {
            return Err(
                "gyro calibration schema, frame, mounting or source does not match HX".into(),
            );
        }
        let bias = calibration.gyro_bias_rad_s;
        if bias.iter().any(|value| !value.is_finite())
            || bias.iter().map(|value| value * value).sum::<f64>().sqrt() > MAX_GYRO_BIAS_NORM_RAD_S
        {
            return Err(format!(
                "gyro calibration bias must be finite with norm <= {MAX_GYRO_BIAS_NORM_RAD_S:.2} rad/s"
            ));
        }
        if calibration
            .verification_corrected_mean_rad_s
            .iter()
            .any(|value| !value.is_finite() || value.abs() > 0.01)
        {
            return Err("gyro calibration independent verification is invalid".into());
        }
        Ok(Some(calibration))
    }
}

/// Tilt alignment to an independently fixed trunk reference, not a gravity offset.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct PoseCalibration {
    schema_version: u32,
    created_utc: String,
    frame: String,
    axes: [String; 3],
    sensor_to_trunk_wxyz: [f64; 4],
    reference_gravity: [f64; 3],
    gyro_created_utc: String,
    source: PairSource,
    i2c_save_command_completed: bool,
    alignments: Alignments,
    #[serde(skip)]
    selected: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct PairSource {
    port: String,
    imu_id: u8,
    i2c_bus: String,
    i2c_address: u8,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Alignments {
    id200: Alignment,
    i2c: Alignment,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Alignment {
    rotation_wxyz: [f64; 4],
    rotation_angle_deg: f64,
    measurement_gravity_mean: [f64; 3],
    verification: PoseVerification,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct PoseVerification {
    gravity_max_error_deg: f64,
    corrected_gyro_mean_rad_s: [f64; 3],
}

impl PoseCalibration {
    pub fn load(
        path: &Path,
        gyro_path: &Path,
        port: &str,
        source: duck_control::hx::ImuSource,
        enabled: bool,
    ) -> Result<Option<Self>, String> {
        if !enabled {
            return Ok(None);
        }
        if source == duck_control::hx::ImuSource::Id201 {
            tracing::info!("ID201 uses its own MCU calibration; ID200/I2C paired alignment is not applied");
            return Ok(None);
        }
        let bytes = match std::fs::read(path) {
            Ok(bytes) => bytes,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(error) => return Err(format!("reading {}: {error}", path.display())),
        };
        let mut calibration: Self = serde_json::from_slice(&bytes)
            .map_err(|error| format!("parsing {}: {error}", path.display()))?;
        if calibration.schema_version != 1
            || calibration.frame != "trunk"
            || calibration.axes != ["forward", "left", "up"]
            || calibration.sensor_to_trunk_wxyz != [0.5; 4]
            || calibration.reference_gravity != [-1.0, 0.0, 0.0]
            || calibration.source.port != port
            || calibration.source.imu_id != 200
            || calibration.source.i2c_bus != "/dev/i2c-5"
            || calibration.source.i2c_address != 0x50
            || !calibration.i2c_save_command_completed
            || calibration.created_utc.is_empty()
        {
            return Err(
                "paired IMU calibration frame, reference, source or SAVE status invalid".into(),
            );
        }
        // The two active files must be from the same run. A partial installation must
        // never quietly combine the former tilt with a newly measured bias.
        let gyro = GyroCalibration::load(gyro_path, port, true)?
            .ok_or("paired IMU calibration requires its associated gyro file")?;
        if gyro.created_utc != calibration.gyro_created_utc
            || calibration.created_utc != calibration.gyro_created_utc
        {
            return Err(
                "paired IMU and gyro calibration timestamps do not match; recalibrate the pair"
                    .into(),
            );
        }
        for alignment in [&calibration.alignments.id200, &calibration.alignments.i2c] {
            let q = alignment.rotation_wxyz;
            let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
            let degrees = 2.0 * q[0].abs().clamp(0.0, 1.0).acos().to_degrees();
            let verification = &alignment.verification;
            let gravity_norm = alignment
                .measurement_gravity_mean
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt();
            let measured = alignment.measurement_gravity_mean.map(|v| v / gravity_norm);
            let fitted = kinematics::Quat::new(q[0], q[1], q[2], q[3]).rotate(measured);
            if q.iter().any(|v| !v.is_finite())
                || (norm - 1.0).abs() > 1e-6
                || !alignment.rotation_angle_deg.is_finite()
                || !(0.0..=15.0).contains(&alignment.rotation_angle_deg)
                || (degrees - alignment.rotation_angle_deg).abs() > 1e-4
                || alignment
                    .measurement_gravity_mean
                    .iter()
                    .any(|v| !v.is_finite())
                || !(0.9..=1.01).contains(&gravity_norm)
                || q[1].abs() > 1e-6
                || fitted
                    .into_iter()
                    .zip(calibration.reference_gravity)
                    .any(|(a, b)| (a - b).abs() > 1e-5)
                || !verification.gravity_max_error_deg.is_finite()
                || !(0.0..=1.0).contains(&verification.gravity_max_error_deg)
                || verification
                    .corrected_gyro_mean_rad_s
                    .iter()
                    .any(|v| !v.is_finite() || v.abs() > 0.01)
            {
                return Err("paired IMU rotation or independent verification invalid".into());
            }
        }
        calibration.selected = source.as_str().into();
        Ok(Some(calibration))
    }

    pub fn correct(&self, mut sensors: Sensors) -> Sensors {
        let alignment = if self.selected == "id200" {
            &self.alignments.id200
        } else {
            &self.alignments.i2c
        };
        let [w, x, y, z] = alignment.rotation_wxyz;
        let rotation = kinematics::Quat::new(w, x, y, z);
        sensors.imu.gyro = rotation.rotate(sensors.imu.gyro);
        sensors.imu.gyro_unfiltered = rotation.rotate(sensors.imu.gyro_unfiltered);
        sensors.imu.gravity = rotation.rotate(sensors.imu.gravity);
        let [w, x, y, z] = sensors.imu.quat;
        sensors.imu.quat = (kinematics::Quat::new(w, x, y, z) * rotation.conjugate()).wxyz();
        sensors
    }
}
