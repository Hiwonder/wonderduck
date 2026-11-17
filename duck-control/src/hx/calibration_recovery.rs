//! Restore a missing ID200 persistent calibration before starting the control loop.

use super::sensing::{PERSISTENT_ACCEL, PERSISTENT_GYRO, persistent_calibration_flags};
use super::*;
use crate::accel_calibration::FixedAccelCalibration;
use crate::gyro_calibration::FixedGyroCalibration;
use crate::gyro_gains::{ADDRESS as GAIN_ADDRESS, GainState, STATUS_LENGTH as GAIN_LENGTH};
use std::path::Path;

const PERSISTENT_GAINS: u32 = 4;
const FLASH_WRITE_TIMEOUT: Duration = Duration::from_secs(2);

/// Keep this across startup retries, including serial port reopenings.
/// A failed or uncertain write is never repeated automatically.
#[derive(Default)]
pub struct CalibrationRestoreAttempts {
    accel: bool,
    gyro: bool,
    blocked: bool,
}

impl HxIo {
    pub fn restore_missing_persistent_calibration(
        &mut self,
        port: &str,
        use_accel: bool,
        attempts: &mut CalibrationRestoreAttempts,
    ) -> Result<()> {
        self.restore_missing_persistent_calibration_from(
            port,
            use_accel,
            attempts,
            Path::new(crate::accel_calibration::ACTIVE_PATH),
            Path::new(crate::gyro_calibration::ACTIVE_PATH),
        )
    }

    fn restore_missing_persistent_calibration_from(
        &mut self,
        port: &str,
        use_accel: bool,
        attempts: &mut CalibrationRestoreAttempts,
        accel_path: &Path,
        gyro_path: &Path,
    ) -> Result<()> {
        if !matches!(self.imu.source(), ImuSource::Id200 | ImuSource::Id200Timing) {
            return Ok(());
        }
        if attempts.blocked {
            return Err(IoError::Bus("MCU calibration restore was not verified; inspect the MCU and backup before restarting robotd".into()));
        }
        let id = IMU_DXL_ID;
        if !self.gyro_bias_state_for_load_at(id)?.supported {
            return Ok(()); // Hardware fusion does not use these calibrations.
        }
        let flags = self.persistent_flags_for_recovery(id)?;
        let need_accel = use_accel && flags & PERSISTENT_ACCEL == 0;
        let need_gyro = flags & PERSISTENT_GYRO == 0;
        if !need_accel && !need_gyro {
            return Ok(());
        }
        if (need_accel && attempts.accel) || (need_gyro && attempts.gyro) {
            return Err(IoError::Bus(
                "MCU calibration is still missing after an earlier restore attempt".into(),
            ));
        }

        // Check both backups and the gain identity before writing either one. A missing
        // or invalid backup must not prevent restoring the other valid calibration.
        let mut backup_error = None;
        let accel = if need_accel {
            match FixedAccelCalibration::load(accel_path, port) {
                Ok(value) => Some(value),
                Err(error) => {
                    backup_error = Some(error);
                    None
                }
            }
        } else {
            None
        };
        let gyro = if need_gyro {
            match FixedGyroCalibration::load(gyro_path, port) {
                Ok(value) => Some(value),
                Err(error) => {
                    if backup_error.is_none() {
                        backup_error = Some(error);
                    }
                    None
                }
            }
        } else {
            None
        };
        if let Some(gyro) = &gyro {
            let state =
                GainState::decode(&self.read_register_block(id, GAIN_ADDRESS, GAIN_LENGTH)?)?;
            if !state.supported || state.busy || state.error != 0 {
                return Err(IoError::Bus(
                    "MCU gyro gain status is unavailable or busy".into(),
                ));
            }
            match &gyro.gyro_gains {
                Some(gains) if gains.device_uid != state.device_uid => {
                    return Err(IoError::Bus(
                        "gyro gain backup belongs to a different IMU".into(),
                    ));
                }
                None if state.installed => {
                    return Err(IoError::Bus(
                        "gyro backup lacks the MCU's active gain calibration".into(),
                    ));
                }
                _ => {}
            }
        }
        if let Some(accel) = &accel {
            let before = self.read_register_block(id, 16, 32)?;
            let (active, generation, bias, scale) = diagnostics::decode_accel_calibration(&before)?;
            let same = active == 1 && bias == accel.bias_g && scale == accel.scale;
            let expected_generation = if same {
                generation
            } else {
                generation
                    .checked_add(1)
                    .ok_or_else(|| IoError::Bus("MCU accelerometer generation overflow".into()))?
            };
            let desired = accel.wire_configuration(expected_generation);
            let mut params = vec![16];
            params.extend_from_slice(&accel.wire_configuration(generation));
            attempts.accel = true;
            attempts.blocked = true;
            let reply = self.flash_write(id, &params, 32);
            check_flash_ack(reply, &desired, "accelerometer")?;
            let after_flags = self.persistent_flags_for_recovery(id)?;
            if after_flags & (flags | PERSISTENT_ACCEL) != flags | PERSISTENT_ACCEL
                || self.read_register_block(id, 16, 32)? != desired
            {
                return Err(IoError::Bus(
                    "MCU persistent accelerometer calibration readback mismatch".into(),
                ));
            }
            attempts.blocked = false;
            tracing::info!(path = %accel_path.display(), "missing MCU accelerometer calibration restored from backup");
        }
        if let Some(gyro) = &gyro {
            let flags_before = self.persistent_flags_for_recovery(id)?;
            if flags_before & PERSISTENT_GYRO != 0 {
                return Err(IoError::Bus(
                    "MCU gyro calibration changed during restore".into(),
                ));
            }
            attempts.gyro = true;
            attempts.blocked = true;
            self.restore_gyro_gains(id, gyro.gyro_gains.as_ref())?;
            let desired = gyro.wire_configuration();
            let mut params = vec![122];
            params.extend_from_slice(&desired);
            let reply = self.flash_write(id, &params, 16);
            check_flash_ack(reply, &desired, "gyro")?;
            let after_flags = self.persistent_flags_for_recovery(id)?;
            let expected_gains = if gyro.gyro_gains.is_some() {
                PERSISTENT_GAINS
            } else {
                0
            };
            if after_flags & (flags_before | PERSISTENT_GYRO) != flags_before | PERSISTENT_GYRO
                || after_flags & PERSISTENT_GAINS != expected_gains
            {
                return Err(IoError::Bus(
                    "MCU persistent gyro calibration flags mismatch".into(),
                ));
            }
            let state =
                GainState::decode(&self.read_register_block(id, GAIN_ADDRESS, GAIN_LENGTH)?)?;
            if !state.matches(gyro.gyro_gains.as_ref())
                || gyro
                    .gyro_gains
                    .as_ref()
                    .is_some_and(|gains| gains.device_uid != state.device_uid)
            {
                return Err(IoError::Bus("MCU gyro gains readback mismatch".into()));
            }
            let deadline = Instant::now() + Duration::from_secs(5);
            loop {
                let state = self.gyro_bias_state_for_load_at(id)?;
                if state.loaded && state.ready && state.bias_rad_s == gyro.gyro_bias_rad_s {
                    break;
                }
                if Instant::now() >= deadline {
                    return Err(IoError::Bus(
                        "MCU restored gyro calibration readback or readiness timeout".into(),
                    ));
                }
                std::thread::sleep(Duration::from_millis(10));
            }
            attempts.blocked = false;
            tracing::info!(path = %gyro_path.display(), "missing MCU gyro calibration restored from backup");
        }
        if let Some(error) = backup_error {
            Err(error)
        } else {
            Ok(())
        }
    }

    fn persistent_flags_for_recovery(&mut self, id: u8) -> Result<u32> {
        let data = self.read_register_block(id, 115, 16).map_err(|error| {
            IoError::Bus(format!(
                "MCU persistent calibration interface required: {error}"
            ))
        })?;
        persistent_calibration_flags(&data)
    }

    fn flash_write(&mut self, id: u8, params: &[u8], length: u8) -> Result<Vec<StatusFrame>> {
        let tx = packet(id, WRITE, params)?;
        self.send_instruction(&tx)?;
        self.receive(
            &tx,
            &[id],
            length,
            Instant::now() + FLASH_WRITE_TIMEOUT,
            false,
            None,
        )
    }
}

fn check_flash_ack(reply: Result<Vec<StatusFrame>>, expected: &[u8], label: &str) -> Result<()> {
    match reply {
        Ok(frames) if frames.len() == 1 && frames[0].data == expected => Ok(()),
        Ok(_) => Err(IoError::Bus(format!(
            "MCU {label} calibration acknowledgement mismatch"
        ))),
        Err(IoError::ReadFailure(failure))
            if matches!(failure.reason, ReadFailureReason::Timeout) =>
        {
            tracing::warn!(%label, "MCU calibration acknowledgement timed out; checking persistent readback");
            Ok(())
        }
        Err(error) => Err(error),
    }
}
