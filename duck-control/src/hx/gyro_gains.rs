//! Restore saved sensitivity once, before loading its matching gyro bias.
use super::*;
use crate::gyro_gains::{ADDRESS, STATUS_LENGTH, GainState, GyroGains};

impl HxIo {
    pub(super) fn restore_gyro_gains(&mut self, id: u8, desired: Option<&GyroGains>) -> Result<()> {
        self.restore_gyro_gains_inner(id, desired, false)
    }

    /// A blank MCU may be provisioned from a valid host backup from another device.
    pub(super) fn restore_missing_gyro_gains(&mut self, id: u8, desired: Option<&GyroGains>) -> Result<()> {
        self.restore_gyro_gains_inner(id, desired, true)
    }

    fn restore_gyro_gains_inner(&mut self, id: u8, desired: Option<&GyroGains>, allow_other_device: bool) -> Result<()> {
        let mut state = GainState::decode(&self.read_register_block(id, ADDRESS, STATUS_LENGTH)?)?;
        if !state.supported || state.busy {
            return Err(IoError::Bus("gyro gain capability unavailable or operation already in progress".into()));
        }
        if let Some(calibration) = desired {
            calibration.validate()?;
            if !allow_other_device && calibration.device_uid != state.device_uid {
                return Err(IoError::Bus("gyro gain backup belongs to a different IMU".into()));
            }
        }
        if state.matches(desired) { return Ok(()); }
        let calibration = desired.ok_or_else(|| IoError::Bus(
            "factory-bias file conflicts with active gyro gains; complete joint gain/bias calibration".into()))?;
        calibration.validate()?;
        let device_uid = state.device_uid;
        let mut params = vec![ADDRESS];
        params.extend_from_slice(&calibration.wire_configuration(device_uid));
        let frames = self.exchange(packet(id, WRITE, &params)?, &[id], STATUS_LENGTH)?;
        if frames.len() != 1 { return Err(IoError::Bus("missing gyro gain acknowledgement".into())); }
        state = GainState::decode(&frames[0].data)?;
        let generation = state.generation;
        let deadline = Instant::now() + Duration::from_secs(4);
        loop {
            if state.device_uid != device_uid || state.generation != generation || state.error != 0 {
                return Err(IoError::Bus(format!("gyro gain restore failed: phase={} error={}", state.phase, state.error)));
            }
            if !state.busy {
                if state.phase == 3 && state.matches(Some(calibration)) {
                    tracing::info!("saved gyro sensitivity restored; loading its matching fixed bias");
                    return Ok(());
                }
                return Err(IoError::Bus("gyro gain restore completion mismatch".into()));
            }
            if Instant::now() >= deadline { return Err(IoError::Bus("gyro gain restore timed out".into())); }
            std::thread::sleep(Duration::from_millis(10));
            state = GainState::decode(&self.read_register_block(id, ADDRESS, STATUS_LENGTH)?)?;
        }
    }
}
