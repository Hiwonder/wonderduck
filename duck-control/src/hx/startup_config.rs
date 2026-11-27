use std::time::Duration;

use super::{ACCELERATION, HX_BUS_JOINT_IDS, HxIo, IoError, ReadFailureReason, Result, ServoRelease, drain_status_port};

const TEMPERATURE_LIMIT: u8 = 13;
const POSITION_PID: u8 = 21;
const MINIMUM_START_TORQUE: u8 = 24;
const NVS_WRITE_PROTECTION: u8 = 55;
const POSITION_PID_EXPECTED: [u8; 3] = [15, 0, 0];
const TEMPERATURE_LIMIT_EXPECTED: u8 = 120;
const CONFIG_READ_ATTEMPTS: usize = 3;
// Includes both persistent configuration and the motion registers at 41..48.
const STARTUP_BLOCK_LENGTH: u8 = 48;

struct Change {
    address: u8,
    actual: Vec<u8>,
    expected: Vec<u8>,
}

pub(super) fn servo_failure(id: u8, error: &IoError) -> String {
    let detail = match error {
        IoError::ReadFailure(failure) => {
            let mut text = String::new();
            let _ = failure.write_reason(&mut text);
            text
        }
        _ => error.to_string().chars().take(240).collect(),
    };
    format!("ID {id}: {detail}")
}

pub(super) fn startup_check_result(fixed: usize, failed: Vec<String>) -> Result<usize> {
    if failed.is_empty() {
        Ok(fixed)
    } else {
        Err(IoError::Bus(format!("HX startup servo checks failed: {}", failed.join("; "))))
    }
}

impl HxIo {
    /// A missing reply or unsafe voltage does not start a configuration attempt.
    /// Once the write budget is exhausted the joint can only recover by readback.
    pub fn check_released_servo_at_startup(&mut self, id: u8, minimum_voltage: f64, remaining: &mut u8) -> Result<usize> {
        self.confirm_startup_servo_released(id, minimum_voltage)?;
        let (count, block) = if *remaining > 0 {
            *remaining -= 1;
            self.check_one_servo_startup(id)?
        } else {
            (0, self.read_verified_startup_configuration(id)?)
        };
        self.cache_startup_configuration(id, &block);
        Ok(count)
    }

    fn read_verified_startup_configuration(&mut self, id: u8) -> Result<Vec<u8>> {
        let block = self.read_configuration(id, 0, STARTUP_BLOCK_LENGTH)?;
        Self::verify_servo_motion_register_block(id, &block[usize::from(ACCELERATION)..])?;
        if let Some(change) = servo_configuration_changes(id, &block).first() {
            return Err(IoError::Bus(format!(
                "ID {id} 寄存器 {} 读回 {:?}，预期 {:?}", change.address, change.actual, change.expected
            )));
        }
        Ok(block)
    }

    fn confirm_startup_servo_released(&mut self, id: u8, minimum_voltage: f64) -> Result<()> {
        match self.read_released_servo(id, minimum_voltage)? {
            ServoRelease::Verified => Ok(()),
            ServoRelease::Unverified(reason) => Err(IoError::Bus(reason)),
            ServoRelease::NoReply => Err(IoError::Bus("舵机无回复".into())),
        }
    }

    pub(super) fn check_servo_startup_configuration(&mut self) -> Result<usize> {
        self.invalidate_observation_configuration();
        let mut fixed = 0;
        let mut failed = Vec::new();
        for id in HX_BUS_JOINT_IDS {
            match self.check_one_servo_startup(id) {
                Ok((count, block)) => {
                    fixed += count;
                    self.cache_startup_configuration(id, &block);
                }
                Err(error) => {
                    tracing::debug!(id, %error, "HX startup servo check failed");
                    failed.push(servo_failure(id, &error));
                    // A bus whose late replies cannot be drained must not receive
                    // additional configuration writes, even for other servos.
                    if self.status_unconfirmed { break; }
                }
            }
        }
        if failed.is_empty() {
            tracing::info!(servos = HX_BUS_JOINT_IDS.len(), corrected = fixed,
                "HX startup configuration verified");
        }
        startup_check_result(fixed, failed)
    }

    fn check_one_servo_startup(&mut self, id: u8) -> Result<(usize, Vec<u8>)> {
        // One addressed read supplies both sets of parameters. If this servo
        // times out, the next request targets another ID; it cannot accept a
        // delayed reply from this servo as its own.
        let mut block = self.read_register_block(id, 0, STARTUP_BLOCK_LENGTH)?;
        let mut fixed = self.check_servo_motion_registers_of(id, &block[usize::from(ACCELERATION)..])?;
        fixed += self.correct_servo_configuration(id, &mut block)?;
        Ok((fixed, block))
    }

    fn correct_servo_configuration(&mut self, id: u8, block: &mut [u8]) -> Result<usize> {
        let changes = servo_configuration_changes(id, block);
        if changes.is_empty() { return Ok(0); }

        let lock = self.read_configuration(id, NVS_WRITE_PROTECTION, 1)?;
        let was_locked = match lock.as_slice() {
            [0] => false,
            [1] => true,
            _ => return Err(IoError::Bus(format!(
                "HX startup configuration: ID {id} has invalid write protection {lock:?}"
            ))),
        };
        let mut fixed = 0;
        let operation = (|| {
            if was_locked {
                self.write_configuration_verified(id, NVS_WRITE_PROTECTION, &[0])?;
            }
            for change in &changes {
                tracing::warn!(id, address = change.address, actual = ?change.actual,
                    expected = ?change.expected, "correcting HX startup configuration");
                self.write_configuration_verified(id, change.address, &change.expected)?;
                let start = usize::from(change.address);
                block[start..start + change.expected.len()].copy_from_slice(&change.expected);
                fixed += 1;
            }
            Ok(())
        })();
        let restore = if was_locked {
            self.write_configuration_verified(id, NVS_WRITE_PROTECTION, &[1])
        } else {
            Ok(())
        };
        match (operation, restore) {
            (Ok(()), Ok(())) => Ok(fixed),
            (Err(error), Ok(())) => Err(error),
            (Ok(()), Err(error)) => Err(IoError::Bus(format!(
                "HX startup configuration: ID {id} write protection restoration failed: {error}"
            ))),
            (Err(error), Err(restore_error)) => Err(IoError::Bus(format!(
                "HX startup configuration: ID {id} failed: {error}; write protection restoration failed: {restore_error}"
            ))),
        }
    }

    fn write_configuration_verified(&mut self, id: u8, address: u8, expected: &[u8]) -> Result<()> {
        self.sync_write_ids(address, expected.len() as u8, &[id], &[expected.to_vec()])?;
        let actual = self.read_configuration(id, address, expected.len() as u8)?;
        if actual != expected {
            return Err(IoError::Bus(format!(
                "HX startup configuration: ID {id} register {address} expected {expected:?}, got {actual:?}"
            )));
        }
        Ok(())
    }

    fn read_configuration(&mut self, id: u8, address: u8, length: u8) -> Result<Vec<u8>> {
        for attempt in 1..=CONFIG_READ_ATTEMPTS {
            match self.read_register_block(id, address, length) {
                Ok(data) => return Ok(data),
                Err(error) if matches!(&error,
                    IoError::ReadFailure(failure) if matches!(&failure.reason, ReadFailureReason::Timeout)) =>
                {
                    // A retry or lock restoration addresses the same servo; wait
                    // for late replies so a reply to another register cannot be
                    // mistaken for the new request's confirmation.
                    if let Err(drain_error) = self.port_mut().and_then(drain_status_port) {
                        self.status_unconfirmed = true;
                        return Err(IoError::Bus(format!(
                            "HX startup configuration ID {id}: cannot settle serial bus: {drain_error}"
                        )));
                    }
                    if attempt == CONFIG_READ_ATTEMPTS { return Err(error); }
                    tracing::debug!(id, address, attempt, "retrying HX configuration read after timeout");
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(error) => return Err(error),
            }
        }
        unreachable!("configuration read loop always returns")
    }
}

fn servo_configuration_changes(id: u8, block: &[u8]) -> Vec<Change> {
    let mut changes = Vec::new();
    let mut require = |address: u8, expected: &[u8]| {
        let start = usize::from(address);
        let actual = &block[start..start + expected.len()];
        if actual != expected {
            changes.push(Change { address, actual: actual.to_vec(), expected: expected.to_vec() });
        }
    };
    for (offset, expected) in POSITION_PID_EXPECTED.iter().copied().enumerate() {
        require(POSITION_PID + offset as u8, &[expected]);
    }
    if id != 34 {
        require(TEMPERATURE_LIMIT, &[TEMPERATURE_LIMIT_EXPECTED]);
        require(MINIMUM_START_TORQUE, &[0, 0]);
    }
    changes
}
