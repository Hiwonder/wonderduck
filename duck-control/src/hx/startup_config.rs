use std::time::Duration;

use super::{HX_BUS_JOINT_IDS, HxIo, IoError, ReadFailureReason, Result};

const TEMPERATURE_LIMIT: u8 = 13;
const POSITION_PID: u8 = 21;
const MINIMUM_START_TORQUE: u8 = 24;
const NVS_WRITE_PROTECTION: u8 = 55;
const POSITION_PID_EXPECTED: [u8; 3] = [15, 0, 0];
const TEMPERATURE_LIMIT_EXPECTED: u8 = 120;
const CONFIG_READ_ATTEMPTS: usize = 3;

struct Change {
    address: u8,
    actual: Vec<u8>,
    expected: Vec<u8>,
}

impl HxIo {
    pub(super) fn check_servo_startup_configuration(&mut self) -> Result<usize> {
        self.invalidate_observation_configuration();
        // One required startup check also obtains the immutable version fields.
        // No device-status subscriber needs to send separate runtime requests.
        let mut blocks = self.sync_read_ids(&HX_BUS_JOINT_IDS, 0, 26)?;

        let mut plans = Vec::new();
        for (index, id) in HX_BUS_JOINT_IDS.iter().copied().enumerate() {
            let mut changes = Vec::new();
            for (offset, expected) in POSITION_PID_EXPECTED.iter().copied().enumerate() {
                if blocks[index][usize::from(POSITION_PID) + offset] != expected {
                    changes.push(Change {
                        address: POSITION_PID + offset as u8,
                        actual: vec![blocks[index][usize::from(POSITION_PID) + offset]],
                        expected: vec![expected],
                    });
                }
            }
            if id != 34 {
                if blocks[index][usize::from(TEMPERATURE_LIMIT)] != TEMPERATURE_LIMIT_EXPECTED {
                    changes.push(Change {
                        address: TEMPERATURE_LIMIT,
                        actual: vec![blocks[index][usize::from(TEMPERATURE_LIMIT)]],
                        expected: vec![TEMPERATURE_LIMIT_EXPECTED],
                    });
                }
                if blocks[index][usize::from(MINIMUM_START_TORQUE)..usize::from(MINIMUM_START_TORQUE) + 2] != [0, 0] {
                    changes.push(Change {
                        address: MINIMUM_START_TORQUE,
                        actual: blocks[index][usize::from(MINIMUM_START_TORQUE)..usize::from(MINIMUM_START_TORQUE) + 2].to_vec(),
                        expected: vec![0, 0],
                    });
                }
            }
            if !changes.is_empty() {
                plans.push((id, changes));
            }
        }
        if plans.is_empty() {
            self.cache_startup_configuration(&blocks);
            return Ok(0);
        }

        let ids: Vec<u8> = plans.iter().map(|(id, _)| *id).collect();
        let locks = self.sync_read_ids(&ids, NVS_WRITE_PROTECTION, 1)?;
        for (&id, lock) in ids.iter().zip(&locks) {
            if !matches!(lock.as_slice(), [0] | [1]) {
                return Err(IoError::Bus(format!(
                    "HX startup configuration: ID {id} has invalid write protection {lock:?}"
                )));
            }
        }

        let mut fixed = 0;
        for ((id, changes), lock) in plans.into_iter().zip(locks) {
            let was_locked = lock[0] == 1;
            let operation = (|| {
                if was_locked {
                    self.write_configuration_verified(id, NVS_WRITE_PROTECTION, &[0])?;
                }
                for change in &changes {
                    tracing::warn!(id, address = change.address, actual = ?change.actual,
                        expected = ?change.expected, "correcting HX startup configuration");
                    self.write_configuration_verified(id, change.address, &change.expected)?;
                    let index = HX_BUS_JOINT_IDS.iter().position(|candidate| *candidate == id).expect("known servo");
                    let start = usize::from(change.address);
                    blocks[index][start..start + change.expected.len()].copy_from_slice(&change.expected);
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
                (Ok(()), Ok(())) => {}
                (Err(error), Ok(())) => return Err(error),
                (Ok(()), Err(error)) => {
                    return Err(IoError::Bus(format!(
                        "HX startup configuration: ID {id} write protection restoration failed: {error}"
                    )));
                }
                (Err(error), Err(restore_error)) => {
                    return Err(IoError::Bus(format!(
                        "HX startup configuration: ID {id} failed: {error}; write protection restoration failed: {restore_error}"
                    )));
                }
            }
        }
        tracing::info!(
            servos = HX_BUS_JOINT_IDS.len(),
            corrected = fixed,
            "HX startup configuration verified"
        );
        self.cache_startup_configuration(&blocks);
        Ok(fixed)
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
                Err(error)
                    if attempt < CONFIG_READ_ATTEMPTS
                        && matches!(&error,
                    IoError::ReadFailure(failure) if matches!(&failure.reason, ReadFailureReason::Timeout)) =>
                {
                    tracing::warn!(
                        id,
                        address,
                        attempt,
                        "retrying HX configuration read after timeout"
                    );
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(error) => return Err(error),
            }
        }
        unreachable!("configuration read loop always returns")
    }
}
