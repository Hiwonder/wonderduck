use super::{HX_BUS_JOINT_IDS, HxIo, IoError, NUM_JOINTS, Result};

const POSITION_P_GAIN: u8 = 21;
const NVS_WRITE_PROTECTION: u8 = 55;

impl HxIo {
    pub(super) fn write_temporary_position_p_gain(&mut self, kp: u8) -> Result<()> {
        if kp == 0 {
            return Err(IoError::Bus(
                "temporary position P gain must be nonzero".into(),
            ));
        }
        // Register 21 also names persistent configuration. Firmware applies it to RAM
        // but skips flash only while register 55 is locked. Reassert and verify the lock
        // before EVERY transition, including restoration; never unlock it on exit.
        // Any partial failure makes the cached position gains untrustworthy.
        let result = self.write_temporary_position_p_gain_inner(kp);
        if result.is_ok() {
            self.set_cached_position_gain(kp);
        } else {
            self.invalidate_observation_configuration();
        }
        result
    }

    fn write_temporary_position_p_gain_inner(&mut self, kp: u8) -> Result<()> {
        self.sync_write_ids(
            NVS_WRITE_PROTECTION,
            1,
            &HX_BUS_JOINT_IDS,
            &vec![vec![1]; NUM_JOINTS],
        )?;
        self.verify_position_register(NVS_WRITE_PROTECTION, 1)?;
        self.sync_write_ids(
            POSITION_P_GAIN,
            1,
            &HX_BUS_JOINT_IDS,
            &vec![vec![kp]; NUM_JOINTS],
        )?;
        self.verify_position_register(POSITION_P_GAIN, kp)
    }

    fn verify_position_register(&mut self, address: u8, expected: u8) -> Result<()> {
        let blocks = self.sync_read_ids(&HX_BUS_JOINT_IDS, address, 1)?;
        for (id, actual) in HX_BUS_JOINT_IDS.iter().zip(blocks) {
            if actual != [expected] {
                return Err(IoError::Bus(format!(
                    "temporary position gain: ID {id} register {address} expected {expected}, got {actual:?}"
                )));
            }
        }
        Ok(())
    }
}
