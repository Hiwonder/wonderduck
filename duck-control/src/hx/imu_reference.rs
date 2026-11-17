//! Record the other IMU without replacing the independently selected policy input.

use super::*;
use crate::io::{ImuTimingSource, ReferenceImu, ReferenceImuSample};

impl HxIo {
    /// Explicit references use timestamped bus samples and never replace policy input.
    pub fn set_reference_imu_source(&mut self, source: ImuSource) -> Result<()> {
        if source == ImuSource::Id200Host
            || source == self.imu.source()
            || source.bus_id().is_some_and(|id| self.imu.source().bus_id() == Some(id)) {
            return Err(IoError::Bus("reference IMU must be a different physical sensor, without host fusion".into()));
        }
        self.reference_source = Some(if source == ImuSource::Id200 {
            ImuSource::Id200Timing
        } else { source });
        self.configure_reference_imu();
        Ok(())
    }

    pub(super) fn configure_reference_imu(&mut self) {
        self.reference_imu = self.reference_source.filter(|_| self.record_feedback).map(|source| {
            let mut reader = ImuReader::new(source);
            if source == ImuSource::Id200 {
                reader.set_last(true).expect("shared-bus reply order");
            }
            reader
        });
    }

    pub(super) fn control_ids(&self) -> &'static [u8] {
        self.reference_imu
            .as_ref()
            .filter(|reader| reader.source().shared_bus() && !reader.source().timed())
            .unwrap_or(&self.imu)
            .control_ids()
    }

    /// I2C uses its worker; a shared-bus reference is retained as unavailable
    /// rather than issuing an optional transaction beside active control reads.
    pub fn prepare_imu_reference(&mut self) -> Result<()> {
        let Some(reader) = self.reference_imu.as_mut() else { return Ok(()) };
        if reader.source() == ImuSource::I2c {
            reader.start()?;
            tracing::info!("I2C recording reference enabled; policy source unchanged");
        } else {
            tracing::warn!("shared-bus reference IMU unavailable during control; primary IMU remains active");
        }
        Ok(())
    }

    pub(super) fn capture_reference_imu(
        &mut self,
        block: &[u8],
        read_start_t_ns: u64,
        read_end_t_ns: u64,
    ) -> Result<ReferenceImu> {
        let raw_block = block.try_into().map_err(|_| IoError::ShortRead {
            what: "reference IMU recording block",
            expected: 28,
            got: block.len(),
        })?;
        let reader = self.reference_imu.as_mut().expect("reference enabled");
        let source = reader.source().timing_source();
        let result = (|| {
            let acceleration = reader.record_acceleration(block)?;
            let (imu, timing, _) = reader.sample(Some(&block[16..]), read_end_t_ns)?;
            let imu = imu.without_host_filter();
            Ok::<_, IoError>(ReferenceImuSample {
                gyro_rad_s: imu.gyro,
                gravity_trunk: imu.gravity,
                quat_trunk_to_world: imu.quat,
                acceleration: Some(acceleration),
                i2c_acceleration_g: None,
                timing,
            })
        })();
        let (sample, error) = match result {
            Ok(sample) => (Some(sample), None),
            Err(error) => {
                tracing::warn!(%error, "invalid shared-bus recording reference; raw reply retained");
                (None, Some("invalid_bus_imu_measurement"))
            }
        };
        Ok(ReferenceImu {
            source,
            read_start_t_ns,
            read_end_t_ns,
            raw_block: Some(raw_block),
            sample,
            error,
        })
    }

    pub(super) fn capture_timed_reference(&mut self, _deadline: Option<Instant>) -> ReferenceImu {
        let source = self.reference_imu.as_ref().expect("reference enabled").source();
        let now = duck_ipc_proto::clock::monotonic_ns();
        ReferenceImu {
            source: source.timing_source(), read_start_t_ns: now, read_end_t_ns: now,
            raw_block: None, sample: None, error: Some("reference_bus_imu_unavailable_during_control"),
        }
    }

    pub(super) fn capture_i2c_reference(&mut self) -> ReferenceImu {
        let started = duck_ipc_proto::clock::monotonic_ns();
        let reader = self.reference_imu.as_mut().expect("reference enabled");
        let (sample, error) = if !reader.ready() {
            (None, Some("i2c_reference_not_ready"))
        } else {
            match reader.sample(None, 0) {
                Ok((imu, timing, acceleration)) => {
                    let imu = imu.without_host_filter();
                    (
                        Some(ReferenceImuSample {
                            gyro_rad_s: imu.gyro,
                            gravity_trunk: imu.gravity,
                            quat_trunk_to_world: imu.quat,
                            acceleration: None,
                            i2c_acceleration_g: acceleration,
                            timing,
                        }),
                        None,
                    )
                }
                // The sampler reports the detailed bus error on state changes. A missing
                // reference is recorded explicitly; it must not stall healthy ID200 control.
                Err(_) => (None, Some("i2c_reference_unavailable")),
            }
        };
        ReferenceImu {
            source: ImuTimingSource::I2c,
            read_start_t_ns: started,
            read_end_t_ns: duck_ipc_proto::clock::monotonic_ns(),
            raw_block: None,
            sample,
            error,
        }
    }
}
