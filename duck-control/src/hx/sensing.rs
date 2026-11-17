//! One joint broadcast with optional same-snapshot IMU timing.

use super::*;

pub(super) const PERSISTENT_ACCEL: u32 = 1;
pub(super) const PERSISTENT_GYRO: u32 = 2;

pub(super) fn persistent_calibration_flags(data: &[u8]) -> Result<u32> {
    if data.len() != 16 || &data[..8] != b"CALP\x01\x00\x10\x00" {
        return Err(IoError::Bus("invalid MCU persistent calibration status".into()));
    }
    let flags = u32::from_le_bytes(data[8..12].try_into().unwrap());
    let generation = u32::from_le_bytes(data[12..16].try_into().unwrap());
    if flags & !15 != 0 || (flags & 4 != 0 && flags & PERSISTENT_GYRO == 0)
        || (flags & 7 != 0 && generation == 0)
    {
        return Err(IoError::Bus("invalid MCU persistent calibration status flags".into()));
    }
    if flags & 8 != 0 {
        return Err(IoError::Bus("MCU persistent calibration storage is damaged; recalibrate or explicitly restore a backup".into()));
    }
    Ok(flags)
}

impl HxIo {
    /// Query the ID200 firmware without affecting calibration or motor state.
    pub fn imu_firmware_version(&mut self) -> Result<String> {
        let data = self.read_register_block(IMU_DXL_ID, 119, 64)?;
        if data.len() != 64 || &data[..8] != b"IMUV\x01\x00\x40\x00" || data[16] != IMU_DXL_ID {
            return Err(IoError::Bus("invalid ID200 firmware information".into()));
        }
        Ok(format!("{}.{}.{}", data[8], data[9], data[10]))
    }

    fn require_persistent_calibration(&mut self, id: u8, required: u32) -> Result<()> {
        let data = self.read_register_block(id, 115, 16).map_err(|error| IoError::Bus(format!(
            "MCU persistent calibration interface required: {error}")))?;
        if persistent_calibration_flags(&data)? & required == 0 {
            return Err(IoError::Bus("MCU has no saved calibration; calibrate or explicitly restore a backup".into()));
        }
        Ok(())
    }

    pub(super) fn read_sensors(&mut self, deadline: Option<Instant>) -> Result<Sensors> {
        self.prepare_recovered_imu()?;
        self.begin_control_observation();
        let result = self.read_sensors_inner(deadline);
        self.finish_control_observation(&result);
        if result.is_err() {
            self.imu.invalidate_host();
        }
        result
    }

    fn read_sensors_inner(&mut self, deadline: Option<Instant>) -> Result<Sensors> {
        let timed = self.imu.source().timed();
        let host = self.imu.source() == ImuSource::Id200Host;
        let mut host_imu = if host && !self.imu.timing_last() {
            Some(self.read_host_imu(deadline)?)
        } else { None };
        let mut timed_imu = if timed && !self.imu.timing_last() {
            Some(self.read_timed_imu(deadline)?)
        } else {
            None
        };
        self.imu.start()?;
        self.imu.begin_control_read();
        if let Some(reference) = &self.reference_imu { reference.begin_control_read(); }
        let (address, length) = if self.record_feedback {
            (RECORD_FEEDBACK_ADDRESS, RECORD_FEEDBACK_LENGTH)
        } else {
            (PRESENT_POSITION, PRESENT_POSITION_LEN)
        };
        let mut feedback = self.record_feedback.then(|| ServoFeedback {
            read_start_t_ns: duck_ipc_proto::clock::monotonic_ns(),
            previous_host_goal: self.last_sent_goals,
            ..ServoFeedback::default()
        });
        let blocks = self.read_control_blocks_with_cycle_deadline(address, length, deadline)?;
        self.imu.end_control_read();
        if let Some(reference) = &self.reference_imu { reference.end_control_read(); }
        let broadcast_end_ns = duck_ipc_proto::clock::monotonic_ns();
        let timed_reference = if self.reference_imu.as_ref().is_some_and(|r| r.source().timed()) {
            Some(self.capture_timed_reference(deadline))
        } else { None };
        if timed && self.imu.timing_last() {
            timed_imu = Some(self.read_timed_imu(deadline)?);
        }
        if host && self.imu.timing_last() {
            host_imu = Some(self.read_host_imu(deadline)?);
        }
        let acquired_t_ns = broadcast_end_ns;
        if let Some(feedback) = &mut feedback {
            feedback.read_end_t_ns = broadcast_end_ns;
        }
        let expected_blocks = self.control_ids().len();
        if blocks.len() != expected_blocks {
            return Err(IoError::ShortRead {
                what: "HX control blocks",
                expected: expected_blocks,
                got: blocks.len(),
            });
        }
        let mut sensors = Sensors::default();
        sensors.positions = DEFAULT_POSITION;
        let mut servo_status = [duck_ipc_proto::ServoStatus::default(); NUM_JOINTS];
        let mut temps_c = [0.0; NUM_JOINTS];
        let mut volts = [0.0; NUM_JOINTS];
        let mut voltage_sum = 0.0;
        let mut voltage_count = 0u32;
        for (bus_index, block) in blocks.iter().take(HX_BUS_JOINT_IDS.len()).enumerate() {
            let joint = HX_BUS_JOINT_INDICES[bus_index];
            if block.len() != usize::from(length) {
                return Err(IoError::ShortRead {
                    what: "HX joint block",
                    expected: usize::from(length),
                    got: block.len(),
                });
            }
            if let Some(feedback) = &mut feedback {
                decode_record_feedback(feedback, joint, block)?;
            }
            let block = &block[usize::from(PRESENT_POSITION - address)..];
            sensors.positions[joint] = decode_position(joint, block)?;
            let velocity = signed_magnitude(u16::from_le_bytes([block[2], block[3]]), 15);
            sensors.velocities[joint] = f64::from(HX_JOINT_DIRECTIONS[joint])
                * f64::from(velocity)
                * HX_RAD_PER_SPEED_COUNT;
            // HX 0x3C is a signed-magnitude 0.001 duty-cycle load, not a current sensor.
            // Keep the shared field meaningful as a load proxy; the slow contract only
            // publishes voltage and temperature, so the HX current register is not exposed here.
            let load = signed_magnitude(u16::from_le_bytes([block[4], block[5]]), 10);
            sensors.currents_ma[joint] = f64::from(load.unsigned_abs()) * 1000.0 / 1000.0;
            temps_c[joint] = f64::from(block[TEMPERATURE_OFFSET]);
            volts[joint] = f64::from(block[VOLTAGE_OFFSET]) * 0.1;
            servo_status[joint] = duck_ipc_proto::ServoStatus {
                raw: u16::from_le_bytes([block[0], block[1]]),
                load_percent: f64::from(load) / 10.0,
                voltage: volts[joint],
                temperature: temps_c[joint],
                packet_status: self.diagnostics.servo_alarm(joint),
                register_status: block[9],
                moving: block[10] != 0,
            };
            if block[VOLTAGE_OFFSET] > 0 {
                voltage_sum += f64::from(block[VOLTAGE_OFFSET]) * 0.1;
                voltage_count += 1;
            }
        }
        sensors.servo_status = Some(servo_status);
        if voltage_count as usize == HX_BUS_JOINT_IDS.len() {
            sensors.min_servo_voltage = volts.iter().copied().filter(|v| *v > 0.0).reduce(f64::min);
        }
        self.slow = (voltage_count > 0).then(|| SlowSensors {
            volts: voltage_sum / f64::from(voltage_count),
            temps_c,
        });
        self.diagnostics.observe_voltages(volts);
        let imu_block = if let Some(packet) = &timed_imu {
            if self.record_feedback {
                sensors.acceleration = Some(self.imu.record_acceleration(&packet.recording_block)?);
            }
            Some(&packet.recording_block[16..])
        } else if self.imu.source() == ImuSource::Id200 {
            if self.record_feedback {
                sensors.acceleration = Some(
                    self.imu
                        .record_acceleration(&blocks[HX_BUS_JOINT_IDS.len()])?,
                );
            }
            let offset = usize::from(PRESENT_POSITION - address);
            Some(&blocks[HX_BUS_JOINT_IDS.len()][offset..])
        } else {
            None
        };
        // The packet's reserved high bit explicitly marks unknown orientation.
        if imu_block.is_some_and(|block| block[11] & 0x80 != 0) {
            return Err(IoError::Bus("MCU IMU not ready or invalid quaternion".into()));
        }
        let (imu, mut timing, i2c_acceleration_g) = if let Some(output) = host_imu {
            let consumed_t_ns = duck_ipc_proto::clock::monotonic_ns();
            sensors.acceleration = Some(output.acceleration);
            (output.imu, crate::io::ImuSampleTiming {
                source: crate::io::ImuTimingSource::Id200Host,
                sequence: u64::from(output.timing.newest_sequence),
                acquired_t_ns: output.acquired_t_ns,
                consumed_t_ns,
                age_us: consumed_t_ns.saturating_sub(output.acquired_t_ns) / 1000,
                payload_unchanged_run: 0,
                mcu: None,
                host: Some(output.timing),
                i2c: None,
            }, None)
        } else {
            self.imu.sample(imu_block, timed_imu.as_ref()
                .map_or(acquired_t_ns, |p| p.timing.host_reply_end_ns))?
        };
        if let Some(packet) = timed_imu {
            timing.mcu = Some(packet.timing);
        }
        sensors.imu = imu;
        sensors.imu_timing = Some(timing);
        if let Some(servo) = feedback {
            let reference_imu = if let Some(reference) = timed_reference {
                Some(reference)
            } else { match self.reference_imu.as_ref().map(ImuReader::source) {
                Some(ImuSource::I2c) => Some(self.capture_i2c_reference()),
                Some(_) => Some(self.capture_reference_imu(
                    &blocks[HX_BUS_JOINT_IDS.len()],
                    servo.read_start_t_ns,
                    servo.read_end_t_ns,
                )?),
                None => None,
            }};
            sensors.feedback = Some(ReadFeedback { servo, imu: timing, i2c_acceleration_g, reference_imu });
        }
        Ok(sensors)
    }

    /// Verify the MCU's own persistent calibration; host files are only backups.
    pub fn load_fixed_accel_calibration(&mut self, port: &str) -> Result<()> {
        self.reject_host_calibration()?;
        self.load_fixed_accel_calibration_at(port, self.imu_device_id())
    }

    pub(super) fn load_fixed_accel_calibration_at(&mut self, _port: &str, id: u8) -> Result<()> {
        let state = self.gyro_bias_state_for_load_at(id)?;
        if !state.supported {
            if !state.ready { return Err(IoError::Bus("MCU fusion is not ready".into())); }
            tracing::info!("MCU hardware fusion; external accelerometer calibration not required");
            return Ok(());
        }
        if id != IMU_DXL_ID {
            return Err(IoError::Bus("host accelerometer calibration belongs to ID200".into()));
        }
        self.require_persistent_calibration(id, PERSISTENT_ACCEL)?;
        let (flags, _, _, _) = diagnostics::decode_accel_calibration(&self.read_register_block(id, 16, 32)?)?;
        if flags != 1 { return Err(IoError::Bus("MCU saved accelerometer calibration is not active".into())); }
        Ok(())
    }

    pub fn configure_fixed_accel_calibration(
        &mut self,
        calibration: &crate::accel_calibration::FixedAccelCalibration,
    ) -> Result<()> {
        self.reject_host_calibration()?;
        self.configure_fixed_accel_calibration_at(calibration, self.imu_device_id())
    }

    pub(super) fn configure_fixed_accel_calibration_at(
        &mut self, calibration: &crate::accel_calibration::FixedAccelCalibration, id: u8,
    ) -> Result<()> {
        if id != IMU_DXL_ID {
            return Err(IoError::Bus("accelerometer calibration file is bound to ID200".into()));
        }
        let before = self.read_register_block(id, 16, 32)?;
        let (flags, generation, bias, scale) = diagnostics::decode_accel_calibration(&before)?;
        if flags == 1 && bias == calibration.bias_g && scale == calibration.scale {
            return Ok(());
        }
        let next_generation = generation.checked_add(1)
            .ok_or_else(|| IoError::Bus("MCU accelerometer RAM generation overflow".into()))?;
        let mut params = vec![16];
        params.extend_from_slice(&calibration.wire_configuration(generation));
        let frames = self.exchange(packet(id, WRITE, &params)?, &[id], 32)?;
        let Some(frame) = frames.first().filter(|_| frames.len() == 1) else {
            return Err(IoError::Bus("MCU accelerometer calibration acknowledgement missing".into()));
        };
        let (flags, actual_generation, bias, scale) = diagnostics::decode_accel_calibration(&frame.data)?;
        if flags != 1 || actual_generation != next_generation
            || bias != calibration.bias_g || scale != calibration.scale
            || self.read_register_block(id, 16, 32)? != frame.data
        {
            return Err(IoError::Bus("MCU RAM accelerometer calibration readback mismatch".into()));
        }
        tracing::info!(?bias, ?scale, "saved MCU RAM accelerometer calibration loaded");
        Ok(())
    }

    pub(super) fn wait_for_imu_ready_at(&mut self, id: u8) -> Result<()> {
        let deadline = Instant::now() + Duration::from_millis(500);
        loop {
            let state = self.gyro_bias_state_for_load_at(id)?;
            if state.ready { return Ok(()); }
            if Instant::now() >= deadline {
                return Err(IoError::Bus("MCU fusion readiness timed out after accelerometer calibration".into()));
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    /// Wait for the MCU to restore its own saved gain and bias.
    pub fn load_fixed_gyro_calibration(&mut self, port: &str) -> Result<()> {
        self.reject_host_calibration()?;
        self.load_fixed_gyro_calibration_at(port, self.imu_device_id())
    }

    pub(super) fn load_fixed_gyro_calibration_at(&mut self, _port: &str, id: u8) -> Result<()> {
        let state = self.gyro_bias_state_for_load_at(id)?;
        if !state.supported {
            if !state.ready { return Err(IoError::Bus("MCU fusion is not ready".into())); }
            tracing::info!("MCU hardware fusion; external fixed gyro calibration not required");
            return Ok(());
        }
        if id != IMU_DXL_ID {
            return Err(IoError::Bus("host fixed gyro calibration belongs to ID200; it cannot be copied to another IMU".into()));
        }
        self.require_persistent_calibration(id, PERSISTENT_GYRO)?;
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let state = self.gyro_bias_state_for_load_at(id)?;
            if state.loaded && state.ready { return Ok(()); }
            if Instant::now() >= deadline {
                return Err(IoError::Bus("MCU saved gyro calibration did not become ready".into()));
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    pub fn configure_fixed_gyro_calibration(
        &mut self,
        calibration: &crate::gyro_calibration::FixedGyroCalibration,
    ) -> Result<()> {
        self.reject_host_calibration()?;
        self.configure_fixed_gyro_calibration_at(calibration, self.imu_device_id())
    }

    fn configure_fixed_gyro_calibration_at(
        &mut self, calibration: &crate::gyro_calibration::FixedGyroCalibration, id: u8,
    ) -> Result<()> {
        if id != IMU_DXL_ID {
            return Err(IoError::Bus("fixed gyro calibration file is bound to ID200".into()));
        }
        let before = self.gyro_bias_state_for_load_at(id)?;
        if !before.supported {
            return Err(IoError::Bus("MCU hardware fusion does not accept external fixed gyro calibration".into()));
        }
        self.restore_gyro_gains(id, calibration.gyro_gains.as_ref())?;
        let before = self.gyro_bias_state_for_load_at(id)?;
        let desired = calibration.wire_configuration();
        if !before.loaded || before.bias_rad_s != calibration.gyro_bias_rad_s {
            let mut params = vec![122];
            params.extend_from_slice(&desired);
            let frames = self.exchange(packet(id, WRITE, &params)?, &[id], 16)?;
            if frames.len() != 1 || frames[0].data != desired {
                return Err(IoError::Bus("fixed gyro calibration acknowledgement mismatch".into()));
            }
        } else if before.ready {
            return Ok(());
        }
        let deadline = Instant::now() + Duration::from_millis(500);
        loop {
            let state = self.gyro_bias_state_for_load_at(id)?;
            if state.loaded && state.bias_rad_s == calibration.gyro_bias_rad_s && state.ready {
                tracing::info!(bias = ?state.bias_rad_s, "saved fixed MCU gyro calibration loaded");
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(IoError::Bus("fixed MCU gyro calibration readback or readiness timeout".into()));
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    pub(super) fn gyro_bias_state(&mut self) -> Result<crate::imu_timing::GyroBiasState> {
        self.gyro_bias_state_at(self.imu_device_id())
    }

    pub(super) fn gyro_bias_state_at(&mut self, id: u8) -> Result<crate::imu_timing::GyroBiasState> {
        let data = self.read_register_block(id, 122, 48)
            .map_err(|e| IoError::Bus(format!("MCU gyro capability status required: {e}")))?;
        crate::imu_timing::decode_gyro_bias(&data)
    }

    pub(super) fn gyro_bias_state_for_load_at(&mut self, id: u8) -> Result<crate::imu_timing::GyroBiasState> {
        let data = self.read_register_block(id, 122, 48)
            .map_err(|e| IoError::Bus(format!("MCU gyro capability status required: {e}")))?;
        crate::imu_timing::decode_gyro_bias_for_load(&data)
    }

    /// Require ready fusion; only fixed-bias firmware requires manual calibration.
    pub fn report_gyro_bias(&mut self) -> Result<()> {
        self.reject_host_calibration()?;
        if !self.imu.source().shared_bus() { return Ok(()); }
        let state = self.gyro_bias_state()?;
        if !state.ready {
            let reason = if state.supported {
                "MCU fixed gyro bias not ready: load the manually saved calibration before control"
            } else {
                "MCU hardware fusion not ready"
            };
            return Err(IoError::Bus(reason.into()));
        }
        if state.supported {
            tracing::info!(?state, "fixed MCU gyro bias status; host subtraction disabled");
        } else {
            tracing::info!(?state, "MCU hardware fusion status; host subtraction disabled");
        }
        Ok(())
    }

    pub(super) fn reject_host_calibration(&self) -> Result<()> {
        if self.imu.source() == ImuSource::Id200Host {
            return Err(IoError::Bus("id200-host uses uncalibrated raw samples; MCU calibration interfaces are disabled".into()));
        }
        Ok(())
    }

    fn read_host_imu(&mut self, deadline: Option<Instant>) -> Result<crate::imu_host::HostImuOutput> {
        use crate::imu_host::{RAW_ADDRESS, RAW_LENGTH};
        let start = duck_ipc_proto::clock::monotonic_ns();
        let request = packet(IMU_DXL_ID, READ, &[RAW_ADDRESS, RAW_LENGTH])?;
        let mut frames = match deadline {
            Some(end) => self.exchange_with_cycle_deadline(request, &[IMU_DXL_ID], RAW_LENGTH, end),
            None => self.exchange(request, &[IMU_DXL_ID], RAW_LENGTH),
        }?;
        let end = duck_ipc_proto::clock::monotonic_ns();
        let frame = frames.pop().ok_or_else(|| IoError::Bus("missing raw IMU batch reply".into()))?;
        self.imu.consume_host_batch(&frame.data, start, end)
    }

    fn read_timed_imu(&mut self, deadline: Option<Instant>) -> Result<crate::imu_timing::TimedImu> {
        let packet = self.read_timed_imu_at(self.imu_device_id(), deadline)?;
        self.imu.accept_mcu_sequence(packet.timing.fusion_sequence)?;
        Ok(packet)
    }

    pub(super) fn read_timed_imu_at(
        &mut self, id: u8, deadline: Option<Instant>,
    ) -> Result<crate::imu_timing::TimedImu> {
        use crate::imu_timing::{TIMING_ADDRESS, TIMING_LENGTH, decode_timing};
        let start = duck_ipc_proto::clock::monotonic_ns();
        // These are virtual query selectors, not a contiguous servo register range.
        let request = packet(id, READ, &[TIMING_ADDRESS, TIMING_LENGTH])?;
        let mut frames = match deadline {
            Some(end) => {
                self.exchange_with_cycle_deadline(request, &[id], TIMING_LENGTH, end)
            }
            None => self.exchange(request, &[id], TIMING_LENGTH),
        }?;
        let end = duck_ipc_proto::clock::monotonic_ns();
        let frame = frames
            .pop()
            .ok_or_else(|| IoError::Bus("missing timed IMU reply".into()))?;
        decode_timing(&frame.data, start, end)
    }
}
