//! Device status derived from the primary control transaction.
//!
//! The status subscriber never sends another packet on the half-duplex motor bus.
//! Each valid reply is retained even when another device makes the control read fail.

use super::*;
use crate::io::{DeviceObservation, ImuObservation, ServoObservation};

pub(super) struct ObservationScan {
    latest: DeviceObservation,
    enabled: bool,
    individual: bool,
    next_joint: usize,
    imu_decoder: crate::imu::SflpDecoder,
}

impl ObservationScan {
    fn new() -> Self {
        Self {
            latest: DeviceObservation {
                servos: JOINT_IDS.into_iter().map(|id| ServoObservation {
                    id, ..Default::default()
                }).collect(),
                imu: ImuObservation::default(),
                bus_error: None,
            },
            enabled: false,
            individual: false,
            next_joint: 0,
            // Do not alter the control path's stateful decoder.
            imu_decoder: crate::imu::SflpDecoder::with_gyro_unit([0.5; 4], 0.035),
        }
    }
}

impl HxIo {
    pub(super) fn begin_observation(&mut self, _generation: u64) {
        // A subscription refresh cannot make startup-verified values stale.
        self.observation.get_or_insert_with(ObservationScan::new).enabled = true;
    }

    pub(super) fn end_observation(&mut self) {
        let Some(scan) = self.observation.as_mut().filter(|scan| scan.enabled) else { return };
        scan.enabled = false;
        for servo in &mut scan.latest.servos {
            let configuration = servo.configuration.take();
            let configuration_error = servo.configuration_error.take();
            *servo = ServoObservation { id: servo.id, configuration, configuration_error, ..Default::default() };
        }
        scan.latest.imu = ImuObservation::default();
    }

    pub(super) fn sample_observation(&mut self, _deadline: Instant) -> DeviceObservation {
        // Maintenance discards the cache while subscribers remain connected.
        let scan = self.observation.get_or_insert_with(ObservationScan::new);
        scan.enabled = true;
        scan.latest.clone()
    }

    pub(super) fn set_individual_observation(&mut self, active: bool) {
        let scan = self.observation.get_or_insert_with(ObservationScan::new);
        if scan.individual == active { return; }
        scan.individual = active;
        scan.next_joint = 0;
        if active {
            for servo in &mut scan.latest.servos {
                *servo = ServoObservation { id: servo.id, ..Default::default() };
            }
        }
    }

    /// A status reply is accepted only as the exact matching complete frame.
    /// Unlike the regular stream parser, this never skips damaged or extra bytes.
    fn read_status_one(&mut self, id: u8) -> Result<StatusFrame> {
        const LENGTH: u8 = 71;
        if self.status_unconfirmed {
            return Err(IoError::Bus("status reply unconfirmed; motor bus is quarantined".into()));
        }
        let request = read_request(id, 0, LENGTH)?;
        self.status_unconfirmed = true;
        // Do not discard unexpected bytes: they may belong to an earlier transaction.
        if self.port_mut()?.bytes_to_read().map_err(io_error)? != 0 {
            return Err(IoError::Bus("unexpected bytes before individual status read".into()));
        }
        self.port_mut()?.set_timeout(READ_TIMEOUT).map_err(io_error)?;
        self.port_mut()?.write_all(&request).map_err(|e| IoError::Bus(format!("status read transmission: {e}")))?;
        let started = Instant::now();
        let mut bytes = Vec::with_capacity(usize::from(LENGTH) + 6);
        let mut chunk = [0u8; 128];
        while started.elapsed() < READ_TIMEOUT {
            let left = READ_TIMEOUT.saturating_sub(started.elapsed());
            self.port_mut()?.set_timeout(left).map_err(io_error)?;
            match self.port_mut()?.read(&mut chunk) {
                Ok(0) => return Err(IoError::Bus("status serial connection closed".into())),
                Ok(count) => bytes.extend_from_slice(&chunk[..count]),
                Err(e) if matches!(e.kind(), std::io::ErrorKind::TimedOut | std::io::ErrorKind::Interrupted) => continue,
                Err(e) => return Err(IoError::Bus(format!("status serial read: {e}"))),
            }
            if (bytes.len() >= 1 && bytes[0] != 0xff)
                || (bytes.len() >= 2 && bytes[1] != 0xff)
                || (bytes.len() >= 3 && bytes[2] != id)
                || (bytes.len() >= 4 && bytes[3] != LENGTH + 2)
            {
                return Err(IoError::Bus("status frame header or length mismatch".into()));
            }
            let total = usize::from(LENGTH) + 6;
            if bytes.len() > total { return Err(IoError::Bus("extra bytes after status frame".into())); }
            if bytes.len() == total {
                if bytes[2..].iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) != 0xff
                    || bytes[5 + 5] != id
                    || self.port_mut()?.bytes_to_read().map_err(io_error)? != 0
                {
                    return Err(IoError::Bus("invalid or unexpected status frame".into()));
                }
                self.port_mut()?.set_timeout(READ_TIMEOUT).map_err(io_error)?;
                self.status_unconfirmed = false;
                return Ok(StatusFrame { id, error: bytes[4], data: bytes[5..total-1].to_vec() });
            }
        }
        Err(IoError::Bus(format!("status read ID {id} timed out; reply is unconfirmed")))
    }

    pub(super) fn sample_one_observation(&mut self, deadline: Instant) -> Result<Option<DeviceObservation>> {
        // Do not start a query when the current control tick has no useful slack.
        if deadline.saturating_duration_since(Instant::now()) < Duration::from_millis(3) {
            return Ok(None);
        }
        let scan = self.observation.get_or_insert_with(ObservationScan::new);
        scan.enabled = true;
        if !scan.individual { return Ok(Some(scan.latest.clone())); }
        let joint = scan.next_joint;
        let id = JOINT_IDS[joint];
        let result = self.read_status_one(id);
        let stamp = duck_ipc_proto::clock::monotonic_ns();
        let scan = self.observation.as_mut().expect("status scan exists");
        match result {
            Ok(frame) => {
                let b = &frame.data;
                let raw = u16::from_le_bytes([b[56], b[57]]);
                let raw = signed_magnitude(raw, 15).clamp(0, 4095) as u16;
                let speed = signed_magnitude(u16::from_le_bytes([b[58], b[59]]), 15);
                let load = signed_magnitude(u16::from_le_bytes([b[60], b[61]]), 10);
                scan.latest.servos[joint] = ServoObservation {
                    id, t_ns: Some(stamp), observed_t_ns: Some(stamp), valid: true,
                    raw: Some(raw), speed_rad_s: Some(f64::from(HX_JOINT_DIRECTIONS[joint]) * f64::from(speed) * HX_RAD_PER_SPEED_COUNT),
                    load_percent: Some(f64::from(load) / 10.0), voltage: Some(f64::from(b[62]) * 0.1),
                    temperature: Some(f64::from(b[63])), torque: Some(b[40]), packet_status: Some(frame.error),
                    register_status: Some(b[65]), moving: Some(b[66] != 0), error: None,
                    configuration: Some(duck_ipc_proto::ServoConfiguration {
                        firmware_version: [b[0], b[1]], servo_version: [b[3], b[4]],
                        position_pid: [b[21], b[23], b[22]], overtemperature_c: b[13],
                        minimum_starting_torque: u16::from_le_bytes([b[24], b[25]]),
                    }), configuration_error: None,
                };
                scan.next_joint = (joint + 1) % JOINT_IDS.len();
                Ok(Some(scan.latest.clone()))
            }
            Err(error) => {
                scan.latest.servos[joint] = ServoObservation {
                    id, observed_t_ns: Some(stamp), valid: false, error: Some(error.to_string()),
                    ..Default::default()
                };
                Err(error)
            }
        }
    }

    /// Cache only registers actually read and verified by the startup checks.
    pub(super) fn cache_startup_configuration(&mut self, blocks: &[Vec<u8>]) {
        let scan = self.observation.get_or_insert_with(ObservationScan::new);
        for (bus_index, bytes) in blocks.iter().enumerate() {
            let joint = HX_BUS_JOINT_INDICES[bus_index];
            scan.latest.servos[joint].configuration = Some(duck_ipc_proto::ServoConfiguration {
                firmware_version: [bytes[0], bytes[1]],
                servo_version: [bytes[3], bytes[4]],
                position_pid: [bytes[21], bytes[23], bytes[22]],
                overtemperature_c: bytes[13],
                minimum_starting_torque: u16::from_le_bytes([bytes[24], bytes[25]]),
            });
            scan.latest.servos[joint].configuration_error = None;
        }
    }

    pub(super) fn invalidate_observation_configuration(&mut self) {
        if let Some(scan) = &mut self.observation {
            for servo in &mut scan.latest.servos {
                servo.configuration = None;
                servo.configuration_error = Some("configuration unavailable until next startup check".into());
            }
        }
    }

    pub(super) fn set_cached_position_gain(&mut self, gain: u8) {
        if let Some(scan) = &mut self.observation {
            for servo in &mut scan.latest.servos {
                if let Some(configuration) = &mut servo.configuration {
                    configuration.position_pid[0] = gain;
                }
            }
        }
    }

    pub(super) fn begin_control_observation(&mut self) {
        let Some(scan) = self.observation.as_mut().filter(|scan| scan.enabled) else { return };
        let timestamp = duck_ipc_proto::clock::monotonic_ns();
        if !scan.individual {
            for servo in &mut scan.latest.servos {
                let configuration = servo.configuration.take();
                let configuration_error = servo.configuration_error.take();
                *servo = ServoObservation {
                    id: servo.id, observed_t_ns: Some(timestamp), configuration,
                    configuration_error, ..Default::default()
                };
            }
        }
        scan.latest.imu = ImuObservation { observed_t_ns: Some(timestamp), ..Default::default() };
    }

    pub(super) fn observe_control_frame(&mut self, frame: &StatusFrame, address: u8) {
        let selected_imu = self.imu.source() == ImuSource::Id200 && frame.id == self.imu_device_id();
        let Some(scan) = self.observation.as_mut().filter(|scan| scan.enabled) else { return };
        let timestamp = duck_ipc_proto::clock::monotonic_ns();
        let offset = usize::from(PRESENT_POSITION - address);
        if let Some(joint) = JOINT_IDS.iter().position(|id| *id == frame.id) {
            if scan.individual { return; }
            let previous = &mut scan.latest.servos[joint];
            let block = &frame.data[offset..];
            if let Err(error) = decode_position(joint, block) {
                previous.error = Some(error.to_string());
                return;
            }
            let speed = signed_magnitude(u16::from_le_bytes([block[2], block[3]]), 15);
            let load = signed_magnitude(u16::from_le_bytes([block[4], block[5]]), 10);
            let configuration = previous.configuration.take();
            let configuration_error = previous.configuration_error.take();
            *previous = ServoObservation {
                id: frame.id, t_ns: Some(timestamp), observed_t_ns: Some(timestamp), valid: true,
                raw: Some(u16::from_le_bytes([block[0], block[1]])),
                speed_rad_s: Some(f64::from(HX_JOINT_DIRECTIONS[joint]) * f64::from(speed) * HX_RAD_PER_SPEED_COUNT),
                load_percent: Some(f64::from(load) / 10.0),
                voltage: Some(f64::from(block[VOLTAGE_OFFSET]) * 0.1),
                temperature: Some(f64::from(block[TEMPERATURE_OFFSET])), torque: None,
                packet_status: Some(frame.error), register_status: Some(block[9]),
                moving: Some(block[10] != 0), error: None, configuration, configuration_error,
            };
        } else if selected_imu {
            let block: &[u8; 12] = frame.data[offset..offset + 12].try_into().expect("validated control length");
            let result = if block[11] & 0x80 != 0 {
                Err("MCU IMU not ready or invalid quaternion")
            } else {
                scan.imu_decoder.decode_packed(block)
            };
            match result {
                Ok(imu) => Self::store_control_imu(&mut scan.latest.imu, imu, timestamp),
                Err(error) => scan.latest.imu.error = Some(error.into()),
            }
        }
    }

    fn store_control_imu(observation: &mut ImuObservation, imu: crate::imu::ImuData, timestamp: u64) {
        let imu = imu.without_host_filter();
        *observation = ImuObservation {
            t_ns: Some(timestamp), observed_t_ns: Some(timestamp), valid: true,
            quaternion_wxyz: Some(imu.quat), gyro_rad_s: Some(imu.gyro),
            gravity: Some(imu.gravity), error: None,
        };
    }

    pub(super) fn finish_control_observation(&mut self, result: &Result<Sensors>) {
        let Some(scan) = self.observation.as_mut().filter(|scan| scan.enabled) else { return };
        match result {
            Ok(sensors) => {
                let timestamp = sensors.imu_timing.map_or_else(duck_ipc_proto::clock::monotonic_ns, |timing| timing.acquired_t_ns);
                Self::store_control_imu(&mut scan.latest.imu, sensors.imu, timestamp);
            }
            Err(error) => {
                let message = error.to_string();
                for servo in &mut scan.latest.servos {
                    if !servo.valid && servo.error.is_none() { servo.error = Some(message.clone()); }
                }
                if !scan.latest.imu.valid && scan.latest.imu.error.is_none() {
                    scan.latest.imu.error = Some(message);
                }
            }
        }
    }
}
