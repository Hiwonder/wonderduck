//! Device observations from control reads or an exclusive individual status scan.

use super::*;
use crate::io::{DeviceObservation, ImuObservation, ServoObservation};

const STATUS_IMU_INTERVAL: Duration = Duration::from_millis(55);
const ABSENT_SERVO_RETRY: Duration = Duration::from_secs(3);
use super::ACCESSORY_SERVO_IDS;
const OPTIONAL_SERVO_INTERVAL: Duration = Duration::from_secs(1);
// Local PTY operation translated by the UNO Q bridge, never sent on the HX wire.
const UNOQ_STATUS_READ: u8 = 0x92;

pub(super) struct ObservationScan {
    latest: DeviceObservation,
    enabled: bool,
    individual: bool,
    next_joint: usize,
    next_optional_servo: usize,
    next_optional_at: Instant,
    absent_optional_servos: [bool; ACCESSORY_SERVO_IDS.len()],
    next_imu: Instant,
    last_read_was_imu: bool,
    absent_servo_retry: [Option<Instant>; JOINT_IDS.len()],
    raw_imu_retry_at: Option<Instant>,
    imu_decoder: crate::imu::SflpDecoder,
}

impl ObservationScan {
    fn new() -> Self {
        Self {
            latest: DeviceObservation {
                servos: JOINT_IDS.into_iter().map(|id| ServoObservation {
                    id, ..Default::default()
                }).collect(),
                ..Default::default()
            },
            enabled: false,
            individual: false,
            next_joint: 0,
            next_optional_servo: 0,
            next_optional_at: Instant::now() + OPTIONAL_SERVO_INTERVAL,
            absent_optional_servos: [false; ACCESSORY_SERVO_IDS.len()],
            next_imu: Instant::now(),
            last_read_was_imu: false,
            absent_servo_retry: [None; JOINT_IDS.len()],
            raw_imu_retry_at: None,
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
        scan.latest.optional_servos.clear();
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
        scan.next_optional_servo = 0;
        scan.next_optional_at = Instant::now() + OPTIONAL_SERVO_INTERVAL;
        scan.absent_optional_servos.fill(false);
        scan.latest.optional_servos.clear();
        scan.next_imu = Instant::now();
        scan.last_read_was_imu = false;
        scan.absent_servo_retry.fill(None);
        scan.raw_imu_retry_at = None;
        if active {
            for servo in &mut scan.latest.servos {
                *servo = ServoObservation { id: servo.id, ..Default::default() };
            }
            scan.latest.imu = ImuObservation::default();
        }
    }

    /// A status reply is accepted only as the exact matching complete frame.
    /// Unlike the regular stream parser, this never skips damaged or extra bytes.
    pub(super) fn read_status_one(&mut self, id: u8, address: u8, length: u8) -> Result<Option<StatusFrame>> {
        if self.status_unconfirmed {
            return Err(IoError::Bus("status reply unconfirmed; motor bus is quarantined".into()));
        }
        let bridged = self.uses_unoq_bridge();
        let request = packet(id, if bridged { UNOQ_STATUS_READ } else { READ }, &[address, length])?;
        self.status_unconfirmed = true;
        // Do not discard unexpected bytes: they may belong to an earlier transaction.
        if self.port_mut()?.bytes_to_read().map_err(io_error)? != 0 {
            return Err(IoError::Bus("unexpected bytes before individual status read".into()));
        }
        self.port_mut()?.set_timeout(READ_TIMEOUT).map_err(io_error)?;
        self.port_mut()?.write_all(&request).map_err(|e| IoError::Bus(format!("status read transmission: {e}")))?;
        let started = Instant::now();
        let mut bytes = Vec::with_capacity(usize::from(length) + 6);
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
                || (bytes.len() >= 4 && bytes[3] != length + 2 && bytes[3] != 2)
            {
                return Err(IoError::Bus("status frame header or length mismatch".into()));
            }
            if bytes.len() < 4 { continue; }
            let error_reply = bytes[3] == 2;
            let bridge_empty = error_reply && bytes.len() >= 5 && bytes[4] == 0;
            if bridge_empty && !bridged {
                return Err(IoError::Bus("status frame has no payload or error".into()));
            }
            let total = usize::from(bytes[3]) + 4;
            if bytes.len() > total { return Err(IoError::Bus("extra bytes after status frame".into())); }
            if bytes.len() == total {
                if bytes[2..].iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) != 0xff
                    || (!error_reply && address == 0 && length == 71 && bytes[5 + 5] != id)
                    || self.port_mut()?.bytes_to_read().map_err(io_error)? != 0
                {
                    return Err(IoError::Bus("invalid or unexpected status frame".into()));
                }
                self.port_mut()?.set_timeout(READ_TIMEOUT).map_err(io_error)?;
                self.status_unconfirmed = false;
                return Ok((!bridge_empty).then(|| StatusFrame {
                    id, error: bytes[4], data: bytes[5..total-1].to_vec(),
                }));
            }
        }
        if !bytes.is_empty() {
            return Err(IoError::Bus(format!("status read ID {id} timed out with an incomplete frame")));
        }
        // A missing device must not hide every other device. While the status
        // session still exclusively owns the port, wait for a bounded quiet
        // interval and validate any late reply before starting the next read.
        let late = self.drain_timed_out_status(id, address, length, bridged)?;
        self.port_mut()?.set_timeout(READ_TIMEOUT).map_err(io_error)?;
        self.status_unconfirmed = false;
        Ok(late)
    }

    fn drain_timed_out_status(&mut self, id: u8, address: u8, length: u8, bridged: bool) -> Result<Option<StatusFrame>> {
        const QUIET: Duration = Duration::from_millis(500);
        const LIMIT: Duration = Duration::from_secs(3);
        let started = Instant::now();
        let mut last_data = started;
        let mut late = Vec::new();
        let mut chunk = [0u8; 128];
        loop {
            let now = Instant::now();
            if now.duration_since(started) >= LIMIT {
                return Err(IoError::Bus("status bus remains noisy after timeout".into()));
            }
            let available = self.port_mut()?.bytes_to_read().map_err(io_error)?;
            if available > 0 {
                let limit = usize::try_from(available).unwrap_or(usize::MAX).min(chunk.len());
                let count = self.port_mut()?.read(&mut chunk[..limit])
                    .map_err(|error| IoError::Bus(format!("status drain read: {error}")))?;
                if count == 0 {
                    return Err(IoError::Bus("status serial connection closed during drain".into()));
                }
                late.extend_from_slice(&chunk[..count]);
                last_data = Instant::now();
                if late.len() > usize::from(length) + 6
                    || (late.len() >= 1 && late[0] != 0xff)
                    || (late.len() >= 2 && late[1] != 0xff)
                    || (late.len() >= 3 && late[2] != id)
                    || (late.len() >= 4 && late[3] != length + 2 && late[3] != 2)
                {
                    return Err(IoError::Bus("unexpected frame during status drain".into()));
                }
            } else if now.duration_since(last_data) >= QUIET {
                if late.is_empty() {
                    return Ok(None);
                }
                if late.len() < 6 || late.len() != usize::from(late[3]) + 4
                    || (late[3] == 2 && late[4] == 0 && !bridged)
                    || late[2..].iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) != 0xff
                    || (address == 0 && length == 71 && late[3] != 2 && late[10] != id)
                {
                    return Err(IoError::Bus("invalid or incomplete late status frame".into()));
                }
                return Ok((late[3] != 2 || late[4] != 0).then(|| StatusFrame {
                    id, error: late[4], data: late[5..late.len() - 1].to_vec()
                }));
            } else {
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    }

    pub(super) fn uses_unoq_bridge(&self) -> bool {
        std::env::var_os("WONDERDUCK_SERVO_PORT")
            .is_some_and(|port| port == std::ffi::OsStr::new(&self.port_path))
            && std::env::var_os("WONDERDUCK_UNOQ_ROUTER").is_some()
    }

    pub(super) fn sample_one_observation(&mut self, deadline: Instant) -> Result<Option<DeviceObservation>> {
        // Do not start a query when the current control tick has no useful slack.
        if deadline.saturating_duration_since(Instant::now()) < Duration::from_millis(3) {
            return Ok(None);
        }
        let scan = self.observation.get_or_insert_with(ObservationScan::new);
        scan.enabled = true;
        if !scan.individual { return Ok(Some(scan.latest.clone())); }
        let now = Instant::now();
        let joint = (0..JOINT_IDS.len())
            .map(|offset| (scan.next_joint + offset) % JOINT_IDS.len())
            .find(|&joint| scan.absent_servo_retry[joint].is_none_or(|retry| now >= retry));
        // A timed-out IMU read can outlast its polling interval. Give an eligible
        // servo a turn before issuing another IMU request.
        if now >= scan.next_imu && (!scan.last_read_was_imu || joint.is_none()) {
            scan.next_imu = now + STATUS_IMU_INTERVAL;
            scan.last_read_was_imu = true;
            let sample = self.sample_one_imu()?;
            if sample.as_ref().is_some_and(|state| !state.imu.valid) {
                self.observation.as_mut().expect("status scan exists").next_imu =
                    Instant::now() + STATUS_IMU_INTERVAL;
            }
            return Ok(sample);
        }
        if now >= scan.next_optional_at
            && let Some(index) = (0..ACCESSORY_SERVO_IDS.len())
                .map(|offset| (scan.next_optional_servo + offset) % ACCESSORY_SERVO_IDS.len())
                .find(|&index| !scan.absent_optional_servos[index])
        {
            scan.next_optional_servo = (index + 1) % ACCESSORY_SERVO_IDS.len();
            scan.next_optional_at = now + OPTIONAL_SERVO_INTERVAL;
            scan.last_read_was_imu = false;
            return self.sample_optional_servo(index);
        }
        let Some(joint) = joint else { return Ok(None); };
        scan.last_read_was_imu = false;
        scan.next_joint = (joint + 1) % JOINT_IDS.len();
        let id = JOINT_IDS[joint];
        let result = self.read_status_one(id, 0, 71);
        let stamp = duck_ipc_proto::clock::monotonic_ns();
        let scan = self.observation.as_mut().expect("status scan exists");
        match result {
            Ok(Some(frame)) => {
                scan.absent_servo_retry[joint] = None;
                if frame.data.is_empty() {
                    scan.latest.servos[joint] = ServoObservation {
                        id, observed_t_ns: Some(stamp), valid: false,
                        error: Some(format!("舵机返回错误 0x{:02X}", frame.error)),
                        ..Default::default()
                    };
                    return Ok(Some(scan.latest.clone()));
                }
                scan.latest.servos[joint] = Self::decode_status_servo(frame, stamp, HX_JOINT_DIRECTIONS[joint]);
                Ok(Some(scan.latest.clone()))
            }
            Ok(None) => {
                let error = match scan.latest.servos[joint].error.as_deref() {
                    Some("读取超时" | "未连接") => "未连接",
                    _ => "读取超时",
                };
                scan.latest.servos[joint] = ServoObservation {
                    id, observed_t_ns: Some(stamp), valid: false,
                    error: Some(error.into()), ..Default::default()
                };
                if error == "未连接" {
                    scan.absent_servo_retry[joint] = Some(Instant::now() + ABSENT_SERVO_RETRY);
                }
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

    fn sample_optional_servo(&mut self, index: usize) -> Result<Option<DeviceObservation>> {
        let id = ACCESSORY_SERVO_IDS[index];
        let bridged = self.uses_unoq_bridge();
        let reply = self.read_status_one(id, 0, 71)?;
        let stamp = duck_ipc_proto::clock::monotonic_ns();
        let scan = self.observation.as_mut().expect("status scan exists");
        // A missing native-bus servo needs a quiet drain after its timeout.
        // Retry on the next status session so absent accessories do not slow the scan.
        scan.absent_optional_servos[index] = !bridged && reply.is_none();
        let optional = &mut scan.latest.optional_servos;
        optional.retain(|servo| servo.id != id);
        if let Some(frame) = reply.filter(|frame| !frame.data.is_empty()) {
            // Use the servo's register direction for unassigned joints.
            optional.push(Self::decode_status_servo(frame, stamp, 1));
        }
        Ok(Some(scan.latest.clone()))
    }

    fn decode_status_servo(frame: StatusFrame, stamp: u64, direction: i8) -> ServoObservation {
        let b = &frame.data;
        let raw = signed_magnitude(u16::from_le_bytes([b[56], b[57]]), 15).clamp(0, 4095) as u16;
        let speed = signed_magnitude(u16::from_le_bytes([b[58], b[59]]), 15);
        let load = signed_magnitude(u16::from_le_bytes([b[60], b[61]]), 10);
        ServoObservation {
            id: frame.id, t_ns: Some(stamp), observed_t_ns: Some(stamp), valid: true,
            raw: Some(raw), speed_rad_s: Some(f64::from(direction) * f64::from(speed) * HX_RAD_PER_SPEED_COUNT),
            load_percent: Some(f64::from(load) / 10.0), voltage: Some(f64::from(b[62]) * 0.1),
            temperature: Some(f64::from(b[63])), torque: Some(b[40]), packet_status: Some(frame.error),
            register_status: Some(b[65]), moving: Some(b[66] != 0), error: None,
            configuration: Some(duck_ipc_proto::ServoConfiguration {
                firmware_version: [b[0], b[1]], servo_version: [b[3], b[4]],
                position_pid: [b[21], b[23], b[22]], overtemperature_c: b[13],
                minimum_starting_torque: u16::from_le_bytes([b[24], b[25]]),
            }), configuration_error: None,
        }
    }

    fn sample_one_imu(&mut self) -> Result<Option<DeviceObservation>> {
        let source = self.imu.source();
        if source == ImuSource::I2c {
            let result = self.imu.start().and_then(|()| self.imu.sample(None, duck_ipc_proto::clock::monotonic_ns()));
            let stamp = duck_ipc_proto::clock::monotonic_ns();
            let scan = self.observation.as_mut().expect("status scan exists");
            match result {
                Ok((imu, timing, _)) => Self::store_control_imu(&mut scan.latest.imu, imu, timing.acquired_t_ns),
                Err(error) => scan.latest.imu = ImuObservation {
                    observed_t_ns: Some(stamp), error: Some(error.to_string()), ..Default::default()
                },
            }
            return Ok(Some(scan.latest.clone()));
        }
        if source == ImuSource::Id200Host {
            let scan = self.observation.as_mut().expect("status scan exists");
            scan.latest.imu = ImuObservation {
                observed_t_ns: Some(duck_ipc_proto::clock::monotonic_ns()),
                error: Some("主机融合 IMU 暂不支持独立状态读取".into()),
                ..Default::default()
            };
            return Ok(Some(scan.latest.clone()));
        }
        let id = self.imu_device_id();
        let result = self.read_status_one(id, PRESENT_POSITION, PRESENT_POSITION_LEN);
        let stamp = duck_ipc_proto::clock::monotonic_ns();
        let raw_sample = if result.as_ref().ok().and_then(|frame| frame.as_ref())
            .is_some_and(|frame| frame.error == 0 && frame.data.len() == 12 && frame.data[11] & 0x80 == 0)
        {
            self.sample_status_raw_imu(id)?
        } else { None };
        let scan = self.observation.as_mut().expect("status scan exists");
        match result {
            Ok(Some(frame)) => {
                let imu = if frame.data.is_empty() {
                    Err(format!("IMU 返回错误 0x{:02X}", frame.error))
                } else if frame.error != 0 {
                    Err(format!("IMU 状态错误 0x{:02X}", frame.error))
                } else if frame.data[11] & 0x80 != 0 {
                    Err("MCU IMU not ready or invalid quaternion".to_owned())
                } else {
                    let block: &[u8; 12] = frame.data.as_slice().try_into().expect("validated IMU length");
                    scan.imu_decoder.decode_packed(block).map_err(str::to_owned)
                };
                match imu {
                    Ok(imu) => {
                        Self::store_control_imu(&mut scan.latest.imu, imu, stamp);
                        scan.latest.imu.raw_sample = raw_sample;
                    }
                    Err(error) => scan.latest.imu = ImuObservation {
                        observed_t_ns: Some(stamp), error: Some(error), ..Default::default()
                    },
                }
                Ok(Some(scan.latest.clone()))
            }
            Ok(None) => {
                scan.latest.imu = ImuObservation {
                    observed_t_ns: Some(stamp), error: Some("读取超时".into()), ..Default::default()
                };
                Ok(Some(scan.latest.clone()))
            }
            Err(error) => {
                scan.latest.imu = ImuObservation {
                    observed_t_ns: Some(stamp), error: Some(error.to_string()), ..Default::default()
                };
                Err(error)
            }
        }
    }

    fn sample_status_raw_imu(&mut self, id: u8) -> Result<Option<duck_ipc_proto::RawImuObservation>> {
        if self.observation.as_ref().and_then(|scan| scan.raw_imu_retry_at)
            .is_some_and(|retry_at| Instant::now() < retry_at) { return Ok(None); }
        let frame = self.read_status_one(id, 68, 48)?;
        let Some(frame) = frame.filter(|frame| frame.error == 0 && frame.data.len() == 48) else {
            self.defer_status_raw_imu();
            return Ok(None);
        };
        let b = &frame.data;
        if b[0] != 1 || b[1] != 1 || b[3] != 0 || b[2] & !15 != 0 {
            self.defer_status_raw_imu();
            return Ok(None);
        }
        let age_us = u32::from_le_bytes(b[8..12].try_into().unwrap());
        if b[2] & 10 != 0 || age_us > 100_000 { return Ok(None); }
        let timestamp = duck_ipc_proto::clock::monotonic_ns().saturating_sub(u64::from(age_us) * 1000);
        let vector = |offset| std::array::from_fn(|i| f64::from(f32::from_le_bytes(
            b[offset + 4*i..offset + 4*i + 4].try_into().unwrap())));
        let raw_g: [f64; 3] = vector(12);
        let corrected_g: [f64; 3] = vector(24);
        let gyro_rad_s: [f64; 3] = vector(36);
        if raw_g.iter().chain(corrected_g.iter()).chain(gyro_rad_s.iter()).any(|value| !value.is_finite())
            || raw_g.iter().any(|value| value.abs() >= 1.98) { return Ok(None); }
        let gyro = self.read_status_one(id, 122, 48)?;
        let Some(gyro) = gyro.filter(|frame| frame.error == 0 && frame.data.len() == 48) else {
            self.defer_status_raw_imu();
            return Ok(None);
        };
        let Ok(gyro) = crate::imu_timing::decode_gyro_bias_for_load(&gyro.data) else { return Ok(None) };
        Ok(Some(duck_ipc_proto::RawImuObservation {
            t_ns: timestamp,
            sequence: u32::from_le_bytes(b[4..8].try_into().unwrap()),
            raw_g, corrected_g, gyro_rad_s,
            raw_gyro_rad_s: gyro.raw_rad_s.map(f64::from),
            ready: b[2] & 1 != 0,
            accel_calibrated: b[2] & 4 != 0,
            gyro_calibrated: gyro.loaded && gyro.ready,
        }))
    }

    fn defer_status_raw_imu(&mut self) {
        // Older firmware may not implement these optional reads; temporary
        // timeouts must also be retried instead of disabling them forever.
        self.observation.as_mut().expect("status scan exists").raw_imu_retry_at =
            Some(Instant::now() + Duration::from_secs(5));
    }

    /// Cache only registers actually read and verified by the startup checks.
    pub(super) fn cache_startup_configuration(&mut self, id: u8, bytes: &[u8]) {
        let scan = self.observation.get_or_insert_with(ObservationScan::new);
        let Some(joint) = HX_BUS_JOINT_IDS.iter().position(|candidate| *candidate == id)
            .map(|index| HX_BUS_JOINT_INDICES[index]) else { return };
        scan.latest.servos[joint].configuration = Some(duck_ipc_proto::ServoConfiguration {
            firmware_version: [bytes[0], bytes[1]],
            servo_version: [bytes[3], bytes[4]],
            position_pid: [bytes[21], bytes[23], bytes[22]],
            overtemperature_c: bytes[13],
            minimum_starting_torque: u16::from_le_bytes([bytes[24], bytes[25]]),
        });
        scan.latest.servos[joint].configuration_error = None;
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
            gravity: Some(imu.gravity), error: None, raw_sample: None,
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
