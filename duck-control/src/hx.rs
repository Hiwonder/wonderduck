//! HX bus backend.
//!
//! HX-30HM uses the same compact `0xFF 0xFF` frame shape as the bench driver, but its
//! production control path is deliberately kept separate from the Dynamixel implementation.
//! Each control tick reads all model joints with one broadcast `SYNC_READ`;
//! the IMU either joins it as ID200 or supplies an independent I2C sample.
//! targets use one broadcast `SYNC_WRITE`.
//! The device firmware must implement those instructions;
//! a stock firmware that only supports per-ID reads is rejected by the startup probe.

use std::f64::consts::PI;

use std::time::{Duration, Instant};

use serialport::{ClearBuffer, SerialPort};
use crate::execution_timing::StageTimer;

mod diagnostics;
mod calibration_recovery;
mod frames;
mod imu_source;
mod imu_reference;
mod sensing;
mod observations;
mod gyro_gains;
mod servo_diagnostics;
mod position_gain;
mod startup_config;
use diagnostics::DiagnosticCache;
use frames::{Frames, StatusFrame};
use imu_source::ImuReader;
pub use imu_source::ImuSource;
pub use calibration_recovery::CalibrationRestoreAttempts;


use crate::io::{
    ImuStale, IoError, JointTargets, ReadFeedback, Result, RobotIo, Sensors, SentGoals,
    ServoFeedback, SlowSensors,
};
use crate::model::{BAUD_RATE, DEFAULT_POSITION, IMU_DXL_ID, JOINT_IDS, NUM_JOINTS};
use crate::read_diagnostic::{ReadFailure, ReadFailureReason};

const BROADCAST_ID: u8 = 0xfe;
const READ: u8 = 0x02;
const WRITE: u8 = 0x03;
const SYNC_READ: u8 = 0x82;
const SYNC_WRITE: u8 = 0x83;

/// HX registers used by the control loop.
const PRESENT_POSITION: u8 = 56;
const PRESENT_POSITION_LEN: u8 = 12;
const GOAL_POSITION: u8 = 42;
const TORQUE_ENABLE: u8 = 40;
const VOLTAGE_OFFSET: usize = 62 - PRESENT_POSITION as usize;
const TEMPERATURE_OFFSET: usize = 63 - PRESENT_POSITION as usize;
const RECORD_FEEDBACK_ADDRESS: u8 = TORQUE_ENABLE;
const RECORD_FEEDBACK_LENGTH: u8 = PRESENT_POSITION + PRESENT_POSITION_LEN - RECORD_FEEDBACK_ADDRESS;

/// How long a move takes, as `(address, width, wanted)` — asserted at startup.
///
/// These are the servo's *motion shaping*, and the firmware loads them from its own defaults
/// at power-up. A running time of 1000 spreads every target over a second, which turns the
/// whole robot into slow motion whatever the policy asks for: measured on ours, 14 joints
/// read 1000 while the mouth read 5, and a 200-count step that settles in 40 ms elsewhere
/// took the full window and stopped 100 counts short. Nothing in this runtime writes these
/// registers, so a board that powers up that way stays that way until somebody reads them —
/// which is the same reason the Dynamixel backend asserts its EEPROM registers.
///
/// Zero means "no limit" in all three; the policy owns the timing.
const MOTION_REGISTERS: &[(u8, u8, u16)] = &[
    (ACCELERATION, 1, 0),
    (RUNNING_TIME, 2, 0),
    (GOAL_SPEED, 2, 0),
];
const ACCELERATION: u8 = 41;
const RUNNING_TIME: u8 = 44;
const GOAL_SPEED: u8 = 46;

const READ_TIMEOUT: Duration = Duration::from_millis(40);
const MAX_STALE_INPUT_RETRIES: usize = 3;
const STALE_INPUT_RETRY_DELAY: Duration = Duration::from_millis(5);
const TORQUE_VERIFY_ATTEMPTS: usize = 3;
const COUNTS_PER_REV: f64 = 4096.0;
const POSITION_MIN: u16 = 0;
const POSITION_MAX: u16 = 4095;
// Measured physical sense in JOINT_NAMES order.  These are the persistent HX
// encoder signs, not the diagnostic multipliers accepted by hx-static-pose.
const HX_JOINT_DIRECTIONS: [i8; NUM_JOINTS] = [
    -1, -1, -1, -1, -1, // left leg
    -1, -1, -1, -1, -1, // neck, head, mouth
    -1, -1, -1, -1, -1, // right leg
];
const HX_RAD_PER_SPEED_COUNT: f64 = 2.0 * PI / COUNTS_PER_REV;
const HX_BUS_JOINT_IDS: [u8; NUM_JOINTS] =
    [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14];
const HX_BUS_JOINT_INDICES: [usize; NUM_JOINTS] =
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14];
pub const PHYSICAL_HOME_ENCODER_TARGETS: [u16; NUM_JOINTS] = [
    2055, 2096, 2300, 2048, 1796, 1780, 1820, 2048, 2048, 2048, 2041, 2000, 1796, 2048, 2300,
];

/// Fixed physical Home, separate from the policy's observation and action reference.
pub fn home_target() -> [f64; NUM_JOINTS] {
    std::array::from_fn(|index| {
        decode_position(index, &PHYSICAL_HOME_ENCODER_TARGETS[index].to_le_bytes())
            .expect("physical Home encoder targets must be valid")
    })
}

fn packet(id: u8, instruction: u8, params: &[u8]) -> Result<Vec<u8>> {
    if id == 0xff || params.len() > 251 {
        return Err(IoError::Bus("invalid HX packet id or length".into()));
    }
    let length = params
        .len()
        .checked_add(2)
        .ok_or_else(|| IoError::Bus("HX packet length overflow".into()))?;
    let mut frame = Vec::with_capacity(params.len() + 6);
    frame.extend_from_slice(&[0xff, 0xff, id, length as u8, instruction]);
    frame.extend_from_slice(params);
    let checksum = !frame[2..]
        .iter()
        .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
    frame.push(checksum);
    Ok(frame)
}

fn read_request(id: u8, address: u8, length: u8) -> Result<Vec<u8>> {
    packet(id, READ, &[address, length])
}

fn sync_read_request(ids: &[u8], address: u8, length: u8) -> Result<Vec<u8>> {
    if ids.is_empty() {
        return Err(IoError::Bus("HX sync_read requires at least one ID".into()));
    }
    if ids.contains(&BROADCAST_ID) {
        return Err(IoError::Bus(
            "HX sync_read IDs may not include broadcast".into(),
        ));
    }
    if length == 0 || u16::from(address) + u16::from(length) > 256 {
        return Err(IoError::Bus(format!(
            "invalid HX read range {address}+{length}"
        )));
    }
    let mut params = vec![address, length];
    params.extend_from_slice(ids);
    packet(BROADCAST_ID, SYNC_READ, &params)
}

fn signed_magnitude(raw: u16, sign_bit: u32) -> i32 {
    let magnitude = i32::from(raw & ((1u16 << sign_bit) - 1));
    if raw & (1u16 << sign_bit) != 0 {
        -magnitude
    } else {
        magnitude
    }
}

fn validate_torque_ids(ids: &[u8]) -> Result<()> {
    if ids.is_empty()
        || ids.contains(&IMU_DXL_ID)
        || ids.iter().any(|id| *id == BROADCAST_ID || *id == 0xff)
        || ids
            .iter()
            .enumerate()
            .any(|(index, id)| ids[..index].contains(id))
    {
        return Err(IoError::Bus("invalid HX torque ID list".into()));
    }
    Ok(())
}

/// Native HX transport implementing the control-loop I/O contract.
pub struct HxIo {
    port: Option<Box<dyn SerialPort>>,
    port_path: String,
    imu: ImuReader,
    reference_imu: Option<ImuReader>,
    reference_source: Option<ImuSource>,
    diagnostics: DiagnosticCache,
    observation: Option<observations::ObservationScan>,
    /// Never issue a subsequent bus instruction after an unconfirmed status read.
    status_unconfirmed: bool,
    slow: Option<SlowSensors>,
    /// Recovery probes cannot send any actuator or configuration write.
    read_only: bool,
    /// The last all-joint torque state confirmed from the servos.
    ///
    /// HX servos resume holding when they receive goal-position traffic after torque has been
    /// disabled. Keep the bus silent after a verified relax so that its physical result matches
    /// the torque-off request rather than relying on the Dynamixel-specific behavior of ignoring
    /// goals while torque is disabled.
    torque_enabled: Option<bool>,
    /// Maintenance can change motion shaping; restore it with the next new target.
    motion_restore_pending: bool,
    record_feedback: bool,
    last_sent_goals: Option<SentGoals>,
    servo_diagnostics_ids: Vec<u8>,
}

impl HxIo {
    pub fn open(port: &str) -> Result<Self> {
        let serial = open_serial(port)?;

        Ok(Self {
            port: Some(serial),
            port_path: port.to_owned(),
            imu: ImuReader::new(ImuSource::default()),
            reference_imu: None,
            reference_source: None,
            diagnostics: DiagnosticCache::default(),
            observation: None,
            status_unconfirmed: false,
            slow: None,
            read_only: false,
            torque_enabled: None,
            motion_restore_pending: false,
            record_feedback: false,
            last_sent_goals: None,
            servo_diagnostics_ids: Vec::new(),
        })
    }

    /// Open the HX port for read-only safety recovery. This handle cannot be
    /// promoted to a writable handle; drop it before ordinary startup checks.
    pub fn open_read_only(port: &str) -> Result<Self> {
        let mut io = Self::open(port)?;
        io.read_only = true;
        Ok(io)
    }

    fn port_mut(&mut self) -> Result<&mut (dyn SerialPort + '_)> {
        match self.port.as_mut() {
            Some(port) => Ok(port.as_mut()),
            None => Err(IoError::Bus("motor bus is in maintenance".into())),
        }
    }

    /// Select before startup checks; changing sources discards prior sensor state.
    pub fn set_imu_source(&mut self, source: ImuSource) {
        self.imu = ImuReader::new(source);
        self.observation = None;
        self.configure_reference_imu();
    }

    fn imu_device_id(&self) -> u8 {
        self.imu.source().bus_id().unwrap_or(IMU_DXL_ID)
    }

    /// Select after the source, before startup checks, to request a later MCU snapshot.
    pub fn set_imu_last(&mut self, enabled: bool) -> Result<()> {
        self.imu.set_last(enabled)
    }

    /// Extend the existing broadcast only while collecting a physical recording.
    pub fn set_record_feedback(&mut self, enabled: bool) {
        self.record_feedback = enabled;
        self.configure_reference_imu();
    }

    /// Probe the selected control devices and verify their startup registers.
    pub fn check_registers(&mut self) -> Result<usize> {
        self.begin_control_observation();
        self.read_control_blocks(PRESENT_POSITION, PRESENT_POSITION_LEN)?;
        let motion_fixed = self.check_servo_motion_registers()?;
        Ok(motion_fixed + self.check_servo_startup_configuration()?)
    }

    /// Verify servo motion shaping without querying the IMU.
    pub fn check_servo_motion_registers(&mut self) -> Result<usize> {
        let mut fixed = 0;
        for &(address, width, want) in MOTION_REGISTERS {
            let blocks = self.sync_read_ids(&HX_BUS_JOINT_IDS, address, width)?;
            for (index, block) in blocks.iter().enumerate() {
                let id = HX_BUS_JOINT_IDS[index];
                let got = decode_register(block, width)?;
                if got == want {
                    continue;
                }
                tracing::warn!(id, address, got, want, "correcting HX motion register");
                self.write_register(id, address, width, want)?;
                fixed += 1;
            }
        }
        Ok(fixed)
    }

    fn verify_servo_motion_registers(&mut self) -> Result<()> {
        let blocks = self.sync_read_ids(&HX_BUS_JOINT_IDS, ACCELERATION, 7)?;
        for (index, block) in blocks.iter().enumerate() {
            for &(address, width, want) in MOTION_REGISTERS {
                let start = usize::from(address - ACCELERATION);
                let got = decode_register(&block[start..start + usize::from(width)], width)?;
                if got != want {
                    return Err(IoError::Bus(format!(
                        "HX motion register did not restore: ID {}, address {address}, got {got}, want {want}",
                        HX_BUS_JOINT_IDS[index]
                    )));
                }
            }
        }
        Ok(())
    }

    /// Set and verify the volatile MCU correction switch before the control loop.
    pub fn configure_accel_calibration(&mut self, enabled: bool) -> Result<()> {
        self.reject_host_calibration()?;
        if !self.imu.source().shared_bus() {
            return Err(IoError::Bus("MCU acceleration calibration requires ID200".into()));
        }
        if !self.gyro_bias_state()?.supported {
            self.imu.set_acceleration_enabled(false);
            tracing::info!(enabled = false,
                "MCU hardware fusion; external acceleration calibration not supported");
            return Ok(());
        }
        let desired = [1, 0, u8::from(enabled), 0];
        let mut params = vec![116];
        params.extend_from_slice(&desired);
        let frames = self.exchange(packet(self.imu_device_id(), WRITE, &params)?, &[self.imu_device_id()], 4)
            .map_err(|error| IoError::Bus(format!(
                "MCU acceleration calibration switch requires updated QMI8658 firmware: {error}"
            )))?;
        if frames.len() != 1 || frames[0].data != desired {
            return Err(IoError::Bus("MCU acceleration calibration write acknowledgement mismatch".into()));
        }
        if self.read_register_block(self.imu_device_id(), 116, 4)? != desired {
            return Err(IoError::Bus("MCU acceleration calibration readback mismatch".into()));
        }
        self.wait_for_imu_ready_at(self.imu_device_id())?;
        self.imu.set_acceleration_enabled(enabled);
        tracing::info!(enabled, persistent = false, "MCU acceleration calibration runtime switch");
        Ok(())
    }

    /// Query stored coefficients once at startup, before the 50 Hz control loop.
    pub fn report_accel_calibration(&mut self) -> Result<()> {
        self.reject_host_calibration()?;
        if !self.imu.source().shared_bus() {
            return Ok(());
        }
        self.report_gyro_bias()?;
        if !self.gyro_bias_state()?.supported {
            self.imu.set_acceleration_enabled(false);
            self.imu.set_acceleration_calibration(crate::io::AccelerationCalibration {
                flags: 0, generation: 0, bias_g: [0.; 3], scale: [1.; 3],
            });
            let sample = self.read_register_block(self.imu_device_id(), 40, 28)?;
            self.imu.record_acceleration(&sample)?;
            tracing::info!(enabled = false,
                "MCU hardware fusion acceleration telemetry; no external correction");
            return Ok(());
        }
        let block = self.read_register_block(self.imu_device_id(), 16, 32)?;
        let (flags, generation, bias_g, scale) = diagnostics::decode_accel_calibration(&block)?;
        self.imu.set_acceleration_calibration(crate::io::AccelerationCalibration {
            flags, generation, bias_g, scale,
        });
        // Confirm the paired firmware before interpreting the changed HX gyro unit.
        let sample = self.read_register_block(self.imu_device_id(), 40, 28)?;
        self.imu.record_acceleration(&sample)?;
        match flags {
            1 if self.imu.acceleration_enabled() => tracing::info!(enabled = true, generation, ?bias_g, ?scale,
                "MCU accel calibration active"),
            1 => tracing::info!(enabled = false, generation, "MCU accel calibration bypassed"),
            0 => tracing::info!(enabled = false, generation,
                "MCU accel calibration not installed"),
            _ => tracing::warn!(enabled = false, generation,
                "MCU accel calibration storage fault; identity correction active"),
        }
        Ok(())
    }

    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        let blocks = self.sync_read_ids(&HX_BUS_JOINT_IDS, PRESENT_POSITION, 2)?;
        let mut positions = DEFAULT_POSITION;
        for (bus_index, block) in blocks.iter().enumerate() {
            let joint_index = HX_BUS_JOINT_INDICES[bus_index];
            positions[joint_index] = decode_position(joint_index, block)?;
        }
        Ok(positions)
    }

    /// Read raw position registers for a diagnostic probe.
    ///
    /// The control-loop contract always uses the configured fifteen joints and converts
    /// positions to radians. A bring-up probe must also be able to exercise one device (for
    /// example an unassembled ID 1 servo), so it uses the same framed `SYNC_READ` path with an
    /// explicit ID list and returns the wire values unchanged.
    pub fn diagnostic_read_positions(&mut self, ids: &[u8]) -> Result<Vec<u16>> {
        let blocks = self.sync_read_ids(ids, PRESENT_POSITION, 2)?;
        blocks
            .iter()
            .map(|block| {
                if block.len() != 2 {
                    return Err(IoError::ShortRead {
                        what: "HX diagnostic position",
                        expected: 2,
                        got: block.len(),
                    });
                }
                let raw = u16::from_le_bytes([block[0], block[1]]);
                if raw > POSITION_MAX {
                    return Err(IoError::Bus(format!(
                        "HX position raw value out of range: {raw}"
                    )));
                }
                Ok(raw)
            })
            .collect()
    }

    /// Read the complete 12-byte present-state block for explicit servo IDs.
    ///
    /// This uses the same register range and HX `SYNC_READ` response validation as the
    /// production loop, except that the caller chooses a servo-only ID list.
    pub fn diagnostic_read_state_blocks(&mut self, ids: &[u8]) -> Result<Vec<Vec<u8>>> {
        self.sync_read_ids(ids, PRESENT_POSITION, PRESENT_POSITION_LEN)
    }

    /// Encode the configured home pose for an explicit diagnostic ID list.
    ///
    /// The caller may omit joints, such as the mouth, or order IDs for a physical bus. Each ID
    /// is looked up in `JOINT_IDS` before using the matching home angle and direction. Unknown
    /// IDs have no defined home pose and are rejected before any serial I/O is attempted.
    pub fn diagnostic_home_positions(ids: &[u8]) -> Result<Vec<u16>> {
        let home = home_target();
        ids.iter()
            .map(|id| {
                let index = JOINT_IDS
                    .iter()
                    .position(|configured| configured == id)
                    .ok_or_else(|| {
                        IoError::Bus(format!(
                            "HX diagnostic home has no configured joint for ID {id}"
                        ))
                    })?;
                encode_position(index, home[index])
            })
            .collect()
    }

    /// Write raw position registers for a diagnostic probe.
    ///
    /// This deliberately does not enable torque, change gains, or wait for acknowledgements:
    /// it mirrors the production HX broadcast write and leaves the caller in control of the
    /// servo's physical state.
    pub fn diagnostic_write_positions(&mut self, ids: &[u8], positions: &[u16]) -> Result<()> {
        if ids.len() != positions.len() {
            return Err(IoError::Bus(format!(
                "HX diagnostic write IDs/positions length mismatch: {} != {}",
                ids.len(),
                positions.len()
            )));
        }
        let data = positions
            .iter()
            .map(|position| {
                if *position > POSITION_MAX {
                    return Err(IoError::Bus(format!(
                        "HX position raw value out of range: {position}"
                    )));
                }
                Ok(position.to_le_bytes().to_vec())
            })
            .collect::<Result<Vec<_>>>()?;
        self.sync_write_ids(GOAL_POSITION, 2, ids, &data)
    }

    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        // A failed multi-servo operation may have changed only some joints.
        self.torque_enabled = None;
        self.set_torque_ids(&HX_BUS_JOINT_IDS, on)?;
        self.torque_enabled = Some(on);
        Ok(())
    }

    /// Set torque on an explicit diagnostic servo list and verify each servo's register.
    ///
    /// HX uses addressed `WRITE` for torque changes. A broadcast `SYNC_WRITE` never returns a
    /// status frame, so a serial write alone cannot establish that a servo actually released.
    /// ID 200 is an IMU and is never part of a torque list.
    pub fn diagnostic_set_torque(&mut self, ids: &[u8], on: bool) -> Result<()> {
        self.set_torque_ids(ids, on)
    }

    pub fn set_gain(&mut self, kp: u16) -> Result<()> {
        // The shared safety layer passes the Dynamixel kP value here. HX has a different
        // PID register contract, so treating that number as an HX P byte would silently
        // retune every actuator. Walking uses the motor's configured PID instead.
        tracing::debug!(kp, "HX backend ignores incompatible runtime position gain");
        Ok(())
    }

    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        let start = self.present_positions()?;
        let steps = (duration.as_secs_f64() / step.as_secs_f64())
            .ceil()
            .max(1.0) as u32;
        for i in 1..=steps {
            let t = f64::from(i) / f64::from(steps);
            let mut next = [0.0; NUM_JOINTS];
            for joint in 0..NUM_JOINTS {
                next[joint] = start[joint] + (target[joint] - start[joint]) * t;
            }
            self.write(&JointTargets::new(next))?;
            std::thread::sleep(step);
        }
        Ok(())
    }

    fn read_control_blocks(&mut self, address: u8, length: u8) -> Result<Vec<Vec<u8>>> {
        self.read_control_blocks_with_cycle_deadline(address, length, None)
    }

    fn read_control_blocks_with_cycle_deadline(
        &mut self,
        address: u8,
        length: u8,
        deadline: Option<Instant>,
    ) -> Result<Vec<Vec<u8>>> {
        let ids = self.control_ids();
        let mut blocks = match deadline {
            Some(end) => {
                self.sync_read_ids_with_cycle_deadline(ids, address, length, end)
            }
            None => self.sync_read_ids(ids, address, length),
        }?;
        if ids.first().is_some_and(|id| matches!(*id, 200 | 201)) {
            // Both wire orders decode to joints followed by the IMU block.
            blocks.rotate_left(1);
        }
        Ok(blocks)
    }

    fn set_torque_ids(&mut self, ids: &[u8], on: bool) -> Result<()> {
        validate_torque_ids(ids)?;

        let expected = u8::from(on);
        let mut failed = Vec::new();
        for &id in ids {
            let mut verified = false;
            let mut last_error = String::new();
            for attempt in 1..=TORQUE_VERIFY_ATTEMPTS {
                match self
                    .write_torque_register(id, expected)
                    .and_then(|_| self.read_torque_register(id))
                {
                    Ok(value) if value == expected => {
                        verified = true;
                        break;
                    }
                    Ok(value) => {
                        last_error = format!(
                            "attempt {attempt}: read back 0x{value:02X}, expected 0x{expected:02X}"
                        );
                    }
                    Err(error) => {
                        last_error = format!("attempt {attempt}: {error}");
                    }
                }
            }
            if !verified {
                failed.push(format!("ID {id}: {last_error}"));
            }
        }
        if failed.is_empty() {
            tracing::info!(?ids, on, "HX torque state verified");
            Ok(())
        } else {
            Err(IoError::Bus(format!(
                "HX torque state did not verify after {TORQUE_VERIFY_ATTEMPTS} attempts: {}",
                failed.join("; ")
            )))
        }
    }

    fn write_torque_register(&mut self, id: u8, value: u8) -> Result<()> {
        self.exchange(
            packet(id, WRITE, &[TORQUE_ENABLE, value])?,
            &[id],
            0,
        )?;
        Ok(())
    }

    /// One addressed register write, for the startup assertion of [`MOTION_REGISTERS`].
    fn write_register(&mut self, id: u8, address: u8, width: u8, value: u16) -> Result<()> {
        let mut params = vec![address];
        params.extend_from_slice(&encode_register(value, width)?);
        self.exchange(packet(id, WRITE, &params)?, &[id], 0)?;
        Ok(())
    }

    fn read_torque_register(&mut self, id: u8) -> Result<u8> {
        let block = self.read_register_block(id, TORQUE_ENABLE, 1)?;
        block
            .first()
            .copied()
            .ok_or_else(|| IoError::Bus(format!("HX torque read returned no data for ID {id}")))
    }

    fn read_register_block(&mut self, id: u8, address: u8, length: u8) -> Result<Vec<u8>> {
        let mut frames = self.exchange(read_request(id, address, length)?, &[id], length)?;
        let frame = frames
            .pop()
            .ok_or_else(|| IoError::Bus(format!("HX read returned no frame for ID {id}")))?;
        Ok(frame.data)
    }

    fn sync_read_ids(&mut self, ids: &[u8], address: u8, length: u8) -> Result<Vec<Vec<u8>>> {
        let expected = ids.to_vec();
        let frames = self.exchange(sync_read_request(ids, address, length)?, &expected, length)?;
        Ok(frames.into_iter().map(|frame| frame.data).collect())
    }

    fn sync_read_ids_with_cycle_deadline(
        &mut self,
        ids: &[u8],
        address: u8,
        length: u8,
        deadline: Instant,
    ) -> Result<Vec<Vec<u8>>> {
        let frames = self.exchange_with_cycle_deadline(
            sync_read_request(ids, address, length)?,
            ids,
            length,
            deadline,
        )?;
        Ok(frames.into_iter().map(|frame| frame.data).collect())
    }

    fn sync_write_ids(
        &mut self,
        address: u8,
        length: u8,
        ids: &[u8],
        data: &[Vec<u8>],
    ) -> Result<()> {
        if length == 0
            || ids.is_empty()
            || data.len() != ids.len()
            || data.iter().any(|item| item.len() != usize::from(length))
            || ids.iter().any(|id| *id == BROADCAST_ID || *id == 0xff)
            || ids
                .iter()
                .enumerate()
                .any(|(index, id)| ids[..index].contains(id))
        {
            return Err(IoError::Bus("invalid HX sync_write data".into()));
        }
        let mut params = vec![address, length];
        for (id, item) in ids.iter().zip(data) {
            params.push(*id);
            params.extend_from_slice(item);
        }
        let frame = packet(BROADCAST_ID, SYNC_WRITE, &params)?;
        self.send_instruction(&frame)
    }

    fn exchange(&mut self, tx: Vec<u8>, expected: &[u8], length: u8) -> Result<Vec<StatusFrame>> {
        self.send_instruction(&tx).map_err(|error| {
            IoError::Bus(format!("HX request transmission: {error}; {}", self.diagnostics.summary()))
        })?;
        self.receive(&tx, expected, length, Instant::now() + READ_TIMEOUT, false, None)
    }

    fn exchange_with_cycle_deadline(
        &mut self,
        tx: Vec<u8>,
        expected: &[u8],
        length: u8,
        deadline: Instant,
    ) -> Result<Vec<StatusFrame>> {
        self.send_instruction(&tx)?;
        let result = self.receive(
            &tx,
            expected,
            length,
            Instant::now() + READ_TIMEOUT,
            true,
            Some(deadline),
        );
        self.port_mut()?.set_timeout(READ_TIMEOUT).map_err(io_error)?;
        result
    }

    fn receive(
        &mut self,
        tx: &[u8],
        expected: &[u8],
        length: u8,
        deadline: Instant,
        bounded: bool,
        cycle_deadline: Option<Instant>,
    ) -> Result<Vec<StatusFrame>> {
        self.receive_inner(tx, expected, length, deadline, bounded, cycle_deadline, true)
    }

    fn receive_inner(
        &mut self,
        tx: &[u8],
        expected: &[u8],
        length: u8,
        deadline: Instant,
        bounded: bool,
        cycle_deadline: Option<Instant>,
        record_servo_status: bool,
    ) -> Result<Vec<StatusFrame>> {
        let started = Instant::now();
        let observe_control = tx.len() >= 9 && tx[4] == SYNC_READ
            && matches!((tx[5], tx[6]),
                (PRESENT_POSITION, PRESENT_POSITION_LEN)
                    | (RECORD_FEEDBACK_ADDRESS, RECORD_FEEDBACK_LENGTH))
            && expected == self.control_ids();
        let mut parser = Frames::default();
        let mut found: Vec<StatusFrame> = Vec::with_capacity(expected.len());
        let mut buffer = [0u8; 256];
        let mut cycle_overdue_logged = false;
        while Instant::now() < deadline {
            if let Some(cycle_end) = cycle_deadline
                && !cycle_overdue_logged
                && Instant::now() >= cycle_end
            {
                let missing: Vec<u8> = expected
                    .iter()
                    .copied()
                    .filter(|id| !found.iter().any(|frame| frame.id == *id))
                    .collect();
                tracing::warn!(
                    missing_ids = ?missing,
                    elapsed_ms = started.elapsed().as_secs_f64() * 1000.0,
                    "HX control read exceeded cycle deadline; awaiting replies"
                );
                cycle_overdue_logged = true;
            }
            if bounded {
                let wait_until = if cycle_overdue_logged {
                    deadline
                } else {
                    cycle_deadline.map_or(deadline, |cycle_end| cycle_end.min(deadline))
                };
                let remaining = wait_until.saturating_duration_since(Instant::now());
                if remaining.is_zero() {
                    continue;
                }
                self.port_mut()?.set_timeout(remaining).map_err(io_error)?;
            }
            match self.port_mut()?.read(&mut buffer) {
                Ok(size) => parser.feed(&buffer[..size]),
                Err(error)
                    if matches!(error.kind(), std::io::ErrorKind::TimedOut
                        | std::io::ErrorKind::Interrupted) => {}
                Err(error) => {
                    return Err(self.read_failure(
                        ReadFailureReason::Response(error.to_string()),
                        tx, &parser, &found, started.elapsed(),
                    ));
                }
            }
            while let Some(frame) = parser.pop() {
                if self.read_only {
                    let evidence = parser.evidence(&found);
                    if evidence.checksum_errors != 0 || evidence.invalid_lengths != 0 || evidence.discarded_bytes != 0 {
                        return Err(self.read_failure(ReadFailureReason::Response(
                            "damaged read-only voltage response".into()), tx, &parser, &found, started.elapsed()));
                    }
                }
                if !expected.contains(&frame.id) {
                    if self.read_only {
                        return Err(self.read_failure(ReadFailureReason::Response(
                            "unexpected read-only voltage response".into()), tx, &parser, &found, started.elapsed()));
                    }
                    parser.unexpected_frames += 1;
                    continue;
                }
                if found.iter().any(|item| item.id == frame.id) {
                    if self.read_only {
                        return Err(self.read_failure(ReadFailureReason::Response(
                            "duplicate read-only voltage response".into()), tx, &parser, &found, started.elapsed()));
                    }
                    parser.duplicate_frames += 1;
                    continue;
                }
                if (matches!(frame.id, 200 | 201) && frame.error != 0) || frame.error & 0xC0 != 0
                    || frame.data.len() != usize::from(length) {
                    let reason = ReadFailureReason::Status {
                        id: frame.id, status: frame.error,
                        payload_len: frame.data.len(), expected_len: length,
                    };
                    return Err(self.read_failure(reason, tx, &parser, &found, started.elapsed()));
                }
                if record_servo_status { self.diagnostics.observe_servo_status(frame.id, frame.error); }
                if observe_control { self.observe_control_frame(&frame, tx[5]); }
                found.push(frame);
                if found.len() == expected.len() {
                    if self.read_only && parser.evidence(&found).pending_bytes != 0 {
                        return Err(self.read_failure(ReadFailureReason::Response(
                            "trailing bytes after read-only voltage response".into()), tx, &parser, &found, started.elapsed()));
                    }
                    self.diagnostics.record_control_read(tx, &found, &parser);
                    found.sort_by_key(|item| expected.iter().position(|id| *id == item.id));
                    return Ok(found);
                }
            }
        }
        Err(self.read_failure(ReadFailureReason::Timeout,
                              tx, &parser, &found, started.elapsed()))
    }

    fn read_failure(
        &mut self,
        reason: ReadFailureReason,
        tx: &[u8],
        parser: &Frames,
        found: &[StatusFrame],
        elapsed: Duration,
    ) -> IoError {
        self.diagnostics.record_control_read(tx, found, parser);
        let expected = if tx[4] == SYNC_READ { &tx[7..tx.len() - 1] } else { &tx[2..3] };
        let mut failure = ReadFailure {
            reason,
            missing_ids: expected.iter().copied()
                .filter(|id| !found.iter().any(|frame| frame.id == *id)).collect(),
            tx: tx.to_vec(), elapsed_us: elapsed.as_micros() as u64,
            wire: parser.evidence(found), host_reads: 0, host_imu_miss: 0, host_servo_miss: 0,
            host_checksum: 0, host_bad_length: 0,
        };
        self.diagnostics.fill_failure(&mut failure);
        let error = IoError::ReadFailure(std::sync::Arc::new(failure));
        tracing::debug!(error = &error as &(dyn std::error::Error + 'static),
            "HX transaction failure detail");
        error
    }

    /// Match the official Dynamixel transport: discard only unexpected input, then enqueue
    /// the packet without a transmit drain. The physical half-duplex interface switches
    /// direction below this process; `write_all` must not be followed by `tcdrain`, whose
    /// current RK3568 UART driver turns a sub-millisecond wire transmission into ~12 ms.
    fn send_instruction(&mut self, frame: &[u8]) -> Result<()> {
        if self.status_unconfirmed {
            return Err(IoError::Bus("status reply unconfirmed; motor bus is quarantined".into()));
        }
        if self.read_only && !matches!(frame.get(4), Some(&READ | &SYNC_READ)) {
            return Err(IoError::Bus("HX recovery bus is read-only".into()));
        }
        let mut timing = (frame.get(4) == Some(&SYNC_WRITE))
            .then(|| StageTimer::<2>::new("serial_sync_write", "input_cleanup"));
        if self.read_only {
            // Do not silently discard an unsolicited/late reply and continue
            // using the bus as if it were an unambiguous recovery sample.
            if self.port_mut()?.bytes_to_read().map_err(io_error)? != 0 {
                return Err(IoError::Bus("unexpected pending bytes on read-only voltage bus".into()));
            }
        } else {
            self.discard_stale_input()?;
        }
        if let Some(timing) = timing.as_mut() { timing.advance("serial_write"); }
        self.port_mut()?
            .write_all(frame)
            .map_err(|error| IoError::Bus(format!("HX serial write: {error}")))
    }

    fn discard_stale_input(&mut self) -> Result<()> {
        let mut timing = StageTimer::<10>::new("serial_input_cleanup", "pending_ioctl");
        for attempt in 0..MAX_STALE_INPUT_RETRIES {
            let pending = self.port_mut()?.bytes_to_read().map_err(io_error)?;
            if pending == 0 {
                return Ok(());
            }
            tracing::debug!(pending_bytes = pending, attempt,
                "HX discarded stale input before instruction");
            timing.advance("clear_ioctl");
            self.port_mut()?.clear(ClearBuffer::Input).map_err(io_error)?;
            timing.advance("retry_sleep");
            std::thread::sleep(STALE_INPUT_RETRY_DELAY);
            timing.advance("pending_ioctl");
        }
        if self.port_mut()?.bytes_to_read().map_err(io_error)? == 0 {
            Ok(())
        } else {
            Err(IoError::Bus(
                "HX input buffer remains nonempty before instruction".into(),
            ))
        }
    }
}

/// One register block, little-endian, as the servo reports it.
fn decode_register(block: &[u8], width: u8) -> Result<u16> {
    match (width, block) {
        (1, [value]) => Ok(u16::from(*value)),
        (2, [low, high]) => Ok(u16::from_le_bytes([*low, *high])),
        (width, block) => Err(IoError::ShortRead {
            what: "HX register block",
            expected: usize::from(width),
            got: block.len(),
        }),
    }
}

/// The bytes a register write carries, in the same widths.
fn encode_register(value: u16, width: u8) -> Result<Vec<u8>> {
    match width {
        1 => Ok(vec![value as u8]),
        2 => Ok(value.to_le_bytes().to_vec()),
        other => Err(IoError::Bus(format!(
            "unsupported HX register width {other}"
        ))),
    }
}

fn io_error(error: serialport::Error) -> IoError {
    IoError::Bus(format!("HX serial I/O: {error}"))
}

#[cfg(unix)]
fn open_serial(path: &str) -> Result<Box<dyn SerialPort>> {
    let mut serial = serialport::new(path, BAUD_RATE)
        .timeout(READ_TIMEOUT)
        .open_native()
        .map_err(|e| IoError::Port {
            path: path.to_owned(),
            source: std::io::Error::other(e),
        })?;
    #[cfg(target_os = "linux")]
    lock_serial(&serial).map_err(|source| IoError::Port { path: path.to_owned(), source })?;
    serial.set_exclusive(true).map_err(io_error)?;
    Ok(Box::new(serial))
}

#[cfg(target_os = "linux")]
fn lock_serial(port: &serialport::TTYPort) -> std::io::Result<()> {
    use std::os::fd::AsRawFd;

    // The maintenance tools take the same advisory lock; unlike TIOCEXCL it also
    // blocks a root process from reclaiming the port while maintenance still owns it.
    if unsafe { libc::flock(port.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } == 0 {
        Ok(())
    } else {
        Err(std::io::Error::last_os_error())
    }
}

#[cfg(not(unix))]
fn open_serial(path: &str) -> Result<Box<dyn SerialPort>> {
    serialport::new(path, BAUD_RATE)
        .timeout(READ_TIMEOUT)
        .open()
        .map_err(|e| IoError::Port {
            path: path.to_owned(),
            source: std::io::Error::other(e),
        })
}

/// After an interrupted status read, discard input until the bus is quiet before
/// sending any further instruction. This is a bounded recovery, not a guarantee
/// against a device whose undocumented response delay exceeds the quiet window.
pub fn drain_status_input(path: &str) -> Result<()> {
    let port = open_serial(path)?;
    let started = Instant::now();
    let mut quiet_since = started;
    let mut discarded = 0u64;
    loop {
        let now = Instant::now();
        if now.duration_since(quiet_since) >= Duration::from_millis(500) {
            tracing::warn!(discarded_bytes = discarded, "status bus recovery completed after quiet interval");
            return Ok(());
        }
        if now.duration_since(started) >= Duration::from_secs(3) {
            return Err(IoError::Bus("status bus remains noisy; cannot resume motor control".into()));
        }
        let pending = port.bytes_to_read().map_err(io_error)?;
        if pending > 0 {
            port.clear(ClearBuffer::Input).map_err(io_error)?;
            discarded += u64::from(pending);
            quiet_since = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Decode an HX encoder position using the same joint calibration as feedback reads.
pub fn decode_position(index: usize, block: &[u8]) -> Result<f64> {
    if block.len() < 2 {
        return Err(IoError::ShortRead {
            what: "HX position",
            expected: 2,
            got: block.len(),
        });
    }
    let direction = HX_JOINT_DIRECTIONS
        .get(index)
        .copied()
        .ok_or_else(|| IoError::Bus(format!("invalid HX joint index {index}")))?;
    let raw = u16::from_le_bytes([block[0], block[1]]);
    if raw > POSITION_MAX {
        return Err(IoError::Bus(format!(
            "HX position raw value out of range: {raw}"
        )));
    }
    Ok(f64::from(direction) * ((2.0 * PI * f64::from(raw) / COUNTS_PER_REV) - PI))
}

/// Encode one model joint using the same calibration as the runtime motor writes.
pub fn encode_position(index: usize, radians: f64) -> Result<u16> {
    if !radians.is_finite() {
        return Err(IoError::Bus("HX target position is not finite".into()));
    }
    if !(-PI..=PI).contains(&radians) {
        return Err(IoError::Bus(format!(
            "HX target position out of range: {radians}"
        )));
    }
    let direction = HX_JOINT_DIRECTIONS
        .get(index)
        .copied()
        .ok_or_else(|| IoError::Bus(format!("invalid HX joint index {index}")))?;
    let raw = (COUNTS_PER_REV * (PI + f64::from(direction) * radians) / (2.0 * PI)).round();
    Ok(raw.clamp(f64::from(POSITION_MIN), f64::from(POSITION_MAX)) as u16)
}

fn decode_record_feedback(feedback: &mut ServoFeedback, joint: usize, block: &[u8]) -> Result<()> {
    if block.len() != usize::from(RECORD_FEEDBACK_LENGTH) {
        return Err(IoError::ShortRead {
            what: "HX recording feedback",
            expected: usize::from(RECORD_FEEDBACK_LENGTH),
            got: block.len(),
        });
    }
    let offset = |address: u8| usize::from(address - RECORD_FEEDBACK_ADDRESS);
    let word = |address: u8| {
        let index = offset(address);
        u16::from_le_bytes([block[index], block[index + 1]])
    };
    let goal = word(GOAL_POSITION);
    feedback.goal_positions_rad[joint] = decode_position(joint, &goal.to_le_bytes())?;
    feedback.goal_position_counts[joint] = goal;
    feedback.goal_acceleration_raw[joint] = block[offset(ACCELERATION)];
    feedback.goal_duration_ms[joint] = word(RUNNING_TIME);
    feedback.goal_speed_counts_s[joint] = word(GOAL_SPEED);
    feedback.trajectory_active[joint] = block[offset(66)];
    feedback.internal_goal_low_byte[joint] = block[offset(67)];
    let pwm = signed_magnitude(word(60), 10);
    feedback.pwm_register_signed[joint] = pwm as i16;
    feedback.pwm_joint_ratio[joint] = f64::from(HX_JOINT_DIRECTIONS[joint]) * f64::from(pwm) / 1000.0;
    feedback.volts[joint] = f64::from(block[offset(62)]) * 0.1;
    feedback.temperatures_c[joint] = f64::from(block[offset(63)]);
    feedback.status[joint] = block[offset(65)];
    feedback.torque_enable[joint] = block[offset(TORQUE_ENABLE)];
    feedback.max_pwm_counts[joint] = word(48);
    Ok(())
}


impl RobotIo for HxIo {
    fn maintenance_handoff(&mut self, release: bool) -> Result<()> {
        if release {
            if self.port.is_none() {
                return Err(IoError::Bus("motor bus is already released".into()));
            }
            self.port.take();
            self.observation = None;
        } else {
            if self.port.is_some() {
                return Err(IoError::Bus("motor bus is already open".into()));
            }
            self.port = Some(open_serial(&self.port_path)?);
            self.torque_enabled = None;
            if self.imu.source().shared_bus() && self.imu.source() != ImuSource::Id200Host {
                if let Err(error) = self.report_accel_calibration() {
                    self.port.take();
                    return Err(error);
                }
            }
            self.motion_restore_pending = true;
        }
        Ok(())
    }

    fn maintenance_reclaim_for_shutdown(&mut self) -> Result<()> {
        if self.port.is_some() {
            return Err(IoError::Bus("motor bus is already open".into()));
        }
        // Shutdown only needs the bus to release torque; control must not resume
        // without the normal IMU calibration and readiness checks.
        self.port = Some(open_serial(&self.port_path)?);
        self.torque_enabled = None;
        Ok(())
    }

    fn record_servo_diagnostics(&mut self, budget: Duration) -> Option<crate::servo_diagnostics::Capture> {
        self.capture_servo_diagnostics(budget)
    }

    fn measures_load(&self) -> bool {
        // The HX load proxy is duty cycle; publishing it as milliamps would mislabel it.
        false
    }

    fn read(&mut self) -> Result<Sensors> {
        self.read_sensors(None)
    }

    fn read_with_cycle_deadline(&mut self, deadline: Instant) -> Result<Sensors> {
        self.read_sensors(Some(deadline))
    }

    fn sample_device_observation(&mut self, deadline: Instant) -> Option<crate::io::DeviceObservation> {
        Some(self.sample_observation(deadline))
    }

    fn sample_individual_device_observation(&mut self, deadline: Instant) -> Result<Option<crate::io::DeviceObservation>> {
        self.sample_one_observation(deadline)
    }

    fn set_individual_device_observation(&mut self, active: bool) {
        self.set_individual_observation(active);
    }

    fn supports_individual_device_observation(&self) -> bool { true }

    fn quarantine_device_bus(&mut self) { self.status_unconfirmed = true; }

    fn torque_release_verified(&self) -> bool {
        !self.status_unconfirmed && self.port.is_some() && self.torque_enabled == Some(false)
    }

    fn recover_device_bus(&mut self) -> Result<()> {
        self.port.take();
        self.observation = None;
        self.status_unconfirmed = true;
        drain_status_input(&self.port_path)?;
        self.status_unconfirmed = false;
        let result = (|| {
            // Do not make torque release depend on the MCU being reachable. The
            // normal maintenance handoff reports IMU calibration immediately;
            // after a bus fault that report can fail even though the servos still
            // accept an individually verified release.
            self.port = Some(open_serial(&self.port_path)?);
            self.torque_enabled = None;
            self.set_torque(false)?;
            if self.imu.source().shared_bus() && self.imu.source() != ImuSource::Id200Host {
                self.report_accel_calibration()?;
                let port = self.port_path.clone();
                if self.imu.acceleration_enabled() { self.load_fixed_accel_calibration(&port)?; }
                self.load_fixed_gyro_calibration(&port)?;
            }
            self.motion_restore_pending = true;
            Ok(())
        })();
        if result.is_err() { self.status_unconfirmed = true; }
        result
    }

    fn begin_device_observation(&mut self, generation: u64) {
        self.begin_observation(generation);
    }

    fn end_device_observation(&mut self) {
        self.end_observation();
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        if self.torque_enabled == Some(false) {
            return Ok(());
        }
        let mut counts = [0; NUM_JOINTS];
        // Restore shaping together with a new target, never on the old target.
        let restoring = self.motion_restore_pending;
        let width = if restoring { 7 } else { 2 };
        let mut params = [0u8; 2 + 8 * NUM_JOINTS];
        params[..2].copy_from_slice(&[if restoring { ACCELERATION } else { GOAL_POSITION }, width as u8]);
        for (bus_index, (&id, &index)) in HX_BUS_JOINT_IDS.iter()
            .zip(&HX_BUS_JOINT_INDICES).enumerate()
        {
            let raw = encode_position(index, targets.positions[index])?;
            counts[index] = raw;
            let offset = 2 + (1 + width) * bus_index;
            params[offset] = id;
            let position = offset + if restoring { 2 } else { 1 };
            params[position..position + 2].copy_from_slice(&raw.to_le_bytes());
        }
        let frame = packet(BROADCAST_ID, SYNC_WRITE, &params[..2 + (1 + width) * NUM_JOINTS])?;
        self.send_instruction(&frame)?;
        if restoring {
            self.verify_servo_motion_registers()?;
            self.motion_restore_pending = false;
        }
        if self.record_feedback {
            self.last_sent_goals = Some(SentGoals {
                position_counts: counts,
                sent_t_ns: duck_ipc_proto::clock::monotonic_ns(),
            });
        }
        Ok(())
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        HxIo::set_torque(self, on)
    }

    fn prepare_torque_only(&mut self, positions: &[f64; NUM_JOINTS]) -> Result<()> {
        let raw = HX_BUS_JOINT_INDICES.iter().map(|&index|
            encode_position(index, positions[index])).collect::<Result<Vec<_>>>()?;
        self.diagnostic_write_positions(&HX_BUS_JOINT_IDS, &raw)?;
        let confirmed = self.sync_read_ids(&HX_BUS_JOINT_IDS, GOAL_POSITION, 2)?;
        if confirmed.len() != raw.len() || confirmed.iter().zip(&raw).any(|(block, &expected)|
            block.as_slice() != expected.to_le_bytes())
        {
            return Err(IoError::Bus("current-pose servo goals did not verify; torque remains off".into()));
        }
        Ok(())
    }

    fn read_positions_raw(&mut self, ids: &[u8]) -> Result<Vec<u16>> {
        let positions = self.diagnostic_read_positions(ids)?;
        validate_raw_joint_positions(ids, &positions)?;
        Ok(positions)
    }

    fn write_positions_raw(&mut self, ids: &[u8], positions: &[u16]) -> Result<()> {
        validate_raw_joint_positions(ids, positions)?;
        self.diagnostic_write_positions(ids, positions)
    }

    fn reboot(&mut self, id: u8) -> Result<()> {
        Err(IoError::Bus(format!(
            "HX backend does not support reboot for ID {id}"
        )))
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        HxIo::set_gain(self, kp)
    }

    fn set_position_p_gain(&mut self, kp: u8) -> Result<()> {
        self.write_temporary_position_p_gain(kp)
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        self.slow.ok_or_else(|| IoError::Bus(
            "voltage/temperature unavailable: no complete control read with valid voltage".into()))
    }

    fn imu_stale(&self) -> ImuStale {
        self.imu.stale()
    }

    fn imu_ready(&self) -> bool {
        self.imu.ready()
    }

    fn restart_imu_stream(&mut self) {
        self.imu.invalidate_host();
    }
}

fn validate_raw_joint_positions(ids: &[u8], positions: &[u16]) -> Result<()> {
    if ids.is_empty() || ids.len() != positions.len() {
        return Err(IoError::Bus("invalid raw joint position list".into()));
    }
    for (index, (&id, &position)) in ids.iter().zip(positions).enumerate() {
        if ids[..index].contains(&id) {
            return Err(IoError::Bus(format!("duplicate joint ID {id}")));
        }
        let joint = JOINT_IDS.iter().position(|candidate| *candidate == id)
            .ok_or_else(|| IoError::Bus(format!("unknown joint ID {id}")))?;
        let angle = decode_position(joint, &position.to_le_bytes())?;
        let (min, max) = crate::model::JOINT_LIMITS[joint];
        if !(min..=max).contains(&angle) {
            return Err(IoError::Bus(format!("raw joint position outside limit: ID {id}, position {position}")));
        }
    }
    Ok(())
}
