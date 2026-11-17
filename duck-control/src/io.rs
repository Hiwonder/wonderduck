//! The seam between the control loop and the physical world.
//!
//! Everything between [`RobotIo::read`] and [`RobotIo::write`] is pure computation over
//! plain data, which is what makes the loop testable without a robot.
//!
//! [`Sensors`] combines joints and a valid IMU sample at the backend boundary. Dynamixel
//! reads them together; HX combines its servo transaction with the latest independent I2C sample.

use std::time::Instant;

use crate::imu::ImuData;
use crate::model::NUM_JOINTS;
pub use duck_ipc_proto::{DeviceObservation, ImuObservation, ServoObservation};

/// One control observation of the robot, assembled by the backend.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sensors {
    /// Joint angles in radians, indexed as [`crate::model::JOINT_NAMES`].
    pub positions: [f64; NUM_JOINTS],
    /// Joint velocities, rad/s.
    pub velocities: [f64; NUM_JOINTS],
    /// Present current magnitude, mA. The sign is dropped by whichever driver fills this —
    /// [`crate::bus`] takes the absolute value as it unpacks the word — because what every
    /// consumer so far wants is load. It is not recoverable from `velocities`: a joint holding
    /// a squat or a foot taking weight is near-zero velocity at non-zero current.
    pub currents_ma: [f64; NUM_JOINTS],
    /// Exact HX status registers from the control read; absent for other backends.
    pub servo_status: Option<[duck_ipc_proto::ServoStatus; NUM_JOINTS]>,
    /// Lowest servo voltage from this complete control read, if available.
    pub min_servo_voltage: Option<f64>,
    pub imu: ImuData,
    /// Acquisition evidence carried even when recording is disabled, including on coasted reads.
    pub imu_timing: Option<ImuSampleTiming>,
    /// Register and timing evidence when physical recording is enabled.
    pub feedback: Option<ReadFeedback>,
    /// Same ID200 sample as the gyro/quaternion, available in the recording broadcast.
    pub acceleration: Option<AccelerationSample>,
}

impl Default for Sensors {
    fn default() -> Self {
        Self {
            positions: [0.0; NUM_JOINTS],
            velocities: [0.0; NUM_JOINTS],
            currents_ma: [0.0; NUM_JOINTS],
            servo_status: None,
            min_servo_voltage: None,
            imu: ImuData::default(),
            imu_timing: None,
            feedback: None,
            acceleration: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct AccelerationCalibration {
    pub flags: u16,
    pub generation: u32,
    pub bias_g: [f32; 3],
    pub scale: [f32; 3],
}

/// Unfiltered acceleration includes gravity and motion; it is not projected gravity.
/// Saturation flags refer to sensor X/Y/Z, before the mounting rotation.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct AccelerationSample {
    pub raw_sensor_g: [f64; 3],
    /// Reconstructed with the installed MCU coefficients and its float32 arithmetic.
    pub corrected_sensor_g: [f64; 3],
    pub raw_trunk_g: [f64; 3],
    pub corrected_trunk_g: [f64; 3],
    pub full_scale_g: u8,
    pub gyro_full_scale_dps: u16,
    pub accel_saturated_axes: [bool; 3],
    pub gyro_saturated_axes: [bool; 3],
    pub calibration: AccelerationCalibration,
}

/// The last goal packet whose host write completed; this is not a device acknowledgement.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct SentGoals {
    pub position_counts: [u16; NUM_JOINTS],
    pub sent_t_ns: u64,
}

/// Registers read in one broadcast. Arrays follow the model's joint order.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize)]
pub struct ServoFeedback {
    pub read_start_t_ns: u64,
    pub read_end_t_ns: u64,
    pub previous_host_goal: Option<SentGoals>,
    pub goal_position_counts: [u16; NUM_JOINTS],
    pub goal_positions_rad: [f64; NUM_JOINTS],
    /// Motion shaping registers from the same broadcast as the final goal.
    pub goal_acceleration_raw: [u8; NUM_JOINTS],
    pub goal_duration_ms: [u16; NUM_JOINTS],
    pub goal_speed_counts_s: [u16; NUM_JOINTS],
    /// Firmware trajectory flag; zero does not mean the mechanical joint is settled.
    pub trajectory_active: [u8; NUM_JOINTS],
    /// Register 67 only. Register 68 is outside this packet, so this is not an angle.
    pub internal_goal_low_byte: [u8; NUM_JOINTS],
    /// Signed register counts, before applying the installed joint direction.
    pub pwm_register_signed: [i16; NUM_JOINTS],
    /// Signed duty ratio in the same direction as joint positions and velocities.
    pub pwm_joint_ratio: [f64; NUM_JOINTS],
    pub volts: [f64; NUM_JOINTS],
    pub temperatures_c: [f64; NUM_JOINTS],
    pub status: [u8; NUM_JOINTS],
    pub torque_enable: [u8; NUM_JOINTS],
    pub max_pwm_counts: [u16; NUM_JOINTS],
}

/// Host acquisition timing, not the fusion chip's unobserved measurement time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "lowercase")]
pub enum ImuTimingSource {
    I2c,
    Id200,
    Id201,
    #[serde(rename = "id200-host")]
    Id200Host,
}

impl ImuTimingSource {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::I2c => "i2c",
            Self::Id200 => "id200",
            Self::Id201 => "id201",
            Self::Id200Host => "id200-host",
        }
    }

    pub fn sequence_basis(self) -> &'static str {
        match self {
            Self::I2c => "device_clock_change",
            Self::Id200 | Self::Id201 => "host_reply",
            Self::Id200Host => "mcu_raw_samples",
        }
    }
}

/// Host transfer times, not physical measurement times inside the sensor.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct I2cReadTiming {
    pub poll_start_t_ns: u64,
    pub quaternion_end_t_ns: u64,
    pub gyro_end_t_ns: u64,
    pub poll_end_t_ns: u64,
    pub requested_ready_t_ns: Option<u64>,
}

/// A new I2C sequence requires a changed module clock. ID200 counts received replies only.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ImuSampleTiming {
    pub source: ImuTimingSource,
    pub sequence: u64,
    pub acquired_t_ns: u64,
    pub consumed_t_ns: u64,
    pub age_us: u64,
    /// Consecutive identical ID200 payloads; equality alone cannot establish sensor staleness.
    pub payload_unchanged_run: u64,
    /// Present only when the actual consumed data carries matching MCU timestamps.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mcu: Option<crate::imu_timing::McuTiming>,
    /// Every raw 100 Hz sample consumed by the host gravity estimator this cycle.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub host: Option<crate::imu_host::HostImuTiming>,
    /// Host I2C transfer bounds; these are not the chip's measurement timestamps.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub i2c: Option<I2cReadTiming>,
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ReadFeedback {
    pub servo: ServoFeedback,
    pub imu: ImuSampleTiming,
    /// From the same I2C sampler object as the primary gyro/quaternion, in trunk-frame g.
    pub i2c_acceleration_g: Option<[f64; 3]>,
    /// The other IMU is recorded independently and never replaces the policy input.
    pub reference_imu: Option<ReferenceImu>,
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ReferenceImu {
    pub source: ImuTimingSource,
    /// Host fetch interval: serial read for ID200, cached sample access for I2C.
    /// The sample's timing.acquired_t_ns identifies its actual acquisition start.
    pub read_start_t_ns: u64,
    pub read_end_t_ns: u64,
    /// ID200 register reply, retained on decode failure; I2C has no such block.
    pub raw_block: Option<[u8; 28]>,
    pub sample: Option<ReferenceImuSample>,
    pub error: Option<&'static str>,
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ReferenceImuSample {
    pub gyro_rad_s: [f64; 3],
    pub gravity_trunk: [f64; 3],
    pub quat_trunk_to_world: [f64; 4],
    pub acceleration: Option<AccelerationSample>,
    /// Module-calibrated acceleration from the same I2C sample, in trunk-frame g.
    pub i2c_acceleration_g: Option<[f64; 3]>,
    pub timing: ImuSampleTiming,
}

/// What the loop commands. Position control only — alpha has no velocity-mode joints.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointTargets {
    pub positions: [f64; NUM_JOINTS],
}

impl JointTargets {
    pub fn new(positions: [f64; NUM_JOINTS]) -> Self {
        Self { positions }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum IoError {
    #[error("serial port {path}: {source}")]
    Port {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("bus transaction failed: {0}")]
    Bus(String),
    #[error("bus transaction failed: {0}")]
    ReadFailure(std::sync::Arc<crate::read_diagnostic::ReadFailure>),
    /// A `sync_read` that returns the wrong number of blocks, or a block of the wrong
    /// length, means a device did not answer. Reported rather than papered over: a
    /// silently short read would leave stale values in half the joint array.
    #[error("{what}: expected {expected}, got {got}")]
    ShortRead {
        what: &'static str,
        expected: usize,
        got: usize,
    },
    #[error("simulated failure")]
    Simulated,
}

pub type Result<T> = std::result::Result<T, IoError>;

/// What the servos report about their own supply and case, rather than their motion.
///
/// Sampled about once a second rather than every tick — see [`RobotIo::slow_sensors`]. A pack
/// does not drain and a motor does not heat up in 20 ms, so the tick would be paying for a
/// second bus transaction to learn nothing new.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlowSensors {
    /// Mean supply voltage across the servos, in volts — the only battery measurement the
    /// robot has, since there is no fuel gauge anywhere on the hat.
    pub volts: f64,
    /// Per-joint case temperature in °C, indexed as [`crate::model::JOINT_NAMES`].
    ///
    /// Per-joint rather than an average because the interesting case is one joint carrying
    /// the load — a knee holding a squat runs far hotter than the mouth, and a mean over
    /// fifteen servos hides exactly the servo about to latch its overheat shutdown.
    pub temps_c: [f64; NUM_JOINTS],
}

/// Repeated IMU observations: identical payloads on Dynamixel, unchanged device clocks on I2C.
///
/// Two numbers, because they answer two different questions and only one of them is worth
/// waking someone for. The *total* says how often the board has repeated itself over the whole
/// run: sporadic hits are ordinary, since the loop and the board keep their own clocks and a
/// tick landing inside one board refresh legitimately sees the same bytes twice. The *run* says
/// whether orientation is frozen right now — a board that has stopped fusing repeats on every
/// single tick, so its run climbs without bound while a total on its own looks the same as a
/// handful of hiccups.
///
/// Reported together so no backend can offer one without the other; a run with no total to
/// scale it against is how the count came to be read as an alarm in the first place.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ImuStale {
    /// Stale reads since startup, cumulative and never reset.
    pub total: u64,
    /// Length of the current unbroken run of stale reads. Any fresh block resets it to zero.
    pub run: u64,
}

pub trait RobotIo {
    /// Joints and a valid IMU sample, assembled into one observation.
    fn read(&mut self) -> Result<Sensors>;
    /// Pass the control cycle deadline to backends that report overdue reads.
    fn read_with_cycle_deadline(&mut self, _deadline: Instant) -> Result<Sensors> {
        self.read()
    }
    /// Obtain per-device observations for an active subscriber.
    /// Partial observations retain per-device times and never become policy input.
    fn sample_device_observation(&mut self, _deadline: Instant) -> Option<DeviceObservation> {
        None
    }
    /// Read at most one device, on the control thread and only while the status guard is active.
    fn sample_individual_device_observation(&mut self, _deadline: Instant) -> Result<Option<DeviceObservation>> {
        Ok(None)
    }
    fn set_individual_device_observation(&mut self, _active: bool) {}
    fn supports_individual_device_observation(&self) -> bool { false }
    fn quarantine_device_bus(&mut self) {}
    /// Start or refresh a device-status subscription.
    fn begin_device_observation(&mut self, _generation: u64) {}
    /// Stop preparing device observations when the last subscriber leaves.
    fn end_device_observation(&mut self) {}
    fn write(&mut self, targets: &JointTargets) -> Result<()>;

    /// Transfer the motor bus to or from a connection-bound maintenance session.
    /// Neither operation may send motor commands.
    fn maintenance_handoff(&mut self, _release: bool) -> Result<()> {
        Err(IoError::Bus("maintenance handoff is unsupported".into()))
    }

    /// Reclaim the bus for shutdown only, without requiring sensors to be ready.
    /// The caller must not resume motion after this operation.
    fn maintenance_reclaim_for_shutdown(&mut self) -> Result<()> {
        self.maintenance_handoff(false)
    }

    /// Whether [`Sensors::velocities`] from this backend is a measurement.
    ///
    /// [`Sensors`] is fixed-size arrays, so a backend with nothing to report has no way to say
    /// so in the sample itself — it reports zeros, which is what a robot standing perfectly
    /// still reports. The wire draws that distinction (`RobotState::velocities` is absent
    /// rather than zero when nothing measured it), and this is how it learns which it has:
    /// a real bus answers yes, `FakeIo` answers no until a test states what the servos report.
    ///
    /// Defaulted true because a backend that reads real servos has nothing to declare.
    fn measures_velocity(&self) -> bool {
        true
    }

    /// The same question for [`Sensors::currents_ma`], separately: the simulator reports
    /// velocity from the physics and may have no load to report at all.
    fn measures_load(&self) -> bool {
        true
    }

    /// Set the position P gain on every joint.
    ///
    /// Here rather than in the bus layer alone because it is what makes "go limp" mean
    /// something. Refusing to command a fallen robot only freezes it in the pose it fell
    /// in; dropping the gain lets it yield. The prototype does the same, at kP 50 against
    /// a running value of 200.
    fn set_gain(&mut self, kp: u16) -> Result<()>;

    /// Set a temporary native position P byte, independently of the Dynamixel gain scale.
    /// Unsupported backends must refuse instead of pretending that a fall softened them.
    fn set_position_p_gain(&mut self, _kp: u8) -> Result<()> {
        Err(IoError::Bus(
            "temporary native position P gain is unsupported".into(),
        ))
    }

    /// Torque on or off, every joint.
    ///
    /// On the trait — rather than only on the bus — because bringing the robot up is now something
    /// the control loop does when a human asks, and everything that reaches a motor goes through
    /// [`crate::safety::Safety`], which owns the only handle.
    ///
    /// **Nothing calls this at startup, and that part has not changed.** A `robotd` restarted by an
    /// update must leave a standing robot standing: torque comes on when someone enables the policy,
    /// never because a process began.
    fn set_torque(&mut self, on: bool) -> Result<()>;

    /// Set and verify the measured pose as the motor goal before turning torque on.
    fn prepare_torque_only(&mut self, _positions: &[f64; NUM_JOINTS]) -> Result<()> {
        Err(IoError::Bus("holding current pose before torque is unsupported".into()))
    }

    /// Read raw position counts for explicitly identified joints.
    fn read_positions_raw(&mut self, _ids: &[u8]) -> Result<Vec<u16>> {
        Err(IoError::Bus("raw servo position read is unsupported".into()))
    }

    /// Write raw position counts for explicitly identified joints.
    fn write_positions_raw(&mut self, _ids: &[u8], _positions: &[u16]) -> Result<()> {
        Err(IoError::Bus("raw servo position write is unsupported".into()))
    }

    /// Reboot one servo: the Protocol 2 REBOOT instruction.
    ///
    /// The way out of a latched hardware error — overload, overheating, electrical shock, the
    /// `shutdown` mask — which otherwise holds torque off until the battery is pulled. The servo is
    /// off the bus for a few hundred milliseconds and comes back with torque off and its RAM
    /// registers, the gains among them, at their EEPROM defaults; the caller owns putting them back.
    fn reboot(&mut self, id: u8) -> Result<()>;

    /// Supply voltage and case temperatures. HX reuses its complete control-read cache;
    /// Dynamixel needs another transaction because its registers are outside the main block.
    fn slow_sensors(&mut self) -> Result<SlowSensors>;

    /// Optional backend diagnostics within the remaining control-cycle budget.
    /// Called by the bus owner at the slow-sensor cadence; never from another thread.
    fn poll_diagnostics(&mut self, _budget: std::time::Duration) {}

    /// Optional firmware counters after the goal write, never part of policy input.
    fn record_servo_diagnostics(&mut self, _budget: std::time::Duration) -> Option<crate::servo_diagnostics::Capture> {
        None
    }

    /// Diagnostics the bus keeps about itself. Default to "nothing to report" so a fake or a
    /// future backend is not obliged to invent them.
    ///
    /// `sync_read` blocks identical to their predecessor: the board answered but did not
    /// refresh, which means the policy would be fed dead orientation. Invisible unless
    /// someone counts it, which is why it is counted.
    fn imu_stale(&self) -> ImuStale {
        ImuStale::default()
    }

    /// Has the orientation filter converged? False for the first moments after startup.
    fn imu_ready(&self) -> bool {
        true
    }

    /// Begin continuous sampling after a deliberately interrupted setup phase.
    /// Only host-owned fusion needs its history discarded; device fusion stays running.
    /// Do not call this for ordinary read failures or policy enable/disable edges.
    fn restart_imu_stream(&mut self) {}
}

/// A robot made of nothing, for tests.
///
/// Always compiled — it is what the test suite runs against, and it is why `cargo test`
/// needs no hardware, no network and no Docker. Positions echo back whatever was last
/// written, so a loop driving it behaves like a servo that tracks perfectly.
pub struct FakeIo {
    sensors: Sensors,
    /// Set to make the next [`RobotIo::read`] fail, then cleared. For exercising the
    /// loop's error path without a flaky bus.
    pub fail_next_read: bool,
    /// Reads still to fail before this one starts answering, decremented per read. Models a
    /// robot whose servos are not powered yet, and then are.
    pub fail_reads: u32,
    pub last_written: Option<JointTargets>,
    pub reads: usize,
    pub writes: usize,
    /// Last gain commanded, so a test can tell "went limp" from "stopped commanding".
    pub last_gain: Option<u16>,
    pub last_position_p_gain: Option<u8>,
    /// Whether the orientation filter reports converged. False models the first seconds after
    /// startup, when projected gravity is not yet a measurement.
    pub imu_ready: bool,
    /// What [`RobotIo::slow_sensors`] reports. Mid-pack and hand-warm by default so `--fake`
    /// shows a plausible robot; set it to exercise a flat pack or a cooking servo. `None`
    /// fails the read, which is what a robot with no bus does.
    pub slow: Option<SlowSensors>,
    /// When true, `read` reports the last written targets as the present positions.
    track_targets: bool,
    /// Whether a test has stated what the servos report for velocity and load. Until one has,
    /// this fake declares it measures neither — a `--fake` robot is not a robot standing still
    /// under no load, and a dashboard must be able to tell those apart.
    velocity_stated: bool,
    load_stated: bool,
    /// Last torque state commanded, or `None` if nothing ever asked. `None` is the assertion that
    /// matters most: it is what "a restart did not move the robot" looks like.
    pub torque: Option<bool>,
    /// How many times torque was written, so a test can tell "brought up once" from "written every
    /// tick" — the latter being a bus transaction per joint per tick.
    pub torque_writes: usize,
    /// Target present when torque was most recently enabled.
    pub torque_on_target: Option<JointTargets>,
    /// Every servo id rebooted, in order.
    pub reboots: Vec<u8>,
}

impl Default for FakeIo {
    fn default() -> Self {
        Self::new()
    }
}

impl FakeIo {
    pub fn new() -> Self {
        Self {
            sensors: Sensors::default(),
            fail_next_read: false,
            fail_reads: 0,
            last_written: None,
            reads: 0,
            writes: 0,
            last_gain: None,
            last_position_p_gain: None,
            imu_ready: true,
            slow: Some(SlowSensors {
                volts: 7.4,
                temps_c: [32.0; NUM_JOINTS],
            }),
            track_targets: true,
            velocity_stated: false,
            load_stated: false,
            torque: None,
            torque_writes: 0,
            torque_on_target: None,
            reboots: Vec::new(),
        }
    }

    /// Start from a known pose — typically [`crate::model::DEFAULT_POSITION`].
    pub fn at(positions: [f64; NUM_JOINTS]) -> Self {
        let mut io = Self::new();
        io.sensors.positions = positions;
        io
    }

    /// Freeze reported positions so they ignore what is written. Models a robot whose
    /// servos are limp, or one being pushed around by hand.
    pub fn frozen(mut self) -> Self {
        self.track_targets = false;
        self
    }

    /// Fail the next `n` reads, then behave normally. Models a board that comes up before
    /// its servo power does.
    pub fn failing_reads(mut self, n: u32) -> Self {
        self.fail_reads = n;
        self
    }

    pub fn set_imu(&mut self, imu: ImuData) {
        self.sensors.imu = imu;
    }

    /// Joint velocities the next [`RobotIo::read`] will report.
    ///
    /// Unlike positions these are never derived from what was written — a fake that echoed
    /// its own targets back as velocity would make any test of the velocity path pass
    /// vacuously.
    pub fn set_velocities(&mut self, velocities: [f64; NUM_JOINTS]) {
        self.sensors.velocities = velocities;
        self.velocity_stated = true;
    }

    /// Present current per joint, mA. Same rule as [`Self::set_velocities`]: nothing derives
    /// this, so a test has to state what the servos are supposed to be reporting.
    pub fn set_currents_ma(&mut self, currents_ma: [f64; NUM_JOINTS]) {
        self.sensors.currents_ma = currents_ma;
        self.load_stated = true;
    }

    pub fn positions(&self) -> [f64; NUM_JOINTS] {
        self.sensors.positions
    }
}

impl RobotIo for FakeIo {
    fn maintenance_handoff(&mut self, _release: bool) -> Result<()> {
        Ok(())
    }

    fn read(&mut self) -> Result<Sensors> {
        if self.fail_next_read {
            self.fail_next_read = false;
            return Err(IoError::Simulated);
        }
        if self.fail_reads > 0 {
            self.fail_reads -= 1;
            return Err(IoError::Simulated);
        }
        self.reads += 1;
        Ok(self.sensors)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        self.writes += 1;
        self.last_written = Some(*targets);
        if self.track_targets {
            self.sensors.positions = targets.positions;
        }
        Ok(())
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        self.last_gain = Some(kp);
        Ok(())
    }

    fn set_position_p_gain(&mut self, kp: u8) -> Result<()> {
        self.last_position_p_gain = Some(kp);
        Ok(())
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        if on { self.torque_on_target = self.last_written; }
        self.torque = Some(on);
        self.torque_writes += 1;
        Ok(())
    }

    fn prepare_torque_only(&mut self, positions: &[f64; NUM_JOINTS]) -> Result<()> {
        self.write(&JointTargets::new(*positions))
    }

    fn reboot(&mut self, id: u8) -> Result<()> {
        self.reboots.push(id);
        Ok(())
    }

    fn measures_velocity(&self) -> bool {
        self.velocity_stated
    }

    fn measures_load(&self) -> bool {
        self.load_stated
    }

    fn imu_ready(&self) -> bool {
        self.imu_ready
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        self.slow.ok_or(IoError::Simulated)
    }
}
