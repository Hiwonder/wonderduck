//! Non-blocking, line-oriented records for comparing a physical run with simulation.
//!
//! The control thread only copies fixed-size data into a bounded channel. JSON encoding, file
//! writes, flushes and disk stalls are isolated in a writer thread, so an unhealthy filesystem
//! cannot turn into a late motor tick.

use std::fs::{File, create_dir_all};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, SyncSender, TrySendError};
use std::thread;

use duck_control::Sensors;
use duck_control::hx::ImuSource;
use duck_control::model::{JOINT_IDS, JOINT_LIMITS, JOINT_NAMES, NUM_JOINTS};
use duck_control::obs::{ACTION_LEN, OBS_LEN};
use duck_control::safety::Limit;
use serde::Serialize;

use crate::params::{BusProtocol, Params, Slot};

/// Enough for about twenty seconds at the normal 50 Hz rate. A full channel means the disk is
/// slower than the run; those samples are counted rather than delaying motor control.
const CHANNEL_CAPACITY: usize = 1024;
const SCHEMA: &str = "robotd-comparison-v1";

/// First JSONL record. It fixes every positional vector's interpretation for offline tools.
#[derive(Debug, Serialize)]
pub struct Metadata {
    #[serde(rename = "type")]
    record_type: &'static str,
    schema: &'static str,
    started_at_ns: u64,
    params_path: String,
    control_period_s: f64,
    cmd_alpha: f64,
    immediate_stop: bool,
    minimum_velocity_command_ms: u64,
    person_follow_only: bool,
    target_follow_only: bool,
    extended_servo_feedback: bool,
    freeze_policy_imu: bool,
    policy_gravity_from_reference: bool,
    policy_gyro_source: &'static str,
    policy_gravity_source: &'static str,
    imu_gyro_filter: bool,
    imu_gravity_filter: bool,
    gyro_bias_owner: Option<&'static str>,
    use_accel_calibration: bool,
    pose_calibration: Option<crate::imu_calibration::PoseCalibration>,
    bus: BusMetadata,
    policy: PolicyMetadata,
    joint_ids: [u8; NUM_JOINTS],
    joint_names: [&'static str; NUM_JOINTS],
    joint_limits_rad: [(f64, f64); NUM_JOINTS],
    observation_layout: [&'static str; 6],
    servo_diagnostics_ids: Vec<u8>,
}

#[derive(Debug, Serialize)]
struct BusMetadata {
    protocol: &'static str,
    port: String,
    imu_source: &'static str,
    imu_last: bool,
    imu_sequence_basis: &'static str,
    imu_acquisition_basis: &'static str,
    reference_imu_source: Option<&'static str>,
}

#[derive(Debug, Serialize)]
struct PolicyMetadata {
    enabled: bool,
    mode: &'static str,
    slots: Vec<PolicySlot>,
    head_lowpass: f64,
    legs_lowpass: f64,
    voltage_adapt: bool,
    nominal_voltage: f64,
    action_scale: f64,
    standing_action_scale: f64,
    ground_pick_action_scale: f64,
}

#[derive(Debug, Serialize)]
struct PolicySlot {
    name: String,
    path: Option<String>,
}

impl Metadata {
    pub fn with_immediate_stop(mut self, enabled: bool) -> Self {
        self.immediate_stop = enabled;
        self
    }

    pub fn with_person_follow_only(mut self, enabled: bool) -> Self {
        self.person_follow_only = enabled;
        self
    }

    pub fn with_target_follow_only(mut self, enabled: bool) -> Self {
        self.target_follow_only = enabled;
        self
    }

    pub fn with_extended_servo_feedback(mut self, enabled: bool) -> Self {
        self.extended_servo_feedback = enabled;
        self
    }

    pub fn with_servo_diagnostics(mut self, ids: &[u8]) -> Self {
        self.servo_diagnostics_ids = ids.to_vec();
        self
    }
    pub fn with_accel_calibration(mut self, enabled: bool) -> Self {
        self.use_accel_calibration = enabled;
        self
    }

    pub fn with_pose_calibration(mut self, calibration: Option<crate::imu_calibration::PoseCalibration>) -> Self {
        self.pose_calibration = calibration;
        self
    }
    pub fn with_imu_source(mut self, source: ImuSource) -> Self {
        if self.bus.protocol == "hx" {
            self.bus.imu_source = source.as_str();
            self.policy_gyro_source = source.as_str();
            self.policy_gravity_source = source.as_str();
            self.gyro_bias_owner = if source == ImuSource::Id200Host {
                Some("none_raw_host_pipeline")
            } else {
                if source == ImuSource::Id201 { Some("mcu") }
                else { source.shared_bus().then_some("mcu_manual_fixed") }
            };
            (self.bus.imu_sequence_basis, self.bus.imu_acquisition_basis) = match source {
                ImuSource::I2c => ("host_sampler", "host_i2c_read_start"),
                ImuSource::Id200 => ("host_replies", "host_transaction_end"),
                ImuSource::Id200Timing | ImuSource::Id201 => ("host_replies_with_mcu_sequence", "host_timed_reply_end"),
                ImuSource::Id200Host => ("mcu_raw_samples", "mcu_sample_age_bounded_by_host_reply"),
            };
        }
        self
    }

    pub fn with_reference_imu_source(mut self, source: Option<ImuSource>) -> Self {
        self.bus.reference_imu_source = source.map(ImuSource::as_str);
        self
    }

    pub fn with_policy_gravity_from_reference(mut self, enabled: bool) -> Self {
        self.policy_gravity_from_reference = enabled;
        if enabled {
            self.policy_gravity_source = self.bus.reference_imu_source.expect("validated reference source");
        }
        tracing::info!(policy_gravity_from_reference = enabled,
            policy_gyro_source = self.policy_gyro_source,
            policy_gravity_source = self.policy_gravity_source,
            "policy IMU observation sources");
        self
    }

    pub fn with_imu_last(mut self, enabled: bool) -> Self {
        self.bus.imu_last = enabled;
        self
    }

    pub fn with_imu_filter(mut self, enabled: bool) -> Self {
        self.imu_gyro_filter = enabled;
        self.imu_gravity_filter = enabled;
        self
    }

    pub fn with_imu_gyro_filter(mut self, enabled: bool) -> Self {
        self.imu_gyro_filter = enabled;
        self
    }

    pub fn from_params(params_path: &Path, params: &Params, freeze_policy_imu: bool) -> Self {
        let resolved = params.policy.resolved();
        let slots = Slot::ALL
            .into_iter()
            .map(|slot| PolicySlot {
                name: slot.as_str().to_owned(),
                path: resolved
                    .slot(slot)
                    .map(|path| path.to_string_lossy().into_owned()),
            })
            .chain(
                resolved.skills.iter()
                    .filter(|skill| Slot::parse(&skill.name).is_none())
                    .map(|skill| PolicySlot {
                        name: skill.name.clone(),
                        path: skill.resolved_path().map(|path| path.to_string_lossy().into_owned()),
                    }),
            )
            .collect();
        let (imu_source, imu_sequence_basis, imu_acquisition_basis) = match params.bus.protocol {
            BusProtocol::Hx => ("i2c", "host_sampler", "host_i2c_read_start"),
            BusProtocol::Dynamixel => ("dynamixel", "not_recorded", "not_recorded"),
        };

        Self {
            record_type: "metadata",
            servo_diagnostics_ids: Vec::new(),
            schema: SCHEMA,
            started_at_ns: duck_ipc_proto::clock::realtime_ns(),
            params_path: params_path.display().to_string(),
            control_period_s: params.period().as_secs_f64(),
            cmd_alpha: params.control.cmd_alpha.clamp(0.0, 1.0),
            immediate_stop: false,
            minimum_velocity_command_ms: 0,
            person_follow_only: false,
            target_follow_only: false,
            extended_servo_feedback: true,
            freeze_policy_imu,
            policy_gravity_from_reference: false,
            policy_gyro_source: imu_source,
            policy_gravity_source: imu_source,
            imu_gyro_filter: true,
            imu_gravity_filter: true,
            gyro_bias_owner: None,
            use_accel_calibration: true,
            pose_calibration: None,
            bus: BusMetadata {
                protocol: params.bus.protocol.as_str(),
                port: params.bus.port.clone(),
                imu_source,
                imu_last: false,
                imu_sequence_basis,
                imu_acquisition_basis,
                reference_imu_source: None,
            },
            policy: PolicyMetadata {
                enabled: params.policy.enabled,
                mode: params.policy.mode.as_str(),
                slots,
                head_lowpass: resolved.head_lowpass.unwrap_or(1.0),
                legs_lowpass: resolved.legs_lowpass.unwrap_or(1.0),
                voltage_adapt: resolved.voltage_adapt,
                nominal_voltage: resolved.nominal_voltage,
                action_scale: resolved.action_scale,
                standing_action_scale: resolved.standing_action_scale,
                ground_pick_action_scale: resolved.ground_pick_action_scale,
            },
            joint_ids: JOINT_IDS,
            joint_names: JOINT_NAMES,
            joint_limits_rad: JOINT_LIMITS,
            observation_layout: [
                "policy_gyro_rad_s[0:3]",
                "gravity_trunk[3:6]",
                "joint_position_relative_rad[6:20]",
                "joint_velocity_rad_s[20:34]",
                "previous_raw_action[34:48]",
                "command[48:61]",
            ],
        }
    }
}

/// The sensor sample the controller used. `fresh` on the enclosing record distinguishes it from
/// a coasted copy after a failed bus transaction.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct SensorSample {
    positions_rad: [f64; NUM_JOINTS],
    velocities_rad_s: [f64; NUM_JOINTS],
    currents_ma: [f64; NUM_JOINTS],
    gyro_rad_s: [f64; 3],
    /// Before host median, in the trunk frame with the same static bias correction.
    gyro_unfiltered_rad_s: [f64; 3],
    gravity_trunk: [f64; 3],
    quat_trunk_to_world: [f64; 4],
    acceleration: Option<duck_control::io::AccelerationSample>,
    imu_timing: Option<duck_control::io::ImuSampleTiming>,
    feedback: Option<duck_control::io::ReadFeedback>,
}

impl From<&Sensors> for SensorSample {
    fn from(sensors: &Sensors) -> Self {
        Self {
            positions_rad: sensors.positions,
            velocities_rad_s: sensors.velocities,
            currents_ma: sensors.currents_ma,
            gyro_rad_s: sensors.imu.gyro,
            gyro_unfiltered_rad_s: sensors.imu.gyro_unfiltered,
            gravity_trunk: sensors.imu.gravity,
            quat_trunk_to_world: sensors.imu.quat,
            acceleration: sensors.acceleration,
            imu_timing: sensors.imu_timing,
            feedback: sensors.feedback,
        }
    }
}

/// Requested or smoothed policy command. Body fields are z, roll, pitch.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct CommandSample {
    twist: [f64; 3],
    head: [f64; 4],
    body: [f64; 3],
    body_active: bool,
}

impl CommandSample {
    pub fn new(twist: [f64; 3], head: [f64; 4], body: [f64; 3], body_active: bool) -> Self {
        Self {
            twist,
            head,
            body,
            body_active,
        }
    }
}

/// Safety decisions as stable booleans. The record's metadata contains the limits needed to
/// reproduce the exact clamp for a `range` sample.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct LimitFlags {
    deadman: bool,
    range: bool,
    not_finite: bool,
}

/// Inputs and retained decision state for a person-following control tick.
#[derive(Debug, Serialize)]
pub struct PersonFollowSample {
    pub realtime_ns: u64,
    pub sequence: u64,
    pub phase: Option<duck_ipc_proto::CoordinatedTrackingPhase>,
    pub update_age_us: u64,
    pub vision_fresh: bool,
    pub camera: Option<duck_ipc_proto::TrackingCamera>,
    pub target: Option<duck_ipc_proto::VisualTrackingTarget>,
    pub head_position_raw: Option<u16>,
    pub head_arrived: bool,
    pub control_eligible: bool,
    pub capture_feedback_missing: bool,
    pub tracking_stopped: bool,
    pub deadzone: [bool; 3],
    pub settings: PersonFollowSettings,
    pub tof: PersonFollowTof,
    pub decision: crate::coordinated_tracking::PersonFollowDiagnostic,
}

/// Inputs, parameters and retained decision state for a color-target tracking tick.
#[derive(Debug, Serialize)]
pub struct TargetFollowSample {
    pub realtime_ns: u64,
    pub sequence: u64,
    pub mode: &'static str,
    pub phase: Option<duck_ipc_proto::CoordinatedTrackingPhase>,
    pub update_age_us: u64,
    pub vision_fresh: bool,
    pub camera: Option<duck_ipc_proto::TrackingCamera>,
    pub target: Option<duck_ipc_proto::VisualTrackingTarget>,
    pub head_position_raw: Option<u16>,
    pub head_arrived: bool,
    pub head_target_rad: Option<[f64; 2]>,
    pub control_eligible: bool,
    pub capture_feedback_missing: bool,
    pub tracking_stopped: bool,
    pub deadzone: [bool; 3],
    pub settings: TargetFollowSettings,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub decision: Option<crate::coordinated_tracking::TargetFollowDiagnostic>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stability: Option<crate::target_follow_stability::Diagnostic>,
}

#[derive(Debug, Serialize)]
pub struct TargetFollowSettings {
    pub head_deadzone: [f64; 2],
    pub head_gains: [[f64; 3]; 2],
    pub body_gains: [f64; 3],
    pub final_body_gains: [f64; 3],
    pub approach_linear_limit: Option<f64>,
    pub final_linear_limit: f64,
    pub final_lateral_limit: f64,
    pub goal_px: [f64; 2],
    pub tolerance_px: [f64; 2],
    pub arrival_delay_s: f64,
    pub arrival_skill: String,
    pub mouth_before: duck_ipc_proto::TimedMouthPosition,
    pub mouth_after: duck_ipc_proto::TimedMouthPosition,
}

impl From<&duck_ipc_proto::CoordinatedTrackingParams> for TargetFollowSettings {
    fn from(params: &duck_ipc_proto::CoordinatedTrackingParams) -> Self {
        Self { head_deadzone: params.head_deadzone, head_gains: params.head_gains,
            body_gains: params.body_gains, final_body_gains: params.final_body_gains,
            approach_linear_limit: params.approach_linear_limit, final_linear_limit: params.final_linear_limit,
            final_lateral_limit: params.final_lateral_limit, goal_px: params.final_point,
            tolerance_px: params.final_tolerance, arrival_delay_s: params.arrival_delay_s,
            arrival_skill: params.arrival_skill.clone(), mouth_before: params.mouth_before,
            mouth_after: params.mouth_after }
    }
}

#[derive(Debug, Serialize)]
pub struct PersonFollowTof {
    pub observed_at_t_ns: u64,
    pub sampled_t_ns: Option<u64>,
    /// Unfiltered VL53L0X result, present only for person-follow recording.
    pub raw_distance_mm: Option<u32>,
    pub distance_mm: Option<u32>,
    pub sensor_status: duck_ipc_proto::DistanceSensorStatus,
    /// False means the controller was not eligible to use the reading this tick.
    pub supplied_to_tracker: bool,
    pub obstacle_assistance_active: bool,
}

#[derive(Debug, Serialize)]
pub struct PersonFollowSettings {
    pub stop_area_px: Option<u64>,
    pub area_deadzone_px: u64,
    pub stop_distance_mm: Option<u32>,
    pub distance_deadzone_mm: u32,
    pub goal_px: [f64; 2],
    pub tolerance_px: [f64; 2],
    pub body_gains: [f64; 3],
    pub linear_limit: Option<f64>,
}

impl From<&duck_ipc_proto::CoordinatedTrackingParams> for PersonFollowSettings {
    fn from(params: &duck_ipc_proto::CoordinatedTrackingParams) -> Self {
        Self {
            stop_area_px: params.stop_area_px,
            area_deadzone_px: params.stop_area_deadzone_px,
            stop_distance_mm: params.stop_distance_mm,
            distance_deadzone_mm: params.stop_distance_deadzone_mm,
            goal_px: params.final_point,
            tolerance_px: params.final_tolerance,
            body_gains: params.body_gains,
            linear_limit: params.approach_linear_limit,
        }
    }
}

impl LimitFlags {
    pub fn from_limits(limits: &[Limit]) -> Self {
        Self {
            deadman: limits.contains(&Limit::Deadman),
            range: limits.contains(&Limit::Range),
            not_finite: limits.contains(&Limit::NotFinite),
        }
    }
}

/// One complete control-loop sample. All fields are final control-path values, except for
/// `targets_before_safety`, whose name makes the subsequent mechanical clamp explicit.
#[derive(Debug, Serialize)]
pub struct Sample {
    #[serde(rename = "type")]
    record_type: &'static str,
    pub tick: u64,
    /// Monotonic timestamp captured immediately before the bus read attempt.
    pub tick_t_ns: u64,
    /// Timestamp of the sensors carried in this record, absent before the first good read.
    pub sensors_t_ns: Option<u64>,
    pub elapsed_s: f64,
    pub fresh: bool,
    pub sensors: Option<SensorSample>,
    /// Exact 61-float tensor handed to ONNX, including frozen IMU when enabled.
    #[serde(serialize_with = "serialize_optional_f32_array")]
    pub policy_observation: Option<[f32; OBS_LEN]>,
    /// Raw 14-float ONNX result, before action scale and target filters.
    pub policy_action: Option<[f32; ACTION_LEN]>,
    /// Monotonic timestamp immediately after the policy produced this tick's targets.
    pub policy_output_t_ns: Option<u64>,
    /// Acquisition age and reuse measured immediately before the controller consumes IMU.
    pub policy_imu_input: Option<crate::imu_input::Input>,
    /// Reference validation at selection and consumption; failure prevents inference.
    pub policy_gravity_input: Option<crate::policy_gravity::Evidence>,
    /// Bounds around applying the targets through safety and the bus write.
    pub apply_start_t_ns: u64,
    pub apply_end_t_ns: u64,
    pub command_requested: CommandSample,
    pub command_applied: CommandSample,
    pub policy_label: String,
    pub driving: bool,
    pub inference_failed: bool,
    pub fallen: bool,
    pub limp: bool,
    pub gain_requested: u16,
    pub gain_applied: Option<u16>,
    pub targets_before_safety: [f64; NUM_JOINTS],
    pub limits: LimitFlags,
    pub write_ok: bool,
    pub tick_elapsed_ms: f64,
    /// Separate query after this frame's motor write; null means collection is disabled/unsupported.
    pub servo_diagnostics: Option<duck_control::servo_diagnostics::Capture>,
    /// Present only for person following while this `--record` session is active.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub person_follow: Option<PersonFollowSample>,
    /// Present only for color-target tracking while this `--record` session is active.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_follow: Option<TargetFollowSample>,
}

impl Sample {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        tick: u64,
        tick_t_ns: u64,
        sensors_t_ns: Option<u64>,
        elapsed_s: f64,
        fresh: bool,
        sensors: Option<&Sensors>,
        policy_observation: Option<[f32; OBS_LEN]>,
        policy_action: Option<[f32; ACTION_LEN]>,
        policy_output_t_ns: Option<u64>,
        apply_start_t_ns: u64,
        apply_end_t_ns: u64,
        command_requested: CommandSample,
        command_applied: CommandSample,
        policy_label: String,
        driving: bool,
        inference_failed: bool,
        fallen: bool,
        limp: bool,
        gain_requested: u16,
        gain_applied: Option<u16>,
        targets_before_safety: [f64; NUM_JOINTS],
        limits: LimitFlags,
        write_ok: bool,
        tick_elapsed_ms: f64,
    ) -> Self {
        Self {
            record_type: "sample",
            servo_diagnostics: None,
            person_follow: None,
            target_follow: None,
            tick,
            tick_t_ns,
            sensors_t_ns,
            elapsed_s,
            fresh,
            sensors: sensors.map(SensorSample::from),
            policy_observation,
            policy_action,
            policy_output_t_ns,
            policy_imu_input: None,
            policy_gravity_input: None,
            apply_start_t_ns,
            apply_end_t_ns,
            command_requested,
            command_applied,
            policy_label,
            driving,
            inference_failed,
            fallen,
            limp,
            gain_requested,
            gain_applied,
            targets_before_safety,
            limits,
            write_ok,
            tick_elapsed_ms,
        }
    }
}

#[derive(Debug, Serialize)]
struct Footer {
    #[serde(rename = "type")]
    record_type: &'static str,
    schema: &'static str,
    ended_at_ns: u64,
    samples_written: u64,
    samples_dropped: u64,
}

/// The sender held by the control thread. [`Self::record`] never blocks.
#[derive(Clone)]
pub struct RecordSender {
    tx: SyncSender<RecordItem>,
    dropped: Arc<AtomicU64>,
    started: Arc<AtomicBool>,
    person_follow_only: bool,
    target_follow_only: bool,
    following: Arc<AtomicBool>,
    flush_pending: Arc<AtomicBool>,
}

enum RecordItem {
    Sample(Sample),
    Flush,
}

impl RecordSender {
    pub fn with_person_follow_only(mut self, enabled: bool) -> Self {
        self.person_follow_only = enabled;
        self
    }

    pub fn with_target_follow_only(mut self, enabled: bool) -> Self {
        self.target_follow_only = enabled;
        self
    }

    /// Called once per control tick; retries a nonblocking flush after following stops.
    pub fn should_record(&self, person_follow: bool, target_follow: bool) -> bool {
        if !self.person_follow_only && !self.target_follow_only { return true; }
        let active = (self.person_follow_only && person_follow) || (self.target_follow_only && target_follow);
        if self.following.swap(active, Ordering::Relaxed) && !active {
            self.flush_pending.store(true, Ordering::Relaxed);
        }
        if self.flush_pending.load(Ordering::Relaxed) {
            match self.tx.try_send(RecordItem::Flush) {
                Ok(()) | Err(TrySendError::Disconnected(_)) => self.flush_pending.store(false, Ordering::Relaxed),
                Err(TrySendError::Full(_)) => {},
            }
        }
        active
    }

    pub fn record(&self, sample: Sample) {
        if !self.started.load(Ordering::Relaxed) {
            if !self.person_follow_only && !self.target_follow_only && !sample.driving && sample.policy_gravity_input.is_none() {
                return;
            }
            if !self.started.swap(true, Ordering::Relaxed) {
                tracing::info!(
                    tick = sample.tick,
                    elapsed_s = sample.elapsed_s,
                    "comparison recording started"
                );
            }
        }
        match self.tx.try_send(RecordItem::Sample(sample)) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
            // The daemon is shutting down or the writer has reported a disk error. The record
            // cannot be recovered at this point, and logging per tick would create another load.
            Err(TrySendError::Disconnected(_)) => {}
        }
    }
}

/// Owns the writer thread. Finish it after joining the control thread so every queued sample is
/// drained before the footer is written.
pub struct Recorder {
    path: PathBuf,
    writer: thread::JoinHandle<io::Result<RecordSummary>>,
}

#[derive(Debug)]
pub struct RecordSummary {
    pub path: PathBuf,
    pub samples_written: u64,
    pub samples_dropped: u64,
}

impl Recorder {
    pub fn start(path: &Path, metadata: Metadata) -> io::Result<(RecordSender, Self)> {
        if let Some(parent) = path.parent().filter(|parent| !parent.as_os_str().is_empty()) {
            create_dir_all(parent)?;
        }
        let file = File::create(path)?;
        let path = path.to_owned();
        let mut writer = BufWriter::new(file);
        write_json_line(&mut writer, &metadata)?;
        writer.flush()?;

        let (tx, rx) = mpsc::sync_channel(CHANNEL_CAPACITY);
        let dropped = Arc::new(AtomicU64::new(0));
        let writer_dropped = Arc::clone(&dropped);
        let writer_path = path.clone();
        let writer_thread = thread::Builder::new()
            .name("comparison-record".to_owned())
            .spawn(move || write_samples(writer, rx, writer_path, writer_dropped))?;

        Ok((
            RecordSender {
                tx,
                dropped,
                started: Arc::new(AtomicBool::new(false)),
                person_follow_only: false,
                target_follow_only: false,
                following: Arc::new(AtomicBool::new(false)),
                flush_pending: Arc::new(AtomicBool::new(false)),
            },
            Self {
                path,
                writer: writer_thread,
            },
        ))
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn finish(self) -> io::Result<RecordSummary> {
        self.writer
            .join()
            .map_err(|_| io::Error::other("comparison record writer panicked"))?
    }
}

fn write_samples(
    mut writer: BufWriter<File>,
    rx: mpsc::Receiver<RecordItem>,
    path: PathBuf,
    dropped: Arc<AtomicU64>,
) -> io::Result<RecordSummary> {
    let mut samples_written = 0u64;
    let mut write_error = None;

    for item in rx {
        if write_error.is_none() {
            match item {
                RecordItem::Sample(sample) => match write_json_line(&mut writer, &sample) {
                    Ok(()) => samples_written += 1,
                    Err(error) => write_error = Some(error),
                },
                RecordItem::Flush => if let Err(error) = writer.flush() { write_error = Some(error); },
            }
        }
    }

    if let Some(error) = write_error {
        return Err(error);
    }

    let summary = RecordSummary {
        path,
        samples_written,
        samples_dropped: dropped.load(Ordering::Relaxed),
    };
    write_json_line(
        &mut writer,
        &Footer {
            record_type: "footer",
            schema: SCHEMA,
            ended_at_ns: duck_ipc_proto::clock::realtime_ns(),
            samples_written: summary.samples_written,
            samples_dropped: summary.samples_dropped,
        },
    )?;
    writer.flush()?;
    writer.get_ref().sync_all()?;
    Ok(summary)
}

fn write_json_line<T: Serialize>(writer: &mut BufWriter<File>, value: &T) -> io::Result<()> {
    serde_json::to_writer(&mut *writer, value).map_err(io::Error::other)?;
    writer.write_all(b"\n")
}

/// `serde` implements arrays only up to 32 elements. Observations are fixed at 61 by the policy
/// contract, so expose this one as an ordinary JSON array without allocating a `Vec`.
fn serialize_optional_f32_array<const N: usize, S>(
    values: &Option<[f32; N]>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    values
        .as_ref()
        .map(|values| values.as_slice())
        .serialize(serializer)
}
