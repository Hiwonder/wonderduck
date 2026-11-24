//! Shared camera-sample timing and control state for coordinated tracking.

use std::collections::VecDeque;
use std::sync::LazyLock;
use std::time::Duration;

use duck_ipc_proto::{CoordinatedTrackingParams, TrackingCamera, VisualTrackingTarget};
use kinematics::visual_tracking::{CameraModel, VisualKinematics, VisualTrackingError};
use kinematics::Quat;
use serde::Serialize;

use crate::head_tracking::{HeadReturn, TRACKING_LIMITS};
use crate::motion_tracking::MotionTracker;

// Retain feedback long enough for asynchronous recognition without allocating in a tick.
const FEEDBACK_CAPACITY: usize = 4096;
const ORIENTATION_MAX_GAP_US: u64 = 100_000;
const YAW_DIFFERENCE_STEP: f64 = 1e-5;
const TRANSLATION_DIFFERENCE_STEP: f64 = 1e-5;
pub(crate) const MIN_BODY_SPEED: [f64; 3] = [0.06, 0.06, 0.4];
// Target tracking acts on camera frames, not control ticks. Limit its head motion and
// filter its frame-to-frame derivative before it can amplify camera shake.
const TARGET_HEAD_DERIVATIVE_TAU_S: f64 = 0.10;
pub(crate) const TARGET_HEAD_SPEED_RAD_S: [f64; 2] = [
    60.0_f64.to_radians(),
    60.0_f64.to_radians(),
];
const TARGET_HEAD_MAX_FRAME_S: f64 = 0.10;
pub(crate) const TARGET_FINAL_PITCH_RAW: u16 = 1034;
pub(crate) const TARGET_APPROACH_SLOWDOWN_START_RAW: u16 = 1800;
const TARGET_PREPOSITION_START_RAW: u16 = 1234;
// A prepositioned head may settle just short of its target because of servo deadzone.
const TARGET_PREPOSITION_PITCH_TOLERANCE_RAW: u16 = 2;
const TARGET_PREPOSITION_ENTER_MARGIN_PX: f64 = 5.0;
const TARGET_PREPOSITION_HOLD_MARGIN_PX: f64 = 10.0;
const TARGET_FINAL_POINT_TAU_S: f64 = 0.25;
const TARGET_FINAL_POINT_MAX_GAP_US: u64 = 600_000;

const IDENTIFIED_MODEL: &str = include_str!("../assets/identified-robot.xml");

/// The user-confirmed upright optical frame: right/down/forward to head forward/left/up.
pub fn geometry() -> Result<&'static VisualKinematics, &'static VisualTrackingError> {
    static GEOMETRY: LazyLock<Result<VisualKinematics, VisualTrackingError>> = LazyLock::new(|| {
        VisualKinematics::new(IDENTIFIED_MODEL, Quat::new(0.5, -0.5, 0.5, -0.5))
    });
    GEOMETRY.as_ref()
}

#[derive(Clone, Copy, Debug)]
struct Feedback {
    unix_us: u64,
    joints: [f64; 4],
    orientation: Option<Orientation>,
}

#[derive(Clone, Copy, Debug)]
struct Orientation {
    trunk_to_world: Quat,
    epoch: u64,
}

pub struct FeedbackHistory {
    samples: VecDeque<Feedback>,
    generation: u64,
    orientation_epoch: u64,
}

impl Default for FeedbackHistory {
    fn default() -> Self {
        Self { samples: VecDeque::with_capacity(FEEDBACK_CAPACITY), generation: 0, orientation_epoch: 0 }
    }
}

impl FeedbackHistory {
    pub fn clear(&mut self) {
        self.samples.clear();
        self.generation = self.generation.wrapping_add(1);
        self.orientation_epoch = self.orientation_epoch.wrapping_add(1);
    }



    /// Record the bus-read time in the media clock domain, plus the IMU orientation.
    /// Invalid or discontinuous orientations never invalidate person-follow joint feedback.
    pub fn push_with_orientation(&mut self, unix_us: u64, joints: [f64; 4], quaternion: [f64; 4]) {
        if !joints.iter().all(|value| value.is_finite()) { return; }
        if self.samples.back().is_some_and(|sample| unix_us <= sample.unix_us) {
            self.clear();
        }
        let norm_sq: f64 = quaternion.iter().map(|value| value * value).sum();
        let valid = quaternion.iter().all(|value| value.is_finite())
            && norm_sq.is_finite() && (0.25..=4.0).contains(&norm_sq);
        let orientation = if valid {
            let q = Quat::new(quaternion[0], quaternion[1], quaternion[2], quaternion[3]).normalized();
            if let Some(previous) = self.samples.back().and_then(|sample| {
                sample.orientation.map(|orientation| (sample.unix_us, orientation))
            }) {
                let gap = unix_us - previous.0;
                let dot = quat_dot(q, previous.1.trunk_to_world).abs().clamp(0.0, 1.0);
                let angle = 2.0 * dot.acos();
                // A reference reset or implausible jump cannot be interpolated through.
                if gap > ORIENTATION_MAX_GAP_US || angle > 0.35 + 8.0 * gap as f64 * 1e-6 {
                    self.orientation_epoch = self.orientation_epoch.wrapping_add(1);
                }
            }
            Some(Orientation { trunk_to_world: q, epoch: self.orientation_epoch })
        } else {
            self.orientation_epoch = self.orientation_epoch.wrapping_add(1);
            None
        };
        if self.samples.len() == FEEDBACK_CAPACITY { self.samples.pop_front(); }
        self.samples.push_back(Feedback { unix_us, joints, orientation });
    }



    fn feedback_at(&self, unix_us: u64) -> Option<Feedback> {
        let mut next = None;
        for sample in self.samples.iter().rev() {
            if sample.unix_us == unix_us { return Some(*sample); }
            if sample.unix_us < unix_us {
                let after: Feedback = next?;
                let fraction = (unix_us - sample.unix_us) as f64
                    / (after.unix_us - sample.unix_us) as f64;
                let joints = std::array::from_fn(|axis| {
                    sample.joints[axis] + fraction * (after.joints[axis] - sample.joints[axis])
                });
                let orientation = match (sample.orientation, after.orientation) {
                    (Some(before), Some(after_orientation))
                        if before.epoch == after_orientation.epoch
                            && after.unix_us - sample.unix_us <= ORIENTATION_MAX_GAP_US => {
                        Some(Orientation { trunk_to_world: interpolate_quat(
                            before.trunk_to_world, after_orientation.trunk_to_world, fraction), epoch: before.epoch })
                    }
                    _ => None,
                };
                return Some(Feedback { unix_us, joints, orientation });
            }
            next = Some(*sample);
        }
        None
    }

    fn is_after_latest(&self, unix_us: u64) -> bool {
        self.samples.back().is_none_or(|sample| unix_us > sample.unix_us)
    }
}

fn quat_dot(a: Quat, b: Quat) -> f64 {
    a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
}

fn interpolate_quat(a: Quat, mut b: Quat, fraction: f64) -> Quat {
    if quat_dot(a, b) < 0.0 { b = Quat::new(-b.w, -b.x, -b.y, -b.z); }
    Quat::new(a.w + fraction * (b.w - a.w), a.x + fraction * (b.x - a.x),
        a.y + fraction * (b.y - a.y), a.z + fraction * (b.z - a.z)).normalized()
}

fn reorient_ray(ray: [f64; 3], from: Orientation, to: Orientation) -> Option<[f64; 3]> {
    (from.epoch == to.epoch).then(|| {
        if from.trunk_to_world == to.trunk_to_world { ray }
        else { to.trunk_to_world.conjugate().rotate(from.trunk_to_world.rotate(ray)) }
    })
}

#[derive(Clone, Copy, Default)]
struct Axis {
    integral: f64,
    sample: Option<(f64, u64)>,
    derivative: f64,
}

impl Axis {


    fn step_with_filter(&mut self, error: f64, dt: f64, sample: Option<(f64, u64)>, gains: [f64; 3],
        derivative_tau_s: f64) -> Option<f64> {
        if let Some((sample_error, stamp)) = sample {
            if let Some((previous, previous_stamp)) = self.sample {
                if stamp <= previous_stamp { return None; }
                let sample_dt = (stamp - previous_stamp) as f64 * 1e-6;
                let derivative = (sample_error - previous) / sample_dt;
                if derivative_tau_s > 0.0 {
                    let alpha = 1.0 - (-sample_dt / derivative_tau_s).exp();
                    self.derivative += alpha * (derivative - self.derivative);
                } else {
                    self.derivative = derivative;
                }
            } else {
                self.derivative = 0.0;
            }
            self.sample = Some((sample_error, stamp));
        }
        self.integral += error * dt;
        let output = gains[0] * error + gains[1] * self.integral + gains[2] * self.derivative;
        output.is_finite().then_some(output)
    }

}

#[derive(Clone, Copy)]
struct Frame {
    target: VisualTrackingTarget,
    camera: TrackingCamera,
    capture_joints: [f64; 4],
    capture_orientation: Option<Orientation>,
    ray: [f64; 3],
}

impl Frame {
    fn ray_now(self, history: &FeedbackHistory) -> Option<[f64; 3]> {
        reorient_ray(self.ray, self.capture_orientation?, history.samples.back()?.orientation?)
    }
}

#[derive(Clone, Copy)]
struct Search {
    elapsed: f64,
    from: [f64; 2],
}

#[derive(Clone, Copy)]
struct FollowRestart {
    direction: [i8; 2],
    elapsed: f64,
    last_frame_us: u64,
    fresh_frames: u8,
}

#[derive(Clone, Copy)]
struct FollowArea {
    value_px: f64,
    captured_at_us: u64,
    camera: TrackingCamera,
}

#[derive(Clone, Copy)]
struct FinalPointFilter {
    point: [f64; 2],
    captured_at_us: u64,
    camera: TrackingCamera,
    goal: [f64; 2],
    tolerance: [f64; 2],
}

#[derive(Clone, Copy)]
enum BodyPhase { Approach, Alignment }

#[derive(Clone, Copy)]
struct ArrivalConfirmation {
    start_us: Option<u64>,
    elapsed: Duration,
    since_frame: Duration,
    final_point: [f64; 2],
    final_tolerance: [f64; 2],
    camera: TrackingCamera,
    feedback_generation: u64,
}

#[derive(Clone, Copy)]
struct Arrival {
    alignment_joints: [f64; 4],
    alignment_orientation: Option<Orientation>,
    motion: HeadReturn,
    home_after_us: Option<u64>,
    last_checked_us: u64,
    projection_failed: bool,
    feedback_generation: u64,
    action_selected: bool,
    delay_s: f64,
    delay_elapsed: Option<f64>,
    stand_confirmed: bool,
    action: ArrivalAction,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ArrivalAction {
    Waiting,
    StandWritten,
    Ready,
    Running,
    ReturningHead,
    Finished,
}

#[derive(Clone, Copy)]
pub struct TargetGeometry {
    pub captured_at_unix_us: u64,
    pub camera: TrackingCamera,
    pub bearing_rad: Option<f64>,
    pub point_px: [f64; 2],
    pub pitch_raw: u16,
}

#[derive(Clone, Copy)]
pub struct Tracker {
    head: [Axis; 2],
    follow_body: MotionTracker,
    head_target: Option<[f64; 2]>,
    head_in_deadzone: bool,
    head_deadzone_target: Option<[f64; 2]>,
    final_prepositioning: bool,
    frame: Option<Frame>,
    target_ray_now: Option<[f64; 3]>,
    target_point_now_px: Option<[f64; 2]>,
    target_rotation_since_capture_rad: Option<f64>,
    last_twist: [f64; 3],
    last_deadzone: [bool; 3],
    search: Option<Search>,
    body_phase: Option<BodyPhase>,
    follow_stopped: bool,
    follow_settled: bool,
    follow_restart: Option<FollowRestart>,
    follow_area: Option<FollowArea>,
    final_point_filter: Option<FinalPointFilter>,
    tof_distance_mm: Option<u32>,
    confirmation: Option<ArrivalConfirmation>,
    target_motion_settled: bool,
    home: [f64; 2],
    arrival: Option<Arrival>,
}

/// The state actually retained by the following controller after this control tick.
#[derive(Debug, Serialize)]
pub struct PersonFollowDiagnostic {
    accepted_target: Option<VisualTrackingTarget>,
    distance_source: Option<&'static str>,
    distance_error: Option<f64>,
    distance_in_deadzone: Option<bool>,
    position_in_window: Option<[bool; 2]>,
    follow_stopped: bool,
    restart_pending_s: Option<f64>,
    filtered_area_px: Option<f64>,
    search_elapsed_s: Option<f64>,
    last_twist: [f64; 3],
}

/// Read-only state for diagnosing color-target tracking after a control tick.
#[derive(Debug, Serialize)]
pub struct TargetFollowDiagnostic {
    accepted_target: Option<VisualTrackingTarget>,
    capture_joints_rad: Option<[f64; 4]>,
    compensated_point_px: Option<[f64; 2]>,
    filtered_final_point_px: Option<[f64; 2]>,
    trunk_rotation_since_capture_rad: Option<f64>,
    head_target_rad: Option<[f64; 2]>,
    head_in_deadzone: bool,
    final_prepositioning: bool,
    body_phase: Option<&'static str>,
    search_elapsed_s: Option<f64>,
    confirmation_elapsed_s: Option<f64>,
    motion_settled: bool,
    last_twist: [f64; 3],
    last_deadzone: [bool; 3],
}

impl Default for Tracker {
    fn default() -> Self {
        let home = duck_control::hx::home_target();
        Self {
            head: [Axis::default(); 2], follow_body: MotionTracker::default(),
            head_target: None, head_in_deadzone: false, head_deadzone_target: None,
            final_prepositioning: false,
            frame: None, target_ray_now: None, target_point_now_px: None,
            target_rotation_since_capture_rad: None,
            last_twist: [0.0; 3], last_deadzone: [false; 3], search: None, body_phase: None,
            follow_stopped: false, follow_settled: false, follow_restart: None, follow_area: None,
            final_point_filter: None, tof_distance_mm: None,
            confirmation: None, target_motion_settled: true,
            home: [home[6], home[7]], arrival: None,
        }
    }
}

// The reference miss loop sleeps for 10 ms in the branch and another 10 ms at its end.
const SEARCH_WAIT: f64 = 2.02;
const SEARCH_WAYPOINT_SECONDS: f64 = 1.0;
// A person already at the target should not restart for a single noisy detection frame.
const FOLLOW_RESTART_CONFIRM_S: f64 = 0.3;
const FOLLOW_AREA_FILTER_TAU_S: f64 = 0.2;
const FOLLOW_AREA_FILTER_MAX_GAP_US: u64 = 600_000;
const ARRIVAL_CONFIRMATION: Duration = Duration::from_millis(500);
const ARRIVAL_CONFIRMATION_US: u64 = 500_000;
pub const INITIAL_PITCH: u16 = 1500;
// Match the five-degree measured-head tolerance used by obstacle walking.
// One HX revolution is 4096 counts; 56 counts remains within five degrees.
pub const INITIAL_PITCH_TOLERANCE: u16 = 56;
const INITIAL_HEAD_SETTLE_S: f64 = 0.5;
// Reference turn output can reach 9 after truncation, despite its nominal range of 8.
const REFERENCE_TURN_PEAK: f64 = 9.0;
// The reference's 150 counts over a 400-count pan range, mapped to 512 counts.
const REFERENCE_PAN_CENTER_WINDOW: u16 = 192;
fn final_point(pixels: [f64; 2]) -> [f64; 2] {
    [pixels[0] / 160.0 - 1.0, pixels[1] / 90.0 - 1.0]
}

fn final_tolerance(pixels: [f64; 2]) -> [f64; 2] {
    [pixels[0] / 160.0, pixels[1] / 90.0]
}

#[derive(Clone, Copy)]
struct FinalWindow {
    goal: [f64; 2],
    tolerance: [f64; 2],
    lower: [f64; 2],
    upper: [f64; 2],
}

impl FinalWindow {
    fn new(goal: [f64; 2], tolerance: [f64; 2]) -> Self {
        Self {
            goal: final_point(goal),
            tolerance: final_tolerance(tolerance),
            lower: final_point(std::array::from_fn(|axis| goal[axis] - tolerance[axis])),
            upper: final_point(std::array::from_fn(|axis| goal[axis] + tolerance[axis])),
        }
    }

    fn contains_axis(self, point: f64, axis: usize) -> bool {
        // Include pixel endpoints even when normalization rounds the two forms differently.
        (point - self.goal[axis]).abs() <= self.tolerance[axis]
            || (point >= self.lower[axis] && point <= self.upper[axis])
    }

    fn contains(self, point: [f64; 2]) -> bool {
        (0..2).all(|axis| self.contains_axis(point[axis], axis))
    }
}

fn search_waypoint(index: usize) -> [f64; 2] {
    // Reference order is left/down, left/up, center/up, right/up, right/down.
    let (pan, tilt) = [(650.0, 300.0), (650.0, 500.0), (500.0, 500.0),
        (350.0, 500.0), (350.0, 300.0)][index];
    let pitch = (1024.0_f64 + (tilt - 260.0) * 976.0 / 240.0).round() as u16;
    let yaw = (2048.0_f64 - (pan - 500.0) * 512.0 / 375.0).round() as u16;
    [
        duck_control::hx::decode_position(6, &pitch.to_le_bytes())
            .expect("search pitch must be valid"),
        duck_control::hx::decode_position(7, &yaw.to_le_bytes())
            .expect("search yaw must be valid"),
    ]
}

/// Existing full-deflection walking commands, with signed bounds per direction.
#[derive(Clone, Copy)]
pub struct MotionLimits {
    pub forward: [f64; 2],
    pub lateral: [f64; 2],
    pub yaw: [f64; 2],
}

impl MotionLimits {
    fn valid(self) -> bool {
        [self.forward, self.lateral, self.yaw].into_iter().all(|[min, max]| {
            min.is_finite() && max.is_finite() && min <= 0.0 && max >= 0.0
        })
    }

    fn scale(self, forward: f64, lateral: f64, yaw: f64) -> [f64; 3] {
        let scale = |value: f64, [min, max]: [f64; 2]| {
            if value >= 0.0 { value * max } else { -value * min }
        };
        [scale(forward, self.forward), scale(lateral, self.lateral), scale(yaw, self.yaw)]
    }

    fn with_linear_limits(mut self, limits: [Option<f64>; 2]) -> Self {
        for (bounds, limit) in [&mut self.forward, &mut self.lateral].into_iter().zip(limits) {
            if let Some(limit) = limit {
                bounds[0] = bounds[0].max(-limit);
                bounds[1] = bounds[1].min(limit);
            }
        }
        self
    }
}

fn approach_error(
    geometry: &VisualKinematics,
    camera: &CameraModel,
    ray: [f64; 3],
    joints: [f64; 4],
    point: [f64; 2],
    window: FinalWindow,
) -> Result<f64, Error> {
    let low = duck_control::hx::decode_position(6, &TARGET_FINAL_PITCH_RAW.to_le_bytes())
        .map_err(|_| Error::InvalidParameters)?;
    let mut reference_joints = joints;
    reference_joints[1] = reference_joints[1].max(low);
    if reference_joints[1] == joints[1]
        && window.contains_axis(point[1], 1)
    {
        return Ok(0.0);
    }
    let angle = |ray: [f64; 3]| -> Result<f64, Error> {
        if ray[0] == 0.0 && ray[2] == 0.0 { return Err(Error::UnobservableTranslation); }
        Ok((-ray[2]).atan2(ray[0]))
    };
    let current = angle(ray)?;
    let mut differences = [0.0; 2];
    for (difference, y) in differences.iter_mut().zip([
        window.lower[1].max(-1.0),
        window.upper[1].min(1.0),
    ]) {
        let goal = geometry.ray_in_trunk(camera, [point[0].clamp(-1.0, 1.0), y], reference_joints)?;
        let delta = angle(goal)? - current;
        *difference = delta.sin().atan2(delta.cos());
    }
    let [first, second] = differences;
    Ok(if first > 0.0 && second > 0.0 { first.min(second) }
        else if first < 0.0 && second < 0.0 { first.max(second) }
        else { 0.0 })
}

fn turn_command(yaw: u16, direction: f64, gain: f64, limits: MotionLimits) -> f64 {
    let offset = yaw.abs_diff(2048);
    if offset <= 10 || direction == 0.0 { return 0.0; }
    let error = f64::from(offset - 10) * std::f64::consts::TAU / 4096.0;
    (gain * error * direction.signum()).clamp(limits.yaw[0], limits.yaw[1])
}

fn translation_response(
    geometry: &VisualKinematics,
    camera: &CameraModel,
    ray: [f64; 3],
    joints: [f64; 4],
    body_axis: usize,
) -> Result<[f64; 2], Error> {
    // At a fixed target, translating the camera moves the target ray oppositely.
    let mut forward = ray;
    let mut backward = ray;
    forward[body_axis] -= TRANSLATION_DIFFERENCE_STEP;
    backward[body_axis] += TRANSLATION_DIFFERENCE_STEP;
    let plus = geometry.project_to_image(camera, forward, joints)?;
    let minus = geometry.project_to_image(camera, backward, joints)?;
    Ok(std::array::from_fn(|axis| {
        (plus[axis] - minus[axis]) / (2.0 * TRANSLATION_DIFFERENCE_STEP)
    }))
}

fn final_alignment(
    geometry: &VisualKinematics,
    camera: &CameraModel,
    ray: [f64; 3],
    joints: [f64; 4],
    point: [f64; 2],
    window: FinalWindow,
    forward_command: f64,
    lateral_gain: f64,
    limits: MotionLimits,
) -> Result<([f64; 3], [bool; 3]), Error> {
    let error = window.goal[0] - point[0];
    if window.contains_axis(point[0], 0) {
        return Ok(([forward_command, 0.0, 0.0], [forward_command == 0.0, true, true]));
    }
    let desired = [(error.abs() - window.tolerance[0]).copysign(error), 0.0];
    let forward = translation_response(geometry, camera, ray, joints, 0)?;
    let lateral = translation_response(geometry, camera, ray, joints, 1)?;
    let determinant = forward[0] * lateral[1] - lateral[0] * forward[1];
    if !determinant.is_finite() || determinant == 0.0 {
        return Err(Error::UnobservableTranslation);
    }
    let body = [
        (lateral[1] * desired[0] - lateral[0] * desired[1]) / determinant,
        (forward[0] * desired[1] - forward[1] * desired[0]) / determinant,
    ];
    let length = body[0].abs().max(body[1].abs());
    if !length.is_finite() || length == 0.0 {
        return Err(Error::UnobservableTranslation);
    }
    let magnitude = desired[0].abs() * lateral_gain;
    let direction = body.map(|value| value / length);
    let requested = direction.map(|value| value * magnitude);
    let mut scale = 1.0_f64;
    for (value, [min, max]) in requested.into_iter().zip([limits.forward, limits.lateral]) {
        if value > max {
            scale = scale.min(max / value);
        } else if value < min {
            scale = scale.min(min / value);
        }
    }
    let twist = [requested[0] * scale, requested[1] * scale, 0.0];
    Ok((twist, [twist[0] == 0.0, twist[1] == 0.0, true]))
}

#[derive(Debug)]
pub enum Error {
    InvalidParameters,
    OutOfOrderFrame,
    SingularJacobian,
    UnobservableTranslation,
    Geometry(VisualTrackingError),
}

impl From<VisualTrackingError> for Error {
    fn from(error: VisualTrackingError) -> Self { Self::Geometry(error) }
}

impl std::fmt::Display for Error {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidParameters => formatter.write_str("invalid coordinated tracking values"),
            Self::OutOfOrderFrame => formatter.write_str("out-of-order or inconsistent camera frame"),
            Self::SingularJacobian => formatter.write_str("the head tracking Jacobian is singular"),
            Self::UnobservableTranslation => formatter.write_str("the image cannot resolve the body translation direction"),
            Self::Geometry(error) => write!(formatter, "{error}"),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Step {
    Holding { head: [f64; 2], missing_feedback: bool },
    Following { head: [f64; 2], twist: [f64; 3], deadzone: [bool; 3] },
}

pub struct InitialHead {
    initial_pitch: Option<u16>,
    target: Option<[f64; 2]>,
    last_command: Option<[f64; 2]>,
    correction: Option<HeadReturn>,
    settle_s: f64,
    wait_report_s: f64,
    arrived: bool,
    handed_off: bool,
}

impl Default for InitialHead {
    fn default() -> Self { Self::new(Some(INITIAL_PITCH)) }
}

#[cfg(test)]
mod initial_head_tests {
    use super::*;

    #[test]
    fn missing_initial_pitch_holds_the_first_feedback_position() {
        let pitch = duck_control::hx::decode_position(6, &2100_u16.to_le_bytes()).unwrap();
        let measured = [0.0, pitch, 0.2, 0.0];
        let mut initial_head = InitialHead::new(None);
        assert!(matches!(initial_head.step(measured, 0.02), Ok(Some(Step::Holding { .. }))));
        assert_eq!(initial_head.take_handoff(), Some([pitch, 0.2]));
        assert!(matches!(initial_head.step([0.0, 0.0, 0.3, 0.0], 0.02), Ok(None)));
        assert_eq!(initial_head.fixed_head(), Some([pitch, 0.2]));
    }
}

impl InitialHead {
    pub fn new(initial_pitch: Option<u16>) -> Self {
        Self { initial_pitch, target: None, last_command: None, correction: None, settle_s: 0.0, wait_report_s: 0.0,
            arrived: false, handed_off: false }
    }

    pub fn arrived(&self) -> bool { self.arrived }

    pub fn step(&mut self, joints: [f64; 4], dt: f64) -> Result<Option<Step>, Error> {
        if self.arrived { return Ok(None); }
        if !joints.iter().all(|value| value.is_finite()) || !dt.is_finite() || dt <= 0.0 {
            return Err(Error::InvalidParameters);
        }
        let pitch = duck_control::hx::encode_position(6, joints[1])
            .map_err(|_| Error::InvalidParameters)?;
        let requested_pitch = *self.initial_pitch.get_or_insert(pitch);
        let initial_pitch = duck_control::hx::decode_position(6, &requested_pitch.to_le_bytes())
            .map_err(|_| Error::InvalidParameters)?;
        let target = *self.target.get_or_insert([initial_pitch, joints[2]]);
        self.arrived = pitch.abs_diff(requested_pitch) <= INITIAL_PITCH_TOLERANCE;
        if self.arrived {
            tracing::info!(servo = 31, position = pitch, target = requested_pitch,
                "coordinated tracking initial head position reached");
        } else {
            self.wait_report_s += dt;
            if self.wait_report_s >= 5.0 {
                tracing::warn!(servo = 31, position = pitch, target = requested_pitch,
                    tolerance = INITIAL_PITCH_TOLERANCE,
                    "coordinated tracking waiting for initial head position");
                self.wait_report_s = 0.0;
            }
        }
        let head = if let (Some(motion), Some(last)) = (self.correction.as_ref(), self.last_command) {
            self.settle_s += dt;
            if !self.arrived && self.settle_s >= INITIAL_HEAD_SETTLE_S {
                // As in obstacle scanning, wait for the first command to settle and then
                // compensate persistent physical error. Keep the captured yaw unchanged.
                motion.next_with_feedback_tolerance([joints[1], target[1]], dt, INITIAL_PITCH_TOLERANCE)
            } else { last }
        } else { target };
        Ok(Some(Step::Holding { head, missing_feedback: false }))
    }

    pub fn confirm(&mut self, head: [f64; 2]) {
        if let Some(target) = self.target {
            let motion = self.correction.get_or_insert_with(|| HeadReturn::new(head, target));
            motion.confirm(head);
            self.last_command = Some(head);
        }
    }

    pub fn take_handoff(&mut self) -> Option<[f64; 2]> {
        if !self.arrived || self.handed_off { return None; }
        self.handed_off = true;
        self.last_command.or(self.target)
    }

    pub fn fixed_head(&self) -> Option<[f64; 2]> {
        self.arrived.then(|| self.last_command.or(self.target)).flatten()
    }

    pub fn pause(&mut self) {
        if !self.arrived {
            self.target = None;
            self.last_command = None;
            self.correction = None;
            self.settle_s = 0.0;
            self.wait_report_s = 0.0;
        }
    }
}

fn camera_model(camera: TrackingCamera) -> CameraModel {
    CameraModel {
        width: camera.width, height: camera.height, rotate: camera.rotate,
        fx: camera.fx, fy: camera.fy, cx: camera.cx, cy: camera.cy,
        distortion: camera.distortion,
    }
}

fn inside(error: f64, deadzone: f64) -> bool {
    deadzone > 0.0 && error.abs() <= deadzone
}

fn minimum_body_speed(mut twist: [f64; 3], limits: MotionLimits) -> [f64; 3] {
    let bounds = [limits.forward, limits.lateral, limits.yaw];
    for axis in 0..3 {
        if twist[axis] != 0.0 {
            let cap = if twist[axis] > 0.0 { bounds[axis][1] } else { -bounds[axis][0] };
            twist[axis] = twist[axis].abs().max(MIN_BODY_SPEED[axis]).min(cap).copysign(twist[axis]);
        }
    }
    twist
}

impl Tracker {
    pub fn seed_head(&mut self, head: [f64; 2]) {
        self.head_target = Some(head);
        self.head_in_deadzone = true;
        self.head_deadzone_target = Some(head);
        self.follow_stopped = true;
    }
    pub fn fix_head(&mut self, head: [f64; 2]) { self.head_target = Some(head); }
    pub fn with_home(home: [f64; 2]) -> Self { Self { home, ..Self::default() } }

    pub fn reset(&mut self) { *self = Self::with_home(self.home); }

    /// Arrival confirmation starts only after the target-follow velocity command has settled.
    pub fn set_target_motion_settled(&mut self, settled: bool) {
        self.target_motion_settled = settled;
    }

    /// Compare the captured ray with image center at physical head-yaw HOME, in the
    /// trunk frame, and reproject that ray through the current measured head pose.
    pub fn target_geometry(
        &self,
        geometry: &VisualKinematics,
        joints: [f64; 4],
    ) -> Result<Option<TargetGeometry>, Error> {
        let Some(frame) = self.frame else { return Ok(None) };
        let camera = camera_model(frame.camera);
        let Some(ray) = self.target_ray_now else { return Ok(None) };
        let Some(point_px) = self.target_point_now_px else { return Ok(None) };
        let mut reference_joints = joints;
        reference_joints[2] = self.home[1];
        let reference = geometry.ray_in_trunk(&camera, [0.0, 0.0], reference_joints)?;
        let bearing_rad = if ray[0].hypot(ray[1]) < 0.05 || reference[0].hypot(reference[1]) < 0.05 {
            None
        } else {
            let angle = ray[1].atan2(ray[0]) - reference[1].atan2(reference[0]);
            Some(angle.sin().atan2(angle.cos()))
        };
        Ok(Some(TargetGeometry {
            captured_at_unix_us: frame.target.captured_at_unix_us,
            camera: frame.camera,
            bearing_rad,
            point_px,
            pitch_raw: duck_control::hx::encode_position(6, joints[1]).map_err(|_| Error::InvalidParameters)?,
        }))
    }

    /// The caller supplies only a valid, fresh sensor reading; None selects image area.
    pub fn set_tof_distance(&mut self, distance_mm: Option<u32>) { self.tof_distance_mm = distance_mm; }

    pub fn target_follow_diagnostic(&self) -> TargetFollowDiagnostic {
        TargetFollowDiagnostic {
            accepted_target: self.frame.map(|frame| frame.target),
            capture_joints_rad: self.frame.map(|frame| frame.capture_joints),
            compensated_point_px: self.target_point_now_px,
            filtered_final_point_px: self.final_point_filter.map(|filter| {
                [(filter.point[0] + 1.0) * 160.0, (filter.point[1] + 1.0) * 90.0]
            }),
            trunk_rotation_since_capture_rad: self.target_rotation_since_capture_rad,
            head_target_rad: self.head_target,
            head_in_deadzone: self.head_in_deadzone,
            final_prepositioning: self.final_prepositioning,
            body_phase: self.body_phase.map(|phase| match phase {
                BodyPhase::Approach => "approach", BodyPhase::Alignment => "alignment",
            }),
            search_elapsed_s: self.search.map(|search| search.elapsed),
            confirmation_elapsed_s: self.confirmation.map(|confirmation| confirmation.elapsed.as_secs_f64()),
            motion_settled: self.target_motion_settled,
            last_twist: self.last_twist,
            last_deadzone: self.last_deadzone,
        }
    }

    /// Read diagnostic evidence only after `step`; this does not advance the controller.
    pub fn person_follow_diagnostic(&self, params: &CoordinatedTrackingParams) -> PersonFollowDiagnostic {
        let frame = self.frame;
        let range = frame.and_then(|frame| self.follow_range(params, frame));
        let window = FinalWindow::new(params.final_point, params.final_tolerance);
        PersonFollowDiagnostic {
            accepted_target: frame.map(|frame| frame.target),
            distance_source: range.map(|_| {
                if self.tof_distance_mm.is_some() && params.stop_distance_mm.is_some() { "tof" } else { "area" }
            }),
            distance_error: range.map(|(_, error)| error),
            distance_in_deadzone: range.map(|(reached, _)| reached),
            position_in_window: frame.map(|frame| [
                window.contains_axis(frame.target.point[0], 0),
                window.contains_axis(frame.target.point[1], 1),
            ]),
            follow_stopped: self.follow_stopped,
            restart_pending_s: self.follow_restart.map(|restart| restart.elapsed),
            filtered_area_px: self.follow_area.map(|area| area.value_px),
            search_elapsed_s: self.search.map(|search| search.elapsed),
            last_twist: self.last_twist,
        }
    }

    fn tof_range(distance: u32, target: u32, deadzone: u32) -> (bool, f64) {
        let difference = i64::from(distance) - i64::from(target);
        let deadzone = i64::from(deadzone);
        if difference.abs() <= deadzone {
            (true, 0.0)
        } else {
            let outside = difference - difference.signum() * deadzone;
            (false, outside as f64 / f64::from(target))
        }
    }

    fn follow_range(&self, params: &CoordinatedTrackingParams, frame: Frame) -> Option<(bool, f64)> {
        let stop_area = params.stop_area_px?;
        if let (Some(distance), Some(target)) = (self.tof_distance_mm, params.stop_distance_mm) {
            Some(Self::tof_range(distance, target, params.stop_distance_deadzone_mm))
        } else {
            let area = self.follow_area.filter(|_| params.stop_area_deadzone_px > 0)
                .map(|area| area.value_px.round() as u64)
                .unwrap_or_else(|| frame.target.area_px.expect("following targets require pixel area"));
            let difference = i128::from(stop_area) - i128::from(area);
            let deadzone = i128::from(params.stop_area_deadzone_px);
            if difference.abs() <= deadzone {
                Some((true, 0.0))
            } else {
                let outside = difference - difference.signum() * deadzone;
                Some((false, outside as f64 / stop_area as f64))
            }
        }
    }

    fn update_follow_area(&mut self, target: VisualTrackingTarget, camera: TrackingCamera) {
        let Some(raw_area) = target.area_px else { self.follow_area = None; return; };
        let stamp = target.captured_at_unix_us;
        let filtered = match self.follow_area {
            Some(previous) if previous.camera == camera && stamp > previous.captured_at_us
                && stamp - previous.captured_at_us <= FOLLOW_AREA_FILTER_MAX_GAP_US => {
                let dt = (stamp - previous.captured_at_us) as f64 * 1e-6;
                let alpha = 1.0 - (-dt / FOLLOW_AREA_FILTER_TAU_S).exp();
                previous.value_px + alpha * (raw_area as f64 - previous.value_px)
            },
            _ => raw_area as f64,
        };
        self.follow_area = Some(FollowArea { value_px: filtered, captured_at_us: stamp, camera });
    }

    fn filter_final_point(&mut self, point: [f64; 2], frame: Frame,
        params: &CoordinatedTrackingParams, window: FinalWindow) -> [f64; 2]
    {
        let stamp = frame.target.captured_at_unix_us;
        let filtered = match self.final_point_filter {
            Some(previous) if previous.camera == frame.camera
                && previous.goal == params.final_point && previous.tolerance == params.final_tolerance
                && stamp > previous.captured_at_us && stamp - previous.captured_at_us <= TARGET_FINAL_POINT_MAX_GAP_US => {
                let dt = (stamp - previous.captured_at_us) as f64 * 1e-6;
                let alpha = 1.0 - (-dt / TARGET_FINAL_POINT_TAU_S).exp();
                std::array::from_fn(|axis| previous.point[axis] + alpha * (point[axis] - previous.point[axis]))
            }
            _ => point,
        };
        self.final_point_filter = Some(FinalPointFilter { point: filtered, captured_at_us: stamp,
            camera: frame.camera, goal: params.final_point, tolerance: params.final_tolerance });
        std::array::from_fn(|axis| {
            if window.contains_axis(point[axis], axis) {
                // Releasing a corrected axis must not wait for the filter to settle.
                point[axis]
            } else if (filtered[axis] - window.goal[axis]) * (point[axis] - window.goal[axis]) < 0.0 {
                // A fast target crossing cannot make a stale filter drive the wrong way.
                window.goal[axis]
            } else {
                filtered[axis]
            }
        })
    }

    fn hold_following(&mut self, joints: [f64; 4]) -> Step {
        let measured = [
            joints[1].clamp(TRACKING_LIMITS[0].0, TRACKING_LIMITS[0].1),
            joints[2].clamp(TRACKING_LIMITS[1].0, TRACKING_LIMITS[1].1),
        ];
        let head = self.head_target.unwrap_or(measured);
        self.head_target = Some(head);
        self.head = [Axis::default(); 2];
        self.follow_body.reset();
        self.head_in_deadzone = false;
        self.body_phase = Some(BodyPhase::Approach);
        self.confirmation = None;
        self.follow_stopped = true;
        self.follow_settled = true;
        self.follow_restart = None;
        self.last_twist = [0.0; 3];
        self.last_deadzone = [true; 3];
        Step::Following { head, twist: [0.0; 3], deadzone: [true; 3] }
    }

    /// Confirm departures from a settled stop, while reacting immediately to a very close ToF target.
    fn wait_for_follow_restart(&mut self, params: &CoordinatedTrackingParams, frame: Frame, error: f64, dt: f64) -> bool {
        if !self.follow_settled { return false; }
        if let (Some(distance), Some(target)) = (self.tof_distance_mm, params.stop_distance_mm)
            && distance.saturating_add(params.stop_distance_deadzone_mm.saturating_mul(2)) < target
        {
            self.follow_settled = false;
            self.follow_restart = None;
            return false;
        }
        let window = FinalWindow::new(params.final_point, params.final_tolerance);
        let horizontal = frame.target.point[0] - window.goal[0];
        let direction = [
            if error > 0.0 { 1 } else if error < 0.0 { -1 } else { 0 },
            if window.contains_axis(frame.target.point[0], 0) { 0 }
            else if horizontal > 0.0 { 1 } else { -1 },
        ];
        let mut restart = self.follow_restart.filter(|restart| restart.direction == direction)
            .unwrap_or(FollowRestart { direction, elapsed: 0.0,
                last_frame_us: frame.target.captured_at_unix_us, fresh_frames: 1 });
        if frame.target.captured_at_unix_us > restart.last_frame_us {
            restart.last_frame_us = frame.target.captured_at_unix_us;
            restart.fresh_frames = restart.fresh_frames.saturating_add(1);
        }
        restart.elapsed += dt;
        let needs_camera_confirmation = direction[1] != 0 || (direction[0] != 0
            && (self.tof_distance_mm.is_none() || params.stop_distance_mm.is_none()));
        if restart.elapsed >= FOLLOW_RESTART_CONFIRM_S && (!needs_camera_confirmation || restart.fresh_frames >= 2) {
            self.follow_settled = false;
            self.follow_restart = None;
            false
        } else {
            self.follow_restart = Some(restart);
            self.follow_stopped = true;
            self.last_twist = [0.0; 3];
            self.last_deadzone = [true; 3];
            true
        }
    }

    pub fn homing(&self) -> bool {
        self.arrival.is_some_and(|arrival| arrival.home_after_us.is_none())
    }



    pub fn phase(&self) -> duck_ipc_proto::CoordinatedTrackingPhase {
        use duck_ipc_proto::CoordinatedTrackingPhase as Phase;
        if let Some(arrival) = self.arrival {
            return match arrival.action {
                ArrivalAction::Running => Phase::RunningAction,
                ArrivalAction::ReturningHead => Phase::ReturningHead,
                ArrivalAction::Waiting if arrival.home_after_us.is_none() => Phase::ReturningHead,
                ArrivalAction::Waiting => Phase::Standing,
                ArrivalAction::StandWritten | ArrivalAction::Ready => Phase::WaitingAction,
                ArrivalAction::Finished => Phase::HoldingStand,
            };
        }
        if self.confirmation.is_some() { return Phase::Confirming; }
        if let Some(search) = self.search {
            return if search.elapsed < SEARCH_WAIT { Phase::WaitingTarget } else { Phase::Searching };
        }
        match self.body_phase {
            Some(BodyPhase::Approach) => Phase::Approach,
            Some(BodyPhase::Alignment) => Phase::FinalAlignment,
            None => Phase::WaitingTarget,
        }
    }

    pub fn linear_limits(&self, params: &CoordinatedTrackingParams) -> [Option<f64>; 2] {
        match self.body_phase {
            Some(BodyPhase::Approach) => [params.approach_linear_limit; 2],
            Some(BodyPhase::Alignment) => [Some(params.final_linear_limit), Some(params.final_lateral_limit)],
            None => [None; 2],
        }
    }

    pub fn arrived(&self) -> bool { self.arrival.is_some() }

    pub fn confirm_stand_written(&mut self) {
        if let Some(arrival) = self.arrival.as_mut() && arrival.home_after_us.is_some() {
            if arrival.action == ArrivalAction::Waiting && arrival.action_selected {
                arrival.action = if arrival.delay_s == 0.0 {
                    ArrivalAction::Ready
                } else {
                    ArrivalAction::StandWritten
                };
                arrival.delay_elapsed = None;
            }
            if matches!(arrival.action, ArrivalAction::StandWritten | ArrivalAction::Ready) {
                arrival.stand_confirmed = true;
            }
        }
    }

    pub fn arrival_action_ready(&self) -> bool {
        self.arrival.is_some_and(|arrival| arrival.action == ArrivalAction::Ready)
    }

    pub fn arrival_action_preparing(&self) -> bool {
        self.arrival.is_some_and(|arrival| arrival.action_selected
            && matches!(arrival.action, ArrivalAction::StandWritten | ArrivalAction::Ready))
    }

    pub fn arrival_action_running(&self) -> bool {
        self.arrival.is_some_and(|arrival| arrival.action == ArrivalAction::Running)
    }

    pub fn arrival_action_finished(&self) -> bool {
        self.arrival.is_some_and(|arrival| arrival.action == ArrivalAction::Finished)
    }

    pub fn resolve_arrival_action(&mut self, started: bool) {
        if let Some(arrival) = self.arrival.as_mut()
            && arrival.action == ArrivalAction::Ready
        {
            arrival.action = if started { ArrivalAction::Running } else { ArrivalAction::Finished };
        }
    }

    pub fn finish_arrival_action(&mut self, measured: [f64; 2]) {
        if let Some(arrival) = self.arrival.as_mut()
            && arrival.action == ArrivalAction::Running
        {
            arrival.motion = HeadReturn::new(measured, self.home);
            arrival.action = ArrivalAction::ReturningHead;
        }
    }

    pub fn confirm_head(&mut self, position: [f64; 2]) {
        if let Some(arrival) = self.arrival.as_mut() { arrival.motion.confirm(position); }
    }

    fn start_arrival(
        &mut self,
        joints: [f64; 4],
        history: &FeedbackHistory,
        params: &CoordinatedTrackingParams,
    ) -> Step {
        tracing::info!("coordinated tracking target reached; returning head home");
        let head = [
            joints[1].clamp(TRACKING_LIMITS[0].0, TRACKING_LIMITS[0].1),
            joints[2].clamp(TRACKING_LIMITS[1].0, TRACKING_LIMITS[1].1),
        ];
        self.arrival = Some(Arrival {
            alignment_joints: joints,
            alignment_orientation: history.samples.back().and_then(|sample| sample.orientation),
            motion: HeadReturn::new(head, self.home),
            home_after_us: None,
            last_checked_us: 0,
            projection_failed: false,
            feedback_generation: history.generation,
            action_selected: !params.arrival_skill.is_empty(),
            delay_s: params.arrival_delay_s,
            delay_elapsed: None,
            stand_confirmed: false,
            action: ArrivalAction::Waiting,
        });
        self.head_target = Some(head);
        self.last_twist = [0.0; 3];
        self.last_deadzone = [true; 3];
        Step::Following { head, twist: [0.0; 3], deadzone: [true; 3] }
    }

    fn after_arrival(
        &mut self,
        geometry: &VisualKinematics,
        history: &FeedbackHistory,
        joints: [f64; 4],
        params: &CoordinatedTrackingParams,
        dt: f64,
    ) -> Result<Option<Step>, Error> {
        let arrival = self.arrival.as_mut().expect("arrival must exist");
        if arrival.feedback_generation != history.generation {
            arrival.feedback_generation = history.generation;
            if matches!(arrival.action, ArrivalAction::Waiting | ArrivalAction::StandWritten | ArrivalAction::Ready) {
                arrival.action = ArrivalAction::Waiting;
                arrival.delay_elapsed = None;
                arrival.stand_confirmed = false;
            }
        }
        arrival.action_selected = !params.arrival_skill.is_empty();
        arrival.delay_s = params.arrival_delay_s;
        if arrival.home_after_us.is_none() {
            if arrival.motion.arrived([joints[1], joints[2]])
                && let Some(sample) = history.samples.back()
            {
                arrival.home_after_us = Some(sample.unix_us);
                tracing::info!("coordinated tracking head reached home; standing");
            }
            let head = arrival.motion.next_with_feedback([joints[1], joints[2]], dt);
            return Ok(Some(Step::Following { head, twist: [0.0; 3], deadzone: [true; 3] }));
        }

        let head = arrival.motion.next_with_feedback([joints[1], joints[2]], dt);
        let hold = || Some(Step::Following { head, twist: [0.0; 3], deadzone: [true; 3] });
        if arrival.action == ArrivalAction::ReturningHead {
            if arrival.motion.arrived([joints[1], joints[2]])
                && let Some(sample) = history.samples.back()
            {
                arrival.home_after_us = Some(sample.unix_us);
                arrival.action = ArrivalAction::Finished;
            }
            return Ok(hold());
        }
        if matches!(arrival.action, ArrivalAction::Waiting | ArrivalAction::StandWritten | ArrivalAction::Ready) {
            if !arrival.motion.arrived([joints[1], joints[2]]) {
                arrival.home_after_us = None;
                arrival.action = ArrivalAction::Waiting;
                arrival.delay_elapsed = None;
                arrival.stand_confirmed = false;
                let head = arrival.motion.next_with_feedback([joints[1], joints[2]], dt);
                return Ok(Some(Step::Following { head, twist: [0.0; 3], deadzone: [true; 3] }));
            }
            if !arrival.action_selected {
                arrival.action = ArrivalAction::Finished;
            } else {
                if matches!(arrival.action, ArrivalAction::StandWritten | ArrivalAction::Ready) {
                    if arrival.stand_confirmed {
                        // The previous tick's dt includes time before its stand write.
                        let elapsed = arrival.delay_elapsed.map_or(0.0, |elapsed| elapsed + dt);
                        arrival.delay_elapsed = Some(elapsed);
                        arrival.action = if elapsed >= arrival.delay_s {
                            ArrivalAction::Ready
                        } else {
                            ArrivalAction::StandWritten
                        };
                        arrival.stand_confirmed = false;
                    } else {
                        arrival.action = ArrivalAction::Waiting;
                        arrival.delay_elapsed = None;
                    }
                }
                return Ok(hold());
            }
        }
        if arrival.action == ArrivalAction::Running || params.pickup_after.is_some() {
            return Ok(hold());
        }
        let Some(target) = params.target else { return Ok(hold()); };
        if target.captured_at_unix_us <= arrival.home_after_us.expect("home was confirmed")
            || target.captured_at_unix_us <= arrival.last_checked_us
        {
            return Ok(hold());
        }
        let Some(at_capture) = history.feedback_at(target.captured_at_unix_us) else { return Ok(hold()); };
        let Some((from, to)) = at_capture.orientation.zip(arrival.alignment_orientation)
            .filter(|(from, to)| from.epoch == to.epoch) else { return Ok(hold()); };
        arrival.last_checked_us = target.captured_at_unix_us;
        let camera = camera_model(params.camera.ok_or(Error::InvalidParameters)?);
        let projected = geometry.ray_in_trunk(&camera, target.point, at_capture.joints)
            .and_then(|ray| geometry.project_to_image(&camera,
                reorient_ray(ray, from, to).expect("orientation epochs were checked"), arrival.alignment_joints));
        let point = match projected {
            Ok(point) => {
                arrival.projection_failed = false;
                point
            }
            Err(error) => {
                if !arrival.projection_failed {
                    tracing::warn!(%error, "cannot compare the reacquired target after head homing");
                    arrival.projection_failed = true;
                }
                return Ok(hold());
            }
        };
        let window = FinalWindow::new(params.final_point, params.final_tolerance);
        if window.contains(point) {
            return Ok(hold());
        }
        tracing::info!("coordinated tracking target moved; resuming approach");
        self.reset();
        Ok(None)
    }

    fn hold(&mut self, joints: [f64; 4], missing_feedback: bool) -> Step {
        let measured = [
            joints[1].clamp(TRACKING_LIMITS[0].0, TRACKING_LIMITS[0].1),
            joints[2].clamp(TRACKING_LIMITS[1].0, TRACKING_LIMITS[1].1),
        ];
        let head = if self.frame.is_some() { measured } else { self.head_target.unwrap_or(measured) };
        self.reset();
        self.head_target = Some(head);
        Step::Holding { head, missing_feedback }
    }

    pub fn pause(&mut self, joints: [f64; 4]) -> Step {
        self.hold(joints, false)
    }

    pub fn pause_fixed(&mut self, head: [f64; 2], missing_feedback: bool) -> Step {
        self.reset();
        self.head_target = Some(head);
        Step::Holding { head, missing_feedback }
    }

    fn search(&mut self, joints: [f64; 4], dt: f64, limits: MotionLimits, fixed_head: bool) -> Result<Step, Error> {
        self.confirmation = None;
        self.follow_stopped = false;
        let elapsed = self.search.map_or(0.0, |search| search.elapsed) + dt;
        if !elapsed.is_finite() { return Err(Error::InvalidParameters); }
        let measured = [
            joints[1].clamp(TRACKING_LIMITS[0].0, TRACKING_LIMITS[0].1),
            joints[2].clamp(TRACKING_LIMITS[1].0, TRACKING_LIMITS[1].1),
        ];
        if self.search.is_none() {
            let from = self.head_target.unwrap_or(measured);
            self.frame = None;
            self.head = [Axis::default(); 2];
            self.head_in_deadzone = false;
            self.head_deadzone_target = None;
            self.final_prepositioning = false;
            self.final_point_filter = None;
            self.search = Some(Search { elapsed: 0.0, from });
            self.head_target = Some(from);
            if fixed_head { self.follow_body.reset(); self.follow_restart = None; }
        }
        let search = self.search.as_mut().expect("search was initialized");
        search.elapsed = elapsed;
        if search.elapsed < SEARCH_WAIT {
            let head = self.head_target.expect("search head was initialized");
            if self.last_twist == [0.0; 3] {
                return Ok(Step::Holding { head, missing_feedback: false });
            }
            return Ok(Step::Following { head, twist: self.last_twist, deadzone: self.last_deadzone });
        }

        if fixed_head {
            self.follow_settled = false;
            self.follow_restart = None;
            return Ok(Step::Following {
                head: self.head_target.expect("search head was initialized"),
                twist: limits.scale(0.0, 0.0, -5.0 / REFERENCE_TURN_PEAK),
                deadzone: [true, true, false],
            });
        }

        let elapsed = search.elapsed - SEARCH_WAIT;
        let index = ((elapsed / SEARCH_WAYPOINT_SECONDS).floor() % 5.0) as usize;
        let fraction = (elapsed % SEARCH_WAYPOINT_SECONDS) / SEARCH_WAYPOINT_SECONDS;
        let from = if elapsed < SEARCH_WAYPOINT_SECONDS {
            search.from
        } else {
            search_waypoint((index + 4) % 5)
        };
        let to = search_waypoint(index);
        let head = std::array::from_fn(|axis| from[axis] + fraction * (to[axis] - from[axis]));
        self.head_target = Some(head);
        Ok(Step::Following {
            head,
            twist: limits.scale(0.0, 0.0, -5.0 / REFERENCE_TURN_PEAK),
            deadzone: [true, true, false],
        })
    }

    /// A detection and both control outputs are advanced as one transaction.
    pub fn step(
        &mut self,
        geometry: &VisualKinematics,
        history: &FeedbackHistory,
        joints: [f64; 4],
        params: CoordinatedTrackingParams,
        dt: f64,
        limits: MotionLimits,
    ) -> Result<Step, Error> {
        if !params.valid() || !dt.is_finite() || dt <= 0.0
            || !joints.iter().all(|value| value.is_finite()) || !limits.valid()
        {
            return Err(Error::InvalidParameters);
        }
        if self.arrival.is_some()
            && let Some(step) = self.after_arrival(geometry, history, joints, &params, dt)?
        {
            return Ok(step);
        }
        let Some(target) = params.target else {
            if let (Some(_), Some(target_distance), Some(distance)) =
                (params.stop_area_px, params.stop_distance_mm, self.tof_distance_mm)
                && self.last_twist[0] != 0.0
            {
                // Without a target, a fresh range may only slow the previous motion.
                let limits = limits.with_linear_limits([params.approach_linear_limit; 2]);
                let (_, error) = Self::tof_range(distance, target_distance, params.stop_distance_deadzone_mm);
                let forward = (params.body_gains[0] * error).clamp(limits.forward[0], limits.forward[1]);
                let cap = minimum_body_speed([forward, 0.0, 0.0], limits)[0];
                self.last_twist[0] = if cap.signum() == self.last_twist[0].signum() {
                    self.last_twist[0].abs().min(cap.abs()).copysign(cap)
                } else { 0.0 };
                self.last_deadzone[0] = self.last_twist[0] == 0.0;
            }
            return if params.camera.is_some() {
                self.search(joints, dt, limits, params.stop_area_px.is_some())
            } else {
                Ok(if params.stop_area_px.is_some() && self.head_target.is_some() {
                    self.pause_fixed(self.head_target.expect("fixed head is present"), false)
                } else { self.hold(joints, false) })
            };
        };
        let camera = params.camera.ok_or(Error::InvalidParameters)?;
        let mut next = *self;
        next.search = None;
        if let Some(confirmation) = next.confirmation.as_mut() {
            if confirmation.final_point != params.final_point
                || confirmation.final_tolerance != params.final_tolerance
                || confirmation.camera != camera
                || confirmation.feedback_generation != history.generation
            {
                next.confirmation = None;
            } else if !next.target_motion_settled {
                // A visually stable target does not count while the body is still slowing.
                confirmation.start_us = None;
                confirmation.elapsed = Duration::ZERO;
                confirmation.since_frame = Duration::ZERO;
            } else {
                let duration = Duration::try_from_secs_f64(dt).map_err(|_| Error::InvalidParameters)?;
                confirmation.elapsed = confirmation.elapsed.checked_add(duration).ok_or(Error::InvalidParameters)?;
                confirmation.since_frame = confirmation.since_frame.checked_add(duration).ok_or(Error::InvalidParameters)?;
                if confirmation.since_frame >= ARRIVAL_CONFIRMATION {
                    confirmation.start_us = None;
                    confirmation.elapsed = Duration::ZERO;
                }
            }
        }
        let is_new = match next.frame {
            None => true,
            Some(frame) if target.captured_at_unix_us < frame.target.captured_at_unix_us => {
                return Err(Error::OutOfOrderFrame);
            }
            Some(frame) if target.captured_at_unix_us == frame.target.captured_at_unix_us => {
                if frame.target != target || frame.camera != camera { return Err(Error::OutOfOrderFrame); }
                false
            }
            Some(_) => true,
        };
        let mut accepted_new_frame = false;
        if is_new {
            if let Some(feedback) = history.feedback_at(target.captured_at_unix_us) {
                let at_capture = feedback.joints;
                if next.frame.is_some_and(|previous| previous.camera != camera) {
                    next.head = [Axis::default(); 2];
                    next.head_in_deadzone = false;
                    next.head_deadzone_target = None;
                    next.final_prepositioning = false;
                }
                let ray = geometry.ray_in_trunk(&camera_model(camera), target.point, at_capture)?;
                if params.stop_area_px.is_some() && params.stop_area_deadzone_px > 0 {
                    next.update_follow_area(target, camera);
                } else {
                    next.follow_area = None;
                }
                next.frame = Some(Frame { target, camera, capture_joints: at_capture,
                    capture_orientation: feedback.orientation, ray });
                accepted_new_frame = true;
            } else if next.frame.is_none() || !history.is_after_latest(target.captured_at_unix_us) {
                return Ok(if params.stop_area_px.is_some() && self.head_target.is_some() {
                    self.pause_fixed(self.head_target.expect("fixed head is present"), true)
                } else { self.hold(joints, true) });
            }
        }
        let frame = next.frame.ok_or(Error::InvalidParameters)?;
        // Person following uses its independent image/ToF controller unchanged.
        // Color-target geometry needs capture and current orientation in one IMU epoch.
        let (target_ray, target_point) = if params.stop_area_px.is_none() {
            let Some(ray) = frame.ray_now(history) else { return Ok(self.hold(joints, true)); };
            let point = if frame.capture_joints == joints && frame.ray == ray {
                frame.target.point
            } else {
                geometry.project_to_image(&camera_model(frame.camera), ray, joints)?
            };
            let current = history.samples.back().and_then(|sample| sample.orientation)
                .expect("ray_now requires current orientation");
            let capture = frame.capture_orientation.expect("ray_now requires capture orientation");
            next.target_ray_now = Some(ray);
            next.target_point_now_px = Some([(point[0] + 1.0) * 160.0, (point[1] + 1.0) * 90.0]);
            next.target_rotation_since_capture_rad = Some(2.0 * quat_dot(capture.trunk_to_world,
                current.trunk_to_world).abs().clamp(0.0, 1.0).acos());
            (Some(ray), Some(point))
        } else {
            next.target_ray_now = None;
            next.target_point_now_px = None;
            next.target_rotation_since_capture_rad = None;
            (None, None)
        };
        if !accepted_new_frame {
            let head = next.head_target.ok_or(Error::InvalidParameters)?;
            if let Some((reached, error)) = next.follow_range(&params, frame) {
                let window = FinalWindow::new(params.final_point, params.final_tolerance);
                if reached && window.contains(frame.target.point) {
                    let step = next.hold_following(joints);
                    *self = next;
                    return Ok(step);
                }
                let was_settled = next.follow_settled;
                if next.wait_for_follow_restart(&params, frame, error, dt) {
                    *self = next;
                    return Ok(Step::Following { head, twist: [0.0; 3], deadzone: [true; 3] });
                }
                next.follow_stopped = false;
                let limits = limits.with_linear_limits([params.approach_linear_limit; 2]);
                let forward = (params.body_gains[0] * error).clamp(limits.forward[0], limits.forward[1]);
                next.last_twist[0] = minimum_body_speed([forward, 0.0, 0.0], limits)[0];
                next.last_deadzone[0] = next.last_twist[0] == 0.0;
                if was_settled {
                    let horizontal = frame.target.point[0] - window.goal[0];
                    let yaw = if window.contains_axis(frame.target.point[0], 0) { 0.0 }
                        else { (-params.body_gains[2] * horizontal).clamp(limits.yaw[0], limits.yaw[1]) };
                    next.last_twist[2] = minimum_body_speed([0.0, 0.0, yaw], limits)[2];
                    next.last_deadzone[2] = next.last_twist[2] == 0.0;
                }
            }
            *self = next;
            return Ok(Step::Following {
                head, twist: self.last_twist, deadzone: self.last_deadzone,
            });
        }
        let frame_dt = self.frame.map_or(dt, |previous| {
            (frame.target.captured_at_unix_us - previous.target.captured_at_unix_us) as f64 * 1e-6
        });
        if let Some((reached, distance_error)) = next.follow_range(&params, frame) {
            let window = FinalWindow::new(params.final_point, params.final_tolerance);
            let measured = [joints[1].clamp(TRACKING_LIMITS[0].0, TRACKING_LIMITS[0].1),
                            joints[2].clamp(TRACKING_LIMITS[1].0, TRACKING_LIMITS[1].1)];
            let head = next.head_target.unwrap_or(measured);
            next.head_target = Some(head);
            next.body_phase = Some(BodyPhase::Approach);
            if reached && window.contains(frame.target.point) {
                let step = next.hold_following(joints);
                *self = next;
                return Ok(step);
            }
            if next.wait_for_follow_restart(&params, frame, distance_error, dt) {
                *self = next;
                return Ok(Step::Following { head, twist: [0.0; 3], deadzone: [true; 3] });
            }
            next.follow_stopped = false;
            let output = next.follow_body.step(
                Some([frame.target.point[0] - window.goal[0], 1.0 - distance_error]), 1.0,
                [0.0, params.final_tolerance[0] / 320.0],
                [[params.body_gains[0], 0.0, 0.0], [params.body_gains[2], 0.0, 0.0]], frame_dt,
            ).ok_or(Error::InvalidParameters)?;
            let limits = limits.with_linear_limits([params.approach_linear_limit; 2]);
            let twist = minimum_body_speed([
                output.twist[0].clamp(limits.forward[0], limits.forward[1]),
                0.0,
                output.twist[2].clamp(limits.yaw[0], limits.yaw[1]),
            ], limits);
            next.last_twist = twist;
            next.last_deadzone = [twist[0] == 0.0, true, twist[2] == 0.0];
            *self = next;
            return Ok(Step::Following { head, twist, deadzone: self.last_deadzone });
        }
        let cam = camera_model(frame.camera);
        let ray = target_ray.expect("target tracking has a compensated ray");
        let point = target_point.expect("target tracking has a compensated point");
        let window = FinalWindow::new(params.final_point, params.final_tolerance);
        if let Some((reached, _)) = next.follow_range(&params, frame)
            && reached
            && window.contains(frame.target.point)
        {
            let step = next.hold_following(joints);
            *self = next;
            return Ok(step);
        }
        next.follow_stopped = false;
        let jacobian = geometry.image_jacobian(&cam, ray, joints)?;
        let determinant = jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0];
        if !determinant.is_finite() || determinant == 0.0 { return Err(Error::SingularJacobian); }
        let inverse = [
            [jacobian[1][1] / determinant, -jacobian[0][1] / determinant],
            [-jacobian[1][0] / determinant, jacobian[0][0] / determinant],
        ];
        if !inverse.iter().flatten().all(|value| value.is_finite()) {
            return Err(Error::SingularJacobian);
        }
        // A body turn rotates the tracked ray in the opposite direction in trunk coordinates.
        let body_image = {
            let rotate = |angle| Quat::from_axis_angle([0.0, 0.0, 1.0], angle).rotate(ray);
            let plus = geometry.project_to_image(&cam, rotate(-YAW_DIFFERENCE_STEP), joints)?;
            let minus = geometry.project_to_image(&cam, rotate(YAW_DIFFERENCE_STEP), joints)?;
            std::array::from_fn::<_, 2, _>(|axis| {
                (plus[axis] - minus[axis]) / (2.0 * YAW_DIFFERENCE_STEP)
            })
        };
        let head_yaw_per_body_yaw = -(inverse[1][0] * body_image[0] + inverse[1][1] * body_image[1]);
        // Continuous following aims at the configured image position while it approaches.
        let goal = if params.stop_area_px.is_some() { window.goal } else { [0.0; 2] };
        let head_errors = [point[1] - goal[1], point[0] - goal[0]];
        let head_deadzone_limit = if params.stop_area_px.is_some() {
            [params.final_tolerance[1] / 180.0, params.final_tolerance[0] / 320.0]
        } else { [f64::INFINITY; 2] };
        let head_deadzone = [
            inside(head_errors[0] / 2.0, params.head_deadzone[0].min(head_deadzone_limit[0])),
            inside(head_errors[1] / 2.0, params.head_deadzone[1].min(head_deadzone_limit[1])),
        ];
        let stamp = frame.target.captured_at_unix_us;
        // PID order is pitch/yaw, while the image Jacobian uses x/y rows.
        let head_sample_errors = [point[1] - goal[1], point[0] - goal[0]];
        let previous_head = next.head;
        let mut head_output = [0.0; 2];
        for axis in 0..2 {
            if head_deadzone[axis] {
                next.head[axis] = Axis::default();
            } else {
                let sample = Some((head_sample_errors[axis], stamp));
                head_output[axis] = next.head[axis].step_with_filter(head_errors[axis], frame_dt, sample,
                    params.head_gains[axis], TARGET_HEAD_DERIVATIVE_TAU_S)
                    .ok_or(Error::InvalidParameters)?;
            }
        }
        let measured = [
            joints[1].clamp(TRACKING_LIMITS[0].0, TRACKING_LIMITS[0].1),
            joints[2].clamp(TRACKING_LIMITS[1].0, TRACKING_LIMITS[1].1),
        ];
        let both_in_deadzone = head_deadzone.into_iter().all(|value| value);
        let previous_target = next.head_target.unwrap_or(measured);
        let mut head = if both_in_deadzone {
            if !next.head_in_deadzone { next.head_deadzone_target = Some(measured); }
            next.head_deadzone_target.unwrap_or(previous_target)
        } else {
            next.head_deadzone_target = None;
            previous_target
        };
        next.head_in_deadzone = both_in_deadzone;
        let current_pitch = duck_control::hx::encode_position(6, joints[1]).map_err(|_| Error::InvalidParameters)?;
        let current_yaw = duck_control::hx::encode_position(7, head[1]).map_err(|_| Error::InvalidParameters)?;
        let final_pitch_reached = current_pitch <= TARGET_FINAL_PITCH_RAW
            || (next.final_prepositioning
                && current_pitch <= TARGET_FINAL_PITCH_RAW + TARGET_PREPOSITION_PITCH_TOLERANCE_RAW);
        let center_pan = params.stop_area_px.is_none() && final_pitch_reached
            && current_yaw.abs_diff(2048) < REFERENCE_PAN_CENTER_WINDOW
            && point[1] >= (params.final_point[1] - 70.0_f64.max(params.final_tolerance[1])) / 90.0 - 1.0;
        // A large arrival window can already contain the captured target at the final
        // head pose while the measured pitch is still above that pose. In that case
        // approach_error is zero, but the old head PID can keep chasing image center
        // forever and never enter final alignment. Move to the final view while stopped.
        let measured_yaw = duck_control::hx::encode_position(7, joints[2]).map_err(|_| Error::InvalidParameters)?;
        let mut final_joints = joints;
        final_joints[1] = duck_control::hx::decode_position(6, &TARGET_FINAL_PITCH_RAW.to_le_bytes())
            .map_err(|_| Error::InvalidParameters)?;
        final_joints[2] = duck_control::hx::decode_position(7, &2048u16.to_le_bytes())
            .map_err(|_| Error::InvalidParameters)?;
        let final_prepositioning = params.stop_area_px.is_none() && !center_pan
            && current_pitch > TARGET_FINAL_PITCH_RAW
            && (next.final_prepositioning || current_pitch <= TARGET_PREPOSITION_START_RAW)
            && measured_yaw.abs_diff(2048) < REFERENCE_PAN_CENTER_WINDOW
            && geometry.project_to_image(&cam, ray, final_joints).is_ok_and(|projected| {
                let margin_px = if next.final_prepositioning {
                    TARGET_PREPOSITION_HOLD_MARGIN_PX
                } else { TARGET_PREPOSITION_ENTER_MARGIN_PX };
                projected.iter().all(|value| value.is_finite()) && (0..2).all(|axis| {
                    let margin = margin_px / if axis == 0 { 160.0 } else { 90.0 };
                    (projected[axis] - window.goal[axis]).abs() <= window.tolerance[axis] + margin
                })
            });
        next.final_prepositioning = final_prepositioning
            || (center_pan && current_pitch > TARGET_FINAL_PITCH_RAW && next.final_prepositioning);
        let screen_velocity = [-head_output[1], -head_output[0]];
        let mut raw_head = head;
        if center_pan {
            // Final body alignment holds pitch and centers yaw until the target crosses the unlock line.
            raw_head[1] = duck_control::hx::decode_position(7, &2048u16.to_le_bytes())
                .map_err(|_| Error::InvalidParameters)?;
            next.head = [Axis::default(); 2];
        } else if final_prepositioning {
            raw_head = [final_joints[1], final_joints[2]];
            next.head_deadzone_target = Some(raw_head);
            next.head = [Axis::default(); 2];
        } else {
            for joint in 0..2 {
                raw_head[joint] += frame_dt * (inverse[joint][0] * screen_velocity[0]
                    + inverse[joint][1] * screen_velocity[1]);
            }
        }
        for joint in 0..2 {
            if !raw_head[joint].is_finite() { return Err(Error::InvalidParameters); }
            let max_step = TARGET_HEAD_SPEED_RAD_S[joint] * frame_dt.min(TARGET_HEAD_MAX_FRAME_S);
            head[joint] = raw_head[joint].clamp(previous_target[joint] - max_step, previous_target[joint] + max_step)
                .clamp(TRACKING_LIMITS[joint].0, TRACKING_LIMITS[joint].1);
        }
        for axis in 0..2 {
            if center_pan { continue; }
            let contribution = -params.head_gains[axis][1] * head_errors[axis] * frame_dt * frame_dt;
            let image_axis = if axis == 0 { 1 } else { 0 };
            if (0..2).any(|joint| {
                let response = inverse[joint][image_axis];
                (raw_head[joint] - head[joint]) * response * contribution > 0.0
            }) {
                next.head[axis].integral = previous_head[axis].integral;
            }
        }
        next.head_target = Some(head);
        if final_prepositioning {
            next.final_point_filter = None;
            next.confirmation = None;
            next.body_phase = Some(BodyPhase::Alignment);
            next.last_twist = [0.0; 3];
            next.last_deadzone = [true; 3];
            *self = next;
            return Ok(Step::Following { head, twist: [0.0; 3], deadzone: [true; 3] });
        }
        let yaw_raw = duck_control::hx::encode_position(7, head[1]).map_err(|_| Error::InvalidParameters)?;
        let final_phase = center_pan;
        next.body_phase = Some(if final_phase { BodyPhase::Alignment } else { BodyPhase::Approach });
        let control_point = if final_phase {
            next.filter_final_point(point, frame, &params, window)
        } else {
            next.final_point_filter = None;
            point
        };
        let control_ray = if control_point != point {
            geometry.ray_in_trunk(&cam, control_point.map(|value| value.clamp(-1.0, 1.0)), joints)?
        } else { ray };
        let limits = limits.with_linear_limits(next.linear_limits(&params));
        let body_gains = if final_phase { params.final_body_gains } else { params.body_gains };
        if center_pan && window.contains(point) {
            let confirmation = next.confirmation.get_or_insert(ArrivalConfirmation {
                start_us: next.target_motion_settled.then_some(stamp),
                elapsed: Duration::ZERO, since_frame: Duration::ZERO,
                final_point: params.final_point,
                final_tolerance: params.final_tolerance, camera, feedback_generation: history.generation,
            });
            let start_us = next.target_motion_settled.then(|| *confirmation.start_us.get_or_insert(stamp));
            confirmation.since_frame = Duration::ZERO;
            if start_us.is_some_and(|start_us| confirmation.elapsed >= ARRIVAL_CONFIRMATION
                && stamp.saturating_sub(start_us) >= ARRIVAL_CONFIRMATION_US)
            {
                next.confirmation = None;
                let step = next.start_arrival(joints, history, &params);
                *self = next;
                return Ok(step);
            }
            next.last_twist = [0.0; 3];
            next.last_deadzone = [true; 3];
            *self = next;
            return Ok(Step::Following { head, twist: [0.0; 3], deadzone: [true; 3] });
        }
        next.confirmation = None;
        let forward = if let Some((_, error)) = next.follow_range(&params, frame) {
            (body_gains[0] * error).clamp(limits.forward[0], limits.forward[1])
        } else if center_pan && window.contains_axis(point[1], 1) {
            0.0
        } else {
            (body_gains[0] * approach_error(geometry, &cam, control_ray, joints, control_point, window)?)
                .clamp(limits.forward[0], limits.forward[1])
        };
        let (mut twist, mut body_deadzone) = if final_phase {
            final_alignment(geometry, &cam, control_ray, joints, control_point, window,
                forward, body_gains[1], limits)?
        } else {
            let yaw_direction = -head[1] * head_yaw_per_body_yaw;
            let yaw = turn_command(yaw_raw, yaw_direction, params.body_gains[2], limits);
            ([forward, 0.0, yaw], [forward == 0.0, true, yaw == 0.0])
        };
        if !twist.iter().all(|value| value.is_finite()) { return Err(Error::InvalidParameters); }
        if final_phase && !window.contains_axis(point[0], 0) {
            // Discard numerically insignificant cross-axis compensation without imposing
            // a minimum speed on the actual correction.
            let primary = usize::from(twist[1].abs() > twist[0].abs());
            let secondary = 1 - primary;
            if twist[secondary].abs() < twist[primary].abs() * 0.01 { twist[secondary] = 0.0; }
        }
        for axis in 0..3 {
            if twist[axis] == 0.0 { body_deadzone[axis] = true; }
        }
        next.last_twist = twist;
        next.last_deadzone = body_deadzone;
        *self = next;
        Ok(Step::Following { head, twist, deadzone: body_deadzone })
    }
}
