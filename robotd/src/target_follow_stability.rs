//! Stabilize color-target approach commands without changing person following.
//!
//! Detection is sampled on camera timestamps; repeated control ticks cannot satisfy a
//! persistence check. Explicit stops, stale vision and other takeovers remain owned by
//! the control loop, outside this normal-motion filter.

use std::time::Duration;

use duck_ipc_proto::{CoordinatedTrackingParams, CoordinatedTrackingPhase, TrackingCamera};
use serde::Serialize;

use crate::coordinated_tracking::{MotionLimits, TargetGeometry, TARGET_APPROACH_SLOWDOWN_START_RAW,
    TARGET_FINAL_PITCH_RAW, TARGET_HEAD_SPEED_RAD_S, MIN_BODY_SPEED};

const BEARING_TAU_S: f64 = 0.10;
const BEARING_MAX_GAP_US: u64 = 600_000;
const TURN_ENTER_RAD: f64 = 5.0_f64.to_radians();
const TURN_EXIT_RAD: f64 = 2.5_f64.to_radians();
const TURN_CONFIRM_US: u64 = 150_000;
const MAX_TARGET_YAW: f64 = 0.4;
const BORDER_CONFIRM_US: u64 = 250_000;
const BORDER_IMMEDIATE_EXIT_PX: f64 = 60.0;
const BORDER_IMMEDIATE_EXIT_PITCH: u16 = 1084;
const FINAL_TRANSLATION_CONFIRM_US: u64 = 250_000;
// A tiny boundary correction should survive the initial body pitch transient.
// Bound the command time and speed; actual travel also depends on the gait.
const NEAR_BOUNDARY_PULSE_US: u64 = 300_000;
const NEAR_BOUNDARY_MARGIN_PX: f64 = 5.0;
const FINAL_SETTLE_DWELL: Duration = Duration::from_millis(150);
// The final image coordinates are in the 320-by-180 tracking view. Limit lateral
// corrections near the window, where a full gait step can carry the target across it.
const FINAL_LATERAL_NEAR_SPEED: f64 = 0.06;
const FINAL_LATERAL_SPEED_PER_PIXEL: f64 = 0.0004;
const FINAL_LATERAL_MAX_SPEED: f64 = 0.07;
const IMAGE_CENTER_Y_PX: f64 = 90.0;
const WALK_GRACE: Duration = Duration::from_millis(250);

#[derive(Clone, Copy, Default)]
struct CandidateTurn {
    direction: i8,
    since_us: u64,
    frames: u8,
}

#[derive(Default)]
struct Heading {
    filtered: Option<f64>,
    last_stamp_us: Option<u64>,
    active_direction: i8,
    candidate: Option<CandidateTurn>,
}

impl Heading {
    fn update(&mut self, stamp: u64, bearing: f64, gain: f64, bounds: [f64; 2]) -> f64 {
        if !bearing.is_finite() || !gain.is_finite() {
            *self = Self::default();
            return 0.0;
        }
        let fresh = self.last_stamp_us != Some(stamp);
        if fresh {
            let filtered = match (self.filtered, self.last_stamp_us) {
                (Some(previous), Some(last)) if stamp > last && stamp - last <= BEARING_MAX_GAP_US => {
                    let dt = (stamp - last) as f64 * 1e-6;
                    let alpha = 1.0 - (-dt / BEARING_TAU_S).exp();
                    let delta = bearing - previous;
                    let delta = delta.sin().atan2(delta.cos());
                    let angle = previous + alpha * delta;
                    angle.sin().atan2(angle.cos())
                }
                _ => {
                    self.active_direction = 0;
                    self.candidate = None;
                    bearing
                }
            };
            self.filtered = Some(filtered);
            self.last_stamp_us = Some(stamp);
            let direction = if filtered > 0.0 { 1 } else if filtered < 0.0 { -1 } else { 0 };
            if self.active_direction != 0
                && (filtered.abs() <= TURN_EXIT_RAD || direction != self.active_direction)
            {
                self.active_direction = 0;
                self.candidate = None;
            }
            if self.active_direction == 0 {
                if filtered.abs() >= TURN_ENTER_RAD {
                    let candidate = self.candidate.filter(|candidate| candidate.direction == direction)
                        .unwrap_or(CandidateTurn { direction, since_us: stamp, frames: 0 });
                    let candidate = CandidateTurn { frames: candidate.frames.saturating_add(1), ..candidate };
                    if candidate.frames >= 3 && stamp.saturating_sub(candidate.since_us) >= TURN_CONFIRM_US {
                        self.active_direction = direction;
                        self.candidate = None;
                    } else {
                        self.candidate = Some(candidate);
                    }
                } else {
                    self.candidate = None;
                }
            }
        }
        let Some(filtered) = self.filtered else { return 0.0 };
        if self.active_direction == 0 { return 0.0; }
        let magnitude = (filtered.abs() - TURN_EXIT_RAD).max(0.0) * gain;
        (magnitude * f64::from(self.active_direction))
            .clamp(bounds[0].max(-MAX_TARGET_YAW), bounds[1].min(MAX_TARGET_YAW))
    }
}

#[derive(Default)]
struct BoundaryHold {
    active: bool,
    outside_since_us: Option<u64>,
}

impl BoundaryHold {
    fn update(&mut self, phase: CoordinatedTrackingPhase, frame: TargetGeometry,
        params: &CoordinatedTrackingParams) -> bool
    {
        if phase == CoordinatedTrackingPhase::Confirming {
            self.active = true;
            self.outside_since_us = None;
            return true;
        }
        if !self.active { return false; }
        if !matches!(phase, CoordinatedTrackingPhase::Approach | CoordinatedTrackingPhase::FinalAlignment) {
            *self = Self::default();
            return false;
        }
        let outside = (0..2).map(|axis| {
            ((frame.point_px[axis] - params.final_point[axis]).abs() - params.final_tolerance[axis]).max(0.0)
        }).fold(0.0_f64, f64::max);
        if outside > BORDER_IMMEDIATE_EXIT_PX || frame.pitch_raw > BORDER_IMMEDIATE_EXIT_PITCH {
            *self = Self::default();
            return false;
        }
        let since = *self.outside_since_us.get_or_insert(frame.captured_at_unix_us);
        if frame.captured_at_unix_us < since
            || frame.captured_at_unix_us - since >= BORDER_CONFIRM_US
        {
            *self = Self::default();
            return false;
        }
        true
    }
}

#[derive(Clone, Copy)]
struct TranslationCandidate {
    axis: usize,
    direction: i8,
    since_us: u64,
    last_us: u64,
    frames: u8,
}

#[derive(Default)]
struct FinalTranslation { candidate: Option<TranslationCandidate> }

#[derive(Clone, Copy)]
struct BoundaryPulse {
    since_us: u64,
    remaining: Duration,
    image_axis: usize,
    side: f64,
    twist: [f64; 3],
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct CorrectionDirection { axis: usize, positive: bool }

#[derive(Default)]
struct CorrectionSettle {
    in_final: bool,
    active: Option<CorrectionDirection>,
    waiting: bool,
    quiet: Duration,
    lateral_release: bool,
}

impl CorrectionSettle {
    fn update(&mut self, phase: CoordinatedTrackingPhase, mut twist: [f64; 3],
        motion_settled: bool, dt: Duration) -> [f64; 3]
    {
        if !matches!(phase, CoordinatedTrackingPhase::FinalAlignment | CoordinatedTrackingPhase::Confirming) {
            *self = Self::default();
            return twist;
        }
        if !self.in_final {
            self.in_final = true;
            self.waiting = true;
        }
        let axis = usize::from(twist[1].abs() > twist[0].abs());
        let direction = (twist[axis] != 0.0).then_some(CorrectionDirection { axis, positive: twist[axis] > 0.0 });
        if self.active.is_some() && self.active != direction {
            self.lateral_release = self.active.is_some_and(|active| active.axis == 1);
            self.active = None;
            self.waiting = true;
            self.quiet = Duration::ZERO;
        }
        if self.waiting {
            self.quiet = if motion_settled {
                self.quiet.saturating_add(dt).min(FINAL_SETTLE_DWELL)
            } else { Duration::ZERO };
            // Do not latch an old quiet interval while no correction is ready. The
            // stand transition itself may make the robot move before a request arrives.
            if self.quiet >= FINAL_SETTLE_DWELL && direction.is_some() {
                self.waiting = false;
                self.quiet = Duration::ZERO;
                self.lateral_release = false;
            } else {
                twist[..2].fill(0.0);
                return twist;
            }
        }
        self.active = direction;
        twist
    }
}

impl FinalTranslation {
    fn update(&mut self, stamp: u64, mut twist: [f64; 3], params: &CoordinatedTrackingParams,
        limits: MotionLimits) -> [f64; 3]
    {
        let axis = usize::from(twist[1].abs() > twist[0].abs());
        let magnitude = twist[axis].abs();
        if magnitude == 0.0 {
            self.candidate = None;
            return twist;
        }
        let bounds = [limits.forward, limits.lateral][axis];
        let configured_cap = [params.final_linear_limit, params.final_lateral_limit][axis];
        let directional_cap = if twist[axis] > 0.0 { bounds[1] } else { -bounds[0] };
        let minimum = MIN_BODY_SPEED[axis].min(configured_cap).min(directional_cap).max(0.0);
        if minimum == 0.0 {
            self.candidate = None;
            return twist;
        }
        let direction = if twist[axis] > 0.0 { 1 } else { -1 };
        let mut candidate = self.candidate.filter(|candidate| {
            candidate.axis == axis && candidate.direction == direction
                && stamp >= candidate.last_us && stamp - candidate.last_us <= BEARING_MAX_GAP_US
        }).unwrap_or(TranslationCandidate { axis, direction, since_us: stamp, last_us: stamp, frames: 0 });
        if candidate.frames == 0 || stamp != candidate.last_us {
            candidate.frames = candidate.frames.saturating_add(1);
            candidate.last_us = stamp;
        }
        self.candidate = Some(candidate);
        if candidate.frames < 3 || stamp.saturating_sub(candidate.since_us) < FINAL_TRANSLATION_CONFIRM_US {
            twist[..2].fill(0.0);
            return twist;
        }
        if magnitude >= minimum { return twist; }
        let scale = minimum / magnitude;
        for (joint, bounds, configured_cap) in [(0, limits.forward, params.final_linear_limit),
                                                  (1, limits.lateral, params.final_lateral_limit)] {
            twist[joint] = (twist[joint] * scale)
                .clamp(bounds[0].max(-configured_cap), bounds[1].min(configured_cap));
        }
        twist
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Diagnostic {
    pub bearing_rad: Option<f64>,
    pub filtered_bearing_rad: Option<f64>,
    pub turn_direction: i8,
    pub turn_pending_ms: Option<u64>,
    pub boundary_hold: bool,
    pub boundary_pulse_active: bool,
    pub final_settling_hold: bool,
    pub final_settling_quiet_ms: u64,
    pub final_lateral_release: bool,
    pub final_lateral_speed_cap: Option<f64>,
    pub approach_linear_cap: Option<f64>,
    pub final_translation_pending_ms: Option<u64>,
    pub final_translation_min_applied: bool,
}

#[derive(Default)]
pub struct TargetFollowStability {
    heading: Heading,
    boundary: BoundaryHold,
    final_translation: FinalTranslation,
    boundary_pulse: Option<BoundaryPulse>,
    correction_settle: CorrectionSettle,
    camera: Option<TrackingCamera>,
    window: Option<([f64; 2], [f64; 2])>,
    last_twist: [f64; 3],
    bearing_rad: Option<f64>,
    approach_linear_cap: Option<f64>,
    final_translation_min_applied: bool,
    final_lateral_speed_cap: Option<f64>,
}

impl TargetFollowStability {
    pub fn reset(&mut self) { *self = Self::default(); }

    /// After a lateral correction, allow the normal velocity threshold to select stand.
    pub fn lateral_release(&self) -> bool { self.correction_settle.lateral_release }



    pub fn apply_with_dt(&mut self, phase: CoordinatedTrackingPhase, geometry: Option<TargetGeometry>,
        params: &CoordinatedTrackingParams, twist: [f64; 3], limits: MotionLimits, dt: Duration,
        motion_settled: bool)
        -> ([f64; 3], [bool; 3])
    {
        self.apply_internal(phase, geometry, params, twist, limits, dt, Some(motion_settled))
    }

    fn apply_internal(&mut self, phase: CoordinatedTrackingPhase, geometry: Option<TargetGeometry>,
        params: &CoordinatedTrackingParams, mut twist: [f64; 3], limits: MotionLimits, dt: Duration,
        motion_settled: Option<bool>)
        -> ([f64; 3], [bool; 3])
    {
        self.approach_linear_cap = None;
        self.bearing_rad = None;
        self.final_translation_min_applied = false;
        self.final_lateral_speed_cap = None;
        if params.target.is_none() {
            let pulse_was_active = self.boundary_pulse.take().is_some();
            self.heading = Heading::default();
            self.boundary = BoundaryHold::default();
            self.final_translation = FinalTranslation::default();
            self.correction_settle = CorrectionSettle::default();
            if phase == CoordinatedTrackingPhase::WaitingTarget {
                // Preserve the last stabilized command for the existing brief-loss grace period.
                twist = if pulse_was_active { [0.0; 3] } else { self.last_twist };
            } else if phase == CoordinatedTrackingPhase::Searching {
                twist[2] = twist[2].clamp(limits.yaw[0].max(-MAX_TARGET_YAW), limits.yaw[1].min(MAX_TARGET_YAW));
            }
            self.last_twist = twist;
            return (twist, twist.map(|value| value == 0.0));
        }
        if !matches!(phase, CoordinatedTrackingPhase::Approach | CoordinatedTrackingPhase::FinalAlignment
            | CoordinatedTrackingPhase::Confirming)
        {
            self.reset();
            return (twist, twist.map(|value| value == 0.0));
        }
        let Some(frame) = geometry else {
            self.reset();
            return ([0.0; 3], [true; 3]);
        };
        let window = (params.final_point, params.final_tolerance);
        if self.camera != Some(frame.camera) || self.window != Some(window) {
            self.reset();
            self.camera = Some(frame.camera);
            self.window = Some(window);
        }
        self.bearing_rad = frame.bearing_rad;
        if phase == CoordinatedTrackingPhase::Approach {
            self.final_translation = FinalTranslation::default();
            let Some(bearing) = frame.bearing_rad else {
                self.heading = Heading::default();
                self.last_twist = [0.0; 3];
                return ([0.0; 3], [true; 3]);
            };
            twist[2] = self.heading.update(frame.captured_at_unix_us, bearing,
                params.body_gains[2], limits.yaw);
            let approach = params.approach_linear_limit.unwrap_or(limits.forward[1])
                .min(limits.forward[1]).max(0.0);
            let final_cap = params.final_linear_limit.min(approach).max(0.0);
            let fraction = f64::from(frame.pitch_raw.saturating_sub(TARGET_FINAL_PITCH_RAW))
                / f64::from(TARGET_APPROACH_SLOWDOWN_START_RAW - TARGET_FINAL_PITCH_RAW);
            let mut cap = final_cap + (approach - final_cap) * fraction.clamp(0.0, 1.0);
            let window_top = params.final_point[1] - params.final_tolerance[1];
            if window_top > IMAGE_CENTER_Y_PX {
                let fraction = ((window_top - frame.point_px[1]) / (window_top - IMAGE_CENTER_Y_PX))
                    .clamp(0.0, 1.0);
                cap = cap.min(final_cap + (approach - final_cap) * fraction);
            }
            twist[0] = twist[0].clamp(-approach, cap);
            self.approach_linear_cap = Some(cap);
        } else {
            self.heading = Heading::default();
        }
        let mut translation_limits = limits;
        if phase == CoordinatedTrackingPhase::FinalAlignment && twist[1] != 0.0 {
            let outside_px = ((frame.point_px[0] - params.final_point[0]).abs()
                - params.final_tolerance[0]).max(0.0);
            let cap = (FINAL_LATERAL_NEAR_SPEED + outside_px * FINAL_LATERAL_SPEED_PER_PIXEL)
                .min(FINAL_LATERAL_MAX_SPEED).min(params.final_lateral_limit);
            translation_limits.lateral[0] = translation_limits.lateral[0].max(-cap);
            translation_limits.lateral[1] = translation_limits.lateral[1].min(cap);
            self.final_lateral_speed_cap = Some(cap);
            if twist[1].abs() > cap {
                let scale = cap / twist[1].abs();
                twist[0] *= scale;
                twist[1] *= scale;
            }
        }
        if self.boundary.update(phase, frame, params) {
            // Count the same fresh frames while waiting at the boundary, so a sustained
            // departure does not have to wait for two consecutive confirmation periods.
            if phase == CoordinatedTrackingPhase::FinalAlignment {
                self.final_translation.update(frame.captured_at_unix_us, twist, params, translation_limits);
            } else {
                self.final_translation = FinalTranslation::default();
            }
            twist = [0.0; 3];
            self.heading = Heading::default();
        } else if phase == CoordinatedTrackingPhase::FinalAlignment {
            let before = twist;
            twist = self.final_translation.update(frame.captured_at_unix_us, twist, params, translation_limits);
            self.final_translation_min_applied = twist[..2].iter().zip(before[..2].iter())
                .any(|(after, before)| after.abs() > before.abs() + 1e-12);
        } else {
            self.final_translation = FinalTranslation::default();
        }
        // If a correction began only a few pixels outside the requested window,
        // pitch transients can repeatedly move the target across the boundary before
        // the gait makes progress. Keep the short, bounded forward/backward request
        // across those phase changes, unless the goal is crossed, horizontal error
        // becomes substantial, or the target/geometry is lost.
        if let Some(mut pulse) = self.boundary_pulse {
            pulse.remaining = pulse.remaining.saturating_sub(dt);
            self.boundary_pulse = Some(pulse);
            let error = frame.point_px[pulse.image_axis] - params.final_point[pulse.image_axis];
            let elapsed = frame.captured_at_unix_us.checked_sub(pulse.since_us);
            let other = 1 - pulse.image_axis;
            let other_inside = (frame.point_px[other] - params.final_point[other]).abs()
                <= params.final_tolerance[other] + NEAR_BOUNDARY_MARGIN_PX;
            if matches!(phase, CoordinatedTrackingPhase::FinalAlignment | CoordinatedTrackingPhase::Confirming)
                && elapsed.is_some_and(|elapsed| elapsed < NEAR_BOUNDARY_PULSE_US)
                && !pulse.remaining.is_zero()
                && error * pulse.side > 0.0 && other_inside
            {
                twist = pulse.twist;
                twist[0] = twist[0].clamp(limits.forward[0].max(-params.final_linear_limit),
                    limits.forward[1].min(params.final_linear_limit));
                twist[1] = twist[1].clamp(limits.lateral[0].max(-params.final_lateral_limit),
                    limits.lateral[1].min(params.final_lateral_limit));
            } else if !matches!(phase, CoordinatedTrackingPhase::FinalAlignment | CoordinatedTrackingPhase::Confirming)
                || elapsed.is_none_or(|elapsed| elapsed >= NEAR_BOUNDARY_PULSE_US)
                || pulse.remaining.is_zero() || error * pulse.side <= 0.0 || !other_inside
            {
                self.boundary_pulse = None;
            }
        }
        if phase == CoordinatedTrackingPhase::FinalAlignment && self.boundary_pulse.is_none()
            && twist[0] != 0.0 && twist[0].abs() >= twist[1].abs()
        {
            let axis = if twist[0].abs() >= twist[1].abs() { 1 } else { 0 };
            let other = 1 - axis;
            let error = frame.point_px[axis] - params.final_point[axis];
            let excess = error.abs() - params.final_tolerance[axis];
            if excess > 0.0 && excess <= NEAR_BOUNDARY_MARGIN_PX
                && (frame.point_px[other] - params.final_point[other]).abs() <= params.final_tolerance[other]
            {
                let mut pulse = [0.0; 3];
                let primary = 1 - axis;
                pulse[primary] = twist[primary].clamp(-MIN_BODY_SPEED[primary], MIN_BODY_SPEED[primary]);
                self.boundary_pulse = Some(BoundaryPulse {
                    since_us: frame.captured_at_unix_us, remaining: Duration::from_micros(NEAR_BOUNDARY_PULSE_US),
                    image_axis: axis, side: error.signum(), twist: pulse,
                });
            }
        }
        if let Some(settled) = motion_settled {
            let allowed = self.correction_settle.update(phase, twist, settled, dt);
            if allowed[..2] == [0.0; 2] && twist[..2] != [0.0; 2] {
                // Do not keep a pulse created for an adjustment that never started.
                self.boundary_pulse = None;
            }
            twist = allowed;
        }
        self.last_twist = twist;
        (twist, twist.map(|value| value == 0.0))
    }

    pub fn diagnostic(&self) -> Diagnostic {
        Diagnostic {
            bearing_rad: self.bearing_rad,
            filtered_bearing_rad: self.heading.filtered,
            turn_direction: self.heading.active_direction,
            turn_pending_ms: self.heading.candidate.map(|candidate| {
                self.heading.last_stamp_us.unwrap_or(candidate.since_us)
                    .saturating_sub(candidate.since_us) / 1000
            }),
            boundary_hold: self.boundary.active,
            boundary_pulse_active: self.boundary_pulse.is_some(),
            final_settling_hold: self.correction_settle.waiting,
            final_settling_quiet_ms: self.correction_settle.quiet.as_millis().min(u128::from(u64::MAX)) as u64,
            final_lateral_release: self.correction_settle.lateral_release,
            final_lateral_speed_cap: self.final_lateral_speed_cap,
            approach_linear_cap: self.approach_linear_cap,
            final_translation_pending_ms: self.final_translation.candidate.and_then(|candidate| {
                let elapsed = candidate.last_us.saturating_sub(candidate.since_us);
                (candidate.frames < 3 || elapsed < FINAL_TRANSLATION_CONFIRM_US).then_some(elapsed / 1000)
            }),
            final_translation_min_applied: self.final_translation_min_applied,
        }
    }
}

#[derive(Default)]
pub struct WalkGrace { remaining: Duration }

impl WalkGrace {
    pub fn update(&mut self, active: bool, moving: bool, period: Duration) -> bool {
        if !active {
            self.remaining = Duration::ZERO;
        } else if moving {
            self.remaining = WALK_GRACE;
        } else {
            self.remaining = self.remaining.saturating_sub(period);
        }
        active && self.remaining > Duration::ZERO
    }
}

/// Limit head targets relative to the last successful write, including after a bus error.
#[derive(Default)]
pub struct HeadRate { confirmed: Option<[f64; 2]> }

impl HeadRate {
    pub fn reset(&mut self) { self.confirmed = None; }

    pub fn limit(&self, requested: [f64; 2], measured: [f64; 2], dt: f64) -> [f64; 2] {
        let previous = self.confirmed.unwrap_or(measured);
        let dt = dt.clamp(0.0, 0.1);
        std::array::from_fn(|axis| {
            let (lower, upper) = duck_control::model::JOINT_LIMITS[axis + 6];
            let previous = previous[axis].clamp(lower, upper);
            let step = TARGET_HEAD_SPEED_RAD_S[axis] * dt;
            requested[axis].clamp(previous - step, previous + step).clamp(lower, upper)
        })
    }

    pub fn confirm(&mut self, written: [f64; 2]) { self.confirmed = Some(written); }
}
