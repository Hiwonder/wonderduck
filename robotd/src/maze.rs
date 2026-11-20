//! Maze navigation keeps an obstacle decision until its scan and turn complete.

use std::f64::consts::PI;
use std::time::{Duration, Instant};

use duck_control::fall::{FallPredictor, FallPredictorConfig};
use duck_control::imu::ImuData;
use duck_control::model::{JOINT_LIMITS, joint_index};

const HEAD_COMPENSATION_WAIT: Duration = Duration::from_millis(500);

pub struct Settings {
    pub threshold_mm: u32,
    pub settle: Duration,
    pub sample: Duration,
    pub restore: Duration,
    pub turn_initial: Duration,
    pub turn_final: Duration,
    pub linear_speed: f64,
    pub angular_speed: f64,
    /// Maximum commanded head yaw change per control tick, in degrees.
    pub head_step_deg: f64,
}

#[derive(Clone, Copy)]
pub struct Reading {
    pub distance_mm: Option<u32>,
    pub stamp: u64,
}

pub struct Output {
    pub twist: [f64; 3],
    pub head: Option<f64>,
    pub immediate_stop: bool,
}

#[derive(Clone, Copy)]
enum Side {
    Left,
    Right,
}

#[derive(Clone, Copy)]
enum Turn {
    Left,
    Right,
    Blocked,
}

enum Phase {
    Forward,
    Recover,
    Stopping,
    Settle(Side),
    Sample(Side),
    Restore(Turn),
    TurnInitial(Turn),
    TurnClearance(Turn),
    TurnFinal(Turn),
    TurnStopping,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryState {
    Normal,
    Paused,
    Resumed,
}

pub struct FallRecovery {
    active: bool,
    standing: FallPredictor,
}

#[cfg(test)]
mod recovery_tests {
    use super::*;

    #[test]
    fn recovery_waits_for_a_full_second_of_fresh_upright_samples() {
        let mut recovery = FallRecovery::new();
        let period = Duration::from_millis(20);
        let upright = ImuData::default();

        assert_eq!(recovery.observe(false, None, period), RecoveryState::Normal);
        assert_eq!(recovery.observe(true, Some(&upright), period), RecoveryState::Paused);
        for _ in 0..49 {
            assert_eq!(recovery.observe(false, Some(&upright), period), RecoveryState::Paused);
        }
        assert_eq!(recovery.observe(false, None, period), RecoveryState::Paused);
        for _ in 0..49 {
            assert_eq!(recovery.observe(false, Some(&upright), period), RecoveryState::Paused);
        }
        assert_eq!(recovery.observe(false, Some(&upright), period), RecoveryState::Resumed);
        assert_eq!(recovery.observe(false, Some(&upright), period), RecoveryState::Normal);
    }
}

impl FallRecovery {
    pub fn new() -> Self {
        Self {
            active: false,
            standing: FallPredictor::new(FallPredictorConfig::default()),
        }
    }

    pub fn observe(
        &mut self,
        fallen: bool,
        imu: Option<&ImuData>,
        period: Duration,
    ) -> RecoveryState {
        if fallen {
            self.active = true;
            self.standing.reset();
            return RecoveryState::Paused;
        }
        if !self.active {
            return RecoveryState::Normal;
        }
        if let Some(imu) = imu {
            self.standing.observe(imu, period);
        } else {
            self.standing.reset();
        }
        if self.standing.standing_confirmed() {
            self.active = false;
            RecoveryState::Resumed
        } else {
            RecoveryState::Paused
        }
    }
}

pub struct Maze {
    original_head: f64,
    commanded_head: f64,
    physical_head_target: Option<f64>,
    compensated_head_target: f64,
    initial_head_sweep_complete: bool,
    head_command_since: Option<Instant>,
    phase: Phase,
    since: Option<Instant>,
    last_tick: Option<Instant>,
    turn_elapsed: Duration,
    latest: Option<Reading>,
    samples: Vec<u32>,
    missing: bool,
    held_head: Option<f64>,
    original_head_ready: bool,
    forward_frames: u8,
    side_frames: u8,
}

impl Maze {
    pub fn new(original_head: f64) -> Self {
        Self {
            original_head,
            commanded_head: original_head,
            physical_head_target: Some(original_head),
            compensated_head_target: original_head,
            initial_head_sweep_complete: true,
            head_command_since: None,
            phase: Phase::Forward,
            since: None,
            last_tick: None,
            turn_elapsed: Duration::ZERO,
            latest: None,
            samples: Vec::new(),
            missing: false,
            held_head: None,
            original_head_ready: true,
            forward_frames: 3,
            side_frames: 0,
        }
    }

    pub fn phase(&self) -> &'static str {
        if self.missing {
            return "waiting_for_tof";
        }
        match self.phase {
            Phase::Forward => "forward",
            Phase::Recover => "recover_head",
            Phase::Stopping => "stopping",
            Phase::Settle(Side::Left) => "settle_left",
            Phase::Settle(Side::Right) => "settle_right",
            Phase::Sample(Side::Left) => "sample_left",
            Phase::Sample(Side::Right) => "sample_right",
            Phase::Restore(_) => "restore",
            Phase::TurnInitial(_) => "turn_initial",
            Phase::TurnClearance(_) => "turn_clearance",
            Phase::TurnFinal(_) => "turn_final",
            Phase::TurnStopping => "turn_stopping",
        }
    }

    /// Discard the pre-fall scan and turn while preserving the original forward head pose.
    pub fn resume_after_fall(&mut self, measured_head: f64) {
        self.phase = Phase::Recover;
        self.commanded_head = measured_head;
        self.clear_head_compensation();
        self.since = None;
        self.last_tick = None;
        self.turn_elapsed = Duration::ZERO;
        self.latest = None;
        self.samples.clear();
        self.missing = false;
        self.held_head = None;
        self.original_head_ready = false;
        self.forward_frames = 0;
        self.side_frames = 0;
    }

    fn scan(&mut self, side: Side) {
        self.phase = Phase::Settle(side);
        self.since = None;
        self.samples.clear();
        self.original_head_ready = false;
        self.forward_frames = 0;
        self.side_frames = 0;
    }

    fn wait_for_clearance(&mut self, turn: Turn) {
        self.phase = Phase::TurnClearance(turn);
        self.turn_elapsed = Duration::ZERO;
        // The head target is unchanged: retain the front-direction median
        // flush already acquired during restore and the initial turn.
    }

    pub fn hold_on_failure(&mut self, measured_head: f64) -> Output {
        let held = *self.held_head.get_or_insert(measured_head);
        self.commanded_head = held;
        self.clear_head_compensation();
        self.missing = true;
        self.since = None;
        self.last_tick = None;
        self.original_head_ready = false;
        self.forward_frames = 0;
        self.side_frames = 0;
        Output {
            twist: [0.0; 3],
            head: Some(held),
            immediate_stop: true,
        }
    }

    fn head_ready(&self, measured_head: f64, target: f64, step: f64) -> bool {
        // The initial sweep must finish, but a later correction must stop as
        // soon as feedback reaches the physical direction.
        let initial_sweep_complete =
            self.physical_head_target == Some(target) && self.initial_head_sweep_complete;
        (initial_sweep_complete || stepped_head(self.commanded_head, target, step) == target)
            && at_target(measured_head, target)
    }

    fn clear_head_compensation(&mut self) {
        self.physical_head_target = None;
        self.compensated_head_target = self.commanded_head;
        self.initial_head_sweep_complete = false;
        self.head_command_since = None;
    }

    fn interpolate(
        &mut self,
        mut output: Output,
        now: Instant,
        measured_head: f64,
        step: f64,
    ) -> Output {
        if let Some(target) = output.head {
            if self.physical_head_target != Some(target) {
                self.physical_head_target = Some(target);
                self.compensated_head_target = target;
                self.initial_head_sweep_complete = false;
                self.head_command_since = None;
            }
            let arrived = at_target(measured_head, target);
            // Time each retry from the current command's endpoint; preserve
            // the emitted correction as soon as the physical direction arrives.
            if arrived && self.initial_head_sweep_complete {
                self.compensated_head_target = self.commanded_head;
                self.head_command_since = None;
            } else if !arrived && self.commanded_head == self.compensated_head_target {
                let since = *self.head_command_since.get_or_insert(now);
                if now.saturating_duration_since(since) >= HEAD_COMPENSATION_WAIT {
                    let (lower, upper) = JOINT_LIMITS
                        [joint_index("head_yaw").expect("head yaw must exist in the joint model")];
                    self.compensated_head_target =
                        (self.compensated_head_target + target - measured_head).clamp(lower, upper);
                    self.head_command_since = None;
                }
            }
            self.commanded_head =
                stepped_head(self.commanded_head, self.compensated_head_target, step);
            if self.commanded_head == target {
                self.initial_head_sweep_complete = true;
            }
            if !arrived
                && self.initial_head_sweep_complete
                && self.commanded_head == self.compensated_head_target
            {
                self.head_command_since.get_or_insert(now);
            }
            output.head = Some(self.commanded_head);
        }
        output
    }

    fn observe_original_head(&mut self, measured_head: f64, new_frame: bool, step: f64) {
        if !self.head_ready(measured_head, self.original_head, step) {
            self.original_head_ready = false;
            self.forward_frames = 0;
        } else if !self.original_head_ready {
            self.original_head_ready = true;
        } else if new_frame && self.forward_frames < 3 {
            self.forward_frames += 1;
        }
    }

    pub fn tick(
        &mut self,
        now: Instant,
        settings: &Settings,
        reading: Option<Reading>,
        measured_head: f64,
        stopped: bool,
    ) -> Output {
        let step = settings.head_step_deg.to_radians();
        let mut output = Output {
            twist: [0.0; 3],
            head: None,
            immediate_stop: false,
        };
        let (distance, new_frame) = if let Some(reading) = reading {
            match self.latest {
                Some(last) if reading.stamp < last.stamp => {
                    return self.hold_on_failure(measured_head);
                }
                Some(last) if reading.stamp == last.stamp => {
                    if self.missing {
                        return self.hold_on_failure(measured_head);
                    }
                    (last.distance_mm, false)
                }
                _ => {
                    self.latest = Some(reading);
                    (reading.distance_mm, true)
                }
            }
        } else {
            (None, false)
        };
        self.held_head = None;

        if self.missing {
            self.missing = false;
            if matches!(
                self.phase,
                Phase::Settle(_) | Phase::Sample(_) | Phase::Restore(_)
            ) {
                self.scan(Side::Left);
            }
        }

        if matches!(
            self.phase,
            Phase::Forward
                | Phase::Recover
                | Phase::Restore(_)
                | Phase::TurnInitial(_)
                | Phase::TurnClearance(_)
                | Phase::TurnFinal(_)
                | Phase::TurnStopping
        ) {
            self.observe_original_head(measured_head, new_frame, step);
        }

        match self.phase {
            Phase::Forward => {
                output.head = Some(self.original_head);
                output.twist[0] = settings.linear_speed;
                // Direction readiness validates obstacle evidence. It must not
                // stop travel when no usable front range is available.
                if self.original_head_ready
                    && self.forward_frames >= 3
                    && distance.is_some_and(|distance| distance < settings.threshold_mm)
                {
                    self.phase = Phase::Stopping;
                    output.twist = [0.0; 3];
                }
            }
            Phase::Recover => {
                output.head = Some(self.original_head);
                if !self.head_ready(measured_head, self.original_head, step) {
                    self.since = None;
                } else if now.saturating_duration_since(*self.since.get_or_insert(now))
                    >= settings.restore
                    && (self.forward_frames >= 3 || distance.is_none())
                {
                    self.phase = if distance.is_some_and(|value| value < settings.threshold_mm) {
                        Phase::Stopping
                    } else {
                        Phase::Forward
                    };
                }
            }
            Phase::Stopping => {
                output.head = Some(self.original_head);
                if stopped {
                    self.scan(Side::Left);
                    output.head = Some(PI / 2.0);
                }
            }
            Phase::Settle(side) => {
                let target = head_target(side);
                output.head = Some(target);
                if !self.head_ready(measured_head, target, step) {
                    self.since = None;
                    self.side_frames = 0;
                } else {
                    let since = match self.since {
                        Some(since) => {
                            if new_frame && self.side_frames < 3 {
                                self.side_frames += 1;
                            }
                            since
                        }
                        None => {
                            self.since = Some(now);
                            now
                        }
                    };
                    if now.saturating_duration_since(since) >= settings.settle {
                        self.phase = Phase::Sample(side);
                        self.since = Some(now);
                        self.samples.clear();
                    }
                }
            }
            Phase::Sample(side) => {
                let target = head_target(side);
                output.head = Some(target);
                if !self.head_ready(measured_head, target, step) {
                    self.scan(side);
                } else {
                    if new_frame && self.side_frames < 3 {
                        self.side_frames += 1;
                    }
                    let since = self.since.expect("sampling has a start time");
                    let elapsed = now.saturating_duration_since(since);
                    // Flush the upstream three-frame median after changing
                    // direction, while keeping the configured sample window.
                    if elapsed <= settings.sample
                        && new_frame
                        && self.side_frames >= 3
                        && let Some(distance) = distance
                    {
                        self.samples.push(distance);
                    }
                    if elapsed >= settings.sample {
                        let open = if self.samples.is_empty() {
                            false // No evidence that this side is clear; try the other side.
                        } else {
                            self.samples.sort_unstable();
                            let n = self.samples.len();
                            let median_twice = u64::from(self.samples[(n - 1) / 2])
                                + u64::from(self.samples[n / 2]);
                            median_twice > 2 * u64::from(settings.threshold_mm)
                        };
                        if open {
                            self.phase = Phase::Restore(match side {
                                Side::Left => Turn::Left,
                                Side::Right => Turn::Right,
                            });
                            self.since = None;
                            output.head = Some(self.original_head);
                        } else if matches!(side, Side::Left) {
                            self.scan(Side::Right);
                            output.head = Some(-PI / 2.0);
                        } else {
                            self.phase = Phase::Restore(Turn::Blocked);
                            self.since = None;
                            output.head = Some(self.original_head);
                        }
                    }
                }
            }
            Phase::Restore(turn) => {
                output.head = Some(self.original_head);
                if !self.head_ready(measured_head, self.original_head, step) {
                    self.since = None;
                } else if now.saturating_duration_since(*self.since.get_or_insert(now))
                    >= settings.restore
                {
                    self.phase = Phase::TurnInitial(turn);
                    self.turn_elapsed = Duration::ZERO;
                    self.last_tick = Some(now);
                    output.twist[2] = turn_speed(turn, settings);
                    if settings.turn_initial.is_zero() {
                        self.wait_for_clearance(turn);
                    }
                }
            }
            Phase::TurnInitial(turn) | Phase::TurnClearance(turn) | Phase::TurnFinal(turn) => {
                output.head = Some(self.original_head);
                if !self.head_ready(measured_head, self.original_head, step) {
                    self.last_tick = None;
                    return self.interpolate(output, now, measured_head, step);
                }
                // The same request crosses both phase boundaries, so the
                // caller's manual-control smoothing never sees a stop/reset.
                output.twist[2] = turn_speed(turn, settings);
                match self.phase {
                    Phase::TurnInitial(_) => {
                        if let Some(last) = self.last_tick {
                            self.turn_elapsed += now.saturating_duration_since(last);
                        }
                        if self.turn_elapsed >= settings.turn_initial {
                            self.wait_for_clearance(turn);
                        }
                    }
                    Phase::TurnClearance(_) => {
                        if new_frame
                            && self.forward_frames >= 3
                            && distance.is_some_and(|distance| distance > settings.threshold_mm)
                        {
                            self.turn_elapsed = Duration::ZERO;
                            if settings.turn_final.is_zero() {
                                self.phase = Phase::TurnStopping;
                                output.twist = [0.0; 3];
                            } else {
                                self.phase = Phase::TurnFinal(turn);
                            }
                        }
                    }
                    Phase::TurnFinal(_) => {
                        if let Some(last) = self.last_tick {
                            self.turn_elapsed += now.saturating_duration_since(last);
                        }
                        // Once clearance is confirmed, later range readings
                        // do not interrupt the configured final turn.
                        if self.turn_elapsed >= settings.turn_final {
                            self.phase = Phase::TurnStopping;
                            output.twist = [0.0; 3];
                        }
                    }
                    _ => unreachable!("turn phases share one handler"),
                }
            }
            Phase::TurnStopping => {
                output.head = Some(self.original_head);
                if stopped && self.head_ready(measured_head, self.original_head, step) {
                    self.phase = Phase::Forward;
                }
            }
        }
        self.last_tick = Some(now);
        self.interpolate(output, now, measured_head, step)
    }
}

fn stepped_head(current: f64, target: f64, step: f64) -> f64 {
    if step == 0.0 {
        return current;
    }
    let remaining = target - current;
    // Snap only roundoff at the endpoint, so 45 two-degree updates reach 90°.
    let rounding = 8.0 * f64::EPSILON * current.abs().max(target.abs()).max(step);
    if remaining.abs() <= step + rounding {
        target
    } else {
        current + step.copysign(remaining)
    }
}

fn head_target(side: Side) -> f64 {
    match side {
        Side::Left => PI / 2.0,
        Side::Right => -PI / 2.0,
    }
}

fn at_target(measured: f64, target: f64) -> bool {
    // Use the user-selected five-degree tolerance for scan and restore targets.
    (measured - target).abs() <= 5.0 * PI / 180.0
}

fn turn_speed(turn: Turn, settings: &Settings) -> f64 {
    match turn {
        Turn::Left => settings.angular_speed,
        Turn::Right | Turn::Blocked => -settings.angular_speed,
    }
}
