//! Image-error PID for the two head joints.

use duck_control::model::JOINT_LIMITS;
use std::sync::LazyLock;

pub(crate) static TRACKING_LIMITS: LazyLock<[(f64, f64); 2]> = LazyLock::new(|| {
    // Raw HX counts are converted through the same calibration used for motor feedback.
    let angle = |joint, raw: u16| {
        duck_control::hx::decode_position(joint, &raw.to_le_bytes())
            .expect("head tracking encoder endpoint must be valid")
    };
    [
        (angle(6, 2250).max(JOINT_LIMITS[6].0), JOINT_LIMITS[6].1),
        (
            angle(7, 2560).max(JOINT_LIMITS[7].0),
            angle(7, 1536).min(JOINT_LIMITS[7].1),
        ),
    ]
});

#[derive(Clone, Copy, Default)]
struct Axis {
    target: f64,
    integral: f64,
    previous: Option<f64>,
    in_deadzone: bool,
}

impl Axis {
    fn step(
        &mut self,
        error: f64,
        deadzone: f64,
        position: f64,
        gains: [f64; 3],
        dt: f64,
        limits: (f64, f64),
    ) -> Option<f64> {
        if deadzone > 0.0 && error.abs() / 2.0 <= deadzone {
            if !self.in_deadzone {
                self.target = position.clamp(limits.0, limits.1);
                self.hold();
                self.in_deadzone = true;
            }
            return Some(self.target);
        }
        self.in_deadzone = false;
        let [kp, ki, kd] = gains;
        let integral = self.integral + error * dt;
        let derivative = self
            .previous
            .map_or(0.0, |previous| (error - previous) / dt);
        let speed = kp * error + ki * integral + kd * derivative;
        let target = self.target + speed * dt;
        if ![integral, derivative, speed, target]
            .into_iter()
            .all(f64::is_finite)
        {
            return None;
        }

        self.integral = integral;
        self.previous = Some(error);
        self.target = target.clamp(limits.0, limits.1);
        Some(self.target)
    }

    fn hold(&mut self) {
        self.integral = 0.0;
        self.previous = None;
        self.in_deadzone = false;
    }
}

/// Independent image tracking for ID 31 (pitch) and ID 32 (yaw).
#[derive(Clone, Copy, Default)]
pub struct HeadTracker {
    pitch: Axis,
    yaw: Axis,
    initialized: bool,
    had_target: bool,
}

// Half the measured 10%-to-80% return speeds in the recorded near-final head pose.
// These shape the commanded positions; the servo's actual response still depends on load.
const RETURN_SPEED_RAD_S: [f64; 2] = [95.6_f64.to_radians(), 28.95_f64.to_radians()];
const GENTLE_HOME_SPEED_RAD_S: [f64; 2] = [25.0_f64.to_radians(), 28.95_f64.to_radians()];
const RETURN_FEEDBACK_GAIN: f64 = 3.0;
const RETURN_TOLERANCE_COUNTS: u16 = 20;

#[derive(Clone, Copy)]
pub struct HeadReturn {
    confirmed: [f64; 2],
    home: [f64; 2],
    home_written: [bool; 2],
    speed: [f64; 2],
}

impl HeadReturn {
    pub fn new(measured: [f64; 2], home: [f64; 2]) -> Self {
        let clamp = |position: [f64; 2]| [
            position[0].clamp(JOINT_LIMITS[6].0, JOINT_LIMITS[6].1),
            position[1].clamp(JOINT_LIMITS[7].0, JOINT_LIMITS[7].1),
        ];
        let measured = clamp(measured);
        let home = clamp(home);
        Self { confirmed: measured, home, home_written: [measured[0] == home[0], measured[1] == home[1]],
            speed: RETURN_SPEED_RAD_S }
    }

    /// Slow pitch return for a tracking workspace handoff.
    pub fn gentle_home(measured: [f64; 2], home: [f64; 2]) -> Self {
        Self { speed: GENTLE_HOME_SPEED_RAD_S, ..Self::new(measured, home) }
    }

    pub fn next(&self, dt: f64) -> [f64; 2] {
        std::array::from_fn(|axis| {
            let difference = self.home[axis] - self.confirmed[axis];
            let step = self.speed[axis] * dt;
            if difference.abs() <= step { self.home[axis] }
            else { self.confirmed[axis] + step.copysign(difference) }
        })
    }

    pub fn confirm(&mut self, position: [f64; 2]) {
        for (axis, value) in position.iter().enumerate() {
            self.home_written[axis] |= *value == self.home[axis];
        }
        self.confirmed = position;
    }

    pub fn next_with_feedback(&self, measured: [f64; 2], dt: f64) -> [f64; 2] {
        self.next_with_feedback_tolerance(measured, dt, RETURN_TOLERANCE_COUNTS)
    }

    pub fn next_with_feedback_tolerance(&self, measured: [f64; 2], dt: f64, tolerance: u16) -> [f64; 2] {
        let initial = self.next(dt);
        std::array::from_fn(|axis| {
            if !self.home_written[axis] { return initial[axis]; }
            let joint = axis + 6;
            let (Ok(actual), Ok(goal)) = (duck_control::hx::encode_position(joint, measured[axis]),
                duck_control::hx::encode_position(joint, self.home[axis])) else {
                return self.confirmed[axis];
            };
            if actual.abs_diff(goal) <= tolerance { return self.confirmed[axis]; }
            let step = self.speed[axis] * dt;
            // Proportional compensation cannot wind up indefinitely against a stalled servo.
            let compensated = self.home[axis] + RETURN_FEEDBACK_GAIN * (self.home[axis] - measured[axis]);
            let correction = ((compensated - self.confirmed[axis]) * dt).clamp(-step, step);
            let (min, max) = TRACKING_LIMITS[axis];
            (self.confirmed[axis] + correction).clamp(min.min(self.home[axis]), max.max(self.home[axis]))
        })
    }

    pub fn arrived(&self, measured: [f64; 2]) -> bool {
        self.home_written.into_iter().all(|written| written) && (0..2).all(|axis| {
            let joint = axis + 6;
            match (duck_control::hx::encode_position(joint, measured[axis]),
                duck_control::hx::encode_position(joint, self.home[axis])) {
                (Ok(actual), Ok(goal)) => actual.abs_diff(goal) <= RETURN_TOLERANCE_COUNTS,
                _ => false,
            }
        })
    }
}

#[derive(Clone, Copy)]
struct HomeHold {
    home: [f64; 2],
    owner: u64,
    motion: Option<HeadReturn>,
    fixed: bool,
}

/// Return the head to physical HOME after tracking until another owner takes over.
#[derive(Default)]
pub struct HeadHandoff {
    tracking: bool,
    body_tracking: bool,
    command: Option<[f64; 2]>,
    measured: Option<[f64; 2]>,
    generation: u64,
    hold: Option<HomeHold>,
}

impl HeadHandoff {
    /// A departing workspace keeps its last target instead of starting a HOME return.
    pub fn hold_current(&mut self, measured: Option<[f64; 2]>, generation: u64) {
        if !self.tracking && !self.body_tracking && self.hold.is_none() { return; }
        let target = self.command.or(measured).or(self.measured);
        *self = Self::default();
        if let Some(target) = target {
            self.command = Some(target);
            self.measured = measured;
            self.generation = generation;
            self.hold = Some(HomeHold { home: target, owner: generation, motion: None, fixed: true });
        }
    }

    pub fn update(
        &mut self,
        tracking: bool,
        body_tracking: bool,
        command: Option<[f64; 2]>,
        measured: Option<[f64; 2]>,
        home: [f64; 2],
        generation: u64,
        takeover: bool,
        dt: f64,
    ) -> Option<[f64; 2]> {
        if takeover {
            *self = Self::default();
            return None;
        }
        let body_ended = self.body_tracking && !body_tracking;
        self.body_tracking = body_tracking;
        if tracking {
            if !self.tracking {
                self.hold = None;
                self.command = None;
                self.measured = None;
            }
            self.tracking = true;
            self.generation = generation;
            if let Some(measured) = measured {
                self.measured = Some(measured);
            }
            if let Some(command) = command {
                self.hold = None;
                return Some(command);
            }
            return self
                .command
                .or(self.measured);
        }
        if self.tracking || body_ended {
            self.tracking = false;
            self.hold = if generation == self.generation {
                Some(HomeHold { home, owner: generation, motion: None, fixed: false })
            } else {
                self.command = None;
                self.measured = None;
                None
            };
        } else if body_tracking {
            self.generation = generation;
            if self.hold.is_none() {
                self.command = None;
                if let Some(position) = measured { self.measured = Some(position); }
            }
        }
        match &mut self.hold {
            Some(hold) if hold.owner == generation => {
                if hold.fixed { return Some(hold.home); }
                // A missing bus read keeps the last known target until feedback resumes.
                let Some(measured) = measured else { return self.command.or(self.measured); };
                let motion = hold.motion.get_or_insert_with(|| HeadReturn::new(measured, hold.home));
                Some(motion.next(dt))
            }
            Some(_) => {
                self.hold = None;
                self.command = None;
                self.measured = None;
                None
            }
            None => None,
        }
    }

    pub fn confirm(&mut self, position: [f64; 2]) {
        if self.tracking || self.hold.is_some() {
            self.command = Some(position);
            if let Some(motion) = self.hold.as_mut().and_then(|hold| hold.motion.as_mut()) {
                motion.confirm(position);
            }
        }
    }
}

impl HeadTracker {
    /// Clear both PID states; the next step initializes from joint feedback.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Return joint targets in radians.
    ///
    /// Positions, deadzones and gains are ordered pitch then yaw; each gain set is `[kp, ki, kd]`.
    /// The normalized image point is `[x, y]`, with errors `y` for pitch and `-x` for yaw.
    /// Entering a deadzone captures the measured axis position once and clears its PID history.
    /// On target loss, hold the measured position within the tracking bounds and clear PID history.
    /// `dt` is in seconds. Invalid input or arithmetic overflow returns `None` without
    /// changing state.
    pub fn step(
        &mut self,
        positions: [f64; 2],
        point: Option<[f64; 2]>,
        deadzone: [f64; 2],
        gains: [[f64; 3]; 2],
        dt: f64,
    ) -> Option<[f64; 2]> {
        if !dt.is_finite()
            || dt <= 0.0
            || !positions.into_iter().all(f64::is_finite)
            || !deadzone.into_iter().all(|value| value.is_finite() && value >= 0.0)
            || !gains.into_iter().flatten().all(f64::is_finite)
            || point.is_some_and(|point| !point.into_iter().all(f64::is_finite))
        {
            return None;
        }

        let mut next = *self;
        let limits = *TRACKING_LIMITS;
        if !next.initialized {
            next.pitch.target = positions[0].clamp(limits[0].0, limits[0].1);
            next.yaw.target = positions[1].clamp(limits[1].0, limits[1].1);
            next.initialized = true;
        }

        let targets = if let Some([x, y]) = point {
            next.had_target = true;
            [
                next.pitch.step(y, deadzone[0], positions[0], gains[0], dt, limits[0])?,
                next.yaw.step(-x, deadzone[1], positions[1], gains[1], dt, limits[1])?,
            ]
        } else {
            if next.had_target {
                next.pitch.target = positions[0].clamp(limits[0].0, limits[0].1);
                next.yaw.target = positions[1].clamp(limits[1].0, limits[1].1);
                next.had_target = false;
            }
            next.pitch.hold();
            next.yaw.hold();
            [next.pitch.target, next.yaw.target]
        };

        *self = next;
        Some(targets)
    }
}
