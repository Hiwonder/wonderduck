//! Incremental control of the HX pitch and yaw servos from a directional stick.

use std::time::{Duration, Instant};

use crate::hx::{MANUAL_HEAD_POSITION_LIMITS, encode_position};
use duck_ipc_proto::RobotState;

const STEP_INTERVAL: Duration = Duration::from_millis(5);
const STEP_COUNTS: f64 = 10.0;
const MAX_STEP_ELAPSED: Duration = Duration::from_millis(150);
pub const FEEDBACK_TTL: Duration = Duration::from_millis(300);

pub fn measured_position(state: &RobotState) -> Option<[u16; 2]> {
    let position = |joint| {
        state
            .servo_status
            .get(joint)
            .map(|servo: &duck_ipc_proto::ServoStatus| servo.raw)
            .or_else(|| encode_position(joint, *state.joints.get(joint)?).ok())
    };
    Some([position(6)?, position(7)?])
}

#[derive(Debug, Default)]
pub struct HeadStick {
    goal: Option<[f64; 2]>,
    last_step: Option<Instant>,
}

impl HeadStick {
    /// Positive input increases encoder counts. Input is ordered yaw, pitch.
    pub fn update(&mut self, [yaw, pitch]: [f64; 2], measured: Option<[u16; 2]>, now: Instant) {
        // Keep the previous goal while the stick is centered or feedback is temporarily unavailable.
        let Some(measured) = measured.filter(|_| yaw != 0.0 || pitch != 0.0) else {
            self.last_step = None;
            return;
        };
        let elapsed = self
            .last_step
            .map_or(STEP_INTERVAL, |last| now.saturating_duration_since(last))
            .min(MAX_STEP_ELAPSED);
        let steps = (elapsed.as_millis() / STEP_INTERVAL.as_millis()) as u32;
        if steps == 0 {
            return;
        }
        self.last_step = Some(now - (elapsed - STEP_INTERVAL * steps));
        let goal = self.goal.get_or_insert_with(|| measured.map(f64::from));
        for ((position, deflection), (min, max)) in goal
            .iter_mut()
            .zip([pitch, yaw])
            .zip(MANUAL_HEAD_POSITION_LIMITS)
        {
            *position = (*position + deflection * f64::from(steps) * STEP_COUNTS)
                .clamp(f64::from(min), f64::from(max));
        }
    }

    pub fn raw_position(&self) -> Option<[u16; 2]> {
        self.goal
            .map(|goal| goal.map(|position| position.round() as u16))
    }
}
