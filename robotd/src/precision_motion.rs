//! Time and command encoding for the handpad's displacement policy.

use std::time::{Duration, Instant};

use duck_ipc_proto::PrecisionAxis;

const MOVE_END: Duration = Duration::from_millis(500);
const SETTLE_END: Duration = Duration::from_millis(1800);

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Frame {
    pub command: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct PrecisionMotion {
    started: Instant,
    encoded: [f64; 3],
}

impl PrecisionMotion {
    pub(crate) fn new(started: Instant, axis: PrecisionAxis, value: f64) -> Result<Self, &'static str> {
        let (index, unit) = match axis {
            PrecisionAxis::X => (0, 0.05),
            PrecisionAxis::Y => (1, 0.05),
            PrecisionAxis::Yaw => (2, 0.25),
        };
        let encoded_value = value / unit;
        if !value.is_finite() || !encoded_value.is_finite() || !(encoded_value as f32).is_finite() {
            return Err("precision command cannot be represented as a finite model input");
        }
        let mut encoded = [0.0; 3];
        encoded[index] = encoded_value;
        Ok(Self { started, encoded })
    }

    pub(crate) fn frame(&self, now: Instant) -> Option<Frame> {
        let elapsed = now.saturating_duration_since(self.started);
        if elapsed < MOVE_END {
            Some(Frame { command: self.encoded })
        } else if elapsed < SETTLE_END {
            Some(Frame { command: [0.0; 3] })
        } else {
            None
        }
    }
}
