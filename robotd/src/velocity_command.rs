//! Gamepad start delay and short-release command, before the command EMA.

use std::time::Duration;

pub(crate) const START_DELAY: Duration = Duration::ZERO;
pub(crate) const SHORT_DURATION: Duration = Duration::ZERO;

#[derive(Clone, Copy, Debug)]
pub(crate) struct GamepadTiming {
    pub start_delay: Duration,
    pub short_duration: Duration,
}

impl Default for GamepadTiming {
    fn default() -> Self {
        Self {
            start_delay: START_DELAY,
            short_duration: SHORT_DURATION,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct VelocityCommandPulse {
    pressed_at: Option<Duration>,
    start_delay: Duration,
    requested: [f64; 3],
    short_until: Option<Duration>,
    short_target: [f64; 3],
    blocked: bool,
}

impl VelocityCommandPulse {
    pub(crate) fn cancel(&mut self) {
        *self = Self {
            blocked: true,
            ..Self::default()
        };
    }

    pub(crate) fn update(
        &mut self,
        requested: [f64; 3],
        now: Duration,
        linear: Option<f64>,
        lateral: Option<f64>,
        backward: Option<f64>,
        angular: Option<f64>,
        timing: GamepadTiming,
    ) {
        self.start_delay = if timing.start_delay.is_zero() || timing.short_duration.is_zero() {
            Duration::ZERO
        } else {
            timing.start_delay
        };
        if requested.iter().any(|value| !value.is_finite()) {
            self.cancel();
            return;
        }
        let moving = requested.iter().any(|value| *value != 0.0);
        if self.blocked {
            if !moving {
                self.blocked = false;
            }
            return;
        }
        if moving {
            if self.pressed_at.is_none() {
                self.pressed_at = Some(now);
            }
            self.requested = requested;
            self.short_until = None;
        } else if let Some(start) = self.pressed_at.take() {
            if now.saturating_sub(start) < self.start_delay {
                let [x, y, yaw] = self.requested;
                let norm = x.hypot(y);
                let mut target = if norm > 0.0 {
                    [0.05 * x / norm, 0.05 * y / norm, 0.0]
                } else {
                    [0.0; 3]
                };
                if yaw != 0.0 {
                    target[2] = 0.1 * yaw.signum();
                }
                let x_limit = if x < 0.0 {
                    match (linear, backward) {
                        (Some(a), Some(b)) => Some(a.min(b)),
                        (a, b) => a.or(b),
                    }
                } else {
                    linear
                };
                if let Some(limit) = x_limit {
                    target[0] = target[0].clamp(-limit, limit);
                }
                if let Some(limit) = lateral {
                    target[1] = target[1].clamp(-limit, limit);
                }
                if let Some(limit) = angular {
                    target[2] = target[2].clamp(-limit, limit);
                }
                self.short_target = target;
                self.short_until = Some(now + timing.short_duration);
            }
            self.requested = [0.0; 3];
        }
    }

    pub(crate) fn target(&self, now: Duration) -> [f64; 3] {
        if let Some(start) = self.pressed_at {
            if now.saturating_sub(start) >= self.start_delay {
                return self.requested;
            }
        } else if self.short_until.is_some_and(|until| now < until) {
            return self.short_target;
        }
        [0.0; 3]
    }
}
