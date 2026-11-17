//! The upstream idle yawn: slow open, hold, quicker close, with a small upward look.
//!
//! Extracted from upstream `idle_head.rs` at 942e58e3aa6dbb28e7037203fff11866122c7567.
//! This feature adds the yawn only; it does not schedule glances or body-height changes.
//! The control loop owns eligibility and actuator writes. No threads, I/O or allocations.

use std::time::{Duration, Instant};

use robotd_params::IdleParams;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Offset {
    pub pitch: f64,
    pub mouth: f64,
}

impl Offset {
    /// A yawn never closes a manually opened mouth; startup hold keeps manual control.
    pub fn mouth_target(self, manual: f64, driving: bool) -> f64 {
        duck_control::model::mouth_target(if driving {
            manual.max(self.mouth)
        } else {
            manual
        })
    }
}

pub struct IdleYawn {
    tuning: IdleParams,
    rng: u64,
    still_since: Option<Instant>,
    started: Option<Instant>,
    yawn: Option<f64>,
    gain: f64,
}

impl IdleYawn {
    pub fn new(tuning: IdleParams, seed: u64) -> Self {
        Self {
            tuning,
            rng: seed.max(1),
            still_since: None,
            started: None,
            yawn: None,
            gain: 0.0,
        }
    }

    pub fn after(&self) -> Duration {
        Duration::from_secs_f64(self.tuning.after_s)
    }

    pub fn tick(&mut self, now: Instant, still: bool, dt: Duration) -> Offset {
        let p = self.tuning;
        self.still_since = if still && p.enabled {
            Some(self.still_since.unwrap_or(now))
        } else {
            None
        };
        let idle = self
            .still_since
            .is_some_and(|since| now.duration_since(since) >= self.after());
        let step = (dt.as_secs_f64() / p.fade_s).clamp(0.0, 1.0);
        self.gain = if idle {
            (self.gain + step).min(1.0)
        } else {
            (self.gain - step).max(0.0)
        };
        if idle && self.started.is_none() {
            self.started = Some(now);
        }
        let Some(started) = self.started else {
            return Offset::default();
        };
        if self.gain == 0.0 {
            self.started = None;
            self.yawn = None;
            return Offset::default();
        }

        let t = now.duration_since(started).as_secs_f64();
        if self.yawn.is_some_and(|start| t - start >= p.yawn_s) {
            self.yawn = None;
        }
        if idle
            && self.yawn.is_none()
            && p.yawn_every_s > 0.0
            && t >= p.yawn_after_s
            && self.random_unit() < dt.as_secs_f64() / p.yawn_every_s
        {
            self.yawn = Some(t);
            tracing::info!("idle yawn started");
        }
        let yawn = self
            .yawn
            .map_or(0.0, |start| yawn_shape((t - start) / p.yawn_s));
        Offset {
            pitch: self.gain * p.yawn_pitch * yawn,
            mouth: self.gain * p.yawn_open * yawn,
        }
    }

    /// Upstream's xorshift draw: variety without a dependency or a control-loop allocation.
    fn random_unit(&mut self) -> f64 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        (self.rng >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn ease(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    x * x * x * (10.0 + x * (-15.0 + 6.0 * x))
}

fn yawn_shape(x: f64) -> f64 {
    match x {
        x if x < 0.4 => ease(x / 0.4),
        x if x < 0.7 => 1.0,
        x => 1.0 - ease((x - 0.7) / 0.3),
    }
}
