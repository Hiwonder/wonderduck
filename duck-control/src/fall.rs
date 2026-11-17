//! Seeing a fall *start*, rather than confirming one that already finished.
//!
//! [`crate::safety::Safety`] has a fall verdict already, and it is the right one for what
//! it does: projected gravity past `fall_gravity_z` (about 60° from upright) held for
//! `fall_debounce` (200 ms). By the time that latches the robot is on the floor, or a few
//! tens of milliseconds from it. That is fine for "refuse to enable a robot lying on its
//! side" and useless for "drop the gains before it lands" — the whole value of limping is
//! spent in the window this predictor exists to find.
//!
//! So this is a second, deliberately separate detector. It answers a different question —
//! *is the robot on its way down right now* — and it answers it early enough to act on,
//! which means it answers on a rate rather than on a position.
//!
//! # How
//!
//! Projected gravity in the trunk frame rotates with the trunk, so its derivative is
//! exactly `ġ = −ω × g` for body-frame angular velocity `ω`. Only the z component matters
//! (that is what the fall threshold reads), which is one multiply-subtract:
//!
//! ```text
//! ġz = −(ωx·gy − ωy·gx)
//! ```
//!
//! No filtering, no differentiation of the quaternion: the gyro is a direct measurement
//! arriving in the same 12-byte block, and differentiating the SFLP quaternion instead
//! would add the filter's own lag to the very number whose whole point is to be early.
//!
//! Extrapolate that linearly over [`FallPredictor`]'s lookahead and the test is "where
//! will gravity be a quarter-second from now" — trigger when *that* is past the point of
//! no return. Three conditions, all required:
//!
//!  - **Already tilted** past `tilt_z`. A gyro spike on an upright robot is a footfall, a
//!    shove that it will absorb, or somebody picking it up. None of those is a fall, and
//!    a predictor with no position gate calls all three one.
//!  - **Still tipping over** (`ġz > 0`) — going *down*, not recovering from a lean.
//!  - **Predicted past `predicted_z`** at the lookahead horizon.
//!
//! and then debounced for a few ticks, because one noisy sample must not cost the robot
//! its gains mid-stride.
//!
//! A fall is eligible only after upright, quiet samples have persisted for one second.
//! A stand-up can briefly cross the fall-verdict threshold and then rock back; that
//! is still a stand-up, not evidence of a fall from standing. Resetting or firing consumes
//! this permission, so a floor start and every hand-back must establish it again.
//!
//! # The tuning is the whole feature
//!
//! Too early and the robot limps out of leans it would have walked off — every false
//! positive is a fall the robot *caused*, which is strictly worse than the fall being
//! dampened. Too late and it lands stiff and the mode has bought nothing. The defaults sit
//! deliberately on the late side of that: `tilt_z = -0.90` is about 26° from upright,
//! which ordinary walking does not reach, and 60 ms of debounce is three ticks at 50 Hz —
//! longer than any footfall impulse, short enough to leave most of the fall to limp
//! through.

use std::time::Duration;

use crate::imu::ImuData;

// Match the existing recorded stand-up check: within 20 degrees, below 0.3 rad/s,
// continuously for one second. Landing's looser 200 ms/1 rad/s test is not evidence
// that a partial rise has finished.
const STANDING_GRAVITY_Z: f64 = -0.939_692_620_785_908_4;
const STANDING_RATE: f64 = 0.3;
const STANDING_HOLD: Duration = Duration::from_secs(1);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FallPredictorConfig {
    /// Projected-gravity z the robot must *already* be past before a prediction counts.
    /// Upright is -1.0; -0.90 is about 26° of tilt.
    pub tilt_z: f64,
    /// Where the extrapolation has to reach to count as a fall in progress. The same sense
    /// as `SafetyConfig::fall_gravity_z`, and by default the same number: "it will be
    /// what safety calls fallen, in `lookahead`".
    pub predicted_z: f64,
    /// How far ahead to extrapolate.
    pub lookahead: Duration,
    /// How long the verdict must hold before it fires.
    pub debounce: Duration,
}

impl Default for FallPredictorConfig {
    fn default() -> Self {
        Self {
            tilt_z: -0.90,
            predicted_z: -0.5,
            lookahead: Duration::from_millis(300),
            debounce: Duration::from_millis(60),
        }
    }
}

/// Watches the IMU for a fall that has started but not landed.
///
/// Edge-triggered: [`Self::observe`] returns `true` on the tick the debounce completes and
/// then stays `false` until quiet upright standing is confirmed again. The caller owns
/// what to do about it and how long to keep doing it — this only reports the moment.
#[derive(Debug, Clone)]
pub struct FallPredictor {
    config: FallPredictorConfig,
    /// How long the three conditions have held together. Any tick that fails one resets it.
    falling_for: Duration,
    upright_for: Duration,
    armed: bool,
}

impl FallPredictor {
    pub fn new(config: FallPredictorConfig) -> Self {
        Self {
            config,
            falling_for: Duration::ZERO,
            upright_for: Duration::ZERO,
            armed: false,
        }
    }

    /// Rate of change of projected-gravity z, per second, from `ġ = −ω × g`.
    ///
    /// Positive means the trunk is tipping *away* from upright — gravity z climbing from
    /// -1 towards 0. Public because it is the number to look at when tuning the
    /// thresholds against a recording, and it is not otherwise recoverable from outside.
    pub fn gravity_z_rate(imu: &ImuData) -> f64 {
        -(imu.gyro[0] * imu.gravity[1] - imu.gyro[1] * imu.gravity[0])
    }

    /// Where gravity z is heading, `lookahead` from now, at the current rate.
    pub fn predicted_z(&self, imu: &ImuData) -> f64 {
        imu.gravity[2] + Self::gravity_z_rate(imu) * self.config.lookahead.as_secs_f64()
    }

    /// Whether the existing quiet, upright standing check has completed.
    pub fn standing_confirmed(&self) -> bool {
        self.armed
    }

    /// One tick. `true` exactly once per fall, when the debounce completes.
    pub fn observe(&mut self, imu: &ImuData, dt: Duration) -> bool {
        if !self.armed {
            let rate_squared = imu.gyro.iter().map(|w| w * w).sum::<f64>();
            let upright = imu.gravity[2] < STANDING_GRAVITY_Z
                && rate_squared < STANDING_RATE * STANDING_RATE;
            self.upright_for = if upright {
                self.upright_for.saturating_add(dt)
            } else {
                Duration::ZERO
            };
            self.armed = self.upright_for >= STANDING_HOLD;
            return false;
        }
        let rate = Self::gravity_z_rate(imu);
        let going_down = imu.gravity[2] > self.config.tilt_z
            && rate > 0.0
            && self.predicted_z(imu) > self.config.predicted_z;

        if !going_down {
            self.falling_for = Duration::ZERO;
            return false;
        }

        self.falling_for = self.falling_for.saturating_add(dt);
        if self.falling_for >= self.config.debounce {
            self.reset();
            return true;
        }
        false
    }

    /// Forget the accumulated verdict — the caller has taken charge (it is limping, or the
    /// policy stopped driving), so the next fall must be detected from scratch.
    pub fn reset(&mut self) {
        self.falling_for = Duration::ZERO;
        self.upright_for = Duration::ZERO;
        self.armed = false;
    }
}
