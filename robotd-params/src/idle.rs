//! Startup tuning for the idle yawn, extracted from the upstream idle behaviour.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct IdleParams {
    pub enabled: bool,
    /// Seconds still before the idle spell starts.
    pub after_s: f64,
    /// Seconds to fade in and out when motion resumes.
    pub fade_s: f64,
    /// Seconds into the idle spell before yawns are eligible.
    pub yawn_after_s: f64,
    /// Mean random wait between yawns; zero disables them.
    pub yawn_every_s: f64,
    /// Length of one yawn, seconds.
    pub yawn_s: f64,
    /// Mouth opening fraction, bounded by the robot's calibrated travel.
    pub yawn_open: f64,
    /// Head pitch command offset, radians; negative looks up.
    pub yawn_pitch: f64,
}

impl Default for IdleParams {
    fn default() -> Self {
        Self {
            enabled: true,
            after_s: 2.0,
            fade_s: 1.5,
            yawn_after_s: 45.0,
            yawn_every_s: 90.0,
            yawn_s: 3.0,
            yawn_open: 0.6,
            yawn_pitch: -0.15,
        }
    }
}

impl IdleParams {
    pub(crate) fn invalid(&self) -> Option<(&'static str, &'static str)> {
        for (key, value) in [
            ("after_s", self.after_s),
            ("fade_s", self.fade_s),
            ("yawn_after_s", self.yawn_after_s),
            ("yawn_every_s", self.yawn_every_s),
            ("yawn_s", self.yawn_s),
            ("yawn_open", self.yawn_open),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Some((key, "must be finite and nonnegative"));
            }
        }
        if self.fade_s == 0.0 {
            return Some(("fade_s", "must be above zero"));
        }
        if self.yawn_s == 0.0 {
            return Some(("yawn_s", "must be above zero"));
        }
        if self.yawn_open > 1.0 {
            return Some(("yawn_open", "must be at most 1"));
        }
        if !self.yawn_pitch.is_finite() {
            return Some(("yawn_pitch", "must be finite"));
        }
        None
    }
}
