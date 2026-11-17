#[derive(Debug, Clone, Copy, Default)]
pub struct RollPhase {
    angle: f64,
    inverted: bool,
    previous_pitch: Option<f64>,
    pub completed: bool,
    inverted_goal: bool,
    reference: Option<[f64; 4]>,
    previous_twist: f64,
    forward_seen: bool,
    backward_seen: bool,
    ambiguous: bool,
}

impl RollPhase {
    /// A half roll ending in a persistent inverted hold, matching headstand training.
    pub fn headstand() -> Self {
        Self {
            inverted_goal: true,
            ..Self::default()
        }
    }

    pub fn holds_inverted(&self) -> bool {
        self.inverted_goal
    }

    pub fn update(&mut self, gravity: [f64; 3], quaternion: [f64; 4]) -> Result<[f64; 3], &'static str> {
        if !gravity.iter().all(|value| value.is_finite()) {
            return Err("roll phase requires finite projected gravity");
        }
        if !self.inverted_goal {
            return self.update_forward(gravity, quaternion);
        }
        let pitch = gravity[0].atan2(-gravity[2]);
        let delta = self.previous_pitch.map_or(0.0, |previous| {
            (pitch - previous + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI
        });
        self.previous_pitch = Some(pitch);
        let sagittal = gravity[1].abs() <= 0.5;
        if !self.completed {
            if delta <= 0.0 || sagittal {
                self.angle += delta;
            }
            self.inverted |= sagittal && gravity[2] > 0.5;
            let (completion_degrees, alignment) = if self.inverted_goal {
                (170.0_f64, gravity[2])
            } else {
                (330.0_f64, -gravity[2])
            };
            self.completed = self.inverted
                && sagittal
                && self.angle >= completion_degrees.to_radians()
                && alignment >= 30.0_f64.to_radians().cos();
        }
        Ok(self.command())
    }

    fn update_forward(&mut self, gravity: [f64; 3], quaternion: [f64; 4]) -> Result<[f64; 3], &'static str> {
        let [w, x, y, z] = normalise_quaternion(quaternion)?;
        if self.completed {
            return Ok(self.command());
        }
        let reference = self.reference.get_or_insert([w, x, y, z]);
        let [rw, rx, ry, rz] = *reference;
        let relative_w = rw*w+rx*x+ry*y+rz*z;
        let relative_y = rw*y-ry*w-rz*x+rx*z;
        if relative_w*relative_w+relative_y*relative_y < 0.25_f64.powi(2) {
            self.ambiguous = true;
        } else {
            let twist = 2.0*relative_y.atan2(relative_w);
            self.angle += (twist-self.previous_twist+std::f64::consts::PI)
                .rem_euclid(std::f64::consts::TAU)-std::f64::consts::PI;
            self.previous_twist = twist;
        }
        self.forward_seen |= gravity[0] >= 0.5;
        self.inverted |= self.forward_seen && gravity[2] > 0.5;
        self.backward_seen |= self.inverted && gravity[0] <= -0.5;
        self.completed = !self.ambiguous && self.backward_seen
            && self.angle >= 330.0_f64.to_radians()
            && -gravity[2] >= 30.0_f64.to_radians().cos();
        Ok(self.command())
    }

    pub fn command(&self) -> [f64; 3] {
        let progress = if self.completed && !self.inverted_goal {
            1.0
        } else {
            (self.angle / std::f64::consts::TAU).clamp(0.0, 1.0)
        };
        [progress, if self.completed { 1.0 } else { 0.0 }, 0.0]
    }
}

fn normalise_quaternion(quaternion: [f64; 4]) -> Result<[f64; 4], &'static str> {
    if !quaternion.iter().all(|value| value.is_finite()) {
        return Err("roll orientation requires a finite wxyz quaternion");
    }
    let norm = quaternion.iter().map(|v| v*v).sum::<f64>().sqrt();
    if norm < 1e-12 {
        return Err("roll orientation requires a nonzero quaternion");
    }
    Ok(quaternion.map(|v| v/norm))
}

pub const HEADING_OBSERVATION: &str = "relative_lateral_yaw";

pub fn heading_contract(value: Option<&str>, forward_roll: bool) -> Result<bool, &'static str> {
    match value {
        None | Some("") => Ok(false),
        Some(HEADING_OBSERVATION) if forward_roll => Ok(true),
        _ => Err("relative_lateral_yaw requires a forward-roll skill"),
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct RollHeadingReference {
    reference: Option<f64>,
}

impl RollHeadingReference {
    pub fn clear(&mut self) {
        self.reference = None;
    }

    /// Capture the first actual skill sample; retain it through the entire roll.
    pub fn update(&mut self, quaternion: [f64; 4]) -> Result<f64, &'static str> {
        let [w, x, y, z] = normalise_quaternion(quaternion)?;
        let heading = (-2.0*(x*y-w*z)).atan2(1.0-2.0*(x*x+z*z));
        let reference = self.reference.get_or_insert(heading);
        let delta = heading-*reference;
        Ok(delta.sin().atan2(delta.cos()))
    }
}
