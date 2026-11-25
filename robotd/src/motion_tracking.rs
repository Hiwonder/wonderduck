//! Velocity PID for color targets and line following.

#[derive(Clone, Copy, Default)]
struct Axis {
    integral: f64,
    previous: Option<f64>,
}

impl Axis {
    fn step(&mut self, error: f64, dt: f64, [kp, ki, kd]: [f64; 3]) -> f64 {
        self.integral += error * dt;
        let derivative = self.previous.map_or(0.0, |previous| (error - previous) / dt);
        self.previous = Some(error);
        kp * error + ki * self.integral + kd * derivative
    }
}

#[derive(Clone, Copy, Default)]
pub struct MotionTracker {
    forward: Axis,
    yaw: Axis,
}

pub struct TrackingOutput {
    pub twist: [f64; 3],
    pub in_deadzone: [bool; 2],
}

impl MotionTracker {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn follow_line(
        &mut self,
        target: Option<[f64; 2]>,
        speed: f64,
        target_x: f64,
        deadzone: f64,
        gains: [f64; 3],
        dt: f64,
    ) -> Option<TrackingOutput> {
        let Some([horizontal, aspect]) = target else {
            self.reset();
            return None;
        };
        let error = horizontal - (target_x / 160.0 - 1.0);
        let centered = error.abs() / 2.0 <= deadzone;
        let yaw = if centered {
            self.yaw = Axis::default();
            0.0
        } else {
            let angle = -(error * aspect).atan();
            self.yaw.step(angle, dt, gains)
        };
        Some(TrackingOutput { twist: [speed, 0.0, yaw], in_deadzone: [false, centered] })
    }

    pub fn step(
        &mut self,
        target: Option<[f64; 2]>,
        reference_area: f64,
        deadzone: [f64; 2],
        gains: [[f64; 3]; 2],
        dt: f64,
    ) -> Option<TrackingOutput> {
        let Some([horizontal, area]) = target else {
            self.reset();
            return None;
        };
        let area_error = (reference_area - area) / reference_area;
        let in_deadzone = [
            deadzone[0] > 0.0 && area_error.abs() <= deadzone[0],
            deadzone[1] > 0.0 && horizontal.abs() / 2.0 <= deadzone[1],
        ];
        let forward = if in_deadzone[0] {
            self.forward = Axis::default();
            0.0
        } else { self.forward.step(area_error, dt, gains[0]) };
        let yaw = if in_deadzone[1] {
            self.yaw = Axis::default();
            0.0
        } else { self.yaw.step(-horizontal, dt, gains[1]) };
        Some(TrackingOutput { twist: [
            forward,
            0.0,
            yaw,
        ], in_deadzone })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_pid_uses_image_angle_and_resets_on_loss() {
        let mut tracker = MotionTracker::default();
        let gains = [1.1, 0.0, 0.1];
        let centered = tracker.follow_line(Some([0.01, 16.0 / 9.0]), 0.15, 160.0, 0.01, gains, 0.02).unwrap();
        assert_eq!(centered.twist, [0.15, 0.0, 0.0]);
        let right = tracker.follow_line(Some([0.5, 16.0 / 9.0]), 0.15, 160.0, 0.01, gains, 0.02).unwrap();
        assert_eq!(right.twist, [0.15, 0.0, -1.1 * (8.0_f64 / 9.0).atan()]);
        assert!(tracker.follow_line(None, 0.15, 160.0, 0.01, gains, 0.02).is_none());
        let left = tracker.follow_line(Some([-0.5, 16.0 / 9.0]), 0.15, 160.0, 0.01, gains, 0.02).unwrap();
        assert_eq!(left.twist[2], -right.twist[2]);
    }
}
