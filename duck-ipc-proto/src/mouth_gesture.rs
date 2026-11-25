//! Mouth trigger timing shared by the physical gamepad and WebSocket controls.

use std::time::{Duration, Instant};

use crate::{SoundParams, SoundTag};

pub const PRESSURE_START: f64 = 0.05;
const PRESSURE_RELEASE: f64 = 0.03;
const REPEAT_AFTER: Duration = Duration::from_secs(1);

#[derive(Debug, Default)]
pub struct MouthGesture {
    pressed: bool,
    held_since: Option<Instant>,
    pending_press: Option<Instant>,
}

impl MouthGesture {
    pub fn update(&mut self, pressure: f64, held: bool, now: Instant) {
        if pressure < PRESSURE_RELEASE {
            self.pressed = false;
        } else if pressure >= PRESSURE_START && !self.pressed {
            self.pressed = true;
            self.pending_press = Some(now);
        }
        if self.pressed && held {
            self.held_since.get_or_insert(now);
        } else {
            self.held_since = None;
        }
    }

    pub fn repeating(&self, now: Instant) -> bool {
        self.held_since
            .is_some_and(|at| now.saturating_duration_since(at) >= REPEAT_AFTER)
    }

    pub fn take_press(&mut self, now: Instant, max_age: Duration) -> Option<SoundParams> {
        self.pending_press
            .take()
            .filter(|at| now.saturating_duration_since(*at) <= max_age)
            .map(|_| SoundParams {
                tag: SoundTag::Mouth,
                hold: None,
            })
    }

    pub fn cancel(&mut self) {
        *self = Self::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_opening_plays_once_then_repeats_after_one_second() {
        let now = Instant::now();
        let freshness = Duration::from_millis(150);
        let mut gesture = MouthGesture::default();
        gesture.update(PRESSURE_START, true, now);
        assert!(gesture.take_press(now, freshness).is_some());
        for elapsed in [20, 100, 350, 999, 1000, 2000] {
            let tick = now + Duration::from_millis(elapsed);
            gesture.update(0.04, true, tick);
            gesture.update(1.0, true, tick);
            assert!(gesture.take_press(tick, freshness).is_none());
            assert_eq!(gesture.repeating(tick), elapsed >= 1000);
        }
        let next = now + Duration::from_secs(3);
        gesture.update(1.0, false, next);
        assert!(!gesture.repeating(next));
        gesture.update(1.0, true, next);
        assert!(!gesture.repeating(next));
        gesture.update(0.0, false, next);
        assert!(!gesture.repeating(next));
        assert!(gesture.take_press(next, freshness).is_none());
        gesture.update(1.0, true, next);
        assert!(!gesture.repeating(next));
        assert!(gesture.take_press(next, freshness).is_some());
        assert!(gesture.take_press(next, freshness).is_none());
    }

    #[test]
    fn cancelling_a_press_discards_an_unsent_sound() {
        let now = Instant::now();
        let mut gesture = MouthGesture::default();
        gesture.update(1.0, true, now);
        gesture.cancel();
        assert!(!gesture.repeating(now + REPEAT_AFTER));
        assert!(
            gesture
                .take_press(now, Duration::from_millis(150))
                .is_none()
        );
    }
}
