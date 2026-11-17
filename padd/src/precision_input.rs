//! Classifies mapped stick gestures and replays long gestures after 300 ms.
//!
//! This module owns no clock or I/O. Call [`GestureInput::step`] with fresh samples and
//! schedule another call at [`Output::next_deadline`] when necessary. Deadlines are
//! expressed in the same monotonic time domain as the input timestamps.

use std::collections::VecDeque;
use std::time::Duration;

const GESTURE_DELAY: Duration = Duration::from_millis(300);
const ZERO: [f64; 3] = [0.0; 3];

/// A single relative target: axis 0/1 is in metres, and axis 2 is in radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShortTarget {
    pub axis: usize,
    pub displacement: f64,
}

/// One original velocity sample and its intended replay time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReplayedVelocity {
    pub due_at: Duration,
    pub velocity: [f64; 3],
}

/// The result of one sample. Targets and replayed samples are returned only once.
#[derive(Debug, PartialEq)]
pub struct Output {
    /// A fresh nonzero input began; the caller owns any policy interruption.
    pub started: bool,
    /// Most recent velocity due for replay, or zero before replay and after cancellation.
    pub delayed_velocity: [f64; 3],
    /// A short-gesture target, unless it started during an earlier long gesture's tail.
    pub short_target: Option<ShortTarget>,
    /// Every newly due sample, in order. Multiple samples can be due in one call.
    pub replayed: Vec<ReplayedVelocity>,
    /// Earliest recognition or replay deadline that requires another fresh sample.
    pub next_deadline: Option<Duration>,
}

#[derive(Clone, Copy, Debug)]
struct Sample {
    at: Duration,
    axes: [f64; 3],
    velocity: [f64; 3],
}

#[derive(Debug)]
enum ActiveGesture {
    Identifying {
        start: Duration,
        samples: Vec<Sample>,
        discard_short: bool,
    },
    Long,
}

/// An independent gesture recognizer. It does not perform smoothing or issue commands.
#[derive(Debug, Default)]
pub struct GestureInput {
    active: Option<ActiveGesture>,
    replay: VecDeque<Sample>,
    delayed_velocity: [f64; 3],
    last_long_release: Option<Duration>,
    last_time: Option<Duration>,
    wait_for_neutral: bool,
}

impl GestureInput {
    pub fn new() -> Self {
        Self::default()
    }

    /// Processes one sample. `now` must be monotonic, including across `cancel`.
    ///
    /// `axes` are already dead-zone filtered and mapped to x, y, yaw; `velocity` is
    /// the corresponding velocity sample. Any nonzero axis starts a gesture. A return
    /// to all-zero axes before 300 ms produces a target from the last nonzero sample;
    /// ties between axis magnitudes choose the lowest index. For x and y, displacement
    /// is velocity times 0.1 seconds; for yaw, it is angular velocity times 15 degrees
    /// per radian/second. No velocity or displacement limits are applied. At 300 ms it
    /// is long. A short gesture that starts before an earlier long gesture's final
    /// zero is due is discarded, even if its release occurs after that deadline.
    ///
    /// A long gesture retains each velocity sample and ends with a zero sample. If the
    /// supplied velocity at release is nonzero, it is preserved immediately before
    /// that zero. Samples are due exactly 300 ms after their input time. A late call
    /// returns all overdue samples in `replayed`, but cannot reproduce elapsed wall
    /// time; callers needing exact timing must call at the reported deadlines. Process
    /// `replayed` in order when every sample matters; `delayed_velocity` is the latest
    /// value and is not an additional sample to replay.
    pub fn step(&mut self, now: Duration, axes: [f64; 3], velocity: [f64; 3]) -> Output {
        if let Some(last_time) = self.last_time {
            assert!(now >= last_time, "gesture input time must be monotonic");
        }
        self.last_time = Some(now);

        let pressed = axes.iter().any(|axis| *axis != 0.0);
        let sample = Sample {
            at: now,
            axes,
            velocity,
        };
        let mut started = false;
        let mut short_target = None;

        if self.wait_for_neutral {
            if !pressed {
                self.wait_for_neutral = false;
            }
        } else {
            match self.active.take() {
                None if pressed => {
                    started = true;
                    self.active = Some(ActiveGesture::Identifying {
                        start: now,
                        samples: vec![sample],
                        discard_short: self.discard_short_during_long_tail(now),
                    });
                }
                None => {}
                Some(ActiveGesture::Identifying {
                    start,
                    mut samples,
                    discard_short,
                }) => {
                    if pressed {
                        samples.push(sample);
                    }
                    if now - start >= GESTURE_DELAY {
                        self.replay.extend(samples);
                        if pressed {
                            self.active = Some(ActiveGesture::Long);
                        } else {
                            self.finish_long(sample);
                        }
                    } else if pressed {
                        self.active = Some(ActiveGesture::Identifying {
                            start,
                            samples,
                            discard_short,
                        });
                    } else if !discard_short {
                        let last = samples.last().expect("a gesture has an initial sample");
                        short_target = Some(target_from(*last));
                    }
                }
                Some(ActiveGesture::Long) if pressed => {
                    self.replay.push_back(sample);
                    self.active = Some(ActiveGesture::Long);
                }
                Some(ActiveGesture::Long) => self.finish_long(sample),
            }
        }

        let mut replayed = Vec::new();
        while self
            .replay
            .front()
            .is_some_and(|sample| now - sample.at >= GESTURE_DELAY)
        {
            let sample = self.replay.pop_front().expect("the front sample exists");
            self.delayed_velocity = sample.velocity;
            replayed.push(ReplayedVelocity {
                // A due sample's deadline cannot exceed `now`.
                due_at: sample.at + GESTURE_DELAY,
                velocity: sample.velocity,
            });
        }

        Output {
            started,
            delayed_velocity: self.delayed_velocity,
            short_target,
            replayed,
            next_deadline: self.next_deadline(),
        }
    }

    /// Clears all pending motion. A held stick must return to neutral before rearming.
    /// The caller must issue its own immediate emergency stop.
    pub fn cancel(&mut self) {
        self.wait_for_neutral |= self.active.is_some();
        self.active = None;
        self.replay.clear();
        self.delayed_velocity = ZERO;
        self.last_long_release = None;
    }

    /// Clears motion when the current stick position is unknown or its mapping changed.
    pub fn cancel_until_neutral(&mut self) {
        self.cancel();
        self.wait_for_neutral = true;
    }

    // Keep the short-gesture policy separate from long-gesture replay.
    fn discard_short_during_long_tail(&self, now: Duration) -> bool {
        self.last_long_release
            .is_some_and(|release| now - release < GESTURE_DELAY)
    }

    fn finish_long(&mut self, release: Sample) {
        self.replay.push_back(release);
        if release.velocity != ZERO {
            self.replay.push_back(Sample {
                velocity: ZERO,
                ..release
            });
        }
        self.last_long_release = Some(release.at);
    }

    fn next_deadline(&self) -> Option<Duration> {
        let recognition = match &self.active {
            Some(ActiveGesture::Identifying { start, .. }) => start.checked_add(GESTURE_DELAY),
            _ => None,
        };
        let replay = self
            .replay
            .front()
            .and_then(|sample| sample.at.checked_add(GESTURE_DELAY));
        match (recognition, replay) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(deadline), None) | (None, Some(deadline)) => Some(deadline),
            (None, None) => None,
        }
    }
}

fn target_from(sample: Sample) -> ShortTarget {
    let mut axis = 0;
    for candidate in 1..3 {
        if sample.axes[candidate].abs() > sample.axes[axis].abs() {
            axis = candidate;
        }
    }
    let scale = if axis == 2 {
        15.0_f64.to_radians()
    } else {
        0.1
    };
    ShortTarget {
        axis,
        displacement: sample.velocity[axis] * scale,
    }
}
