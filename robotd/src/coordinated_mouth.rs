//! Timed mouth positions around an autonomous arrival action.

use std::time::Instant;

use duck_control::model::mouth_target;
use duck_ipc_proto::TimedMouthPosition;

#[derive(Clone, Copy)]
struct Ramp {
    settings: TimedMouthPosition,
    start: f64,
    target: f64,
    elapsed: f64,
    pending_elapsed: f64,
    last_written: Option<Instant>,
    written: f64,
    complete: bool,
}

impl Ramp {
    fn new(settings: TimedMouthPosition, measured: f64) -> Self {
        let measured = measured.clamp(duck_control::model::MOUTH_CLOSED, duck_control::model::MOUTH_OPEN);
        Self { settings, start: measured, target: mouth_target(settings.open),
            elapsed: 0.0, pending_elapsed: 0.0, last_written: None, written: measured, complete: false }
    }

    fn next(&mut self, now: Instant) -> f64 {
        if self.complete { return self.target; }
        let delta = self.last_written.map_or(0.0, |last| now.saturating_duration_since(last).as_secs_f64());
        self.pending_elapsed = self.elapsed + delta;
        if self.settings.duration_s == 0.0 || self.pending_elapsed >= self.settings.duration_s {
            self.target
        } else {
            self.start + (self.target - self.start) * (self.pending_elapsed / self.settings.duration_s)
        }
    }

    fn confirm(&mut self, position: f64, now: Instant) {
        self.written = position;
        self.elapsed = self.pending_elapsed;
        self.last_written = Some(now);
        if position == self.target
            && (self.settings.duration_s == 0.0 || self.elapsed >= self.settings.duration_s)
        {
            self.complete = true;
        }
    }

    fn pause(&mut self) { self.last_written = None; }
}

#[derive(Clone, Copy, Default)]
enum Stage {
    #[default]
    Idle,
    Before { settings: TimedMouthPosition, ramp: Option<Ramp> },
    Running { held: Option<f64>, sitting: bool },
    After { ramp: Ramp },
    Holding { position: f64, manual: f64 },
}

#[derive(Default)]
pub struct Sequence {
    stage: Stage,
}

impl Sequence {
    pub fn active(&self) -> bool { !matches!(self.stage, Stage::Idle) }

    pub fn reset(&mut self) { self.stage = Stage::Idle; }

    pub fn prepare(&mut self, settings: TimedMouthPosition, measured: f64) {
        if matches!(self.stage, Stage::Before { settings: previous, .. } if previous == settings) {
            return;
        }
        let start = match self.stage {
            Stage::Before { ramp: Some(ramp), .. } => ramp.written,
            Stage::Holding { position, .. } => position,
            _ => measured,
        };
        self.stage = Stage::Before { settings,
            ramp: settings.enabled.then(|| Ramp::new(settings, start)) };
    }

    pub fn preparing(&self) -> bool {
        matches!(self.stage, Stage::Before { ramp: Some(ramp), .. } if !ramp.complete)
    }

    pub fn before_ready(&self) -> bool {
        matches!(self.stage, Stage::Before { ramp: None, .. })
            || matches!(self.stage, Stage::Before { ramp: Some(ramp), .. } if ramp.complete)
    }

    pub fn cancel_preparation(&mut self) {
        if matches!(self.stage, Stage::Before { .. }) { self.reset(); }
    }

    pub fn action_started(&mut self, sitting: bool) {
        if !self.before_ready() { return; }
        let held = match self.stage {
            Stage::Before { ramp: Some(ramp), .. } => Some(ramp.target),
            _ => None,
        };
        self.stage = Stage::Running { held, sitting };
    }

    pub fn sitting_action_running(&self) -> bool {
        matches!(self.stage, Stage::Running { sitting: true, .. })
    }

    pub fn action_finished(&mut self, settings: TimedMouthPosition, measured: f64) {
        if !matches!(self.stage, Stage::Running { .. }) { return; }
        self.stage = if settings.enabled {
            Stage::After { ramp: Ramp::new(settings, measured) }
        } else {
            Stage::Idle
        };
    }

    pub fn update_after(&mut self, settings: TimedMouthPosition) {
        if let Stage::After { ramp } = self.stage && ramp.settings != settings {
            self.stage = if settings.enabled {
                Stage::After { ramp: Ramp::new(settings, ramp.written) }
            } else {
                Stage::Idle
            };
        }
    }

    pub fn finishing(&self) -> bool { matches!(self.stage, Stage::After { .. }) }

    pub fn command(&mut self, now: Instant, manual: f64) -> Option<f64> {
        match &mut self.stage {
            Stage::Before { ramp: Some(ramp), .. } | Stage::After { ramp } => Some(ramp.next(now)),
            Stage::Running { held, .. } => *held,
            Stage::Holding { position, manual: previous } if manual == *previous => Some(*position),
            Stage::Holding { .. } => { self.reset(); None }
            Stage::Idle | Stage::Before { ramp: None, .. } => None,
        }
    }

    pub fn pause(&mut self) -> Option<f64> {
        match &mut self.stage {
            Stage::Before { ramp: Some(ramp), .. } | Stage::After { ramp } => {
                ramp.pause();
                Some(ramp.written)
            }
            Stage::Running { held, .. } => *held,
            Stage::Holding { position, .. } => Some(*position),
            Stage::Idle | Stage::Before { ramp: None, .. } => None,
        }
    }

    pub fn confirm(&mut self, position: f64, now: Instant, manual: f64) {
        match &mut self.stage {
            Stage::Before { ramp: Some(ramp), .. } => ramp.confirm(position, now),
            Stage::After { ramp } => {
                ramp.confirm(position, now);
                if ramp.complete { self.stage = Stage::Holding { position: ramp.target, manual }; }
            }
            _ => {}
        }
    }
}
