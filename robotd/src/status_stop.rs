//! Decide when a maintenance handoff may stop using the walking policy.

use std::time::{Duration, Instant};

pub(crate) const WALK_STAND_SETTLE: Duration = Duration::from_millis(200);

pub(crate) struct StatusMaintenanceStop {
    started: Instant,
    hold_walk: bool,
    zero_since: Option<Instant>,
    stand_since: Option<Instant>,
}

impl StatusMaintenanceStop {
    pub(crate) fn new(now: Instant, walking: bool) -> Self {
        Self { started: now, hold_walk: walking, zero_since: None, stand_since: None }
    }

    pub(crate) fn force_walk(&mut self, now: Instant, command: [f64; 3]) -> bool {
        if !self.hold_walk { return false; }
        if command != [0.0; 3] {
            self.zero_since = None;
            return true;
        }
        let since = *self.zero_since.get_or_insert(now);
        if now.saturating_duration_since(since) < WALK_STAND_SETTLE { return true; }
        self.hold_walk = false;
        false
    }

    /// Once the walking handoff has settled, a maintenance page can disable policy inference.
    pub(crate) fn policy_can_stop(&self, command: [f64; 3]) -> bool {
        !self.hold_walk && command == [0.0; 3]
    }

    pub(crate) fn ready(&mut self, now: Instant, fresh_and_still: bool,
                        transitions_active: bool, posture_stable: bool, command: [f64; 3]) -> bool {
        let settled = fresh_and_still && !transitions_active && posture_stable
            && command == [0.0; 3] && !self.hold_walk;
        if !settled { self.stand_since = None; return false; }
        now.saturating_duration_since(*self.stand_since.get_or_insert(now)) >= WALK_STAND_SETTLE
    }

    pub(crate) fn timed_out(&self, now: Instant) -> bool {
        now.saturating_duration_since(self.started) >= Duration::from_secs(4)
    }
}
