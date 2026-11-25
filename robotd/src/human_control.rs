//! Connection-bound human control leases and their expiration.

use std::collections::HashSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use crate::intents::Intents;
use duck_ipc_proto::{HumanControlResult, HumanControlSource};

const LEASE_DURATION: Duration = Duration::from_millis(750);
const ACTION_SETTLE: Duration = Duration::from_millis(150);
static NEXT_CONNECTION_ID: AtomicU64 = AtomicU64::new(1);

/// One value per accepted connection; never clone or reuse after disconnect.
#[derive(Debug)]
pub(crate) struct Session {
    id: u64,
    source: Option<HumanControlSource>,
    connected: bool,
}

impl Session {
    pub(crate) fn new() -> Self {
        let id = NEXT_CONNECTION_ID
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
            .expect("human control connection IDs exhausted");
        Self {
            id,
            source: None,
            connected: true,
        }
    }

    pub(crate) fn source(&self) -> Option<HumanControlSource> {
        self.source
    }

    pub(crate) fn id(&self) -> u64 {
        self.id
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Access {
    Unregistered,
    NotOwner,
    Owner,
}

#[derive(Debug)]
struct Lease {
    connection_id: u64,
    source: HumanControlSource,
    deadline: Instant,
    action_until: Option<Instant>,
}

#[derive(Debug, Default)]
struct State {
    lease: Option<Lease>,
    last_released: Option<u64>,
    maintenance: Option<Maintenance>,
    emergency_release: bool,
    emergency_release_complete: bool,
    manual_blockers: HashSet<u64>,
    status_blockers: HashSet<u64>,
    maintenance_transition_blockers: HashSet<u64>,
    status_journal: std::path::PathBuf,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MaintenancePhase {
    Requested,
    Active,
    Returning,
    Failed,
}

#[derive(Debug)]
struct Maintenance {
    connection_id: u64,
    phase: MaintenancePhase,
    error: Option<String>,
    status_stop: bool,
    status_reads: bool,
    partial: bool,
    status_pending: Option<(u8, u8, u8)>,
}

pub(crate) struct Arbiter {
    state: Mutex<State>,
    intents: Arc<Intents>,
}

/// Hold this guard across access checks, intent mutations and any required cleanup.
pub(crate) struct Guard<'a> {
    state: MutexGuard<'a, State>,
    now: Instant,
}

impl Arbiter {
    pub(crate) fn new(intents: Arc<Intents>) -> Self {
        Self {
            state: Mutex::new(State::default()),
            intents,
        }
    }

    pub(crate) fn set_status_journal(&self, path: std::path::PathBuf) {
        self.state.lock().expect("human control arbiter poisoned").status_journal = path;
    }

    pub(crate) fn lock(&self) -> Guard<'_> {
        self.lock_with_clock(Instant::now)
    }

    fn lock_with_clock(&self, now: impl FnOnce() -> Instant) -> Guard<'_> {
        let mut state = self.state.lock().expect("human control arbiter poisoned");
        let now = now();
        if let Some(lease) = &state.lease
            && now >= lease.deadline
        {
            let session = lease.connection_id;
            state.last_released = Some(session);
            state.lease = None;
            // Complete revocation before releasing the lock or admitting a new owner.
            self.intents.cancel_motion_tracking();
            self.intents.cancel_maze();
            crate::clear_human_intents(&self.intents);
            tracing::warn!(session, "human control lease expired; motion stopped");
        }
        Guard { state, now }
    }
}

impl Guard<'_> {
    pub(crate) fn has_owner(&self) -> bool {
        self.state.lease.is_some() || self.state.maintenance.is_some() || self.state.emergency_release
    }

    /// Page guards are connection-bound and independent of movement/maintenance leases.
    pub(crate) fn manual_blocked(&self) -> bool {
        !self.state.manual_blockers.is_empty()
    }

    pub(crate) fn block_manual(&mut self, session: &Session) -> HumanControlResult {
        if let Err(reason) = self.registered(session, HumanControlSource::Web) {
            return self.result(false, Some(reason));
        }
        self.state.manual_blockers.insert(session.id);
        self.result(true, None)
    }

    pub(crate) fn unblock_manual(&mut self, session: &Session) -> HumanControlResult {
        if let Err(reason) = self.registered(session, HumanControlSource::Web) {
            return self.result(false, Some(reason));
        }
        self.state.manual_blockers.remove(&session.id);
        self.result(true, None)
    }

    pub(crate) fn status_blocked(&self) -> bool {
        !self.state.status_blockers.is_empty()
    }

    pub(crate) fn block_status(&mut self, session: &Session) -> HumanControlResult {
        if let Err(reason) = self.registered(session, HumanControlSource::Web) {
            return self.result(false, Some(reason));
        }
        self.state.status_blockers.insert(session.id);
        self.result(true, None)
    }

    pub(crate) fn unblock_status(&mut self, session: &Session) -> HumanControlResult {
        if let Err(reason) = self.registered(session, HumanControlSource::Web) {
            return self.result(false, Some(reason));
        }
        self.state.status_blockers.remove(&session.id);
        self.result(true, None)
    }

    pub(crate) fn maintenance_transition_blocked(&self) -> bool {
        !self.state.maintenance_transition_blockers.is_empty()
    }

    pub(crate) fn block_maintenance_transition(&mut self, session: &Session) -> HumanControlResult {
        if let Err(reason) = self.registered(session, HumanControlSource::Web) {
            return self.result(false, Some(reason));
        }
        self.state.maintenance_transition_blockers.insert(session.id);
        self.result(true, None)
    }

    pub(crate) fn unblock_maintenance_transition(&mut self, session: &Session) -> HumanControlResult {
        if let Err(reason) = self.registered(session, HumanControlSource::Web) {
            return self.result(false, Some(reason));
        }
        self.state.maintenance_transition_blockers.remove(&session.id);
        self.result(true, None)
    }

    /// Prevent control and bus handoff until release has been verified.
    /// An unverified attempt keeps this gate closed; a later request may retry.
    pub(crate) fn begin_emergency_release(&mut self) -> Result<(), &'static str> {
        if self.state.maintenance.is_some() {
            return Err("maintenance_active");
        }
        self.emergency_stop();
        self.state.emergency_release = true;
        self.state.emergency_release_complete = false;
        Ok(())
    }

    pub(crate) fn finish_emergency_release(&mut self) {
        self.state.emergency_release = false;
        self.state.emergency_release_complete = false;
    }

    pub(crate) fn mark_emergency_release_unverified(&mut self) {
        if self.state.emergency_release { self.state.emergency_release_complete = true; }
    }

    pub(crate) fn emergency_release_completed(&self) -> bool {
        self.state.emergency_release && self.state.emergency_release_complete
    }

    /// Includes both an in-flight release and a failed, unverified release.
    pub(crate) fn emergency_release_active(&self) -> bool {
        self.state.emergency_release
    }

    pub(crate) fn maintenance_phase(&self) -> Option<MaintenancePhase> {
        self.state.maintenance.as_ref().map(|maintenance| maintenance.phase)
    }

    pub(crate) fn partial_maintenance(&self) -> bool {
        self.state.maintenance.as_ref().is_some_and(|maintenance| maintenance.partial)
    }

    /// Whether this connected session holds the active ordinary maintenance lease.
    pub(crate) fn holds_active_maintenance(&self, session: &Session) -> bool {
        session.connected && self.state.maintenance.as_ref().is_some_and(|maintenance|
            maintenance.connection_id == session.id
                && maintenance.phase == MaintenancePhase::Active
                && !maintenance.status_reads)
    }

    pub(crate) fn status_stop_requested(&self) -> bool {
        self.state.maintenance.as_ref().is_some_and(|maintenance|
            maintenance.status_stop && maintenance.phase == MaintenancePhase::Requested)
    }

    pub(crate) fn status_read_session_requested(&self) -> bool {
        self.state.maintenance.as_ref().is_some_and(|maintenance|
            maintenance.status_reads && maintenance.phase == MaintenancePhase::Requested)
    }

    pub(crate) fn maintenance_result(&self, session: &Session) -> duck_ipc_proto::MaintenanceResult {
        match &self.state.maintenance {
            None => duck_ipc_proto::MaintenanceResult { accepted: true, phase: "idle".into(), reason: None, status_imu_length: None },
            Some(maintenance) => {
                let own = maintenance.connection_id == session.id;
                let phase = match maintenance.phase {
                    MaintenancePhase::Requested => "requested",
                    MaintenancePhase::Active => "active",
                    MaintenancePhase::Returning => "returning",
                    MaintenancePhase::Failed => "failed",
                };
                duck_ipc_proto::MaintenanceResult {
                    accepted: own && maintenance.phase != MaintenancePhase::Failed,
                    phase: phase.into(),
                    reason: if own { maintenance.error.clone() } else { Some("maintenance_busy".into()) },
                    status_imu_length: None,
                }
            }
        }
    }

    pub(crate) fn begin_maintenance(&mut self, session: &Session, status_stop: bool) -> Result<(), &'static str> {
        self.begin_maintenance_inner(session, status_stop, false)
    }

    pub(crate) fn begin_partial_maintenance(&mut self, session: &Session) -> Result<(), &'static str> {
        self.begin_maintenance_inner(session, false, true)
    }

    fn begin_maintenance_inner(&mut self, session: &Session, status_stop: bool, partial: bool) -> Result<(), &'static str> {
        if !session.connected { return Err("disconnected"); }
        if self.state.emergency_release && !(partial && self.state.emergency_release_complete) {
            return Err("emergency_release_in_progress");
        }
        if partial && !self.emergency_release_completed() { return Err("partial_release_not_confirmed"); }
        if self.state.maintenance.is_some() { return Err("maintenance_busy"); }
        self.state.lease = None;
        self.state.last_released = None;
        self.state.maintenance = Some(Maintenance {
            connection_id: session.id,
            phase: MaintenancePhase::Requested,
            error: None,
            // Every exclusive bus handoff first follows the normal stop path.
            status_stop: true,
            status_reads: status_stop,
            partial,
            status_pending: None,
        });
        Ok(())
    }

    pub(crate) fn return_maintenance(&mut self, session: &Session) -> Result<(), &'static str> {
        let maintenance = self.state.maintenance.as_mut().ok_or("no_maintenance")?;
        if maintenance.connection_id != session.id { return Err("not_maintenance_owner"); }
        if maintenance.status_pending.is_some() { return Err("status_read_pending"); }
        maintenance.phase = MaintenancePhase::Returning;
        Ok(())
    }

    pub(crate) fn status_read_pending(&self) -> bool {
        self.state.maintenance.as_ref().is_some_and(|maintenance| maintenance.status_pending.is_some())
    }

    pub(crate) fn finish_status_recovery(&mut self) {
        if let Some(maintenance) = self.state.maintenance.as_mut() {
            maintenance.status_pending = None;
        }
    }

    pub(crate) fn begin_status_read(&mut self, session: &Session, id: u8, address: u8, length: u8) -> Result<(), String> {
        let path = self.state.status_journal.clone();
        let maintenance = self.state.maintenance.as_mut().ok_or("no_maintenance")?;
        if maintenance.connection_id != session.id || maintenance.phase != MaintenancePhase::Active || !maintenance.status_reads {
            return Err("not_status_owner".into());
        }
        if maintenance.status_pending.is_some() { return Err("status_read_unconfirmed".into()); }
        if id >= 254 || length == 0 || u16::from(address) + u16::from(length) > 256 {
            return Err("invalid_status_read".into());
        }
        if let Err(error) = crate::status_bus::begin(&path, id, address, length) {
            if path.symlink_metadata().is_ok() { maintenance.status_pending = Some((id, address, length)); }
            return Err(format!("status_read_journal_failed: {error}"));
        }
        maintenance.status_pending = Some((id, address, length));
        Ok(())
    }

    pub(crate) fn complete_status_read(&mut self, session: &Session, id: u8, address: u8, length: u8) -> Result<(), String> {
        let path = self.state.status_journal.clone();
        let maintenance = self.state.maintenance.as_mut().ok_or("no_maintenance")?;
        if maintenance.connection_id != session.id || maintenance.phase != MaintenancePhase::Active || !maintenance.status_reads {
            return Err("not_status_owner".into());
        }
        if maintenance.status_pending != Some((id, address, length)) { return Err("status_read_mismatch".into()); }
        crate::status_bus::complete(&path)
            .map_err(|error| format!("status_read_journal_failed: {error}"))?;
        maintenance.status_pending = None;
        Ok(())
    }

    pub(crate) fn finish_maintenance_handoff(&mut self, phase: MaintenancePhase, error: Option<String>) {
        if let Some(maintenance) = self.state.maintenance.as_mut() {
            if maintenance.phase == MaintenancePhase::Requested {
                maintenance.phase = phase;
                maintenance.error = error;
            }
        }
    }

    pub(crate) fn finish_maintenance_return(&mut self) {
        if self.maintenance_phase() == Some(MaintenancePhase::Returning) && !self.status_read_pending() {
            self.state.maintenance = None;
        }
    }

    /// Call only after checking ownership and accepting a discrete action under this guard.
    pub(crate) fn mark_action(&mut self) -> bool {
        if let Some(lease) = self.state.lease.as_mut() {
            lease.action_until = Some(self.now + ACTION_SETTLE);
            true
        } else {
            false
        }
    }

    pub(crate) fn action_pending(&self) -> bool {
        self.state.lease.as_ref().is_some_and(|lease| {
            lease
                .action_until
                .is_some_and(|deadline| self.now < deadline)
        })
    }

    fn owner(&self) -> Option<HumanControlSource> {
        self.state.lease.as_ref().map(|lease| lease.source)
    }

    fn result(&self, accepted: bool, reason: Option<&str>) -> HumanControlResult {
        HumanControlResult {
            accepted,
            owner: self.owner(),
            reason: reason.map(str::to_owned),
        }
    }

    fn registered(
        &self,
        session: &Session,
        source: HumanControlSource,
    ) -> Result<(), &'static str> {
        match (session.connected, session.source) {
            (false, _) => Err("disconnected"),
            (true, None) => Err("unregistered"),
            (true, Some(bound)) if bound != source => Err("source_mismatch"),
            (true, Some(_)) => Ok(()),
        }
    }

    pub(crate) fn register(
        &mut self,
        session: &mut Session,
        source: HumanControlSource,
    ) -> HumanControlResult {
        if !session.connected {
            return self.result(false, Some("disconnected"));
        }
        if session.source.is_some() {
            return self.result(false, Some("already_registered"));
        }
        session.source = Some(source);
        self.result(true, None)
    }

    /// Distinguishes ordinary unregistered IPC clients from registered non-owners.
    pub(crate) fn access(&self, session: &Session) -> Access {
        if self.state.maintenance.is_some() || self.state.emergency_release { return Access::NotOwner; }
        if !session.connected || session.source.is_none() {
            return Access::Unregistered;
        }
        if self
            .state
            .lease
            .as_ref()
            .is_some_and(|lease| lease.connection_id == session.id)
        {
            Access::Owner
        } else {
            Access::NotOwner
        }
    }

    pub(crate) fn acquire(
        &mut self,
        session: &Session,
        source: HumanControlSource,
    ) -> HumanControlResult {
        if source == HumanControlSource::Gamepad && self.manual_blocked() {
            return self.result(false, Some("manual_control_disabled"));
        }
        if self.state.emergency_release {
            return self.result(false, Some("emergency_release_in_progress"));
        }
        if self.state.maintenance.is_some() {
            return self.result(false, Some("maintenance_active"));
        }
        if let Err(reason) = self.registered(session, source) {
            return self.result(false, Some(reason));
        }
        if self
            .state
            .lease
            .as_ref()
            .is_some_and(|lease| lease.connection_id != session.id && lease.source == source)
        {
            return self.result(false, Some("busy"));
        }
        if let Some(lease) = self
            .state
            .lease
            .as_mut()
            .filter(|lease| lease.connection_id == session.id)
        {
            lease.deadline = self.now + LEASE_DURATION;
        } else {
            self.state.last_released = None;
            self.state.lease = Some(Lease {
                connection_id: session.id,
                source,
                deadline: self.now + LEASE_DURATION,
                action_until: None,
            });
        }
        self.result(true, None)
    }

    /// Recovery must never replace a control session that arrived while the
    /// original caller was waiting for persistent storage.
    pub(crate) fn acquire_if_free(
        &mut self,
        session: &Session,
        source: HumanControlSource,
    ) -> HumanControlResult {
        if self.state.lease.as_ref().is_some_and(|lease| lease.connection_id != session.id) {
            return self.result(false, Some("busy"));
        }
        self.acquire(session, source)
    }

    pub(crate) fn refresh(
        &mut self,
        session: &Session,
        source: HumanControlSource,
    ) -> HumanControlResult {
        if let Err(reason) = self.registered(session, source) {
            return self.result(false, Some(reason));
        }
        if self.access(session) != Access::Owner {
            return self.result(false, Some("not_owner"));
        }
        self.state.lease.as_mut().expect("owner has lease").deadline = self.now + LEASE_DURATION;
        self.result(true, None)
    }

    /// The caller determines whether controls or an action are still active under this guard.
    pub(crate) fn release(
        &mut self,
        session: &Session,
        source: HumanControlSource,
        busy: bool,
    ) -> HumanControlResult {
        if let Err(reason) = self.registered(session, source) {
            return self.result(false, Some(reason));
        }
        if self.access(session) != Access::Owner {
            return self.result(false, Some("not_owner"));
        }
        if busy || self.action_pending() {
            self.state.lease.as_mut().expect("owner has lease").deadline =
                self.now + LEASE_DURATION;
            return self.result(false, Some("busy"));
        }
        self.state.lease = None;
        self.state.last_released = Some(session.id);
        self.result(true, None)
    }

    /// A lost input device relinquishes only its own lease, even while motion is active.
    pub(crate) fn abandon(
        &mut self,
        session: &Session,
        source: HumanControlSource,
    ) -> HumanControlResult {
        if let Err(reason) = self.registered(session, source) {
            return self.result(false, Some(reason));
        }
        if self.access(session) != Access::Owner {
            if self.state.maintenance.is_none() && self.state.lease.is_none()
                && self.state.last_released == Some(session.id)
            {
                return self.result(true, None);
            }
            return self.result(false, Some("not_owner"));
        }
        self.state.lease = None;
        self.state.last_released = Some(session.id);
        self.result(true, None)
    }

    /// Status is available to unregistered monitoring connections.
    pub(crate) fn status(
        &self,
        session: &Session,
        source: HumanControlSource,
    ) -> HumanControlResult {
        if !session.connected {
            return self.result(false, Some("disconnected"));
        }
        if session.source.is_some_and(|bound| bound != source) {
            return self.result(false, Some("source_mismatch"));
        }
        self.result(true, None)
    }

    /// True means the caller must stop and clear intents before releasing this guard.
    pub(crate) fn disconnect(&mut self, session: &mut Session) -> bool {
        self.state.manual_blockers.remove(&session.id);
        self.state.status_blockers.remove(&session.id);
        self.state.maintenance_transition_blockers.remove(&session.id);
        let was_owner = self.access(session) == Access::Owner;
        if let Some(maintenance) = self.state.maintenance.as_mut()
            && maintenance.connection_id == session.id
        {
            maintenance.phase = MaintenancePhase::Returning;
        }
        session.connected = false;
        if self.state.last_released == Some(session.id) { self.state.last_released = None; }
        if was_owner {
            self.state.lease = None;
        }
        was_owner
    }

    /// Any connection may request an emergency stop; returns the revoked owner, if any.
    pub(crate) fn emergency_stop(&mut self) -> Option<HumanControlSource> {
        self.state.last_released = None;
        self.state.lease.take().map(|lease| lease.source)
    }

    /// Revoke only a gamepad lease, leaving a later web takeover untouched.
    pub(crate) fn revoke_gamepad(&mut self) -> bool {
        if self.state.lease.as_ref().is_some_and(|lease| lease.source == HumanControlSource::Gamepad) {
            self.state.lease = None;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::intents::{PoseIntent, PowerRequest};
    use duck_ipc_proto::{HeadParams, MotionTrackingAction, MotionTrackingParams, RestPhase};

    #[test]
    fn read_only_status_revokes_expired_control_and_clears_intents_once() {
        let intents = Arc::new(Intents::new());
        let arbiter = Arbiter::new(Arc::clone(&intents));
        let mut operator = Session::new();
        let reader = Session::new();
        let at = Instant::now();
        let source = HumanControlSource::Web;
        {
            let mut guard = arbiter.lock_with_clock(|| at);
            assert!(guard.register(&mut operator, source).accepted);
            assert!(guard.acquire(&operator, source).accepted);
            intents.set_twist([0.15, 0.0, 0.0]);
            intents.set_head(HeadParams {
                raw_position: Some([2000, 2300]),
                ..HeadParams::default()
            });
            intents.set_pose(PoseIntent { body: [0.01, 0.0, 0.0], active: true });
            assert!(intents.update_motion_tracking(operator.id(), MotionTrackingParams {
                action: MotionTrackingAction::Start,
                initial_pitch: None,
                target: None,
                line_speed: Some(0.1),
                line_target_x: None, reference_area: 1.0,
                deadzone: [0.0; 2],
                gains: [[0.0; 3]; 2],
            }));
            intents.request_ground_pick();
            assert!(intents.request_rest());
            intents.request_relax();
            intents.request_confirmed_shutdown();
        }

        let before = intents.snapshot();
        let deadline = at + LEASE_DURATION;
        assert_eq!(
            arbiter.lock_with_clock(|| deadline - Duration::from_millis(1)).status(&reader, source).owner,
            Some(source),
        );
        assert!(before.motion_tracking.is_some());

        assert_eq!(arbiter.lock_with_clock(|| deadline).status(&reader, source).owner, None);
        let after = intents.snapshot();
        assert_eq!(after.command.twist, [0.0; 3]);
        assert_eq!(after.head_position_raw, None);
        assert!(!after.pose.active);
        assert!(after.motion_tracking.is_none());
        assert!(!intents.take_skills().ground_pick);
        assert_eq!(intents.rest_phase(), RestPhase::Failed);
        assert!(matches!(intents.take_power_request(), Some(PowerRequest::Relax)));
        assert!(intents.take_shutdown());

        drop(arbiter.lock_with_clock(|| deadline));
        assert_eq!(after.stop_generation, before.stop_generation + 1);
        assert_eq!(intents.snapshot().stop_generation, after.stop_generation);
    }

    #[test]
    fn refreshed_deadline_and_late_disconnect_preserve_the_current_owner() {
        let intents = Arc::new(Intents::new());
        let arbiter = Arbiter::new(Arc::clone(&intents));
        let mut previous = Session::new();
        let mut next = Session::new();
        let source = HumanControlSource::Web;
        let at = Instant::now();
        let refreshed_at = at + Duration::from_millis(500);
        {
            let mut guard = arbiter.lock_with_clock(|| at);
            assert!(guard.register(&mut previous, source).accepted);
            assert!(guard.acquire(&previous, source).accepted);
            assert!(guard.register(&mut next, source).accepted);
            intents.set_twist([0.15, 0.0, 0.0]);
        }
        assert!(arbiter.lock_with_clock(|| refreshed_at).refresh(&previous, source).accepted);
        {
            let mut guard = arbiter.lock_with_clock(|| at + LEASE_DURATION);
            assert_eq!(guard.access(&previous), Access::Owner);
            assert!(!guard.acquire_if_free(&next, source).accepted);
            assert_eq!(intents.snapshot().command.twist, [0.15, 0.0, 0.0]);
        }

        let deadline = refreshed_at + LEASE_DURATION;
        {
            let mut guard = arbiter.lock_with_clock(|| deadline);
            assert!(guard.acquire_if_free(&next, source).accepted);
            assert_eq!(intents.snapshot().command.twist, [0.0; 3]);
            intents.set_twist([-0.1, 0.0, 0.0]);
        }
        let mut guard = arbiter.lock_with_clock(|| deadline);
        assert!(!guard.disconnect(&mut previous));
        assert_eq!(guard.access(&next), Access::Owner);
        assert_eq!(intents.snapshot().command.twist, [-0.1, 0.0, 0.0]);
    }

    #[test]
    fn maintenance_at_the_deadline_clears_queued_actions_before_handoff() {
        let intents = Arc::new(Intents::new());
        let arbiter = Arbiter::new(Arc::clone(&intents));
        let mut operator = Session::new();
        let maintenance = Session::new();
        let source = HumanControlSource::Web;
        let at = Instant::now();
        {
            let mut guard = arbiter.lock_with_clock(|| at);
            assert!(guard.register(&mut operator, source).accepted);
            assert!(guard.acquire(&operator, source).accepted);
            assert!(guard.mark_action());
            intents.request_sit_toggle();
        }

        let mut guard = arbiter.lock_with_clock(|| at + LEASE_DURATION);
        guard.begin_maintenance(&maintenance, false).unwrap();
        assert_eq!(guard.maintenance_phase(), Some(MaintenancePhase::Requested));
        assert!(!guard.acquire_if_free(&operator, source).accepted);
        assert!(!intents.take_skills().sit_toggle);
    }
}
