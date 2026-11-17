//! Connection-bound human control leases. Robot intent changes belong to the caller.

use std::collections::HashSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

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
    manual_blockers: HashSet<u64>,
    status_blockers: HashSet<u64>,
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
    status_pending: Option<(u8, u8, u8)>,
}

#[derive(Debug, Default)]
pub(crate) struct Arbiter {
    state: Mutex<State>,
}

/// Hold this guard across access checks, intent mutations and any required cleanup.
pub(crate) struct Guard<'a> {
    state: MutexGuard<'a, State>,
    now: Instant,
    expired: bool,
}

impl Arbiter {
    pub(crate) fn set_status_journal(&self, path: std::path::PathBuf) {
        self.state.lock().expect("human control arbiter poisoned").status_journal = path;
    }
    pub(crate) fn lock(&self) -> Guard<'_> {
        let state = self.state.lock().expect("human control arbiter poisoned");
        Guard::new(state, Instant::now())
    }


}

impl<'a> Guard<'a> {
    fn new(mut state: MutexGuard<'a, State>, now: Instant) -> Self {
        let expired = state
            .lease
            .as_ref()
            .is_some_and(|lease| now >= lease.deadline);
        if expired {
            state.last_released = state.lease.as_ref().map(|lease| lease.connection_id);
            state.lease = None;
        }
        Self {
            state,
            now,
            expired,
        }
    }

    /// If true, stop and clear the previous owner's intents before using this guard.
    pub(crate) fn expired(&self) -> bool {
        self.expired
    }

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

    /// Prevent control and bus handoff while a verified release is in flight.
    pub(crate) fn begin_emergency_release(&mut self) -> Result<(), &'static str> {
        if self.state.maintenance.is_some() {
            return Err("maintenance_active");
        }
        self.emergency_stop();
        self.state.emergency_release = true;
        Ok(())
    }

    pub(crate) fn finish_emergency_release(&mut self) {
        self.state.emergency_release = false;
    }

    pub(crate) fn emergency_release_active(&self) -> bool {
        self.state.emergency_release
    }

    pub(crate) fn maintenance_phase(&self) -> Option<MaintenancePhase> {
        self.state.maintenance.as_ref().map(|maintenance| maintenance.phase)
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
        if !session.connected { return Err("disconnected"); }
        if self.state.emergency_release { return Err("emergency_release_in_progress"); }
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
