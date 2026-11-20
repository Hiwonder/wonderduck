//! Durable lock after a low-voltage trip. No marker is created during normal operation.
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::{Duration, Instant};
#[cfg(unix)]
use std::os::unix::fs::OpenOptionsExt;

pub(crate) const PHYSICAL_MARKER: &str = "/var/lib/robotd-low-voltage.lock";
pub(crate) const RECOVER_VOLTAGE: f64 = 7.2;
const RECOVER_DURATION: Duration = Duration::from_secs(2);
const MAX_SAMPLE_GAP: Duration = Duration::from_millis(250);

struct State {
    locked: bool,
    recovery_since: Option<Instant>,
    last_normal: Option<Instant>,
    requires_rearm: bool,
    persistence_confirmed: bool,
    awaiting_start_on_disk: bool,
}
pub(crate) struct LowVoltageLock {
    path: Option<PathBuf>,
    state: Mutex<State>,
}
impl LowVoltageLock {
    pub(crate) fn open(path: Option<PathBuf>) -> io::Result<Self> {
        let (active, recovered) = match path.as_deref() {
            Some(path) => (marker_exists(path)?, marker_exists(&awaiting_start_path(path))?),
            None => (false, false),
        };
        let existing = active || recovered;
        Ok(Self { path, state: Mutex::new(State { locked: existing, requires_rearm: existing,
            recovery_since: None, last_normal: None, persistence_confirmed: existing,
            awaiting_start_on_disk: !active && recovered }) })
    }
    /// Persist the lock before starting the sit-and-release sequence. If this
    /// fails, admission stays locked and the caller must not report completion.
    pub(crate) fn trip(&self) -> io::Result<()> {
        let mut state = self.state.lock().expect("low-voltage lock poisoned");
        state.locked = true;
        state.requires_rearm = true;
        state.recovery_since = None;
        state.last_normal = None;
        if state.awaiting_start_on_disk {
            state.persistence_confirmed = false;
            if let Some(path) = self.path.as_deref() {
                fs::rename(awaiting_start_path(path), path)?;
            }
            state.awaiting_start_on_disk = false;
        }
        if state.persistence_confirmed { return Ok(()); }
        if let Some(path) = self.path.as_ref() {
            let mut options = OpenOptions::new();
            options.write(true).create(true).truncate(true);
            #[cfg(unix)]
            options.mode(0o600);
            #[cfg(target_os = "linux")]
            options.custom_flags(libc::O_NOFOLLOW);
            let mut file = options.open(path)?;
            if !file.metadata()?.is_file() { return Err(io::Error::other("invalid low-voltage marker")); }
            file.write_all(b"low\n")?;
            file.sync_all()?;
            sync_parent(path)?;
        }
        state.persistence_confirmed = true;
        Ok(())
    }
    pub(crate) fn locked(&self) -> bool { self.state.lock().expect("low-voltage lock poisoned").locked }
    pub(crate) fn requires_rearm(&self) -> bool { self.state.lock().expect("low-voltage lock poisoned").requires_rearm }
    pub(crate) fn persistence_confirmed(&self) -> bool { self.state.lock().expect("low-voltage lock poisoned").persistence_confirmed }
    /// A recovery requires a continuous stream of complete, fresh minimum
    /// voltages. Unknown, stale, out-of-order or low data resets the timer.
    pub(crate) fn observe_recovery(&self, voltage: Option<f64>, now: Instant) -> bool {
        let mut state = self.state.lock().expect("low-voltage lock poisoned");
        if !state.requires_rearm { return false; }
        if !voltage.is_some_and(|value| value.is_finite() && value >= RECOVER_VOLTAGE) {
            state.locked = true; state.recovery_since = None; state.last_normal = None; return false;
        }
        if state.last_normal.is_some_and(|last| now < last || now.duration_since(last) > MAX_SAMPLE_GAP) {
            state.recovery_since = None;
            state.locked = true;
        }
        state.last_normal = Some(now);
        let since = *state.recovery_since.get_or_insert(now);
        if now.duration_since(since) < RECOVER_DURATION { return false; }
        let changed = state.locked;
        state.locked = false;
        changed
    }
    /// Persist recovery separately so a reboot before Start still requires a fresh request.
    pub(crate) fn mark_recovered(&self) -> io::Result<()> {
        let mut state = self.state.lock().expect("low voltage lock poisoned");
        if state.locked || !state.requires_rearm {
            return Err(io::Error::other("voltage has not recovered"));
        }
        if let Some(path) = self.path.as_deref() {
            if !state.awaiting_start_on_disk {
                fs::rename(path, awaiting_start_path(path))?;
                state.awaiting_start_on_disk = true;
            }
            state.persistence_confirmed = false;
            sync_parent(path)?;
            state.persistence_confirmed = true;
        } else {
            state.awaiting_start_on_disk = true;
        }
        Ok(())
    }

    /// Start consumes the recovery record and clears stale commands and control leases.
    pub(crate) fn rearm(&self, now: Instant) -> io::Result<bool> {
        let mut state = self.state.lock().expect("low-voltage lock poisoned");
        if !state.requires_rearm { return Ok(true); }
        if state.locked || !state.persistence_confirmed || !state.awaiting_start_on_disk || state.last_normal.is_none_or(|last|
            now < last || now.duration_since(last) > MAX_SAMPLE_GAP) { return Ok(false); }
        if let Some(path) = self.path.as_deref().map(awaiting_start_path) {
            fs::remove_file(&path)?;
            if let Err(error) = sync_parent(&path) {
                // A directory-sync failure means the removal's durability is
                // unknown. Restore the marker best-effort, and stay locked.
                let _ = OpenOptions::new().write(true).create_new(true).open(&path)
                    .and_then(|mut file| { file.write_all(b"recovered\n")?; file.sync_all() })
                    .and_then(|()| sync_parent(&path));
                state.locked = true;
                return Err(error);
            }
        }
        state.requires_rearm = false;
        state.persistence_confirmed = false;
        state.awaiting_start_on_disk = false;
        Ok(true)
    }
}
fn marker_exists(path: &Path) -> io::Result<bool> {
    match fs::symlink_metadata(path) {
        Ok(metadata) if metadata.is_file() => Ok(true),
        Ok(_) => Err(io::Error::other("invalid low-voltage marker")),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(false),
        Err(error) => Err(error),
    }
}

fn awaiting_start_path(path: &Path) -> PathBuf {
    path.with_extension("start-required")
}

fn sync_parent(path: &Path) -> io::Result<()> {
    fs::File::open(path.parent().ok_or_else(|| io::Error::other("missing marker directory"))?)?.sync_all()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn recover(lock: &LowVoltageLock, start: Instant) -> Instant {
        for step in 0..=20 {
            lock.observe_recovery(Some(7.6), start + Duration::from_millis(step * 100));
        }
        start + RECOVER_DURATION
    }

    #[test]
    fn stable_voltage_replaces_low_marker_and_waits_for_start_across_reboot() {
        let directory = std::env::temp_dir().join(format!("robotd-voltage-recovery-{}", std::process::id()));
        fs::create_dir(&directory).unwrap();
        let path = directory.join("low-voltage.lock");
        let waiting = awaiting_start_path(&path);
        let lock = LowVoltageLock::open(Some(path.clone())).unwrap();
        lock.trip().unwrap();
        assert!(path.exists());

        let now = recover(&lock, Instant::now());
        assert!(!lock.rearm(now).unwrap());
        lock.mark_recovered().unwrap();
        assert!(!path.exists());
        assert!(waiting.exists());
        assert!(lock.requires_rearm());

        let reopened = LowVoltageLock::open(Some(path)).unwrap();
        assert!(reopened.locked());
        assert!(!reopened.rearm(now).unwrap());
        let now = recover(&reopened, now);
        reopened.mark_recovered().unwrap();
        reopened.trip().unwrap();
        assert!(!waiting.exists());
        assert!(!reopened.rearm(now).unwrap());

        let now = recover(&reopened, now);
        reopened.mark_recovered().unwrap();
        assert!(reopened.rearm(now).unwrap());
        assert!(!waiting.exists());
        assert!(!reopened.requires_rearm());
        fs::remove_dir(directory).unwrap();
    }
}
