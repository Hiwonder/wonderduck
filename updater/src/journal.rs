//! Update log, single-flight lock, and boot counter — the engine's own persistent
//! state.
//!
//! All of it lives in `state_dir`, which must be **outside** every component's
//! `install_dir`: a symlink swap or rollback must not be able to destroy the
//! record of what happened. [`crate::config::Config::validate`]
//! enforces that.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::{Path, PathBuf};

use crate::Error;
use crate::fsutil::write_atomic;
use crate::proto::{CheckAttempt, LogEntry, Outcome};

/// Unix seconds now. The engine stamps log entries; nothing else in the engine
/// depends on the clock except preflight's sanity check.
pub fn now_unix() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0)
}

/// Append-only record of update attempts.
///
/// The first thing support asks for when a client says "the update failed", so it
/// records refused and rolled-back attempts too — not just successes.
///
pub struct Journal {
    path: PathBuf,
    max_entries: usize,
}

impl Journal {
    pub fn open(state_dir: &Path, max_entries: usize) -> Result<Self, Error> {
        std::fs::create_dir_all(state_dir).map_err(|e| Error::Io {
            path: state_dir.to_path_buf(),
            source: e,
        })?;
        Ok(Self {
            path: state_dir.join("update-log.jsonl"),
            max_entries,
        })
    }

    /// Append one entry.
    ///
    /// Durable enough to survive the power loss a failed update can itself
    /// provoke: newline-delimited JSON, then `sync_data`. A torn final line is
    /// tolerated on read rather than treated as corruption — losing the last entry
    /// is acceptable; refusing to read the log because of it is not.
    pub fn append(&self, entry: &LogEntry) -> Result<(), Error> {
        let mut line = serde_json::to_vec(entry)
            .map_err(|e| Error::Internal(format!("serialising log entry: {e}")))?;
        line.push(b'\n');

        {
            let mut file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&self.path)
                .map_err(|e| Error::Io {
                    path: self.path.clone(),
                    source: e,
                })?;
            file.write_all(&line).map_err(|e| Error::Io {
                path: self.path.clone(),
                source: e,
            })?;
            file.sync_data().map_err(|e| Error::Io {
                path: self.path.clone(),
                source: e,
            })?;
        }

        self.trim()
    }

    /// Keep the log bounded. Runs after each append; cheap at these sizes.
    fn trim(&self) -> Result<(), Error> {
        let all = self.read_all()?;
        if all.len() <= self.max_entries {
            return Ok(());
        }

        let keep = &all[all.len() - self.max_entries..];
        let mut buf = Vec::new();
        for entry in keep {
            serde_json::to_writer(&mut buf, entry)
                .map_err(|e| Error::Internal(format!("serialising log entry: {e}")))?;
            buf.push(b'\n');
        }
        // Write-and-rename so a crash mid-trim can't leave a truncated log.
        write_atomic(&self.path, &buf)
    }

    /// Every entry, oldest first. Skips unparseable lines (a torn tail).
    fn read_all(&self) -> Result<Vec<LogEntry>, Error> {
        let text = match std::fs::read_to_string(&self.path) {
            Ok(t) => t,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
            Err(e) => {
                return Err(Error::Io {
                    path: self.path.clone(),
                    source: e,
                });
            }
        };
        Ok(text
            .lines()
            .filter(|l| !l.trim().is_empty())
            .filter_map(|l| serde_json::from_str(l).ok())
            .collect())
    }

    /// Most recent entries, newest first.
    pub fn recent(&self, limit: usize) -> Result<Vec<LogEntry>, Error> {
        let mut all = self.read_all()?;
        all.reverse();
        all.truncate(limit);
        Ok(all)
    }

    pub fn last_for(&self, component: &str) -> Result<Option<LogEntry>, Error> {
        Ok(self
            .read_all()?
            .into_iter()
            .rev()
            .find(|e| e.component.as_str() == component))
    }

    /// Versions whose **most recent** recorded outcome for this component was a
    /// rollback.
    ///
    /// Used to keep `rollback` from landing back on a release that already failed
    /// its gate. Latest-outcome rather than ever-failed, so a version that failed
    /// once and later succeeded is not blacklisted forever.
    pub fn known_bad(&self, component: &str) -> Result<Vec<semver::Version>, Error> {
        let mut seen: Vec<(semver::Version, bool)> = Vec::new();
        // Newest first: the first outcome seen for a version is its latest.
        for entry in self.read_all()?.into_iter().rev() {
            if entry.component.as_str() != component {
                continue;
            }
            let Some(version) = entry.to.clone() else {
                continue;
            };
            if seen.iter().any(|(v, _)| *v == version) {
                continue;
            }
            let bad = matches!(entry.outcome, Outcome::RolledBack { .. });
            seen.push((version, bad));
        }
        Ok(seen
            .into_iter()
            .filter_map(|(v, bad)| bad.then_some(v))
            .collect())
    }
}

/// Single-flight lock, using std's advisory file locking
/// (`File::try_lock`, stable since Rust 1.89 — no dependency needed).
///
/// An OS lock rather than a PID file so the kernel releases it if `updaterd` is
/// killed — a stale PID file would leave the robot permanently unable to update,
/// which is a worse failure than a race.
pub struct UpdateLock {
    /// Dropping the handle releases the lock.
    _file: std::fs::File,
}

impl UpdateLock {
    /// Acquire without blocking.
    ///
    /// `Ok(None)` when already held: "busy" is a normal answer to report to the
    /// app, not an error.
    pub fn try_acquire(state_dir: &Path) -> Result<Option<Self>, Error> {
        use std::fs::TryLockError;

        let path = state_dir.join("update.lock");
        let file = std::fs::OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .truncate(false)
            .open(&path)
            .map_err(|e| Error::Io {
                path: path.clone(),
                source: e,
            })?;

        match file.try_lock() {
            Ok(()) => Ok(Some(Self { _file: file })),
            // Held by another process: report "busy", don't fail.
            Err(TryLockError::WouldBlock) => Ok(None),
            Err(TryLockError::Error(source)) => Err(Error::Io { path, source }),
        }
    }
}

/// Runtime version pins, keyed by component.
///
/// Kept in `state_dir` rather than written back into `updater.toml`: a pin is
/// **device state**, so it belongs outside the shipped config (§5.7), and rewriting
/// a human-edited TOML file in place would lose comments and formatting. The config
/// may still carry a `pinned` value as a deployment-time default; this file
/// overrides it.
pub struct Pins {
    path: PathBuf,
}

impl Pins {
    pub fn open(state_dir: &Path) -> Self {
        Self {
            path: state_dir.join("pins.json"),
        }
    }

    fn read_all(&self) -> Result<BTreeMap<String, semver::Version>, Error> {
        match std::fs::read(&self.path) {
            Ok(bytes) => Ok(serde_json::from_slice(&bytes).unwrap_or_default()),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(BTreeMap::new()),
            Err(e) => Err(Error::Io {
                path: self.path.clone(),
                source: e,
            }),
        }
    }

    pub fn get(&self, component: &str) -> Result<Option<semver::Version>, Error> {
        Ok(self.read_all()?.remove(component))
    }

    /// Set or clear a pin. `None` unpins.
    pub fn set(&self, component: &str, version: Option<&semver::Version>) -> Result<(), Error> {
        let mut all = self.read_all()?;
        match version {
            Some(v) => {
                all.insert(component.to_owned(), v.clone());
            }
            None => {
                all.remove(component);
            }
        }
        if all.is_empty() {
            return match std::fs::remove_file(&self.path) {
                Ok(()) => Ok(()),
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
                Err(e) => Err(Error::Io {
                    path: self.path.clone(),
                    source: e,
                }),
            };
        }
        let bytes = serde_json::to_vec(&all)
            .map_err(|e| Error::Internal(format!("serialising pins: {e}")))?;
        write_atomic(&self.path, &bytes)
    }
}

/// When each component's update source last answered with a manifest that verified, and how the
/// last check went.
///
/// A robot that cannot reach its source (a blocked host, a DNS that stopped resolving, a clock TLS
/// will not accept) looks like a robot with nothing to install: the scheduled check fails, and
/// every other answer about the robot stays the same. How long ago the source last answered is the
/// one thing that differs, so it is written down here and `update.status` reports it
/// A source replaying an old signed manifest
/// still answers, and this does not catch that one; expiry would.
///
/// Only the source's latest counts. An exact version is one a source that stopped moving still
/// serves.
///
/// The last attempt is kept beside it, in its own file, because "never answered" is two states
/// and only it tells them apart: a board whose checks have been failing, which is the loudest case
/// there is, and an `updaterd` that has not run its first check yet, which is every board for the
/// minute after it starts. It also carries the failure, so the robot can say *why* it has not
/// heard from its source without anyone reading the journal.
pub struct Checked {
    path: PathBuf,
    attempts: PathBuf,
}

impl Checked {
    pub fn open(state_dir: &Path) -> Self {
        Self {
            path: state_dir.join("checked.json"),
            attempts: state_dir.join("check-attempts.json"),
        }
    }

    /// When `component`'s source last answered, in unix seconds. `None` when it never has on this
    /// board, or the record cannot be read: this is a report riding on `update.status`, and a
    /// report that failed the call it rides on would be worse than a missing one.
    pub fn get(&self, component: &str) -> Option<i64> {
        read_map(&self.path).unwrap_or_default().remove(component)
    }

    /// `component`'s last check, answered or not. `None` before the first one, and when the
    /// record cannot be read, for the reason [`Self::get`] gives.
    pub fn last_attempt(&self, component: &str) -> Option<CheckAttempt> {
        read_map(&self.attempts)
            .unwrap_or_default()
            .remove(component)
    }

    /// Record that `component`'s source answered now.
    pub fn record(&self, component: &str) -> Result<(), Error> {
        let at = now_unix();
        // The attempt first and whatever the clock says: it is the half that is not gated on it.
        let attempt = self.record_attempt(component, at, None);
        self.record_at(component, at)?;
        attempt
    }

    /// Record that a check of `component`'s source did not get an answer, and why.
    pub fn record_failure(&self, component: &str, error: &str) -> Result<(), Error> {
        self.record_attempt(component, now_unix(), Some(error.to_owned()))
    }

    /// A clock before the preflight floor is not written down. A board with no RTC boots in 1970,
    /// a `local_dir` source needs no TLS to answer, and a time from then would report the source as
    /// fifty years quiet the moment the clock caught up.
    ///
    /// Said out loud, at debug, because the board it happens on is one where every check succeeds
    /// and nothing is ever recorded — and the report staying empty forever is otherwise
    /// indistinguishable from a source that has gone quiet.
    fn record_at(&self, component: &str, at: i64) -> Result<(), Error> {
        if at < crate::preflight::CLOCK_FLOOR_UNIX {
            tracing::debug!(
                component,
                at,
                floor = crate::preflight::CLOCK_FLOOR_UNIX,
                "source answered, but the clock is before the preflight floor: not recorded"
            );
            return Ok(());
        }
        let mut all = read_map(&self.path)?;
        all.insert(component.to_owned(), at);
        write_map(&self.path, &all)
    }

    /// Unlike the answer, an attempt is recorded under any clock. A board whose clock has not
    /// synced is the one whose HTTPS checks fail on the certificate dates, and leaving that
    /// attempt out would report it as not having checked yet — the one reading that says nothing
    /// is wrong. The time is what the clock said; the failure is the part that is read.
    fn record_attempt(&self, component: &str, at: i64, error: Option<String>) -> Result<(), Error> {
        let mut all = read_map(&self.attempts)?;
        all.insert(component.to_owned(), CheckAttempt { at, error });
        write_map(&self.attempts, &all)
    }
}

/// Absent is empty; unreadable is an error, because the caller is about to write the whole map
/// back. Swallowing an IO error here would replace every other component's entry with a map
/// holding only the one being recorded — losing, on a transient failure, exactly the record this
/// exists to keep. Damaged JSON is different: nothing can read any entry in it, so starting over
/// loses nothing that was still there.
fn read_map<T: serde::de::DeserializeOwned>(path: &Path) -> Result<BTreeMap<String, T>, Error> {
    match std::fs::read(path) {
        Ok(bytes) => Ok(serde_json::from_slice(&bytes).unwrap_or_else(|e| {
            tracing::warn!(
                path = %path.display(),
                error = %e,
                "check record unreadable: starting it again"
            );
            BTreeMap::new()
        })),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(BTreeMap::new()),
        Err(e) => Err(Error::Io {
            path: path.to_path_buf(),
            source: e,
        }),
    }
}

fn write_map<T: serde::Serialize>(path: &Path, all: &BTreeMap<String, T>) -> Result<(), Error> {
    let bytes = serde_json::to_vec(all)
        .map_err(|e| Error::Internal(format!("serialising the check record: {e}")))?;
    write_atomic(path, &bytes)
}

/// What `scripts/robot-rescue` leaves in the state dir when it swaps `current` to golden.
///
/// The file it writes, verbatim:
///
/// ```text
/// at=1786453421
/// install_dir=/opt/robot/daemon
/// from=1.2.0
/// to=1.0.0
/// because=boot check: robotd.service (failed, 7 restarts)
/// ```
///
/// `key=value` rather than JSON because the writer is a shell script running on a board where things
/// are already going wrong, and quoting JSON correctly in `sh` is a way to produce a record nothing
/// can read. Which makes this the reader for it — and a lenient one on purpose: **every field is
/// optional**. A breadcrumb that is half-written, or written by an older rescue that did not carry
/// `install_dir`, still has to be recognised, because the alternative is a board whose loop guard
/// never opens (`Engine::record_rescue` clears the file, and only recognises it here).
#[derive(Debug, Default, PartialEq, Eq)]
pub struct Breadcrumb {
    pub at: Option<i64>,
    pub install_dir: Option<String>,
    pub from: Option<String>,
    pub to: Option<String>,
    pub because: Option<String>,
}

/// Name of that file inside the state dir.
pub const RESCUE_BREADCRUMB: &str = "rescued";

impl Breadcrumb {
    pub fn parse(text: &str) -> Self {
        let mut crumb = Self::default();
        for line in text.lines() {
            let Some((key, value)) = line.split_once('=') else {
                continue;
            };
            // Trimmed, because a human may well have looked at this file — and `from=(none)` is
            // what the rescue writes for a board that had no live release, which is not a version.
            let value = value.trim();
            if value.is_empty() || value == "(none)" {
                continue;
            }
            match key.trim() {
                "at" => crumb.at = value.parse().ok(),
                "install_dir" => crumb.install_dir = Some(value.to_owned()),
                "from" => crumb.from = Some(value.to_owned()),
                "to" => crumb.to = Some(value.to_owned()),
                "because" => crumb.because = Some(value.to_owned()),
                _ => {}
            }
        }
        crumb
    }
}

/// Counts boots since an update that has not yet proven itself healthy.
///
/// Catches the failure the in-process health gate cannot: a release that doesn't
/// come up at all, or wedges hard enough to take `updaterd` with it. If a pending
/// update hasn't been confirmed after `max_attempts` boots, revert unconditionally.
///
pub struct BootCounter {
    path: PathBuf,
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct PendingUpdate {
    pub component: String,
    pub version: semver::Version,
    pub previous: Option<semver::Version>,
    #[serde(default)]
    pub boots: u32,
}

impl BootCounter {
    pub fn open(state_dir: &Path) -> Self {
        Self {
            path: state_dir.join("pending.json"),
        }
    }

    /// Record that `version` is on trial for `pending.component`.
    ///
    /// Written **before** the swap, so a crash between swap and health check is
    /// still recoverable — the opposite order would leave an unrecorded bad release
    /// live.
    ///
    /// Trials are **keyed by component**: a model selection must not overwrite or
    /// clear a daemon update's trial, or a daemon release that crashed after its
    /// swap would silently lose the record the never-brick guarantee rests on.
    pub fn arm(&self, pending: &PendingUpdate) -> Result<(), Error> {
        let mut all = self.read_all()?;
        all.insert(pending.component.clone(), pending.clone());
        self.write_all(&all)
    }

    fn read_all(&self) -> Result<BTreeMap<String, PendingUpdate>, Error> {
        match std::fs::read(&self.path) {
            Ok(bytes) => Ok(serde_json::from_slice(&bytes).unwrap_or_default()),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(BTreeMap::new()),
            Err(e) => Err(Error::Io {
                path: self.path.clone(),
                source: e,
            }),
        }
    }

    fn write_all(&self, all: &BTreeMap<String, PendingUpdate>) -> Result<(), Error> {
        if all.is_empty() {
            // No trials outstanding: remove the file rather than leave `{}` behind,
            // so "is anything pending?" is answerable by existence.
            return match std::fs::remove_file(&self.path) {
                Ok(()) => Ok(()),
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
                Err(e) => Err(Error::Io {
                    path: self.path.clone(),
                    source: e,
                }),
            };
        }
        let bytes = serde_json::to_vec(all)
            .map_err(|e| Error::Internal(format!("serialising pending updates: {e}")))?;
        write_atomic(&self.path, &bytes)
    }

    pub fn pending_for(&self, component: &str) -> Result<Option<PendingUpdate>, Error> {
        Ok(self.read_all()?.remove(component))
    }

    /// Every outstanding trial.
    pub fn all(&self) -> Result<Vec<PendingUpdate>, Error> {
        Ok(self.read_all()?.into_values().collect())
    }

    /// Clear one component's trial after a passed health gate.
    pub fn confirm(&self, component: &str) -> Result<(), Error> {
        let mut all = self.read_all()?;
        all.remove(component);
        self.write_all(&all)
    }

    /// Increment every outstanding trial and return them. Called once per boot,
    /// before serving.
    pub fn record_boot(&self) -> Result<Vec<PendingUpdate>, Error> {
        let mut all = self.read_all()?;
        if all.is_empty() {
            return Ok(Vec::new());
        }
        for pending in all.values_mut() {
            pending.boots = pending.boots.saturating_add(1);
        }
        self.write_all(&all)?;
        Ok(all.into_values().collect())
    }

    /// Should we give up and revert?
    pub fn exhausted(pending: &PendingUpdate, max_attempts: u32) -> bool {
        pending.boots >= max_attempts
    }
}
