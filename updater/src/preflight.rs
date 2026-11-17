//! Preconditions checked before anything is downloaded or changed.
//!
//! Every failure here aborts cleanly with **no side effects**. See
//!
//!
//! Single-flight is *not* one of these checks: it is enforced by the on-disk lock
//! [`crate::journal::UpdateLock`], taken before any of this runs, and surfaces as
//! [`crate::Error::Busy`]. Listing it here as well would imply a second, redundant
//! mechanism.
//!
//! Run twice per apply: once with no manifest (clock, robot stopped, no live
//! session) *before* any network access, then again for the disk-space check once
//! the manifest's `size` is known. Ordering matters — the manifest fetch is HTTPS,
//! and an unsynced clock breaks it with an opaque TLS error rather than the
//! diagnostic the clock check exists to give.

use std::time::Duration;

use crate::Error;
use crate::robot::{RobotClient, SafeToRestart};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Check {
    /// The clock is plausible.
    ///
    /// A board with no battery-backed RTC boots with a wrong clock, and HTTPS
    /// then fails cert-date validation before any download can start. minisign
    /// itself is time-independent, but TLS is not.
    Clock,
    /// Not mid-motion.
    RobotStopped,
    /// No live telepresence session.
    NoRemoteSession,
    /// Room for download + extract + retained releases.
    DiskSpace,
    /// The `--from` directory is one *this process* can read.
    ///
    /// `updaterd.service` sets `PrivateTmp=yes`, which gives the unit its own `/tmp` **and**
    /// its own `/var/tmp`. A release copied to either from a shell — the obvious place to put
    /// one, and where `scripts/dev-push.sh` used to put it — is therefore not the one this
    /// process sees, and every message downstream is a lie: "no manifest for version X in
    /// /var/tmp/duck-sideload", against a directory whose `ls` shows that exact manifest, its
    /// signature and the artifact. Nothing in that output points at the namespace, and the
    /// caller has done nothing wrong.
    ///
    /// So it is a named check rather than a better error message further down: it fails before
    /// any lookup, it says which mount namespace is responsible, and a board that does not
    /// privatise `/var/tmp` passes it without noticing it exists.
    SideloadDir,
}

#[derive(Debug, Clone)]
pub struct CheckResult {
    pub check: Check,
    pub passed: bool,
    /// Why it failed, safe to display.
    pub detail: Option<String>,
}

#[derive(Debug, Clone)]
pub struct Report {
    pub results: Vec<CheckResult>,
}

impl Report {
    pub fn passed(&self) -> bool {
        self.results.iter().all(|r| r.passed)
    }

    pub fn first_failure(&self) -> Option<&CheckResult> {
        self.results.iter().find(|r| !r.passed)
    }
}

pub struct Preflight<'a> {
    pub robot: &'a dyn RobotClient,
    /// Bytes needed for download + extract, from the manifest, plus headroom.
    pub required_bytes: u64,
    pub available_bytes: u64,
    /// Skip only the remote-session check. Never affects verification.
    pub interrupt_sessions: bool,
    pub robot_query_timeout: Duration,
    /// The directory `apply --from <dir>` was pointed at, if any. `None` for every other
    /// target, which reads its manifest from the configured source instead.
    pub from_dir: Option<&'a std::path::Path>,
}

/// Clock floor: a system time before this cannot be right, and TLS would fail
/// cert-date validation. 2025-01-01T00:00:00Z.
///
/// A board with no battery-backed RTC boots at the epoch (or at its image's build
/// date), so this catches exactly the "never synced NTP yet" case without needing
/// to talk to `timedatectl`.
pub(crate) const CLOCK_FLOOR_UNIX: i64 = 1_735_689_600;

impl Preflight<'_> {
    /// Run every check and report all results.
    ///
    /// Deliberately does **not** short-circuit: telling the user "clock is wrong
    /// AND disk is full" in one round beats making them fix one, retry, and
    /// discover the next.
    pub async fn run(&self) -> Result<Report, Error> {
        let mut results = Vec::new();

        results.push(self.check_clock());
        results.push(self.check_disk());
        results.push(self.check_sideload_dir());
        results.push(self.check_robot_stopped().await);
        results.push(self.check_no_remote_session().await);

        Ok(Report { results })
    }

    fn check_clock(&self) -> CheckResult {
        let now = crate::journal::now_unix();
        let ok = now >= CLOCK_FLOOR_UNIX;
        CheckResult {
            check: Check::Clock,
            passed: ok,
            detail: (!ok).then(|| {
                "system clock is implausibly early (NTP has not synced); HTTPS would fail \
                 certificate date validation"
                    .to_owned()
            }),
        }
    }

    fn check_disk(&self) -> CheckResult {
        let ok = self.available_bytes >= self.required_bytes;
        CheckResult {
            check: Check::DiskSpace,
            passed: ok,
            detail: (!ok).then(|| {
                format!(
                    "needs {} bytes free, only {} available",
                    self.required_bytes, self.available_bytes
                )
            }),
        }
    }

    /// Whether `--from <dir>` names a directory this process can see.
    ///
    /// Two failures, and they need different sentences. A path under `/tmp` or `/var/tmp` is
    /// almost certainly there for the caller and hidden from here by `PrivateTmp=yes`
    /// ([`Check::SideloadDir`]), so the message names the namespace and where to put the
    /// release instead. Anywhere else, a missing directory is a missing directory.
    ///
    /// `exists()` and not a permission check: this runs as root, which reads any path it can
    /// reach, so what remains is reachability.
    fn check_sideload_dir(&self) -> CheckResult {
        let Some(dir) = self.from_dir else {
            return CheckResult {
                check: Check::SideloadDir,
                passed: true,
                detail: None,
            };
        };
        if dir.exists() {
            return CheckResult {
                check: Check::SideloadDir,
                passed: true,
                detail: None,
            };
        }

        let private = dir.starts_with("/tmp") || dir.starts_with("/var/tmp");
        CheckResult {
            check: Check::SideloadDir,
            passed: false,
            detail: Some(if private {
                format!(
                    "{} is not there for updaterd, whichever shell created it: this unit runs \
                     with PrivateTmp=yes, so it has a /tmp and a /var/tmp of its own and neither \
                     is the one you copied into. The release is fine — put it anywhere outside \
                     those two (a home directory, or /var/lib) and install from there.",
                    dir.display()
                )
            } else {
                format!("{} does not exist", dir.display())
            }),
        }
    }

    async fn check_robot_stopped(&self) -> CheckResult {
        let verdict = self.robot.safe_to_restart(self.robot_query_timeout).await;
        // Unreachable counts as safe: if the control loop isn't running, nothing is
        // moving — and that is precisely the case where an update is the fix. An answer
        // that arrived and could not be read does not, because the loop *is* running.
        let passed = verdict.permits_restart();
        CheckResult {
            check: Check::RobotStopped,
            passed,
            detail: match &verdict {
                SafeToRestart::No(reason) => Some(reason.clone()),
                // Names the contract rather than the robot, and names the way out. Without the
                // second half this is a refusal with no stated remedy, on a board where the
                // remedy is not guessable: the robot looks fine, because it is.
                SafeToRestart::Incompatible(detail) => Some(format!(
                    "robotd answered in a shape this updaterd cannot read ({detail}), so whether \
                     it is moving is unknown and a restart is refused. The robot may be perfectly \
                     healthy and the contract is what disagrees — usually a release that added a \
                     field. To update anyway, make it genuinely stopped rather than unreadable: \
                     systemctl stop robotd"
                )),
                _ => None,
            },
        }
    }

    async fn check_no_remote_session(&self) -> CheckResult {
        if self.interrupt_sessions {
            return CheckResult {
                check: Check::NoRemoteSession,
                passed: true,
                detail: Some("session check bypassed by request".into()),
            };
        }

        let active = self
            .robot
            .remote_session_active(self.robot_query_timeout)
            .await;
        CheckResult {
            check: Check::NoRemoteSession,
            passed: !active,
            detail: active.then(|| {
                "a remote/telepresence session is active; restarting would drop it".to_owned()
            }),
        }
    }
}
