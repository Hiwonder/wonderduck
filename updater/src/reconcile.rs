//! Did the restarts an update scheduled actually happen?
//!
//! An update restarts the units a release ships, then — five seconds after its reply is on the wire
//! — restarts the two it could not touch while running: itself, and `btd`, which may be carrying
//! the reply. Both are scheduled through `systemd-run`, and **scheduling is all that is checked
//! today.** `systemd-run` succeeding means a transient timer was created, not that the restart ran,
//! and not that the new binary started. Failures are logged and swallowed on purpose, because an
//! update that worked must not report failure over a restart it could not arrange.
//!
//! That leaves one way for a robot to end up running a release it did not install, and it is silent.
//! This closes it: on every `updaterd` start, each unit's running binary is compared with the
//! release its component has active, and anything stale is restarted.
//!
//! ## What it reads
//!
//! Each daemon publishes its own identity at startup — see [`duck_ipc_proto::Identity`] — so this
//! reads a file rather than interrogating a process. A daemon that published nothing is *not* treated
//! as stale: it is either stopped, or too old to publish, and restarting a robot's daemons because
//! they are old is a decision nobody asked for. The next update makes them able to answer.
//!
//! ## Why startup is the right moment
//!
//! It is the first moment the answer can be trusted. `updaterd` cannot watch its own restart land —
//! it is the process being replaced — so the check has to run in the *successor*, which is exactly
//! what a fresh start is. It also catches everything, not merely a missed timer: a restart that
//! failed, a new binary that would not start, a unit someone stopped mid-update, a rollback that
//! left one daemon behind.
//!
//! At boot this is a no-op, since everything starts from the same symlink. It costs one file read
//! per unit.
//!
//! ## Restarting rather than only reporting
//!
//! Reporting alone would leave a robot running the wrong code with a warning in a journal nobody is
//! reading — the situation this exists to end. So a stale unit is restarted.
//!
//! **Except this one.** `updaterd` restarting itself here would be a loop if the new binary ever
//! disagreed about what "current" is, and a loop in the process that owns recovery is the one
//! failure with no way out. It is reported and left alone, which is safe: it has just started, so
//! anything stale about it was decided before this code ran.
//!
//! That exception used to be the end of the story, and it left one skew nothing repaired. It is now
//! reached from the other side: [`stale_units`] is the same reading without the acting, and
//! `Engine::apply` calls it when a release turns out to be already installed — so an operator running
//! `apply` on a robot that looks wrong schedules the restart this module refuses to perform. Once,
//! on a request, which is not the loop guarded against above.

use duck_ipc_proto as proto;

/// What the check found for one unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Finding {
    pub unit: String,
    /// The release the running process was started from. `None` when the unit is stopped, or its
    /// binary sits outside the release layout — a hand-built one on a dev board.
    pub running: Option<semver::Version>,
    pub expected: semver::Version,
    pub verdict: Verdict,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// Running the release its component has active.
    Current,
    /// Stale, and restarted.
    Restarted,
    /// Stale, and deliberately not restarted: `updaterd` must not restart itself from its own
    /// startup path.
    ReportedOnly,
    /// Stale, and the restart failed. The journal has systemd's reason.
    RestartFailed,
    /// Nothing to compare: stopped, or running a binary outside the release layout.
    Unknown,
}

/// Decide what to do about one unit, given what it is running and what it should be.
///
/// Pure, and separated from every syscall in this file for one reason: the decisions are the part
/// that can be wrong, and they are impossible to arrange on a real board — a unit running a release
/// that is not the active one is precisely the state that requires a broken update to produce.
pub fn verdict_for(
    running: Option<&semver::Version>,
    expected: &semver::Version,
    is_self: bool,
) -> Verdict {
    match running {
        // A stopped unit is not a stale unit. Something stopped it deliberately, and starting it
        // here would override that decision on every `updaterd` restart.
        None => Verdict::Unknown,
        Some(running) if running == expected => Verdict::Current,
        Some(_) if is_self => Verdict::ReportedOnly,
        Some(_) => Verdict::Restarted,
    }
}

/// Which of these units are not running `expected`, without touching any of them.
///
/// The observing half of [`check`], for the one caller that must act differently. `Engine::apply`
/// reaching an already-current release is the only way a stale `updaterd` gets looked at — this
/// module will not restart it, deliberately — and it cannot drive `systemctl` the way [`check`] does:
/// restarting `updaterd` or `btd` from inside a running update kills the process doing the restarting
/// or drops the transport carrying the reply. So it reads here and schedules there.
///
/// [`verdict_for`] rather than a second comparison, so what counts as stale stays in one place: a
/// stopped unit does not, and neither does one whose build predates the identity file.
///
/// `is_self` is false and both stale verdicts are accepted, because whether a unit may be restarted
/// is the caller's question and not this one's. Every stale unit is named.
pub fn stale_units(expected: &semver::Version, units: &[String]) -> Vec<String> {
    units
        .iter()
        .filter(|unit| {
            let verdict = verdict_for(running_release(unit).as_ref(), expected, false);
            matches!(verdict, Verdict::Restarted | Verdict::ReportedOnly)
        })
        .cloned()
        .collect()
}

/// The release each unit is running, checked against what its component has active.
///
/// `systemctl` is a parameter for the same reason it is one in `engine`: a test hands it a stub. It
/// is only used to *act* now — the reading is a file read, needing nothing.
pub async fn check(
    systemctl: &str,
    expected: &semver::Version,
    self_unit: &str,
    units: Vec<String>,
) -> Vec<Finding> {
    let mut findings = Vec::new();

    for unit in units {
        let running = running_release(&unit);
        let is_self = unit == self_unit;
        let mut verdict = verdict_for(running.as_ref(), expected, is_self);

        if verdict == Verdict::Restarted {
            tracing::warn!(
                unit = %unit,
                running = %running.as_ref().map(|v| v.to_string()).unwrap_or_default(),
                %expected,
                "still running the release it had before the update; restarting it now"
            );
            if let Err(e) = restart(systemctl, &unit).await {
                tracing::error!(unit = %unit, error = %e, "could not restart it");
                verdict = Verdict::RestartFailed;
            }
        } else if verdict == Verdict::ReportedOnly {
            tracing::warn!(
                unit = %unit,
                running = %running.as_ref().map(|v| v.to_string()).unwrap_or_default(),
                %expected,
                "this process is not running the active release, and will not restart itself"
            );
        }

        findings.push(Finding {
            unit,
            running,
            expected: expected.clone(),
            verdict,
        });
    }

    findings
}

/// The release the unit's process says it is running.
///
/// From the identity the daemon published, so the answer comes from the process rather than from a
/// path someone inferred — and needs no privilege, no D-Bus and no `/proc` read. `None` when nothing
/// was published: stopped, or a build predating the mechanism.
fn running_release(unit: &str) -> Option<semver::Version> {
    let service = unit.strip_suffix(".service").unwrap_or(unit);
    proto::read_identity(service)?.release()
}

async fn restart(systemctl: &str, unit: &str) -> Result<(), String> {
    let mut command = tokio::process::Command::new(systemctl);
    command.arg("restart").arg(unit);
    let status = crate::spawn::retrying_busy(&mut command)
        .await
        .map_err(|e| e.to_string())?
        .wait()
        .await
        .map_err(|e| e.to_string())?;
    if status.success() {
        Ok(())
    } else {
        Err(status.to_string())
    }
}
