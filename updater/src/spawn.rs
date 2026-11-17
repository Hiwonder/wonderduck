//! Spawning a child that the kernel may refuse to `exec` yet.
//!
//! One function, and the reason it exists is worth more than the code.

use std::time::Duration;

/// How many times to retry a spawn that fails with `ETXTBSY`, and how long to wait between.
///
/// Ten attempts over ~100 ms. The window this closes is the few microseconds between a
/// `fork` and the child's `execve`, so the first retry almost always succeeds; the budget
/// exists so a pathological case gives up rather than hanging an update.
const BUSY_RETRIES: u32 = 10;
const BUSY_BACKOFF: Duration = Duration::from_millis(10);

/// Spawn, retrying `ETXTBSY` — which is not a real failure here.
///
/// The kernel refuses to `exec` a file that *any* process has open for writing, tracked on the
/// inode. We extract a release — writing `hooks/preinstall` — and then exec it moments later, while
/// also spawning other children around the same time: the other hook, `systemctl` for `on_apply`, a
/// command-style health probe. A child inherits a duplicate of the whole fd table at fork and only
/// drops `O_CLOEXEC` descriptors at its own `execve`, so for a few microseconds some unrelated child
/// holds a write handle to the file we are about to run, and the exec fails with "Text file busy".
///
/// Left unhandled that is a failed hook, which fails the update, which rolls back a release that was
/// fine — rarely, unreproducibly, and reported on the robot as "failed at RunningPreHook" with
/// nothing anyone can act on. It first showed up as an intermittently red CI job, which is the same
/// bug wearing a costume.
///
/// Retrying is the remedy because nothing else addresses it: the offending descriptor belongs to a
/// *different* process, so closing or syncing ours changes nothing, and `rename` keeps the same
/// inode.
///
/// **Every spawn in this crate goes through here, including ones whose program we never write.**
/// `systemctl` is not a file the updater rewrites, so on a board it cannot be busy for this reason —
/// but the test suite writes stub `systemctl` scripts and executes them from parallel threads of one
/// process, which is the same race with the same cause, and it failed a pull request that had touched
/// nothing but documentation. A spawn helper that some callers skip is a helper that will be skipped
/// by the next caller too.
pub(crate) async fn retrying_busy(
    command: &mut tokio::process::Command,
) -> std::io::Result<tokio::process::Child> {
    for attempt in 1..=BUSY_RETRIES {
        match command.spawn() {
            Ok(child) => {
                if attempt > 1 {
                    tracing::debug!(attempt, "spawned after ETXTBSY");
                }
                return Ok(child);
            }
            // `raw_os_error`, not `kind`: ETXTBSY has no stable `ErrorKind` — it maps to
            // `Uncategorized`, which is unstable to match on.
            Err(e) if e.raw_os_error() == Some(libc::ETXTBSY) && attempt < BUSY_RETRIES => {
                tracing::debug!(attempt, "busy for exec; retrying");
                tokio::time::sleep(BUSY_BACKOFF).await;
            }
            Err(e) => return Err(e),
        }
    }
    unreachable!("the loop returns on the final attempt")
}
