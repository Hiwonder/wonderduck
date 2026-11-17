//! Give only the physical control thread a larger normal-scheduler weight.

use std::io;

/// New threads and subprocesses reset negative nice values to zero. This keeps
/// inference loading, audio and state publication from inheriting control weight.
pub fn configure(nice: i32) -> io::Result<()> {
    let parameter = libc::sched_param { sched_priority: 0 };
    // SAFETY: pid 0 denotes the calling thread; the parameter lives through the call.
    let result = unsafe {
        libc::sched_setscheduler(0, libc::SCHED_OTHER | libc::SCHED_RESET_ON_FORK, &parameter)
    };
    if result != 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: the kernel validates the requested nice value and permissions.
    if unsafe { libc::setpriority(libc::PRIO_PROCESS, 0, nice) } != 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: these calls only inspect the calling thread's scheduler state.
    let (policy, actual) = unsafe {
        (
            libc::sched_getscheduler(0),
            libc::getpriority(libc::PRIO_PROCESS, 0),
        )
    };
    if policy != libc::SCHED_OTHER | libc::SCHED_RESET_ON_FORK || actual != nice {
        return Err(io::Error::other(format!(
            "control scheduler verification failed: policy={policy}, nice={actual}, requested={nice}"
        )));
    }
    tracing::info!(
        nice,
        reset_children = true,
        "control thread normal scheduling configured"
    );
    Ok(())
}
