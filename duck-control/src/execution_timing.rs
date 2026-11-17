//! Rare slow-operation evidence without changing the control or transport result.
//!
//! Wall time alone cannot distinguish computation from descheduling or blocking.
//! Capture thread resource counters at phase boundaries and emit only on a slow
//! operation, including early error returns. No files are opened on this path.

use duck_ipc_proto::clock;

const SLOW_OPERATION_NS: u64 = 2_000_000;

#[derive(Clone, Copy, Debug)]
struct Usage {
    user_us: u64,
    system_us: u64,
    voluntary: u64,
    involuntary: u64,
    minor_faults: u64,
    major_faults: u64,
}

#[derive(Clone, Copy)]
struct Stamp {
    wall_ns: u64,
    usage: Result<Option<Usage>, i32>,
}

impl Stamp {
    fn now() -> Self {
        Self {
            wall_ns: clock::monotonic_ns(),
            usage: thread_usage(),
        }
    }
}

#[cfg(target_os = "linux")]
fn thread_usage() -> Result<Option<Usage>, i32> {
    let mut raw = std::mem::MaybeUninit::<libc::rusage>::uninit();
    // RUSAGE_THREAD reports only the calling control thread, never ONNX workers
    // or the process-wide logging threads. The pointer is valid for this call.
    if unsafe { libc::getrusage(libc::RUSAGE_THREAD, raw.as_mut_ptr()) } != 0 {
        return Err(std::io::Error::last_os_error()
            .raw_os_error()
            .unwrap_or(libc::EIO));
    }
    let raw = unsafe { raw.assume_init() };
    let micros = |t: libc::timeval| t.tv_sec as u64 * 1_000_000 + t.tv_usec as u64;
    Ok(Some(Usage {
        user_us: micros(raw.ru_utime),
        system_us: micros(raw.ru_stime),
        voluntary: raw.ru_nvcsw as u64,
        involuntary: raw.ru_nivcsw as u64,
        minor_faults: raw.ru_minflt as u64,
        major_faults: raw.ru_majflt as u64,
    }))
}

#[cfg(not(target_os = "linux"))]
fn thread_usage() -> Result<Option<Usage>, i32> {
    Ok(None)
}

#[derive(Clone, Copy, Debug)]
struct Measurement {
    elapsed_ns: u64,
    user_us: Option<u64>,
    system_us: Option<u64>,
    voluntary: Option<u64>,
    involuntary: Option<u64>,
    minor_faults: Option<u64>,
    major_faults: Option<u64>,
    usage_error: Option<i32>,
}

impl Measurement {
    fn between(start: Stamp, end: Stamp) -> Self {
        let mut result = Self {
            elapsed_ns: end.wall_ns.saturating_sub(start.wall_ns),
            user_us: None,
            system_us: None,
            voluntary: None,
            involuntary: None,
            minor_faults: None,
            major_faults: None,
            usage_error: None,
        };
        match (start.usage, end.usage) {
            (Ok(Some(a)), Ok(Some(b))) => {
                result.user_us = b.user_us.checked_sub(a.user_us);
                result.system_us = b.system_us.checked_sub(a.system_us);
                result.voluntary = b.voluntary.checked_sub(a.voluntary);
                result.involuntary = b.involuntary.checked_sub(a.involuntary);
                result.minor_faults = b.minor_faults.checked_sub(a.minor_faults);
                result.major_faults = b.major_faults.checked_sub(a.major_faults);
            }
            (Err(error), _) | (_, Err(error)) => result.usage_error = Some(error),
            _ => {}
        }
        result
    }

    fn report(&self, operation: &'static str, phase: &'static str, start_ns: u64, end_ns: u64) {
        let cpu_us = self.user_us.zip(self.system_us).map(|(u, s)| u + s);
        tracing::warn!(target: "duck_control::execution_timing",
            operation, phase, start_t_ns = start_ns, end_t_ns = end_ns,
            elapsed_ms = self.elapsed_ns as f64 / 1e6,
            cpu_us = ?cpu_us, user_us = ?self.user_us, system_us = ?self.system_us,
            voluntary_switches = ?self.voluntary, involuntary_switches = ?self.involuntary,
            minor_faults = ?self.minor_faults, major_faults = ?self.major_faults,
            resource_error = ?self.usage_error,
            "slow control operation phase");
    }
}

#[derive(Clone, Copy)]
struct Phase {
    name: &'static str,
    start: Stamp,
    end: Stamp,
}

/// Stack-only phase storage; drop preserves evidence even if an IO call fails.
pub struct StageTimer<const N: usize> {
    operation: &'static str,
    phase: &'static str,
    start: Stamp,
    previous: Stamp,
    phases: [Option<Phase>; N],
    count: usize,
}

impl<const N: usize> StageTimer<N> {
    pub fn new(operation: &'static str, phase: &'static str) -> Self {
        let start = Stamp::now();
        Self {
            operation,
            phase,
            start,
            previous: start,
            phases: [None; N],
            count: 0,
        }
    }

    pub fn advance(&mut self, next_phase: &'static str) {
        self.close_phase(Stamp::now());
        self.phase = next_phase;
    }

    fn close_phase(&mut self, end: Stamp) {
        assert!(self.count < N, "control timing phase capacity exceeded");
        self.phases[self.count] = Some(Phase {
            name: self.phase,
            start: self.previous,
            end,
        });
        self.count += 1;
        self.previous = end;
    }
}

impl<const N: usize> Drop for StageTimer<N> {
    fn drop(&mut self) {
        let end = Stamp::now();
        self.close_phase(end);
        let total = Measurement::between(self.start, end);
        if total.elapsed_ns < SLOW_OPERATION_NS && total.usage_error.is_none() {
            return;
        }
        total.report(self.operation, "total", self.start.wall_ns, end.wall_ns);
        for phase in self.phases[..self.count].iter().flatten() {
            Measurement::between(phase.start, phase.end).report(
                self.operation,
                phase.name,
                phase.start.wall_ns,
                phase.end.wall_ns,
            );
        }
    }
}
