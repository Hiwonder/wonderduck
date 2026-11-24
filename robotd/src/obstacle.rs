//! Single-zone range input and robot-wide walking assistance.

use std::io;
use std::path::Path;
use std::process::Stdio;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use arc_swap::{ArcSwap, ArcSwapOption};
use duck_ipc_proto as proto;
use serde::Deserialize;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::task::JoinHandle;

const MIN_DISTANCE_MM: u32 = 20;
const MAX_DISTANCE_MM: u32 = 2500;
const READING_MAX_AGE_NS: u64 = 800_000_000;

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
struct Reading {
    distance_mm: Option<u32>,
    sensor_status: proto::DistanceSensorStatus,
    sampled_monotonic_ns: u64,
    #[serde(default)]
    raw_distance_mm: Option<u32>,
}

#[derive(Debug, Clone, Copy)]
pub struct SensorConfig {
    pub hz: u32,
    pub timing_budget_ms: Option<u32>,
    pub signal_rate_limit: Option<f64>,
    pub record_raw: bool,
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self { hz: 20, timing_budget_ms: None, signal_rate_limit: None, record_raw: false }
    }
}

pub struct Shared {
    sensor_enabled: bool,
    setting: ArcSwap<proto::ObstacleParams>,
    reading: ArcSwapOption<Reading>,
    active: AtomicBool,
}

impl Default for Shared {
    fn default() -> Self {
        Self {
            sensor_enabled: true,
            setting: ArcSwap::from_pointee(proto::ObstacleParams::default()),
            reading: ArcSwapOption::empty(),
            active: AtomicBool::new(false),
        }
    }
}

impl Shared {
    pub fn new(sensor_enabled: bool) -> Self {
        Self { sensor_enabled, ..Self::default() }
    }

    pub fn sensor_enabled(&self) -> bool {
        self.sensor_enabled
    }

    pub fn set(&self, params: proto::ObstacleParams, _now_ns: u64) -> Result<(), &'static str> {
        if params.enabled && !self.sensor_enabled {
            return Err("the ToF sensor is disabled");
        }
        if !(proto::OBSTACLE_MIN_THRESHOLD_MM..=proto::OBSTACLE_MAX_THRESHOLD_MM)
            .contains(&params.threshold_mm)
        {
            return Err("obstacle threshold must be between 100 and 2000 mm");
        }
        if params.clear_delay_ms > proto::OBSTACLE_MAX_CLEAR_DELAY_MS {
            return Err("obstacle clear delay must be between 0 and 5000 ms");
        }
        self.setting.store(Arc::new(params));
        if !params.enabled {
            self.active.store(false, Ordering::Relaxed);
        }
        Ok(())
    }

    fn params(&self) -> proto::ObstacleParams {
        **self.setting.load()
    }

    /// Keep fresh invalid frames distinct from absent/stale frames for scan timing.
    pub fn frame(&self, now_ns: u64) -> Option<(Option<u32>, u64)> {
        let reading = self.reading.load();
        Self::reading_frame(reading.as_deref()?, now_ns)
    }

    fn reading_frame(reading: &Reading, now_ns: u64) -> Option<(Option<u32>, u64)> {
        if reading.sampled_monotonic_ns == 0
            || now_ns < reading.sampled_monotonic_ns
            || now_ns - reading.sampled_monotonic_ns > READING_MAX_AGE_NS
        {
            return None;
        }
        Some((reading.distance_mm
            .filter(|distance| (MIN_DISTANCE_MM..=MAX_DISTANCE_MM).contains(distance)),
            reading.sampled_monotonic_ns))
    }

    fn distance(&self, now_ns: u64) -> Option<u32> {
        self.frame(now_ns).and_then(|(distance, _)| distance)
    }

    /// Return only a fresh, explicitly valid measurement.
    pub fn valid_distance(&self, now_ns: u64) -> Option<u32> {
        let status = self.status(now_ns);
        (status.sensor_status == proto::DistanceSensorStatus::Valid).then_some(status.distance_mm).flatten()
    }

    fn sensor_status(reading: Option<&Reading>, now_ns: u64) -> proto::DistanceSensorStatus {
        use proto::DistanceSensorStatus as Status;
        let Some(reading) = reading else {
            return Status::Unknown;
        };
        if reading.sampled_monotonic_ns == 0 || now_ns < reading.sampled_monotonic_ns {
            return Status::Unknown;
        }
        if matches!(reading.sensor_status, Status::Unavailable | Status::Error) {
            return reading.sensor_status;
        }
        if now_ns - reading.sampled_monotonic_ns > READING_MAX_AGE_NS {
            return Status::Unknown;
        }
        if reading.sensor_status == Status::Valid
            && !reading.distance_mm.is_some_and(|distance| {
                (MIN_DISTANCE_MM..=MAX_DISTANCE_MM).contains(&distance)
            })
        {
            return Status::Invalid;
        }
        reading.sensor_status
    }

    pub fn status(&self, now_ns: u64) -> proto::ObstacleStatus {
        self.status_with_sample_time(now_ns).0
    }

    /// Status and timestamp from the same sensor snapshot, for following diagnostics.
    pub fn status_with_sample_time(&self, now_ns: u64) -> (proto::ObstacleStatus, Option<u64>) {
        let (status, sampled_at, _) = self.status_with_raw(now_ns);
        (status, sampled_at)
    }

    /// The status, sample time, and optional unfiltered reading come from one snapshot.
    pub fn status_with_raw(&self, now_ns: u64) -> (proto::ObstacleStatus, Option<u64>, Option<u32>) {
        let params = self.params();
        let reading = self.reading.load();
        let reading = reading.as_deref();
        (proto::ObstacleStatus {
            distance_mm: reading
                .and_then(|reading| Self::reading_frame(reading, now_ns))
                .and_then(|(distance, _)| distance),
            sensor_status: if self.sensor_enabled {
                Self::sensor_status(reading, now_ns)
            } else {
                proto::DistanceSensorStatus::Unavailable
            },
            enabled: params.enabled,
            threshold_mm: params.threshold_mm,
            clear_delay_ms: params.clear_delay_ms,
            active: params.enabled && self.active.load(Ordering::Relaxed),
        }, reading.map(|reading| reading.sampled_monotonic_ns), reading.and_then(|reading| reading.raw_distance_mm))
    }
}

#[derive(Default)]
pub struct Assistance {
    active: bool,
    clear_since_ns: Option<u64>,
}

impl Assistance {
    /// Apply only the additional yaw; manual smoothing remains independent.
    pub fn apply(
        &mut self,
        shared: &Shared,
        now_ns: u64,
        eligible: bool,
        requested: [f64; 3],
        mut smoothed: [f64; 3],
        max_angular: Option<f64>,
    ) -> [f64; 3] {
        let params = shared.params();
        if !params.enabled || !eligible || requested.iter().all(|value| *value == 0.0) {
            self.active = false;
            self.clear_since_ns = None;
        } else {
            let distance = shared.distance(now_ns).unwrap_or(MAX_DISTANCE_MM);
            if distance < params.threshold_mm {
                self.active = true;
                self.clear_since_ns = None;
            } else if distance > params.threshold_mm && self.active {
                let since = *self.clear_since_ns.get_or_insert(now_ns);
                if now_ns.saturating_sub(since) >= u64::from(params.clear_delay_ms) * 1_000_000 {
                    self.active = false;
                    self.clear_since_ns = None;
                }
            } else {
                self.clear_since_ns = None;
            }
        }
        if self.active {
            smoothed[2] += if requested[2] > 0.0 { 1.0 } else { -1.0 };
            if let Some(limit) = max_angular {
                smoothed[2] = smoothed[2].clamp(-limit, limit);
            }
        }
        shared.active.store(self.active, Ordering::Relaxed);
        smoothed
    }
}

/// The optional adjacent launcher exists only where this sensor is installed.
pub fn start_sensor(real_hardware: bool, shared: Arc<Shared>, config: SensorConfig) -> Option<JoinHandle<()>> {
    if !real_hardware {
        return None;
    }
    Some(tokio::spawn(async move {
        if let Err(error) = run_sensor(Arc::clone(&shared), config).await {
            tracing::warn!(%error, "single-zone distance sensor unavailable");
        }
        shared.reading.store(None);
    }))
}

async fn run_sensor(shared: Arc<Shared>, config: SensorConfig) -> io::Result<()> {
    let executable = std::env::current_exe()?;
    let launcher = executable
        .parent()
        .ok_or_else(|| io::Error::other("no executable directory"))?
        .join("tof-distance-service");
    run_sensor_at(shared, &launcher, config).await
}

async fn run_sensor_at(shared: Arc<Shared>, launcher: &Path, config: SensorConfig) -> io::Result<()> {
    if !launcher.exists() {
        tracing::info!(path = %launcher.display(), "single-zone sensor launcher not installed");
        return Ok(());
    }
    let mut command = tokio::process::Command::new(&launcher);
    command
        .args(["--bus", "5", "--hz"])
        .arg(config.hz.to_string())
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true);
    if let Some(ms) = config.timing_budget_ms { command.arg("--timing-budget-ms").arg(ms.to_string()); }
    if let Some(rate) = config.signal_rate_limit { command.arg("--signal-rate-limit").arg(rate.to_string()); }
    if config.record_raw { command.arg("--include-raw"); }
    tracing::info!(?config, "single-zone distance sensor configured");
    #[cfg(target_os = "linux")]
    {
        use std::os::unix::process::CommandExt;
        let parent = unsafe { libc::getpid() };
        command.as_std_mut().process_group(0);
        // SAFETY: the child only invokes libc and constructs errno errors after fork.
        unsafe {
            command.as_std_mut().pre_exec(move || {
                if libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGTERM) != 0 {
                    return Err(io::Error::last_os_error());
                }
                if libc::getppid() != parent {
                    return Err(io::Error::from_raw_os_error(libc::ESRCH));
                }
                Ok(())
            });
        }
    }
    let mut child = command.spawn()?;
    let stdout = child
        .stdout
        .take()
        .ok_or_else(|| io::Error::other("missing sensor output"))?;
    let stderr = child.stderr.take().ok_or_else(|| io::Error::other("missing sensor diagnostics"))?;
    let diagnostics = tokio::spawn(async move {
        let mut lines = BufReader::new(stderr).lines();
        loop {
            match lines.next_line().await {
                Ok(Some(line)) => tracing::warn!("single-zone sensor diagnostic: {line}"),
                Ok(None) => break,
                Err(error) => {
                    tracing::warn!(%error, "single-zone sensor diagnostic stream failed");
                    break;
                }
            }
        }
    });
    let mut lines = BufReader::new(stdout).lines();
    while let Some(line) = lines.next_line().await? {
        match serde_json::from_str::<Reading>(&line) {
            Ok(reading) => shared.reading.store(Some(Arc::new(reading))),
            Err(error) => {
                shared.reading.store(None);
                return Err(io::Error::other(format!("invalid sensor frame: {error}")));
            }
        }
    }
    let status = child.wait().await?;
    if let Err(error) = diagnostics.await {
        tracing::warn!(%error, "single-zone sensor diagnostic task failed");
    }
    Err(io::Error::other(format!("sensor process exited: {status}")))
}
