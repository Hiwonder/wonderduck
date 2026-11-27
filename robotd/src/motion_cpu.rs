//! CPU frequency and wake-up latency requests during physical motion control.

use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

pub(crate) struct MotionCpuFloor {
    khz: u32,
    minimum: PathBuf,
    saved: PathBuf,
    applied: Option<bool>,
    retry_at: Option<Instant>,
    latency: Option<MotionCpuLatency>,
}

impl MotionCpuFloor {
    pub(crate) fn configured() -> Option<Self> {
        let khz = std::env::var("WONDERDUCK_MOTION_CPU_MIN_KHZ")
            .ok()
            .and_then(|value| match value.parse::<u32>() {
                Ok(khz) => Some(khz),
                Err(_) => {
                    tracing::warn!(%value, "invalid motion CPU frequency floor");
                    None
                }
            })
            .unwrap_or(0);
        let latency = std::env::var("WONDERDUCK_MOTION_CPU_DMA_LATENCY_US")
            .ok()
            .and_then(|value| match value.parse::<i32>() {
                Ok(micros) if micros >= 0 => Some(MotionCpuLatency {
                    micros,
                    device: "/dev/cpu_dma_latency".into(),
                    request: None,
                    retry_at: None,
                }),
                _ => {
                    tracing::warn!(%value, "invalid motion CPU wake-up latency");
                    None
                }
            });
        (khz > 0 || latency.is_some()).then(|| Self {
            khz,
            minimum: "/sys/devices/system/cpu/cpufreq/policy0/scaling_min_freq".into(),
            saved: "/run/robotd/motion-cpu-min-khz".into(),
            applied: None,
            retry_at: None,
            latency,
        })
    }

    pub(crate) fn update(&mut self, active: bool, now: Instant) {
        if let Some(latency) = &mut self.latency {
            latency.update(active, now);
        }
        if self.khz == 0 || self.applied == Some(active) || self.retry_at.is_some_and(|retry| now < retry) {
            return;
        }
        let result = if active { self.raise() } else { self.restore() };
        match result {
            Ok(()) => {
                self.applied = Some(active);
                self.retry_at = None;
            }
            Err(error) => {
                if self.retry_at.is_none() {
                    tracing::warn!(%error, active, "cannot change motion CPU frequency floor");
                }
                self.applied = None;
                self.retry_at = Some(now + Duration::from_secs(1));
            }
        }
    }

    fn raise(&self) -> io::Result<()> {
        // Save the idle setting before touching sysfs. ExecStopPost can then
        // restore it even if the control process is killed during the transition.
        if !self.saved.try_exists()? {
            let idle = Self::read_khz(&self.minimum)?;
            if idle >= self.khz {
                return Ok(());
            }
            let temporary = self.saved.with_extension("tmp");
            fs::write(&temporary, idle.to_string())?;
            fs::rename(temporary, &self.saved)?;
        }
        fs::write(&self.minimum, self.khz.to_string())
    }

    fn restore(&self) -> io::Result<()> {
        let idle = match Self::read_khz(&self.saved) {
            Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(()),
            result => result?,
        };
        fs::write(&self.minimum, idle.to_string())?;
        fs::remove_file(&self.saved)
    }

    fn read_khz(path: &Path) -> io::Result<u32> {
        fs::read_to_string(path)?
            .trim()
            .parse()
            .map_err(io::Error::other)
    }
}

impl Drop for MotionCpuFloor {
    fn drop(&mut self) {
        if self.khz != 0 && let Err(error) = self.restore() {
            tracing::warn!(%error, "cannot restore idle CPU frequency floor");
        }
    }
}

struct MotionCpuLatency {
    micros: i32,
    device: PathBuf,
    request: Option<fs::File>,
    retry_at: Option<Instant>,
}

impl MotionCpuLatency {
    fn update(&mut self, active: bool, now: Instant) {
        if !active {
            self.request = None;
            self.retry_at = None;
            return;
        }
        if self.request.is_some() || self.retry_at.is_some_and(|retry| now < retry) {
            return;
        }
        match fs::OpenOptions::new().write(true).open(&self.device).and_then(|mut file| {
            file.write_all(&self.micros.to_ne_bytes())?;
            Ok(file)
        }) {
            // The kernel releases this PM QoS request when the descriptor closes.
            Ok(file) => {
                self.request = Some(file);
                self.retry_at = None;
            }
            Err(error) => {
                if self.retry_at.is_none() {
                    tracing::warn!(%error, "cannot request motion CPU wake-up latency");
                }
                self.retry_at = Some(now + Duration::from_secs(1));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn stationary_motion_and_idle_use_different_floors() {
        use crate::{DEFAULT_POSITION, FakeIo, Intents, Params, PolicyOverrides, RobotState};
        use std::sync::{Arc, atomic::Ordering};

        let dir = std::env::temp_dir().join(format!("robotd-cpu-loop-{}", std::process::id()));
        fs::create_dir(&dir).unwrap();
        let minimum = dir.join("scaling_min_freq");
        fs::write(&minimum, "300000").unwrap();
        let floor = MotionCpuFloor {
            khz: 1_612_800,
            minimum: minimum.clone(),
            saved: dir.join("original"),
            applied: None,
            retry_at: None,
            latency: None,
        };
        let mut params = Params::default();
        params.policy.enabled = false;
        let config = dir.join("robotd.toml");
        let intents = Arc::new(Intents::new());
        let state = Arc::new(RobotState::new(&params, &config, Arc::clone(&intents)));
        let period = Duration::from_millis(20);
        let controller = crate::control_loop(
            FakeIo::at(DEFAULT_POSITION),
            state.clone(),
            intents.clone(),
            params,
            config,
            PolicyOverrides::default(),
            period,
            Arc::new(|| {}),
            None,
            Some(floor),
        );
        let exercise = async {
            tokio::time::timeout(Duration::from_secs(3), async {
                while state.ticks.load(Ordering::Relaxed) == 0 {
                    tokio::time::sleep(period).await;
                }
                assert_eq!(MotionCpuFloor::read_khz(&minimum).unwrap(), 300_000);
                for (enabled, expected) in [(true, 1_612_800), (false, 300_000)] {
                    intents.set_enabled(enabled);
                    while MotionCpuFloor::read_khz(&minimum).unwrap() != expected {
                        tokio::time::sleep(Duration::from_millis(1)).await;
                    }
                }
            })
            .await
            .unwrap();
            state.shutdown.store(true, Ordering::Release);
        };
        tokio::join!(controller, exercise);
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn restores_the_previous_floor_and_recovers_an_interrupted_boost() {
        let dir = std::env::temp_dir().join(format!("robotd-motion-cpu-{}", std::process::id()));
        fs::create_dir(&dir).unwrap();
        let minimum = dir.join("scaling_min_freq");
        let saved = dir.join("original");
        let make_floor = || MotionCpuFloor {
            khz: 1_612_800,
            minimum: minimum.clone(),
            saved: saved.clone(),
            applied: None,
            retry_at: None,
            latency: None,
        };
        let now = Instant::now();
        fs::write(&minimum, "614400\n").unwrap();
        let mut floor = make_floor();
        floor.update(false, now);
        assert!(!saved.exists());
        floor.update(true, now);
        floor.update(true, now);
        assert_eq!(MotionCpuFloor::read_khz(&saved).unwrap(), 614_400);
        assert_eq!(MotionCpuFloor::read_khz(&minimum).unwrap(), 1_612_800);
        floor.update(false, now);
        assert_eq!(MotionCpuFloor::read_khz(&minimum).unwrap(), 614_400);
        assert!(!saved.exists());

        fs::write(&minimum, "1804800").unwrap();
        floor.update(true, now);
        assert!(!saved.exists());
        assert_eq!(MotionCpuFloor::read_khz(&minimum).unwrap(), 1_804_800);
        drop(floor);

        fs::write(&minimum, "1612800").unwrap();
        fs::write(&saved, "300000").unwrap();
        let mut recovered = make_floor();
        recovered.update(true, now);
        drop(recovered);
        assert_eq!(MotionCpuFloor::read_khz(&minimum).unwrap(), 300_000);
        assert!(!saved.exists());
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn holds_cpu_latency_request_only_while_active() {
        let device = std::env::temp_dir().join(format!("robotd-cpu-qos-{}", std::process::id()));
        fs::write(&device, []).unwrap();
        let mut latency = MotionCpuLatency {
            micros: 100,
            device: device.clone(),
            request: None,
            retry_at: None,
        };
        let now = Instant::now();
        latency.update(false, now);
        assert!(latency.request.is_none());
        latency.update(true, now);
        assert!(latency.request.is_some());
        assert_eq!(fs::read(&device).unwrap(), 100i32.to_ne_bytes());
        latency.update(false, now);
        assert!(latency.request.is_none());
        fs::remove_file(device).unwrap();
    }
}
