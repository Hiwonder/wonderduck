//! CPU frequency floor requested by the UNO Q service during physical motion control.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

pub(crate) struct MotionCpuFloor {
    khz: u32,
    minimum: PathBuf,
    saved: PathBuf,
    applied: Option<bool>,
    retry_at: Option<Instant>,
}

impl MotionCpuFloor {
    pub(crate) fn configured() -> Option<Self> {
        let value = std::env::var("WONDERDUCK_MOTION_CPU_MIN_KHZ").ok()?;
        let Ok(khz) = value.parse::<u32>() else {
            tracing::warn!(%value, "invalid motion CPU frequency floor");
            return None;
        };
        (khz > 0).then(|| Self {
            khz,
            minimum: "/sys/devices/system/cpu/cpufreq/policy0/scaling_min_freq".into(),
            saved: "/run/robotd/motion-cpu-min-khz".into(),
            applied: None,
            retry_at: None,
        })
    }

    pub(crate) fn update(&mut self, active: bool, now: Instant) {
        if self.applied == Some(active) || self.retry_at.is_some_and(|retry| now < retry) {
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
        if let Err(error) = self.restore() {
            tracing::warn!(%error, "cannot restore idle CPU frequency floor");
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
        };
        let mut params = Params::default();
        params.policy.enabled = false;
        let config = dir.join("robotd.toml");
        let state = Arc::new(RobotState::new(&params, &config));
        let intents = Arc::new(Intents::new());
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
}
