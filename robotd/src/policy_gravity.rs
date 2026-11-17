//! An explicit, freshness-gated gravity input from the other physical bus IMU.

use std::time::Duration;

use duck_control::Sensors;
use duck_control::io::ImuTimingSource;
use serde::Serialize;

use crate::imu_input;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Evidence {
    pub gyro_source: Option<ImuTimingSource>,
    pub gravity_source: Option<ImuTimingSource>,
    pub checked_at_ns: u64,
    pub input: Option<imu_input::Input>,
    pub reference_error: Option<&'static str>,
    pub error: Option<&'static str>,
    /// Set only when handed to the controller's observation construction.
    pub used_gravity: Option<[f64; 3]>,
    #[serde(skip)]
    gravity: Option<[f64; 3]>,
}

impl Evidence {
    /// Recheck immediately before the controller call without consuming the sequence twice.
    pub fn at_consumption(&mut self, now_ns: u64) -> Result<[f64; 3], &'static str> {
        self.checked_at_ns = now_ns;
        if let Some(input) = self.input.as_mut() {
            input.inference_t_ns = now_ns;
            input.age_us = now_ns.saturating_sub(input.acquired_t_ns) / 1000;
            input.age_exceeded = input.age_us > input.limit_us;
            input.software_read_age_bounds_us =
                input.mcu.map(|mcu| mcu.software_age_bounds_us(now_ns));
            if self.error.is_none() {
                self.error = freshness_error(input);
            }
        }
        if let Some(error) = self.error {
            return Err(error);
        }
        let gravity = self.gravity.ok_or("reference_gravity_missing")?;
        self.used_gravity = Some(gravity);
        Ok(gravity)
    }
}

fn freshness_error(input: &imu_input::Input) -> Option<&'static str> {
    if input.coasted {
        Some("reference_from_coasted_read")
    } else if input.reused {
        Some("reference_host_sample_reused")
    } else if input.mcu_sample_reused {
        Some("reference_mcu_sample_reused")
    } else if input.age_exceeded {
        Some("reference_host_age_exceeded")
    } else if input.mcu.is_none() {
        Some("reference_mcu_timing_missing")
    } else if input
        .software_read_age_bounds_us
        .is_some_and(|bounds| bounds[1] > input.limit_us)
    {
        Some("reference_mcu_age_exceeded")
    } else {
        None
    }
}

pub struct Selector {
    monitor: imu_input::Monitor,
    error: Option<&'static str>,
}

impl Selector {
    pub fn new(period: Duration) -> Self {
        Self {
            monitor: imu_input::Monitor::new(period.min(Duration::from_millis(20))),
            error: None,
        }
    }

    /// Only an intentional pause clears sequence history; rejected references never do.
    pub fn pause(&mut self) {
        self.monitor.pause();
        self.error = None;
    }

    pub fn select(&mut self, sensors: &Sensors, now_ns: u64, fresh: bool) -> Evidence {
        let reference = sensors.feedback.and_then(|feedback| feedback.reference_imu);
        let sample = reference.and_then(|reference| reference.sample);
        let mut evidence = Evidence {
            gyro_source: sensors.imu_timing.map(|timing| timing.source),
            gravity_source: reference.map(|reference| reference.source),
            checked_at_ns: now_ns,
            input: None,
            reference_error: reference.and_then(|reference| reference.error),
            error: None,
            used_gravity: None,
            gravity: sample.map(|sample| sample.gravity_trunk),
        };
        evidence.error = (|| {
            let expected = match evidence.gyro_source {
                Some(ImuTimingSource::Id200) => ImuTimingSource::Id201,
                Some(ImuTimingSource::Id201) => ImuTimingSource::Id200,
                _ => return Some("primary_physical_imu_source_missing"),
            };
            let Some(reference) = reference else {
                return Some("reference_missing");
            };
            if reference.source != expected {
                return Some("reference_source_mismatch");
            }
            // Retain the last valid source's sequence across transport/decode failures.
            if reference.error.is_some() {
                return Some("reference_read_error");
            }
            let Some(sample) = sample else {
                return Some("reference_sample_missing");
            };
            if sample.timing.source != expected {
                return Some("reference_timing_source_mismatch");
            }
            evidence.input = self.monitor.consume(Some(sample.timing), now_ns, fresh);
            let input = evidence.input.as_ref().expect("timing provided");
            if sample.timing.acquired_t_ns > now_ns
                || input.mcu.is_some_and(|mcu| {
                    mcu.host_request_start_ns > mcu.host_reply_end_ns
                        || mcu.host_reply_end_ns > now_ns
                })
            {
                return Some("reference_timing_invalid");
            }
            if let Some(error) = freshness_error(input) {
                return Some(error);
            }
            let gravity = sample.gravity_trunk;
            let norm_squared: f64 = gravity.iter().map(|value| value * value).sum();
            if gravity.iter().any(|value| !value.is_finite()) || (norm_squared - 1.0).abs() > 0.001
            {
                return Some("reference_gravity_invalid");
            }
            None
        })();
        if evidence.error.is_some() {
            self.report(&evidence);
        }
        evidence
    }

    pub fn report(&mut self, evidence: &Evidence) {
        if evidence.error != self.error {
            if let Some(reason) = evidence.error {
                tracing::warn!(reason, gyro_source = ?evidence.gyro_source,
                    gravity_source = ?evidence.gravity_source, reference_error = ?evidence.reference_error,
                    "policy reference gravity rejected; inference blocked, holding pose");
            } else {
                tracing::info!(gyro_source = ?evidence.gyro_source,
                    gravity_source = ?evidence.gravity_source,
                    "policy reference gravity recovered; inference may resume");
            }
        }
        self.error = evidence.error;
    }
}
