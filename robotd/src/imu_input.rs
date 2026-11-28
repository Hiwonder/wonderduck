//! Freshness of the sample actually handed to the controller, without extra device reads.

use std::time::Duration;

use duck_control::io::{ImuSampleTiming, ImuTimingSource};
use serde::Serialize;

const REPORT_INTERVAL_NS: u64 = 1_000_000_000;
const UNCHANGED_PAYLOAD_RUN: u64 = 3;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Input {
    pub source: ImuTimingSource,
    pub sequence: u64,
    pub acquired_t_ns: u64,
    pub inference_t_ns: u64,
    /// Host acquisition age, or the upper bound on UNO Q feedback scan age.
    pub age_us: u64,
    pub limit_us: u64,
    pub reused: bool,
    pub coasted: bool,
    pub age_exceeded: bool,
    pub payload_unchanged_run: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mcu: Option<duck_control::imu_timing::McuTiming>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub host: Option<duck_control::imu_host::HostImuTiming>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub i2c: Option<duck_control::io::I2cReadTiming>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub software_read_age_bounds_us: Option<[u64; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unoq_feedback_age_bounds_us: Option<[u64; 2]>,
    pub mcu_sample_reused: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Fault {
    OldInput,
    UnchangedPayload,
}

impl Input {
    fn fault(self) -> Option<Fault> {
        if self.reused || self.age_exceeded || self.mcu_sample_reused {
            Some(Fault::OldInput)
        } else if self.mcu.is_none() && matches!(self.source, ImuTimingSource::Id200 | ImuTimingSource::Id201)
            && self.payload_unchanged_run >= UNCHANGED_PAYLOAD_RUN
        {
            Some(Fault::UnchangedPayload)
        } else {
            None
        }
    }
}

pub struct Monitor {
    limit_us: u64,
    previous: Option<ImuSampleTiming>,
    fault: Option<Fault>,
    last_report_t_ns: u64,
    run: u64,
    max_age_us: u64,
}

impl Monitor {
    pub fn new(period: Duration) -> Self {
        Self {
            limit_us: period.as_micros() as u64,
            previous: None,
            fault: None,
            last_report_t_ns: 0,
            run: 0,
            max_age_us: 0,
        }
    }

    /// Disabled, homing and warmup ticks do not count as controller consumption.
    pub fn pause(&mut self) {
        self.previous = None;
        self.fault = None;
        self.run = 0;
        self.max_age_us = 0;
    }

    pub fn consume(
        &mut self,
        timing: Option<ImuSampleTiming>,
        inference_t_ns: u64,
        fresh: bool,
    ) -> Option<Input> {
        let Some(timing) = timing else {
            // Simulated and other backends do not supply physical acquisition evidence.
            self.pause();
            return None;
        };
        let previous_sequence = self.previous.map(|previous| previous.sequence);
        let age_us = inference_t_ns.saturating_sub(timing.acquired_t_ns) / 1000;
        let unoq_feedback_age_bounds_us = timing.unoq_feedback_scan_start_ns
            .map(|[earliest_ns, latest_ns]| [latest_ns, earliest_ns]
                .map(|timestamp| inference_t_ns.saturating_sub(timestamp) / 1000));
        let confirmed_age_us = unoq_feedback_age_bounds_us.map_or(age_us, |bounds| bounds[0]);
        let input = Input {
            source: timing.source,
            sequence: timing.sequence,
            acquired_t_ns: timing.acquired_t_ns,
            inference_t_ns,
            age_us,
            limit_us: self.limit_us,
            reused: !fresh
                || self.previous.is_some_and(|previous| {
                    previous.source == timing.source && previous.sequence == timing.sequence
                }),
            coasted: !fresh,
            age_exceeded: confirmed_age_us > self.limit_us,
            payload_unchanged_run: timing.payload_unchanged_run,
            mcu: timing.mcu,
            host: timing.host,
            i2c: timing.i2c,
            software_read_age_bounds_us: timing.mcu.map(|mcu| mcu.software_age_bounds_us(inference_t_ns))
                .or_else(|| timing.host.map(|host| host.software_age_bounds_us(inference_t_ns))),
            unoq_feedback_age_bounds_us,
            mcu_sample_reused: timing.mcu.is_some_and(|mcu| self.previous
                .and_then(|p| p.mcu).is_some_and(|p| p.fusion_sequence == mcu.fusion_sequence)),
        };
        self.previous = Some(timing);
        let fault = input.fault();
        let changed = fault != self.fault;
        if changed {
            self.run = 0;
            self.max_age_us = 0;
        }
        if let Some(fault) = fault {
            self.run = self.run.saturating_add(1);
            self.max_age_us = self.max_age_us.max(age_us);
            if changed || inference_t_ns.saturating_sub(self.last_report_t_ns) >= REPORT_INTERVAL_NS
            {
                let reason = match fault {
                    Fault::OldInput if input.mcu_sample_reused => "mcu_sample_reused",
                    Fault::OldInput if input.reused && input.age_exceeded => "reused_and_over_age",
                    Fault::OldInput if input.reused => "reused_frame",
                    Fault::OldInput => "over_age",
                    Fault::UnchangedPayload => "payload_unchanged_sensor_freshness_unknown",
                };
                let source = input.source.as_str();
                let sequence_basis = if input.mcu.is_some() { "host_reply_with_mcu_fusion_sequence" }
                    else { input.source.sequence_basis() };
                let age_ms = input.age_us as f64 / 1000.0;
                let limit_ms = input.limit_us as f64 / 1000.0;
                let run = self.run;
                let sequence = input.sequence;
                let acquired_t_ns = input.acquired_t_ns;
                let reused = input.reused;
                let coasted = input.coasted;
                let payload_unchanged_run = input.payload_unchanged_run;
                let max_age_us = self.max_age_us;
                // Unknown internal sensor freshness must never be called a confirmed timeout.
                match fault {
                    Fault::OldInput => tracing::warn!(
                        source,
                        reason,
                        age_ms,
                        limit_ms,
                        run,
                        sequence_basis,
                        sequence,
                        ?previous_sequence,
                        acquired_t_ns,
                        inference_t_ns,
                        reused,
                        coasted,
                        payload_unchanged_run,
                        unoq_feedback_age_bounds_us = ?input.unoq_feedback_age_bounds_us,
                        max_age_us,
                        "policy IMU input stale"
                    ),
                    Fault::UnchangedPayload => tracing::info!(
                        source,
                        reason,
                        age_ms,
                        limit_ms,
                        run,
                        sequence_basis,
                        sequence,
                        ?previous_sequence,
                        acquired_t_ns,
                        inference_t_ns,
                        reused,
                        coasted,
                        payload_unchanged_run,
                        max_age_us,
                        "policy IMU payload unchanged; sensor freshness unknown"
                    ),
                }
                self.last_report_t_ns = inference_t_ns;
            }
        } else if changed {
            tracing::info!(
                source = input.source.as_str(),
                age_ms = age_us as f64 / 1000.0,
                sequence = input.sequence,
                inference_t_ns,
                "policy IMU input recovered"
            );
        }
        self.fault = fault;
        Some(input)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unoq_reports_only_confirmed_over_age_and_retains_the_interval() {
        let mut monitor = Monitor::new(Duration::from_millis(20));
        let mut timing = ImuSampleTiming {
            source: ImuTimingSource::Id200,
            sequence: 1,
            acquired_t_ns: 10_000_000,
            consumed_t_ns: 30_000_000,
            age_us: 20_000,
            payload_unchanged_run: 0,
            unoq_feedback_scan_start_ns: Some([10_000_000, 16_000_000]),
            mcu: None,
            host: None,
            i2c: None,
        };
        let input = monitor.consume(Some(timing), 33_000_000, true).unwrap();
        assert_eq!(input.unoq_feedback_age_bounds_us, Some([17_000, 23_000]));
        assert!(!input.age_exceeded);
        assert_eq!(input.fault(), None);

        let reused = monitor.consume(Some(timing), 34_000_000, true).unwrap();
        assert!(reused.reused);
        assert_eq!(reused.fault(), Some(Fault::OldInput));

        timing.sequence = 2;
        let old = monitor.consume(Some(timing), 37_000_000, true).unwrap();
        assert!(old.age_exceeded);
        assert_eq!(old.fault(), Some(Fault::OldInput));
    }
}
