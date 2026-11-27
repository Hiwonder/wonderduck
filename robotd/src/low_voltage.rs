use std::path::Path;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use tokio::sync::Notify;

const PROMPT_MARKER: &str = "/var/lib/robot/prompt-low-battery.active";

struct Monitor {
    settings: proto::LowVoltageParams,
    low_since: Option<Instant>,
    last_sample: Option<Instant>,
    voltage: Option<(f64, Instant)>,
    warning_active: bool,
    triggered: bool,
}

impl Monitor {
    fn observe_voltage(&mut self, voltage: Option<f64>, now: Instant) -> Option<Duration> {
        let voltage = voltage.filter(|value| value.is_finite() && *value > 0.0);
        self.voltage = voltage.map(|value| (value, now));
        if voltage.is_some_and(|value| value >= self.settings.threshold_v) {
            self.warning_active = false;
        }
        if voltage.is_none_or(|value| value >= self.settings.threshold_v) {
            self.low_since = None;
            self.last_sample = None;
            return None;
        }
        if self.last_sample.is_some_and(|last| now < last || now.duration_since(last) > Duration::from_millis(250)) {
            self.low_since = None;
        }
        self.last_sample = Some(now);
        let since = *self.low_since.get_or_insert(now);
        let elapsed = now.saturating_duration_since(since);
        self.warning_active |= self.triggered || elapsed >= Duration::from_secs(1);
        Some(elapsed)
    }
}

pub struct LowVoltage {
    monitor: Mutex<Monitor>,
    pub triggered: Notify,
    pub use_prompt_marker: bool,
}

impl Default for LowVoltage {
    fn default() -> Self {
        Self {
            monitor: Mutex::new(Monitor {
                settings: proto::LowVoltageParams::default(),
                low_since: None,
                last_sample: None,
                voltage: None,
                warning_active: false,
                triggered: false,
            }),
            triggered: Notify::new(),
            use_prompt_marker: false,
        }
    }
}

impl LowVoltage {
    pub fn status(&self) -> proto::LowVoltageStatus {
        self.status_at(Instant::now())
    }

    pub fn client_status(&self) -> proto::LowVoltageStatus {
        let mut status = self.status();
        // The prompt service can also detect low voltage from maintenance
        // snapshots while the control loop does not own the bus.
        if self.use_prompt_marker && !status.warning_active {
            status.warning_active = Path::new(PROMPT_MARKER).exists();
        }
        status
    }

    fn status_at(&self, now: Instant) -> proto::LowVoltageStatus {
        let monitor = self.monitor.lock().expect("low voltage lock poisoned");
        let voltage_v = monitor.voltage.and_then(|(voltage, sampled)|
            (now >= sampled && now.duration_since(sampled) <= Duration::from_millis(250)).then_some(voltage));
        let low_for_ms = voltage_v.and_then(|_| monitor.low_since).map(|since|
            u64::try_from(now.saturating_duration_since(since).as_millis()).unwrap_or(u64::MAX));
        proto::LowVoltageStatus {
            settings: monitor.settings,
            voltage_v,
            low_for_ms,
            warning_active: monitor.warning_active || monitor.triggered && voltage_v.is_none(),
            triggered: monitor.triggered,
        }
    }

    pub fn set(&self, settings: proto::LowVoltageParams) -> Result<(), &'static str> {
        let monitor = self.monitor.lock().expect("low voltage lock poisoned");
        if monitor.triggered {
            return Err("low voltage shutdown already triggered");
        }
        if monitor.settings != settings {
            return Err("low voltage protection settings are fixed");
        }
        Ok(())
    }

    /// Called for every control read, including failures. A missing sample breaks continuity.
    pub fn observe(&self, voltage: Option<f64>, now: Instant) -> bool {
        let mut monitor = self.monitor.lock().expect("low voltage lock poisoned");
        let Some(elapsed) = monitor.observe_voltage(voltage, now) else { return false; };
        if monitor.triggered || elapsed < Duration::from_secs(monitor.settings.duration_s) {
            return false;
        }
        monitor.triggered = true;
        true
    }

    /// Publish read-only recovery samples without inferring a trip from the startup interlock.
    pub fn observe_recovery(&self, voltage: Option<f64>, now: Instant) {
        let mut monitor = self.monitor.lock().expect("low voltage lock poisoned");
        monitor.observe_voltage(voltage, now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn waiting_for_voltage_recovery_does_not_report_a_new_trip() {
        let monitor = LowVoltage::default();
        let now = Instant::now();
        monitor.observe_recovery(Some(7.9), now);
        let later = now + Duration::from_secs(1);
        monitor.observe_recovery(None, later);
        let status = monitor.status_at(later);
        assert!(status.voltage_v.is_none());
        assert!(!status.triggered);
        assert!(!status.warning_active);
    }

    #[test]
    fn warning_starts_before_protection_and_survives_a_missing_sample() {
        let monitor = LowVoltage::default();
        let start = Instant::now();
        for step in 0..=10 {
            let now = start + Duration::from_millis(step * 100);
            assert!(!monitor.observe(Some(6.9), now));
            let status = monitor.status_at(now);
            assert_eq!(status.warning_active, step == 10);
            assert!(!status.triggered);
        }
        let now = start + Duration::from_millis(1100);
        monitor.observe(None, now);
        assert!(monitor.status_at(now).warning_active);
        monitor.observe(Some(7.0), now);
        assert!(!monitor.status_at(now).warning_active);
    }
}
