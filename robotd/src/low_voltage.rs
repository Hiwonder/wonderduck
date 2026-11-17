use std::sync::Mutex;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use tokio::sync::Notify;

struct Monitor {
    settings: proto::LowVoltageParams,
    low_since: Option<Instant>,
    last_sample: Option<Instant>,
    voltage: Option<(f64, Instant)>,
    triggered: bool,
}

pub struct LowVoltage {
    monitor: Mutex<Monitor>,
    pub triggered: Notify,
}

impl Default for LowVoltage {
    fn default() -> Self {
        Self {
            monitor: Mutex::new(Monitor {
                settings: proto::LowVoltageParams::default(),
                low_since: None,
                last_sample: None,
                voltage: None,
                triggered: false,
            }),
            triggered: Notify::new(),
        }
    }
}

impl LowVoltage {
    pub fn status(&self) -> proto::LowVoltageStatus {
        self.status_at(Instant::now())
    }

    fn status_at(&self, now: Instant) -> proto::LowVoltageStatus {
        let monitor = self.monitor.lock().expect("low voltage lock poisoned");
        let voltage_v = monitor.voltage.and_then(|(voltage, sampled)|
            (now >= sampled && now.duration_since(sampled) <= Duration::from_millis(250)).then_some(voltage));
        let low_for_ms = voltage_v.and_then(|_| monitor.low_since).map(|since|
            u64::try_from(now.saturating_duration_since(since).as_millis()).unwrap_or(u64::MAX));
        proto::LowVoltageStatus { settings: monitor.settings, voltage_v, low_for_ms,
                                  triggered: monitor.triggered }
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
        monitor.voltage = voltage.filter(|v| v.is_finite() && *v > 0.0).map(|v| (v, now));
        let low = voltage.is_some_and(|v| v.is_finite() && v > 0.0 && v < monitor.settings.threshold_v);
        if !low {
            monitor.low_since = None;
            monitor.last_sample = None;
            return false;
        }
        if monitor.last_sample.is_some_and(|last| now < last || now.duration_since(last) > Duration::from_millis(250)) {
            monitor.low_since = None;
        }
        monitor.last_sample = Some(now);
        let since = *monitor.low_since.get_or_insert(now);
        if monitor.triggered || now.saturating_duration_since(since) < Duration::from_secs(monitor.settings.duration_s) {
            return false;
        }
        monitor.triggered = true;
        true
    }

    /// Publish read-only recovery samples without triggering a new sit request.
    pub fn observe_recovery(&self, voltage: Option<f64>, now: Instant, triggered: bool) {
        let mut monitor = self.monitor.lock().expect("low voltage lock poisoned");
        monitor.triggered = triggered;
        monitor.voltage = voltage.filter(|v| v.is_finite() && *v > 0.0).map(|v| (v, now));
        let low = voltage.is_some_and(|v| v.is_finite() && v > 0.0 && v < monitor.settings.threshold_v);
        if !low {
            monitor.low_since = None;
            monitor.last_sample = None;
            return;
        }
        if monitor.last_sample.is_some_and(|last| now < last || now.duration_since(last) > Duration::from_millis(250)) {
            monitor.low_since = None;
        }
        monitor.last_sample = Some(now);
        monitor.low_since.get_or_insert(now);
    }
}
