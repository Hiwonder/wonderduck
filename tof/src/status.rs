//! What the daemon says about its sensor, for a subscriber that has just asked.
//!
//! Written by the sensor thread, read by every connection — one small value, so a
//! mutex is the whole mechanism. It answers the question a viewer must not have
//! to guess at: is there no sensor, or has it simply not produced a frame yet?

use std::sync::Mutex;

use duck_ipc_proto as proto;

pub struct Status {
    hz: u8,
    inner: Mutex<Inner>,
}

struct Inner {
    /// The generation that answered, once one has.
    sensor: Option<&'static str>,
    /// Why there is none, when there is none.
    unavailable: Option<String>,
}

impl Status {
    pub fn new(hz: u8) -> Self {
        Self {
            hz,
            inner: Mutex::new(Inner {
                sensor: None,
                // Before the first attempt finishes, "starting" is the honest
                // answer: the firmware upload takes seconds, and a viewer that
                // opened in that window should see a reason rather than nothing.
                unavailable: Some("bringing the sensor up".to_owned()),
            }),
        }
    }

    /// A sensor is ranging.
    pub fn up(&self, sensor: &'static str) {
        let mut inner = self.lock();
        inner.sensor = Some(sensor);
        inner.unavailable = None;
    }

    /// There is no sensor, and this is why.
    pub fn down(&self, why: &str) {
        let mut inner = self.lock();
        inner.sensor = None;
        inner.unavailable = Some(why.to_owned());
    }

    /// The answer to `tof.stream`.
    pub fn result(&self) -> proto::TofStreamResult {
        let inner = self.lock();
        proto::TofStreamResult {
            // Accepted either way: the subscription is valid and frames will
            // arrive if a sensor appears. Refusing would make a client that
            // subscribed one second early give up for good.
            accepted: true,
            sensor: inner.sensor.map(str::to_owned),
            unavailable: inner.unavailable.clone(),
            rows: tof::ROWS as u8,
            cols: tof::COLS as u8,
            hz: self.hz,
        }
    }

    /// A poisoned lock cannot happen — nothing here panics while holding it — and
    /// treating one as fatal would take the daemon down over a status field.
    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }
}
