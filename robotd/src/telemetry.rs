//! Latest control snapshots; geometry and board-temperature IO belong to this worker.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, TryLockError};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use duck_control::{NUM_JOINTS, Sensors};
use duck_ipc_proto as proto;

use crate::{RobotState, mapping, soc};

#[derive(Default)]
pub(crate) struct Subscriptions {
    next_id: AtomicU64,
    periods: Mutex<BTreeMap<u64, u64>>,
    fastest_ns: AtomicU64,
}

pub(crate) struct Subscription { subscriptions: Arc<Subscriptions>, id: u64 }

impl Subscriptions {
    pub(crate) fn register(self: &Arc<Self>, period: Duration) -> Subscription {
        let id = self.next_id.fetch_add(1, Ordering::Relaxed);
        let mut periods = self.periods.lock().expect("subscription registry poisoned");
        periods.insert(id, period.as_nanos() as u64);
        self.fastest_ns.store(*periods.values().min().expect("new subscription"), Ordering::Relaxed);
        Subscription { subscriptions: Arc::clone(self), id }
    }

    pub(crate) fn period(&self, control_period: Duration) -> Duration {
        let ns = self.fastest_ns.load(Ordering::Relaxed);
        if ns == 0 { control_period } else { Duration::from_nanos(ns).max(control_period) }
    }
}

impl Drop for Subscription {
    fn drop(&mut self) {
        let mut periods = self.subscriptions.periods.lock().expect("subscription registry poisoned");
        periods.remove(&self.id);
        self.subscriptions.fastest_ns.store(periods.values().copied().min().unwrap_or(0), Ordering::Relaxed);
    }
}

/// Starts each status scan at the requested subscription rate and finishes it across ticks.
#[derive(Default)]
pub(crate) struct DeviceSchedule {
    round: Option<(Instant, u64, bool)>,
    configuration_generation: u64,
}

impl DeviceSchedule {
    pub(crate) fn reset(&mut self) { self.round = None; }

    pub(crate) fn refresh_configuration(&mut self, generation: u64) -> bool {
        if self.configuration_generation == generation { return false; }
        self.configuration_generation = generation;
        true
    }

    pub(crate) fn due(&mut self, now: Instant, now_ns: u64, interval: Duration) -> bool {
        match self.round {
            Some((_, _, false)) => true,
            Some((started, _, true)) if now.saturating_duration_since(started) < interval => false,
            _ => {
                self.round = Some((now, now_ns, false));
                true
            }
        }
    }

    pub(crate) fn observe(&mut self, observation: &proto::DeviceObservation) {
        let Some((_, since_ns, complete)) = &mut self.round else { return };
        let attempted = |time: Option<u64>| time.is_some_and(|time| time >= *since_ns);
        *complete = !observation.servos.is_empty()
            && observation.servos.iter().all(|servo| attempted(servo.observed_t_ns))
            && attempted(observation.imu.observed_t_ns);
    }
}

pub(crate) struct Snapshot {
    pub captured_at: Instant,
    pub t: f64,
    pub t_ns: u64,
    pub sensors: Sensors,
    pub targets: [f64; NUM_JOINTS],
    pub requested: [f64; 3],
    pub applied: [f64; 3],
    pub head: [f64; 4],
    pub limited: [bool; 3],
    pub policy: Arc<str>,
    pub tracking_phase: Option<proto::CoordinatedTrackingPhase>,
    pub safety: proto::SafetyState,
    pub control_loop: proto::LoopState,
    pub odom: proto::OdomState,
    pub publish_velocity: bool,
    pub publish_load: bool,
    pub theremin: Option<proto::ThereminState>,
    pub chorale: Option<proto::ChoraleState>,
}

impl Snapshot {
    fn into_frame(self) -> proto::RobotState {
        proto::RobotState {
            t: self.t, t_ns: self.t_ns,
            movement: proto::MoveState {
                requested: self.requested, applied: self.applied,
                limited_by: self.limited.into_iter().zip([
                    duck_control::safety::Limit::Deadman, duck_control::safety::Limit::Range,
                    duck_control::safety::Limit::NotFinite])
                    .filter(|(active, _)| *active).map(|(_, limit)| crate::limit_name(limit).to_owned()).collect(),
            },
            head: self.head, policy: self.policy.to_string(), tracking_phase: self.tracking_phase,
            safety: self.safety, control_loop: self.control_loop,
            joints: self.sensors.positions.to_vec(), targets: self.targets.to_vec(),
            velocities: if self.publish_velocity { self.sensors.velocities.to_vec() } else { Vec::new() },
            currents_ma: if self.publish_load { self.sensors.currents_ma.to_vec() } else { Vec::new() },
            servo_status: self.sensors.servo_status.map_or_else(Vec::new, |status| status.to_vec()),
            odom: self.odom, theremin: self.theremin, chorale: self.chorale,
            imu: Some(proto::ImuState { gyro: self.sensors.imu.gyro, quat: self.sensors.imu.quat }),
            frames: Some(mapping::frames_at(mapping::head_joints_of(&self.sensors.positions))),
            skeleton: mapping::skeleton_at(&self.sensors.positions),
        }
    }
}

pub(crate) struct Publisher {
    latest: Arc<Mutex<Option<Snapshot>>>,
    stop: Arc<AtomicBool>,
    skipped: Arc<AtomicU64>,
    worker: Option<JoinHandle<()>>,
    label: Arc<str>,
}

impl Publisher {
    pub(crate) fn start(state: Arc<RobotState>, control_period: Duration) -> std::io::Result<Self> {
        let latest = Arc::new(Mutex::new(None::<Snapshot>));
        let stop = Arc::new(AtomicBool::new(false));
        let skipped = Arc::new(AtomicU64::new(0));
        let worker_latest = Arc::clone(&latest);
        let worker_stop = Arc::clone(&stop);
        let worker_skipped = Arc::clone(&skipped);
        let worker = thread::Builder::new().name("state-output".into()).spawn(move || {
            let mut next_temperature = Instant::now();
            let mut next_frame = Instant::now();
            while !worker_stop.load(Ordering::Acquire) {
                let now = Instant::now();
                if now >= next_temperature {
                    if let Some(celsius) = soc::hottest_zone_c() {
                        state.cpu_temp_c.store(celsius.to_bits(), Ordering::Relaxed);
                    }
                    if let Some(throttle) = soc::cpu_throttle() {
                        state.cpu_throttle.store(Some(Arc::new(throttle)));
                    }
                    let skipped = worker_skipped.swap(0, Ordering::Relaxed);
                    if skipped > 0 { tracing::warn!(skipped, "state snapshots skipped: output slot busy"); }
                    next_temperature = Instant::now() + Duration::from_secs(1);
                }
                if state.state_tx.receiver_count() == 0 {
                    worker_latest.lock().expect("telemetry slot poisoned").take();
                    next_frame = Instant::now();
                    thread::park_timeout(next_temperature.saturating_duration_since(Instant::now()));
                    continue;
                }
                if Instant::now() >= next_frame {
                    // Drop the slot lock before geometry, logging or subscriber fan-out.
                    let snapshot = worker_latest.lock().expect("telemetry slot poisoned").take();
                    if let Some(snapshot) = snapshot {
                        if snapshot.captured_at.elapsed() <= control_period * 2 {
                            let _ = state.state_tx.send(snapshot.into_frame());
                        }
                        next_frame += state.subscriptions.period(control_period);
                        if next_frame <= Instant::now() { next_frame = Instant::now(); }
                    }
                }
                let until = next_frame.min(next_temperature).saturating_duration_since(Instant::now());
                // A due frame without a new snapshot waits for the producer's next unpark.
                thread::park_timeout(if until.is_zero() { control_period } else { until });
            }
        })?;
        Ok(Self { latest, stop, skipped, worker: Some(worker), label: Arc::from("held") })
    }

    pub(crate) fn label(&mut self, label: &str) -> Arc<str> {
        if self.label.as_ref() != label { self.label = Arc::from(label); }
        Arc::clone(&self.label)
    }

    pub(crate) fn publish(&self, snapshot: Snapshot) {
        match self.latest.try_lock() {
            Ok(mut slot) => *slot = Some(snapshot),
            Err(TryLockError::WouldBlock) => { self.skipped.fetch_add(1, Ordering::Relaxed); }
            Err(TryLockError::Poisoned(_)) => panic!("telemetry slot poisoned"),
        }
        self.worker.as_ref().expect("telemetry worker").thread().unpark();
    }
}

impl Drop for Publisher {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            worker.thread().unpark();
            if worker.join().is_err() { tracing::error!("state output worker panicked"); }
        }
    }
}
