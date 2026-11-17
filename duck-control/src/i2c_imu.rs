//! Six-axis module fusion over Linux I2C, sampled independently of the servo bus.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::imu::{ImuData, median3_each, mul, rotate, rotate_inverse};
use crate::io::{I2cReadTiming, ImuStale, IoError, Result};

pub const BUS_PATH: &str = "/dev/i2c-5";
pub const ADDRESS: u16 = 0x50;
// Apply the 180-degree sensor-Y rotation before sensor-to-trunk mounting:
// mount_id200 * rot_sensor_y(pi), giving [-sensor_z, -sensor_x, sensor_y].
pub const MOUNT: [f64; 4] = [0.5, 0.5, -0.5, -0.5];
const PERIOD: Duration = Duration::from_millis(10);
const MAX_AGE: Duration = Duration::from_millis(50);
const WARMUP_SAMPLES: u32 = 25;
const RETRY_DELAY: Duration = Duration::from_millis(500);
const GYRO_SCALE: f64 = 2000.0 / 32768.0 * std::f64::consts::PI / 180.0;
const COHERENCE_READS: usize = 4;
const READY_MARGIN: Duration = Duration::from_millis(1);

// These are word registers. Bit 7 is not an I2C configuration prefix.
const CLOCK: u8 = 0x32;
const ACC_GYRO: u8 = 0x34;
const QUATERNION: u8 = 0x51;
const KEY: u8 = 0x69;
const PROFILE: [(u8, u16); 6] = [
    (0x04, 6),     // 115200 baud permits 100 Hz automatic reporting.
    (0x02, 0x207), // Time, accelerometer, gyro and quaternion only.
    (0x03, 9),     // 100 Hz.
    (0x1f, 2),     // 98 Hz bandwidth; restore the runtime setting after power loss.
    (0x24, 1),     // Six-axis fusion; zero selects nine-axis fusion.
    (0x2d, 1),     // Automatic reporting enables the live data table.
];

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub imu: ImuData,
    /// Acceleration in the trunk frame, in g, without host calibration.
    pub accel_g: [f64; 3],
    pub sequence: u64,
    pub acquired_at: Instant,
    pub acquired_t_ns: u64,
    pub read_timing: Option<I2cReadTiming>,
}

struct RawSample {
    clock: [u8; 4],
    acc_gyro: [u8; 12],
    quat: [u8; 8],
    read_timing: Option<I2cReadTiming>,
}

/// Retimes an existing poll to finish just before the observed UART read ends.
/// Only the sampler waits; the control thread never waits for an I2C transaction.
#[derive(Default)]
struct PollSchedule {
    control_started: Option<Instant>,
    control_duration: Option<Duration>,
    poll_duration: Option<Duration>,
    requested_ready: Option<(Instant, u64)>,
    announced: bool,
}

impl PollSchedule {
    fn begin_control(&mut self, now: Instant, now_ns: u64) -> bool {
        self.control_started = Some(now);
        if let (Some(control), Some(_)) = (self.control_duration, self.poll_duration) {
            self.requested_ready = Some((now + control, now_ns + control.as_nanos() as u64));
            let first = !self.announced;
            self.announced = true;
            first
        } else {
            false
        }
    }

    fn end_control(&mut self, now: Instant) {
        if let Some(started) = self.control_started.take() {
            Self::update_duration(
                &mut self.control_duration,
                now.saturating_duration_since(started),
            );
        }
    }

    fn update_duration(estimate: &mut Option<Duration>, measured: Duration) {
        *estimate = Some(estimate.map_or(measured, |old| (old * 3 + measured) / 4));
    }

    fn next_poll(&mut self, now: Instant, regular: Instant) -> (Instant, Option<u64>) {
        if let Some((ready, stamp)) = self.requested_ready {
            if now.saturating_duration_since(ready) > MAX_AGE {
                self.requested_ready = None;
            } else if let Some(duration) = self.poll_duration {
                let start = ready.checked_sub(duration + READY_MARGIN).unwrap_or(now);
                if now >= start {
                    self.requested_ready = None;
                    return (now, Some(stamp));
                }
                return (start, None);
            }
        }
        (regular.max(now), None)
    }
}

/// A live word can change between its low and high bytes. Accept two equal
/// consecutive blocks, allowing a normal producer update to settle. This checks
/// byte consistency within one poll; it does not smooth across sample periods or
/// claim that separate vector/quaternion blocks have one physical sample time.
/// `None` rejects this poll while keeping the transport open. A changing data
/// table is not a disconnected bus and must not trigger its 500 ms reconnect.
#[cfg(any(target_os = "linux", test))]
fn read_consistent<const N: usize>(
    register: u8,
    mut read: impl FnMut() -> Result<[u8; N]>,
) -> Result<Option<[u8; N]>> {
    let mut previous = read()?;
    for _ in 1..COHERENCE_READS {
        let current = read()?;
        if current == previous {
            return Ok(Some(current));
        }
        previous = current;
    }
    tracing::warn!(
        register,
        attempts = COHERENCE_READS,
        "I2C IMU inconsistent block; poll rejected, retrying next sample period"
    );
    Ok(None)
}

#[derive(Default)]
struct Decoder {
    gyro_history: Option<[[f64; 3]; 2]>,
    gravity_history: Option<[[f64; 3]; 2]>,
}

impl Decoder {
    fn quaternion(bytes: &[u8; 8]) -> Result<[f64; 4]> {
        let sensor_quat = std::array::from_fn(|i| {
            f64::from(i16::from_le_bytes([bytes[i * 2], bytes[i * 2 + 1]])) / 32768.0
        });
        let norm = sensor_quat
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        if !(0.95..=1.05).contains(&norm) {
            return Err(IoError::Bus(format!(
                "I2C IMU invalid quaternion norm {norm:.6}"
            )));
        }
        Ok(sensor_quat.map(|value| value / norm))
    }

    fn decode(&mut self, raw: &RawSample) -> Result<(ImuData, [f64; 3])> {
        let word = |bytes: &[u8], index: usize| {
            f64::from(i16::from_le_bytes([bytes[index * 2], bytes[index * 2 + 1]]))
        };
        let sensor_quat = Self::quaternion(&raw.quat)?;
        let quat = mul(sensor_quat, [MOUNT[0], -MOUNT[1], -MOUNT[2], -MOUNT[3]]);
        let gravity = rotate_inverse(quat, [0.0, 0.0, -1.0]);
        let gyro = rotate(
            MOUNT,
            std::array::from_fn(|i| word(&raw.acc_gyro, i + 3) * GYRO_SCALE),
        );
        let accel_g = rotate(
            MOUNT,
            std::array::from_fn(|i| word(&raw.acc_gyro, i) * 16.0 / 32768.0),
        );
        let gyro_history = self.gyro_history.get_or_insert([gyro; 2]);
        let gravity_history = self.gravity_history.get_or_insert([gravity; 2]);
        let imu = ImuData {
            gyro: median3_each(gyro_history, gyro),
            gyro_unfiltered: gyro,
            gravity: median3_each(gravity_history, gravity),
            quat,
        };
        *gyro_history = [gyro_history[1], gyro];
        *gravity_history = [gravity_history[1], gravity];
        Ok((imu, accel_g))
    }
}

struct State {
    sample: Option<Sample>,
    clock: Option<[u8; 4]>,
    valid_samples: u32,
    stale: ImuStale,
    error: Option<String>,
}

impl Default for State {
    fn default() -> Self {
        Self {
            sample: None,
            clock: None,
            valid_samples: 0,
            stale: ImuStale::default(),
            error: Some("I2C IMU has not produced a sample".into()),
        }
    }
}

impl State {
    fn fail(&mut self, error: String) -> bool {
        self.valid_samples = 0;
        let changed = self.error.as_ref() != Some(&error);
        self.error = Some(error);
        changed
    }

    fn observe_poll(
        &mut self,
        raw: Option<&RawSample>,
        decoder: &mut Decoder,
        at: Instant,
        acquired_t_ns: u64,
    ) -> Result<bool> {
        match raw {
            Some(raw) => self.observe(raw, decoder, at, acquired_t_ns),
            // Do not refresh sequence, age, warmup, clock or filter history on a
            // rejected poll. The last verified sample still expires at MAX_AGE.
            None => Ok(false),
        }
    }

    fn observe(
        &mut self,
        raw: &RawSample,
        decoder: &mut Decoder,
        at: Instant,
        acquired_t_ns: u64,
    ) -> Result<bool> {
        if self.clock == Some(raw.clock) {
            Decoder::quaternion(&raw.quat)?;
            self.stale.total = self.stale.total.saturating_add(1);
            self.stale.run = self.stale.run.saturating_add(1);
            if self.sample_at(at).is_err() {
                self.valid_samples = 0;
            }
            return Ok(false);
        }
        let (imu, accel_g) = decoder.decode(raw)?;
        if self
            .sample
            .is_some_and(|sample| at.saturating_duration_since(sample.acquired_at) > MAX_AGE)
        {
            self.valid_samples = 0;
        }
        self.clock = Some(raw.clock);
        self.stale.run = 0;
        self.valid_samples = self.valid_samples.saturating_add(1);
        let sequence = self
            .sample
            .map_or(1, |sample| sample.sequence.saturating_add(1));
        self.sample = Some(Sample {
            imu,
            accel_g,
            sequence,
            acquired_at: at,
            acquired_t_ns,
            read_timing: raw.read_timing,
        });
        Ok(self.error.take().is_some())
    }

    fn sample_at(&self, now: Instant) -> Result<Sample> {
        if let Some(error) = &self.error {
            return Err(IoError::Bus(error.clone()));
        }
        let sample = self
            .sample
            .ok_or_else(|| IoError::Bus("I2C IMU has no valid sample".into()))?;
        let age = now.saturating_duration_since(sample.acquired_at);
        if age > MAX_AGE {
            return Err(IoError::Bus(format!(
                "I2C IMU sample stale; age_ms={:.2}",
                age.as_secs_f64() * 1000.0
            )));
        }
        Ok(sample)
    }

    fn ready_at(&self, now: Instant) -> bool {
        self.valid_samples >= WARMUP_SAMPLES && self.sample_at(now).is_ok()
    }
}

/// Owns one sampler thread. Construction does not block the control loop on I2C.
pub struct I2cImu {
    state: Arc<Mutex<State>>,
    schedule: Arc<Mutex<PollSchedule>>,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl I2cImu {
    pub fn open() -> Result<Self> {
        #[cfg(not(target_os = "linux"))]
        return Err(IoError::Bus("I2C IMU requires Linux i2c-dev".into()));

        #[cfg(target_os = "linux")]
        {
            let state = Arc::new(Mutex::new(State::default()));
            let stop = Arc::new(AtomicBool::new(false));
            let schedule = Arc::new(Mutex::new(PollSchedule::default()));
            let worker_state = Arc::clone(&state);
            let worker_stop = Arc::clone(&stop);
            let worker_schedule = Arc::clone(&schedule);
            let worker = thread::Builder::new()
                .name("imu-i2c".into())
                .spawn(move || sample_loop(worker_state, worker_stop, worker_schedule))
                .map_err(|error| IoError::Bus(format!("starting I2C IMU thread: {error}")))?;
            Ok(Self {
                state,
                schedule,
                stop,
                worker: Some(worker),
            })
        }
    }

    pub fn read_latest(&self) -> Result<ImuData> {
        self.sample().map(|sample| sample.imu)
    }

    pub fn sample(&self) -> Result<Sample> {
        self.state
            .lock()
            .expect("I2C IMU state poisoned")
            .sample_at(Instant::now())
    }

    pub(crate) fn begin_control_read(&self) {
        let first = self
            .schedule
            .lock()
            .expect("I2C schedule poisoned")
            .begin_control(Instant::now(), duck_ipc_proto::clock::monotonic_ns());
        if first {
            tracing::info!(
                ready_margin_us = READY_MARGIN.as_micros() as u64,
                "I2C sampler aligned to control read; gyro read last; control thread does not wait"
            );
        }
        if let Some(worker) = &self.worker {
            worker.thread().unpark();
        }
    }

    pub(crate) fn end_control_read(&self) {
        self.schedule
            .lock()
            .expect("I2C schedule poisoned")
            .end_control(Instant::now());
    }

    pub fn ready(&self) -> bool {
        self.state
            .lock()
            .expect("I2C IMU state poisoned")
            .ready_at(Instant::now())
    }

    pub fn stale(&self) -> ImuStale {
        self.state.lock().expect("I2C IMU state poisoned").stale
    }


}

impl Drop for I2cImu {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            worker.thread().unpark();
            if worker.join().is_err() {
                tracing::error!("I2C IMU sampler panicked");
            }
        }
    }
}

#[cfg(target_os = "linux")]
struct Device(i2cdev::linux::LinuxI2CDevice);

#[cfg(target_os = "linux")]
impl Device {
    fn open() -> Result<Self> {
        let mut device = Self(
            i2cdev::linux::LinuxI2CDevice::new(BUS_PATH, ADDRESS).map_err(|error| {
                IoError::Bus(format!(
                    "opening I2C IMU {BUS_PATH} at 0x{ADDRESS:02x}: {error}"
                ))
            })?,
        );
        if device.word(0x1a)? != ADDRESS || device.word(0x20)? != 3 || device.word(0x21)? != 3 {
            return Err(IoError::Bus("I2C IMU address/range mismatch; expected word protocol with +/-16 g and +/-2000 dps".into()));
        }
        for (register, expected) in PROFILE {
            if device.word(register)? != expected {
                device.write(KEY, 0xb588)?;
                thread::sleep(Duration::from_millis(20));
                device.write(register, expected)?;
                thread::sleep(Duration::from_millis(20));
            }
            let actual = device.word(register)?;
            if actual != expected {
                return Err(IoError::Bus(format!(
                    "I2C IMU config 0x{register:02x}: expected 0x{expected:04x}, got 0x{actual:04x}"
                )));
            }
        }
        tracing::info!(
            bus = BUS_PATH,
            address = ADDRESS,
            hz = 100,
            bandwidth_hz = 98,
            bandwidth_code = 2,
            axis6 = true,
            host_calibration = false,
            "I2C IMU runtime profile verified"
        );
        Ok(device)
    }

    fn read<const N: usize>(&mut self, register: u8) -> Result<[u8; N]> {
        use i2cdev::core::{I2CMessage, I2CTransfer};
        use i2cdev::linux::LinuxI2CMessage;
        let pointer = [register];
        let mut bytes = [0; N];
        let mut messages = [
            LinuxI2CMessage::write(&pointer),
            LinuxI2CMessage::read(&mut bytes),
        ];
        let count = self
            .0
            .transfer(&mut messages)
            .map_err(|error| IoError::Bus(format!("I2C IMU register 0x{register:02x}: {error}")))?;
        if count != 2 {
            return Err(IoError::ShortRead {
                what: "I2C IMU messages",
                expected: 2,
                got: count as usize,
            });
        }
        Ok(bytes)
    }

    fn word(&mut self, register: u8) -> Result<u16> {
        Ok(u16::from_le_bytes(self.read(register)?))
    }

    fn write(&mut self, register: u8, value: u16) -> Result<()> {
        use i2cdev::core::I2CDevice;
        let bytes = value.to_le_bytes();
        self.0
            .write(&[register, bytes[0], bytes[1]])
            .map_err(|error| IoError::Bus(format!("I2C IMU write 0x{register:02x}: {error}")))
    }

    fn sample(
        &mut self,
        started_ns: u64,
        requested_ready_t_ns: Option<u64>,
    ) -> Result<Option<RawSample>> {
        // Keep byte-coherence checks, but acquire the fast-changing gyro after
        // the quaternion so that completing the pose read cannot age the gyro.
        let clock = self.read(CLOCK)?;
        let Some(quat) = read_consistent(QUATERNION, || self.read(QUATERNION))? else {
            return Ok(None);
        };
        let quaternion_end_t_ns = duck_ipc_proto::clock::monotonic_ns();
        let Some(acc_gyro) = read_consistent(ACC_GYRO, || self.read(ACC_GYRO))? else {
            return Ok(None);
        };
        let gyro_end_t_ns = duck_ipc_proto::clock::monotonic_ns();
        Ok(Some(RawSample {
            clock,
            acc_gyro,
            quat,
            read_timing: Some(I2cReadTiming {
                poll_start_t_ns: started_ns,
                quaternion_end_t_ns,
                gyro_end_t_ns,
                poll_end_t_ns: duck_ipc_proto::clock::monotonic_ns(),
                requested_ready_t_ns,
            }),
        }))
    }
}

#[cfg(target_os = "linux")]
fn sample_loop(
    state: Arc<Mutex<State>>,
    stop: Arc<AtomicBool>,
    schedule: Arc<Mutex<PollSchedule>>,
) {
    while !stop.load(Ordering::Acquire) {
        let mut device = match Device::open() {
            Ok(device) => device,
            Err(error) => {
                report_failure(&state, error);
                wait_until(&stop, Instant::now() + RETRY_DELAY);
                continue;
            }
        };
        let mut decoder = Decoder::default();
        let mut regular = Instant::now();
        while !stop.load(Ordering::Acquire) {
            let requested_ready = loop {
                if stop.load(Ordering::Acquire) {
                    return;
                }
                let now = Instant::now();
                let (start, requested) = schedule
                    .lock()
                    .expect("I2C schedule poisoned")
                    .next_poll(now, regular);
                if start <= now {
                    break requested;
                }
                // A new control-cycle request can wake and retime this wait.
                thread::park_timeout(start.saturating_duration_since(now));
            };
            let started = Instant::now();
            let acquired_t_ns = duck_ipc_proto::clock::monotonic_ns();
            let raw = match device.sample(acquired_t_ns, requested_ready) {
                Ok(raw) => raw,
                Err(error) => {
                    report_failure(&state, error);
                    break;
                }
            };
            // An incomplete, rejected poll must not shorten the full-read estimate.
            if raw.is_some() {
                PollSchedule::update_duration(
                    &mut schedule
                        .lock()
                        .expect("I2C schedule poisoned")
                        .poll_duration,
                    started.elapsed(),
                );
            }
            let result = state.lock().expect("I2C IMU state poisoned").observe_poll(
                raw.as_ref(),
                &mut decoder,
                started,
                acquired_t_ns,
            );
            match result {
                Ok(true) => tracing::info!("I2C IMU samples available; waiting for warmup"),
                Ok(false) => {}
                Err(error) => report_failure(&state, error),
            }
            regular = started + PERIOD;
        }
        if !stop.load(Ordering::Acquire) {
            wait_until(&stop, Instant::now() + RETRY_DELAY);
        }
    }
}

/// Control-cycle wakeups must not bypass the reconnect backoff.
#[cfg(target_os = "linux")]
fn wait_until(stop: &AtomicBool, deadline: Instant) {
    while !stop.load(Ordering::Acquire) {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        thread::park_timeout(remaining);
    }
}

#[cfg(target_os = "linux")]
fn report_failure(state: &Mutex<State>, error: IoError) {
    let error = error.to_string();
    if state
        .lock()
        .expect("I2C IMU state poisoned")
        .fail(error.clone())
    {
        tracing::warn!(%error, "I2C IMU unavailable");
    }
}
