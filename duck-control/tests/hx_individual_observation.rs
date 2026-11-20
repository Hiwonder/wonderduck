#![cfg(target_os = "linux")]

use std::ffi::CStr;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::FromRawFd;
use std::sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use duck_control::hx::{HxIo, ImuSource};
use duck_control::io::RobotIo;
use duck_control::model::JOINT_IDS;

#[derive(Clone, Copy, PartialEq)]
enum RawReply { Normal, TimeoutOnce, Error, InvalidPayload, InvalidFrame }

struct BusState { missing_servo: Option<u8>, raw_reply: RawReply, raw_requests: usize }
struct SimBus { io: HxIo, state: Arc<Mutex<BusState>>, running: Arc<AtomicBool>, worker: Option<JoinHandle<()>> }

fn packet(id: u8, error: u8, payload: &[u8]) -> Vec<u8> {
    let mut reply = vec![0xff, 0xff, id, payload.len() as u8 + 2, error];
    reply.extend_from_slice(payload);
    reply.push(!reply[2..].iter().fold(0u8, |sum, value| sum.wrapping_add(*value)));
    reply
}

fn raw_accel() -> Vec<u8> {
    let mut data = vec![0; 48];
    data[..4].copy_from_slice(&[1, 1, 5, 0]);
    data[4..8].copy_from_slice(&42u32.to_le_bytes());
    data[8..12].copy_from_slice(&1_000u32.to_le_bytes());
    for (index, value) in [0.2f32, -0.1, 1.0, 0.1, -0.2, 1.0, 0.01, 0.02, 0.03].iter().enumerate() {
        data[12 + 4 * index..16 + 4 * index].copy_from_slice(&value.to_le_bytes());
    }
    data
}

fn raw_gyro() -> Vec<u8> {
    let mut data = vec![0; 48];
    data[..8].copy_from_slice(b"IMUB\x02\x00\x30\x00");
    data[8..12].copy_from_slice(&7u32.to_le_bytes());
    for (index, value) in [0.04f32, -0.05, 0.06].iter().enumerate() {
        data[16 + 4 * index..20 + 4 * index].copy_from_slice(&value.to_le_bytes());
    }
    data[40..44].copy_from_slice(&1_000u32.to_le_bytes());
    data[44..48].copy_from_slice(b"QST3");
    data
}

fn respond(state: &mut BusState, request: &[u8]) -> Vec<u8> {
    assert_eq!(request[4], 0x02, "status scanning must use individual reads");
    let (id, address, length) = (request[2], request[5], request[6]);
    if id == 200 {
        return match (address, length) {
            (56, 12) => {
                let mut data = [0; 12];
                data[6..].copy_from_slice(&[0x00, 0x80, 0x91, 0xbe, 0xde, 0x4f]);
                packet(id, 0, &data)
            }
            (68, 48) => {
                state.raw_requests += 1;
                match state.raw_reply {
                    RawReply::TimeoutOnce if state.raw_requests == 1 => Vec::new(),
                    RawReply::Error => packet(id, 0x08, &[]),
                    RawReply::InvalidPayload => packet(id, 0, &[0; 48]),
                    RawReply::InvalidFrame => {
                        let mut invalid = packet(id, 0, &raw_accel());
                        *invalid.last_mut().unwrap() ^= 1;
                        invalid
                    }
                    _ => packet(id, 0, &raw_accel()),
                }
            }
            (122, 48) => packet(id, 0, &raw_gyro()),
            _ => panic!("unexpected IMU register {address} length {length}"),
        };
    }
    assert_eq!((address, length), (0, 71));
    if state.missing_servo == Some(id) { return Vec::new(); }
    let mut data = [0; 71];
    data[5] = id;
    data[56..58].copy_from_slice(&2000u16.to_le_bytes());
    data[62] = 78;
    packet(id, 0, &data)
}

impl SimBus {
    fn new(missing_servo: Option<u8>, raw_reply: RawReply) -> Self {
        let (mut master, mut slave) = (-1, -1);
        assert_eq!(unsafe { libc::openpty(&mut master, &mut slave, std::ptr::null_mut(),
            std::ptr::null(), std::ptr::null()) }, 0);
        let mut name = [0i8; 128];
        assert_eq!(unsafe { libc::ttyname_r(slave, name.as_mut_ptr(), name.len()) }, 0);
        let path = unsafe { CStr::from_ptr(name.as_ptr()) }.to_str().unwrap();
        let mut io = HxIo::open(path).unwrap();
        unsafe { libc::close(slave); }
        io.set_imu_source(ImuSource::Id200);
        io.begin_device_observation(1);
        io.set_individual_device_observation(true);
        assert_eq!(unsafe { libc::fcntl(master, libc::F_SETFL, libc::O_NONBLOCK) }, 0);
        let mut master = unsafe { File::from_raw_fd(master) };
        let state = Arc::new(Mutex::new(BusState { missing_servo, raw_reply, raw_requests: 0 }));
        let running = Arc::new(AtomicBool::new(true));
        let worker_state = Arc::clone(&state);
        let worker_running = Arc::clone(&running);
        let worker = thread::spawn(move || {
            let mut pending = Vec::new();
            let mut chunk = [0; 128];
            while worker_running.load(Ordering::Relaxed) {
                match master.read(&mut chunk) {
                    Ok(n) => pending.extend_from_slice(&chunk[..n]),
                    Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => (),
                    Err(error) => panic!("simulated serial read: {error}"),
                }
                while pending.len() >= 4 {
                    let size = usize::from(pending[3]) + 4;
                    if pending.len() < size { break; }
                    let request = pending.drain(..size).collect::<Vec<_>>();
                    let response = respond(&mut worker_state.lock().unwrap(), &request);
                    if !response.is_empty() { master.write_all(&response).unwrap(); }
                }
                thread::sleep(Duration::from_millis(1));
            }
        });
        Self { io, state, running, worker: Some(worker) }
    }

    fn scan(&mut self) -> duck_control::io::Result<duck_control::io::DeviceObservation> {
        let mut latest = None;
        for _ in 0..=JOINT_IDS.len() {
            latest = self.io.sample_individual_device_observation(Instant::now() + Duration::from_secs(2))?;
        }
        Ok(latest.unwrap())
    }
}

impl Drop for SimBus {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        self.worker.take().unwrap().join().unwrap();
    }
}

#[test]
fn missing_servo_does_not_hide_other_servos_or_raw_imu() {
    let mut bus = SimBus::new(Some(34), RawReply::Normal);
    let data = bus.scan().unwrap();
    assert!(data.servos.iter().filter(|servo| servo.id != 34).all(|servo| servo.valid));
    assert!(!data.servos.iter().find(|servo| servo.id == 34).unwrap().valid);
    assert_eq!(data.servos.iter().find(|servo| servo.id == 34).unwrap().error.as_deref(), Some("读取超时"));
    assert!(data.imu.valid);
    let raw = data.imu.raw_sample.unwrap();
    assert_eq!(raw.sequence, 42);
    assert!((raw.raw_g[0] - 0.2).abs() < 0.0001);
    assert!((raw.raw_gyro_rad_s[1] + 0.05).abs() < 0.0001);
    assert!(raw.ready && raw.accel_calibrated && raw.gyro_calibrated);
    let second = bus.scan().unwrap();
    assert_eq!(second.servos.iter().find(|servo| servo.id == 34).unwrap().error.as_deref(), Some("未连接"));
    bus.state.lock().unwrap().missing_servo = None;
    let reconnected = bus.scan().unwrap();
    assert!(reconnected.servos.iter().all(|servo| servo.valid && servo.error.is_none()));
}

#[test]
fn optional_raw_timeout_keeps_other_devices_and_is_retried() {
    let mut bus = SimBus::new(None, RawReply::TimeoutOnce);
    let first = bus.scan().unwrap();
    assert!(first.imu.valid && first.imu.raw_sample.is_none());
    let second = bus.scan().unwrap();
    assert!(second.servos.iter().all(|servo| servo.valid) && second.imu.valid);
    assert_eq!(bus.state.lock().unwrap().raw_requests, 1);
    thread::sleep(Duration::from_millis(5100));
    assert!(bus.scan().unwrap().imu.raw_sample.is_some());
    assert_eq!(bus.state.lock().unwrap().raw_requests, 2);
}

#[test]
fn unsupported_raw_register_or_schema_does_not_hide_other_devices() {
    for reply in [RawReply::Error, RawReply::InvalidPayload] {
        let mut bus = SimBus::new(None, reply);
        let first = bus.scan().unwrap();
        assert!(first.imu.valid && first.imu.raw_sample.is_none());
        let second = bus.scan().unwrap();
        assert!(second.servos.iter().all(|servo| servo.valid) && second.imu.valid);
        assert_eq!(bus.state.lock().unwrap().raw_requests, 1);
    }
}

#[test]
fn corrupt_raw_frame_is_not_accepted_as_a_servo_reply() {
    let mut bus = SimBus::new(None, RawReply::InvalidFrame);
    assert!(bus.scan().is_err());
    assert!(bus.io.sample_individual_device_observation(Instant::now() + Duration::from_secs(2)).is_err());
}
