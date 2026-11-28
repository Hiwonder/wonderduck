#![cfg(target_os = "linux")]

use std::collections::HashMap;
use std::ffi::CStr;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::FromRawFd;
use std::sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}};
use std::thread;
use std::time::Duration;

use duck_control::hx::{HxIo, decode_position};
use duck_control::io::{JointTargets, RobotIo};

const IDS: [u8; 15] = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14];

struct Bus {
    registers: HashMap<u8, Vec<u8>>,
    requests: Vec<Vec<u8>>,
    missing: Option<u8>,
    failed_verification: Option<u8>,
}

fn status(id: u8, data: &[u8]) -> Vec<u8> {
    let mut bytes = vec![0xff, 0xff, id, data.len() as u8 + 2, 0];
    bytes.extend_from_slice(data);
    bytes.push(!bytes[2..].iter().fold(0u8, |sum, value| sum.wrapping_add(*value)));
    bytes
}

fn respond(bus: &mut Bus, frame: &[u8]) -> Vec<u8> {
    let id = frame[2];
    let instruction = frame[4];
    bus.requests.push(frame.to_vec());
    match instruction {
        0x02 => {
            let address = usize::from(frame[5]);
            let length = usize::from(frame[6]);
            if bus.missing == Some(id) || (bus.failed_verification == Some(id) && address == 21) {
                return Vec::new();
            }
            if id == 200 { return status(id, &vec![0; length]); }
            let Some(registers) = bus.registers.get(&id) else { return Vec::new() };
            status(id, &registers[address..address + length])
        }
        0x03 => {
            let address = usize::from(frame[5]);
            if let Some(registers) = bus.registers.get_mut(&id) {
                let value = &frame[6..frame.len() - 1];
                registers[address..address + value.len()].copy_from_slice(value);
                status(id, &[])
            } else { Vec::new() }
        }
        0x83 => {
            let address = usize::from(frame[5]);
            let width = usize::from(frame[6]);
            for item in frame[7..frame.len() - 1].chunks_exact(width + 1) {
                if let Some(registers) = bus.registers.get_mut(&item[0]) {
                    registers[address..address + width].copy_from_slice(&item[1..]);
                }
            }
            Vec::new()
        }
        0x82 => {
            let address = usize::from(frame[5]);
            let length = usize::from(frame[6]);
            let mut response = Vec::new();
            for id in &frame[7..frame.len() - 1] {
                if bus.missing == Some(*id) { continue; }
                if *id == 200 {
                    response.extend(status(*id, &vec![0; length]));
                } else if let Some(registers) = bus.registers.get(id) {
                    response.extend(status(*id, &registers[address..address + length]));
                }
            }
            response
        }
        _ => panic!("unexpected instruction {instruction:#x}"),
    }
}

fn run_startup(missing: Option<u8>, failed_verification: Option<u8>) -> (duck_control::io::Result<usize>, Bus) {
    run_bus(missing, failed_verification, HxIo::check_registers)
}

fn run_bus<T>(
    missing: Option<u8>,
    failed_verification: Option<u8>,
    run: impl FnOnce(&mut HxIo) -> T,
) -> (T, Bus) {
    run_bus_with_state(missing, failed_verification, |io, _| run(io))
}

fn run_bus_with_state<T>(
    missing: Option<u8>,
    failed_verification: Option<u8>,
    run: impl FnOnce(&mut HxIo, &Mutex<Bus>) -> T,
) -> (T, Bus) {
    let mut master = -1;
    let mut slave = -1;
    assert_eq!(unsafe { libc::openpty(&mut master, &mut slave, std::ptr::null_mut(),
        std::ptr::null(), std::ptr::null()) }, 0);
    let mut name = [0i8; 128];
    assert_eq!(unsafe { libc::ttyname_r(slave, name.as_mut_ptr(), name.len()) }, 0);
    let path = unsafe { CStr::from_ptr(name.as_ptr()) }.to_str().unwrap().to_owned();
    let mut io = HxIo::open(&path).expect("open simulated HX serial port");
    unsafe { libc::close(slave); }
    assert_eq!(unsafe { libc::fcntl(master, libc::F_SETFL, libc::O_NONBLOCK) }, 0);
    let mut master = unsafe { File::from_raw_fd(master) };

    let registers = IDS.into_iter().map(|id| {
        let mut block = vec![0; 71];
        block[5] = id;
        block[13] = 80;
        block[21..24].copy_from_slice(&[9, 7, 2]);
        block[24] = 15;
        block[41] = 1;
        block[44] = 23;
        block[46] = 5;
        block[55] = 1;
        block[62] = 79;
        (id, block)
    }).collect();
    let bus = Arc::new(Mutex::new(Bus { registers, requests: Vec::new(), missing, failed_verification }));
    let running = Arc::new(AtomicBool::new(true));
    let worker_bus = Arc::clone(&bus);
    let worker_running = Arc::clone(&running);
    let worker = thread::spawn(move || {
        let mut pending = Vec::new();
        let mut chunk = [0u8; 512];
        while worker_running.load(Ordering::Relaxed) {
            match master.read(&mut chunk) {
                Ok(size) => pending.extend_from_slice(&chunk[..size]),
                Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {},
                Err(error) => panic!("simulated bus read: {error}"),
            }
            while pending.len() >= 4 {
                if pending[0..2] != [0xff, 0xff] { pending.remove(0); continue; }
                let total = usize::from(pending[3]) + 4;
                if pending.len() < total { break; }
                let frame: Vec<u8> = pending.drain(..total).collect();
                let response = respond(&mut worker_bus.lock().unwrap(), &frame);
                if !response.is_empty() { master.write_all(&response).unwrap(); }
            }
            thread::sleep(Duration::from_millis(1));
        }
    });
    let result = run(&mut io, &bus);
    running.store(false, Ordering::Relaxed);
    worker.join().unwrap();
    drop(io);
    (result, Arc::try_unwrap(bus).ok().unwrap().into_inner().unwrap())
}

#[test]
fn startup_accessory_detection_confirms_either_id_without_writing() {
    for (devices, expected) in [
        (&[][..], None),
        (&[(35, 35)][..], Some(35)),
        (&[(36, 36)][..], Some(36)),
        (&[(35, 36), (36, 36)][..], Some(36)),
    ] {
        let (detected, bus) = run_bus_with_state(None, None, |io, bus| {
            {
                let mut bus = bus.lock().unwrap();
                for &(reply_id, register_id) in devices {
                    let mut registers = vec![0; 71];
                    registers[5] = register_id;
                    bus.registers.insert(reply_id, registers);
                }
            }
            io.detect_startup_accessory_servo()
        });
        assert_eq!(detected, expected);
        assert!(bus.requests.iter().all(|frame| frame[4..7] == [0x02, 5, 1]));
    }
}

fn assert_corrected(bus: &Bus, id: u8) {
    let block = &bus.registers[&id];
    assert_eq!(&block[21..24], &[15, 0, 0], "PID ID {id}");
    assert_eq!(block[41], 0, "acceleration ID {id}");
    assert_eq!(&block[44..48], &[0, 0, 0, 0], "motion timing ID {id}");
    assert_eq!(block[55], 1, "write protection ID {id}");
    if id != 34 {
        assert_eq!(block[13], 120, "temperature ID {id}");
        assert_eq!(&block[24..26], &[0, 0], "starting torque ID {id}");
    }
}

#[test]
fn failed_servo_does_not_hide_other_servo_settings() {
    let (result, bus) = run_startup(Some(21), Some(22));
    let error = result.expect_err("unverified motors must block control").to_string();
    assert!(error.contains("ID 21"), "{error}");
    assert!(error.contains("ID 22"), "{error}");
    for id in IDS.into_iter().filter(|id| ![21, 22].contains(id)) { assert_corrected(&bus, id); }
    assert_eq!(bus.registers[&22][55], 1, "restore protection after failed verification");
    assert!(!bus.requests.iter().any(|frame| frame[4] == 0x82), "no startup parameter sync read");
    assert!(!bus.requests.iter().any(|frame| frame[4] == 0x03 && [40, 42].contains(&frame[5])),
        "failed startup must not enable torque or command position");
}

#[test]
fn healthy_servos_are_configured_before_the_control_stream_probe() {
    let (result, bus) = run_startup(None, None);
    assert!(result.unwrap() > 0);
    for id in IDS { assert_corrected(&bus, id); }
    let sync_reads: Vec<_> = bus.requests.iter().filter(|frame| frame[4] == 0x82).collect();
    assert_eq!(sync_reads.len(), 1);
    assert_eq!((sync_reads[0][5], sync_reads[0][6]), (56, 12));
    assert_eq!(bus.requests.last().unwrap()[4], 0x82);
}

#[test]
fn early_startup_configuration_checks_each_joint_voltage_and_torque() {
    let (_, bus) = run_bus(Some(21), None, |io| {
        let mut remaining = 3;
        assert!(io.check_released_servo_at_startup(20, 8.0, &mut remaining).is_err());
        assert!(io.check_released_servo_at_startup(21, 7.2, &mut remaining).is_err());
        assert_eq!(remaining, 3);
        assert!(io.check_released_servo_at_startup(22, 7.2, &mut remaining).unwrap() > 0);
        assert_eq!(remaining, 2);
    });
    assert_eq!(&bus.registers[&20][21..24], &[9, 7, 2]);
    assert_eq!(&bus.registers[&21][21..24], &[9, 7, 2]);
    assert_corrected(&bus, 22);
    assert!(!bus.requests.iter().any(|frame| matches!(frame[4], 0x03 | 0x83) && [40, 42].contains(&frame[5])));
}

#[test]
fn recovered_startup_servo_is_verified_without_further_writes() {
    let (first_recheck, bus) = run_bus_with_state(Some(10), None, |io, bus| {
        let mut remaining = 1;
        for _ in 0..3 {
            assert!(io.check_released_servo_at_startup(10, 7.2, &mut remaining).is_err());
        }
        assert_eq!(remaining, 1);
        bus.lock().unwrap().missing = None;
        assert!(io.check_released_servo_at_startup(10, 7.2, &mut remaining).unwrap() > 0);
        assert_eq!(remaining, 0);
        let first_recheck = bus.lock().unwrap().requests.len();
        assert_eq!(io.check_released_servo_at_startup(10, 7.2, &mut remaining).unwrap(), 0);

        bus.lock().unwrap().registers.get_mut(&10).unwrap()[21] = 9;
        let error = io.check_released_servo_at_startup(10, 7.2, &mut remaining).unwrap_err();
        assert!(error.to_string().contains("寄存器 21"), "{error}");
        first_recheck
    });
    assert_eq!(bus.registers[&10][21], 9);
    assert!(bus.requests[first_recheck..].iter().all(|frame| frame[4] == 0x02));
}

#[test]
fn timed_frames_send_goal_and_duration_together_and_restore_policy_timing() {
    let (_, bus) = run_bus(None, None, |io| {
        io.set_torque(true).unwrap();
        let mut positions = [None; IDS.len()];
        positions[5] = Some(decode_position(5, &2691u16.to_le_bytes()).unwrap());
        positions[6] = Some(decode_position(6, &1336u16.to_le_bytes()).unwrap());
        assert!(io.write_timed_positions(&positions, &[Some(2040), Some(2051)], 4000).unwrap());
        positions.fill(None);
        positions[6] = Some(decode_position(6, &1820u16.to_le_bytes()).unwrap());
        assert!(io.write_timed_positions(&positions, &[None; 2], 2000).unwrap());
        io.set_torque(false).unwrap();
        let targets = JointTargets::new([0.0; IDS.len()]);
        io.write(&targets).unwrap();
        io.set_torque(true).unwrap();
        io.write(&targets).unwrap();
    });
    let writes: Vec<_> = bus.requests.iter().filter(|frame| frame[4] == 0x83).collect();
    assert_eq!(writes.len(), 4);
    assert_eq!(&writes[0][5..writes[0].len() - 1], &[
        40, 8, 30, 1, 0, 0x83, 0x0a, 0xa0, 0x0f, 0, 0,
        31, 1, 0, 0x38, 0x05, 0xa0, 0x0f, 0, 0,
        35, 1, 0, 0xf8, 0x07, 0xa0, 0x0f, 0, 0,
        36, 1, 0, 0x03, 0x08, 0xa0, 0x0f, 0, 0,
    ]);
    assert_eq!(&writes[1][5..writes[1].len() - 1], &[
        41, 7, 31, 0, 0x1c, 0x07, 0xd0, 0x07, 0, 0,
    ]);
    assert_eq!(&writes[2][5..writes[2].len() - 1], &[40, 1, 35, 0, 36, 0]);
    assert_eq!(bus.registers[&30][44..48], [0; 4]);
    assert_eq!(bus.registers[&31][44..48], [0; 4]);
}
