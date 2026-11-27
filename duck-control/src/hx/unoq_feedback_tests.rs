use super::*;
use std::ffi::CStr;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::FromRawFd;
use std::thread;

const IDS: [u8; 16] = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14, 200];

fn response(sequence: u32, age_us: u32) -> Vec<u8> {
    let mut metadata = sequence.to_le_bytes().to_vec();
    metadata.extend_from_slice(&age_us.to_le_bytes());
    let mut result = packet(UNOQ_FEEDBACK_METADATA_ID, 0, &metadata).unwrap();
    for id in IDS {
        result.extend(packet(id, 0, &[0; 12]).unwrap());
    }
    result
}

fn exercise_read(responses: Vec<Vec<u8>>, address: u8, length: u8) -> Vec<Result<Vec<StatusFrame>>> {
    let (mut master, mut slave) = (-1, -1);
    assert_eq!(unsafe { libc::openpty(&mut master, &mut slave, std::ptr::null_mut(),
                                      std::ptr::null(), std::ptr::null()) }, 0);
    let mut name = [0i8; 128];
    assert_eq!(unsafe { libc::ttyname_r(slave, name.as_mut_ptr(), name.len()) }, 0);
    let path = unsafe { CStr::from_ptr(name.as_ptr()) }.to_str().unwrap();
    let mut io = HxIo::open(path).unwrap();
    io.unoq_feedback = true;
    unsafe { libc::close(slave); }
    let mut master = unsafe { File::from_raw_fd(master) };
    let count = responses.len();
    let (done, wait) = std::sync::mpsc::channel();
    let worker = thread::spawn(move || {
        for reply in responses {
            let mut request = [0; 24];
            master.read_exact(&mut request).unwrap();
            assert_eq!(request[4], UNOQ_FEEDBACK_READ);
            assert_eq!(request[5..7], [address, length]);
            assert_eq!(request[2..].iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte)), 0xff);
            master.write_all(&reply).unwrap();
        }
        wait.recv().unwrap();
    });
    let mut results = Vec::new();
    for _ in 0..count {
        results.push(io.exchange_with_cycle_deadline(
            sync_read_request(&IDS, address, length).unwrap(),
            &IDS, length, Instant::now() + Duration::from_millis(20),
        ));
    }
    done.send(()).unwrap();
    worker.join().unwrap();
    results
}

fn exercise(responses: Vec<Vec<u8>>) -> Vec<Result<Vec<StatusFrame>>> {
    exercise_read(responses, PRESENT_POSITION, PRESENT_POSITION_LEN)
}

#[test]
fn recording_feedback_uses_cache_with_full_blocks() {
    let mut metadata = 1u32.to_le_bytes().to_vec();
    metadata.extend_from_slice(&8_000u32.to_le_bytes());
    let mut reply = packet(UNOQ_FEEDBACK_METADATA_ID, 0, &metadata).unwrap();
    for id in IDS {
        reply.extend(packet(id, 0, &[id; RECORD_FEEDBACK_LENGTH as usize]).unwrap());
    }
    let result = exercise_read(vec![reply], RECORD_FEEDBACK_ADDRESS, RECORD_FEEDBACK_LENGTH);
    for (frame, id) in result[0].as_ref().unwrap().iter().zip(IDS) {
        assert_eq!(frame.data, [id; RECORD_FEEDBACK_LENGTH as usize]);
    }
}

#[test]
fn cached_feedback_requires_fresh_metadata() {
    let accepted = exercise(vec![response(1, 8_000)]);
    assert_eq!(accepted[0].as_ref().unwrap().len(), IDS.len());
    let stale = exercise(vec![response(1, UNOQ_FEEDBACK_MAX_AGE_US + 1)]);
    assert!(stale[0].is_err());
    let duplicate = exercise(vec![response(1, 8_000), response(1, 25_000)]);
    assert!(duplicate[0].is_ok());
    assert!(duplicate[1].is_ok());
    let backwards = exercise(vec![response(2, 8_000), response(1, 8_000)]);
    assert!(backwards[0].is_ok());
    assert!(backwards[1].is_err());
    let missing = exercise(vec![IDS.into_iter().flat_map(|id| packet(id, 0, &[0; 12]).unwrap()).collect()]);
    assert!(missing[0].is_err());
}
