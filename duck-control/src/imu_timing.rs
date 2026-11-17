//! Same-snapshot MCU evidence. Host/MCU clocks are never directly subtracted.

use crate::io::{IoError, Result};

pub const TIMING_ADDRESS: u8 = 120;
pub const TIMING_LENGTH: u8 = 160;
pub const STATS_ADDRESS: u8 = 121;
pub const STATS_LENGTH: u8 = 208;

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct McuTiming {
    pub fusion_sequence: u32,
    pub flags: u32,
    pub read_start_us: u32,
    pub read_end_us: u32,
    pub fusion_start_us: u32,
    pub fusion_end_us: u32,
    pub published_us: u32,
    pub snapshot_us: u32,
    pub sensor_clock_before: u32,
    pub sensor_clock_after: u32,
    pub sensor_status: u32,
    pub reset_flags: u32,
    pub calibration_generation: u32,
    pub published_age_us: u32,
    pub read_age_us: u32,
    pub read_to_publish_us: u32,
    pub host_request_start_ns: u64,
    pub host_reply_end_ns: u64,
    pub corrected_accel_sensor_g: [f32; 3],
    pub gyro_sensor_rad_s: [f32; 3],
    pub quaternion_sensor_wxyz: [f32; 4],
}

impl McuTiming {
    /// Bounds measured from MCU read start, excluding sensor-internal latency.
    pub fn software_age_bounds_us(self, consumed_ns: u64) -> [u64; 2] {
        [self.host_reply_end_ns, self.host_request_start_ns]
            .map(|t| u64::from(self.read_age_us) + consumed_ns.saturating_sub(t) / 1000)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TimedImu {
    pub recording_block: [u8; 28],
    pub timing: McuTiming,
}

fn invalid() -> IoError {
    IoError::Bus("invalid shared-bus IMU same-snapshot timing packet; matching firmware required".into())
}

pub fn decode_timing(data: &[u8], request_ns: u64, reply_ns: u64) -> Result<TimedImu> {
    if data.len() != usize::from(TIMING_LENGTH)
        || &data[..4] != b"IMUT"
        || data[4..8] != [1, 0, TIMING_LENGTH, 0]
        || data[140..].iter().any(|v| *v != 0)
        || reply_ns < request_ns
    {
        return Err(invalid());
    }
    let word = |i: usize| u32::from_le_bytes(data[i..i + 4].try_into().unwrap());
    let vector = |offset: usize| {
        std::array::from_fn(|i| {
            f32::from_le_bytes(data[offset + 4 * i..offset + 4 * i + 4].try_into().unwrap())
        })
    };
    let timing = McuTiming {
        fusion_sequence: word(12),
        flags: word(8),
        read_start_us: word(16),
        read_end_us: word(20),
        fusion_start_us: word(24),
        fusion_end_us: word(28),
        published_us: word(32),
        snapshot_us: word(36),
        sensor_clock_before: word(40),
        sensor_clock_after: word(44),
        sensor_status: word(48),
        reset_flags: word(52),
        calibration_generation: word(124),
        published_age_us: word(128),
        read_age_us: word(132),
        read_to_publish_us: word(136),
        host_request_start_ns: request_ns,
        host_reply_end_ns: reply_ns,
        corrected_accel_sensor_g: vector(84),
        gyro_sensor_rad_s: vector(96),
        quaternion_sensor_wxyz: std::array::from_fn(|i| {
            f32::from_le_bytes(data[108 + 4 * i..112 + 4 * i].try_into().unwrap())
        }),
    };
    let t = timing;
    let durations = [
        t.read_end_us.wrapping_sub(t.read_start_us),
        t.published_us.wrapping_sub(t.read_end_us),
        t.fusion_end_us.wrapping_sub(t.fusion_start_us),
        t.published_us.wrapping_sub(t.fusion_end_us),
        t.snapshot_us.wrapping_sub(t.published_us),
        // Hardware-fused FIFO arrival can occur inside the bus read interval.
        t.fusion_start_us.wrapping_sub(t.read_start_us),
    ];
    let quaternion_norm: f32 = t.quaternion_sensor_wxyz.iter().map(|v| v * v).sum();
    let raw_accel: [f32; 3] = vector(56);
    let full_scale = data[68];
    let gyro_range = u16::from_le_bytes([data[70], data[71]]);
    if t.flags & !31 != 0
        || t.flags & 7 != 5
        || t.sensor_clock_before > 0xffffff
        || t.sensor_clock_after > 0xffffff
        || t.sensor_status > 255
        || durations.iter().any(|d| *d > 100_000)
        || t.snapshot_us.wrapping_sub(t.read_start_us) != t.read_age_us
        || durations[4] != t.published_age_us
        || t.published_us.wrapping_sub(t.read_start_us) != t.read_to_publish_us
        || t.read_age_us > 100_000
        || ![2, 4, 8, 16].contains(&full_scale)
        || data[69] & 0xc0 != 0
        || !(1..=1146).contains(&gyro_range)
        || raw_accel
            .iter()
            .any(|v| !v.is_finite() || v.abs() > f32::from(full_scale))
        || t.corrected_accel_sensor_g
            .iter()
            .chain(t.gyro_sensor_rad_s.iter())
            .any(|v| !v.is_finite())
        || !quaternion_norm.is_finite()
        || (quaternion_norm - 1.).abs() > 0.01
    {
        return Err(invalid());
    }
    Ok(TimedImu {
        recording_block: data[56..84].try_into().unwrap(),
        timing,
    })
}



/// Sensor-frame radians/second, with an explicit MCU fixed-bias capability.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GyroBiasState {
    pub supported: bool,
    pub ready: bool,
    pub loaded: bool,
    pub raw_rad_s: [f32; 3],
    pub bias_rad_s: [f32; 3],
}

pub fn decode_gyro_bias(data: &[u8]) -> Result<GyroBiasState> {
    decode_gyro_bias_state(data, false)
}

pub(crate) fn decode_gyro_bias_for_load(data: &[u8]) -> Result<GyroBiasState> {
    decode_gyro_bias_state(data, true)
}

fn decode_gyro_bias_state(data: &[u8], allow_unloaded_bias: bool) -> Result<GyroBiasState> {
    let invalid = || IoError::Bus("invalid MCU unified gyro bias contract".into());
    if data.len() != 48 || &data[..8] != b"IMUB\x02\x00\x30\x00" {
        return Err(invalid());
    }
    let word = |i| u32::from_le_bytes(data[i..i + 4].try_into().unwrap());
    let flags = word(8);
    let vector = |offset| std::array::from_fn(|i| {
        f32::from_le_bytes(data[offset + 4*i..offset + 4*i + 4].try_into().unwrap())
    });
    let state = GyroBiasState {
        supported: flags & 1 != 0,
        ready: flags & 4 != 0,
        loaded: flags & 2 != 0, raw_rad_s: vector(16), bias_rad_s: vector(28),
    };
    if flags & !7 != 0 || (!state.supported && state.loaded)
        || (state.supported && state.ready && !state.loaded)
        || &data[44..48] != b"QST3" || word(40) > 100000 || word(12) != 0
        || (!state.loaded && state.bias_rad_s != [0.0; 3]
            && !(allow_unloaded_bias && state.supported && !state.ready))
        || state.raw_rad_s.iter().chain(state.bias_rad_s.iter()).any(|v| !v.is_finite())
        || state.bias_rad_s.iter().map(|v| v*v).sum::<f32>().sqrt() > 0.30 {
        return Err(invalid());
    }
    Ok(state)
}
