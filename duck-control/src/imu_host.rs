//! Timestamped raw IMU batches and receive-side gravity estimation.
//!
//! A 50 Hz read consumes every new 100 Hz sample in MCU timestamp order. This is
//! mathematically the same recurrence as processing those samples individually;
//! it does not reproduce an independently scheduled host receive callback.

use crate::imu::{ImuData, rotate};
use crate::io::{AccelerationCalibration, AccelerationSample, IoError, Result};

pub const RAW_ADDRESS: u8 = 125;
pub const RAW_LENGTH: u8 = 104;
const MAX_GAP_US: u32 = 100_000;
const MAX_SAMPLE_AGE_US: u32 = 30_000;
const READY_SAMPLES: u32 = 25;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub struct RawImuSample {
    pub sequence: u32,
    pub sample_us: u32,
    pub acceleration_sensor_counts: [i16; 3],
    pub gyro_sensor_counts: [i16; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ProcessedImuSample {
    pub raw: RawImuSample,
    pub dt_s: f64,
    /// Specific force, before converting its sign to the gravity observation.
    pub acceleration_trunk_g: [f64; 3],
    pub gyro_trunk_rad_s: [f64; 3],
    pub gravity_trunk: [f64; 3],
    pub gyro_ema_rad_s: [f64; 3],
    pub acceleration_trust: f64,
    pub accel_saturated_axes: [bool; 3],
    pub gyro_saturated_axes: [bool; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct HostImuTiming {
    pub snapshot_us: u32,
    pub newest_sequence: u32,
    pub newest_sample_us: u32,
    pub snapshot_sample_age_us: u32,
    pub profile: u8,
    pub ctrl2: u8,
    pub ctrl3: u8,
    pub ctrl5: u8,
    pub full_scale_g: u8,
    pub gyro_full_scale_dps: u16,
    pub host_request_start_ns: u64,
    pub host_reply_end_ns: u64,
    pub input_count: u8,
    pub processed_count: u8,
    pub input_samples: [Option<RawImuSample>; 4],
    pub processed_samples: [Option<ProcessedImuSample>; 4],
    pub continuous_samples: u32,
}

impl HostImuTiming {
    /// Bounds exclude the sensor's internal sampling and conversion latency.
    pub fn software_age_bounds_us(self, consumed_ns: u64) -> [u64; 2] {
        [self.host_reply_end_ns, self.host_request_start_ns].map(|host_ns| {
            u64::from(self.snapshot_sample_age_us) + consumed_ns.saturating_sub(host_ns) / 1000
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HostImuOutput {
    pub imu: ImuData,
    pub acceleration: AccelerationSample,
    pub timing: HostImuTiming,
    /// Conservative host timestamp: the MCU snapshot occurs after request start.
    pub acquired_t_ns: u64,
}

#[derive(Debug, Clone, Copy)]
pub struct RawImuBatch {
    snapshot_us: u32,
    newest_sequence: u32,
    count: usize,
    samples: [Option<RawImuSample>; 4],
}

fn invalid(reason: &str) -> IoError {
    IoError::Bus(format!("ID200 host IMU: {reason}"))
}

pub fn decode_batch(data: &[u8]) -> Result<RawImuBatch> {
    if data.len() != usize::from(RAW_LENGTH) || &data[..4] != b"IMUH"
        || data[4..8] != [1, 0, RAW_LENGTH, 0]
    {
        return Err(invalid("raw batch contract mismatch; HOST firmware required"));
    }
    if data[17..21] != [1, 0x04, 0x64, 0x00] || data[22..24] != [0, 0] {
        return Err(invalid("expected ODR4, +/-2g, +/-1024dps, both chip lowpasses off"));
    }
    let count = usize::from(data[16]);
    if !(1..=4).contains(&count) || data[21] != 1 {
        return Err(invalid("raw samples unavailable or unhealthy"));
    }
    let word = |offset| u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
    let snapshot_us = word(8);
    let mut samples = [None; 4];
    for (slot, sample) in samples.iter_mut().enumerate().take(count) {
        let start = 24 + slot * 20;
        let vector = |offset| std::array::from_fn(|i| {
            i16::from_le_bytes(data[offset + 2*i..offset + 2*i + 2].try_into().unwrap())
        });
        *sample = Some(RawImuSample {
            sequence: word(start), sample_us: word(start + 4),
            acceleration_sensor_counts: vector(start + 8), gyro_sensor_counts: vector(start + 14),
        });
    }
    if data[24 + count*20..].iter().any(|value| *value != 0) {
        return Err(invalid("unused raw slots must be zero"));
    }
    for pair in samples[..count].windows(2) {
        let (before, after) = (pair[0].unwrap(), pair[1].unwrap());
        let elapsed = after.sample_us.wrapping_sub(before.sample_us);
        if after.sequence.wrapping_sub(before.sequence) != 1 || elapsed == 0 || elapsed > MAX_GAP_US {
            return Err(invalid("raw batch sequence or timestamps discontinuous"));
        }
    }
    let newest = samples[count - 1].unwrap();
    if newest.sequence != word(12) || snapshot_us.wrapping_sub(newest.sample_us) > MAX_SAMPLE_AGE_US {
        return Err(invalid("raw batch newest sample is stale or mismatched"));
    }
    if samples[..count].iter().any(|sample|
        snapshot_us.wrapping_sub(sample.unwrap().sample_us) > MAX_GAP_US)
    {
        return Err(invalid("raw batch contains stale sample history"));
    }
    Ok(RawImuBatch { snapshot_us, newest_sequence: newest.sequence, count, samples })
}

#[derive(Default)]
pub struct HostImu {
    gravity: Option<[f64; 3]>,
    gyro_ema: [f64; 3],
    previous: Option<RawImuSample>,
    previous_reply_ns: Option<u64>,
    continuous_samples: u32,
}

impl HostImu {
    pub fn ready(&self) -> bool { self.continuous_samples >= READY_SAMPLES }

    /// Discard the startup probe's history before continuous control sampling begins.
    pub fn restart(&mut self) { *self = Self::default(); }

    /// Any discontinuity invalidates readiness and is returned to the normal I/O fault path.
    pub fn consume(&mut self, data: &[u8], request_ns: u64, reply_ns: u64) -> Result<HostImuOutput> {
        let result = self.consume_checked(data, request_ns, reply_ns);
        if result.is_err() { self.restart(); }
        result
    }

    fn consume_checked(&mut self, data: &[u8], request_ns: u64, reply_ns: u64) -> Result<HostImuOutput> {
        if reply_ns < request_ns || reply_ns - request_ns > u64::from(MAX_GAP_US)*1000
            || self.previous_reply_ns.is_some_and(|previous|
            reply_ns <= previous || reply_ns - previous > u64::from(MAX_GAP_US)*1000)
        {
            return Err(invalid("host receive clock gap; fusion reset and readiness required"));
        }
        let batch = decode_batch(data)?;
        let newest = batch.samples[batch.count - 1].unwrap();
        let first_new = if let Some(previous) = self.previous {
            let step = newest.sequence.wrapping_sub(previous.sequence);
            if step == 0 { return Err(invalid("no new raw sample")); }
            if step >= 1 << 31 { return Err(invalid("MCU sequence regressed or reset")); }
            if step > batch.count as u32 { return Err(invalid("raw ring overflow; samples lost")); }
            if batch.samples.iter().flatten().any(|sample|
                sample.sequence == previous.sequence && *sample != previous)
            {
                return Err(invalid("previous raw sample changed inside overlapping batch"));
            }
            batch.count - step as usize
        } else { 0 };
        let mut processed = [None; 4];
        let mut processed_count = 0;
        for sample in batch.samples[first_new..batch.count].iter().flatten() {
            let dt = if let Some(previous) = self.previous {
                let elapsed = sample.sample_us.wrapping_sub(previous.sample_us);
                if sample.sequence.wrapping_sub(previous.sequence) != 1 || elapsed == 0 || elapsed > MAX_GAP_US {
                    return Err(invalid("missing sample or MCU timestamp reset/gap"));
                }
                f64::from(elapsed) / 1e6
            } else { 0.01 };
            let acceleration_sensor_g = sample.acceleration_sensor_counts.map(|v| f64::from(v) / 16384.);
            let acceleration = rotate([0.5; 4], acceleration_sensor_g);
            // The chip reports specific force (+up). The filter observes gravity (-up).
            let measurement = acceleration.map(|v| -v);
            let norm = magnitude(measurement);
            let unit = (norm > 1e-6).then(|| measurement.map(|v| v / norm));
            let gyro = rotate([0.5; 4], sample.gyro_sensor_counts.map(|v|
                (f64::from(v) / 32.).to_radians()));
            let trust = (1. - (norm - 1.).abs() / 0.2).max(0.);
            let dt_used = dt.clamp(0.001, 0.1);
            let gravity = if let Some([gx, gy, gz]) = self.gravity {
                let [wx, wy, wz] = gyro;
                let predicted = normalize([
                    gx + (wz*gy - wy*gz)*dt_used,
                    gy + (wx*gz - wz*gx)*dt_used,
                    gz + (wy*gx - wx*gy)*dt_used,
                ])?;
                let gain = (1. - (-dt_used / 0.3).exp()) * trust;
                let retention = 0.5f64.powf(dt_used / 0.01);
                self.gyro_ema = std::array::from_fn(|i|
                    retention*self.gyro_ema[i] + (1. - retention)*gyro[i]);
                match unit {
                    Some(unit) => normalize(std::array::from_fn(|i|
                        (1. - gain)*predicted[i] + gain*unit[i]))?,
                    None => predicted,
                }
            } else {
                self.gyro_ema = gyro;
                unit.ok_or_else(|| invalid("zero acceleration cannot initialize gravity"))?
            };
            self.gravity = Some(gravity);
            self.previous = Some(*sample);
            self.continuous_samples = self.continuous_samples.saturating_add(1);
            processed[processed_count] = Some(ProcessedImuSample {
                raw: *sample, dt_s: dt, acceleration_trunk_g: acceleration,
                gyro_trunk_rad_s: gyro, gravity_trunk: gravity, gyro_ema_rad_s: self.gyro_ema,
                acceleration_trust: trust,
                accel_saturated_axes: sample.acceleration_sensor_counts.map(saturated),
                gyro_saturated_axes: sample.gyro_sensor_counts.map(saturated),
            });
            processed_count += 1;
        }
        let gravity = self.gravity.ok_or_else(|| invalid("empty raw fusion batch"))?;
        self.previous_reply_ns = Some(reply_ns);
        let age = batch.snapshot_us.wrapping_sub(newest.sample_us);
        let raw_acc = newest.acceleration_sensor_counts.map(|v| f64::from(v) / 16384.);
        let trunk_acc = rotate([0.5; 4], raw_acc);
        let timing = HostImuTiming {
            snapshot_us: batch.snapshot_us, newest_sequence: batch.newest_sequence,
            newest_sample_us: newest.sample_us, snapshot_sample_age_us: age,
            profile: 1, ctrl2: 0x04, ctrl3: 0x64, ctrl5: 0,
            full_scale_g: 2, gyro_full_scale_dps: 1024,
            host_request_start_ns: request_ns, host_reply_end_ns: reply_ns,
            input_count: batch.count as u8, processed_count: processed_count as u8,
            input_samples: batch.samples, processed_samples: processed,
            continuous_samples: self.continuous_samples,
        };
        Ok(HostImuOutput {
            imu: ImuData { gyro: self.gyro_ema, gyro_unfiltered: self.gyro_ema,
                           gravity, quat: gravity_quaternion(gravity) },
            acceleration: AccelerationSample {
                raw_sensor_g: raw_acc, corrected_sensor_g: raw_acc,
                raw_trunk_g: trunk_acc, corrected_trunk_g: trunk_acc,
                full_scale_g: 2, gyro_full_scale_dps: 1024,
                accel_saturated_axes: newest.acceleration_sensor_counts.map(saturated),
                gyro_saturated_axes: newest.gyro_sensor_counts.map(saturated),
                calibration: AccelerationCalibration { flags: 0, generation: 0, bias_g: [0.; 3], scale: [1.; 3] },
            },
            timing, acquired_t_ns: request_ns.saturating_sub(u64::from(age)*1000),
        })
    }
}

fn saturated(value: i16) -> bool { value == i16::MIN || value == i16::MAX }
fn magnitude(vector: [f64; 3]) -> f64 { vector.iter().map(|v| v*v).sum::<f64>().sqrt() }
fn normalize(vector: [f64; 3]) -> Result<[f64; 3]> {
    let norm = magnitude(vector);
    if !norm.is_finite() || norm < 1e-12 { return Err(invalid("nonfinite gravity recurrence")); }
    Ok(vector.map(|v| v / norm))
}

/// Roll/pitch representative with zero yaw; yaw is not observable from gravity.
fn gravity_quaternion(gravity: [f64; 3]) -> [f64; 4] {
    let roll = (-gravity[1]).atan2(-gravity[2]);
    let pitch = gravity[0].atan2(gravity[1].hypot(gravity[2]));
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    [cr*cp, sr*cp, cr*sp, -sr*sp]
}
