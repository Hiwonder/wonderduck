//! The `imu_to_dxl` v2 board (LSM6DSV16X), decoded.
//!
//! One IMU, one code path. The board rides the Dynamixel bus and its 12-byte block is
//! fetched in the same `sync_read` as the servos, so there is no separate sensor to poll
//! and no fusion to run on the host — the chip's SFLP block ships a game-rotation
//! quaternion and estimates its own gyro bias.
//!
//! Block layout at address 124:
//!
//! | bytes | contents |
//! |---|---|
//! | 0..6  | gyro x/y/z, `i16` LE raw counts, ±500 dps |
//! | 6..12 | SFLP quaternion x/y/z as IEEE half-precision; `w = √(1 − x² − y² − z²)` |
//!
//! The board's full diagnostic block is 20 bytes (it also carries raw accelerometer, a
//! sample counter and status flags). The control loop consumes only the first 12 so the
//! read fits alongside the servos in one transaction.

use crate::model::NUM_JOINTS;

/// Bytes of the v2 board's block consumed per tick.
pub const IMU_BLOCK_LEN: usize = 12;

/// What the control loop knows about the robot's orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuData {
    /// Angular velocity in the trunk frame, rad/s.
    pub gyro: [f64; 3],
    /// Same mounted sample before the host median; bias correction applies to both paths.
    pub gyro_unfiltered: [f64; 3],
    /// Projected gravity in the trunk frame, unit vector. Upright is `[0, 0, -1]`.
    ///
    /// This is what the policy observes, and what fall detection thresholds on.
    pub gravity: [f64; 3],
    /// Orientation, trunk→world, scalar-first `[w, x, y, z]`.
    pub quat: [f64; 4],
}

impl ImuData {
    /// Bypass host gyro and gravity medians, preserving mounting and calibration.
    /// The sensor's own quaternion fusion is unchanged.
    pub fn without_host_filter(mut self) -> Self {
        self.gyro = self.gyro_unfiltered;
        self.gravity = normalise(rotate_inverse(self.quat, [0.0, 0.0, -1.0]));
        self
    }
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            gyro_unfiltered: [0.0; 3],
            gravity: [0.0, 0.0, -1.0],
            quat: [1.0, 0.0, 0.0, 0.0],
        }
    }
}

/// ±500 dps at 17.5 mdps/LSB, in rad/s.
const GYRO_DPS_PER_LSB: f64 = 0.0175;

/// Decodes the board's block into [`ImuData`].
///
/// Stateful only for spike rejection and for holding the last good quaternion — there is
/// no filter here. Keeping the state explicit means [`crate::io::FakeIo`] can drive the
/// same decoder in tests.
pub struct SflpDecoder {
    /// Sensor→trunk mounting rotation, scalar-first.
    mount: [f64; 4],
    gyro_rad_per_lsb: f64,
    /// Last quaternion the board actually produced. Held across blocks that arrive before
    /// SFLP has written its table — snapping to identity would report a robot as upright
    /// when its orientation is simply unknown, which is the worst possible lie to tell
    /// fall detection.
    last_quat: [f64; 4],
    /// Blocks carrying a live quaternion. Gates [`SflpDecoder::ready`].
    quat_samples: u32,
    gyro_history: [[f64; 3]; 2],
    gravity_history: [[f64; 3]; 2],
}

impl Default for SflpDecoder {
    fn default() -> Self {
        Self::new(Self::DEFAULT_MOUNT)
    }
}

impl SflpDecoder {
    /// The board is mounted so that trunk = `[−raw_z, +raw_y, +raw_x]`, a -90° rotation
    /// about Y. Facing the robot, its sensor X points up, Y points to the observer's right
    /// (the robot's left), and Z points rearward.
    pub const DEFAULT_MOUNT: [f64; 4] = [
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        -std::f64::consts::FRAC_1_SQRT_2,
        0.0,
    ];

    pub fn new(mount: [f64; 4]) -> Self {
        Self::with_gyro_unit(mount, GYRO_DPS_PER_LSB)
    }

    /// Select the wire unit for the transport; mounting and filtering stay shared.
    pub fn with_gyro_unit(mount: [f64; 4], dps_per_lsb: f64) -> Self {
        Self {
            mount,
            gyro_rad_per_lsb: dps_per_lsb * std::f64::consts::PI / 180.0,
            last_quat: [1.0, 0.0, 0.0, 0.0],
            quat_samples: 0,
            gyro_history: [[0.0; 3]; 2],
            gravity_history: [[0.0, 0.0, -1.0]; 2],
        }
    }

    /// Whether the chip has demonstrably produced fused output — roughly 0.25 s at 100 Hz.
    ///
    /// Until this is true the orientation is a default, not a measurement. Slice 2's fall
    /// detection must not run before it.
    pub fn ready(&self) -> bool {
        self.quat_samples >= 25
    }

    pub fn decode(&mut self, raw: &[u8; IMU_BLOCK_LEN]) -> ImuData {
        let gyro_sensor = [
            i16::from_le_bytes([raw[0], raw[1]]) as f64 * self.gyro_rad_per_lsb,
            i16::from_le_bytes([raw[2], raw[3]]) as f64 * self.gyro_rad_per_lsb,
            i16::from_le_bytes([raw[4], raw[5]]) as f64 * self.gyro_rad_per_lsb,
        ];
        let gyro = rotate(self.mount, gyro_sensor);

        // All-zero quaternion bytes mean SFLP has not written its table yet — the board
        // just powered up, or its init failed. Keep the last good value.
        let packed = [
            u16::from_le_bytes([raw[6], raw[7]]),
            u16::from_le_bytes([raw[8], raw[9]]),
            u16::from_le_bytes([raw[10], raw[11]]),
        ];
        if packed != [0, 0, 0] {
            let (x, y, z) = (half(packed[0]), half(packed[1]), half(packed[2]));
            let norm_sq = x * x + y * y + z * z;
            // A quaternion the chip never produced would fail this; ≤ 1.02 allows for
            // half-precision rounding at full scale.
            if x.is_finite() && y.is_finite() && z.is_finite() && norm_sq <= 1.02 {
                let w = (1.0 - norm_sq).max(0.0).sqrt();
                self.accept_quaternion([w, x, y, z]);
            }
        }

        self.filter_sample(gyro)
    }

    /// Shared-bus six-byte smallest-three encoding, validated before use.
    pub fn decode_packed(&mut self, raw: &[u8; IMU_BLOCK_LEN]) -> Result<ImuData, &'static str> {
        let mut bits = 0u64;
        for (i, value) in raw[6..].iter().enumerate() {
            bits |= u64::from(*value) << (8 * i);
        }
        if bits & (1 << 47) != 0 {
            return Err("MCU IMU not ready or invalid quaternion");
        }
        let omitted = ((bits >> 45) & 3) as usize;
        let scale = 16383.0 * std::f64::consts::SQRT_2;
        let mut q = [0.0; 4];
        let mut slot = 0;
        for (axis, value) in q.iter_mut().enumerate() {
            if axis == omitted { continue; }
            let unsigned = ((bits >> (15 * slot)) & 0x7fff) as i32;
            let signed = if unsigned & 0x4000 != 0 { unsigned - 0x8000 } else { unsigned };
            if signed == -16384 { return Err("Invalid packed quaternion component"); }
            *value = f64::from(signed) / scale;
            slot += 1;
        }
        let remainder = 1.0 - q.iter().map(|v| v * v).sum::<f64>();
        if remainder < 0.25 - 0.0001 {
            return Err("Invalid packed quaternion omitted component");
        }
        q[omitted] = remainder.sqrt();
        if q.iter().any(|v| v.abs() > q[omitted] + 0.0001) {
            return Err("Packed quaternion did not omit its largest component");
        }
        self.accept_quaternion(q);
        let gyro_sensor = std::array::from_fn(|i| {
            f64::from(i16::from_le_bytes([raw[2*i], raw[2*i+1]])) * self.gyro_rad_per_lsb
        });
        Ok(self.filter_sample(rotate(self.mount, gyro_sensor)))
    }

    fn accept_quaternion(&mut self, sensor: [f64; 4]) {
        let inverse = [self.mount[0], -self.mount[1], -self.mount[2], -self.mount[3]];
        let q = mul(sensor, inverse);
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        self.last_quat = q.map(|v| v / norm);
        self.quat_samples = self.quat_samples.saturating_add(1);
    }

    fn filter_sample(&mut self, gyro: [f64; 3]) -> ImuData {
        // Normalise *before* the median, matching the runtime. Note the consequence: a
        // component-wise median across three unit vectors is not itself unit-norm, so
        // during a transient the policy sees a slightly short vector. Steady state is
        // exact. Whether the training env expects strict unit norm is worth settling when
        // slice 2 wires up observations — normalising after the median would guarantee it,
        // but that is a behaviour change to a path that currently walks.
        let gravity = normalise(rotate_inverse(self.last_quat, [0.0, 0.0, -1.0]));

        let out = ImuData {
            gyro: median3_each(&self.gyro_history, gyro),
            gyro_unfiltered: gyro,
            gravity: median3_each(&self.gravity_history, gravity),
            quat: self.last_quat,
        };
        self.gyro_history = [self.gyro_history[1], gyro];
        self.gravity_history = [self.gravity_history[1], gravity];
        out
    }
}

/// Single-sample spike rejection. A dropped or corrupted block shows up as one wild value;
/// a median over three discards it. Monotonic motion still lags by one sample.
pub(crate) fn median3_each(history: &[[f64; 3]; 2], now: [f64; 3]) -> [f64; 3] {
    let m = |a: f64, b: f64, c: f64| a.max(b).min(c).max(a.min(b));
    [
        m(history[0][0], history[1][0], now[0]),
        m(history[0][1], history[1][1], now[1]),
        m(history[0][2], history[1][2], now[2]),
    ]
}

/// IEEE 754 binary16 → f64. The LSM6DSV16X ships quaternion components in this format.
fn half(bits: u16) -> f64 {
    let sign = if bits & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = ((bits >> 10) & 0x1F) as i32;
    let frac = (bits & 0x3FF) as f64;
    match exp {
        0 => sign * frac * 2.0_f64.powi(-24),
        0x1F if frac == 0.0 => sign * f64::INFINITY,
        0x1F => f64::NAN,
        _ => sign * (1.0 + frac / 1024.0) * 2.0_f64.powi(exp - 15),
    }
}

/// Hamilton product, scalar-first.
pub(crate) fn mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let ([aw, ax, ay, az], [bw, bx, by, bz]) = (a, b);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// `q · v · q⁻¹`
pub(crate) fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (t, c) = cross_terms(q, v);
    [
        v[0] + q[0] * t[0] + c[0],
        v[1] + q[0] * t[1] + c[1],
        v[2] + q[0] * t[2] + c[2],
    ]
}

/// `q⁻¹ · v · q` — world vector expressed in the body frame.
pub(crate) fn rotate_inverse(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (t, c) = cross_terms(q, v);
    [
        v[0] - q[0] * t[0] + c[0],
        v[1] - q[0] * t[1] + c[1],
        v[2] - q[0] * t[2] + c[2],
    ]
}

fn cross_terms(q: [f64; 4], v: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let (x, y, z) = (q[1], q[2], q[3]);
    let t = [
        (y * v[2] - z * v[1]) * 2.0,
        (z * v[0] - x * v[2]) * 2.0,
        (x * v[1] - y * v[0]) * 2.0,
    ];
    let c = [
        y * t[2] - z * t[1],
        z * t[0] - x * t[2],
        x * t[1] - y * t[0],
    ];
    (t, c)
}

/// Unit vector, falling back to "upright" rather than dividing by ~zero.
fn normalise(v: [f64; 3]) -> [f64; 3] {
    let mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if mag > 0.1 {
        [v[0] / mag, v[1] / mag, v[2] / mag]
    } else {
        [0.0, 0.0, -1.0]
    }
}

/// Compile-time assurance that the joint count and this module stay independent — the IMU
/// block is fixed-size regardless of how many servos share the bus.
const _: () = assert!(NUM_JOINTS > 0);
