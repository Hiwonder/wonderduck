//! The robot, as data.
//!
//! One variant — **alpha** — because that is the only robot that exists. Every shipped
//! policy is `alpha_*`; v1/v1.5/v1.6 are history. A second revision becomes a second set
//! of tables, which is honest until there is a second robot to generalise from.
//!
//! The numeric values here are lifted from `microduck_runtime`'s `motor.rs`, where they
//! were measured against hardware rather than derived. Re-deriving them from a datasheet
//! is exactly the kind of change that looks right and walks wrong.

/// Left leg (5) · neck/head/mouth (5) · right leg (5).
pub const NUM_JOINTS: usize = 15;

/// Dynamixel IDs, indexed as [`JOINT_NAMES`].
pub const JOINT_IDS: [u8; NUM_JOINTS] = [
    20, 21, 22, 23, 24, // left leg
    30, 31, 32, 33, 34, // neck, head, mouth
    10, 11, 12, 13, 14, // right leg
];

/// Joint names, from the protocol crate — the wire indexes `joints` and `targets`
/// positionally, so that order and this one cannot be allowed to drift apart. The
/// assertion below is what makes "cannot" true.
pub use duck_ipc_proto::JOINT_NAMES;

const _: () = assert!(JOINT_NAMES.len() == NUM_JOINTS);

/// The mouth is absent from every alpha policy — they are all 61-D observation, 14-action,
/// and the action vector skips this index. Named so that omission is deliberate rather
/// than an off-by-one someone has to rediscover.
pub const MOUTH_INDEX: usize = 9;

/// Home pose. The trunk sits ~5 mm further forward than the v1.5 pose so the CoM is over
/// the ankle axis; the old pose biased the robot backwards.
///
/// Must match `HOME_FRAME` in the training env — a policy is trained against these angles
/// and observes joint positions *relative* to them, so a discrepancy here is a constant
/// offset on 14 observation slots.
pub const DEFAULT_POSITION: [f64; NUM_JOINTS] = [
    0.0,     // left_hip_yaw
    -0.0873, // left_hip_roll
    -0.4579, // left_hip_pitch
    -0.0049, // left_knee
    0.4530,  // left_ankle
    0.3491,  // neck_pitch
    0.3491,  // head_pitch
    0.0,     // head_yaw
    0.0,     // head_roll
    0.0,     // mouth
    0.0,     // right_hip_yaw
    0.0873,  // right_hip_roll
    0.4579,  // right_hip_pitch
    0.0049,  // right_knee
    -0.4530, // right_ankle
];

/// Rest pose: where a seated robot settles with its torque off.
///
/// The shutdown eases the joints here from the policy's seat before cutting torque. The seat
/// the sitstand network holds is not where the robot comes to rest once nothing holds it, so
/// cutting torque straight from the seat let the robot sag forward. Arriving here first, it
/// is already where it would fall to on the recorded robot. This robot's measured travel
/// takes precedence: bound the endpoint before interpolation and close the mouth at rest.
/// Stability of the bounded pose still requires hardware validation.
pub const REST_POSITION: [f64; NUM_JOINTS] = bound_rest_position([
    0.3129,  // left_hip_yaw
    0.4050,  // left_hip_roll
    -0.2338, // left_hip_pitch
    1.5493,  // left_knee
    0.5522,  // left_ankle
    0.9572,  // neck_pitch
    83.0 * std::f64::consts::PI / 180.0, // head_pitch
    -0.0129, // head_yaw
    -0.0522, // head_roll
    -0.0121, // mouth
    0.0353,  // right_hip_yaw
    -0.5660, // right_hip_roll
    0.2853,  // right_hip_pitch
    -1.4696, // right_knee
    -0.4096, // right_ankle
]);

const fn bound_rest_position(mut position: [f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
    let mut i = 0;
    while i < NUM_JOINTS {
        let (lower, upper) = JOINT_LIMITS[i];
        if position[i] < lower {
            position[i] = lower;
        } else if position[i] > upper {
            position[i] = upper;
        }
        i += 1;
    }
    position[MOUTH_INDEX] = MOUTH_CLOSED;
    position
}

/// Mouth travel, radians, as the servo counts it is sent as.
///
/// The mouth is not part of any policy — every alpha network is 14 actions with this joint
/// skipped — so these two numbers and [`mouth_target`] are the whole of mouth control.
///
/// Calibration: the servo counts 4096 per turn and the mouth's zero is closed, so
/// `present_position` 2048 is the shut jaw and 1600 is the jaw wide open — 448 counts,
/// 39.375°.
///
/// The HX joint direction is negative: a positive opening angle lowers the encoder count.
/// A negative opening angle would drive past the closed stop instead.
pub const MOUTH_CLOSED: f64 = 0.0;
pub const MOUTH_OPEN: f64 = 39.375 * std::f64::consts::PI / 180.0;

/// Mechanical travel for every joint, indexed as [`JOINT_NAMES`].
///
/// The policy joints use the shared training ranges except for robot-specific bounds on
/// left hip yaw, head pitch, head yaw, and right hip yaw. Mouth travel remains independently
/// measured because it is absent from the policy.
/// All normal motor writes pass through [`crate::safety::Safety`] before the bus.
pub const JOINT_LIMITS: [(f64, f64); NUM_JOINTS] = [
    ( // left_hip_yaw
        -26.0 * std::f64::consts::PI / 180.0,
        30.0 * std::f64::consts::PI / 180.0,
    ),
    ( // left_hip_roll
        -40.0 * std::f64::consts::PI / 180.0,
        26.0 * std::f64::consts::PI / 180.0,
    ),
    ( // left_hip_pitch
        -110.0 * std::f64::consts::PI / 180.0,
        43.0 * std::f64::consts::PI / 180.0,
    ),
    ( // left_knee
        -70.0 * std::f64::consts::PI / 180.0,
        74.0 * std::f64::consts::PI / 180.0,
    ),
    ( // left_ankle
        -83.0 * std::f64::consts::PI / 180.0,
        70.0 * std::f64::consts::PI / 180.0,
    ),
    ( // neck_pitch
        -130.0 * std::f64::consts::PI / 180.0,
        55.0 * std::f64::consts::PI / 180.0,
    ),
    ( // head_pitch
        -90.0 * std::f64::consts::PI / 180.0,
        90.0 * std::f64::consts::PI / 180.0,
    ),
    ( // head_yaw
        -120.0 * std::f64::consts::PI / 180.0,
        120.0 * std::f64::consts::PI / 180.0,
    ),
    ( // head_roll
        -17.0 * std::f64::consts::PI / 180.0,
        17.0 * std::f64::consts::PI / 180.0,
    ),
    (MOUTH_CLOSED, MOUTH_OPEN),
    ( // right_hip_yaw
        -30.0 * std::f64::consts::PI / 180.0,
        20.0 * std::f64::consts::PI / 180.0,
    ),
    ( // right_hip_roll
        -26.0 * std::f64::consts::PI / 180.0,
        40.0 * std::f64::consts::PI / 180.0,
    ),
    ( // right_hip_pitch
        -43.0 * std::f64::consts::PI / 180.0,
        110.0 * std::f64::consts::PI / 180.0,
    ),
    ( // right_knee
        -74.0 * std::f64::consts::PI / 180.0,
        70.0 * std::f64::consts::PI / 180.0,
    ),
    ( // right_ankle
        -70.0 * std::f64::consts::PI / 180.0,
        83.0 * std::f64::consts::PI / 180.0,
    ),
];

/// Joint angle for a mouth opening fraction. 0 is closed, 1 is fully open; anything outside
/// is clamped rather than fed to a servo as an out-of-travel target.
pub fn mouth_target(open: f64) -> f64 {
    let open = if open.is_finite() {
        open.clamp(0.0, 1.0)
    } else {
        0.0
    };
    MOUTH_CLOSED + open * (MOUTH_OPEN - MOUTH_CLOSED)
}

/// The `imu_to_dxl` v2 board's Dynamixel ID. It rides the motor bus and is read in the
/// same transaction as the servos ([`crate::bus`]).
pub const IMU_DXL_ID: u8 = 200;

pub const BAUD_RATE: u32 = 1_000_000;

/// What a servo answers as out of the box: ID 1 at 57 600 baud. Both are deliberately unused
/// on this bus — no joint is ID 1 and nothing runs at that speed — which is what lets a
/// replacement be told apart from every servo already fitted ([`crate::bus`]).
pub const FACTORY_ID: u8 = 1;
pub const FACTORY_BAUD_RATE: u32 = 57_600;

/// EEPROM registers asserted (and corrected) at startup.
///
/// `return_delay_time` is the load-bearing one: the XL330 ships at 250, which is 500 µs of
/// turnaround *per device*. Across 16 devices that is 8 ms per tick — 40% of a 20 ms budget
/// — spent waiting for servos to get around to answering. The rest are here because the
/// runtime found them worth pinning.
///
/// `shutdown = 52` is `0b110100` — overload, electrical shock, overheating — with the
/// input-voltage bit **clear**, where the factory's 53 sets it. That bit is what clears torque
/// once the supply passes the servo's `Max Voltage Limit`, which nothing here writes and which
/// therefore stays at its default 7.0 V. A charged 2S pack sits above that, so the clear bit is
/// the reason fifteen servos do not latch themselves off a fully charged battery. Read as
/// "latches on input-voltage faults" it says the opposite of what it does.
pub const EXPECTED_REGISTERS: &[(&str, u8)] = &[
    ("return_delay_time", 0),
    ("baud_rate", 3), // 3 = 1 Mbps, must agree with BAUD_RATE
    ("pwm_slope", 255),
    ("shutdown", 52),
];

/// `homing_offset` on the knees, in raw position counts (0.088° each, so −45°).
///
/// Every other joint is at 0. The value is `microduck_runtime`'s `setup_motor_rpi.py`, the
/// flashing rig every servo on a built robot went through, so [`DEFAULT_POSITION`] and every
/// policy assume it. A servo straight from the box arrives at 0, so a knee adopted without this
/// reads and drives 45° off, with nothing in the journal to say why.
pub const KNEE_HOMING_OFFSET: i32 = -512;

/// Right and left knee: the only joints whose `homing_offset` is not 0.
pub const KNEE_IDS: [u8; 2] = [13, 23];

/// The `homing_offset` a servo should hold, asserted alongside [`EXPECTED_REGISTERS`]. It is
/// kept apart from them because it is an `i32` and differs per joint.
pub fn homing_offset(id: u8) -> i32 {
    if KNEE_IDS.contains(&id) {
        KNEE_HOMING_OFFSET
    } else {
        0
    }
}

/// Index of a joint by name. Linear scan over 15 entries, used at startup and in tests.
pub fn joint_index(name: &str) -> Option<usize> {
    JOINT_NAMES.iter().position(|n| *n == name)
}

// ── battery ──────────────────────────────────────────────────────────────────
//
// There is no fuel gauge and no ADC. The only measurement available is what the servos
// report as their own supply (`crate::bus::DynamixelIo::bus_voltage`), which is the pack
// seen through the bus — so it sags under load and recovers when the robot stands still.
// That is why the span below is *usable-under-load*, not the cell chemistry's range.
//
// The span is a statement about the *rail*, and it holds because that rail is the pack: the
// servos are fed the 2S battery, not a regulated 5 V. A servo on a bench supply reads its own
// supply voltage like any other, so 5 V lands under `BATTERY_EMPTY_V` and maps to 0% — the
// mapping working as defined rather than a fault to chase. Anything that regulates the servo
// rail has to move these two constants with it.

/// Off a full charge, under load. NP-F550, 2S Li-ion.
pub const BATTERY_FULL_V: f64 = 8.2;

/// The sag floor: below this the robot starts struggling, well before the pack's own
/// protection trips. Empty for our purposes, not empty for the cells'.
pub const BATTERY_EMPTY_V: f64 = 6.6;

/// Fraction of a pack, 0–100, for a bus voltage.
///
/// Linear, and the numbers come from `microduck_runtime`'s `check_battery`, where they were
/// arrived at by running a duck flat. It lives here rather than in `robotd` so there is one
/// mapping: the prototype had it in a CLI *and* re-derived in the app, which is how two
/// screens end up disagreeing about the same pack.
///
/// A non-finite or non-positive reading is 0 — those mean "no answer from the bus", and the
/// caller is expected to report that as unknown rather than to display this number.
pub fn battery_percent(volts: f64) -> f64 {
    if !volts.is_finite() || volts <= 0.0 {
        return 0.0;
    }
    ((volts - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V)).clamp(0.0, 1.0) * 100.0
}
