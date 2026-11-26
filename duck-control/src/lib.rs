//! The robot control core: everything between reading the bus and writing it.
//!
//! Deliberately not a daemon. There is no tokio here, no socket, no systemd — `robotd`
//! owns all of that. The boundary is enforced by the compiler rather than by discipline,
//! which is what stops process concerns leaking into the code that drives motors.
//!
//! The control path it holds — model, bus, [`io::RobotIo`], observations, policy, safety — is
//! implemented in this crate.

pub mod bus;
pub mod accel_calibration;
pub mod execution_timing;
pub mod fall;
pub mod gyro_calibration;
pub mod gyro_gains;
pub mod hx;
pub mod i2c_imu;
pub mod imu;
pub mod imu_host;
pub mod imu_timing;
pub mod manual_head;
pub mod io;
pub mod servo_diagnostics;
pub mod model;
pub mod obs;
pub mod policy;
pub mod roll_phase;
pub mod safety;
/// A robot in MuJoCo, over TCP.
pub mod sim;

pub use imu::ImuData;
pub use io::{FakeIo, IoError, JointTargets, RobotIo, Sensors, SlowSensors};
pub mod read_diagnostic;
pub use model::{
    BATTERY_EMPTY_V, BATTERY_FULL_V, DEFAULT_POSITION, JOINT_IDS, JOINT_NAMES, NUM_JOINTS,
    REST_POSITION, battery_percent,
};
pub use obs::{ACTION_LEN, Command, OBS_LEN, Observation};
