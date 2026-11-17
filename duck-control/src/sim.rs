//! A [`RobotIo`] whose robot is in MuJoCo.
//!
//! MuJoCo and `RemoteIo` simulation backend. Everything above this trait — the
//! control loop, the policy, `Safety`, fall detection, odometry, kinematics, every IPC call and
//! `robotctl` — runs unchanged and cannot tell the difference. That is the whole point: the seam
//! is the one place a simulator is allowed to exist.
//!
//! ## Why TCP and not a unix socket
//!
//! Two reasons, both learned rather than assumed. A unix path is capped at `SUN_LEN` — about 108
//! bytes — which a scratch directory blows through immediately. And the simulator has to be
//! reachable from *outside* whatever the daemons run in: a container on Linux, and on macOS a Linux
//! VM with MuJoCo on the host beside it. A port crosses all of those; a socket path does not.
//!
//! ## Why JSON
//!
//! One tick is fifteen joints in and fifteen out — about a kilobyte, so 50 KB/s at the loop's 50 Hz,
//! which is nothing next to being able to read a frame with `nc` and write the other half of it in
//! twenty lines of Python. The alternative is a packed struct shared between two repositories in
//! two languages, which is exactly the shape of thing this project has already lost days to when an
//! offset was wrong and the failure was silent.
//!
//! ## What it does when the simulator goes away
//!
//! It goes away *often*: MuJoCo compiles its model, so changing the number of ducks means restarting
//! it, and the ducks are expected to survive that. So a dead connection is an error returned to the
//! caller and a reconnect on the next call — no backoff thread, because **the control loop is the
//! retry timer**. `robotd` already treats a failed `read` as a tick to skip rather than a reason to
//! exit; this is the same tolerance it has for a missing `tofd`.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use serde::{Deserialize, Serialize};

use crate::imu::ImuData;
use crate::io::{IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::NUM_JOINTS;

/// Bumped when the wire format changes in a way an older peer would misread.
///
/// Checked in the handshake and reported by *both* numbers when it fails, because the two halves
/// live in two repositories and "the simulator is too old" and "the daemon is too old" are the same
/// symptom otherwise.
pub const PROTOCOL: u32 = 1;

/// How long to wait for the simulator to answer one request.
///
/// Comfortably longer than a tick, because a simulator that has just been asked to step a
/// contact-heavy scene can be late without being broken — and comfortably shorter than forever,
/// because a wedged simulator must not wedge the control loop with it.
const TIMEOUT: Duration = Duration::from_millis(200);

/// One request. `op` is the tag, so a frame is readable in a log without a decoder.
#[derive(Debug, Serialize)]
#[serde(tag = "op")]
enum Request<'a> {
    #[serde(rename = "hello")]
    Hello { protocol: u32, joints: usize },
    #[serde(rename = "read")]
    Read,
    #[serde(rename = "write")]
    Write { targets: &'a [f64; NUM_JOINTS] },
    #[serde(rename = "gain")]
    Gain { kp: u16 },
    #[serde(rename = "torque")]
    Torque { on: bool },
    #[serde(rename = "slow")]
    Slow,
}

#[derive(Debug, Deserialize)]
struct Hello {
    protocol: u32,
}

/// What the simulator reports, in exactly the units [`Sensors`] wants — radians, rad/s, mA, and the
/// IMU already resolved into the trunk frame.
///
/// **The simulator does the conversion, not this.** MuJoCo knows its own model's joint order,
/// scaling and frame; a translation layer here would be a second place for that knowledge to live
/// and drift. What crosses the wire is the robot's own units.
#[derive(Debug, Deserialize)]
struct SensorFrame {
    positions: [f64; NUM_JOINTS],
    velocities: [f64; NUM_JOINTS],
    /// Optional, and `None` is not a simulator reporting no load: a MuJoCo model without
    /// actuator force to report simply omits it, and filling the gap with zeros would put
    /// "every joint unloaded" on the state stream. [`RobotIo::measures_load`] carries the
    /// difference out of here.
    #[serde(default)]
    currents_ma: Option<[f64; NUM_JOINTS]>,
    imu: ImuFrame,
}

#[derive(Debug, Deserialize)]
struct ImuFrame {
    gyro: [f64; 3],
    gravity: [f64; 3],
    quat: [f64; 4],
}

#[derive(Debug, Deserialize)]
struct SlowFrame {
    volts: f64,
    temps_c: [f64; NUM_JOINTS],
}

/// An acknowledgement, so a write that the simulator refused is not silently a write that worked.
#[derive(Debug, Deserialize)]
struct Ack {
    #[serde(default)]
    error: Option<String>,
}

/// A robot in MuJoCo, reached over TCP.
pub struct RemoteIo {
    addr: String,
    link: Option<Link>,
    /// Reported once rather than every tick a disconnected loop runs.
    complained: bool,
    /// Whether the last frame carried a load block. Per-frame rather than settled at startup:
    /// nothing is connected until the first read, so there is no earlier moment at which this
    /// could be known.
    load_reported: bool,
}

struct Link {
    write: TcpStream,
    read: BufReader<TcpStream>,
}

impl RemoteIo {
    /// Name the simulator. Nothing is connected until the first call — a daemon must start whether
    /// or not the simulator is up yet, exactly as it starts without a robot on the bus.
    pub fn at(addr: impl Into<String>) -> Self {
        Self {
            addr: addr.into(),
            link: None,
            complained: false,
            load_reported: false,
        }
    }

    fn connect(&mut self) -> Result<&mut Link> {
        if self.link.is_none() {
            let address = self
                .addr
                .to_socket_addrs()
                .and_then(|mut a| {
                    a.next()
                        .ok_or_else(|| std::io::Error::other("resolved to no address"))
                })
                .map_err(|source| IoError::Port {
                    path: self.addr.clone(),
                    source,
                })?;

            // A connect timeout as well as a read one: a port nobody listens on refuses instantly,
            // but a host that silently drops packets would otherwise hang the loop.
            let stream =
                TcpStream::connect_timeout(&address, TIMEOUT).map_err(|source| IoError::Port {
                    path: self.addr.clone(),
                    source,
                })?;

            // **Nagle would be catastrophic here and silent.** It delays a small write waiting for
            // more to send, up to ~40 ms — twice the tick — turning every transaction into a
            // missed deadline that looks like a slow simulator.
            let _ = stream.set_nodelay(true);
            let _ = stream.set_read_timeout(Some(TIMEOUT));
            let _ = stream.set_write_timeout(Some(TIMEOUT));

            let read = BufReader::new(stream.try_clone().map_err(|source| IoError::Port {
                path: self.addr.clone(),
                source,
            })?);
            let mut link = Link {
                write: stream,
                read,
            };

            let hello: Hello = exchange(
                &mut link,
                &Request::Hello {
                    protocol: PROTOCOL,
                    joints: NUM_JOINTS,
                },
            )?;
            if hello.protocol != PROTOCOL {
                return Err(IoError::Bus(format!(
                    "the simulator speaks protocol {} and this daemon speaks {PROTOCOL} — one of \
                     the two is out of date, and they are in different repositories",
                    hello.protocol
                )));
            }

            tracing::info!(addr = %self.addr, protocol = PROTOCOL, "simulated body");
            self.complained = false;
            self.link = Some(link);
        }
        Ok(self.link.as_mut().expect("just connected"))
    }

    /// One request and its answer, dropping the connection if anything about it fails.
    ///
    /// Dropping on *any* error, not only on a closed socket: a frame that will not parse means the
    /// two ends disagree about where a message begins, and there is no way to resynchronise a
    /// line protocol except by starting again.
    fn call<T: for<'de> Deserialize<'de>>(&mut self, request: &Request<'_>) -> Result<T> {
        let result = self.connect().and_then(|link| exchange(link, request));
        if let Err(error) = &result {
            self.link = None;
            if !self.complained {
                self.complained = true;
                tracing::warn!(
                    addr = %self.addr, %error,
                    "no simulated body; retrying every tick until it answers"
                );
            }
        }
        result
    }
}

fn exchange<T: for<'de> Deserialize<'de>>(link: &mut Link, request: &Request<'_>) -> Result<T> {
    let mut line = serde_json::to_string(request).map_err(|e| IoError::Bus(e.to_string()))?;
    line.push('\n');
    link.write
        .write_all(line.as_bytes())
        .map_err(|e| IoError::Bus(format!("sending {}: {e}", tag(request))))?;

    let mut answer = String::new();
    let read = link
        .read
        .read_line(&mut answer)
        .map_err(|e| IoError::Bus(format!("waiting for {}: {e}", tag(request))))?;
    if read == 0 {
        return Err(IoError::Bus(format!(
            "the simulator closed the connection during {}",
            tag(request)
        )));
    }
    serde_json::from_str(&answer).map_err(|e| {
        IoError::Bus(format!(
            "{} answered with something this cannot read: {e}",
            tag(request)
        ))
    })
}

fn tag(request: &Request<'_>) -> &'static str {
    match request {
        Request::Hello { .. } => "hello",
        Request::Read => "read",
        Request::Write { .. } => "write",
        Request::Gain { .. } => "gain",
        Request::Torque { .. } => "torque",
        Request::Slow => "slow",
    }
}

fn acked(ack: Ack, what: &str) -> Result<()> {
    match ack.error {
        None => Ok(()),
        Some(why) => Err(IoError::Bus(format!("the simulator refused {what}: {why}"))),
    }
}

impl RobotIo for RemoteIo {
    fn read(&mut self) -> Result<Sensors> {
        let frame: SensorFrame = self.call(&Request::Read)?;
        self.load_reported = frame.currents_ma.is_some();
        Ok(Sensors {
            positions: frame.positions,
            velocities: frame.velocities,
            currents_ma: frame.currents_ma.unwrap_or([0.0; NUM_JOINTS]),
            servo_status: None,
            min_servo_voltage: None,
            imu: ImuData {
                gyro: frame.imu.gyro,
                gyro_unfiltered: frame.imu.gyro,
                gravity: frame.imu.gravity,
                quat: frame.imu.quat,
            },
            feedback: None,
            acceleration: None,
            imu_timing: None,
        })
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        let ack: Ack = self.call(&Request::Write {
            targets: &targets.positions,
        })?;
        acked(ack, "the joint targets")
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        let ack: Ack = self.call(&Request::Gain { kp })?;
        acked(ack, "the position gain")
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        let ack: Ack = self.call(&Request::Torque { on })?;
        acked(ack, if on { "torque on" } else { "torque off" })
    }

    /// A simulated servo has no latched hardware error to clear and no firmware to restart, so
    /// there is nothing to send: the reboot succeeds at once and the caller restores the gains as
    /// it would on a robot. Deliberately not an op on the wire — the simulator would only have to
    /// answer it with an ack.
    /// The simulator computes joint velocity from the physics, so this is a measurement in the
    /// same sense the bus's is. Load is the one that may be missing, and says so per frame.
    fn measures_load(&self) -> bool {
        self.load_reported
    }

    fn reboot(&mut self, id: u8) -> Result<()> {
        tracing::debug!(id, "reboot of a simulated servo: nothing to do");
        Ok(())
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        let frame: SlowFrame = self.call(&Request::Slow)?;
        Ok(SlowSensors {
            volts: frame.volts,
            temps_c: frame.temps_c,
        })
    }
}
