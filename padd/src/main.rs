//! `padd` — a gamepad, as an intent client.
//!
//! It has no privileged access to the robot. It reads a pad, turns sticks and buttons into
//! intents, and sends them over `robotd`'s socket like any other client.
//!
//! That is the point of it being a separate process rather than a thread inside `robotd`.
//! The intent API is the path the app, the SDK and any remote client will use, and here it
//! gets exercised every day by whoever is working on the robot — so it cannot quietly rot
//! the way an API only the phone app uses inevitably would. The cost is a socket hop: tens
//! of microseconds against a 20 ms tick.
//!
//! ## The mapping
//!
//! The sticks drive continuously, the D-pad and left/right face buttons provide fixed-speed
//! movement, and the two small buttons in the middle are holds.
//!
//! A and the bumpers remain configurable in `[pad]`; triangle always requests ground pick.
//!
//! ```text
//! A (South)       sit ↔ stand
//! Y (North)       ground pick
//! X / B           turn left / right at 0.5 rad/s while held
//! LB / RB         left / right kick
//! L2              mouth opens while held and closes on release, currently silent
//! D-pad           forward / back / strafe at 0.15 m/s while held
//! Left stick      forward / back / strafe
//! Right stick     turn left / right
//! Start           short press initializes and enables the policy
//! Start, 3 s      request rest
//! Select + Start  toggle between drive and drive + head control
//! ```
//!
//! Smoothing lives in `robotd` (`[control] cmd_alpha` / `head_alpha`), not here: this
//! process sends raw targets, so every client gets the same feel.
//!
//! ## Roller mode
//!
//! At startup this asks `robot.mode`. On a roller robot the stick mapping becomes the
//! prototype's roller preset — asymmetric forward/brake (0.6 / 0.5), no strafe, ±0.3 rad/s
//! heading — and Y triggers the crouch that lives in the ground-pick slot. The other
//! skills ride along on wheels, as the rebased roller line has them. Switching between walk and
//! roller is `robot.setMode`; it is no longer on the pad.
//!
//! ## On the robot, this runs itself
//!
//! `padd.service` starts at boot and stays up whether or not a pad is present, so driving takes one
//! step and it is a pairing step: `sudo robotctl pad pair`, with the pad in pairing mode. The
//! pad is bonded *and trusted*, so it reconnects by itself afterwards, and this process picks it up
//! within a tick.
//!
//! Waiting with no pad sends no stick commands. Losing the controlling pad stops its motion.
//! Holding Start for three seconds requests rest when the gamepad can acquire control.
//!
//! Pairing is **not** done here: bonding a device needs root and BlueZ, and a `padd` holding
//! either would stop being the unprivileged client whose whole value is having no special
//! access. It lives in `configd`, next to wifi.
//!
//! ## It also hands out the pad's raw input
//!
//! One socket, read-only, for `pad.input` and nothing else — `src/tap.rs`, and it does not make this
//! a privileged process. It exists because `padd` is the reason a stalled radio is invisible: the
//! sticks are *polled*, so the last known value keeps being sent — at the full rate, since a stick
//! reading anything but centre is never held back — whether or not the pad is still talking, and
//! every surface downstream then shows a robot with a live driver. The event
//! stream one layer below has the evidence, so it is passed out unaltered rather than summarised.
//! `robotctl monitor` draws it; `docs/robot/pair-a-gamepad.md` says how to read it.
//!

use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use clap::{Parser, ValueEnum};
use duck_ipc_proto as proto;
use gilrs::{Axis, Button, Gilrs, MappingSource};
use robotd_params::BusProtocol;

mod connection;
mod control;
mod precision_drive;
mod precision_input;
#[cfg(target_os = "linux")]
mod ps3_idle;
#[cfg(target_os = "linux")]
mod ps3_trigger;

use control::StartAction;

use precision_drive::{
    PrecisionDrive, poll_precision_status, precision_axes, precision_request, reset_precision,
    sample_precision,
};





/// The physical labelling convention used when presenting the standard controls to an operator.
///
/// `gilrs` normalises both conventions before this process sees an event: the action buttons are
/// South/East/North/West, the shoulder buttons are the triggers, and the stick axes have one
/// meaning. The classic convention additionally requires that normalisation to exist, because a
/// raw generic device has no safe, portable interpretation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum Layout {
    /// The existing four-face-button convention.
    Standard,
    /// The older four-symbol-button convention.
    Classic,
}

impl Layout {
    fn requires_normalized_mapping(self) -> bool {
        matches!(self, Self::Classic)
    }
}

#[cfg(target_os = "linux")]
mod tap;

/// The raw tap, on a platform with no evdev to read.
///
/// A `padd` on a Mac still drives a pad — that is the bench setup in the crate docs above, and it
/// would be a poor trade to lose it over a debug facility. It serves no tap, and `robotctl monitor`
/// finds no socket and says so, which is the truth rather than an empty stream.
#[cfg(not(target_os = "linux"))]
mod tap {
    pub struct Tap;

    impl Tap {
        pub fn serve(_socket: &std::path::Path) -> std::io::Result<Self> {
            Err(std::io::Error::other(
                "the raw pad tap reads evdev, which only Linux has",
            ))
        }

        pub fn watch(&self, _pad: &gilrs::Gamepad<'_>) {}

        pub fn idle(&self) {}

        pub fn imu_control(&self, _on: bool) {}

        pub fn attitude(&self) -> Option<[f32; 4]> {
            None
        }
    }
}

#[derive(Parser, Debug)]
#[command(name = "padd", about = "Drive the robot from a gamepad", version)]
struct Args {
    /// `robotd`'s socket.
    #[arg(long, default_value = "/run/robotd.sock")]
    socket: PathBuf,

    /// Where the button bindings are read from — the same file everything else is configured in.
    #[arg(long, default_value = robotd_params::DEFAULT_PATH)]
    config: PathBuf,

    /// Physical labelling convention. `classic` requires a standard `gilrs` mapping rather than
    /// guessing a device-specific raw button number.
    #[arg(long, value_enum, default_value = "standard")]
    layout: Layout,

    /// How often to read the pad, 1–1000 Hz. While a pad is present the loop runs at least
    /// every 100 ms for lease renewal and rest-chord detection.
    ///
    /// Not quite how often intents are *sent*: a frame identical to the last one and asking
    /// for no motion is held back, down to [`HEARTBEAT`]. See [`Continuous`].
    // Bounded both ways, and refused rather than clamped so the flag says what it did.
    // Zero reaches `1.0 / 0.0` and `Duration::from_secs_f64` panics on infinity. The ceiling
    // is the other half of the same line: a rate this loop cannot keep gives a period of 0 ns,
    // `checked_sub` never has anything left to sleep on, and the pad spins on robotd's socket —
    // the same busy loop `robotctl monitor` clamps for, and the range `control.hz` already
    // rejects outside.
    #[arg(long, default_value_t = 50, value_parser = clap::value_parser!(u32).range(1..=1000))]
    hz: u32,

    /// Deflection below this counts as centre. Analogue sticks rarely rest at exactly zero,
    /// and without this the robot creeps. The prototype's value.
    #[arg(long, default_value_t = 0.1)]
    deadzone: f64,

    /// Recognize short stick gestures as precision movements and delay long gestures.
    #[arg(long)]
    gamepad_precision: bool,

    /// Full-deflection forward speed, m/s.
    #[arg(long, default_value_t = 0.4)]
    max_linear: f64,

    /// Full-deflection lateral speed, m/s (default: --max-linear).
    #[arg(long, value_parser = parse_speed_limit)]
    max_lateral: Option<f64>,

    /// Full-deflection backward speed, m/s — the prototype caps reverse separately.
    #[arg(long, default_value_t = 0.4)]
    max_linear_backward: f64,

    /// Full-deflection turn rate, rad/s.
    #[arg(long, default_value_t = 1.0)]
    max_angular: f64,

    /// Head travel from center at full deflection, before the joint limits, radians.
    #[arg(long, default_value_t = 2.5, value_parser = parse_speed_limit)]
    max_head: f64,

    /// Where to serve the raw input tap: the pad's own event stream, for `robotctl monitor`.
    ///
    /// Read-only, and nothing on the driving path depends on it — if the socket cannot be created
    /// `padd` says so once and drives anyway.
    #[arg(long, default_value = proto::socket::PAD)]
    tap_socket: PathBuf,
}

/// How long to wait between checks when there is no pad.
///
/// Longer than a control tick on purpose. This process now runs from boot on every robot, and most
/// of the time there is no pad connected at all — spinning at the control rate to discover that
/// again is a wakeup every 20 ms, forever, for nothing. Half a second is imperceptible when someone
/// switches a pad on and is not a background load.
const IDLE_POLL: Duration = Duration::from_millis(500);

/// The prototype's roller-mode stick shaping: push and brake are asymmetric, there is no
/// strafe, and heading is capped at 0.3 rad/s regardless of the walking limits — the
/// roller launch line's `--max-angular-vel 0.3`, unchanged across both of its eras.
const ROLLER_PUSH: f64 = 0.6;
const ROLLER_BRAKE: f64 = 0.5;
const ROLLER_YAW: f64 = 0.3;

/// How often to look for a rewritten config. See the loop.
const BINDINGS_POLL: Duration = Duration::from_secs(1);

const BUTTON_LINEAR_MPS: f64 = 0.15;
const BUTTON_TURN_RAD_S: f64 = 0.5;
const PICK_MOUTH_OPEN_AFTER: Duration = Duration::from_millis(200);
const PICK_MOUTH_CLOSE_AFTER: Duration = Duration::from_millis(1500);
const MOUTH_MANUAL_THRESHOLD: f64 = 0.05;

/// Mouth timing for an accepted ground-pick request. An L2 press takes over immediately.
#[derive(Default)]
struct PickMouth {
    started_at: Option<Instant>,
}

impl PickMouth {
    fn start(&mut self, at: Instant) {
        self.started_at = Some(at);
    }

    fn cancel(&mut self) -> bool {
        self.started_at.take().is_some()
    }

    fn output(&mut self, now: Instant, manual: f64) -> f64 {
        let Some(started_at) = self.started_at else { return manual };
        if manual > MOUTH_MANUAL_THRESHOLD {
            self.cancel();
            return manual;
        }
        let elapsed = now.saturating_duration_since(started_at);
        if elapsed >= PICK_MOUTH_CLOSE_AFTER {
            self.cancel();
            0.0
        } else if elapsed >= PICK_MOUTH_OPEN_AFTER {
            1.0
        } else {
            manual
        }
    }
}

fn parse_speed_limit(value: &str) -> Result<f64, String> {
    let limit = value.parse::<f64>().map_err(|_| "expected a finite speed >= 0")?;
    if !limit.is_finite() || limit < 0.0 {
        return Err("expected a finite speed >= 0".into());
    }
    Ok(limit)
}



/// Configured axis ranges within the operator's startup and button-adjusted speed limits.
fn walking_limits(
    drive: &robotd_params::PadDriveParams,
    args: &Args,
    walk_speed: f64,
    turn_speed: f64,
) -> robotd_params::PadDriveParams {
    let reverse = if args.max_linear > 0.0 {
        args.max_linear_backward.max(0.0) * walk_speed / args.max_linear
    } else {
        0.0
    };
    let lateral = if args.max_linear > 0.0 {
        args.max_lateral.unwrap_or(args.max_linear).max(0.0) * walk_speed / args.max_linear
    } else {
        args.max_lateral.unwrap_or(0.0).max(0.0)
    };
    robotd_params::PadDriveParams {
        vx_max: drive.vx_max.min(walk_speed),
        vx_min: drive.vx_min.max(-reverse),
        vy_max: drive.vy_max.min(lateral),
        vy_min: drive.vy_min.max(-lateral),
        vyaw_max: drive.vyaw_max.min(turn_speed),
        vyaw_min: drive.vyaw_min.max(-turn_speed),
    }
}

/// The configurable button bindings and walking speeds, or the defaults.
///
/// A file that will not parse is never a reason to leave somebody without a pad: the defaults
/// are a working robot, and the reason is logged. That matters more here than elsewhere because
/// this is re-read while running — a half-saved file caught mid-write must not take the buttons
/// away, and the next read a second later gets the finished one.
fn read_bindings(
    path: &Path,
) -> (
    robotd_params::PadParams,
    robotd_params::PadDriveParams,
    robotd_params::PadImuHeadControlParams,
    BusProtocol,
) {
    match robotd_params::Params::load(path, false) {
        Ok(params) => {
            let pad = params.pad;
            let drive = params.pad_drive;
            tracing::info!(
                a = %pad.a, lb = %pad.lb, rb = %pad.rb,
                vx = ?(drive.vx_min, drive.vx_max), vy = ?(drive.vy_min, drive.vy_max),
                vyaw = ?(drive.vyaw_min, drive.vyaw_max),
                "button bindings"
            );
            (pad, drive, params.pad_imu_head_control, params.bus.protocol)
        }
        Err(e) => {
            tracing::warn!(
                error = %e,
                path = %path.display(),
                "cannot read the button bindings; using the default mapping"
            );
            (
                robotd_params::PadParams::default(),
                robotd_params::PadDriveParams::default(),
                robotd_params::PadImuHeadControlParams::default(),
                BusProtocol::default(),
            )
        }
    }
}

/// Map the controller's attitude relative to its neutral pose to head joint angles.
fn head_from_pad(relative: [f32; 4], gain: f64, max_head: f64) -> proto::HeadParams {
    let [pitch, roll, yaw] = pad_imu::euler_deg(relative);
    let angle = |degrees: f32| (f64::from(degrees).to_radians() * gain).clamp(-max_head, max_head);
    proto::HeadParams {
        head_pitch: angle(pitch),
        head_yaw: angle(yaw),
        head_roll: -angle(roll),
        ..proto::HeadParams::default()
    }
}

fn head_drive_target(protocol: BusProtocol, head: proto::HeadParams) -> proto::HeadParams {
    if protocol != BusProtocol::Hx {
        return head;
    }

    use duck_control::hx::{
        MANUAL_HEAD_POSITION_LIMITS, PHYSICAL_HOME_ENCODER_TARGETS, decode_position, encode_position,
    };
    let offsets = [head.head_pitch, head.head_yaw];
    let raw_position = std::array::from_fn(|axis| {
        let joint = axis + 6;
        let decode = |raw: u16| {
            decode_position(joint, &raw.to_le_bytes()).expect("valid HX head calibration")
        };
        let home = decode(PHYSICAL_HOME_ENCODER_TARGETS[joint]);
        let (min_raw, max_raw) = MANUAL_HEAD_POSITION_LIMITS[axis];
        let offset = if offsets[axis].is_finite() { offsets[axis] } else { 0.0 };
        let target = (home + offset).clamp(decode(max_raw), decode(min_raw));
        encode_position(joint, target).expect("target within HX head travel")
    });
    proto::HeadParams { raw_position: Some(raw_position), ..proto::HeadParams::default() }
}

#[cfg(test)]
mod head_drive_tests {
    use super::{BusProtocol, head_drive_target, proto};

    #[test]
    fn hx_head_is_absolute_and_only_sends_pitch_and_yaw() {
        let target = |pitch, yaw| {
            head_drive_target(BusProtocol::Hx, proto::HeadParams {
                head_pitch: pitch,
                head_yaw: yaw,
                head_roll: 0.4,
                neck_pitch: 0.3,
                ..proto::HeadParams::default()
            })
        };
        for (pitch, yaw, expected) in [
            (0.0, 0.0, [1820, 2048]),
            (-2.5, 2.5, [2300, 1024]),
            (2.5, -2.5, [1100, 3072]),
            (0.1, -0.1, [1755, 2113]),
            (0.0, 0.0, [1820, 2048]),
        ] {
            assert_eq!(target(pitch, yaw), proto::HeadParams {
                raw_position: Some(expected),
                ..proto::HeadParams::default()
            });
        }
    }
}

/// When the config was last written, for spotting a change. `None` for a file that is not there,
/// which is a real state and compares equal to itself.
fn config_mtime(path: &Path) -> Option<std::time::SystemTime> {
    std::fs::metadata(path).ok()?.modified().ok()
}

fn is_active_button(button: Button) -> bool {
    matches!(
        button,
        Button::South | Button::East | Button::West | Button::North
            | Button::LeftTrigger | Button::RightTrigger | Button::LeftTrigger2
            | Button::Start | Button::DPadUp | Button::DPadDown
            | Button::DPadLeft | Button::DPadRight
    )
}

fn pad_neutral(pad: &gilrs::Gamepad<'_>, deadzone: f64, head_drive: bool, r2: u8) -> bool {
    let axes = [
        Axis::LeftStickX,
        Axis::LeftStickY,
        Axis::RightStickX,
    ];
    let buttons = [
        Button::South,
        Button::East,
        Button::North,
        Button::West,
        Button::LeftTrigger,
        Button::RightTrigger,
        Button::LeftTrigger2,
        Button::Start,
        Button::DPadUp,
        Button::DPadDown,
        Button::DPadLeft,
        Button::DPadRight,
    ];
    r2 == 0 && axes.into_iter()
        .chain(head_drive.then_some(Axis::RightStickY))
        .all(|axis| {
            let value = (pad.value(axis) as f64).abs();
            value == 0.0 || value < deadzone
        })
        && buttons.iter().all(|button| !pad.is_pressed(*button))
        && [Button::LeftTrigger2]
            .iter()
            .all(|button| {
                pad.button_data(*button)
                    .is_none_or(|data| data.value() <= 0.2)
            })
}

fn intent_accepted(response: Option<proto::Response>) -> bool {
    response.is_some_and(|response| {
        response.error.is_none()
            && response
                .result_as::<proto::IntentResult>()
                .is_ok_and(|result| result.accepted)
    })
}

fn policies_status(response: Option<proto::Response>) -> std::io::Result<proto::PoliciesResult> {
    let response = response.ok_or_else(|| std::io::Error::other("missing policies response"))?;
    if let Some(error) = &response.error {
        return Err(std::io::Error::other(format!(
            "policies refused: {}",
            error.message
        )));
    }
    response.result_as().map_err(Into::into)
}

#[derive(Debug, PartialEq, Eq)]
enum StartResult {
    AlreadyEnabled,
    Requested,
    InitializationRefused,
    EnableRefused,
}

fn start_robot(stream: &mut UnixStream, next_id: &mut u64) -> std::io::Result<StartResult> {
    let policies = policies_status(request(stream, next_id, &proto::Call::RobotPolicies)?)?;
    if policies.motion_enabled == Some(true) {
        return Ok(StartResult::AlreadyEnabled);
    }
    if policies.homed != Some(true)
        && !intent_accepted(request(stream, next_id, &proto::Call::RobotInit)?)
    {
        return Ok(StartResult::InitializationRefused);
    }
    let enable = proto::Call::RobotEnable(proto::EnableParams { on: true, toggle: false });
    if intent_accepted(request(stream, next_id, &enable)?) {
        Ok(StartResult::Requested)
    } else {
        Ok(StartResult::EnableRefused)
    }
}

fn maintain_control(
    stream: &mut UnixStream,
    next_id: &mut u64,
    gate: &mut control::Gate,
) -> std::io::Result<()> {
    let now = Instant::now();
    gate.expire(now);
    if gate.refresh_due(now) {
        let result = control::call(stream, next_id, proto::HumanControlAction::Refresh)?;
        gate.refreshed(&result, now);
        gate.expire(Instant::now());
    }
    Ok(())
}

fn explicit_low_voltage_rearm(stream: &mut UnixStream, next_id: &mut u64, gate: &mut control::Gate) -> std::io::Result<bool> {
    let id = proto::Id::Number(*next_id);
    *next_id += 1;
    let request = proto::Request::call(id, &proto::Call::RobotLowVoltageRearm);
    let response = match connection::exchange_with_timeout(stream, &request, Duration::from_secs(10)) {
        Ok(response) => response,
        Err(error) => { gate.lost(); return Err(error); }
    };
    if let Some(error) = response.error {
        gate.lost();
        tracing::warn!(message = %error.message, "low-voltage recovery refused");
        return Ok(false);
    }
    let response = match response.result_as::<proto::IntentResult>() {
        Ok(response) => response,
        Err(error) => { gate.lost(); return Err(error.into()); }
    };
    if !response.accepted { gate.lost(); return Ok(false); }
    if response.reason.as_deref() == Some(proto::LOW_VOLTAGE_REARMED) {
        // The daemon revoked our old lease. The Start press that cleared the
        // interlock is not a new movement command: every button (including
        // Start) and every stick must be neutral before a later press can
        // acquire control again.
        gate.lost();
        tracing::warn!("low-voltage protection cleared; release all controls, then press Start again");
        return Ok(false);
    }
    // No lock was cleared; retain control only if this same session still
    // owns it. A different source may have taken over during a slow reply.
    let requested_at = Instant::now();
    match control::call(stream, next_id, proto::HumanControlAction::Refresh) {
        Ok(result) => { gate.refreshed(&result, requested_at); Ok(gate.driving()) }
        Err(error) => { gate.lost(); Err(error) }
    }
}

fn main() -> std::process::ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    // Before anything that can fail, and before the gamepad subsystem especially: `padd` was the
    // one daemon whose journal could not say which build was running, which came up while chasing
    // exactly that question across all five.
    duck_ipc_proto::log_startup_identity!("padd");

    let mut gilrs = match Gilrs::new() {
        Ok(gilrs) => gilrs,
        Err(e) => {
            tracing::error!(error = %e, "no gamepad subsystem");
            return std::process::ExitCode::FAILURE;
        }
    };

    // Start the tap before connecting to robotd so it remains available during connection retries.
    // Its own failure is logged and stepped over — see `--tap-socket`.
    let tap = match tap::Tap::serve(&args.tap_socket) {
        Ok(tap) => Some(tap),
        Err(e) => {
            tracing::warn!(
                error = %e, socket = %args.tap_socket.display(),
                "no raw pad tap — `robotctl monitor` cannot show the pad's own event stream"
            );
            None
        }
    };

    loop {
        let _ = run_session(&args, &mut gilrs, tap.as_ref());
        if let Some(tap) = tap.as_ref() {
            tap.idle();
        }
        tracing::warn!("robot session ended; retrying");
        std::thread::sleep(IDLE_POLL);
    }
}

fn run_session(
    args: &Args,
    gilrs: &mut Gilrs,
    tap: Option<&tap::Tap>,
) -> std::process::ExitCode {
    let mut stream = match connection::connect(&args.socket) {
        Ok(stream) => stream,
        Err(e) => {
            tracing::error!(error = %e, socket = %args.socket.display(), "cannot reach robotd");
            return std::process::ExitCode::FAILURE;
        }
    };

    let mut next_id = 1u64;
    match control::call(
        &mut stream,
        &mut next_id,
        proto::HumanControlAction::Register,
    ) {
        Ok(result) if result.accepted => {}
        Ok(result) => {
            tracing::error!(reason = ?result.reason, "gamepad registration refused");
            return std::process::ExitCode::FAILURE;
        }
        Err(error) => {
            tracing::error!(%error, "gamepad registration failed");
            return std::process::ExitCode::FAILURE;
        }
    }
    let mut gate = control::Gate::default();

    // Which robot is this? A roller duck wants the roller stick shaping. Asked at startup and then
    // again with the config check below, so a `robot.setMode` from anywhere — the pad no longer
    // switches modes itself — changes the stick shaping within a second.
    let mut roller = match ask_roller(&mut stream, &mut next_id) {
        Ok(roller) => roller.unwrap_or(false),
        Err(e) => {
            tracing::error!(error = %e, "mode request failed");
            return std::process::ExitCode::FAILURE;
        }
    };
    tracing::warn!(
        socket = %args.socket.display(),
        hz = args.hz,
        roller,
        "driving — sticks and D-pad move, X/B turn, Y ground pick, A sit, LB/RB kicks, \
         L2 mouth; Start initializes and enables, hold Start for 3 s to rest, Select + Start toggles head control"
    );

    let period = Duration::from_secs_f64(1.0 / args.hz as f64);
    // The button bindings, read once like every other daemon reads its config. A file that will
    // not parse is not a reason to leave somebody without a pad: the default mapping is the
    // fallback, and the reason is logged.
    let (mut bindings, mut drive, mut imu_head, bus_protocol) = read_bindings(&args.config);
    let accessory_trigger = bus_protocol == BusProtocol::Hx;
    // When the file was last written, so a change is picked up without a restart.
    let mut bindings_at = config_mtime(&args.config);
    let mut bindings_checked = Instant::now();

    let mut buttons_were_driving = false;
    // Whether a pad was there last tick, so appearing and disappearing are each logged once.
    let mut driving = false;
    let mut active_pad_id = None;
    let mut start = control::StartControls::default();
    let mut head_drive = false;
    let mut imu_reference = None;
    let mut center_sticks = false;
    let mut rest_rearm_required = false;
    let mut pick_mouth = PickMouth::default();
    // The continuous intents, and the buffer this tick's are built in. Both live across
    // ticks so a steady state neither allocates nor re-sends — see [`Continuous`].
    let mut continuous = Continuous::default();
    let mut idle_since = None;
    let mut release_checked = None;
    let mut previous_imu_pose: Option<[f64; 2]> = None;
    let mut frame: Vec<proto::Call> = Vec::with_capacity(if accessory_trigger { 3 } else { 2 });
    #[cfg(target_os = "linux")]
    let mut ps3_r2 = None;
    #[cfg(target_os = "linux")]
    let mut ps3_idle = ps3_idle::Ps3Idle::default();
    let mut precision = args
        .gamepad_precision
        .then(|| PrecisionDrive::new(period.min(control::REFRESH_INTERVAL)));

    loop {
        let tick = Instant::now();

        // Once a second, not every tick: a `stat` at 50 Hz to catch a file somebody edits by
        // hand a few times a week is work for nothing, and a second is faster than typing the
        // next command.
        if tick.duration_since(bindings_checked) >= BINDINGS_POLL {
            bindings_checked = tick;
            let now = config_mtime(&args.config);
            if now != bindings_at {
                bindings_at = now;
                (bindings, drive, imu_head, _) = read_bindings(&args.config);
                tracing::warn!("button bindings reloaded");
            }
            match ask_roller(&mut stream, &mut next_id) {
                Ok(Some(now)) if now != roller => {
                    if let Err(error) =
                        reset_precision(&mut precision, &mut stream, &mut next_id, gate.owns())
                    {
                        tracing::error!(%error, "precision cancellation failed");
                        return std::process::ExitCode::FAILURE;
                    }
                    roller = now;
                    tracing::warn!(roller, "drive mode changed — stick shaping follows");
                }
                Ok(_) => {}
                Err(e) => {
                    tracing::error!(error = %e, "mode request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        // Drain the queue so axis and movement-button polling below sees the current state.
        //
        // The driving pad is the first one, and only its events may act: with two pads
        // connected, a Start or Select from the *other* one would otherwise steer a robot
        // whose sticks belong to somebody else.
        let pad_id = if driving {
            active_pad_id
        } else {
            gilrs.gamepads().next().map(|(id, _)| id)
        };
        // Which bindable buttons went down this tick, by their config name. A list rather than
        // a flag apiece, because what each one runs is config now and this loop no longer knows.
        let mut pressed: Vec<&'static str> = Vec::new();
        let mut button_activity = false;
        let mut pad_disconnected = false;
        while let Some(event) = gilrs.next_event() {
            #[cfg(target_os = "linux")]
            ps3_idle.event(event, tick, args.deadzone);
            if Some(event.id) != pad_id {
                continue;
            }
            if matches!(event.event, gilrs::EventType::ButtonPressed(button, _) if is_active_button(button)) {
                button_activity = true;
            }
            if matches!(event.event, gilrs::EventType::Disconnected) {
                pad_disconnected = true;
            }
            match event.event {
                gilrs::EventType::ButtonPressed(button, _) => start.button(button, true, tick),
                gilrs::EventType::ButtonReleased(button, _) => start.button(button, false, tick),
                _ => {}
            }
            match event.event {
                gilrs::EventType::ButtonPressed(button, _) => match button {
                    // gilrs names the *bumpers* `LeftTrigger`/`RightTrigger`; the analog
                    // triggers are `LeftTrigger2`/`RightTrigger2`. Getting that backwards binds
                    // a skill to a control nobody presses.
                    Button::South => pressed.push("a"),
                    Button::North => pressed.push("y"),
                    Button::LeftTrigger => pressed.push("lb"),
                    Button::RightTrigger => pressed.push("rb"),
                    // Home. Not bindable: it is not a skill, and the robot owns the toggle.
                    _ => {}
                },
                _ => {}
            }
        }

        // Re-read the pad by id after the drain: a `Disconnected` dequeued above may have
        // taken it, and asking for the first pad again would silently hand the robot the
        // *other* pad's sticks. One tick of "pad gone" beats that.
        let Some(pad) = pad_id
            .filter(|_| !pad_disconnected)
            .and_then(|id| gilrs.connected_gamepad(id))
        else {
            // A lost pad must cancel a pending short command immediately.
            //
            // Logged once per transition, at `warn` so it survives `RUST_LOG=warn` on a board.
            // "The pad went away" is the single most useful line in the journal when the robot
            // stops responding mid-drive, and one line per tick would bury it.
            pick_mouth.cancel();
            gate.expire(Instant::now());
            let cancellation = reset_precision(
                &mut precision,
                &mut stream,
                &mut next_id,
                gate.owns(),
            );
            if control::stop_on_pad_loss(driving, gate.owns()) {
                match control::call(
                    &mut stream,
                    &mut next_id,
                    proto::HumanControlAction::Abandon,
                ) {
                    Ok(result) if result.accepted => {
                        tracing::warn!("pad lost; control released and motion stopped")
                    }
                    Ok(_) => tracing::info!("pad lost after control was transferred"),
                    Err(error) => {
                        tracing::warn!(%error, "pad loss release failed; closing the session");
                        return std::process::ExitCode::FAILURE;
                    }
                }
                gate.lost();
            }
            if let Err(error) = cancellation {
                tracing::error!(%error, "precision cancellation on pad loss failed");
                return std::process::ExitCode::FAILURE;
            }
            if driving {
                tracing::warn!("pad gone");
                driving = false;
                active_pad_id = None;
            }
            start = control::StartControls::default();
            imu_reference = None;
            buttons_were_driving = false;
            rest_rearm_required = false;
            if let Some(tap) = tap.as_ref() {
                tap.idle();
            }
            gate.lost();
            idle_since = None;
            previous_imu_pose = None;
            #[cfg(target_os = "linux")]
            ps3_idle.poll(gilrs, tick, args.deadzone, None);
            std::thread::sleep(IDLE_POLL);
            continue;
        };
        #[cfg(target_os = "linux")]
        if accessory_trigger && active_pad_id != pad_id {
            ps3_r2 = ps3_trigger::Ps3Trigger::open(&pad);
        }
        active_pad_id = pad_id;

        if args.layout.requires_normalized_mapping() && pad.mapping_source() == MappingSource::None
        {
            tracing::error!("classic layout requires a normalized gamepad mapping");
            return std::process::ExitCode::FAILURE;
        }
        let (start_action, suppress_motion) = start.finish(
            pad.is_pressed(Button::Start),
            pad.is_pressed(Button::Select),
            tick,
        );
        let r2 = if accessory_trigger {
            let mapped = pad.button_data(Button::RightTrigger2).map(|data| data.value()).unwrap_or(0.0);
            #[cfg(target_os = "linux")]
            let pressure = ps3_r2.as_mut().map_or(mapped, |device| device.pressure(tick).unwrap_or(0.0));
            #[cfg(not(target_os = "linux"))]
            let pressure = mapped;
            if pressure > 0.02 { (pressure.min(1.0) * 100.0).round() as u8 } else { 0 }
        } else {
            0
        };
        if start_action == Some(StartAction::Rest) {
            gate.expire(Instant::now());
            if gate.driving() {
                if pick_mouth.cancel()
                    && let Err(error) = notify(&mut stream, &proto::Call::RobotMouth(proto::MouthParams::default()))
                {
                    tracing::error!(%error, "closing mouth before rest failed");
                    return std::process::ExitCode::FAILURE;
                }
                if let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true) {
                    tracing::error!(%error, "precision cancellation before rest failed");
                    return std::process::ExitCode::FAILURE;
                }
                if let Err(error) = notify(&mut stream, &proto::Call::RobotMove(proto::MoveParams::default())) {
                    tracing::error!(%error, "stopping before rest failed");
                    return std::process::ExitCode::FAILURE;
                }
                match request(&mut stream, &mut next_id, &proto::Call::RobotRest) {
                    Ok(response) => {
                        if intent_accepted(response) {
                            tracing::warn!("Start held for three seconds — rest requested");
                            rest_rearm_required = true;
                        } else {
                            tracing::warn!("Start held for three seconds — rest refused");
                        }
                    }
                    Err(error) => {
                        tracing::error!(%error, "rest request failed");
                        return std::process::ExitCode::FAILURE;
                    }
                }
            }
        }
        if suppress_motion || (rest_rearm_required && !pad_neutral(&pad, args.deadzone, head_drive, r2)) {
            #[cfg(target_os = "linux")]
            ps3_idle.poll(gilrs, tick, args.deadzone, (r2 > 0).then_some(pad.id()));
            if let Some(precision) = precision.as_mut() {
                precision.reset();
                precision.awaiting_completion = false;
            }
            imu_reference = None;
            buttons_were_driving = false;
            if let Err(error) = maintain_control(&mut stream, &mut next_id, &mut gate) {
                tracing::error!(%error, "control refresh during rest failed");
                return std::process::ExitCode::FAILURE;
            }
            if let Some(tap) = tap.as_ref() {
                tap.watch(&pad);
                tap.imu_control(false);
            }
            driving = true;
            if let Some(remaining) = period
                .min(control::REFRESH_INTERVAL)
                .checked_sub(tick.elapsed())
            {
                std::thread::sleep(remaining);
            }
            continue;
        }
        rest_rearm_required = false;

        // Track the IMU even without ownership so a fresh controller tilt can
        // request control. Compare to the last significant pose to filter drift.
        let attitude = tap.and_then(|tap| {
            tap.watch(&pad);
            tap.imu_control(head_drive && imu_head.enabled);
            if head_drive && imu_head.enabled { tap.attitude() } else { None }
        });
        if attitude.is_none() {
            imu_reference = None;
        }
        let imu_head_pose = attitude.map(|attitude| {
            let reference = *imu_reference.get_or_insert(attitude);
            head_from_pad(pad_imu::relative(reference, attitude), imu_head.gain, args.max_head)
        });
        let imu_angles = imu_head_pose.map(|head| [head.head_pitch, head.head_yaw]);
        let imu_moving = imu_angles.zip(previous_imu_pose).is_some_and(|(next, previous)| {
            next.into_iter().zip(previous).any(|(next, previous)| (next - previous).abs() >= 0.025)
        });
        if imu_moving || previous_imu_pose.is_none() || imu_angles.is_none() {
            previous_imu_pose = imu_angles;
        }
        let neutral = pad_neutral(&pad, args.deadzone, head_drive, r2);
        let activity = button_activity || imu_moving || !neutral;
        #[cfg(target_os = "linux")]
        ps3_idle.poll(gilrs, tick, args.deadzone, activity.then_some(pad.id()));
        if activity {
            idle_since = None;
        }
        gate.expire(Instant::now());
        if gate.observe(neutral, activity) {
            let requested_at = Instant::now();
            match control::call(
                &mut stream,
                &mut next_id,
                proto::HumanControlAction::AcquireIfFree,
            ) {
                Ok(result) => {
                    gate.acquired(&result, requested_at);
                    if gate.driving() {
                        continuous = Continuous::default();
                    } else {
                        tracing::info!(reason = ?result.reason, "gamepad control unavailable");
                    }
                }
                Err(e) => {
                    tracing::error!(error = %e, "control acquire failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }
        if let Err(e) = maintain_control(&mut stream, &mut next_id, &mut gate) {
            tracing::error!(error = %e, "control maintenance failed");
            return std::process::ExitCode::FAILURE;
        }
        if !gate.driving() {
            pick_mouth.cancel();
            if let Some(precision) = precision.as_mut() {
                precision.reset();
                precision.awaiting_completion = false;
                if neutral {
                    let now = precision.epoch.elapsed();
                    precision.gesture.step(now, [0.0; 3], [0.0; 3]);
                }
            }
            idle_since = None;
            buttons_were_driving = false;
            driving = true;
            let sleep_period = period.min(control::REFRESH_INTERVAL);
            if let Some(remaining) = sleep_period.checked_sub(tick.elapsed()) {
                std::thread::sleep(remaining);
            }
            continue;
        }
        if let Some(precision) = precision.as_mut()
            && let Err(error) =
                poll_precision_status(precision, &mut stream, &mut next_id, Instant::now())
        {
            tracing::error!(%error, "precision status query failed");
            return std::process::ExitCode::FAILURE;
        }
        if !driving {
            tracing::warn!(pad = pad.name(), "pad connected — driving");
            driving = true;
        }

        let mode_changed = start_action == Some(StartAction::ToggleHeadDrive);
        if mode_changed {
            if let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true) {
                tracing::error!(%error, "precision cancellation on mode change failed");
                return std::process::ExitCode::FAILURE;
            }
            head_drive = !head_drive;
            imu_reference = None;
            center_sticks = true;
            tracing::info!(head_drive, "Select and Start — gamepad mode switched");
        }

        // A mode change discards the IMU pose calculated in the previous mode.
        let imu_head_pose = imu_head_pose.filter(|_| !mode_changed);

        if start_action == Some(StartAction::Initialize) {
            match explicit_low_voltage_rearm(&mut stream, &mut next_id, &mut gate) {
                Ok(true) => {}, Ok(false) => continue,
                Err(error) => { tracing::warn!(%error, "low-voltage recovery result unknown; reconnecting and requiring neutral controls"); return std::process::ExitCode::FAILURE; }
            }
            if let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true) {
                tracing::error!(%error, "precision cancellation failed");
                return std::process::ExitCode::FAILURE;
            }
            match start_robot(&mut stream, &mut next_id) {
                Ok(StartResult::AlreadyEnabled) => tracing::info!("Start — policy already enabled"),
                Ok(StartResult::Requested) => tracing::info!("Start — initialization and policy requested"),
                Ok(StartResult::InitializationRefused) => tracing::warn!("Start — initialization refused"),
                Ok(StartResult::EnableRefused) => tracing::warn!("Start — policy enable refused"),
                Err(error) => {
                    tracing::error!(%error, "start request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        // One-shot skills. Answered, because "refused, and here is why" is a real outcome — a
        // skill this robot does not have, one mid-flight, or the policy not driving.
        //
        // The name comes from config and is sent as it was written. `padd` does not check it
        // against anything: which skills exist is the robot's to know, and it answers an unknown
        // one with the list it does have, which is a better error than this side could give.
        for button in &pressed {
            // An empty binding is a button switched off on purpose, not a fault.
            let skill = if *button == "y" {
                "ground_pick"
            } else {
                bindings.skill(button).unwrap_or_default()
            };
            if skill.is_empty() {
                tracing::debug!(button, "no skill bound");
                continue;
            }
            let call = proto::Call::RobotDo(proto::DoParams {
                skill: skill.to_owned(),
            });
            match request(&mut stream, &mut next_id, &call) {
                Ok(response) => {
                    if intent_accepted(response) {
                        if skill == "ground_pick" {
                            pick_mouth.start(tick);
                        }
                        if let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true) {
                            tracing::error!(%error, "precision cancellation failed");
                            return std::process::ExitCode::FAILURE;
                        }
                    }
                }
                Err(error) => {
                    tracing::error!(%error, "skill request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        let deadzone = |v: f32| {
            let v = v as f64;
            if v.abs() < args.deadzone { 0.0 } else { v }
        };
        let axes = [Axis::LeftStickY, Axis::LeftStickX, Axis::RightStickX, Axis::RightStickY]
            .map(|axis| deadzone(pad.value(axis)));
        if axes == [0.0; 4] {
            center_sticks = false;
        }
        let [left_y, left_x, right_x, right_y] = if center_sticks { [0.0; 4] } else { axes };
        let (strafe, turn) = if head_drive && imu_head_pose.is_none() {
            (0.0, left_x)
        } else {
            (left_x, right_x)
        };

        // An accepted pick opens and closes the mouth; L2 can take over manually.
        let lt = pad.button_data(Button::LeftTrigger2).map(|d| d.value()).unwrap_or(0.0) as f64;
        let mouth = pick_mouth.output(Instant::now(), lt);
        if let Err(e) = notify(
            &mut stream,
            &proto::Call::RobotMouth(proto::MouthParams { open: mouth }),
        ) {
            tracing::error!(error = %e, "send failed");
            return std::process::ExitCode::FAILURE;
        }

        let walk_drive = walking_limits(&drive, &args, args.max_linear.max(0.0), args.max_angular.max(0.0));
        let limits = DriveLimits {
            roller,
            drive: &walk_drive,
        };
        let digital = DigitalDrive {
            forward: pad.is_pressed(Button::DPadUp),
            backward: pad.is_pressed(Button::DPadDown),
            left: pad.is_pressed(Button::DPadLeft),
            right: pad.is_pressed(Button::DPadRight),
            turn_left: pad.is_pressed(Button::West),
            turn_right: pad.is_pressed(Button::East),
        };
        let buttons_driving = digital.active();
        if buttons_driving && !buttons_were_driving
            && let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true)
        {
            tracing::error!(%error, "precision cancellation before button driving failed");
            return std::process::ExitCode::FAILURE;
        }
        buttons_were_driving = buttons_driving;

        frame.clear();
        frame.push(proto::Call::RobotMove(digital.apply(
            limits.walk(left_y, strafe, turn),
            &walk_drive,
            roller,
        )));

        if head_drive {
            let head = imu_head_pose.unwrap_or(proto::HeadParams {
                head_pitch: -right_y * args.max_head,
                head_yaw: -right_x * args.max_head,
                ..proto::HeadParams::default()
            });
            frame.push(proto::Call::RobotHead(head_drive_target(bus_protocol, head)));
        } else if mode_changed {
            frame.push(proto::Call::RobotHead(proto::HeadParams::default()));
        }
        if accessory_trigger {
            frame.push(proto::Call::RobotAccessoryTrigger(proto::AccessoryTriggerParams { pressure: r2 }));
        }

        let mut precision_start = None;
        let mut force_frame = mode_changed;
        // Fixed-speed buttons are held commands, not precision gestures. Cancel any outstanding
        // gesture on entry and bypass the gesture scheduler while a movement button is held.
        if precision.is_some() && !buttons_driving {
            let (mapping, axes) = precision_axes(
                roller,
                left_y,
                strafe,
                turn,
            );
            let mapping_changed = precision.as_ref().is_some_and(|precision| {
                precision
                    .mapping
                    .is_some_and(|previous| previous != mapping)
            });
            if mapping_changed
                && let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true)
            {
                tracing::error!(%error, "precision cancellation on mapping change failed");
                return std::process::ExitCode::FAILURE;
            }
            let precision = precision.as_mut().expect("precision is enabled");
            precision.mapping = Some(mapping);
            if let Some(axes) = axes {
                if precision.awaiting_completion && axes.iter().any(|axis| *axis != 0.0) {
                    if let Err(error) = maintain_control(&mut stream, &mut next_id, &mut gate) {
                        tracing::error!(%error, "control maintenance before precision status failed");
                        return std::process::ExitCode::FAILURE;
                    }
                    if !gate.driving() {
                        precision.reset();
                        precision.awaiting_completion = false;
                        continue;
                    }
                }
                let movement = match &frame[0] {
                    proto::Call::RobotMove(movement) => *movement,
                    _ => unreachable!("every control frame starts with movement"),
                };
                let now = precision.epoch.elapsed();
                let out = match sample_precision(
                    precision,
                    &mut stream,
                    now,
                    axes,
                    movement,
                ) {
                    Ok(out) => out,
                    Err(error) => {
                        tracing::error!(%error, "precision interruption failed");
                        return std::process::ExitCode::FAILURE;
                    }
                };
                precision_start = out.request;
                frame[0] = proto::Call::RobotMove(out.movement);
                force_frame |= precision.awaiting_completion
                    || out.replayed
                    || precision_start.is_some();
            } else {
                force_frame |= precision.awaiting_completion;
            }
        }

        let sent = if force_frame {
            continuous.send_forced(&mut stream, &frame, tick)
        } else {
            continuous.send(&mut stream, &frame, tick)
        };
        if let Err(e) = sent {
            tracing::error!(error = %e, "send failed");
            return std::process::ExitCode::FAILURE;
        }
        if let Some(params) = precision_start {
            let precision = precision.as_mut().expect("precision is enabled");
            match precision_request(
                &mut stream,
                &mut next_id,
                params,
                precision.heartbeat_period,
            ) {
                Ok(result) if result.accepted => precision.accepted(Instant::now()),
                Ok(_) => {}
                Err(error) => {
                    tracing::error!(%error, "precision movement request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        let busy = activity || pick_mouth.started_at.is_some()
            || precision.as_ref().is_some_and(|drive| drive.awaiting_completion);
        if busy {
            idle_since = None;
        } else {
            let idle = tick.saturating_duration_since(*idle_since.get_or_insert(tick));
            let retry_due = release_checked.is_none_or(|at| {
                tick.saturating_duration_since(at) >= control::REFRESH_INTERVAL
            });
            if idle >= control::IDLE_RELEASE && retry_due {
                release_checked = Some(tick);
                match control::call(&mut stream, &mut next_id, proto::HumanControlAction::Release) {
                    Ok(result) if result.accepted
                        || result.owner != Some(proto::HumanControlSource::Gamepad) =>
                    {
                        gate.lost();
                        gate.observe(true, false);
                    }
                    Ok(_) => {}
                    Err(error) => {
                        tracing::error!(%error, "idle gamepad control release failed");
                        return std::process::ExitCode::FAILURE;
                    }
                }
            }
        }

        if let Some(remaining) = period
            .min(control::REFRESH_INTERVAL)
            .checked_sub(tick.elapsed())
        {
            let remaining = precision
                .as_ref()
                .and_then(|precision| {
                    precision
                        .next_deadline
                        .map(|deadline| deadline.saturating_sub(precision.epoch.elapsed()))
                })
                .map_or(remaining, |deadline| remaining.min(deadline));
            let remaining = precision
                .as_ref()
                .and_then(|precision| precision.next_status_check)
                .map_or(remaining, |deadline| {
                    remaining.min(deadline.saturating_duration_since(Instant::now()))
                });
            std::thread::sleep(remaining);
        }
    }
}

/// Which fixed-speed movement buttons are held on this tick.
#[derive(Default)]
struct DigitalDrive {
    forward: bool,
    backward: bool,
    left: bool,
    right: bool,
    turn_left: bool,
    turn_right: bool,
}

impl DigitalDrive {
    fn active(&self) -> bool {
        self.forward || self.backward || self.left || self.right || self.turn_left || self.turn_right
    }

    fn apply(
        &self,
        mut movement: proto::MoveParams,
        limits: &robotd_params::PadDriveParams,
        roller: bool,
    ) -> proto::MoveParams {
        let direction = |positive: bool, negative: bool, speed: f64| {
            (i8::from(positive) - i8::from(negative)) as f64 * speed
        };
        if self.forward || self.backward {
            movement.vx = direction(self.forward, self.backward, BUTTON_LINEAR_MPS)
                .clamp(limits.vx_min, limits.vx_max);
        }
        if self.left || self.right {
            movement.vy = if roller {
                0.0
            } else {
                direction(self.left, self.right, BUTTON_LINEAR_MPS)
                    .clamp(limits.vy_min, limits.vy_max)
            };
        }
        if self.turn_left || self.turn_right {
            movement.vyaw = direction(self.turn_left, self.turn_right, BUTTON_TURN_RAD_S)
                .clamp(limits.vyaw_min, limits.vyaw_max);
            if roller {
                movement.vyaw = movement.vyaw.clamp(-ROLLER_YAW, ROLLER_YAW);
            }
        }
        movement
    }
}

/// How full stick deflection maps to velocity.
#[derive(Debug, Clone, Copy)]
struct DriveLimits<'a> {
    roller: bool,
    /// `[pad_drive]`: each direction of each axis onto its own signed bound.
    drive: &'a robotd_params::PadDriveParams,
}

impl DriveLimits<'_> {
    /// A velocity from three stick axes: forward/back, strafe and turn. Which physical axis is
    /// which depends on the mode; the shaping does not.
    fn walk(&self, forward: f64, strafe: f64, turn: f64) -> proto::MoveParams {
        if self.roller {
            // The prototype's roller shaping: push harder than you can brake, no strafe,
            // heading capped independently of the walking limits.
            return proto::MoveParams {
                vx: forward
                    * if forward >= 0.0 {
                        ROLLER_PUSH
                    } else {
                        ROLLER_BRAKE
                    },
                vy: 0.0,
                vyaw: -turn * ROLLER_YAW,
            };
        }
        let scale = robotd_params::PadDriveParams::scale;
        let d = self.drive;
        proto::MoveParams {
            vx: scale(forward, d.vx_min, d.vx_max),
            // `vy` is positive to the left; stick-left reads negative on every pad gilrs
            // normalises.
            vy: scale(-strafe, d.vy_min, d.vy_max),
            vyaw: scale(-turn, d.vyaw_min, d.vyaw_max),
        }
    }
}

/// Whether this robot is on wheels, by asking it. `None` for an answer that did not say.
fn ask_roller(stream: &mut UnixStream, next_id: &mut u64) -> std::io::Result<Option<bool>> {
    let answer = request(stream, next_id, &proto::Call::RobotMode)?;
    Ok(answer
        .and_then(|answer| answer.result_as::<proto::ModeResult>().ok())
        .map(|mode| mode.mode == "roller"))
}

/// Send a continuous intent: no `id`, no reply, nothing to wait for.
fn notify(stream: &mut UnixStream, call: &proto::Call) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(&proto::Request::notify(call))?;
    line.push(b'\n');
    connection::send(stream, &line)
}

/// How long an unchanged frame may go unsent while the robot is being asked to stand still.
///
/// The sticks are *polled*, so an untouched pad re-sent the same three zeros fifty times a
/// second — a `serde_json` encode, a `write_all`, a `flush`, and a `serde_json` parse on
/// `robotd`'s side, a hundred messages a second in the modes that send two, to say nothing
/// changed. Ten a second says it as well.
///
/// **Nothing about the robot's safety rests on this number**, which is why it can be picked
/// for legibility rather than argued against `[safety] deadman_ms` — a value this daemon
/// cannot read and does not know. A frame is only ever held back when it is byte-identical
/// to the last one sent *and* asks for no motion ([`Continuous::may_hold`]); a stick that is
/// doing anything goes out on every tick as it always did. What the heartbeat buys is the
/// report: without it `robotd`'s twist would age past the deadman while a pad sat connected
/// and idle, and `robot.state` would carry `limited_by: ["deadman"]` for a robot that is
/// stationary because it was asked to be.
const HEARTBEAT: Duration = Duration::from_millis(100);

/// The continuous intents, encoded once a tick and sent when they say something new.
#[derive(Default)]
struct Continuous {
    /// This tick's frame. Kept across ticks so the encode reuses its buffer.
    line: Vec<u8>,
    /// The bytes last put on the socket, to compare this tick's against.
    last: Vec<u8>,
    /// When that was. `None` until the first send, which therefore always happens.
    at: Option<Instant>,
}

impl Continuous {
    /// Put this tick's intents on the socket, unless they are the ones already there.
    ///
    /// One write for the whole frame: the calls describe a single instant, and a peer that
    /// read half of one would be acting on a head pose without the velocity that came with
    /// it. `robotd` splits the buffer back into lines on its own read.
    fn send(
        &mut self,
        stream: &mut UnixStream,
        calls: &[proto::Call],
        now: Instant,
    ) -> std::io::Result<()> {
        self.send_inner(stream, calls, now, false)
    }

    fn send_forced(
        &mut self,
        stream: &mut UnixStream,
        calls: &[proto::Call],
        now: Instant,
    ) -> std::io::Result<()> {
        self.send_inner(stream, calls, now, true)
    }

    fn send_inner(
        &mut self,
        stream: &mut UnixStream,
        calls: &[proto::Call],
        now: Instant,
        force: bool,
    ) -> std::io::Result<()> {
        self.line.clear();
        for call in calls {
            serde_json::to_writer(&mut self.line, &proto::Request::notify(call))?;
            self.line.push(b'\n');
        }

        if !force && self.line == self.last && self.may_hold(calls, now) {
            return Ok(());
        }

        connection::send(stream, &self.line)?;
        // Swapped rather than cloned: the buffer this displaces becomes next tick's
        // scratch, so a steady state allocates nothing at all.
        std::mem::swap(&mut self.last, &mut self.line);
        self.at = Some(now);
        Ok(())
    }

    /// Whether an unchanged frame may be left unsent this tick.
    ///
    /// Only while it asks for no velocity. The deadman zeroes the twist and nothing else, so
    /// on a frame that already commands zero, letting it fire changes nothing about what the
    /// robot does — and on a frame that commands motion it would stop a robot whose stick is
    /// still held. That is the whole of the argument, and it holds whatever `deadman_ms` is
    /// set to.
    ///
    /// A held stick therefore keeps sending at the full rate. That is the case where the
    /// robot is walking and the daemon has something to say; this is about the one where it
    /// is not and does not.
    fn may_hold(&self, calls: &[proto::Call], now: Instant) -> bool {
        let asks_for_motion = calls.iter().any(|call| match call {
            proto::Call::RobotMove(p) => p.vx != 0.0 || p.vy != 0.0 || p.vyaw != 0.0,
            _ => false,
        });
        !asks_for_motion && self.at.is_some_and(|at| now.duration_since(at) < HEARTBEAT)
    }
}

/// Send a discrete intent and read its answer.
///
/// Answered, unlike the continuous ones, because "refused, and here is why" is a real
/// outcome — a skill with no policy loaded, a sound with no bank — and a client that
/// ignored it would leave the operator wondering why nothing happened.
fn request(
    stream: &mut UnixStream,
    next_id: &mut u64,
    call: &proto::Call,
) -> std::io::Result<Option<proto::Response>> {
    let id = proto::Id::Number(*next_id);
    *next_id += 1;
    let response = connection::exchange(stream, &proto::Request::call(id, call))?;
    if let Some(error) = &response.error {
        tracing::warn!(code = error.code, message = %error.message, "refused");
    } else if let Ok(result) = response.result_as::<proto::IntentResult>()
        && !result.accepted
    {
        tracing::warn!(reason = ?result.reason, "not accepted");
    }
    Ok(Some(response))
}
