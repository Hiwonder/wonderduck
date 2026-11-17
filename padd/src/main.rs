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
//! Laid out so that each part of the pad has one job: the face buttons and bumpers run skills,
//! the D-pad picks what the sticks mean, and the two small buttons in the middle are holds —
//! nothing that cuts torque or powers the robot off fires on a brush of the thumb.
//!
//! The six one-shot buttons are `[pad]` in `robotd.toml`, so a robot that has learned a new skill
//! can put it on a button without a release. Only those six: `Start`, the D-pad and held `Select`
//! are not `robot.do` calls, and the button that powers a robot off is the one binding worth not
//! being able to lose to a config edit.
//!
//! This daemon still knows nothing about what a skill *is*. It reads which button went down,
//! looks up the name beside it, and sends that name; `robotd` decides whether the robot has such
//! a thing and answers with the list it does have when it does not.
//!
//! ```text
//! A (South)       sit ↔ stand
//! B (East)        ground pick
//! X / Y           configured skills (X defaults to roulade)
//! LB / RB         left / right kick
//! L2 / R2         mouth (either trigger; the max wins), currently silent
//! L3 / R3         walking linear speed -/+ 0.01 m/s
//! D-pad left + L3 / R3   walking turn speed -/+ 0.1 rad/s
//! D-pad up        head mode — the sticks pose the head, the body holds still
//! D-pad right     head + move — left stick walks and turns, right stick looks around
//! D-pad left      move — the sticks walk, strafe and turn
//! D-pad down      body + head — left stick crouches and leans sideways, right stick looks around
//! Start           first press stands up, then toggles the policy
//! Start, 1.5 s    home pose, motors stiff, policy off — a seated robot stays seated
//! Select, 2–4 s   let go: sit, rest pose, then torque off and every servo rebooted
//! Select, 4 s     sit, rest pose, then power off
//! ```
//!
//! The D-pad *selects* a mode rather than toggling one, so a press always lands where its arrow
//! says whatever mode the robot was in. Head and body + head mode both zero the velocity while
//! active — a robot that keeps walking because you started posing its head is a bad surprise —
//! and leaving a mode puts back what it moved: the body snaps to nominal, the head re-centres.
//!
//! Smoothing lives in `robotd` (`[control] cmd_alpha` / `head_alpha`), not here: this
//! process sends raw targets, so every client gets the same feel.
//!
//! ## Roller mode
//!
//! At startup this asks `robot.mode`. On a roller robot the stick mapping becomes the
//! prototype's roller preset — asymmetric forward/brake (0.6 / 0.5), no strafe, ±0.3 rad/s
//! heading — and B triggers the crouch that lives in the ground-pick slot. The other
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
//! Start and Select together request an emergency stop even without the lease.
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

mod connection;
mod control;
mod precision_drive;
mod precision_input;

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

        pub fn has_imu(&self) -> bool {
            false
        }

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
    /// every 100 ms for lease renewal and emergency-stop detection.
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

    /// Full-deflection head travel, radians. The head command feeds the policy's
    /// observation rather than a servo directly, so this is the prototype's generous 2.5 —
    /// the network itself decides how far the head actually goes.
    #[arg(long, default_value_t = 2.5)]
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

/// Start held this long brings the robot to its home pose with the motors stiff and the policy
/// off: the "put everything back" button. Long enough that a press meant for the policy toggle
/// never reaches it, short enough to be the obvious thing to do when the robot is somewhere odd.
const HOME_HOLD: Duration = Duration::from_millis(1500);

/// Select let go after this long, and before [`SHUTDOWN_HOLD`], puts the robot down for a rest:
/// `robot.rest` — it sits if it is driving, eases into the rest pose, then torque goes off and
/// every servo reboots. Decided on the release, because until then the hold may still become a
/// power-off, and both start the same way.
const REST_HOLD: Duration = Duration::from_secs(2);

/// Select held this long powers the robot off: `robot.shutdown`, the same sit and rest pose ending
/// in a power-off. Sent the moment the hold gets here; the release after it does nothing.
const SHUTDOWN_HOLD: Duration = Duration::from_secs(4);

// Keep the trigger sound mapping available for a later sound change.
const TRIGGER_SOUNDS_ENABLED: bool = false;

fn mouth_trigger_commands(
    lt: f64,
    rt: f64,
    chirp_pressed: bool,
    previous_lt: f64,
) -> (proto::MouthParams, [Option<proto::SoundParams>; 2]) {
    let mouth = proto::MouthParams { open: rt.max(lt) };
    if !TRIGGER_SOUNDS_ENABLED {
        return (mouth, [None, None]);
    }
    const SOUND_THRESHOLD: f64 = 0.3;
    let chirp = chirp_pressed.then_some(proto::SoundParams {
        tag: proto::SoundTag::Chirp,
        hold: None,
    });
    let wheee = if lt >= SOUND_THRESHOLD {
        Some(proto::SoundParams { tag: proto::SoundTag::Wheee, hold: Some(true) })
    } else if previous_lt >= SOUND_THRESHOLD {
        Some(proto::SoundParams { tag: proto::SoundTag::Wheee, hold: Some(false) })
    } else {
        None
    };
    (mouth, [chirp, wheee])
}

/// Consume every trigger value, including a press and release inside one poll.
/// Separate press/release thresholds keep pressure noise from repeating an activity edge.
#[derive(Default)]
struct ChirpTrigger {
    held: bool,
}

impl ChirpTrigger {
    fn update(&mut self, value: f32) -> bool {
        if value <= 0.2 {
            self.held = false;
        } else if value >= 0.3 && !self.held {
            self.held = true;
            return true;
        }
        false
    }
}



/// A button that does different things depending on how long it is held.
///
/// Each threshold fires once, on the tick the hold crosses it, so a longer hold walks through
/// every shorter one on the way — Select offers rest at two seconds and powers off at four.
/// A release that crossed no threshold is a [`HoldAction::Tap`]; the release after a hold that
/// did fire says nothing, because the thumb coming off is not a second request.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct HoldButton {
    /// When the current hold began. `None` between presses.
    held_since: Option<Instant>,
    /// How many thresholds this hold has crossed.
    fired: usize,
    /// The pad went away during a hold that had already fired. Whatever is still held when it
    /// comes back is the tail of that hold, and does nothing until the release.
    spent: bool,
}

/// What a [`HoldButton`] asks for this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HoldAction {
    Nothing,
    /// Pressed and let go before the first threshold.
    Tap,
    /// The hold just crossed this threshold, by index.
    Reached(usize),
    /// Let go after a hold whose furthest threshold was this one, by index. For a button whose
    /// action depends on where the hold stopped, which is only known at the release.
    ReleasedAfter(usize),
}

impl HoldButton {
    /// One tick: is the button down now, and did it come up since the last tick.
    ///
    /// `released` is the edge from the event queue, because a press and release inside one tick
    /// leaves `pressed` false on both sides and would otherwise be a tap nobody saw.
    fn tick(
        &mut self,
        pressed: bool,
        released: bool,
        now: Instant,
        thresholds: &[Duration],
    ) -> HoldAction {
        if pressed {
            let since = *self.held_since.get_or_insert(now);
            if !self.spent
                && let Some(&next) = thresholds.get(self.fired)
                && now.duration_since(since) >= next
            {
                self.fired += 1;
                return HoldAction::Reached(self.fired - 1);
            }
            return HoldAction::Nothing;
        }
        let was_held = released || self.held_since.is_some();
        let (fired, spent) = (self.fired, self.spent);
        *self = Self::default();
        match (fired, spent) {
            // The tail of a hold cut by a dropout says nothing — see `reset`.
            (_, true) => HoldAction::Nothing,
            (0, false) if released => HoldAction::Tap,
            (0, false) => HoldAction::Nothing,
            (n, false) if was_held => HoldAction::ReleasedAfter(n - 1),
            (_, false) => HoldAction::Nothing,
        }
    }

    /// Forget a hold in flight. Called when the pad goes away: the hold's start was measured
    /// against *that* pad's button, and carrying it onto the next pad would turn a button still
    /// held across a long dropout into its longest action on the first tick back.
    ///
    /// A hold that already fired is marked spent rather than forgotten, because what it did is a
    /// fact about the robot rather than about the pad: the release that follows must stay silent,
    /// and the rest of the hold must not reach the next threshold from a fresh start.
    fn reset(&mut self) {
        self.held_since = None;
        if self.fired > 0 {
            self.spent = true;
        }
    }
}

/// Body-pose stick ranges, from the training env via the prototype: z is asymmetric
/// (little headroom up at the standing height, more crouch down), angles capped at ~15°.
const BODY_MAX_Z_UP: f64 = 0.010;
const BODY_MAX_Z_DOWN: f64 = 0.025;
const BODY_MAX_ANGLE: f64 = 0.2618;

/// The prototype's roller-mode stick shaping: push and brake are asymmetric, there is no
/// strafe, and heading is capped at 0.3 rad/s regardless of the walking limits — the
/// roller launch line's `--max-angular-vel 0.3`, unchanged across both of its eras.
const ROLLER_PUSH: f64 = 0.6;
const ROLLER_BRAKE: f64 = 0.5;
const ROLLER_YAW: f64 = 0.3;

/// What the sticks drive, picked on the D-pad. Modal because two sticks cannot express nine
/// degrees of freedom.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// D-pad left. Left stick walks and strafes, right stick turns.
    Drive,
    /// D-pad up. All four axes pose the head; the body holds still.
    Head,
    /// D-pad right. Left stick walks and turns, right stick looks around — or, with
    /// `[pad_imu_head_control]` on a pad that has an IMU, the pad's tilt poses the head and the
    /// sticks keep the whole [`Mode::Drive`] mapping.
    HeadDrive,
    /// D-pad down. Body + head: the left stick crouches and leans the standing robot sideways,
    /// the right stick looks around. The body does not walk.
    BodyPose,
}

impl Mode {
    /// Whether this mode sends head poses, so leaving it has a head to put back.
    fn poses_head(self) -> bool {
        matches!(self, Self::Head | Self::HeadDrive | Self::BodyPose)
    }
}

/// What has to be sent when the sticks stop meaning `from` and start meaning `to`.
///
/// Leaving a mode puts back what it moved, because nothing else will: a body left leaning or a
/// head left turned stays that way, and the next mode does not send the joints the last one did.
/// Moving between modes that all pose the head keeps the head, since the next one goes on posing
/// it.
fn mode_exit_calls(from: Mode, to: Mode) -> Vec<proto::Call> {
    let mut calls = Vec::new();
    if from == Mode::BodyPose && to != Mode::BodyPose {
        calls.push(proto::Call::RobotPose(proto::PoseParams {
            active: false,
            ..Default::default()
        }));
    }
    if from.poses_head() && !to.poses_head() {
        calls.push(proto::Call::RobotHead(proto::HeadParams::default()));
    }
    calls
}

/// How often to look for a rewritten config. See the loop.
const BINDINGS_POLL: Duration = Duration::from_secs(1);

/// Stick buttons change the walking command limits in small, predictable increments. The
/// limits are bounded by the command-line maxima so a button cannot drive the policy outside the
/// range selected by the operator at startup.
const WALK_SPEED_STEP_MPS: f64 = 0.01;
const WALK_TURN_SPEED_STEP_RAD_S: f64 = 0.1;

fn parse_speed_limit(value: &str) -> Result<f64, String> {
    let limit = value.parse::<f64>().map_err(|_| "expected a finite speed >= 0")?;
    if !limit.is_finite() || limit < 0.0 {
        return Err("expected a finite speed >= 0".into());
    }
    Ok(limit)
}



fn adjust_speed(current: f64, direction: i8, step: f64, maximum: f64) -> f64 {
    let maximum = maximum.max(0.0);
    let next = (current + f64::from(direction) * step).clamp(0.0, maximum);
    ((next * 100.0).round() / 100.0).clamp(0.0, maximum)
}

fn speed_step(button: Button, turn_modifier: bool) -> (i8, i8) {
    let direction = match button {
        Button::LeftThumb => -1,
        Button::RightThumb => 1,
        _ => return (0, 0),
    };
    if turn_modifier {
        (0, direction)
    } else {
        (direction, 0)
    }
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

/// The button bindings, the IMU head switch and the walking speeds, or the defaults.
///
/// A file that will not parse is never a reason to leave somebody without a pad: the defaults
/// are a working robot, and the reason is logged. That matters more here than elsewhere because
/// this is re-read while running — a half-saved file caught mid-write must not take the buttons
/// away, and the next read a second later gets the finished one.
fn read_bindings(
    path: &Path,
) -> (
    robotd_params::PadParams,
    robotd_params::PadImuHeadControlParams,
    robotd_params::PadDriveParams,
) {
    match robotd_params::Params::load(path, false) {
        Ok(params) => {
            let pad = params.pad;
            let imu_head = params.pad_imu_head_control;
            let drive = params.pad_drive;
            tracing::info!(
                a = %pad.a, b = %pad.b, x = %pad.x, y = %pad.y, lb = %pad.lb, rb = %pad.rb,
                pad_imu_head_control = imu_head.enabled, pad_imu_head_gain = imu_head.gain,
                vx = ?(drive.vx_min, drive.vx_max), vy = ?(drive.vy_min, drive.vy_max),
                vyaw = ?(drive.vyaw_min, drive.vyaw_max),
                "button bindings"
            );
            (pad, imu_head, drive)
        }
        Err(e) => {
            tracing::warn!(
                error = %e,
                path = %path.display(),
                "cannot read the button bindings; using the default mapping"
            );
            (
                robotd_params::PadParams::default(),
                robotd_params::PadImuHeadControlParams::default(),
                robotd_params::PadDriveParams::default(),
            )
        }
    }
}

/// The head pose for a pad attitude relative to its reference.
///
/// `relative` is body → world of the pad now, in the frame of the reference — [`pad_imu::relative`].
/// Its pitch, roll and yaw become the head's, scaled by `gain` and clamped to `max_head`. The signs
/// follow the joint axes: pad nose-up gives positive `head_pitch`, pad yaw left gives positive
/// `head_yaw`, and pad roll right gives negative `head_roll`. The neck stays at zero.
fn head_from_pad(relative: [f32; 4], gain: f64, max_head: f64) -> proto::HeadParams {
    let [pitch, roll, yaw] = pad_imu::euler_deg(relative);
    let angle = |degrees: f32| (f64::from(degrees).to_radians() * gain).clamp(-max_head, max_head);
    proto::HeadParams {
        neck_pitch: 0.0,
        head_pitch: angle(pitch),
        head_yaw: angle(yaw),
        head_roll: -angle(roll),
    }
}

/// When the config was last written, for spotting a change. `None` for a file that is not there,
/// which is a real state and compares equal to itself.
fn config_mtime(path: &Path) -> Option<std::time::SystemTime> {
    std::fs::metadata(path).ok()?.modified().ok()
}

fn pad_neutral(pad: &gilrs::Gamepad<'_>, deadzone: f64) -> bool {
    let axes = [
        Axis::LeftStickX,
        Axis::LeftStickY,
        Axis::RightStickX,
        Axis::RightStickY,
    ];
    let buttons = [
        Button::South,
        Button::East,
        Button::North,
        Button::West,
        Button::LeftTrigger,
        Button::RightTrigger,
        Button::LeftTrigger2,
        Button::RightTrigger2,
        Button::Start,
        Button::Select,
        Button::LeftThumb,
        Button::RightThumb,
        Button::DPadUp,
        Button::DPadDown,
        Button::DPadLeft,
        Button::DPadRight,
    ];
    axes.iter().all(|axis| {
        let value = (pad.value(*axis) as f64).abs();
        value == 0.0 || value < deadzone
    }) && buttons.iter().all(|button| !pad.is_pressed(*button))
        && [Button::LeftTrigger2, Button::RightTrigger2]
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

fn policies_homed(response: Option<proto::Response>) -> std::io::Result<bool> {
    let response = response.ok_or_else(|| std::io::Error::other("missing policies response"))?;
    if let Some(error) = &response.error {
        return Err(std::io::Error::other(format!(
            "policies refused: {}",
            error.message
        )));
    }
    let policies: proto::PoliciesResult = response.result_as()?;
    policies.homed.ok_or_else(|| {
        std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "policies response has no homed state",
        )
    })
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
    if gate.releasing() {
        let calls = [
            proto::Call::RobotMove(proto::MoveParams::default()),
            proto::Call::RobotHead(proto::HeadParams::default()),
            proto::Call::RobotPose(proto::PoseParams {
                active: false,
                ..Default::default()
            }),
            proto::Call::RobotMouth(proto::MouthParams::default()),
            proto::Call::RobotSound(proto::SoundParams {
                tag: proto::SoundTag::Wheee,
                hold: Some(false),
            }),
        ];
        for call in calls {
            notify(stream, &call)?;
        }
        let result = control::call(stream, next_id, proto::HumanControlAction::Release)?;
        gate.released(&result);
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

    let mut emergency = control::Emergency::default();
    loop {
        let _ = run_session(&args, &mut gilrs, tap.as_ref(), &mut emergency);
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
    emergency: &mut control::Emergency,
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
        "driving — A sit, B ground pick, LB/RB kicks, triggers mouth; D-pad up head, \
         right head + move, left move, down body + head; Start stands up then toggles the policy, \
         Start (1.5s) home pose; Select (2-4s) rest, Select (4s) power off"
    );

    let period = Duration::from_secs_f64(1.0 / args.hz as f64);
    // The button bindings, read once like every other daemon reads its config. A file that will
    // not parse is not a reason to leave somebody without a pad: the default mapping is the
    // fallback, and the reason is logged.
    let (mut bindings, mut imu_head_cfg, mut drive) = read_bindings(&args.config);
    // When the file was last written, so a change is picked up without a restart.
    let mut bindings_at = config_mtime(&args.config);
    let mut bindings_checked = Instant::now();

    let mut walk_speed = args.max_linear.max(0.0);
    let mut turn_speed = args.max_angular.max(0.0);
    let mut mode = Mode::Drive;
    // The pad attitude that reads as "head centred" while head + move follows the pad's IMU.
    // `None` whenever it is not following — another mode, the feature off, no IMU, or the pad
    // gone: a reference taken against one pad means nothing to the next.
    let mut imu_reference: Option<[f32; 4]> = None;
    // Whether a pad was there last tick, so appearing and disappearing are each logged once.
    let mut driving = false;
    let mut active_pad_id = None;
    let mut start = HoldButton::default();
    let mut select = HoldButton::default();
    // RT consumes queued edges; LT remains a continuously polled hold.
    let mut chirp_trigger = ChirpTrigger::default();
    let mut prev_lt = 0.0f64;
    // Does this pad think the robot is up (torque on, at the home pose)? When not, Start sends
    // `robot.init` (stand up and hold) instead of enabling the policy; the next Start enables it.
    // Starts false: padd starts with the robot, and a robotd restart restarts padd too.
    let mut up = false;
    // The continuous intents, and the buffer this tick's are built in. Both live across
    // ticks so a steady state neither allocates nor re-sends — see [`Continuous`].
    let mut continuous = Continuous::default();
    let mut frame: Vec<proto::Call> = Vec::with_capacity(2);
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
                (bindings, imu_head_cfg, drive) = read_bindings(&args.config);
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

        // Drain the queue so axis polling below sees present state, and catch button
        // *edges* — a held D-pad must select once, not fifty times a second.
        //
        // The driving pad is the first one, and only its events may act: with two pads
        // connected, a Start or Select from the *other* one would otherwise steer a robot
        // whose sticks belong to somebody else.
        let pad_id = if driving {
            active_pad_id
        } else {
            gilrs.gamepads().next().map(|(id, _)| id)
        };
        let mut wanted_mode: Option<Mode> = None;
        // Which bindable buttons went down this tick, by their config name. A list rather than
        // a flag apiece, because what each one runs is config now and this loop no longer knows.
        let mut pressed: Vec<&'static str> = Vec::new();
        let mut linear_direction = 0i8;
        let mut turn_direction = 0i8;
        let mut start_released = false;
        let mut select_released = false;
        let mut chirp_pressed = false;
        let mut button_activity = false;
        let mut pad_disconnected = false;
        while let Some(event) = gilrs.next_event() {
            if Some(event.id) != pad_id {
                continue;
            }
            if matches!(event.event, gilrs::EventType::ButtonPressed(_, _)) {
                button_activity = true;
            }
            if matches!(event.event, gilrs::EventType::Disconnected) {
                pad_disconnected = true;
            }
            match event.event {
                gilrs::EventType::ButtonPressed(button, _) => emergency.button(button, true),
                gilrs::EventType::ButtonReleased(button, _) => emergency.button(button, false),
                _ => {}
            }
            match event.event {
                gilrs::EventType::ButtonChanged(Button::RightTrigger2, value, _) => {
                    chirp_pressed |= chirp_trigger.update(value);
                }
                // Start and Select are holds: what they do depends on how long they stay down,
                // so they are read off their state every tick and their release here.
                gilrs::EventType::ButtonReleased(Button::Start, _) => start_released = true,
                gilrs::EventType::ButtonReleased(Button::Select, _) => select_released = true,
                gilrs::EventType::ButtonPressed(button, _) => match button {
                    // The six bindable ones. What each runs is `[pad]` in the config; this
                    // only knows which physical control was pressed.
                    //
                    // gilrs names the *bumpers* `LeftTrigger`/`RightTrigger`; the analog
                    // triggers are `LeftTrigger2`/`RightTrigger2`. Getting that backwards binds
                    // a skill to a control nobody presses.
                    Button::LeftThumb | Button::RightThumb => {
                        let turn_modifier = pad_id
                            .and_then(|id| gilrs.connected_gamepad(id))
                            .is_some_and(|pad| pad.is_pressed(Button::DPadLeft));
                        let (linear, turn) = speed_step(button, turn_modifier);
                        linear_direction += linear;
                        turn_direction += turn;
                    }
                    Button::South => pressed.push("a"),
                    Button::East => pressed.push("b"),
                    Button::West => pressed.push("x"),
                    Button::North => pressed.push("y"),
                    Button::LeftTrigger => pressed.push("lb"),
                    Button::RightTrigger => pressed.push("rb"),
                    // The D-pad picks what the sticks mean.
                    Button::DPadUp => wanted_mode = Some(Mode::Head),
                    Button::DPadRight => wanted_mode = Some(Mode::HeadDrive),
                    Button::DPadLeft => wanted_mode = Some(Mode::Drive),
                    Button::DPadDown => wanted_mode = Some(Mode::BodyPose),
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
            let emergency_stop = emergency.disconnected();
            gate.expire(Instant::now());
            let cancellation = reset_precision(
                &mut precision,
                &mut stream,
                &mut next_id,
                !emergency_stop && gate.owns(),
            );
            if emergency_stop {
                if let Err(e) = request(&mut stream, &mut next_id, &proto::Call::RobotStop) {
                    tracing::error!(error = %e, "emergency stop failed");
                    return std::process::ExitCode::FAILURE;
                }
                gate.lost();
            } else if control::stop_on_pad_loss(driving, gate.owns()) {
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
            // A hold in flight was measured against the pad that just left: drop it, or a
            // Select still down when the pad returns lands its full hold time at once — a
            // power-off nobody asked for.
            start.reset();
            select.reset();
            chirp_trigger = ChirpTrigger::default();
            imu_reference = None;
            if let Some(tap) = tap.as_ref() {
                tap.idle();
            }
            gate.lost();
            std::thread::sleep(IDLE_POLL);
            continue;
        };
        active_pad_id = pad_id;

        let emergency_action = emergency.finish(
            pad.is_pressed(Button::Start),
            pad.is_pressed(Button::Select),
        );
        if emergency_action.stop {
            tracing::warn!("Start and Select — emergency stop");
            if let Some(precision) = precision.as_mut() {
                precision.reset();
                precision.awaiting_completion = false;
            }
            if let Err(e) = request(&mut stream, &mut next_id, &proto::Call::RobotStop) {
                tracing::error!(error = %e, "emergency stop failed");
                return std::process::ExitCode::FAILURE;
            }
            gate.lost();
        }
        if emergency_action.suppress {
            if let Some(precision) = precision.as_mut() {
                precision.reset();
                precision.awaiting_completion = false;
            }
            gate.lost();
            start = HoldButton::default();
            select = HoldButton::default();
            chirp_trigger = ChirpTrigger::default();
            prev_lt = 0.0;
            imu_reference = None;
            if let Some(tap) = tap.as_ref() {
                tap.watch(&pad);
                tap.imu_control(imu_head_cfg.enabled);
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

        if args.layout.requires_normalized_mapping() && pad.mapping_source() == MappingSource::None
        {
            tracing::error!("classic layout requires a normalized gamepad mapping");
            return std::process::ExitCode::FAILURE;
        }
        let neutral = pad_neutral(&pad, args.deadzone);
        let activity = button_activity || chirp_pressed || !neutral;
        gate.expire(Instant::now());
        if gate.observe(neutral, activity) {
            let requested_at = Instant::now();
            match control::call(
                &mut stream,
                &mut next_id,
                proto::HumanControlAction::Acquire,
            ) {
                Ok(result) => {
                    gate.acquired(&result, requested_at);
                    if gate.driving() {
                        match request(&mut stream, &mut next_id, &proto::Call::RobotPolicies) {
                            Ok(response) => match policies_homed(response) {
                                Ok(homed) => up = homed,
                                Err(e) => {
                                    tracing::error!(error = %e, "cannot synchronize robot home state");
                                    gate.release();
                                }
                            },
                            Err(e) => {
                                tracing::error!(error = %e, "policies request failed");
                                return std::process::ExitCode::FAILURE;
                            }
                        }
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
            if let Some(precision) = precision.as_mut() {
                precision.reset();
                precision.awaiting_completion = false;
                if neutral {
                    let now = precision.epoch.elapsed();
                    precision.gesture.step(now, [0.0; 3], [0.0; 3]);
                }
            }
            start = HoldButton::default();
            select = HoldButton::default();
            chirp_trigger = ChirpTrigger::default();
            prev_lt = 0.0;
            imu_reference = None;
            if let Some(tap) = tap.as_ref() {
                tap.watch(&pad);
                tap.imu_control(imu_head_cfg.enabled);
            }
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
        walk_speed = adjust_speed(
            walk_speed,
            linear_direction,
            WALK_SPEED_STEP_MPS,
            args.max_linear,
        );
        turn_speed = adjust_speed(
            turn_speed,
            turn_direction,
            WALK_TURN_SPEED_STEP_RAD_S,
            args.max_angular,
        );
        if linear_direction != 0 || turn_direction != 0 {
            tracing::info!(walk_speed, turn_speed, "walking speed limits");
        }

        if !driving {
            tracing::warn!(pad = pad.name(), "pad connected — driving");
            driving = true;
        }

        // Every tick rather than on the transition above: a pad that drops and comes back between
        // two ticks never clears `driving`, and it comes back as a different event node often
        // enough that a tap following the old one would report the rest of the session as silence.
        if let Some(tap) = tap.as_ref() {
            tap.watch(&pad);
            // Every tick, like the bindings: switching the feature on in the config has to start
            // the IMU reader without a restart, and off has to let it go.
            tap.imu_control(imu_head_cfg.enabled);
        }

        // The pad's attitude this tick, when the feature is on and the pad has an IMU that has
        // said something believable. `None` is every other case, and head + move then poses the
        // head from the right stick.
        let attitude = if imu_head_cfg.enabled {
            tap.as_ref().and_then(|tap| tap.attitude())
        } else {
            None
        };
        if attitude.is_none() && imu_reference.is_some() {
            // The feature went off, or the IMU went away under us. Not silent: a head that stops
            // following mid-turn wants a line in the journal saying why.
            tracing::info!("IMU head control off — the right stick poses the head");
            imu_reference = None;
        }

        // Start: a tap stands the robot up, then toggles the policy; held, it is the way home.
        let mut go_home = false;
        match start.tick(
            pad.is_pressed(Button::Start),
            start_released,
            tick,
            &[HOME_HOLD],
        ) {
            // Start's one threshold acts when it is reached; its release says nothing more.
            HoldAction::Nothing | HoldAction::ReleasedAfter(_) => {}
            HoldAction::Reached(_) => {
                match explicit_low_voltage_rearm(&mut stream, &mut next_id, &mut gate) {
                    Ok(true) => {}, Ok(false) => continue,
                    Err(error) => { tracing::warn!(%error, "low-voltage recovery result unknown; reconnecting and requiring neutral controls"); return std::process::ExitCode::FAILURE; }
                }
                go_home = true;
            }
            HoldAction::Tap if !up => {
                match explicit_low_voltage_rearm(&mut stream, &mut next_id, &mut gate) {
                    Ok(true) => {}, Ok(false) => continue,
                    Err(error) => { tracing::warn!(%error, "low-voltage recovery result unknown; reconnecting and requiring neutral controls"); return std::process::ExitCode::FAILURE; }
                }
                if let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true)
                {
                    tracing::error!(%error, "precision cancellation failed");
                    return std::process::ExitCode::FAILURE;
                }
                tracing::warn!("Start — robot.init: standing up. Press Start again to drive");
                match request(&mut stream, &mut next_id, &proto::Call::RobotInit) {
                    Err(e) => {
                        tracing::error!(error = %e, "init failed");
                        return std::process::ExitCode::FAILURE;
                    }
                    Ok(response) => up = intent_accepted(response),
                }
            }
            HoldAction::Tap => {
                match explicit_low_voltage_rearm(&mut stream, &mut next_id, &mut gate) {
                    Ok(true) => {}, Ok(false) => continue,
                    Err(error) => { tracing::warn!(%error, "low-voltage recovery result unknown; reconnecting and requiring neutral controls"); return std::process::ExitCode::FAILURE; }
                }
                if let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true)
                {
                    tracing::error!(%error, "precision cancellation failed");
                    return std::process::ExitCode::FAILURE;
                }
                // The robot owns the toggle. A local on/off belief here drifts from the
                // robot's the moment anything else moves it — robot.relax, the shutdown
                // sequence, either side restarting — and a stale belief turns Start into a
                // button that does nothing every other press. `toggle` flips the robot's own
                // state; turning OFF returns it to the home pose (the prototype's "returning
                // to default pose"), so turning on always starts the policy from home.
                let call = proto::Call::RobotEnable(proto::EnableParams {
                    on: false,
                    toggle: true,
                });
                match request(&mut stream, &mut next_id, &call) {
                    Err(e) => {
                        tracing::error!(error = %e, "enable failed");
                        return std::process::ExitCode::FAILURE;
                    }
                    Ok(response) => {
                        // The robot names the state it ended in; that is the log, since padd
                        // no longer has a belief of its own to report.
                        let outcome = response
                            .and_then(|r| r.result_as::<proto::IntentResult>().ok())
                            .and_then(|r| r.reason)
                            .unwrap_or_else(|| "toggled".to_owned());
                        tracing::warn!(%outcome, "policy");
                    }
                }
            }
        }

        if go_home {
            if let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true) {
                tracing::error!(%error, "precision cancellation failed");
                return std::process::ExitCode::FAILURE;
            }
            // Everything back: the sticks to plain driving, the policy off, and the robot ramped
            // to its home pose with torque on — from limp, from mid-walk or from a crouch alike.
            // The disable comes first so the policy cannot pick the robot up again the moment the
            // ramp ends, which is what `robot.init` alone does on a robot that is driving.
            tracing::warn!("Start held — home pose, motors stiff, policy off");
            for call in mode_exit_calls(mode, Mode::Drive) {
                if let Err(e) = notify(&mut stream, &call) {
                    tracing::error!(error = %e, "send failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
            mode = Mode::Drive;
            imu_reference = None;
            let disable = proto::Call::RobotEnable(proto::EnableParams {
                on: false,
                toggle: false,
            });
            for call in [disable, proto::Call::RobotInit] {
                match request(&mut stream, &mut next_id, &call) {
                    Ok(response) => {
                        if matches!(call, proto::Call::RobotInit) {
                            up = intent_accepted(response);
                        }
                    }
                    Err(e) => {
                        tracing::error!(error = %e, "home request failed");
                        return std::process::ExitCode::FAILURE;
                    }
                }
            }
        }

        if let Some(next) = wanted_mode {
            if next != mode {
                if let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true)
                {
                    tracing::error!(%error, "precision cancellation failed");
                    return std::process::ExitCode::FAILURE;
                }
                if let Err(e) = notify(
                    &mut stream,
                    &proto::Call::RobotMove(proto::MoveParams::default()),
                ) {
                    tracing::error!(error = %e, "mode change zero failed");
                    return std::process::ExitCode::FAILURE;
                }
                for call in mode_exit_calls(mode, next) {
                    if let Err(e) = notify(&mut stream, &call) {
                        tracing::error!(error = %e, "send failed");
                        return std::process::ExitCode::FAILURE;
                    }
                }
                mode = next;
                tracing::info!(?mode, "mode");
            }
            // Head + move follows the pad's IMU when it can, from wherever the pad is at the
            // press — and pressing it again re-centres, which is how a person beats the gyro's
            // yaw drift without a magnetometer.
            imu_reference = if mode == Mode::HeadDrive {
                attitude
            } else {
                None
            };
            if mode == Mode::HeadDrive {
                if imu_reference.is_some() {
                    tracing::info!("IMU head control: following the pad from here");
                } else if imu_head_cfg.enabled && tap.as_ref().is_some_and(|tap| tap.has_imu()) {
                    // The IMU is there and has not spoken yet — a second after connecting,
                    // typically. Saying so beats silently doing the other thing.
                    tracing::warn!(
                        "the pad's IMU has no attitude yet; the right stick has the head this once"
                    );
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
            let skill = bindings.skill(button).unwrap_or_default();
            if skill.is_empty() {
                tracing::debug!(button, "no skill bound");
                continue;
            }
            let call = proto::Call::RobotDo(proto::DoParams {
                skill: skill.to_owned(),
            });
            match request(&mut stream, &mut next_id, &call) {
                Ok(response) => {
                    if intent_accepted(response)
                        && let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true)
                    {
                        tracing::error!(%error, "precision cancellation failed");
                        return std::process::ExitCode::FAILURE;
                    }
                }
                Err(error) => {
                    tracing::error!(%error, "skill request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        // X held: keep a chaining skill going. The robot starts another when a request lands
        // near the end of the current one, so "held" is spelled "resent every tick" — as a
        // notification, because fifty answered requests a second would spend their time waiting
        // on replies, and the press above already got the real answer.
        //
        // Whatever X is bound to: a skill that does not chain simply refuses
        // the resend, which costs a notification nobody reads. Only X, because it is the button
        // the prototype held the roulade on.
        let held = bindings.skill("x").unwrap_or_default();
        if pad.is_pressed(Button::West)
            && !pressed.contains(&"x")
            && !held.is_empty()
            && let Err(e) = notify(
                &mut stream,
                &proto::Call::RobotDo(proto::DoParams {
                    skill: held.to_owned(),
                }),
            )
        {
            tracing::error!(error = %e, "send failed");
            return std::process::ExitCode::FAILURE;
        }

        // Select: nothing on a tap. Let go between two and four seconds, a rest; held to four, a
        // power-off. Both sit and ease into the rest pose before torque goes, which is why the
        // rest waits for the release: until then the same hold may still become a power-off.
        match select.tick(
            pad.is_pressed(Button::Select),
            select_released,
            tick,
            &[REST_HOLD, SHUTDOWN_HOLD],
        ) {
            HoldAction::ReleasedAfter(0) => {
                if let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true)
                {
                    tracing::error!(%error, "precision cancellation failed");
                    return std::process::ExitCode::FAILURE;
                }
                tracing::warn!(
                    "Select released — robot.rest: sit, rest pose, then torque off and servo reboot"
                );
                // A reboot at the end rather than a bare relax: a servo that tripped its overload
                // comes back with it, so this is also the way out of a tripped servo without
                // pulling the battery. The robot ends limp, so the next Start stands it up again
                // rather than toggling the policy on a robot that is lying on the floor.
                match request(&mut stream, &mut next_id, &proto::Call::RobotRest) {
                    Ok(response) => {
                        if intent_accepted(response) {
                            up = false;
                        }
                    }
                    Err(e) => {
                        tracing::error!(error = %e, "rest request failed");
                        return std::process::ExitCode::FAILURE;
                    }
                }
            }
            HoldAction::Nothing | HoldAction::ReleasedAfter(_) => {}
            HoldAction::Tap => {
                tracing::info!("Select tapped — hold it 2 s to rest, 4 s to power off")
            }
            HoldAction::Reached(0) => {
                tracing::warn!("Select held 2 s — let go to rest, keep holding to power off")
            }
            HoldAction::Reached(_) => {
                if let Err(error) = reset_precision(&mut precision, &mut stream, &mut next_id, true)
                {
                    tracing::error!(%error, "precision cancellation failed");
                    return std::process::ExitCode::FAILURE;
                }
                tracing::warn!("Select held on — asking the robot to power off");
                match request(&mut stream, &mut next_id, &proto::Call::RobotShutdown) {
                    Ok(response) => {
                        if intent_accepted(response) {
                            up = false;
                        }
                    }
                    Err(e) => {
                        tracing::error!(error = %e, "shutdown request failed");
                        return std::process::ExitCode::FAILURE;
                    }
                }
            }
        }

        let deadzone = |v: f32| {
            let v = v as f64;
            if v.abs() < args.deadzone { 0.0 } else { v }
        };
        let left_x = deadzone(pad.value(Axis::LeftStickX));
        let left_y = deadzone(pad.value(Axis::LeftStickY));
        let right_x = deadzone(pad.value(Axis::RightStickX));
        let right_y = deadzone(pad.value(Axis::RightStickY));

        // Either trigger opens the mouth; the larger value wins.
        let trigger = |b: Button| pad.button_data(b).map(|d| d.value()).unwrap_or(0.0) as f64;
        let rt = trigger(Button::RightTrigger2);
        let lt = trigger(Button::LeftTrigger2);
        let (mouth, sound_calls) = mouth_trigger_commands(lt, rt, chirp_pressed, prev_lt);
        if let Err(e) = notify(
            &mut stream,
            &proto::Call::RobotMouth(mouth),
        ) {
            tracing::error!(error = %e, "send failed");
            return std::process::ExitCode::FAILURE;
        }
        prev_lt = lt;
        for params in sound_calls.into_iter().flatten() {
            if let Err(e) = notify(&mut stream, &proto::Call::RobotSound(params)) {
                tracing::error!(error = %e, "send failed");
                return std::process::ExitCode::FAILURE;
            }
        }

        let walk_drive = walking_limits(&drive, &args, walk_speed, turn_speed);
        let limits = DriveLimits {
            roller,
            drive: &walk_drive,
        };

        // This tick's continuous intents, as one frame. Reused rather than built fresh:
        // a `Vec` per tick is an allocation fifty times a second to say what the sticks
        // were doing, which is the shape of thing this loop is meant not to do.
        frame.clear();
        match mode {
            Mode::Drive => frame.push(proto::Call::RobotMove(limits.walk(left_y, left_x, right_x))),
            Mode::HeadDrive => match (imu_reference, attitude) {
                // The pad's tilt has the head, so the sticks keep the whole drive mapping. In the
                // same frame: the pad's tilt and the sticks describe one instant.
                (Some(reference), Some(now)) => {
                    frame.push(proto::Call::RobotMove(limits.walk(left_y, left_x, right_x)));
                    frame.push(proto::Call::RobotHead(head_from_pad(
                        pad_imu::relative(reference, now),
                        imu_head_cfg.gain,
                        args.max_head,
                    )));
                }
                // Left stick walks and turns — no strafe, the right stick is busy — and the right
                // stick looks around, with the signs head mode uses for its left stick.
                _ => {
                    frame.push(proto::Call::RobotMove(limits.walk(left_y, 0.0, left_x)));
                    frame.push(proto::Call::RobotHead(proto::HeadParams {
                        neck_pitch: 0.0,
                        head_pitch: -right_y * args.max_head,
                        head_yaw: -right_x * args.max_head,
                        head_roll: 0.0,
                    }));
                }
            },
            Mode::Head => {
                // The body must not keep its last velocity while the sticks are posing the
                // head. The deadman would catch it eventually; a robot that keeps walking
                // because you started moving its head is a bad enough surprise to be
                // explicit about.
                //
                // In the same frame as the head rather than a notification of its own: the
                // two describe one instant, and sending them separately was two `write_all`
                // and two `flush` syscalls a tick to say so.
                frame.push(proto::Call::RobotMove(proto::MoveParams::default()));
                // The prototype's alpha mapping, signs included (its head_pitch/head_yaw
                // joint axes are inverted relative to stick direction — verified on
                // hardware there, kept verbatim here).
                frame.push(proto::Call::RobotHead(proto::HeadParams {
                    neck_pitch: right_y * args.max_head,
                    head_pitch: -left_y * args.max_head,
                    head_yaw: -left_x * args.max_head,
                    head_roll: right_x * args.max_head,
                }));
            }
            Mode::BodyPose => {
                frame.push(proto::Call::RobotMove(proto::MoveParams::default()));
                frame.push(proto::Call::RobotPose(proto::PoseParams {
                    z: left_y
                        * if left_y >= 0.0 {
                            BODY_MAX_Z_UP
                        } else {
                            BODY_MAX_Z_DOWN
                        },
                    // No forward/back tilt here: the right stick has the head. The side lean
                    // keeps the old body mode's sign, moved from the right stick to the left.
                    pitch: 0.0,
                    roll: left_x * BODY_MAX_ANGLE,
                    active: true,
                }));
                // The same look-around as head + move, so the right stick means one thing
                // wherever it poses the head.
                frame.push(proto::Call::RobotHead(proto::HeadParams {
                    neck_pitch: 0.0,
                    head_pitch: -right_y * args.max_head,
                    head_yaw: -right_x * args.max_head,
                    head_roll: 0.0,
                }));
            }
        }

        let mut precision_start = None;
        let mut force_frame = false;
        if precision.is_some() {
            let (mapping, axes) = precision_axes(
                mode,
                imu_reference.is_some() && attitude.is_some(),
                roller,
                left_y,
                left_x,
                right_x,
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
                force_frame = precision.awaiting_completion
                    || out.replayed
                    || precision_start.is_some();
            } else {
                force_frame = precision.awaiting_completion;
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

/// How full deflection maps to velocity, for the modes that walk.
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
