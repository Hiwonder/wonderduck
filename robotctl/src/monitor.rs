//! `robotctl monitor` — the live view of the control loop.
//!
//! Two renderings of one stream, chosen by where stdout goes:
//!
//!  - **a terminal**: a frame that repaints in place, so a number that matters can be
//!    *watched* rather than reconstructed from a thousand scrolled lines. Joint tracking
//!    lives here, which is the reason this exists: fifteen measured angles beside fifteen
//!    commanded ones is unreadable as text at 10 Hz, and is obvious as fifteen bars.
//!  - **anything else** — a pipe, a file, `--json`: one line per tick, exactly as before.
//!    `robotctl monitor > log` and `| grep` must keep working, and a screen-painting CLI
//!    that writes escape codes into a log file is a CLI nobody can script.
//!
//! The stream is read on its own thread. The socket read blocks, terminal events do not
//! arrive on the socket, and a UI that can only notice a keystroke when the robot happens
//! to send a frame stops responding the moment the robot does.

use std::collections::{HashMap, HashSet, VecDeque};
use std::io::BufRead;
use std::path::Path;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, TryRecvError};
use std::thread;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use ratatui::DefaultTerminal;
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::layout::{Alignment, Constraint, Layout};
use ratatui::style::{Color, Modifier, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{
    Block, Cell, Paragraph, RenderDirection, Row, Sparkline, Table, TableState,
};

use crate::{Client, Failure, camera, duck, exit, frame, imu_view, path_map};

/// Tracking error at the edge of a deviation bar, radians.
///
/// A display scale, **not** a limit: nothing refuses a joint for exceeding it. Sized so the
/// bar spends its time in the middle at rest and swings visibly while walking — a bar that
/// is always saturated and one that never moves are equally uninformative.
///
/// Kept in radians whatever the display is set to: it is compared against a wire value, and a
/// constant that changes unit with a keypress is a threshold nobody can reason about.
const BAR_FULL_SCALE: f64 = 0.20;

/// Half-width of a deviation bar, in cells. The bar is `2 * HALF + 1` wide: zero has a
/// column of its own, because "no error" must not look like "a little to the left".
const BAR_HALF: usize = 12;

/// How much loop-rate history the trace keeps. More than any terminal is wide, so the
/// window is bounded by the display rather than by this number.
const TRACE_SAMPLES: usize = 600;

/// Redraw at least this often even with no new frame, so a stalled stream is visibly
/// stalled — the age in the header has to keep counting up.
const IDLE_REDRAW: Duration = Duration::from_millis(250);

/// Rows the robot block occupies: two borders, a four-row half for the command and the IMU
/// side by side, then the limits, the head, the odometry and the power row. Fixed, because a
/// header that grows when a limit appears would shift every joint row down at the moment the
/// reader is staring at one.
const HEADER_HEIGHT: u16 = 10;

/// Request id for the subscribe call, so its answer can be told apart from the stream that
/// follows it on the same connection.
const SUBSCRIBE_ID: u64 = 1;

/// Rows the pad block occupies while it is open: two borders, the cadence, the gap trace, three
/// rows of axes and the buttons.
///
/// Fixed, like the header and for the same reason — but *only while open*. Toggling it is a
/// deliberate act, and everything below moving then is what the reader asked for.
const PAD_HEIGHT: u16 = 8;

/// Rows the pad block grows by while the pad has an IMU: two borders and ten rows of a
/// wireframe pad tilting with the real one, beside the numbers. Ten because the picture is drawn
/// two pixels a row and a pad needs about twenty to read as one.
///
/// Only while one is attached. An Xbox pad has none and its block is exactly what it was; the
/// Pro Controller clones have one and the block opens up the moment `padd` finds it — which is the
/// right moment for the reader to learn the pad can do that at all.
const IMU_HEIGHT: u16 = 12;

/// How often IMU samples alone may repaint the frame.
///
/// The clone sends six hundred a second, and a terminal redrawn at that rate is a terminal doing
/// nothing else. Thirty a second is smooth to the eye and leaves the CPU for the filter.
const IMU_REPAINT: Duration = Duration::from_millis(33);

/// Rows the ToF block occupies while it is open: two borders and the eight rows
/// of an 8×8 frame.
///
/// Ten rows is a lot, and it is what the sensor is: an 8×8 matrix drawn as
/// anything less is a matrix with rows missing. Closed by default for that
/// reason, like the pad block, and everything below it moves only when the reader
/// asks for it with `t`.
const TOF_HEIGHT: u16 = 10;

/// Rows the camera block occupies while it is open: two borders and fourteen rows of picture.
///
/// Sixteen rows is a lot, and it buys a picture 28 pixels tall — which, after the quarter turn
/// this camera is mounted at, is 16 wide. That is small, and it is enough for what the block is
/// for: where the head is pointing, whether the room is lit, whether the lens is covered. Fewer
/// rows and it stops answering those; more and it is the whole terminal.
///
/// Fixed, like the pad and ToF blocks and for the same reason — but only while open, and opening
/// it is a deliberate act.
const CAMERA_HEIGHT: u16 = 16;

/// Between frames while the camera block is open.
///
/// Twice a second, deliberately, and not the camera's 30: every request makes `mediad` copy 1.84
/// MiB off the capture branch that it would otherwise drop unread, and a picture of a room does
/// not change faster than a person can look at it.
const CAMERA_PERIOD: Duration = Duration::from_millis(500);

/// Before asking again after a failed request — a stopped `mediad`, or a camera that did not
/// produce a frame in time. Long enough that a duck with no camera is not answering a connect
/// twice a second for as long as the block is open.
const CAMERA_RETRY: Duration = Duration::from_secs(2);

/// Rows of axis cells the pad block draws.
///
/// Three fits every axis of an Xbox pad from 80 columns up, and the caption says how many were left
/// out when they do not fit — a grid that stops where the room ran out, with nothing saying so, is
/// a pad drawn as though it had fewer axes than it has.
const AXIS_ROWS: usize = 3;

/// Columns one axis cell needs: six for the name, nine of bar, eight for the raw value, and the
/// space that separates it from the cell before it.
const AXIS_CELL: u16 = 24;

/// Full height of the gap trace, milliseconds.
///
/// The point where a late report starts to be felt, **not** the deadman. Drawn to the deadman a
/// healthy link is a blank row — an 8 ms gap is 1.6% of 500 and rounds away — and a trace that is
/// empty while everything works cannot be told from one that is not being fed. At this scale an
/// ordinary report is a low bar, sticky is full height, and a stall past the deadman saturates: its
/// size is on the cadence line, where a number can say 620 without needing 620 rows.
const GAP_FULL_SCALE_MS: u64 = proto::pad_link::NOTABLE_MS;

/// How many report gaps the pad trace keeps. As with the loop trace, more than any terminal is
/// wide, so the window is bounded by the display.
const GAP_SAMPLES: usize = 600;

/// How long to wait before reaching for `padd`'s tap again.
///
/// Longer than the socket's own restart delay would be pointless precision — `padd.service` waits
/// five seconds between attempts, so this will usually catch it on the second or third try — and
/// short enough that a pad tap started after the monitor turns up while somebody is still watching.
const PAD_RETRY: Duration = Duration::from_secs(2);

/// How often `robotd` is asked how it is, and how long before a failed poll is retried.
///
/// A poll rather than a subscription because that is the shape `robot.health` has, and cheap
/// enough to be one: the far side is a handful of atomic loads. Two seconds is chosen for the
/// slowest thing on the answer — the battery, which `robotd` reports as a ~10 s EMA — so a
/// faster poll would redraw the same number.
const HEALTH_POLL: Duration = Duration::from_secs(2);

/// How long a health answer may be the newest one before the row says how old it is.
///
/// Three polls. `robotd` closing the socket is reported by the poller itself; this catches the
/// other shape — a daemon that accepts, answers once and then goes quiet — where the pack's
/// charge would otherwise sit on screen looking current forever.
const HEALTH_STALE: Duration = Duration::from_secs(6);

/// Battery percentages the reading is coloured at.
///
/// Not display taste: 0% is `duck_control::model::BATTERY_EMPTY_V`, which is the voltage at
/// which `robotd` sits the robot down and cuts power. The percentage is a countdown to that, so
/// the last fifth of it is worth seeing from across a room.
const BATTERY_LOW_PCT: f64 = 30.0;
const BATTERY_CRITICAL_PCT: f64 = 15.0;

/// Request id for the health poll. Its own connection, but matched rather than assumed: a
/// `robotd` newer than this build may say things on it this one has no use for.
const HEALTH_ID: u64 = 2;

/// Columns the tables keep for themselves before the 3D robot view gets any: the joints
/// table with its bar, plus borders and slack. The view only ever eats width the tables
/// were not using, so opening it never truncates a number.
const DUCK_MAIN_MIN: u16 = 84;

/// The robot view's width, columns. Below the minimum a duck is a smudge, so the view
/// silently waits for a wider terminal rather than drawing one; past the maximum the
/// extra cells stop adding legibility and the tables can breathe instead.
const DUCK_MIN_WIDTH: u16 = 26;
const DUCK_MAX_WIDTH: u16 = 74;

/// Rows the 3D view keeps for itself before the path map may take its slice —
/// below this the duck is a smudge and the map would be the thing that did it.
const DUCK_MIN_HEIGHT: u16 = 18;

/// The path map's rows, borders included. Fixed: growing the panel with the
/// path would push the 3D view around; the map zooms its world out instead.
const PATH_HEIGHT: u16 = 12;

/// Gaps the cadence figure is averaged over.
///
/// A rate from the whole history answers "how has this link been", which the trace already shows.
/// The number beside it should answer "how is it now", and a second of reports is what a hand on a
/// stick can feel.
const CADENCE_WINDOW: usize = 100;

/// Which unit the angles on screen are drawn in.
///
/// The wire is radians and stays radians: [`proto::RobotState`] carries nothing else, and the
/// piped rendering keeps them, because a script parsing that output must not have its numbers
/// change under it. This is a reading aid for the live view alone — a hip at `-0.52` and a hip
/// at `-30°` are the same joint, and only one of them can be pictured without arithmetic.
///
/// Radians are still one keypress away rather than gone, because they are what every other
/// surface speaks: the protocol docs, a policy's own inputs, and the numbers a client sends.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Units {
    Degrees,
    Radians,
}

impl Units {
    /// The other one. There are exactly two, so this is the whole of the `u` keybinding.
    fn toggled(self) -> Self {
        match self {
            Self::Degrees => Self::Radians,
            Self::Radians => Self::Degrees,
        }
    }

    /// One wire angle as text.
    ///
    /// Two decimals of a degree is finer than three of a radian, so the display resolves at
    /// least as much either way — flipping the unit must not quietly hide a difference the
    /// other unit was showing.
    fn angle(self, radians: f64) -> String {
        match self {
            Self::Degrees => format!("{:+.2}°", radians.to_degrees()),
            Self::Radians => format!("{radians:+.3}"),
        }
    }

    /// The same, for a rate: the twist's yaw is per second.
    fn rate(self, radians_per_second: f64) -> f64 {
        match self {
            Self::Degrees => radians_per_second.to_degrees(),
            Self::Radians => radians_per_second,
        }
    }

    fn rate_unit(self) -> &'static str {
        match self {
            Self::Degrees => "°/s",
            Self::Radians => "rad/s",
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::Degrees => "degrees",
            Self::Radians => "radians",
        }
    }

    /// The deviation bar's full scale, said in whatever is on screen. The bar is drawn from a
    /// ratio, so only its caption has a unit at all.
    fn bar_scale(self) -> String {
        match self {
            Self::Degrees => format!("±{:.1}°", BAR_FULL_SCALE.to_degrees()),
            Self::Radians => format!("±{BAR_FULL_SCALE:.2} rad"),
        }
    }
}

/// What a reader thread has to say.
enum Update {
    State(Box<proto::RobotState>),
    /// The subscribe acknowledgement, which names the policy this `robotd` is running.
    Policy(Box<proto::SubscribeResult>),
    /// The stream ended. Carries the sentence to exit with.
    Ended(String),
    /// One report from `padd`'s raw pad tap.
    Pad(Box<proto::PadReport>),
    /// The tap is not there, or stopped.
    ///
    /// **Not fatal, unlike [`Self::Ended`].** The pad tap is a second connection to a second
    /// daemon, and a monitor that quit because `padd` was stopped would be a monitor that stops
    /// working on every robot nobody has paired a pad to.
    PadLost(String),
    /// One depth frame from `tofd`.
    Tof(Box<proto::TofFrame>),
    /// `tofd`'s answer to the subscription: which sensor, or why there is none.
    TofStatus(Box<proto::TofStreamResult>),
    /// The depth stream is not there, or stopped. Not fatal, for the same reason
    /// [`Self::PadLost`] is not: it is a third daemon on a third socket, and most
    /// ducks have no ToF fitted at all.
    TofLost(String),
    /// One frame off `mediad`'s local snapshot endpoint, asked for because the camera block is
    /// open. Nothing arrives while it is closed — see [`read_camera`].
    Camera(Box<camera::Shot>),
    /// Why there is no picture. Not fatal, for the reason [`Self::TofLost`] is not: it is a
    /// connection of its own to a daemon the state stream does not come from, and a duck whose
    /// `mediad` is stopped is still a duck worth watching.
    CameraLost(String),
    /// One answer to `robot.health` — the battery, the temperatures, the bus counters and the
    /// IMU's staleness, none of which is on the state stream.
    Health(Box<proto::HealthResult>),
    /// The health poll is not answering. Not fatal either: it is a connection of its own, and
    /// the state stream ending is what [`Self::Ended`] is for.
    HealthLost(String),
}

/// Subscribe to `robot.state` and render it until interrupted.
///
/// Never returns `Ok` on its own: `q`/`Ctrl-C` is the exit in the live view, Ctrl-C alone in
/// the piped one. A closed socket is an error either way — that is what `robotd` restarting
/// mid-update looks like, and it is worth seeing rather than hanging through.
pub fn run(
    robot_socket: &Path,
    pad_socket: &Path,
    tof_socket: &Path,
    media_socket: &Path,
    hz: u32,
    json: bool,
) -> Result<(), Failure> {
    let subscribe = || {
        proto::Request::call(
            proto::Id::Number(SUBSCRIBE_ID),
            &proto::Call::RobotSubscribe(proto::SubscribeParams {
                hz: (hz > 0).then_some(hz),
                ..Default::default()
            }),
        )
    };

    // A pipe or `--json` is a stream *of robot state*, so there it stays an error: emitting
    // nothing forever is not a useful answer to `monitor > log`.
    if json || !stdout_is_a_terminal() {
        let mut client = Client::connect_to("robotd", robot_socket)?;
        client.send(&subscribe())?;
        return stream_lines(client, json);
    }

    // The live view does **not** require a robot.
    //
    // `padd`'s tap is a second daemon on a second socket, and the pad is worth watching on a board
    // whose servos are unpowered, whose `robotd` is stopped, or which is being bisected — exactly
    // the boards someone reaches for this on. Refusing to open at all made the pad block reachable
    // only where it was least needed.
    //
    // The reason is carried rather than discarded, so the frame says which of "no robotd" and "no
    // state yet" it is looking at instead of showing one sentence for both.
    let (robot, no_robot) = match Client::connect_to("robotd", robot_socket) {
        Ok(mut client) => match client.send(&subscribe()) {
            Ok(()) => (Some(client), None),
            Err(e) => (None, Some(e.message)),
        },
        Err(e) => (None, Some(e.message)),
    };

    let (tx, rx) = mpsc::channel();
    let pad_tx = tx.clone();
    let camera_tx = tx.clone();
    let tx_for_tof = tx.clone();
    let health_tx = tx.clone();
    let pad_socket = pad_socket.to_path_buf();

    // Held for as long as the view lives, not dropped: it is the write half of the subscription,
    // and closing it tells `robotd` this client has gone away, which would end the stream being
    // rendered.
    let _writer = robot.map(|client| {
        let Client { reader, writer, .. } = client;
        thread::spawn(move || read_states(reader, &tx));
        writer
    });

    // Its own thread and its own connection, for the same reason the robot's stream has one: a
    // blocking read on either socket must not be able to hold up the other, and `padd`'s tap goes
    // quiet for minutes at a time whenever nobody is touching the sticks.
    thread::spawn(move || read_pad(&pad_socket, &pad_tx));

    // A third connection to a third daemon, and the same reasoning again: `tofd`
    // is on most boards and its sensor is on few, so neither its absence nor its
    // silence may hold up the two streams that are always there.
    let tof_tx = tx_for_tof;
    let tof_socket = tof_socket.to_path_buf();
    thread::spawn(move || read_tof(&tof_socket, &tof_tx));

    // A fourth connection — to the daemon the first one is already streaming from. The battery,
    // the temperatures and the bus counters answer `robot.health`, which is a *call*, and the
    // stream's reader thread owns every line that arrives on its socket. So: its own connection,
    // its own thread, polled.
    //
    // It runs even when there is no robot to stream from, which is the case it matters most in:
    // a board whose servo power is off never completes a control tick, so no state ever arrives,
    // and the reason why is on this answer and nowhere else.
    let health_socket = robot_socket.to_path_buf();
    thread::spawn(move || poll_health(&health_socket, &health_tx));

    // The camera has no reader thread yet, and deliberately: `media.frame` is a *request*, and one
    // is only made while the block is open. `live` owns the toggle, so it owns the thread.
    let mut terminal = ratatui::init();
    let outcome = live(&mut terminal, &rx, camera_tx, hz, no_robot, media_socket);
    ratatui::restore();
    outcome
}

/// One line per tick, for a pipe, a file, or `--json`.
fn stream_lines(mut client: Client, json: bool) -> Result<(), Failure> {
    let mut line = String::new();
    loop {
        line.clear();
        let read = client
            .reader
            .read_line(&mut line)
            .map_err(|e| Failure::new(exit::UNREACHABLE, format!("stream ended: {e}")))?;
        if read == 0 {
            return Err(Failure::new(
                exit::UNREACHABLE,
                "robotd closed the connection".to_owned(),
            ));
        }

        let Ok(request) = serde_json::from_str::<proto::Request>(&line) else {
            continue;
        };
        let Some(state) = request.as_state() else {
            // The subscribe acknowledgement, or anything else this client does not model.
            continue;
        };

        if json {
            println!("{}", line.trim_end());
            continue;
        }

        let limits = if state.movement.limited_by.is_empty() {
            String::new()
        } else {
            format!("  [{}]", state.movement.limited_by.join(","))
        };
        // Gravity and gain sit next to the fall verdict on purpose: `fallen` is derived from
        // the first and overrides the second, and reading the verdict without its input made
        // "the robot is down" indistinguishable from "the IMU frame is wrong".
        println!(
            "{:8.2}  {:>5}  {:5.1}Hz miss={:<4} {}  g[{:+.2} {:+.2} {:+.2}] kp={:<4} \
             req[{:+.2} {:+.2} {:+.2}] app[{:+.2} {:+.2} {:+.2}]{}",
            state.t,
            state.policy,
            state.control_loop.hz,
            state.control_loop.missed,
            if state.safety.fallen {
                "FALLEN"
            } else {
                "ok    "
            },
            state.safety.gravity[0],
            state.safety.gravity[1],
            state.safety.gravity[2],
            state
                .safety
                .gain
                .map_or_else(|| "-".to_owned(), |g| g.to_string()),
            state.movement.requested[0],
            state.movement.requested[1],
            state.movement.requested[2],
            state.movement.applied[0],
            state.movement.applied[1],
            state.movement.applied[2],
            limits,
        );
    }
}

/// Decode the stream on a thread of its own. Ends by describing how it ended, so the UI can
/// exit with the reason rather than with a channel that merely went quiet.
fn read_states(mut reader: impl BufRead, tx: &mpsc::Sender<Update>) {
    let mut line = String::new();
    loop {
        line.clear();
        let ended = match reader.read_line(&mut line) {
            Err(e) => format!("stream ended: {e}"),
            Ok(0) => "robotd closed the connection".to_owned(),
            Ok(_) => {
                if let Some(update) = decode(&line)
                    && tx.send(update).is_err()
                {
                    return; // the UI is gone
                }
                continue;
            }
        };
        let _ = tx.send(Update::Ended(ended));
        return;
    }
}

/// Read one line of the connection.
///
/// Two shapes arrive on it: the answer to `robot.subscribe`, once, and `robot.state`
/// notifications forever after. A notification carries `method`, a response does not, so the
/// two cannot be confused — and anything else is ignored rather than treated as an error,
/// because a `robotd` newer than this build may say things this one has no use for.
fn decode(line: &str) -> Option<Update> {
    if let Ok(request) = serde_json::from_str::<proto::Request>(line) {
        return request.as_state().map(|s| Update::State(Box::new(s)));
    }
    let response = serde_json::from_str::<proto::Response>(line).ok()?;
    if response.id != Some(proto::Id::Number(SUBSCRIBE_ID)) {
        return None;
    }
    // A `robotd` that predates `SubscribeResult` answered with `IntentResult`, whose
    // `accepted` field this parses and whose missing policy fields stay `None` — so an old
    // robot reports an unknown policy rather than failing to render.
    response
        .result_as::<proto::SubscribeResult>()
        .ok()
        .map(|r| Update::Policy(Box::new(r)))
}

/// Read `padd`'s raw pad tap on a thread of its own, reconnecting for as long as the view lives.
///
/// Every way this ends is a sentence for the pad block rather than an error for the process. The tap
/// is a debug facility on a second daemon: it is absent on a robot with no pad paired, absent while
/// `padd` is stopped, and absent on a release older than the one that added it — and none of those is
/// a reason for `robotctl monitor` to stop showing the robot.
///
/// **It retries, because `padd` restarting is routine rather than exceptional.** Its unit is
/// `Restart=always`, and it exits deliberately whenever `robotd`'s socket is not there — during
/// every update, for one. A reader that gave up on the first refusal would leave a monitor session
/// that outlived one restart permanently blind to the pad, with a stale sentence explaining why.
fn read_pad(socket: &Path, tx: &mpsc::Sender<Update>) {
    loop {
        // The reason is reported on every attempt rather than only when it changes: the block is
        // where it is read, and it is read whenever someone presses `p`, which may be long after.
        if let Err(why) = subscribe_to_pad(socket, tx)
            && tx.send(Update::PadLost(why)).is_err()
        {
            return; // the UI is gone
        }
        thread::sleep(PAD_RETRY);
    }
}

/// One connection to the tap, from its subscribe to whatever ended it.
fn subscribe_to_pad(socket: &Path, tx: &mpsc::Sender<Update>) -> Result<(), String> {
    let mut client = Client::connect_to("padd", socket).map_err(|e| e.message)?;
    client
        .send(&proto::Request::call(
            proto::Id::Number(SUBSCRIBE_ID),
            &proto::Call::PadInput,
        ))
        .map_err(|e| e.message)?;

    let mut line = String::new();
    loop {
        line.clear();
        return Err(match client.reader.read_line(&mut line) {
            Err(e) => format!("the pad tap stopped: {e}"),
            Ok(0) => "padd closed the pad tap".to_owned(),
            Ok(_) => match decode_pad(&line) {
                // A refusal, which `padd` gives only where it cannot read input devices at all.
                Some(Err(refused)) => refused,
                Some(Ok(report)) => {
                    if tx.send(Update::Pad(Box::new(report))).is_err() {
                        return Ok(()); // the UI is gone
                    }
                    continue;
                }
                None => continue,
            },
        });
    }
}

/// Watch `tofd`'s depth stream, retrying forever.
///
/// Same shape as [`read_pad`] and for the same reasons: `tofd` restarts (its unit
/// is `Restart=always`), and a duck with no sensor fitted still answers here —
/// with a reason, which is what the block shows instead of an empty grid.
fn read_tof(socket: &Path, tx: &mpsc::Sender<Update>) {
    loop {
        if let Err(why) = subscribe_to_tof(socket, tx)
            && tx.send(Update::TofLost(why)).is_err()
        {
            return; // the UI is gone
        }
        thread::sleep(PAD_RETRY);
    }
}

/// One connection to the depth stream, from its subscribe to whatever ended it.
fn subscribe_to_tof(socket: &Path, tx: &mpsc::Sender<Update>) -> Result<(), String> {
    let mut client = Client::connect_to("tofd", socket).map_err(|e| e.message)?;
    client
        .send(&proto::Request::call(
            proto::Id::Number(SUBSCRIBE_ID),
            &proto::Call::TofStream,
        ))
        .map_err(|e| e.message)?;

    let mut line = String::new();
    loop {
        line.clear();
        return Err(match client.reader.read_line(&mut line) {
            Err(e) => format!("the depth stream stopped: {e}"),
            Ok(0) => "tofd closed the depth stream".to_owned(),
            Ok(_) => {
                // The answer names the sensor; everything after it is a frame.
                // Both are forwarded, and anything else is skipped rather than
                // treated as an end — a future `tofd` may say more than this
                // build knows how to read.
                if let Ok(response) = serde_json::from_str::<proto::Response>(&line)
                    && let Ok(status) = response.result_as::<proto::TofStreamResult>()
                {
                    if tx.send(Update::TofStatus(Box::new(status))).is_err() {
                        return Ok(()); // the UI is gone
                    }
                    continue;
                }
                match serde_json::from_str::<proto::Request>(&line)
                    .ok()
                    .and_then(|r| r.as_tof_frame())
                {
                    Some(frame) => {
                        if tx.send(Update::Tof(Box::new(frame))).is_err() {
                            return Ok(()); // the UI is gone
                        }
                        continue;
                    }
                    None => continue,
                }
            }
        });
    }
}

/// Ask `mediad` for a picture, twice a second, and only while the camera block is open.
///
/// **The only reader here that has to be told to read.** The pad tap and the depth stream are
/// subscriptions: they cost the daemon at the other end nothing extra while nobody is looking,
/// so they run whether or not their block is open. `media.frame` is a request, and answering one
/// makes `mediad` copy 1.84 MiB off the capture branch it would otherwise drop unread — so this
/// thread parks on `asked_for` until the reader presses `c`, and parks again when they press it
/// twice.
///
/// Every failure is a sentence for the block rather than an error for the process, exactly as the
/// pad and ToF readers treat theirs: most ducks run a camera, and none of them stop being worth
/// watching because `mediad` is being restarted.
fn read_camera(socket: &Path, tx: &mpsc::Sender<Update>, asked_for: &Receiver<bool>) {
    let mut open = false;
    loop {
        if !open {
            match asked_for.recv() {
                Ok(next) => open = next,
                Err(_) => return, // the UI is gone
            }
            continue;
        }
        let asked = Instant::now();
        let (update, again) = match frame::fetch(socket) {
            Ok((header, data)) => (
                Update::Camera(Box::new(camera::Shot {
                    header,
                    data,
                    waited: asked.elapsed(),
                })),
                CAMERA_PERIOD,
            ),
            Err(why) => (Update::CameraLost(why.to_string()), CAMERA_RETRY),
        };
        if tx.send(update).is_err() {
            return; // the UI is gone
        }
        // Waiting on the line rather than sleeping on it: closing the block must stop the asking
        // now, not at the end of the period.
        match asked_for.recv_timeout(again) {
            Ok(next) => open = next,
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return,
        }
    }
}

/// Ask `robotd` how it is, for as long as the view lives.
///
/// Every way this ends is a sentence for the power row rather than an error for the process,
/// exactly as [`read_pad`] treats the tap: this is a second connection to a daemon whose state
/// stream is being rendered on the first, and a battery reading that stopped arriving is not a
/// reason to tear down a monitor that is still showing the loop.
///
/// **It reconnects**, for the reason the pad reader does: `robotd` restarts during every update,
/// and a session that outlived one would otherwise be permanently blind to the pack.
fn poll_health(socket: &Path, tx: &mpsc::Sender<Update>) {
    loop {
        if let Err(why) = ask_health(socket, tx)
            && tx.send(Update::HealthLost(why)).is_err()
        {
            return; // the UI is gone
        }
        thread::sleep(HEALTH_POLL);
    }
}

/// One connection, polled until something ends it.
///
/// The connection is kept open across polls rather than reopened per poll: a connect, a
/// handshake and an accept every two seconds for the length of a session is a lot of socket
/// churn for a number that does not move.
fn ask_health(socket: &Path, tx: &mpsc::Sender<Update>) -> Result<(), String> {
    let mut client = Client::connect_to("robotd", socket).map_err(|e| e.message)?;
    let mut line = String::new();
    loop {
        client
            .send(&proto::Request::call(
                proto::Id::Number(HEALTH_ID),
                &proto::Call::RobotHealth,
            ))
            .map_err(|e| e.message)?;

        loop {
            line.clear();
            match client.reader.read_line(&mut line) {
                Err(e) => return Err(format!("the health poll stopped: {e}")),
                Ok(0) => return Err("robotd closed the connection".to_owned()),
                // Anything that is not this poll's answer is skipped rather than treated as an
                // end, for the same reason the state stream skips what it does not model.
                Ok(_) => match decode_health(&line) {
                    None => continue,
                    // A refusal, or an answer this build cannot read. Reported once and then
                    // retried from a fresh connection — an old `robotd` that does not serve
                    // this call will say so again, which is the honest answer.
                    Some(Err(why)) => return Err(why),
                    Some(Ok(health)) => {
                        if tx.send(Update::Health(Box::new(health))).is_err() {
                            return Ok(()); // the UI is gone
                        }
                        break;
                    }
                },
            }
        }
        thread::sleep(HEALTH_POLL);
    }
}

/// The answer to one `robot.health`: `None` for anything else on the socket, `Err` for a
/// refusal or a shape this build cannot read.
fn decode_health(line: &str) -> Option<Result<proto::HealthResult, String>> {
    let response = serde_json::from_str::<proto::Response>(line).ok()?;
    if response.id != Some(proto::Id::Number(HEALTH_ID)) {
        return None;
    }
    if let Some(error) = response.error {
        return Some(Err(error.message));
    }
    Some(
        response
            .result_as::<proto::HealthResult>()
            .map_err(|e| format!("robotd answered robot.health unreadably: {e}")),
    )
}

// ── the depth matrix's cells ────────────────────────────────────────────────

/// Columns one zone takes: four for `1.42`, one to separate it from its
/// neighbour. Fixed, so the grid stays square and two columns stay comparable.
const TOF_CELL: usize = 5;

/// How long a frame may be the newest one before it is called stale. Six frames
/// at 15 Hz — long enough that a slow repaint is not an alarm, short enough that
/// a sensor which stopped shows up as one.
const TOF_STALE: Duration = Duration::from_millis(400);

/// Said in the block's border, because the two non-numeric cells are the ones a
/// reader has no way to guess.
const TOF_LEGEND: &str = " · nothing in range · x could not measure · green floor · near→far ";

/// The near-to-far colour ramp, warm to cool.
///
/// Near is warm because near is what matters on an obstacle sensor: a hand in
/// front of the head reads red, a far wall reads blue, and the eye finds the
/// close thing without reading a number. Indexed rather than named colours so the
/// steps are an actual gradient rather than whatever six things a theme calls
/// "red" through "blue"; the thresholds are the sensor's useful span, which is
/// centimetres to about four metres.
const TOF_RAMP: [(f32, u8); 7] = [
    (0.25, 196), // scarlet — inside arm's reach
    (0.50, 202),
    (0.80, 208),
    (1.20, 220),    // amber — a room away
    (1.80, 118),    // green
    (2.60, 51),     // cyan
    (f32::MAX, 33), // blue — as far as it sees
];

/// Re-exported so this file can name the three zone classes without depending on
/// the whole `tof` crate: `robotctl` talks to `tofd` over a socket, and pulling in
/// a crate that compiles two C drivers to render a grid would be a poor trade.
mod tof_zone {
    /// Mirrors `tof::Zone`. The interpretation rules live in the protocol's
    /// documentation of [`duck_ipc_proto::TofFrame`], and both ends implement
    /// them from there.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum Zone {
        Range(f32),
        NoTarget,
        Unusable(u8),
    }
}

/// ST's status codes for a usable range, and for "measured, nothing there".
const TOF_STATUS_VALID: [u8; 2] = [5, 9];
const TOF_STATUS_NO_TARGET: u8 = 255;

/// One zone of a frame, interpreted — the same rules as `tof::Frame::zone`.
///
/// Kept in step deliberately rather than shared: the wire format is the contract
/// between the two, and a `robotctl` that linked the driver crate to read a
/// number would be a client that cannot be built without a C toolchain.
fn frame_zone(frame: &proto::TofFrame, index: usize) -> tof_zone::Zone {
    let status = frame
        .status
        .get(index)
        .copied()
        .unwrap_or(TOF_STATUS_NO_TARGET);
    let distance = frame.distance_mm.get(index).copied().unwrap_or(0);
    if TOF_STATUS_VALID.contains(&status) {
        if distance > 0 {
            return tof_zone::Zone::Range(f32::from(distance) / 1000.0);
        }
        return tof_zone::Zone::Unusable(status);
    }
    if status == TOF_STATUS_NO_TARGET {
        return tof_zone::Zone::NoTarget;
    }
    tof_zone::Zone::Unusable(status)
}

/// One cell: the distance coloured by range, or a mark for the two cases that
/// have no distance.
fn tof_cell(zone: tof_zone::Zone, class: Option<kinematics::tof::Zone>) -> Span<'static> {
    match zone {
        tof_zone::Zone::Range(metres) => {
            // The reprojection's verdict beats the colour ramp: a floor return
            // is a real distance, but it is not a thing in the way, and green
            // is the difference on screen. Too-close returns recede — under
            // ~10 cm the sensor's crosstalk makes the number untrustworthy.
            match class {
                Some(kinematics::tof::Zone::Floor { .. }) => {
                    return Span::styled(
                        format!("{:>4.2} ", metres.min(9.99)),
                        Style::new().fg(Color::Green).dim(),
                    );
                }
                Some(kinematics::tof::Zone::TooClose) => {
                    return Span::raw(format!("{:>4.2} ", metres.min(9.99))).dim();
                }
                _ => {}
            }
            let colour = TOF_RAMP
                .iter()
                .find(|(limit, _)| metres < *limit)
                .map_or(33, |(_, colour)| *colour);
            // Four characters for every range the sensor can report: `0.42`
            // through `9.99`, and clamped above that so a spurious huge reading
            // cannot shift the grid sideways.
            Span::styled(
                format!("{:>4.2} ", metres.min(9.99)),
                Style::new().fg(Color::Indexed(colour)),
            )
        }
        // Dim, and not a number: empty space should recede rather than compete
        // with the thing that is actually near.
        tof_zone::Zone::NoTarget => Span::raw("   · ").dim(),
        // Magenta: not a distance, not empty, and not to be mistaken for either.
        tof_zone::Zone::Unusable(_) => Span::styled("   x ", Style::new().fg(Color::Magenta)),
    }
}

/// One line of the tap: a report, a refusal, or something this build has no use for.
fn decode_pad(line: &str) -> Option<Result<proto::PadReport, String>> {
    if let Ok(request) = serde_json::from_str::<proto::Request>(line)
        && let Some(report) = request.as_pad_report()
    {
        return Some(Ok(report));
    }
    let response = serde_json::from_str::<proto::Response>(line).ok()?;
    if response.id != Some(proto::Id::Number(SUBSCRIBE_ID)) {
        return None;
    }
    if let Some(error) = response.error {
        return Some(Err(error.message));
    }
    match response.result_as::<proto::PadInputResult>() {
        Ok(result) if !result.accepted => Some(Err(result
            .reason
            .unwrap_or_else(|| "padd refused the subscription".to_owned()))),
        // Accepted. The pad itself arrives as `PadReport::Attached`, so there is nothing to say.
        _ => None,
    }
}

/// The live view's loop: absorb whatever has arrived, honour the keyboard, repaint.
fn live(
    terminal: &mut DefaultTerminal,
    rx: &Receiver<Update>,
    tx: mpsc::Sender<Update>,
    hz: u32,
    no_robot: Option<String>,
    media_socket: &Path,
) -> Result<(), Failure> {
    // A frame every `1/hz`, so waiting a fifth of a period keeps the keyboard responsive
    // without spinning. Clamped: `--hz 1000` must not turn this into a busy loop.
    let poll = Duration::from_secs_f64(1.0 / f64::from(hz.clamp(1, 200)) / 5.0);
    let mut view = View::new(hz, no_robot);
    let mut painted = Instant::now();

    // The camera's thread, and the line this loop tells it to start and stop asking on. It sleeps
    // on that line while the block is shut, which is the whole point: a request is 1.84 MiB
    // `mediad` would otherwise never have copied.
    let (wanted, asked_for) = mpsc::channel();
    let camera_socket = media_socket.to_path_buf();
    thread::spawn(move || read_camera(&camera_socket, &tx, &asked_for));

    loop {
        let mut fresh = false;
        while event::poll(Duration::ZERO).map_err(terminal_failure)? {
            match event::read().map_err(terminal_failure)? {
                Event::Key(key) if key.kind == KeyEventKind::Press => match key.code {
                    KeyCode::Char('q') | KeyCode::Esc => return Ok(()),
                    // Ctrl-C by hand: raw mode delivers it as a keypress rather than a signal,
                    // so the key that stops every other `robotctl` command has to stop this one
                    // too.
                    KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                        return Ok(());
                    }
                    // Scrolling only does anything on a terminal too short for every joint.
                    KeyCode::Up | KeyCode::Char('k') => {
                        view.scroll_by(-1);
                        fresh = true;
                    }
                    KeyCode::Down | KeyCode::Char('j') => {
                        view.scroll_by(1);
                        fresh = true;
                    }
                    KeyCode::PageUp => {
                        view.scroll_pages(-1);
                        fresh = true;
                    }
                    KeyCode::PageDown => {
                        view.scroll_pages(1);
                        fresh = true;
                    }
                    KeyCode::Home => {
                        view.scroll_home();
                        fresh = true;
                    }
                    // Radians back, for reading a number that is about to be compared with the
                    // wire, a policy's input, or the protocol docs.
                    KeyCode::Char('u') => {
                        view.toggle_units();
                        fresh = true;
                    }
                    // The pad's own event stream. Off by default: it is worth eight rows only to
                    // someone asking about the pad, and those rows come out of the joints table.
                    KeyCode::Char('p') => {
                        view.toggle_pad();
                        fresh = true;
                    }
                    // The depth matrix. Off by default: ten rows is most of a
                    // short terminal, and only someone asking about the ToF wants
                    // them.
                    KeyCode::Char('t') => {
                        view.toggle_tof();
                        fresh = true;
                    }
                    // The camera. Off by default, and the only block that *costs* the robot
                    // something to have open, so the keypress is also what starts the asking.
                    KeyCode::Char('c') => {
                        view.toggle_camera();
                        let _ = wanted.send(view.show_camera);
                        fresh = true;
                    }
                    // The 3D robot view. On by default — it appears whenever the
                    // terminal is wide enough — so the key mostly exists to turn it off.
                    KeyCode::Char('d') => {
                        view.toggle_duck();
                        fresh = true;
                    }
                    KeyCode::Char('[') | KeyCode::Left => {
                        view.orbit_duck(-0.25);
                        fresh = true;
                    }
                    KeyCode::Char(']') | KeyCode::Right => {
                        view.orbit_duck(0.25);
                        fresh = true;
                    }
                    _ => {}
                },
                // A resize changes what fits, and the next frame may be a whole period away.
                Event::Resize(_, _) => fresh = true,
                _ => {}
            }
        }

        match rx.recv_timeout(poll) {
            Ok(update) => fresh |= view.absorb(update)?,
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => {
                return Err(Failure::new(
                    exit::UNREACHABLE,
                    "the robot.state reader stopped".to_owned(),
                ));
            }
        }
        // Drain the backlog: on a slow terminal the newest frame is the only one worth
        // drawing, and rendering a queue one frame at a time is how a view falls behind and
        // stays behind.
        loop {
            match rx.try_recv() {
                Ok(update) => fresh |= view.absorb(update)?,
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => break,
            }
        }

        if fresh || painted.elapsed() >= IDLE_REDRAW {
            terminal
                .draw(|frame| view.render(frame))
                .map_err(terminal_failure)?;
            painted = Instant::now();
        }
    }
}

fn terminal_failure(e: std::io::Error) -> Failure {
    Failure::new(exit::FAILED, format!("terminal error: {e}"))
}

/// The pad's own event stream, as the monitor accumulates it.
///
/// **Everything here is derived from raw reports, and nothing here is `padd`'s opinion.** That is
/// the point of the tap: `padd` is the process whose view of the pad is already known to be
/// misleading — it resends the last stick value at 50 Hz, so a link that has stopped delivering
/// still looks like a driver with a hand on the stick from everywhere downstream.
#[derive(Default)]
struct PadView {
    /// Why there is nothing to show, when there is nothing: no tap on this robot, `padd` stopped,
    /// or a pad that has gone away. Never left blank — a pad block that is simply empty is the one
    /// failure mode a debug view must not have.
    trouble: Option<String>,
    /// The device the reports are coming from. Every number below is read against it.
    device: Option<proto::PadInputDevice>,
    /// Where each axis is now, by evdev code. Seeded from the device so an untouched stick has a
    /// position rather than a blank.
    axes: HashMap<u16, i32>,
    held: HashSet<u16>,
    /// When the last report arrived *here*.
    ///
    /// The monitor's own clock, deliberately: [`proto::PadFrame::at_us`] is the robot's, and this
    /// view is often running on a laptop whose clock has no relationship to it. Sub-millisecond
    /// socket latency is noise against a gap that matters.
    arrived: Option<Instant>,
    reports: u64,
    /// Gaps between reports while the sticks were moving, newest first, milliseconds.
    gaps: VecDeque<u64>,
    worst_ms: u64,
    over_notable: u64,
    over_deadman: u64,
    /// Spells of silence past [`proto::pad_link::IDLE_MS`]: a pad at rest, not a link that stopped.
    quiet: u64,
    /// Reports the kernel discarded because `padd` fell behind — `SYN_DROPPED`. Says nothing about
    /// the radio, and makes the gap around it meaningless.
    after_drops: u64,
    /// Reports dropped between `padd` and here, because this monitor fell behind.
    socket_dropped: u64,
    /// Times the robot's realtime clock stepped backwards between two reports. Expected exactly
    /// once on a board with no RTC, when its first NTP reply lands.
    clock_steps: u64,
    /// The pad's inertial unit, while `padd` has one open. Its own lifecycle: a pad without one
    /// never sets this, and a pad that drops takes it away through `ImuDetached`, not `Detached`.
    /// The two paths no report ever arrives on are covered instead below: a new `Attached` clears
    /// it until the new pad's own `ImuAttached`, and a lost tap clears it with the pad.
    imu: Option<pad_imu::Imu>,
    /// When a sample last earned a repaint — see [`IMU_REPAINT`].
    imu_painted: Option<Instant>,
}

impl PadView {
    /// Has enough time passed since the last IMU-driven repaint for another one?
    fn imu_repaint_due(&mut self) -> bool {
        let now = Instant::now();
        match self.imu_painted {
            Some(at) if now.duration_since(at) < IMU_REPAINT => false,
            _ => {
                self.imu_painted = Some(now);
                true
            }
        }
    }

    /// Take one report.
    fn absorb(&mut self, report: proto::PadReport) {
        match report {
            proto::PadReport::Attached { device } => {
                // A new device is a new measurement: the counters and the trace describe *a link*,
                // and carrying the old ones over would blame this pad for the last one's stalls.
                // The IMU as well: if the new pad has one its `ImuAttached` follows, and a pad
                // without one must not inherit the last pad's panel.
                let device = *device;
                self.axes = device.axes.iter().map(|a| (a.code, a.value)).collect();
                self.held = device
                    .buttons
                    .iter()
                    .filter(|b| b.pressed)
                    .map(|b| b.code)
                    .collect();
                self.device = Some(device);
                self.imu = None;
                self.trouble = None;
                self.arrived = None;
                self.reports = 0;
                self.gaps.clear();
                self.worst_ms = 0;
                self.over_notable = 0;
                self.over_deadman = 0;
                self.quiet = 0;
                self.after_drops = 0;
                self.socket_dropped = 0;
                self.clock_steps = 0;
            }
            proto::PadReport::Detached { why } => {
                // The device goes, the counters stay: they are what someone reads *after* the pad
                // dropped, and clearing them here would erase the evidence at the moment it
                // became interesting.
                self.device = None;
                self.trouble = Some(why);
            }
            proto::PadReport::Frame(frame) => self.frame(frame),
            proto::PadReport::ImuAttached { device } => {
                self.imu = Some(pad_imu::Imu::new(*device));
            }
            proto::PadReport::Imu(batch) => {
                if let Some(imu) = self.imu.as_mut() {
                    imu.absorb(&batch);
                }
            }
            proto::PadReport::ImuDetached { .. } => self.imu = None,
        }
    }

    fn frame(&mut self, frame: proto::PadFrame) {
        self.reports += 1;
        self.arrived = Some(Instant::now());
        self.socket_dropped += frame.socket_dropped;
        if frame.after_drop {
            self.after_drops += 1;
        }

        for event in &frame.events {
            if event.is_axis() {
                self.axes.insert(event.code, event.value);
            } else if event.is_button() {
                // Non-zero, not `== 1`: a repeat arrives as 2, and treating it as a release would
                // show a held button letting go while it is being held down.
                if event.value != 0 {
                    self.held.insert(event.code);
                } else {
                    self.held.remove(&event.code);
                }
            }
        }

        let Some(since_us) = frame.since_us else {
            return; // the first report after attaching: nothing to measure from
        };
        if since_us < 0 {
            self.clock_steps += 1;
            return;
        }
        // The gap after a `SYN_DROPPED` spans events the kernel threw away, so it measures this
        // reader falling behind rather than the link. Counted as neither a stall nor a quiet spell.
        if frame.after_drop {
            return;
        }

        let ms = (since_us / 1_000) as u64;
        if ms > proto::pad_link::IDLE_MS {
            self.quiet += 1;
            return;
        }
        if self.gaps.len() == GAP_SAMPLES {
            self.gaps.pop_back();
        }
        self.gaps.push_front(ms);
        self.worst_ms = self.worst_ms.max(ms);
        if ms > proto::pad_link::NOTABLE_MS {
            self.over_notable += 1;
        }
        if ms > proto::pad_link::DEADMAN_MS {
            self.over_deadman += 1;
        }
    }

    /// Reports per second while the sticks are moving, over the last [`CADENCE_WINDOW`] gaps.
    ///
    /// Against elapsed time this would read as a catastrophically slow pad the moment anyone pauses,
    /// which is every real session — the same trap `scripts/pad-link-test.sh` rates against driving
    /// time to avoid.
    fn cadence(&self) -> Option<f64> {
        let window: Vec<u64> = self.gaps.iter().copied().take(CADENCE_WINDOW).collect();
        if window.is_empty() {
            return None;
        }
        let mean = window.iter().sum::<u64>() as f64 / window.len() as f64;
        (mean > 0.0).then(|| 1_000.0 / mean)
    }

    /// How long since a report, and what that silence means.
    fn silence(&self) -> Option<(Duration, Silence)> {
        let age = self.arrived?.elapsed();
        let ms = age.as_millis() as u64;
        let verdict = if ms > proto::pad_link::IDLE_MS {
            Silence::Idle
        } else if ms > proto::pad_link::DEADMAN_MS {
            Silence::PastTheDeadman
        } else if ms > proto::pad_link::NOTABLE_MS {
            Silence::Notable
        } else {
            Silence::Arriving
        };
        Some((age, verdict))
    }
}

/// The pad's inertial unit: a wireframe pad posed like the real one, and the numbers beside it.
///
/// Left, the picture; right, the words. The picture is what a person reads — a pad tilting on
/// screen as it tilts in the hand is understood in the time it takes to see it — and the numbers
/// are for when the picture is wrong: they say which axis, by how much, and whether the gyro bias
/// the picture depends on has been learned yet.
fn render_imu(imu: &pad_imu::Imu, frame: &mut ratatui::Frame, area: ratatui::layout::Rect) {
    let device = imu.device();
    let mut title = vec![
        Span::raw(" imu "),
        Span::styled(
            device.name.clone(),
            Style::new().fg(Color::Magenta).add_modifier(Modifier::BOLD),
        ),
    ];
    match imu.rate_hz() {
        Some(hz) => title.push(Span::raw(format!(" · {hz:.0} Hz ")).dim()),
        None => title.push(Span::raw(" · waiting for samples ").dim()),
    }
    let block = Block::bordered()
        .border_style(Style::new().fg(Color::Magenta))
        .title(Line::from(title))
        .title_bottom(
            Line::from(format!(
                " {} samples · {} · yellow bar = front edge ",
                imu.samples(),
                device.node
            ))
            .dim(),
        );
    let inner = block.inner(area);
    frame.render_widget(block, area);

    // The picture gets the left half of the width, capped where a wider one stops adding detail;
    // the words take the rest. The picture is the one that earns its place, so it is sized first.
    let picture_width = (inner.width / 2).clamp(12, 48);
    let [picture, words] =
        Layout::horizontal([Constraint::Length(picture_width), Constraint::Min(0)])
            .areas::<2>(inner);
    imu_view::draw(imu, picture, frame.buffer_mut());

    let [pitch, roll, yaw] = imu.euler_deg();
    let a = imu.accel_g();
    let g = imu.gyro_dps();
    let magnitude = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    let signed = |v: f32| format!("{v:+7.2}");
    let bias = match imu.bias_dps() {
        Some(b) => Line::from(vec![
            Span::raw(" bias  "),
            Span::raw(format!(
                "{} {} {} °/s ",
                signed(b[0]),
                signed(b[1]),
                signed(b[2])
            )),
            Span::styled("settled", Style::new().fg(Color::Green)),
            Span::raw(" — removed from the rates above").dim(),
        ]),
        None => Line::from(vec![
            Span::raw(" bias  "),
            Span::styled(
                "learning — hold the pad still for half a second",
                Style::new().fg(Color::Yellow),
            ),
        ]),
    };
    let mut lines = vec![
        Line::from(vec![
            Span::raw(" tilt  "),
            Span::styled(
                format!("pitch {pitch:+6.1}°  roll {roll:+6.1}°"),
                Style::new().add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!("  yaw {yaw:+6.1}°")),
            Span::raw(" (gyro only, drifts)").dim(),
        ]),
        Line::from(format!(
            " accel {} {} {} g   |a| {magnitude:.2} g",
            signed(a[0]),
            signed(a[1]),
            signed(a[2])
        )),
        Line::from(format!(
            " gyro  {} {} {} °/s",
            signed(g[0]),
            signed(g[1]),
            signed(g[2])
        )),
        bias,
        Line::from(" axes  +X front · +Y left · +Z up · at rest Z reads +1 g").dim(),
    ];
    if imu.socket_dropped() > 0 {
        lines.push(Line::from(vec![Span::styled(
            format!(
                " {} samples dropped reaching this view",
                imu.socket_dropped()
            ),
            Style::new().fg(Color::Magenta),
        )]));
    }
    frame.render_widget(Paragraph::new(lines), words);
}

/// What the time since the last report means.
///
/// The distinction between the last two is the whole difficulty of measuring a pad link, and
/// getting it wrong is not hypothetical: counting quiet as a stall is how the first measurement on
/// this robot reported three breaches of the deadman — the longest 75 seconds — on a link that
/// never faltered. A pad on a table sends nothing, and nothing is what a dead radio sends too.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Silence {
    /// Reports are arriving.
    Arriving,
    /// Late enough to feel sticky.
    Notable,
    /// Late enough that `robotd` has zeroed the velocity — if the sticks were moving.
    PastTheDeadman,
    /// Longer than any link stays up while silent, so it is the sticks at rest.
    Idle,
}

/// Everything on screen, plus the little history the trace needs.
struct View {
    /// Requested rate, for judging whether the stream has stalled.
    hz: u32,
    /// Which policy this `robotd` is running, from the subscribe acknowledgement. `None`
    /// until it arrives, and on a `robotd` too old to say.
    policy: Option<proto::SubscribeResult>,
    latest: Option<proto::RobotState>,
    /// When `latest` arrived, so its age can be shown. A frozen number with nothing saying
    /// it is frozen is the one failure mode a live view must not have.
    arrived: Option<Instant>,
    /// Achieved loop rate, **newest first** — the trace is drawn right to left, so the newest
    /// sample is the one at the right edge and history scrolls away to the left.
    trace: VecDeque<u64>,
    /// Best rate seen since the command started — the trace's full height. Never falls, so
    /// the baseline a dip is read against stays put.
    peak: u64,
    frames: u64,
    /// First joint row on screen. Only ever non-zero on a terminal too short for all of them,
    /// and it exists so that case is *navigable* rather than a table that quietly stops at the
    /// last row that happened to fit — half a leg, presented as the whole robot.
    scroll: usize,
    /// Joint rows the last frame had room for. Known only at render time, and kept because
    /// clamping the scroll and sizing a page both need it.
    visible: usize,
    /// What every angle on screen is drawn in. Degrees to start with: this view is read while
    /// looking at the robot, and a leg is a picture in degrees and arithmetic in radians.
    units: Units,
    /// The pad's raw event stream. Accumulated whether or not it is on screen, so opening the block
    /// shows a link's history rather than starting a measurement from the keypress.
    pad: PadView,
    /// Is the pad block open? Closed to begin with when there is a robot to look at — see
    /// [`PAD_HEIGHT`] — and open from the start when there is not, because then it is the only
    /// thing this view has to show and nobody should have to guess `p`.
    show_pad: bool,
    /// Why there is no robot stream, when there is none. `None` means one is connected, so an
    /// absent state is merely one that has not arrived yet.
    no_robot: Option<String>,
    /// The 3D robot view's camera and pixel cache.
    duck: duck::DuckView,
    /// Is the robot view wanted? Distinct from whether it is *drawn*: it also needs a
    /// state to pose from, a terminal wide enough, and a model that parsed.
    show_duck: bool,
    /// The odometry track, drawn as a top-down map under the robot view.
    path: path_map::PathMap,
    /// ToF beams through the head FK, so the depth grid can name the floor.
    reprojector: kinematics::tof::Reprojector,
    /// The last depth frame, and when it arrived by this view's clock — the frame's
    /// own `at_us` is `tofd`'s, and the question here is how stale what is on
    /// screen is.
    tof: Option<proto::TofFrame>,
    tof_arrived: Option<Instant>,
    /// What `tofd` said about its sensor, if it has answered.
    tof_status: Option<proto::TofStreamResult>,
    /// Why there is no depth stream, when there is none.
    tof_lost: Option<String>,
    /// Is the ToF block open? Closed to begin with — see [`TOF_HEIGHT`].
    show_tof: bool,
    /// The last frame off `mediad`, and the picture drawn from it.
    camera: camera::CameraView,
    /// Is the camera block open? Closed to begin with, and unlike every other block that also
    /// means nothing is being *fetched* — see [`read_camera`].
    show_camera: bool,
    /// The last answer to `robot.health`: the battery, the temperatures, the bus counters and
    /// how stale the IMU's reads are. None of it is on the state stream.
    health: Option<proto::HealthResult>,
    /// When that answer arrived. A charge reading is the one number on this screen somebody
    /// plans around, and one that quietly stopped being refreshed is worse than none.
    health_at: Option<Instant>,
    /// Why the health poll is not answering, when it is not.
    health_lost: Option<String>,
}

impl View {
    fn new(hz: u32, no_robot: Option<String>) -> Self {
        Self {
            hz,
            policy: None,
            latest: None,
            arrived: None,
            trace: VecDeque::with_capacity(TRACE_SAMPLES),
            peak: 0,
            frames: 0,
            scroll: 0,
            visible: 0,
            units: Units::Degrees,
            pad: PadView::default(),
            show_pad: no_robot.is_some(),
            no_robot,
            duck: duck::DuckView::new(),
            show_duck: true,
            path: path_map::PathMap::new(),
            reprojector: kinematics::tof::Reprojector::alpha(),
            tof: None,
            tof_arrived: None,
            tof_status: None,
            tof_lost: None,
            show_tof: false,
            camera: camera::CameraView::default(),
            show_camera: false,
            health: None,
            health_at: None,
            health_lost: None,
        }
    }

    fn toggle_units(&mut self) {
        self.units = self.units.toggled();
    }

    fn toggle_tof(&mut self) {
        self.show_tof = !self.show_tof;
    }

    /// Open or close the camera block. Closing forgets the picture: nothing is fetched while it
    /// is shut, so what is held would be minutes old by the time it was shown again.
    fn toggle_camera(&mut self) {
        self.show_camera = !self.show_camera;
        if !self.show_camera {
            self.camera.forget();
        }
    }

    fn toggle_pad(&mut self) {
        self.show_pad = !self.show_pad;
    }

    fn toggle_duck(&mut self) {
        self.show_duck = !self.show_duck;
    }

    fn orbit_duck(&mut self, radians: f32) {
        self.duck.orbit(radians);
    }

    /// Move the joint window. Clamped on render, where the number of rows that fit is known.
    fn scroll_by(&mut self, rows: isize) {
        self.scroll = self.scroll.saturating_add_signed(rows);
    }

    /// A page is whatever the last frame had room for, so the keys mean the same thing on a
    /// terminal of any height.
    fn scroll_pages(&mut self, pages: isize) {
        self.scroll_by(pages * self.visible.max(1) as isize);
    }

    fn scroll_home(&mut self) {
        self.scroll = 0;
    }

    /// Take one update. `true` when the screen has something new to say.
    fn absorb(&mut self, update: Update) -> Result<bool, Failure> {
        match update {
            Update::Ended(why) => Err(Failure::new(exit::UNREACHABLE, why)),
            Update::Policy(policy) => {
                self.policy = Some(*policy);
                Ok(true)
            }
            Update::Pad(report) => {
                let sample = matches!(*report, proto::PadReport::Imu(_));
                self.pad.absorb(*report);
                // A repaint even while the block is closed would be a redraw per report, at up to
                // 125 a second, for something nobody is looking at — and an IMU sample, at six
                // hundred a second, earns one only every [`IMU_REPAINT`] even when it is.
                if !self.show_pad {
                    return Ok(false);
                }
                Ok(!sample || self.pad.imu_repaint_due())
            }
            Update::Tof(frame) => {
                self.tof = Some(*frame);
                self.tof_arrived = Some(Instant::now());
                self.tof_lost = None;
                // Same reasoning as the pad: no repaint for a block nobody has
                // open, or this would redraw fifteen times a second unseen.
                Ok(self.show_tof)
            }
            Update::TofStatus(status) => {
                self.tof_status = Some(*status);
                self.tof_lost = None;
                Ok(self.show_tof)
            }
            Update::TofLost(why) => {
                self.tof_lost = Some(why);
                self.tof = None;
                self.tof_status = None;
                Ok(self.show_tof)
            }
            Update::PadLost(why) => {
                self.pad.device = None;
                // The IMU goes with it: the `ImuDetached` for this pad died with the tap, and
                // the next subscription seeds only what the new `padd` has — a pad without an
                // IMU would leave the last one's panel frozen on screen forever.
                self.pad.imu = None;
                self.pad.trouble = Some(why);
                Ok(self.show_pad)
            }
            // A frame only arrives because the block is open, so it is always worth a repaint.
            Update::Camera(shot) => {
                self.camera.absorb(*shot);
                Ok(true)
            }
            Update::CameraLost(why) => {
                self.camera.lost(why);
                Ok(self.show_camera)
            }
            Update::Health(health) => {
                self.health = Some(*health);
                self.health_at = Some(Instant::now());
                self.health_lost = None;
                Ok(true)
            }
            Update::HealthLost(why) => {
                // The last reading is **kept**, not cleared: it was true when it arrived, and
                // [`Self::health_at`] already says how long ago that was. Blanking the pack's
                // charge because `robotd` restarted during an update throws away the last thing
                // known about it and puts nothing in its place.
                self.health_lost = Some(why);
                Ok(true)
            }
            Update::State(state) => {
                if self.trace.len() == TRACE_SAMPLES {
                    self.trace.pop_back();
                }
                let rate = state.control_loop.hz.round().max(0.0) as u64;
                self.trace.push_front(rate);
                self.peak = self.peak.max(rate);
                self.frames += 1;
                self.arrived = Some(Instant::now());
                self.path.observe(
                    state.odom.position[0],
                    state.odom.position[1],
                    state.odom.yaw,
                );
                self.latest = Some(*state);
                Ok(true)
            }
        }
    }

    /// Has the stream gone quiet? Five periods, floored at half a second, so a slow
    /// `--hz 1` is not accused of stalling between two perfectly ordinary frames.
    fn stalled_for(&self) -> Option<Duration> {
        let age = self.arrived?.elapsed();
        let quiet = Duration::from_secs_f64(5.0 / f64::from(self.hz.clamp(1, 200)))
            .max(Duration::from_millis(500));
        (age > quiet).then_some(age)
    }

    fn render(&mut self, frame: &mut ratatui::Frame) {
        let area = frame.area();
        let Some(rows) = self.latest.as_ref().map(joint_rows) else {
            // No robot state — either `robotd` has not sent one yet, or there is no `robotd` to
            // connect to. Either way the pad block is still drawn when it is open: it reads a
            // different daemon over a different socket, and a robot is not a precondition for
            // watching the sticks.
            let pad_height = self.pad_height();
            let tof_height = if self.show_tof { TOF_HEIGHT } else { 0 };
            let camera_height = if self.show_camera { CAMERA_HEIGHT } else { 0 };
            let [pad, tof, camera, rest] = Layout::vertical([
                Constraint::Length(pad_height),
                Constraint::Length(tof_height),
                Constraint::Length(camera_height),
                Constraint::Min(3),
            ])
            .areas(area);
            if self.show_pad {
                self.render_pad(frame, pad);
            }
            if self.show_tof {
                self.render_tof(frame, tof);
            }
            // Drawn here too, and this is the case it is most needed in: a camera answers on a
            // board whose servo power is off, so a picture is one of the few things this frame
            // can still show when no state will ever arrive.
            if self.show_camera {
                self.render_camera(frame, camera);
            }
            let waiting = match &self.no_robot {
                // Named rather than folded into "waiting", because waiting for a robot that is
                // there and waiting for one that is not need different things done about them.
                Some(why) => {
                    format!(
                        "no robotd: {why}\nthe pad, tof and camera blocks still work — p, t and c toggle them"
                    )
                }
                None => "waiting for robot.state…".to_owned(),
            };
            // The power row is drawn here as well, and this is the case that earns it: a board
            // whose servo power is off never completes a control tick, so no state ever arrives
            // and this panel is the whole screen — while `robot.health` answers, and its answer
            // is what says why. Outside the dimmed sentences, because a flat pack is not a
            // footnote.
            let mut lines: Vec<Line<'static>> = waiting
                .lines()
                .map(|l| Line::from(l.to_owned()).dim())
                .collect();
            lines.push(Line::from(""));
            lines.push(self.power(rest.width.saturating_sub(2)));
            frame.render_widget(
                Paragraph::new(lines).block(Block::bordered().title(" monitor ")),
                rest,
            );
            return;
        };

        // Split by hand rather than by constraint solving, because the order the space is
        // wanted in is a decision: the joints table gets the height it needs, the trace gets
        // what is left. Floored at three rows so the trace survives an 80×24 terminal — a joint
        // scrolled off the bottom is still reachable, whereas a missing loop rate is the one
        // number that says whether the others can be trusted at all. Capped at six because a
        // mostly-flat rate drawn twenty rows tall is a wall of ink, not more information.
        //
        // The 3D view takes the right edge first, when it is wanted and there is room —
        // width the tables were never using — and everything below lays out in what is
        // left, exactly as it did before the view existed.
        let (area, duck) = self.duck_split(area);

        // The pad block, when open, sits directly under the header rather than at the bottom: it is
        // the *source* of the command the header reports, and the frame then reads top to bottom in
        // the order the robot does — sticks, command, joints, loop rate.
        let pad_height = self.pad_height();
        let tof_height = if self.show_tof { TOF_HEIGHT } else { 0 };
        let camera_height = if self.show_camera { CAMERA_HEIGHT } else { 0 };
        let trace_height = area
            .height
            .saturating_sub(
                HEADER_HEIGHT + pad_height + tof_height + camera_height + rows as u16 + 3,
            )
            .clamp(3, 6);
        let [header, pad, tof, camera, joints, trace] = Layout::vertical([
            Constraint::Length(HEADER_HEIGHT),
            Constraint::Length(pad_height),
            Constraint::Length(tof_height),
            Constraint::Length(camera_height),
            Constraint::Min(4),
            Constraint::Length(trace_height),
        ])
        .areas(area);

        // Two borders and the column header come off the table's height before any joint fits.
        self.visible = usize::from(joints.height.saturating_sub(3));
        self.scroll = self.scroll.min(rows.saturating_sub(self.visible));

        let (scroll, visible) = (self.scroll, self.visible);
        let state = self.latest.as_ref().expect("a state, checked above");

        self.render_header(frame, header, state);
        if self.show_pad {
            self.render_pad(frame, pad);
        }
        // Under the pad and above the joints: the frame then reads in the order
        // the robot does — what it was told, what it sees, what it did.
        if self.show_tof {
            self.render_tof(frame, tof);
        }
        // Under the sensors and above the joints, still in the robot's own order: the camera is
        // another thing it sees, and the joints below are what it did about all of them.
        if self.show_camera {
            self.render_camera(frame, camera);
        }
        frame.render_stateful_widget(
            self.joints(state, rows, visible),
            joints,
            &mut TableState::new().with_offset(scroll),
        );
        frame.render_widget(self.trace(), trace);
        if let Some(duck) = duck {
            self.render_duck(frame, duck);
        }
    }

    /// Rows the pad block takes: none while closed, [`PAD_HEIGHT`] open, and [`IMU_HEIGHT`] more
    /// while the pad has an inertial unit to show.
    fn pad_height(&self) -> u16 {
        if !self.show_pad {
            return 0;
        }
        PAD_HEIGHT
            + if self.pad.imu.is_some() {
                IMU_HEIGHT
            } else {
                0
            }
    }

    /// Carve the robot view's column off the right, when it is wanted and fits.
    fn duck_split(
        &self,
        area: ratatui::layout::Rect,
    ) -> (ratatui::layout::Rect, Option<ratatui::layout::Rect>) {
        if !self.show_duck || duck::model().is_none() {
            return (area, None);
        }
        let spare = area.width.saturating_sub(DUCK_MAIN_MIN);
        if spare < DUCK_MIN_WIDTH {
            return (area, None);
        }
        let [main, duck] = Layout::horizontal([
            Constraint::Min(DUCK_MAIN_MIN),
            Constraint::Length(spare.min(DUCK_MAX_WIDTH)),
        ])
        .areas(area);
        (main, Some(duck))
    }

    /// The robot, drawn: the baked sim model posed by the measured joints and tilted by
    /// the IMU. See [`duck`] for what this is for and how it stays cheap.
    fn render_duck(&mut self, frame: &mut ratatui::Frame, area: ratatui::layout::Rect) {
        let (Some(model), Some(state)) = (duck::model(), self.latest.as_ref()) else {
            return;
        };
        // The path map takes a fixed slice under the 3D view when the column is
        // tall enough for both — the map's panel never grows; its *world* zooms.
        let (area, map) = if area.height >= DUCK_MIN_HEIGHT + PATH_HEIGHT {
            let [duck, map] = Layout::vertical([
                Constraint::Min(DUCK_MIN_HEIGHT),
                Constraint::Length(PATH_HEIGHT),
            ])
            .areas::<2>(area);
            (duck, Some(map))
        } else {
            (area, None)
        };

        let block = Block::bordered()
            .title(" robot ")
            .title_bottom(Line::from(" d hides · ← → orbits ").dim().right_aligned());
        let inner = block.inner(area);
        frame.render_widget(block, area);
        // The depth frame rides along as contact points — yellow for obstacles,
        // green for confirmed floor — so what the sensor sees is drawn where it
        // sees it, and the view zooms out just enough to keep them in frame.
        let markers = self.tof_markers();
        self.duck.draw(
            model,
            &state.joints,
            state.safety.gravity,
            &markers,
            inner,
            frame.buffer_mut(),
        );

        if let Some(map) = map {
            self.render_path(frame, map);
        }
    }

    /// The odometry track, top-down: boot-forward is up, the origin is `+`, the
    /// robot is `●` with a heading ray. See [`path_map`].
    fn render_path(&self, frame: &mut ratatui::Frame, area: ratatui::layout::Rect) {
        let block = Block::bordered().title(" path ").title_bottom(
            // The zoom level, or the caption is a shape with no size.
            Line::from(format!(" {:.1} m across ", self.path.extent_m()))
                .dim()
                .right_aligned(),
        );
        let inner = block.inner(area);
        frame.render_widget(block, area);
        self.path.draw(inner, frame.buffer_mut());
    }

    /// The whole-robot block: what was asked of it, what it did, and what it can feel.
    ///
    /// Everything is spelled out — axis names, units, the sense of each direction — because
    /// the previous version wrote `req [+0.30 +0.00 +0.10]` and a reader had to already know
    /// that a velocity twist is `vx, vy, vyaw`, that the numbers are m/s and rad/s, and which
    /// way positive turns. That convention is written down in `duck-ipc-proto`, and a display
    /// that omits it makes every reader re-derive it — which is the exact failure the protocol
    /// docs name as the reason the prototype grew five sign-flip flags.
    fn render_header(
        &self,
        frame: &mut ratatui::Frame,
        area: ratatui::layout::Rect,
        state: &proto::RobotState,
    ) {
        // The policy's identity goes on the bottom border: it never changes while the command
        // runs, so it belongs where a caption belongs rather than taking a row from the joints.
        let block = Block::bordered()
            .title(Line::from(self.title(state)))
            .title_top(
                // The units key is named next to the others because a reader who does not know
                // it exists has no way to discover that the numbers could be radians instead.
                // The robot view is only named here while it is hidden: visible, it
                // carries its own caption, and this title clips on a narrow terminal.
                Line::from(format!(
                    " q quits · ↑↓ scrolls · u {} · p {}{}{}{} ",
                    self.units.toggled().name(),
                    if self.show_pad {
                        "hides the pad"
                    } else {
                        "the raw pad"
                    },
                    // Named only while hidden, like the robot view below and for
                    // the same reason: open, the block carries its own title, and
                    // every character here is one the left-hand title loses.
                    if self.show_tof { "" } else { " · t the tof" },
                    if self.show_camera {
                        ""
                    } else {
                        " · c the camera"
                    },
                    if self.show_duck {
                        ""
                    } else {
                        " · d the robot"
                    }
                ))
                .dim()
                .right_aligned(),
            )
            .title_bottom(Line::from(self.policy_caption()))
            .title_bottom(Line::from(Self::camera_caption()).right_aligned());
        let inner = block.inner(area);
        frame.render_widget(block, area);

        // Command on the left, sensing on the right: two four-row columns rather than eight
        // stacked rows, because every row here is a row the joints table does not get.
        // The power row takes the header's last line, under the three that describe the
        // command: it is the robot's *condition* rather than its behaviour, it comes from a
        // different call, and it is the row somebody glances at without reading the rest.
        let [top, bottom, power] = Layout::vertical([
            Constraint::Length(4),
            Constraint::Length(3),
            Constraint::Length(1),
        ])
        .areas::<3>(inner);
        let [asked, felt] =
            Layout::horizontal([Constraint::Percentage(52), Constraint::Percentage(48)])
                .areas::<2>(top);

        frame.render_widget(self.movement(state), asked);
        frame.render_widget(self.imu(state), felt);
        frame.render_widget(self.limits_and_head(state), bottom);
        frame.render_widget(Paragraph::new(self.power(power.width)), power);
    }

    /// Which network is driving this robot, from the subscribe acknowledgement.
    ///
    /// [`proto::RobotState::policy`] says `walk`, `stand` or `held` — the mode of *this tick*.
    /// That is not the same question as which policy is loaded, and it was the only one this
    /// view could answer: two releases with different gaits both say `walk`, and "which
    /// network is this?" is the first thing anyone comparing them asks.
    fn policy_caption(&self) -> Vec<Span<'static>> {
        let Some(policy) = self.policy.as_ref() else {
            // No acknowledgement yet, or a `robotd` that predates it. Said out loud, because
            // the alternative is a caption that looks like a robot with no policy.
            return vec![Span::raw(" policy · not reported by robotd ").dim()];
        };

        let mut caption = vec![Span::raw(" policy · ").dim()];
        match policy.walk.as_deref() {
            Some(walk) => {
                caption.push(Span::styled(
                    walk.to_owned(),
                    Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
                ));
                if let Some(stand) = policy.stand.as_deref() {
                    caption.push(Span::raw(" · standing ").dim());
                    caption.push(Span::styled(stand.to_owned(), Style::new().fg(Color::Cyan)));
                } else {
                    // Not an omission: without a standing network the walking one runs at
                    // every velocity, which changes how the robot behaves at rest.
                    caption.push(Span::raw(" · no standing policy").dim());
                }
            }
            None => caption.push(Span::raw("none loaded").dim()),
        }
        // The skill networks, compactly: which one-shots this robot can actually do. A
        // robot with no kick refuses the button with a reason, but the reason arrives in
        // padd's journal — this is where someone at the screen finds out first.
        // The two fixed ones keep their short labels; the rest are whatever this robot has
        // configured, named by the robot rather than by a list compiled in here — which is the
        // point of a skill being config.
        let mut skills: Vec<&str> = Vec::new();
        if policy.sitstand.is_some() {
            skills.push("sit");
        }
        if policy.ground_pick.is_some() {
            skills.push("pick");
        }
        skills.extend(policy.skills.iter().map(String::as_str));
        if policy.walk.is_some() {
            if skills.is_empty() {
                caption.push(Span::raw(" · no skills").dim());
            } else {
                caption.push(Span::raw(" · skills ").dim());
                caption.push(Span::styled(skills.join("+"), Style::new().fg(Color::Cyan)));
            }
        }
        if let Some(why) = policy.unavailable.as_deref() {
            caption.push(Span::styled(
                format!(" — {why}"),
                Style::new().fg(Color::Yellow),
            ));
        }
        caption.push(Span::raw(" ").dim());
        caption
    }

    /// The block's own title: who is driving, how fast the loop is going, and whether the
    /// frame on screen is still arriving.
    fn title(&self, state: &proto::RobotState) -> Vec<Span<'static>> {
        let missed = state.control_loop.missed;
        let mut title = vec![
            Span::raw(" policy "),
            Span::styled(
                state.policy.clone(),
                Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!(" · t {:.2} s · ", state.t)),
            Span::styled(
                format!("{:.1} Hz", state.control_loop.hz),
                Style::new().fg(Color::Cyan),
            ),
            Span::raw(" · missed "),
            Span::styled(
                missed.to_string(),
                if missed > 0 {
                    Style::new().fg(Color::Yellow)
                } else {
                    Style::new().dim()
                },
            ),
            Span::raw(" "),
        ];
        if let Some(age) = self.stalled_for() {
            title.push(Span::styled(
                format!("· STALLED {:.1}s ", age.as_secs_f64()),
                Style::new().fg(Color::Red).add_modifier(Modifier::BOLD),
            ));
        }
        title
    }

    /// The velocity twist, one labelled row per axis: what a client asked for, and what
    /// actually reached the policy after safety had its say.
    fn movement(&self, state: &proto::RobotState) -> Paragraph<'static> {
        // `angular` says whether this axis is a turn rate: the two linear ones are m/s in any
        // unit setting, and converting them would be nonsense dressed as consistency.
        let axis = |name: &str, sense: &str, unit: &str, i: usize, angular: bool| {
            let (asked, applied) = (state.movement.requested[i], state.movement.applied[i]);
            // Highlight the difference, not the pair: "asked for 0.3, got 0.15" is the whole
            // reason this command exists, and it is invisible when both numbers look alike.
            // Judged on the wire values, so the same clamp reads the same in either unit.
            let style = if (asked - applied).abs() > 1e-6 {
                Style::new().fg(Color::Yellow)
            } else {
                Style::new()
            };
            let show = |v: f64| if angular { self.units.rate(v) } else { v };
            let (asked, applied) = (show(asked), show(applied));
            Line::from(vec![
                Span::raw(format!(" {name:<5}{sense:<10}{asked:>+7.2}")),
                Span::styled(format!("{applied:>+8.2}"), style),
                Span::raw(format!("  {unit}")).dim(),
            ])
        };

        Paragraph::new(vec![
            Line::from(" move                asked applied").dim(),
            axis("vx", "forward", "m/s", 0, false),
            axis("vy", "left", "m/s", 1, false),
            axis("vyaw", "turn left", self.units.rate_unit(), 2, true),
        ])
    }

    /// What the IMU is telling the robot, and the verdict drawn from it.
    ///
    /// Projected gravity is the only IMU quantity on this stream — `robot.health` has the
    /// stale-read counters — and it is here rather than reduced to `fallen` because the
    /// verdict alone cannot tell a robot lying on its side from an IMU mounted the way this
    /// build does not expect. Upright is about `[0, 0, -1]`.
    fn imu(&self, state: &proto::RobotState) -> Paragraph<'static> {
        let axis = |name: &str, i: usize| {
            Line::from(format!(" {name:<11}{:>+6.2}", state.safety.gravity[i]))
        };
        let mut down = axis("z up", 2).spans;
        down.push(Span::raw("   "));
        down.push(fall_verdict(&state.safety));

        Paragraph::new(vec![
            Line::from(" imu · gravity in the trunk frame").dim(),
            axis("x forward", 0),
            axis("y left", 1),
            Line::from(down),
        ])
    }

    /// Three rows that are always present whether or not they have anything to report, because
    /// a header that changes height moves the joints table under the reader's eyes.
    fn limits_and_head(&self, state: &proto::RobotState) -> Paragraph<'static> {
        let limits = if state.movement.limited_by.is_empty() {
            Line::from(" limits  none — the command went through untouched").dim()
        } else {
            // Explained, not just named: `deadman` is a token the reader has to look up, and
            // the sentence is the thing they were looking it up for.
            Line::from(vec![
                Span::raw(" limits  "),
                Span::styled(
                    state
                        .movement
                        .limited_by
                        .iter()
                        .map(|l| explain_limit(l))
                        .collect::<Vec<_>>()
                        .join("; "),
                    Style::new().fg(Color::Yellow),
                ),
            ])
        };

        let angle = |i: usize| self.units.angle(state.head[i]);
        let mut head = vec![Span::raw(format!(
            " head    neck_pitch {}  head_pitch {}  head_yaw {}  head_roll {}",
            angle(0),
            angle(1),
            angle(2),
            angle(3)
        ))];
        // Degrees carry their own `°`; radians are bare, and so have to be named here or the
        // row is four numbers in no unit at all.
        head.push(
            Span::raw(match self.units {
                Units::Degrees => "   ",
                Units::Radians => " rad   ",
            })
            .dim(),
        );
        // The gain the servos are actually running at, next to `limp`, which is what a gain
        // that safety has overridden looks like from the outside.
        head.push(Span::raw(format!(
            "kp {}",
            state
                .safety
                .gain
                .map_or_else(|| "-".to_owned(), |g| g.to_string())
        )));
        if state.safety.limp {
            head.push(Span::styled(
                " limp — gains dropped so the robot yields",
                Style::new().fg(Color::Yellow),
            ));
        }

        // Contact odometry: where the legs and IMU say the robot has walked to since boot.
        // Yaw follows the angle-unit toggle like everything else; metres are metres.
        let odom = Line::from(format!(
            " odom    x {:>+6.2} m  y {:>+6.2} m  yaw {}",
            state.odom.position[0],
            state.odom.position[1],
            self.units.angle(state.odom.yaw),
        ));

        Paragraph::new(vec![limits, Line::from(head), odom])
    }

    /// What the pack has left, what is hot, and what `robot.health` says is wrong — the numbers
    /// `robot.state` does not carry, on the one row that is always on screen.
    ///
    /// **Two classes of content, not one row of equals.** The charge and anything wrong are
    /// *said* — drawn whatever the width, and clipped by the terminal if it comes to that,
    /// because a clipped sentence is one somebody can go and finish in `robotctl health` while a
    /// dropped one is an alarm nobody knows fired. The temperatures are trimmings: drawn only
    /// when they fit whole, and abandoned at the first one that does not, because half a number
    /// is not a smaller number — ` · cpu 5` is a lie about a board at 52 °C.
    fn power(&self, width: u16) -> Line<'static> {
        let Some(health) = self.health.as_ref() else {
            // Two absences, and they are different: nothing has answered yet, versus something
            // answered and then stopped.
            let sentence = match &self.health_lost {
                Some(why) => format!(" power   no health from robotd — {}", brief(why)),
                None => " power   asking robotd for the battery…".to_owned(),
            };
            return Line::from(Span::raw(sentence).dim());
        };

        // What gets said whatever the width, in the order it matters.
        let mut said: Vec<Span<'static>> = Vec::new();

        // The charge first, and unconditionally. 0% is `BATTERY_EMPTY_V`, which is where
        // `robotd` sits the robot down and cuts power — so this is a countdown, not a gauge,
        // and the last fifth of it is worth seeing from across a room.
        said.push(match &health.battery {
            Some(b) => Span::styled(
                format!("batt {:.2} V {:.0}%", b.volts, b.percent),
                if b.percent <= BATTERY_CRITICAL_PCT {
                    Style::new().fg(Color::Red).add_modifier(Modifier::BOLD)
                } else if b.percent <= BATTERY_LOW_PCT {
                    Style::new().fg(Color::Yellow)
                } else {
                    Style::new().fg(Color::Green)
                },
            ),
            // Absent means *not measured* — the first second of uptime, or a bus that cannot
            // answer. Drawn as `0.00 V, 0%` it would put a flat-pack warning in front of
            // somebody whose robot has a full one, which is the mistake `robotd`'s own
            // sentinel exists to avoid.
            None => Span::raw("batt not read yet").dim(),
        });

        // Said early, because it qualifies everything after it: these are the numbers from the
        // last answer, and this is how long ago that answer was.
        if let Some(age) = self.health_age() {
            said.push(Span::styled(
                format!(" · {} s old", age.as_secs()),
                Style::new().fg(Color::Yellow),
            ));
        }
        if let Some(why) = &self.health_lost {
            said.push(Span::styled(
                format!(" · health poll: {}", brief(why)),
                Style::new().fg(Color::Yellow),
            ));
        }

        // What is wrong, before the temperatures: this is the line that explains a robot
        // standing still, which is the question somebody is holding when they look here.
        if !health.healthy {
            let reason = health.reason.as_deref().unwrap_or("no reason given");
            said.push(if health.degraded {
                // A fact about the board rather than about the release — see
                // `HealthResult::degraded`. Yellow, not red: nothing is broken, the robot
                // simply cannot move, and no update is going to change that.
                Span::styled(
                    format!(" · degraded: {reason}"),
                    Style::new().fg(Color::Yellow),
                )
            } else {
                Span::styled(
                    format!(" · unhealthy: {reason}"),
                    Style::new().fg(Color::Red).add_modifier(Modifier::BOLD),
                )
            });
        }

        // Frozen orientation does not make `robotd` unhealthy and cannot be seen anywhere else
        // on this screen: the board keeps answering, so the bus reports no error and `ready`
        // stays true, while the gravity vector in the header holds a plausible attitude for as
        // long as it takes somebody to notice it has not moved.
        if let Some(imu) = &health.imu
            && imu.frozen()
        {
            said.push(Span::styled(
                format!(
                    " · orientation frozen — {} stale reads",
                    imu.consecutive_stale_blocks
                ),
                Style::new().fg(Color::Red).add_modifier(Modifier::BOLD),
            ));
        }

        // Bus trouble under a running loop only. The startup case — nothing answered on the bus
        // at all — is already the degraded reason above, and saying it twice on one row costs
        // the temperatures for no information.
        if health.bus.consecutive_errors > 0 {
            said.push(Span::styled(
                format!(
                    " · bus {} read failures running",
                    health.bus.consecutive_errors
                ),
                Style::new().fg(Color::Yellow),
            ));
        }

        // A capped clock is said here rather than added to the temperatures below, and it takes
        // the board temperature with it. Two reasons, and they are the same one twice.
        //
        // It cannot be trimmed. The temperatures are nice to have and get dropped on a narrow
        // row; this explains a duck walking badly, and it is the wrong thing to lose to a
        // terminal's width — a board at 408 MHz of 1800 has under a quarter of the CPU the
        // gaits were tuned on.
        //
        // And it is coloured, which the temperatures deliberately are not. No threshold is being
        // invented here: the kernel's own thermal governor has already decided and acted, and
        // this is a report of that action rather than an opinion about a number.
        let capped = health.cpu_throttle.filter(proto::CpuThrottle::throttled);
        if let Some(t) = capped {
            // The temperature rides along when there is one, so the pair still reads as one
            // fact — and is simply left out when the kernel offers cpufreq but no thermal
            // zone, rather than filled in with a zero that would read as a cold board.
            let temp = match health.cpu_temp_c {
                Some(c) => format!("{c:.0} °C "),
                None => String::new(),
            };
            said.push(Span::styled(
                format!(" · cpu {temp}throttled to {} MHz", t.khz / 1000),
                Style::new().fg(Color::Yellow),
            ));
        }

        // Uncoloured, deliberately: nothing in this workspace defines a servo temperature that
        // is too high, and a threshold invented here would be one `robotctl health` does not
        // agree with. The number and the joint are what somebody acts on.
        let mut trimmings: Vec<Span<'static>> = Vec::new();
        if let Some(m) = &health.motors {
            trimmings.push(Span::raw(format!(
                " · motors {:.0} °C max ({})",
                m.max_c, m.hottest
            )));
        }
        // Beside the servos rather than merged with them: a robot that has been walking has hot
        // motors, a board in an enclosure with a blocked vent has a hot SoC, and the two are
        // fixed differently — which is the reason `HealthResult` carries them separately.
        if let Some(cpu) = health.cpu_temp_c
            && capped.is_none()
        {
            trimmings.push(Span::raw(format!(" · cpu {cpu:.0} °C")));
        }

        // The label is nine columns, like `limits`, `head` and `odom` above it.
        let mut row = vec![Span::raw(" power   ")];
        let mut used = 9;
        for span in said {
            used += span.content.chars().count();
            row.push(span);
        }
        // Whole ones only, and stopping at the first that does not fit rather than skipping it:
        // a row that dropped the motors and kept the cooler CPU beside them would read as a
        // robot with no servo temperatures at all.
        for span in trimmings {
            let len = span.content.chars().count();
            if used + len > usize::from(width) {
                break;
            }
            used += len;
            row.push(span);
        }
        Line::from(row)
    }

    /// How stale the health answer is, when it is stale enough to be worth saying.
    ///
    /// `robotd` closing the socket is reported by the poller itself. This catches the other
    /// shape: a daemon that accepts, answers, and then goes quiet — where the charge would
    /// otherwise sit on screen looking current for as long as the session lasts.
    fn health_age(&self) -> Option<Duration> {
        let age = self.health_at?.elapsed();
        (age > HEALTH_STALE).then_some(age)
    }

    /// Measured against commanded, per joint, with the difference as a bar.
    ///
    /// The bar is the point. A servo that is not keeping up, a leg holding a load, a policy
    /// asking for something the joint cannot do — all of them are a column of numbers that
    /// look plausible, and a bar that is obviously off centre.
    fn joints(&self, state: &proto::RobotState, count: usize, visible: usize) -> Table<'_> {
        let rows = (0..count).map(|i| {
            let measured = state.joints.get(i).copied();
            let target = state.targets.get(i).copied();
            let error = measured.zip(target).map(|(m, t)| m - t);
            Row::new(vec![
                // A joint the wire has but this build cannot name: a `robotd` running a
                // model this `robotctl` predates. Show the index rather than dropping the
                // row — an unnamed joint is still a joint someone is debugging.
                Cell::from(
                    proto::JOINT_NAMES
                        .get(i)
                        .map_or_else(|| format!("joint {i}"), |name| (*name).to_owned()),
                ),
                Cell::from(Line::from(self.angle(measured)).right_aligned()),
                Cell::from(Line::from(self.angle(target)).right_aligned()),
                Cell::from(
                    Line::from(match error {
                        Some(e) => Span::styled(self.units.angle(e), error_style(e)),
                        None => Span::raw("-").dim(),
                    })
                    .right_aligned(),
                ),
                Cell::from(deviation_bar(error)),
            ])
        });

        Table::new(
            rows,
            [
                // Wide enough for a degree: `-123.45°` is two characters longer than the
                // `-2.155` a radian needs, and a column that fits one unit but truncates the
                // other turns the toggle into a way to lose digits.
                Constraint::Length(15),
                Constraint::Length(9),
                Constraint::Length(9),
                Constraint::Length(9),
                Constraint::Min(BAR_HALF as u16 * 2 + 1),
            ],
        )
        .header(
            // "commanded", not "target": the wire calls it `targets`, but next to a *measured*
            // angle the word target reads as a goal the robot is working towards rather than
            // the number that was written to the servo on this very tick.
            Row::new(vec!["joint", "measured", "commanded", "error", "deviation"]).style(
                Style::new()
                    .add_modifier(Modifier::BOLD)
                    .add_modifier(Modifier::DIM),
            ),
        )
        .column_spacing(1)
        .block(
            Block::bordered().title(" joints ").title_top(
                Line::from(self.window_note(count, visible))
                    .dim()
                    .right_aligned(),
            ),
        )
    }

    /// One joint angle, or a dash where the wire carried none.
    fn angle(&self, v: Option<f64>) -> Span<'static> {
        match v {
            Some(v) => Span::raw(self.units.angle(v)),
            None => Span::raw("-").dim(),
        }
    }

    /// What the joints block says about itself on the right of its border: the bar's scale
    /// when every joint is on screen, and *which* joints are on screen when they are not.
    /// Never silently truncated — a table that stops mid-leg with nothing saying so is a
    /// display that lies.
    fn window_note(&self, count: usize, visible: usize) -> String {
        let (unit, scale) = (self.units.name(), self.units.bar_scale());
        if visible >= count {
            return format!(" {unit} · bar reaches {scale} ");
        }
        let last = (self.scroll + visible).min(count);
        format!(
            " {unit} · bar {scale} · {}–{last} of {count} · ↑↓ scrolls ",
            self.scroll + 1
        )
    }

    /// The camera, on the bottom border rather than in a block of its own.
    ///
    /// A block would cost rows the joints table needs, and the camera is two numbers — a rate and
    /// a drop count — that only matter when one of them is wrong. On the border they are visible
    /// at all times and cost nothing.
    ///
    /// Read from `mediad`'s published file on every redraw. That is a ~200-byte read from a tmpfs
    /// at the redraw rate, which is cheaper than holding a socket open to a daemon that is
    /// disabled on most robots.
    ///
    /// **`--json` output is deliberately untouched.** One line per tick is a scripted interface,
    /// and adding a field to it would break `monitor | grep` for everyone. `robotctl health
    /// --json` carries the same numbers for anything that wants to read them.
    fn camera_caption() -> Vec<Span<'static>> {
        let Some(camera) = duck_ipc_proto::read_camera_stats() else {
            // Silent rather than "camera unknown": a board with no camera runs no `mediad`, and a
            // caption implying a fault on every one of them is worse than no caption. Whether the
            // daemon is running belongs to `robotctl health`, which says so in words.
            return vec![];
        };

        let below_target = camera.fps < f64::from(camera.target_fps) * 0.9;
        let rate = Span::styled(
            format!("{:.1} fps", camera.fps),
            if below_target {
                Style::new().fg(Color::Yellow).add_modifier(Modifier::BOLD)
            } else {
                Style::new().fg(Color::Cyan)
            },
        );

        let mut caption = vec![Span::raw(" camera · ").dim(), rate];
        if below_target {
            caption.push(Span::raw(format!(" of {} ", camera.target_fps)).dim());
        }
        if camera.dropped > 0 {
            caption.push(Span::raw(" · ").dim());
            caption.push(Span::styled(
                format!("{} dropped", camera.dropped),
                Style::new().fg(Color::Yellow),
            ));
        }
        caption.push(
            Span::raw(match camera.consumers {
                0 => " · no viewer ".to_owned(),
                1 => " · 1 viewer ".to_owned(),
                n => format!(" · {n} viewers "),
            })
            .dim(),
        );
        caption
    }

    // ── the depth matrix ────────────────────────────────────────────────────

    /// The ToF frame as an 8×8 heatmap, one cell per zone.
    ///
    /// **Three classes, drawn three ways**, because the sensor answers three
    /// different things and a distance-only grid hides two of them:
    ///
    ///  - a range: the distance in metres, coloured near-to-far
    ///  - nothing in range: `·`, dim — the sensor looked and the space is empty,
    ///    which is information a map wants
    ///  - a failed measurement: `x` — it could not tell, which is *not* the same
    ///    as empty and must never look like it
    ///
    /// Row 0 is the sensor's first row as the driver reports it. No reprojection
    /// happens anywhere in this daemon path, so this is the sensor's own frame,
    /// not the robot's — which is exactly what someone debugging a mounting angle
    /// wants to see.
    fn render_tof(&self, frame: &mut ratatui::Frame, area: ratatui::layout::Rect) {
        let block = Block::bordered()
            .title(Line::from(self.tof_title()))
            .title_bottom(Line::from(TOF_LEGEND.to_owned()).dim())
            .title_bottom(Line::from(self.tof_caption()).right_aligned());
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let Some(tof) = self.tof.as_ref() else {
            frame.render_widget(Paragraph::new(self.tof_absence()).dim(), inner);
            return;
        };

        let rows = usize::from(tof.rows).min(usize::from(inner.height));
        let cols = usize::from(tof.cols);
        // Cells are fixed-width so the grid stays a grid: a column that shrank to
        // fit would stop being comparable with the one beside it.
        let per_row = usize::from(inner.width.saturating_sub(1)) / TOF_CELL;
        let drawn = cols.min(per_row.max(1));

        // Reproject through the head FK when the robot stream is up, so the
        // grid can say which returns are just the floor. The joint indices are
        // `JOINT_NAMES` order: neck_pitch, head_pitch, head_yaw, head_roll.
        let classified = self.classified_tof();

        let lines: Vec<Line> = (0..rows)
            .map(|row| {
                // A leading space so the first column is not against the border —
                // a number touching a box edge reads as part of it.
                let mut cells = vec![Span::raw(" ")];
                cells.extend((0..drawn).map(|col| {
                    let i = row * cols + col;
                    let class = classified.as_ref().and_then(|c| c.get(i).copied());
                    tof_cell(frame_zone(tof, i), class)
                }));
                Line::from(cells)
            })
            .collect();
        frame.render_widget(Paragraph::new(lines), inner);
    }

    /// The camera frame, drawn in half-blocks. See [`camera`] for what it costs and why it only
    /// costs it while this block is open.
    fn render_camera(&self, frame: &mut ratatui::Frame, area: ratatui::layout::Rect) {
        let block = Block::bordered()
            .title(Line::from(self.camera.title()))
            .title_bottom(Line::from(" c hides ".to_owned()).dim())
            .title_bottom(self.camera.caption().right_aligned());
        let inner = block.inner(area);
        let absence = self.camera.absence();
        frame.render_widget(block, area);
        if self.camera.has_picture() {
            self.camera.draw(inner, frame.buffer_mut());
        } else {
            frame.render_widget(Paragraph::new(absence).dim(), inner);
        }
    }

    /// The frame's zones through the head FK, when both streams are up: which
    /// returns are floor, which are obstacles, and where each sits in the
    /// trunk frame. The joint indices are `JOINT_NAMES` order — neck_pitch,
    /// head_pitch, head_yaw, head_roll at 5..9.
    fn classified_tof(
        &self,
    ) -> Option<[kinematics::tof::Zone; kinematics::tof::ROWS * kinematics::tof::COLS]> {
        let tof = self.tof.as_ref()?;
        let state = self.latest.as_ref()?;
        let head: Vec<f64> = (5..9)
            .filter_map(|i| state.joints.get(i).copied())
            .collect();
        let head: [f64; 4] = head.try_into().ok()?;
        let mut ranges = [None; kinematics::tof::ROWS * kinematics::tof::COLS];
        for (i, slot) in ranges.iter_mut().enumerate() {
            if let tof_zone::Zone::Range(m) = frame_zone(tof, i) {
                *slot = Some(f64::from(m));
            }
        }
        // The trunk's own tilt and measured height, so a robot leaned by hand
        // still calls the floor the floor. Odometry Z of zero means "no
        // estimate" (an older robotd, or an unconverged IMU) — fall back to
        // the model's rest height rather than believing the trunk is buried.
        let posture = kinematics::tof::Posture {
            gravity: state.safety.gravity,
            trunk_height_m: (state.odom.position[2] > 0.02).then_some(state.odom.position[2]),
        };
        Some(self.reprojector.project(&ranges, head, &posture))
    }

    /// The depth frame as contact points for the 3D view: yellow for a thing,
    /// green for confirmed floor — the same colours the grid uses. Empty when
    /// the frame is stale, because points hanging where the sensor no longer
    /// looks would be a lie.
    fn tof_markers(&self) -> Vec<duck::Marker> {
        if !self.tof_arrived.is_some_and(|at| at.elapsed() <= TOF_STALE) {
            return Vec::new();
        }
        let Some(zones) = self.classified_tof() else {
            return Vec::new();
        };
        zones
            .iter()
            .filter_map(|zone| {
                let (point, rgb) = match zone {
                    kinematics::tof::Zone::Hit { point, .. } => (point, [225, 200, 70]),
                    kinematics::tof::Zone::Floor { point } => (point, [70, 160, 80]),
                    _ => return None,
                };
                Some(duck::Marker {
                    at: point.map(|v| v as f32),
                    rgb,
                })
            })
            .collect()
    }

    /// `tof <sensor> · <hz> · <rows>×<cols> · <n>/<total> ranged · <min>–<max> m`
    fn tof_title(&self) -> Vec<Span<'static>> {
        let mut title = vec![Span::raw(" tof ")];
        match self.tof_status.as_ref() {
            Some(status) => {
                let name = status
                    .sensor
                    .clone()
                    .unwrap_or_else(|| "no sensor".to_owned());
                title.push(Span::styled(
                    name,
                    Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
                ));
                title.push(Span::raw(format!(" · {} Hz", status.hz)));
                title.push(Span::raw(format!(" · {}×{} ", status.rows, status.cols)));
            }
            None => title.push(Span::raw("connecting… ").dim()),
        }

        // What the frame on screen actually contains: how much of it is a
        // measurement, and the range of what it measured. A grid of numbers
        // without those two is a grid nobody can sanity-check at a glance.
        if let Some(tof) = self.tof.as_ref() {
            let ranges: Vec<f32> = (0..tof.distance_mm.len())
                .filter_map(|i| match frame_zone(tof, i) {
                    tof_zone::Zone::Range(m) => Some(m),
                    _ => None,
                })
                .collect();
            let total = tof.distance_mm.len();
            title.push(Span::raw(format!("· {}/{total} ranged ", ranges.len())));
            if let (Some(near), Some(far)) = (
                ranges.iter().copied().reduce(f32::min),
                ranges.iter().copied().reduce(f32::max),
            ) {
                title.push(Span::styled(
                    format!("· {near:.2}–{far:.2} m "),
                    Style::new().fg(Color::Cyan),
                ));
            }
        }
        title
    }

    /// The frame's own bookkeeping: which frame, and how long ago it landed.
    ///
    /// Staleness is measured by *this* view's clock rather than the frame's
    /// `at_us`, which is `tofd`'s — the question a reader has is "is what I am
    /// looking at current", and only the receiver can answer it.
    fn tof_caption(&self) -> Vec<Span<'static>> {
        let Some(tof) = self.tof.as_ref() else {
            return vec![Span::raw(" ")];
        };
        let mut caption = vec![Span::raw(format!(" seq {} ", tof.seq)).dim()];
        if let Some(arrived) = self.tof_arrived {
            let age = arrived.elapsed();
            // A frame is due every 1/hz; several periods late means the sensor
            // stopped, not that the terminal is slow.
            let stale = age > TOF_STALE;
            caption.push(Span::styled(
                format!("· {} ms ago ", age.as_millis()),
                if stale {
                    Style::new().fg(Color::Yellow)
                } else {
                    Style::new().dim()
                },
            ));
        }
        caption
    }

    /// What to say instead of a grid: `tofd` unreachable, no sensor fitted, or a
    /// sensor that has not produced its first frame yet. Three different fixes,
    /// so three different sentences.
    fn tof_absence(&self) -> String {
        if let Some(why) = self.tof_lost.as_ref() {
            return format!("no depth stream: {why}\nretrying — tofd may be restarting");
        }
        match self.tof_status.as_ref() {
            Some(status) => match status.unavailable.as_ref() {
                Some(why) => format!("no sensor: {why}"),
                None => "waiting for the first frame…".to_owned(),
            },
            None => "connecting to tofd…".to_owned(),
        }
    }

    /// The pad's own event stream: what the sticks are doing, and whether the reports carrying
    /// that are arriving.
    ///
    /// The second half is the reason this exists. A stick position is also visible in the header's
    /// `asked` column, one layer of interpretation later; the *cadence* of the reports behind it is
    /// visible nowhere else, and it is what fails when a robot walks on a command nobody is giving.
    fn render_pad(&self, frame: &mut ratatui::Frame, area: ratatui::layout::Rect) {
        // Before the block, because the caption has to say how many axes fit and the block is built
        // once. Two columns of border come off the width the cells get.
        let columns = usize::from(area.width.saturating_sub(2) / AXIS_CELL).max(1);
        let axis_count = self.pad.device.as_ref().map_or(0, |d| d.axes.len());

        let block = Block::bordered()
            .title(Line::from(self.pad_title()))
            .title_bottom(Line::from(self.pad_caption(columns, axis_count)).dim())
            .title_bottom(Line::from(self.pad_alarms()).right_aligned());
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let Some(device) = self.pad.device.as_ref() else {
            frame.render_widget(Paragraph::new(self.pad_absence()), inner);
            return;
        };

        // Fixed rows, in the order the questions arrive: is it arriving, has it been arriving, and
        // what it is saying — and, when the pad has one, where the pad is in space.
        let imu_height = if self.pad.imu.is_some() {
            IMU_HEIGHT
        } else {
            0
        };
        let [cadence, gaps, axes, held, imu] = Layout::vertical([
            Constraint::Length(1),
            Constraint::Length(1),
            Constraint::Length(AXIS_ROWS as u16),
            Constraint::Length(1),
            Constraint::Length(imu_height),
        ])
        .areas::<5>(inner);
        if let Some(unit) = self.pad.imu.as_ref() {
            render_imu(unit, frame, imu);
        }

        frame.render_widget(self.pad_cadence(), cadence);
        // A label beside the trace rather than a title above it: a bordered block for one row of
        // sparkline would cost three rows to draw one.
        let [label, trace] =
            Layout::horizontal([Constraint::Length(10), Constraint::Min(8)]).areas::<2>(gaps);
        frame.render_widget(Paragraph::new(Line::from(" gap ms").dim()), label);
        frame.render_widget(self.pad_gaps(), trace);
        frame.render_widget(Paragraph::new(self.pad_axes(device, columns)), axes);
        frame.render_widget(Paragraph::new(self.pad_held(device)), held);
    }

    /// Which pad, on which node, over what.
    fn pad_title(&self) -> Vec<Span<'static>> {
        let Some(device) = self.pad.device.as_ref() else {
            return vec![Span::raw(" pad · raw input ")];
        };
        let mut title = vec![
            Span::raw(" pad "),
            Span::styled(
                device.name.clone(),
                Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!(" · {} ", device.node)).dim(),
        ];
        if let Some(unique) = device.unique.as_deref() {
            // The address, because it is the join key: a `btmon` capture of the same minutes is
            // keyed on it, and this stream's timestamps are the kernel's, on the same clock.
            title.push(Span::raw(format!("· {unique} ")).dim());
        }
        if !device.over_bluetooth() {
            // A pad on a cable has no link to measure. Saying so beats reporting a flawless one —
            // `pad-link-test.sh` refuses to run at all in this case.
            title.push(Span::styled(
                "· on USB, so there is no radio here ",
                Style::new().fg(Color::Yellow),
            ));
        }
        if self.pad.imu.is_some() {
            // Loud on purpose: a pad with an IMU is news, and the block below has just grown.
            title.push(Span::styled(
                "· IMU ",
                Style::new().fg(Color::Magenta).add_modifier(Modifier::BOLD),
            ));
        }
        title
    }

    /// What the trace is drawn to, and how much of a link is behind it.
    ///
    /// On the border rather than in a row of its own, and always present: a sparkline with no stated
    /// scale is a shape, not a measurement, and the reader cannot tell a low bar from a short one.
    fn pad_caption(&self, columns: usize, axes: usize) -> Vec<Span<'static>> {
        // Terse on purpose. A bottom border is clipped from the right, and the count of what is
        // hidden sits at the end — so every word spent on the scale is a word that can push the
        // notice off a narrow screen. "Newest right" is not repeated here: the loop-rate trace
        // directly below says it, and both are drawn the same way.
        let mut caption = vec![Span::raw(format!(
            " {} reports · gap ≤{} ms ",
            self.pad.reports, GAP_FULL_SCALE_MS
        ))];
        // Never silently short: a grid that stops at the last cell that happened to fit is a pad
        // presented as having fewer axes than it has. The same rule the joints table follows.
        let room = columns * AXIS_ROWS;
        if axes > room {
            caption.push(Span::raw(format!("· {room} of {axes} axes ")));
        }
        caption
    }

    /// The counters that only ever mean something went wrong, on the block's bottom border.
    ///
    /// Kept out of the rows above because they are almost always zero and each one invalidates a
    /// different part of what is above it — a caption is where a qualification belongs.
    fn pad_alarms(&self) -> Vec<Span<'static>> {
        let pad = &self.pad;
        let mut alarms = Vec::new();
        if pad.after_drops > 0 {
            alarms.push(Span::styled(
                format!(
                    " syn_dropped {} — padd fell behind, not the radio ",
                    pad.after_drops
                ),
                Style::new().fg(Color::Magenta),
            ));
        }
        if pad.socket_dropped > 0 {
            alarms.push(Span::styled(
                format!(
                    " {} reports dropped reaching this view ",
                    pad.socket_dropped
                ),
                Style::new().fg(Color::Magenta),
            ));
        }
        if pad.clock_steps > 0 {
            alarms.push(Span::styled(
                format!(" the robot's clock stepped {}× ", pad.clock_steps),
                Style::new().fg(Color::Yellow),
            ));
        }
        if alarms.is_empty() {
            // Not blank: "nothing has invalidated any of this" is a thing the reader needs told,
            // and an empty border is indistinguishable from a view that does not check.
            alarms.push(Span::raw(" reports intact ").dim());
        }
        alarms
    }

    /// Why there is nothing to show. Always says what to do next.
    fn pad_absence(&self) -> Vec<Line<'static>> {
        let mut lines = vec![match self.pad.trouble.as_deref() {
            Some(trouble) => Line::from(vec![
                Span::raw(" no raw pad stream · "),
                Span::styled(trouble.to_owned(), Style::new().fg(Color::Yellow)),
            ]),
            None => Line::from(" waiting for padd to open a pad…").dim(),
        }];
        lines.push(
            Line::from(
                " `robotctl pad status` says whether a pad is connected and whether padd is running.",
            )
            .dim(),
        );
        // The counters outlive the device on purpose, so a pad that has just dropped still has its
        // measurement on screen — that is the moment someone looks.
        if self.pad.reports > 0 {
            lines.push(Line::from(vec![
                Span::raw(format!(
                    " last link · {} reports · worst gap {} ms · over {} ms {} · over {} ms ",
                    self.pad.reports,
                    self.pad.worst_ms,
                    proto::pad_link::NOTABLE_MS,
                    self.pad.over_notable,
                    proto::pad_link::DEADMAN_MS,
                )),
                Span::styled(
                    self.pad.over_deadman.to_string(),
                    if self.pad.over_deadman > 0 {
                        Style::new().fg(Color::Red)
                    } else {
                        Style::new().dim()
                    },
                ),
            ]));
        }
        lines
    }

    /// Is it arriving, and how fast.
    fn pad_cadence(&self) -> Paragraph<'static> {
        let pad = &self.pad;
        // Before any report there is no rate, no age and no worst gap, and a row of dashes and
        // zeroes would read as a link delivering nothing rather than as a pad nobody has touched.
        // An evdev device is silent until something moves, so this is the ordinary opening state.
        if pad.reports == 0 {
            return Paragraph::new(
                Line::from(" cadence  nothing yet — an evdev pad sends nothing until it moves")
                    .dim(),
            );
        }

        let mut line = vec![Span::raw(" cadence  ")];
        match pad.cadence() {
            Some(rate) => line.push(Span::styled(
                format!("{rate:.0}/s"),
                Style::new().fg(Color::Cyan),
            )),
            // Not "0/s": reports have arrived but no gap has been measurable yet — one report, or
            // nothing but quiet spells — and a zero rate reads as a dead link.
            None => line.push(Span::raw("—").dim()),
        }
        line.push(Span::raw(" while driving · last ").dim());

        match pad.silence() {
            Some((age, verdict)) => {
                let (word, style) = match verdict {
                    Silence::Arriving => (String::new(), Style::new().fg(Color::Green)),
                    Silence::Notable => (" — sticky".to_owned(), Style::new().fg(Color::Yellow)),
                    Silence::PastTheDeadman => (
                        format!(" — past the {} ms deadman", proto::pad_link::DEADMAN_MS),
                        Style::new().fg(Color::Red).add_modifier(Modifier::BOLD),
                    ),
                    // The distinction that matters: this is a pad on a table, not a dead radio.
                    Silence::Idle => (
                        " — the sticks are still".to_owned(),
                        Style::new().add_modifier(Modifier::DIM),
                    ),
                };
                line.push(Span::styled(format!("{} ms ago", age.as_millis()), style));
                if !word.is_empty() {
                    line.push(Span::styled(word, style));
                }
            }
            None => line.push(Span::raw("nothing yet").dim()),
        }

        line.push(Span::raw(format!(
            " · worst {} ms · over {} ms {} · over {} ms ",
            self.pad.worst_ms,
            proto::pad_link::NOTABLE_MS,
            self.pad.over_notable,
            proto::pad_link::DEADMAN_MS,
        )));
        line.push(Span::styled(
            self.pad.over_deadman.to_string(),
            if self.pad.over_deadman > 0 {
                Style::new().fg(Color::Red).add_modifier(Modifier::BOLD)
            } else {
                Style::new().dim()
            },
        ));
        if self.pad.quiet > 0 {
            line.push(Span::raw(format!(" · {} quiet spells", self.pad.quiet)).dim());
        }

        Paragraph::new(Line::from(line))
    }

    /// Every gap between reports while driving, newest right.
    ///
    /// Drawn to a fixed [`GAP_FULL_SCALE_MS`] rather than to the tallest gap on screen: an
    /// auto-scaled trace moves its own baseline as the window slides, so a link stalling every
    /// second draws exactly like a healthy one. Quiet spells are left out entirely — they are the
    /// operator's hand, and a full-height bar for every pause would bury the stalls this is
    /// looking for.
    fn pad_gaps(&self) -> Sparkline<'static> {
        // Floored at one level, which is the smallest mark a sparkline can make. An 8 ms gap is 8%
        // of the scale and rounds to a blank cell, so a link behaving perfectly drew an empty row —
        // indistinguishable from a trace nobody is feeding, which is the one thing it must not be.
        // A bar at the floor claims only "a report arrived here, below the first step the row can
        // resolve"; the cadence line carries the precision.
        // `div_ceil`, not `/`: a sparkline row is eight ticks and the widget scales with integer
        // arithmetic, so `max / 8` lands a tick *below* the first one that draws anything.
        let floor = GAP_FULL_SCALE_MS.div_ceil(8);
        let gaps: Vec<u64> = self.pad.gaps.iter().map(|ms| (*ms).max(floor)).collect();
        Sparkline::default()
            .data(gaps)
            .direction(RenderDirection::RightToLeft)
            .max(GAP_FULL_SCALE_MS)
            .style(Style::new().fg(Color::Cyan))
    }

    /// Where every axis is, against the range the device declares for it.
    fn pad_axes(&self, device: &proto::PadInputDevice, columns: usize) -> Vec<Line<'static>> {
        let mut lines = Vec::new();
        let mut axes = device.axes.iter();
        for _ in 0..AXIS_ROWS {
            let row: Vec<&proto::PadAxis> = axes.by_ref().take(columns).collect();
            if row.is_empty() {
                break;
            }
            let mut spans = Vec::new();
            for axis in row {
                spans.extend(axis_cell(axis, self.pad.axes.get(&axis.code).copied()));
            }
            lines.push(Line::from(spans));
        }
        lines
    }

    /// Which buttons are down, by the name the kernel gives them.
    ///
    /// Raw names rather than `padd`'s vocabulary — `BTN_START`, not "Start". The mapping from one to
    /// the other is gilrs's SDL database, and a pad whose Start does nothing is a pad someone needs
    /// the raw code of.
    fn pad_held(&self, device: &proto::PadInputDevice) -> Line<'static> {
        let mut names: Vec<String> = device
            .buttons
            .iter()
            .filter(|button| self.pad.held.contains(&button.code))
            .map(|button| button.name.clone())
            .collect();
        // A code the device never declared but is sending anyway. Worth showing rather than
        // dropping: it is either a pad this build has not seen or a stream nobody understands yet.
        names.extend(
            self.pad
                .held
                .iter()
                .filter(|code| !device.buttons.iter().any(|b| b.code == **code))
                .map(|code| format!("{}:{code}", proto::PadEvent::KEY)),
        );
        names.sort();

        if names.is_empty() {
            return Line::from(" held     none").dim();
        }
        Line::from(vec![
            Span::raw(" held     "),
            Span::styled(names.join(" "), Style::new().fg(Color::Green)),
        ])
    }

    /// Achieved loop rate over time.
    ///
    /// The instantaneous number in the header cannot show a *dropout*: a loop that fell to
    /// 20 Hz for half a second and recovered reads as 50 Hz by the time anybody looks. This
    /// is where a robot that stutters every few seconds becomes visible.
    fn trace(&self) -> Sparkline<'_> {
        let (low, high) = self
            .trace
            .iter()
            .fold((u64::MAX, 0), |(lo, hi), &v| (lo.min(v), hi.max(v)));
        let title = if self.trace.is_empty() {
            " loop rate ".to_owned()
        } else {
            format!(
                " loop rate · {low}–{high} Hz over {} frames · full height {} Hz · newest right ",
                self.frames, self.peak
            )
        };
        // Scaled to the best rate seen this session rather than to the tallest bar on screen.
        // An auto-scaled trace moves its own baseline as the window slides, and a loop running
        // uniformly at half speed then draws exactly like a healthy one.
        Sparkline::default()
            .data(self.trace.iter().copied())
            .direction(RenderDirection::RightToLeft)
            .max(self.peak.max(1))
            .style(Style::new().fg(Color::Cyan))
            .block(Block::bordered().title(Line::from(title).alignment(Alignment::Left)))
    }
}

/// One axis: its name, where it is in its own range, and the raw value.
///
/// The name is the kernel's with `ABS_` taken off: `ABS_HAT0X` does not fit a cell that also has to
/// hold a bar and a five-digit value, and every axis here carries the same prefix, so dropping it
/// costs nothing a reader needs. Everything else is the raw code's own — no remapping, no gilrs
/// vocabulary.
///
/// The bar is centred for an axis whose range crosses zero and left-anchored for one that starts
/// there, because those are two different controls. A stick rests at the middle of −32768..32767 and
/// a trigger rests at the bottom of 0..1023 — drawing the trigger centred would show it half pulled
/// at rest, which is exactly the kind of quiet lie this view exists to avoid.
fn axis_cell(axis: &proto::PadAxis, value: Option<i32>) -> Vec<Span<'static>> {
    /// Cells of bar, leaving room in [`AXIS_CELL`] for the name and the value.
    const WIDTH: usize = 9;

    let name = axis.name.trim_start_matches("ABS_").to_owned();
    let Some(value) = value else {
        return vec![Span::raw(format!(" {name:<6}{:WIDTH$}{:>8}", "", "-")).dim()];
    };

    let span = f64::from(axis.max) - f64::from(axis.min);
    let fraction = if span > 0.0 {
        ((f64::from(value) - f64::from(axis.min)) / span).clamp(0.0, 1.0)
    } else {
        // A degenerate range the driver reported: draw nothing rather than divide by it.
        0.0
    };

    let bar = if axis.min < 0 {
        centred_bar(fraction, WIDTH)
    } else {
        let filled = (fraction * WIDTH as f64).round() as usize;
        format!("{}{}", "█".repeat(filled), "·".repeat(WIDTH - filled))
    };

    vec![
        Span::raw(format!(" {name:<6}")),
        Span::styled(bar, Style::new().fg(Color::Cyan)),
        Span::raw(format!("{value:>8}")),
    ]
}

/// A bar growing from the middle of `width` cells, for an axis that rests at centre.
fn centred_bar(fraction: f64, width: usize) -> String {
    let half = width / 2;
    let offset = ((fraction - 0.5) * 2.0 * half as f64).round();
    let cells = (offset.abs() as usize).min(half);
    if offset < 0.0 {
        format!(
            "{}{}│{}",
            "·".repeat(half - cells),
            "█".repeat(cells),
            "·".repeat(width - half - 1)
        )
    } else {
        format!(
            "{}│{}{}",
            "·".repeat(half),
            "█".repeat(cells),
            "·".repeat(width - half - 1 - cells)
        )
    }
}

/// How many joint rows this frame has. The names this build knows, extended by anything
/// extra the wire carried — a `robotd` speaking a longer joint vector than this `robotctl`
/// was built against must not have the tail silently dropped.
fn joint_rows(state: &proto::RobotState) -> usize {
    state
        .joints
        .len()
        .max(state.targets.len())
        .max(proto::JOINT_NAMES.len())
}

/// Say what a limit *means*, not just what it is called.
///
/// The wire carries `duck_control::safety::Limit`'s names — `deadman`, `joint_range`,
/// `not_finite`, `fallen` — and each one is a token whose meaning lives in a doc comment in
/// another crate. Anything unrecognised is passed through verbatim: a `robotd` newer than this
/// `robotctl` may have limits this build has never heard of, and printing the raw name is
/// strictly better than hiding it.
/// The first line of a reason, for somewhere that has one line to put it.
///
/// A connect failure is a paragraph — the `systemctl status` hint below it is the useful half,
/// and `robotctl health` trims the same way for the same reason. Dropped into a `Line` whole, the
/// rest of it draws as control characters across a row that is on screen at all times.
fn brief(reason: &str) -> &str {
    reason.lines().next().unwrap_or("unavailable")
}

fn explain_limit(limit: &str) -> String {
    match limit {
        "deadman" => "deadman — no intent arrived recently, velocity zeroed".to_owned(),
        "joint_range" => "joint_range — a target was outside the actuator's travel".to_owned(),
        "not_finite" => "not_finite — a target was NaN or infinite".to_owned(),
        "fallen" => "fallen — the robot is down, the policy is not driving".to_owned(),
        other => other.to_owned(),
    }
}

fn fall_verdict(safety: &proto::SafetyState) -> Span<'static> {
    if safety.fallen {
        Span::styled(
            "FALLEN",
            Style::new().fg(Color::Red).add_modifier(Modifier::BOLD),
        )
    } else {
        Span::styled("upright", Style::new().fg(Color::Green))
    }
}

/// Green while the joint is where it was told to be, red once it plainly is not. Thresholds
/// are fractions of the bar's own scale, so the colour and the bar always agree.
fn error_style(error: f64) -> Style {
    let magnitude = error.abs() / BAR_FULL_SCALE;
    if !error.is_finite() {
        Style::new().fg(Color::Magenta).add_modifier(Modifier::BOLD)
    } else if magnitude < 0.25 {
        Style::new().fg(Color::Green)
    } else if magnitude < 0.6 {
        Style::new().fg(Color::Yellow)
    } else {
        Style::new().fg(Color::Red)
    }
}

/// Tracking error as a bar growing from a fixed centre, left for negative, right for
/// positive.
///
/// Centred rather than left-anchored because the sign is diagnostic: a knee that lags
/// behind its target and a knee that overshoots it are different faults, and a bar drawn
/// from the left end makes them the same picture. Saturation is marked, so an error off the
/// scale cannot be mistaken for one that merely reaches the edge.
fn deviation_bar(error: Option<f64>) -> Line<'static> {
    let Some(error) = error.filter(|e| e.is_finite()) else {
        return Line::from(Span::raw("no reading").dim());
    };

    let cells = (error.abs() / BAR_FULL_SCALE * BAR_HALF as f64).round();
    let saturated = cells > BAR_HALF as f64;
    let filled = (cells as usize).min(BAR_HALF);
    let style = error_style(error);
    // A dim rail rather than blank space: the bar's extent is what makes its length mean
    // something, and a bar with no visible track is a number drawn in a different font.
    let pad = |n: usize| Span::raw("·".repeat(n)).dim();
    let bar = |n: usize| Span::styled("█".repeat(n), style);
    let edge = |mark: &'static str, on: bool| {
        if on {
            Span::styled(mark, style.add_modifier(Modifier::BOLD))
        } else {
            Span::raw(" ")
        }
    };

    if error < 0.0 {
        Line::from(vec![
            edge("«", saturated),
            pad(BAR_HALF - filled),
            bar(filled),
            Span::raw("│").dim(),
            pad(BAR_HALF),
            edge(" ", false),
        ])
    } else {
        Line::from(vec![
            edge(" ", false),
            pad(BAR_HALF),
            Span::raw("│").dim(),
            bar(filled),
            pad(BAR_HALF - filled),
            edge("»", saturated),
        ])
    }
}

/// Is stdout a terminal? Decides which of the two renderings runs.
pub(crate) fn stdout_is_a_terminal() -> bool {
    // SAFETY: `isatty` only inspects a file descriptor; it touches no memory of ours.
    unsafe { libc::isatty(libc::STDOUT_FILENO) == 1 }
}
