//! What clients are asking the robot to do.
//!
//! Written by IPC tasks, read once per tick by the control loop. The loop must never wait
//! on a client, so each slot is an [`ArcSwap`]: the reader does one atomic load and the
//! writer one atomic store, and neither can hold up the other.
//!
//! **Twist and head are separate slots on purpose.** A single combined slot would need
//! read-modify-write to update one field, and two clients — a gamepad driving the body and
//! something else driving the head — would silently lose each other's updates. Separate
//! slots make each one single-writer in practice, so last-writer-wins means what it says.
//!
//! Every slot is stamped, because the loop's real question is never "what is the value" but
//! "how old is it". That is what the deadman reads.

use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};

/// "No mode switch pending" in [`Intents::mode_switch`].
///
/// 255 rather than 0, because 0 is a real mode code and a sentinel that collides with a value is
/// how a "switch to walking" request becomes a no-op.
pub const MODE_NONE: u8 = u8::MAX;

/// Encodings for [`Intents::power`]. An `AtomicU8` rather than two bools, so "init" and "relax"
/// cannot both be pending — they are alternatives, and the last one asked for wins.
const POWER_NONE: u8 = 0;

/// Encodings for [`Intents::theremin`]. An edge rather than a level, and for a concrete
/// reason: the loop puts the instrument down by itself when the robot starts walking, and a
/// level slot still reading "up" would pick it straight back up with a background of
/// somewhere the duck no longer is.
const THEREMIN_NONE: u8 = 0;
const THEREMIN_UP: u8 = 1;
const THEREMIN_DOWN: u8 = 2;
const POWER_INIT: u8 = 1;
const POWER_RELAX: u8 = 2;
const REST_IDLE: u8 = 0;
const REST_RESTING: u8 = 1;
const REST_RESTED: u8 = 2;
const REST_FAILED: u8 = 3;
use std::time::{Duration, Instant};

use arc_swap::{ArcSwap, ArcSwapOption};
use duck_control::obs::{BodyPose, Command};

use crate::velocity_command::{GamepadTiming, VelocityCommandPulse};

/// A value and when it arrived.
#[derive(Debug, Clone, Copy)]
struct Stamped<T> {
    value: T,
    /// Microseconds since the [`Intents`] epoch.
    at_us: u64,
}

#[derive(Clone, Copy)]
struct HeadIntent {
    target: duck_ipc_proto::HeadParams,
    at_us: u64,
    hold: bool,
}

#[derive(Clone, Copy)]
struct TwistIntent {
    value: [f64; 3],
    at_us: u64,
    gamepad: bool,
    pulse: VelocityCommandPulse,
    rearmed_stop_generation: Option<u64>,
}

const MOTION_TRACKING_TIMEOUT_US: u64 = 500_000;
const AUTONOMOUS_SESSION_TIMEOUT_US: u64 =
    duck_ipc_proto::AUTONOMOUS_SESSION_TIMEOUT.as_micros() as u64;
// The preview and tracking start share one web control lease.
const HEAD_HANDOFF_TIMEOUT_US: u64 = 700_000;

#[derive(Clone, Copy, Default)]
struct HeadTrackingIntent {
    owner: u64,
    active: bool,
    at_us: u64,
    epoch: u64,
    sequence: u64,
    target: Option<[f64; 2]>,
    pitch_position: Option<u16>,
    deadzone: [f64; 2],
    gains: [[f64; 3]; 2],
}

#[derive(Debug, Clone, Copy)]
pub struct HeadTrackingSnapshot {
    pub target: Option<[f64; 2]>,
    pub pitch_position: Option<u16>,
    pub deadzone: [f64; 2],
    pub gains: [[f64; 3]; 2],
    pub sequence: u64,
}

#[derive(Clone, Copy, Default)]
struct MotionTrackingIntent {
    owner: u64,
    initial_pitch: Option<u16>,
    active: bool,
    at_us: u64,
    epoch: u64,
    sequence: u64,
    target: Option<[f64; 2]>,
    line_speed: Option<f64>,
    line_target_x: Option<f64>,
    reference_area: f64,
    deadzone: [f64; 2],
    gains: [[f64; 3]; 2],
}

#[derive(Debug, Clone, Copy)]
pub struct MotionTrackingSnapshot {
    pub initial_pitch: Option<u16>,
    pub target: Option<[f64; 2]>,
    pub line_speed: Option<f64>,
    pub line_target_x: Option<f64>,
    pub reference_area: f64,
    pub deadzone: [f64; 2],
    pub gains: [[f64; 3]; 2],
    pub sequence: u64,
}

#[derive(Clone, Default)]
struct CoordinatedTrackingIntent {
    owner: u64,
    initial_pitch: Option<u16>,
    at_us: u64,
    epoch: u64,
    sequence: u64,
    params: Option<duck_ipc_proto::CoordinatedTrackingParams>,
    pickup_after_active: bool,
}

#[derive(Debug, Clone)]
pub struct CoordinatedTrackingSnapshot {
    pub initial_pitch: Option<u16>,
    pub params: duck_ipc_proto::CoordinatedTrackingParams,
    pub sequence: u64,
    pub fresh: bool,
    pub age_us: u64,
    pub pickup_stage: Option<duck_ipc_proto::PickupStage>,
}

#[derive(Clone, Copy, Default)]
struct MazeIntent {
    owner: u64,
    at_us: u64,
    epoch: u64,
    sequence: u64,
    params: Option<duck_ipc_proto::MazeParams>,
}

#[derive(Debug, Clone, Copy)]
pub struct MazeSnapshot {
    pub sequence: u64,
    pub params: duck_ipc_proto::MazeParams,
}

/// The body-pose intent, as the loop consumes it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseIntent {
    /// z, roll, pitch — the order the observation's body block uses.
    pub body: [f64; 3],
    /// While true the loop glides toward `body`; false snaps it back to nominal at once,
    /// which is the prototype's B-button exit.
    pub active: bool,
}

impl Default for PoseIntent {
    fn default() -> Self {
        Self {
            body: [0.0; 3],
            active: false,
        }
    }
}

/// Pending one-shot skill requests, taken once per tick.
///
/// Booleans rather than a queue: within one 20 ms tick a second press of the same button
/// means nothing extra, and two *different* requests both deserve to be seen — which a
/// single last-writer slot would lose.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SkillRequests {
    /// Phase-scripted, and still its own thing until the descriptor grows a command generator.
    pub ground_pick: bool,
    /// Latched, and driven internally by the shutdown sit as well as
    /// by a button, which is why it is not one of the configurable one-shots either.
    pub sit_toggle: bool,
    /// The configurable one-shots, as a mask over the resolved skill list.
    ///
    /// A mask and not a queue for the same reason the whole set was one before: two buttons in
    /// one tick should both arrive, and the priority order decides which wins. A chaining skill
    /// is a *level* in practice — a client holds the button by re-sending every tick.
    pub skills: u32,
}

impl SkillRequests {
    pub fn any(&self) -> bool {
        self.ground_pick || self.sit_toggle || self.skills != 0
    }

    /// The indices requested, lowest first — which is priority order, since the skill list is
    /// ordered by config.
    pub fn requested(&self) -> impl Iterator<Item = usize> + '_ {
        (0..MAX_SKILLS).filter(move |i| self.skills & (1 << i) != 0)
    }
}

/// A one-shot request that must report the control loop's actual start decision.
pub(crate) struct ManualSkillRequest {
    pub name: String,
    pub deadline: Instant,
    pub reply: tokio::sync::oneshot::Sender<duck_ipc_proto::IntentResult>,
}

// Bit positions for the two skills that are not in the configurable list.
const SKILL_GROUND_PICK: u32 = 1 << 0;
const SKILL_SIT_TOGGLE: u32 = 1 << 1;

/// How many configurable one-shots a robot may have, bounded by the mask being a `u32`.
///
/// Not a number anybody should reach: every skill's network is loaded and warmed at startup, so
/// thirty of them is thirty ONNX sessions and thirty warm-up inferences before the first tick.
/// The ceiling is here so that exceeding it is a refusal with a reason rather than a silently
/// ignored button.
pub const MAX_SKILLS: usize = 30;

/// Bits the two non-configurable skills occupy at the bottom of the mask.
const SKILL_BITS: usize = 2;

/// How fresh a wheee hold must be to still count as held. `padd` re-notifies every tick
/// (20 ms) while the trigger is down, so anything much older means the client stopped
/// holding — or stopped existing. The stale hold lands the ride instead of looping forever.
pub const WHEEE_HOLD_FRESH: Duration = Duration::from_millis(300);
const MOUTH_REPEAT_FRESH: Duration = Duration::from_millis(300);

/// What the wheee level says, as the loop consumes it. Three states rather than a bool
/// because the two ways of *not* being held are different sounds: a client that spells out
/// `hold: false` wants the ride cut where it stands (the prototype's release), while a hold
/// that simply stopped arriving wants the ride played out through its end segment — nobody
/// asked for a cut there, the client just went away. Collapsing them loses the end segment
/// entirely, since a cut ride's writer only ever sees a broken pipe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WheeeHold {
    /// A `hold: true` fresh enough to trust.
    Held,
    /// A client that is still there and said to stop.
    Released,
    /// A hold that went stale — the client stopped re-notifying, or stopped existing.
    Decayed,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum SoundHoldTarget {
    Released,
    Wheee,
    Clip(Arc<str>),
}

/// The current exclusive loop and its freshness, read as one atomic snapshot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SoundHold {
    Wheee(WheeeHold),
    Clip(Arc<str>),
}

pub struct Intents {
    /// Epoch for every stamp. `Instant` so the clock cannot run backwards under us.
    epoch: Instant,
    max_linear: Option<f64>,
    max_lateral: Option<f64>,
    max_angular: Option<f64>,
    twist: ArcSwap<TwistIntent>,
    max_backward: Option<f64>,
    gamepad_timing: GamepadTiming,
    stop_generation: std::sync::atomic::AtomicU64,
    abandon_generation: std::sync::atomic::AtomicU64,
    abandon_head_generation: std::sync::atomic::AtomicU64,
    precision: ArcSwapOption<PrecisionRequest>,
    last_precision: ArcSwapOption<PrecisionRequest>,
    precision_running: AtomicBool,
    head: ArcSwap<HeadIntent>,
    head_generation: std::sync::atomic::AtomicU64,
    head_tracking: ArcSwap<HeadTrackingIntent>,
    head_tracking_epoch: std::sync::atomic::AtomicU64,
    motion_tracking: ArcSwap<MotionTrackingIntent>,
    motion_tracking_epoch: std::sync::atomic::AtomicU64,
    coordinated_tracking: ArcSwap<CoordinatedTrackingIntent>,
    coordinated_tracking_epoch: std::sync::atomic::AtomicU64,
    maze: ArcSwap<MazeIntent>,
    maze_epoch: std::sync::atomic::AtomicU64,
    /// Standing body pose. Unstamped: `active: false` is its own "nobody is posing".
    pose: ArcSwap<PoseIntent>,
    /// Continuous jaw position and the most recent request to repeat its sound.
    mouth: ArcSwap<Stamped<duck_ipc_proto::MouthParams>>,
    accessory_trigger: ArcSwapOption<Stamped<u8>>,
    /// Whether the policy should drive. Discrete, so a plain flag rather than a slot.
    enabled: AtomicBool,
    /// A pending `robot.init` / `robot.relax`, as [`PowerRequest`].
    ///
    /// A request rather than a flag, and taken rather than read: powering the joints is an *edge*,
    /// not a state the loop should keep re-applying. One `set_torque` is a bus transaction per
    /// joint, so a level here would put sixteen writes into every tick for as long as it stayed set.
    ///
    /// It lives with the intents because this is where the loop reads what clients asked for, and
    /// because the loop is the only thing that may touch the bus — the IPC task cannot do it itself.
    power: AtomicU8,
    /// A pending pick-up or put-down for the ToF theremin. See [`THEREMIN_NONE`].
    theremin: AtomicU8,
    /// The same, for the chorale.
    chorale: AtomicU8,
    /// The piece pinned by the pending chorale request, 0 for none. Written before the
    /// request flag, read after — the flag's swap is the synchronisation point.
    chorale_piece: AtomicU8,
    /// Beacons `btd` has heard, waiting for the loop.
    ///
    /// A queue behind a mutex rather than an `ArcSwap` slot, unlike every other intent here, and
    /// for a specific reason: the other intents are *levels*, where a value missed is a value
    /// superseded. A beacon is an *event* carrying an arrival time, and a beat dropped by
    /// last-writer-wins is a beat the phase lock never gets to average.
    chorale_heard: std::sync::Mutex<Vec<duck_ipc_proto::ChoraleHeard>>,
    /// Pending skill requests, a bitmask taken (swapped to zero) once per tick. A mask
    /// rather than one slot so two different buttons in the same tick both arrive.
    skills: std::sync::atomic::AtomicU32,
    manual_skill: Mutex<Option<ManualSkillRequest>>,
    /// A shutdown was requested. A level, not an edge: once asked, the sequence runs.
    shutdown: AtomicBool,
    confirmed_shutdown: AtomicBool,
    /// Rest uses the shutdown posture sequence without powering off.
    rest: AtomicBool,
    rest_phase: AtomicU8,
    /// A drive-mode switch was requested, and which mode to switch to.
    ///
    /// An `AtomicU8` holding [`MODE_NONE`] or a mode's code, for the same reason `shutdown` is a
    /// flag: the IPC thread writes and the control loop takes, with nothing to lock. The last
    /// request wins — two arriving in the same tick means somebody pressed twice, and the second
    /// answer is the one they are waiting for.
    mode_switch: AtomicU8,
    /// A pending policy change, or `None` — which is nearly always.
    ///
    /// An `ArcSwapOption` rather than the `AtomicU8` `mode_switch` uses, because the request
    /// carries a path and a mode code does not. Same last-writer-wins semantics for the same
    /// reason: two loads in one tick means somebody changed their mind, and the second is the
    /// answer they are waiting for.
    ///
    /// Unlike `mode_switch` this does name a domain type, [`crate::params::Slot`]. The mode code
    /// exists because `Mode` lives in `main.rs` and this module should not reach into it; `Slot`
    /// lives in the shared params crate that owns the config keys it names, and encoding one as
    /// a number here would only move the parse somewhere with less to check it against.
    policy_change: ArcSwapOption<PolicyChange>,
    /// Pending one-shot sound tags, a bitmask taken once per tick like the skills.
    sounds: std::sync::atomic::AtomicU32,
    /// A one-shot clip is taken once, with the last request winning within a tick.
    sound_clip: ArcSwapOption<String>,
    /// Default and custom loops share one stamped slot so switching is atomic.
    sound_hold: ArcSwap<Stamped<SoundHoldTarget>>,
    /// `robot.rebootMotors`, pending: the ids to reboot (empty = all). A mutex rather than an
    /// atomic because it carries a list; the loop takes it with `try_lock`, so it can never wait.
    reboot_motors: Mutex<Option<Vec<u8>>>,
}

/// A pending policy change, as [`Intents::request_policy_change`] took it.
///
/// An enum and not the wire type's pair of `Option`s, because there are three things to say and
/// two `Option`s can only say four — one of which is meaningless and one of which was, briefly,
/// wrong. [`Self::Reload`] and [`Self::ResetAll`] both name no slot and no path, and conflating
/// them wiped every override on a board whenever a newer policy set was installed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PolicyChange {
    /// Put one slot on a file, or — with `path: None` — back to its default.
    Slot {
        slot: crate::params::Slot,
        path: Option<std::path::PathBuf>,
    },
    /// Every slot back to its default. `robotctl policy reset`.
    ResetAll,
    /// Re-resolve and rebuild, changing no configuration at all.
    ///
    /// For when the *files* moved and the paths did not: installing a policy set swaps
    /// `current` underneath every slot, so the config is already right and only the loaded
    /// sessions are stale.
    Reload,
}

/// What a client asked for, once.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerRequest {
    /// Torque on, ramp to the home pose.
    Init,
    /// Torque off. The robot collapses if nothing holds it.
    Relax,
}

#[derive(Debug)]
pub struct PrecisionRequest {
    pub params: duck_ipc_proto::PrecisionMoveParams,
    pub stop_generation: u64,
    pub interrupt_precision: bool,
    cancel_requested: AtomicBool,
    finished: AtomicBool,
}

impl PrecisionRequest {
    pub fn finish(&self) {
        self.finished.store(true, Ordering::Release);
    }
}

/// What the loop reads at the top of a tick.
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub command: Command,
    /// Age of the most recent *twist*, which is what the deadman guards. A stale head pose
    /// is harmless; a stale velocity walks the robot into a wall.
    pub twist_age: Duration,
    pub gamepad_twist: bool,
    pub gamepad_rearmed_stop_generation: Option<u64>,
    /// Explicit stops cancel any pending minimum-duration command in the loop.
    pub stop_generation: u64,
    pub abandon_generation: u64,
    pub abandon_head_generation: u64,
    /// Age of the most recent head intent, so idle yawning yields to its owner.
    pub head_age: Duration,
    pub head_position_raw: Option<[u16; 2]>,
    pub head_position_held: bool,
    pub head_generation: u64,
    pub head_tracking: Option<HeadTrackingSnapshot>,
    pub head_position_handoff: Option<u16>,
    pub motion_tracking: Option<MotionTrackingSnapshot>,
    pub coordinated_tracking: Option<CoordinatedTrackingSnapshot>,
    pub maze: Option<MazeSnapshot>,
    pub enabled: bool,
    /// The body-pose intent. The loop smooths `body` into `command.body` itself, because
    /// smoothing is per-tick state the intent slots must not own.
    pub pose: PoseIntent,
    /// Mouth opening, 0..1.
    pub mouth: f64,
}

impl Default for Intents {
    fn default() -> Self {
        Self::new()
    }
}

impl Intents {
    pub fn new() -> Self {
        Self::new_with_motion_limits(None, None)
    }

    pub fn new_with_motion_limits(max_linear: Option<f64>, max_angular: Option<f64>) -> Self {
        Self::new_with_gamepad_limits(
            max_linear, max_linear, max_angular, GamepadTiming::default(),
        )
    }

    pub fn new_with_gamepad_limits(
        max_linear: Option<f64>,
        max_backward: Option<f64>,
        max_angular: Option<f64>,
        gamepad_timing: GamepadTiming,
    ) -> Self {
        Self::new_with_axis_limits(max_linear, max_linear, max_backward, max_angular, gamepad_timing)
    }

    pub fn new_with_axis_limits(
        max_linear: Option<f64>,
        max_lateral: Option<f64>,
        max_backward: Option<f64>,
        max_angular: Option<f64>,
        gamepad_timing: GamepadTiming,
    ) -> Self {
        let epoch = Instant::now();
        Self {
            epoch,
            max_linear,
            max_lateral,
            max_backward,
            max_angular,
            gamepad_timing,
            // Stamped at zero, so before any client connects the twist already reads as
            // maximally stale and the deadman holds the robot still. Starting "fresh" would
            // mean a robot that briefly believes it has a live driver.
            twist: ArcSwap::from_pointee(TwistIntent {
                value: [0.0; 3],
                at_us: 0,
                gamepad: false,
                pulse: VelocityCommandPulse::default(),
                rearmed_stop_generation: None,
            }),
            stop_generation: std::sync::atomic::AtomicU64::new(0),
            abandon_generation: std::sync::atomic::AtomicU64::new(0),
            abandon_head_generation: std::sync::atomic::AtomicU64::new(0),
            precision: ArcSwapOption::empty(),
            last_precision: ArcSwapOption::empty(),
            precision_running: AtomicBool::new(false),
            head: ArcSwap::from_pointee(HeadIntent {
                target: duck_ipc_proto::HeadParams::default(),
                at_us: 0,
                hold: false,
            }),
            head_generation: std::sync::atomic::AtomicU64::new(0),
            head_tracking: ArcSwap::from_pointee(HeadTrackingIntent::default()),
            head_tracking_epoch: std::sync::atomic::AtomicU64::new(0),
            motion_tracking: ArcSwap::from_pointee(MotionTrackingIntent::default()),
            motion_tracking_epoch: std::sync::atomic::AtomicU64::new(0),
            coordinated_tracking: ArcSwap::from_pointee(CoordinatedTrackingIntent::default()),
            coordinated_tracking_epoch: std::sync::atomic::AtomicU64::new(0),
            maze: ArcSwap::from_pointee(MazeIntent::default()),
            maze_epoch: std::sync::atomic::AtomicU64::new(0),
            pose: ArcSwap::from_pointee(PoseIntent::default()),
            mouth: ArcSwap::from_pointee(Stamped {
                value: duck_ipc_proto::MouthParams::default(),
                at_us: 0,
            }),
            accessory_trigger: ArcSwapOption::empty(),
            enabled: AtomicBool::new(false),
            power: AtomicU8::new(POWER_NONE),
            theremin: AtomicU8::new(THEREMIN_NONE),
            chorale: AtomicU8::new(THEREMIN_NONE),
            chorale_piece: AtomicU8::new(0),
            chorale_heard: std::sync::Mutex::new(Vec::new()),
            skills: std::sync::atomic::AtomicU32::new(0),
            manual_skill: Mutex::new(None),
            shutdown: AtomicBool::new(false),
            confirmed_shutdown: AtomicBool::new(false),
            rest: AtomicBool::new(false),
            rest_phase: AtomicU8::new(REST_IDLE),
            mode_switch: AtomicU8::new(MODE_NONE),
            policy_change: ArcSwapOption::empty(),
            sounds: std::sync::atomic::AtomicU32::new(0),
            sound_clip: ArcSwapOption::empty(),
            reboot_motors: Mutex::new(None),
            sound_hold: ArcSwap::from_pointee(Stamped {
                value: SoundHoldTarget::Released,
                at_us: 0,
            }),
        }
    }

    fn now_us(&self) -> u64 {
        self.epoch.elapsed().as_micros() as u64
    }

    pub fn gamepad_timing(&self) -> GamepadTiming {
        self.gamepad_timing
    }

    pub fn max_angular(&self) -> Option<f64> {
        self.max_angular
    }

    pub fn limit_twist(&self, twist: [f64; 3]) -> [f64; 3] {
        [
            self.max_linear.map_or(twist[0], |limit| twist[0].clamp(-limit, limit)),
            self.max_lateral.map_or(twist[1], |limit| twist[1].clamp(-limit, limit)),
            self.max_angular.map_or(twist[2], |limit| twist[2].clamp(-limit, limit)),
        ]
    }

    pub fn set_twist(&self, twist: [f64; 3]) {
        self.set_twist_from(twist, false);
    }

    pub fn set_gamepad_twist(&self, twist: [f64; 3]) {
        self.set_twist_from(twist, true);
    }

    fn set_twist_from(&self, twist: [f64; 3], gamepad: bool) {
        let twist = self.limit_twist(twist);
        let at_us = self.now_us();
        self.twist.rcu(|previous| {
            let mut next = **previous;
            if gamepad {
                next.pulse.update(
                    twist,
                    Duration::from_micros(at_us),
                    self.max_linear,
                    self.max_lateral,
                    self.max_backward,
                    self.max_angular,
                    self.gamepad_timing,
                );
            } else {
                next.pulse = VelocityCommandPulse::default();
                next.rearmed_stop_generation = None;
            }
            next.value = twist;
            next.at_us = at_us;
            next.gamepad = gamepad;
            Arc::new(next)
        });
    }

    pub fn cancel_gamepad_twist(&self, rearmed_stop_generation: Option<u64>) {
        self.twist.rcu(|previous| {
            let mut next = **previous;
            if next.gamepad && next.rearmed_stop_generation == rearmed_stop_generation {
                next.pulse.cancel();
                next.value = [0.0; 3];
            }
            Arc::new(next)
        });
    }

    /// A newly granted gamepad lease follows a client-side neutral observation.
    pub fn rearm_gamepad_twist(&self) {
        let generation = self.stop_generation.load(Ordering::Acquire);
        let at_us = self.now_us();
        self.twist.rcu(|previous| {
            let mut next = **previous;
            next.value = [0.0; 3];
            next.at_us = at_us;
            next.gamepad = false;
            next.pulse = VelocityCommandPulse::default();
            next.rearmed_stop_generation = Some(generation);
            Arc::new(next)
        });
    }

    pub fn set_head(&self, head: duck_ipc_proto::HeadParams) {
        if self.head.load().target != head {
            self.head_generation.fetch_add(1, Ordering::AcqRel);
        }
        self.head.store(Arc::new(HeadIntent {
            target: head,
            at_us: self.now_us(),
            hold: false,
        }));
    }

    pub fn clear_head(&self) {
        self.head.store(Arc::new(HeadIntent {
            target: duck_ipc_proto::HeadParams::default(),
            at_us: self.now_us(),
            hold: false,
        }));
    }

    pub fn hold_manual_head_on_release(&self) {
        let head = **self.head.load();
        if head.target.raw_position.is_some() && self.now_us().saturating_sub(head.at_us) <= 250_000 {
            self.head.store(Arc::new(HeadIntent { hold: true, ..head }));
        } else {
            self.clear_head();
        }
    }

    pub fn motion_tracking_active(&self) -> bool {
        let tracking = self.motion_tracking.load();
        tracking.active && tracking.epoch == self.motion_tracking_epoch.load(Ordering::Acquire)
    }

    pub fn head_tracking_active(&self) -> bool {
        let tracking = self.head_tracking.load();
        tracking.active
            && self.now_us().saturating_sub(tracking.at_us) <= MOTION_TRACKING_TIMEOUT_US
            && tracking.epoch == self.head_tracking_epoch.load(Ordering::Acquire)
    }

    pub fn cancel_head_tracking(&self) {
        self.head_tracking_epoch.fetch_add(1, Ordering::AcqRel);
    }

    pub fn update_head_tracking(&self, owner: u64, params: duck_ipc_proto::HeadTrackingParams) -> bool {
        use duck_ipc_proto::MotionTrackingAction;
        if !params.valid() {
            return false;
        }
        let now = self.now_us();
        let epoch = self.head_tracking_epoch.load(Ordering::Acquire);
        let accepted = AtomicBool::new(false);
        self.head_tracking.rcu(|previous| {
            let valid = previous.active && now.saturating_sub(previous.at_us) <= MOTION_TRACKING_TIMEOUT_US
                && previous.epoch == epoch;
            let mut next = **previous;
            let allow = match params.action {
                MotionTrackingAction::Start => !valid || previous.owner == owner,
                MotionTrackingAction::Update => valid && previous.owner == owner,
                MotionTrackingAction::Stop if params.pitch_position.is_some() =>
                    valid && previous.owner == owner && previous.pitch_position == params.pitch_position,
                MotionTrackingAction::Stop => !valid || previous.owner == owner,
            };
            accepted.store(allow, Ordering::Relaxed);
            if allow {
                match params.action {
                    MotionTrackingAction::Start | MotionTrackingAction::Update => {
                        next = HeadTrackingIntent { owner, active: true, at_us: now, epoch,
                            sequence: if params.action == MotionTrackingAction::Start { previous.sequence.wrapping_add(1) } else { previous.sequence },
                            target: params.target, pitch_position: params.pitch_position,
                            deadzone: params.deadzone, gains: params.gains };
                    }
                    MotionTrackingAction::Stop => {
                        next.active = false;
                        next.pitch_position = params.pitch_position;
                        next.at_us = now;
                    }
                }
            }
            Arc::new(next)
        });
        accepted.load(Ordering::Relaxed)
    }

    fn head_position_handoff(&self, owner: u64, now: u64) -> Option<u16> {
        let head = self.head_tracking.load();
        (!head.active && head.owner == owner
            && head.epoch == self.head_tracking_epoch.load(Ordering::Acquire)
            && now.saturating_sub(head.at_us) <= HEAD_HANDOFF_TIMEOUT_US)
            .then_some(head.pitch_position).flatten()
    }

    fn clear_head_position_handoff(&self, owner: u64) {
        self.head_tracking.rcu(|previous| {
            let mut next = **previous;
            if !next.active && next.owner == owner { next.pitch_position = None; }
            Arc::new(next)
        });
    }

    pub fn cancel_motion_tracking(&self) {
        self.motion_tracking_epoch.fetch_add(1, Ordering::AcqRel);
    }

    pub fn coordinated_tracking_active(&self) -> bool {
        let tracking = self.coordinated_tracking.load();
        tracking.params.is_some()
            && tracking.epoch == self.coordinated_tracking_epoch.load(Ordering::Acquire)
    }

    pub fn cancel_coordinated_tracking(&self) {
        self.coordinated_tracking_epoch.fetch_add(1, Ordering::AcqRel);
    }

    pub fn update_coordinated_tracking(
        &self,
        owner: u64,
        params: duck_ipc_proto::CoordinatedTrackingParams,
    ) -> bool {
        use duck_ipc_proto::MotionTrackingAction;
        if !params.valid() { return false; }
        let now = self.now_us();
        let epoch = self.coordinated_tracking_epoch.load(Ordering::Acquire);
        let initial_pitch = params.initial_pitch.or_else(|| self.head_position_handoff(owner, now));
        let accepted = AtomicBool::new(false);
        self.coordinated_tracking.rcu(|previous| {
            let valid = previous.params.is_some() && previous.epoch == epoch;
            let allow = match params.action {
                MotionTrackingAction::Start => !valid || previous.owner == owner,
                MotionTrackingAction::Update => valid && previous.owner == owner
                    && previous.params.as_ref().is_some_and(|old|
                        old.pickup_after.is_some() == params.pickup_after.is_some()),
                MotionTrackingAction::Stop => !valid || previous.owner == owner,
            };
            accepted.store(allow, Ordering::Relaxed);
            let mut next = (**previous).clone();
            if allow {
                match params.action {
                    MotionTrackingAction::Start | MotionTrackingAction::Update => {
                        let pitch_changed = params.action == MotionTrackingAction::Update
                            && params.initial_pitch.is_some() && params.initial_pitch != previous.initial_pitch;
                        let same_frame = params.action == MotionTrackingAction::Update
                            && !pitch_changed
                            && params.target.is_some_and(|target| previous.params.as_ref()
                                .and_then(|old| old.target)
                                .is_some_and(|old| old.captured_at_unix_us == target.captured_at_unix_us));
                        next = CoordinatedTrackingIntent {
                            owner,
                            initial_pitch: if params.action == MotionTrackingAction::Start { initial_pitch }
                                else { params.initial_pitch.or(previous.initial_pitch) },
                            at_us: if same_frame { previous.at_us } else { now }, epoch,
                            sequence: if params.action == MotionTrackingAction::Start || pitch_changed {
                                previous.sequence.wrapping_add(1)
                            } else { previous.sequence },
                            params: Some(params.clone()),
                            pickup_after_active: params.action == MotionTrackingAction::Update
                                && previous.pickup_after_active,
                        };
                    }
                    MotionTrackingAction::Stop => next.params = None,
                }
            }
            Arc::new(next)
        });
        let accepted = accepted.load(Ordering::Relaxed);
        if accepted && params.action == MotionTrackingAction::Start { self.clear_head_position_handoff(owner); }
        accepted
    }

    pub fn advance_pickup(&self, sequence: u64, stop_generation: u64) {
        let epoch = self.coordinated_tracking_epoch.load(Ordering::Acquire);
        self.coordinated_tracking.rcu(|previous| {
            if self.stop_generation.load(Ordering::Acquire) != stop_generation
                || previous.epoch != epoch
                || self.coordinated_tracking_epoch.load(Ordering::Acquire) != epoch
                || previous.sequence != sequence || previous.pickup_after_active
                || self.now_us().saturating_sub(previous.at_us) > MOTION_TRACKING_TIMEOUT_US
                || previous.params.as_ref().is_none_or(|params|
                    params.pickup_after.is_none() || params.camera.is_none())
            {
                return Arc::clone(previous);
            }
            let mut next = (**previous).clone();
            next.pickup_after_active = true;
            next.sequence = sequence.wrapping_add(1);
            Arc::new(next)
        });
    }

    pub fn maze_active(&self) -> bool {
        let maze = self.maze.load();
        maze.params.is_some() && self.now_us().saturating_sub(maze.at_us) <= AUTONOMOUS_SESSION_TIMEOUT_US
            && maze.epoch == self.maze_epoch.load(Ordering::Acquire)
    }

    pub fn cancel_maze(&self) {
        self.maze_epoch.fetch_add(1, Ordering::AcqRel);
    }

    pub fn update_maze(&self, owner: u64, params: duck_ipc_proto::MazeParams) -> bool {
        use duck_ipc_proto::MotionTrackingAction;
        if !params.valid() { return false; }
        let now = self.now_us();
        let epoch = self.maze_epoch.load(Ordering::Acquire);
        let accepted = AtomicBool::new(false);
        self.maze.rcu(|previous| {
            let valid = previous.params.is_some()
                && now.saturating_sub(previous.at_us) <= AUTONOMOUS_SESSION_TIMEOUT_US
                && previous.epoch == epoch;
            let mut next = **previous;
            let allow = match params.action {
                MotionTrackingAction::Start => !valid || previous.owner == owner,
                MotionTrackingAction::Update => valid && previous.owner == owner,
                MotionTrackingAction::Stop => !valid || previous.owner == owner,
            };
            accepted.store(allow, Ordering::Relaxed);
            if allow {
                match params.action {
                    MotionTrackingAction::Start | MotionTrackingAction::Update => {
                        next = MazeIntent { owner, at_us: now, epoch,
                            sequence: if params.action == MotionTrackingAction::Start { previous.sequence.wrapping_add(1) } else { previous.sequence },
                            params: Some(params) };
                    }
                    MotionTrackingAction::Stop => next.params = None,
                }
            }
            Arc::new(next)
        });
        accepted.load(Ordering::Relaxed)
    }

    pub fn update_motion_tracking(&self, owner: u64, params: duck_ipc_proto::MotionTrackingParams) -> bool {
        use duck_ipc_proto::MotionTrackingAction;
        if !params.valid() {
            return false;
        }
        let now = self.now_us();
        let epoch = self.motion_tracking_epoch.load(Ordering::Acquire);
        let initial_pitch = params.initial_pitch.or_else(|| self.head_position_handoff(owner, now));
        let accepted = AtomicBool::new(false);
        self.motion_tracking.rcu(|previous| {
            // A stale sample stops movement in snapshot(). The same owner can
            // supply a fresh sample as long as the tracking session still exists.
            let valid = previous.active && previous.epoch == epoch;
            let mut next = **previous;
            let allow = match params.action {
                MotionTrackingAction::Start => !valid || previous.owner == owner,
                MotionTrackingAction::Update => valid && previous.owner == owner,
                MotionTrackingAction::Stop => !valid || previous.owner == owner,
            };
            accepted.store(allow, Ordering::Relaxed);
            if allow {
                match params.action {
                    MotionTrackingAction::Start | MotionTrackingAction::Update => {
                        next = MotionTrackingIntent { owner, active: true, at_us: now, epoch,
                            initial_pitch: if params.action == MotionTrackingAction::Start { initial_pitch }
                                else { params.initial_pitch.or(previous.initial_pitch) },
                            sequence: if params.action == MotionTrackingAction::Start { previous.sequence.wrapping_add(1) } else { previous.sequence },
                            target: params.target, line_speed: params.line_speed, line_target_x: params.line_target_x,
                            reference_area: params.reference_area, deadzone: params.deadzone, gains: params.gains };
                    }
                    MotionTrackingAction::Stop => next.active = false,
                }
            }
            Arc::new(next)
        });
        let accepted = accepted.load(Ordering::Relaxed);
        if accepted && params.action == MotionTrackingAction::Start { self.clear_head_position_handoff(owner); }
        accepted
    }

    /// Clear the movement request while letting an existing scripted action finish.
    pub fn stop_for_maintenance(&self) {
        self.set_twist([0.0; 3]);
        self.accessory_trigger.store(None);
        self.request_precision(duck_ipc_proto::PrecisionMoveParams::Cancel);
    }

    /// Zero the velocity now. Distinct from the deadman only in that it is deliberate.
    pub fn stop(&self) {
        self.accessory_trigger.store(None);
        let at_us = self.now_us();
        self.twist.rcu(|previous| {
            let mut next = **previous;
            next.value = [0.0; 3];
            next.at_us = at_us;
            next.gamepad = false;
            next.pulse.cancel();
            next.rearmed_stop_generation = None;
            Arc::new(next)
        });
        self.stop_generation.fetch_add(1, Ordering::Release);
        if let Some(pending) = self.precision.swap(None) {
            pending.finish();
        }
        if let Some(pending) = self.take_manual_skill() {
            let _ = pending.reply.send(duck_ipc_proto::IntentResult::refused("the skill request was stopped"));
        }
    }

    pub fn stop_generation(&self) -> u64 {
        self.stop_generation.load(Ordering::Acquire)
    }

    pub fn abandon_head_return(&self) {
        self.abandon_head_generation.store(self.head_generation.load(Ordering::Acquire), Ordering::Relaxed);
        self.abandon_generation.fetch_add(1, Ordering::Release);
    }

    /// Submit an edge to the control loop, paired with the stop generation that authorized it.
    pub fn request_precision(&self, params: duck_ipc_proto::PrecisionMoveParams) {
        let generation = self.stop_generation.load(Ordering::Acquire);
        let previous = self.last_precision.load_full();
        // The marker survives both consumption and replacement of the Cancel edge.
        if matches!(params, duck_ipc_proto::PrecisionMoveParams::Cancel)
            && let Some(previous) = previous.as_ref()
            && previous.stop_generation == generation
        {
            previous.cancel_requested.store(true, Ordering::Release);
        }
        let request = Arc::new(PrecisionRequest {
            params,
            stop_generation: generation,
            interrupt_precision: matches!(params, duck_ipc_proto::PrecisionMoveParams::Start { .. })
                && previous.as_ref().is_some_and(|previous| {
                    previous.stop_generation == generation
                        && previous.cancel_requested.load(Ordering::Acquire)
                }),
            cancel_requested: AtomicBool::new(false),
            finished: AtomicBool::new(false),
        });
        if matches!(params, duck_ipc_proto::PrecisionMoveParams::Start { .. }) {
            self.last_precision.store(Some(Arc::clone(&request)));
        }
        if let Some(previous) = self.precision.swap(Some(request)) {
            previous.finish();
        }
    }

    pub fn take_precision(&self) -> Option<Arc<PrecisionRequest>> {
        self.precision.swap(None)
    }

    pub fn precision_active(&self) -> bool {
        self.precision_running.load(Ordering::Acquire)
            || self.last_precision.load().as_ref()
                .is_some_and(|request| !request.finished.load(Ordering::Acquire))
    }

    pub fn precision_cancel_requested(&self) -> bool {
        let generation = self.stop_generation.load(Ordering::Acquire);
        self.last_precision.load().as_ref().is_some_and(|request| {
            request.stop_generation == generation
                && request.cancel_requested.load(Ordering::Acquire)
        })
    }

    pub fn set_precision_running(&self, running: bool) {
        self.precision_running.store(running, Ordering::Release);
    }

    pub fn set_pose(&self, pose: PoseIntent) {
        self.pose.store(Arc::new(pose));
    }

    pub fn set_mouth(&self, mouth: duck_ipc_proto::MouthParams) {
        self.mouth.store(Arc::new(Stamped { value: mouth, at_us: self.now_us() }));
    }

    pub fn mouth_sound_repeating(&self) -> bool {
        let mouth = self.mouth.load();
        mouth.value.repeat_sound
            && mouth.value.open >= duck_ipc_proto::mouth_gesture::PRESSURE_START
            && Duration::from_micros(self.now_us().saturating_sub(mouth.at_us)) < MOUTH_REPEAT_FRESH
    }

    pub fn set_accessory_trigger(&self, pressure: u8) {
        self.accessory_trigger.store(Some(Arc::new(Stamped { value: pressure, at_us: self.now_us() })));
    }

    pub fn accessory_trigger(&self) -> u8 {
        let trigger = self.accessory_trigger.load();
        trigger.as_ref().filter(|sample| self.now_us().saturating_sub(sample.at_us) <= 300_000)
            .map_or(0, |sample| sample.value)
    }

    /// Queue the ground pick, which is not one of the configurable one-shots.
    pub fn request_ground_pick(&self) {
        self.skills
            .fetch_or(SKILL_GROUND_PICK, std::sync::atomic::Ordering::Relaxed);
    }

    /// Queue the sit toggle, likewise.
    pub fn request_sit_toggle(&self) {
        self.skills
            .fetch_or(SKILL_SIT_TOGGLE, std::sync::atomic::Ordering::Relaxed);
    }

    /// Queue a configurable one-shot by its index in the resolved skill list.
    pub fn request_skill(&self, index: usize) {
        if index >= MAX_SKILLS {
            return;
        }
        self.skills.fetch_or(
            1 << (index + SKILL_BITS),
            std::sync::atomic::Ordering::Relaxed,
        );
    }

    /// Take the pending skill requests, leaving none. Once per tick, like the power request.
    pub fn take_skills(&self) -> SkillRequests {
        let bits = self.skills.swap(0, std::sync::atomic::Ordering::Relaxed);
        SkillRequests {
            ground_pick: bits & SKILL_GROUND_PICK != 0,
            sit_toggle: bits & SKILL_SIT_TOGGLE != 0,
            skills: bits >> SKILL_BITS,
        }
    }

    pub(crate) fn request_manual_skill(&self, request: ManualSkillRequest) -> Result<(), &'static str> {
        let mut pending = self.manual_skill.lock().expect("manual skill mutex");
        if pending.as_ref().is_some_and(|pending| !pending.reply.is_closed()) {
            return Err("the skill request is already pending");
        }
        *pending = Some(request);
        Ok(())
    }

    pub(crate) fn take_manual_skill(&self) -> Option<ManualSkillRequest> {
        self.manual_skill.lock().expect("manual skill mutex").take()
    }

    /// Queue a one-shot or refresh the current held sound.
    pub fn request_sound(&self, params: duck_ipc_proto::SoundParams) {
        use duck_ipc_proto::SoundTag;
        if params.tag == SoundTag::Mouth && params.hold.is_some() {
            // Legacy hold refreshes are not opening events.
            return;
        }
        if params.tag == SoundTag::Wheee {
            let target = SoundHoldTarget::Wheee;
            if params.hold.unwrap_or(true) {
                self.sound_hold.store(Arc::new(Stamped {
                    value: target,
                    at_us: self.now_us(),
                }));
            } else if self.sound_hold.load().value == target {
                self.sound_hold.rcu(|previous| {
                    if previous.value == target {
                        Arc::new(Stamped { value: SoundHoldTarget::Released, at_us: self.now_us() })
                    } else {
                        Arc::clone(previous)
                    }
                });
            }
            return;
        }
        let bit = 1u32 << (params.tag as u32);
        self.sounds
            .fetch_or(bit, std::sync::atomic::Ordering::Relaxed);
    }

    /// Queue a clip or update its held loop. Releases only affect the named clip.
    pub fn request_sound_clip(&self, params: &duck_ipc_proto::SoundClipParams) {
        match params.hold {
            None => self.sound_clip.store(Some(Arc::new(params.name.clone()))),
            Some(true) => self.sound_hold.store(Arc::new(Stamped {
                value: SoundHoldTarget::Clip(Arc::from(params.name.as_str())),
                at_us: self.now_us(),
            })),
            Some(false) => {
                self.sound_hold.rcu(|previous| {
                    if matches!(&previous.value, SoundHoldTarget::Clip(name) if name.as_ref() == params.name.as_str()) {
                        Arc::new(Stamped { value: SoundHoldTarget::Released, at_us: self.now_us() })
                    } else {
                        Arc::clone(previous)
                    }
                });
            }
        }
    }

    pub fn take_sound_clip(&self) -> Option<Arc<String>> {
        self.sound_clip.swap(None)
    }

    /// Release whichever loop was owned by a disconnected human controller.
    pub fn clear_sound_hold(&self) {
        self.sound_hold.store(Arc::new(Stamped {
            value: SoundHoldTarget::Released,
            at_us: self.now_us(),
        }));
    }

    /// Take the pending one-shot sounds, leaving none. Once per tick.
    pub fn take_sounds(&self) -> Vec<duck_ipc_proto::SoundTag> {
        use duck_ipc_proto::SoundTag;
        let bits = self.sounds.swap(0, std::sync::atomic::Ordering::Relaxed);
        [
            SoundTag::Alarm,
            SoundTag::Greet,
            SoundTag::Inquire,
            SoundTag::Peck,
            SoundTag::Chirp,
            SoundTag::Coo,
            SoundTag::Mouth,
        ]
        .into_iter()
        .filter(|tag| bits & (1u32 << (*tag as u32)) != 0)
        .collect()
    }

    /// Take one consistent snapshot of the active loop and its expiry.
    pub fn sound_hold(&self) -> SoundHold {
        let stamp = self.sound_hold.load();
        let fresh = Duration::from_micros(self.now_us().saturating_sub(stamp.at_us))
            < WHEEE_HOLD_FRESH;
        match &stamp.value {
            SoundHoldTarget::Wheee if fresh => SoundHold::Wheee(WheeeHold::Held),
            SoundHoldTarget::Wheee => SoundHold::Wheee(WheeeHold::Decayed),
            SoundHoldTarget::Clip(name) if fresh => SoundHold::Clip(Arc::clone(name)),
            SoundHoldTarget::Clip(_) | SoundHoldTarget::Released => {
                SoundHold::Wheee(WheeeHold::Released)
            }
        }
    }

    /// Ask for a drive-mode switch, by the code the caller and the loop agree on.
    ///
    /// The code rather than [`Mode`] itself, so this module does not have to know what modes
    /// exist — `main.rs` owns that mapping and the loop reads it back.
    pub fn request_mode_switch(&self, code: u8) {
        self.mode_switch.store(code, Ordering::Relaxed);
    }

    /// Take a pending mode switch. Taken, so the sequence runs once per request.
    pub fn take_mode_switch(&self) -> Option<u8> {
        match self.mode_switch.swap(MODE_NONE, Ordering::Relaxed) {
            MODE_NONE => None,
            code => Some(code),
        }
    }

    /// Ask for a policy change: one slot to a file, one slot back to its default, or all of
    /// them back to theirs.
    ///
    /// The caller has already validated the file — see `duck_control::policy::validate` — so
    /// reaching here means the load is expected to succeed. It can still fail at the home pose
    /// seconds later, which is why the loop keeps the controller it has until the new one is
    /// built.
    pub fn request_policy_change(&self, change: PolicyChange) {
        self.policy_change.store(Some(Arc::new(change)));
    }

    /// Take a pending policy change. Taken, so the sequence runs once per request.
    pub fn take_policy_change(&self) -> Option<PolicyChange> {
        self.policy_change
            .swap(None)
            .map(|change| PolicyChange::clone(&change))
    }

    /// Ask for the sit-then-power-off sequence.
    pub fn request_shutdown(&self) {
        self.shutdown.store(true, Ordering::Relaxed);
    }

    /// A confirmed global request survives control-lease changes and disconnects.
    pub fn request_confirmed_shutdown(&self) {
        self.confirmed_shutdown.store(true, Ordering::Release);
    }

    pub fn cancel_pending_shutdown(&self) {
        self.shutdown.store(false, Ordering::Relaxed);
    }

    /// Take a pending shutdown request. Taken rather than read so the sequence starts once.
    pub fn take_shutdown(&self) -> bool {
        let regular = self.shutdown.swap(false, Ordering::Relaxed);
        let confirmed = self.confirmed_shutdown.swap(false, Ordering::AcqRel);
        regular || confirmed
    }

    pub fn request_rest(&self) -> bool {
        let mut previous = self.rest_phase.load(Ordering::Acquire);
        loop {
            if previous == REST_RESTING {
                return false;
            }
            match self.rest_phase.compare_exchange(previous, REST_RESTING, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => break,
                Err(current) => previous = current,
            }
        }
        self.rest.store(true, Ordering::Relaxed);
        true
    }

    pub fn take_rest(&self) -> bool {
        self.rest.swap(false, Ordering::Relaxed)
    }

    pub fn rest_phase(&self) -> duck_ipc_proto::RestPhase {
        match self.rest_phase.load(Ordering::Acquire) {
            REST_IDLE => duck_ipc_proto::RestPhase::Idle,
            REST_RESTING => duck_ipc_proto::RestPhase::Resting,
            REST_RESTED => duck_ipc_proto::RestPhase::Rested,
            _ => duck_ipc_proto::RestPhase::Failed,
        }
    }

    pub fn finish_rest(&self, success: bool) {
        let next = if success { REST_RESTED } else { REST_FAILED };
        let _ = self.rest_phase.compare_exchange(REST_RESTING, next, Ordering::AcqRel, Ordering::Acquire);
    }

    pub fn cancel_pending_rest(&self) {
        if self.take_rest() {
            self.finish_rest(false);
        }
    }

    pub fn resume_after_rest(&self) {
        let _ = self.rest_phase.compare_exchange(REST_RESTED, REST_IDLE, Ordering::AcqRel, Ordering::Acquire);
    }

    pub fn set_enabled(&self, on: bool) {
        self.enabled.store(on, Ordering::Relaxed);
    }

    /// The current enable state — what `robot.enable`'s `toggle` flips. The loop reads its
    /// copy through [`Self::snapshot`]; this is for the IPC side, which owns the toggle.
    pub fn enabled(&self) -> bool {
        self.enabled.load(Ordering::Relaxed)
    }

    /// Ask the loop to power the joints and stand up.
    pub fn request_init(&self) {
        self.power.store(POWER_INIT, Ordering::Relaxed);
    }

    /// Ask the loop to cut power to the joints.
    ///
    /// Also clears `enabled`: a robot that has been asked to go limp is not one the policy should
    /// Ask the loop to reboot these servos (empty = all). Clears `enabled` for the same reason
    /// [`Self::request_relax`] does: the robot comes back up when someone asks, not mid-reboot.
    pub fn request_reboot_motors(&self, ids: Vec<u8>) {
        self.enabled.store(false, Ordering::Relaxed);
        if let Ok(mut slot) = self.reboot_motors.lock() {
            *slot = Some(ids);
        }
    }

    /// Take the pending reboot request, if any. Never blocks: a contended lock reads as "nothing
    /// yet" and the request is picked up on the next tick.
    pub fn take_reboot_motors(&self) -> Option<Vec<u8>> {
        self.reboot_motors
            .try_lock()
            .ok()
            .and_then(|mut slot| slot.take())
    }

    /// keep driving, and leaving that flag set would have the next tick bring it straight back up.
    pub fn request_relax(&self) {
        self.enabled.store(false, Ordering::Relaxed);
        self.power.store(POWER_RELAX, Ordering::Relaxed);
    }

    /// Take the pending request, leaving none.
    ///
    /// Called once per tick by the loop. A later request replaces an unread earlier one, which is
    /// the right resolution: if someone asked to stand up and then to relax within 20 ms, the
    /// second is what they meant.
    /// Ask for the chorale to start listening (`true`) or stop (`false`), optionally pinning
    /// which piece this duck picks when it conducts.
    pub fn request_chorale(&self, active: bool, piece: Option<u8>) {
        // The pin first, so the loop that consumes the request on its next tick sees it. Piece
        // ids start at 1, so zero is "no pin".
        self.chorale_piece
            .store(piece.unwrap_or(0), Ordering::Relaxed);
        self.chorale.store(
            if active { THEREMIN_UP } else { THEREMIN_DOWN },
            Ordering::Relaxed,
        );
    }

    /// The pending chorale request, cleared by the read.
    pub fn take_chorale_request(&self) -> Option<(bool, Option<u8>)> {
        let piece = match self.chorale_piece.load(Ordering::Relaxed) {
            0 => None,
            id => Some(id),
        };
        match self.chorale.swap(THEREMIN_NONE, Ordering::Relaxed) {
            THEREMIN_UP => Some((true, piece)),
            THEREMIN_DOWN => Some((false, None)),
            _ => None,
        }
    }

    /// A beacon `btd` heard. Queued rather than latched: two ducks' beacons in one tick are two
    /// observations, and a beat lost to last-writer-wins is a beat the phase lock never sees.
    pub fn heard_chorale(&self, heard: duck_ipc_proto::ChoraleHeard) {
        let mut queue = self.chorale_heard.lock().expect("not poisoned");
        // Bounded: the control loop drains this every tick, so a backlog means the loop is not
        // running — in which case the newest beacons are the only ones worth having.
        if queue.len() >= 64 {
            queue.remove(0);
        }
        queue.push(heard);
    }

    /// Everything heard since the last tick.
    pub fn take_chorale_heard(&self) -> Vec<duck_ipc_proto::ChoraleHeard> {
        std::mem::take(&mut *self.chorale_heard.lock().expect("not poisoned"))
    }

    /// Ask for the theremin to be picked up (`true`) or put down (`false`).
    pub fn request_theremin(&self, active: bool) {
        self.theremin.store(
            if active { THEREMIN_UP } else { THEREMIN_DOWN },
            Ordering::Relaxed,
        );
    }

    /// The pending theremin request, cleared by the read.
    pub fn take_theremin_request(&self) -> Option<bool> {
        match self.theremin.swap(THEREMIN_NONE, Ordering::Relaxed) {
            THEREMIN_UP => Some(true),
            THEREMIN_DOWN => Some(false),
            _ => None,
        }
    }

    pub fn take_power_request(&self) -> Option<PowerRequest> {
        match self.power.swap(POWER_NONE, Ordering::Relaxed) {
            POWER_INIT => Some(PowerRequest::Init),
            POWER_RELAX => Some(PowerRequest::Relax),
            _ => None,
        }
    }

    pub fn snapshot(&self) -> Snapshot {
        let now = self.now_us();
        let twist = self.twist.load();
        let head = self.head.load();
        let head_tracking = self.head_tracking.load();
        let tracking = self.motion_tracking.load();
        let coordinated = self.coordinated_tracking.load();
        let maze = self.maze.load();
        let pose = **self.pose.load();
        Snapshot {
            command: Command {
                twist: if twist.gamepad {
                    twist.pulse.target(Duration::from_micros(now))
                } else {
                    twist.value
                },
                head: [head.target.neck_pitch, head.target.head_pitch, head.target.head_yaw, head.target.head_roll],
                // The loop owns the smoothing that turns the pose intent into this block;
                // the raw target travels in `pose` below. Nominal zero is the trained
                // encoding, not a placeholder.
                body: BodyPose::default(),
            },
            twist_age: Duration::from_micros(now.saturating_sub(twist.at_us)),
            gamepad_twist: twist.gamepad,
            gamepad_rearmed_stop_generation: twist.rearmed_stop_generation,
            stop_generation: self.stop_generation.load(Ordering::Acquire),
            abandon_generation: self.abandon_generation.load(Ordering::Acquire),
            abandon_head_generation: self.abandon_head_generation.load(Ordering::Relaxed),
            head_age: Duration::from_micros(now.saturating_sub(head.at_us)),
            head_position_raw: head.target.raw_position,
            head_position_held: head.hold,
            head_generation: self.head_generation.load(Ordering::Acquire),
            head_tracking: (head_tracking.active && now.saturating_sub(head_tracking.at_us) <= MOTION_TRACKING_TIMEOUT_US
                && head_tracking.epoch == self.head_tracking_epoch.load(Ordering::Acquire))
                .then_some(HeadTrackingSnapshot { target: head_tracking.target, pitch_position: head_tracking.pitch_position, deadzone: head_tracking.deadzone, gains: head_tracking.gains, sequence: head_tracking.sequence }),
            head_position_handoff: (!head_tracking.active
                && head_tracking.epoch == self.head_tracking_epoch.load(Ordering::Acquire)
                && now.saturating_sub(head_tracking.at_us) <= HEAD_HANDOFF_TIMEOUT_US)
                .then_some(head_tracking.pitch_position).flatten(),
            motion_tracking: (tracking.active && now.saturating_sub(tracking.at_us) <= MOTION_TRACKING_TIMEOUT_US
                && tracking.epoch == self.motion_tracking_epoch.load(Ordering::Acquire))
                .then_some(MotionTrackingSnapshot { initial_pitch: tracking.initial_pitch, target: tracking.target,
                    line_speed: tracking.line_speed, line_target_x: tracking.line_target_x, reference_area: tracking.reference_area,
                    deadzone: tracking.deadzone, gains: tracking.gains, sequence: tracking.sequence }),
            coordinated_tracking: (coordinated.epoch == self.coordinated_tracking_epoch.load(Ordering::Acquire))
                .then(|| coordinated.params.clone()).flatten()
                .map(|mut params| {
                    use duck_ipc_proto::PickupStage;
                    let mut initial_pitch = coordinated.initial_pitch;
                    let pickup_stage = if coordinated.pickup_after_active {
                        if let Some(after) = params.pickup_after.take() {
                            let mut after = *after;
                            after.camera = params.camera;
                            after.target = params.target;
                            initial_pitch = after.initial_pitch;
                            params = after;
                        }
                        Some(PickupStage::After)
                    } else if params.pickup_after.is_some() {
                        Some(PickupStage::Before)
                    } else {
                        None
                    };
                    CoordinatedTrackingSnapshot {
                        initial_pitch, params, pickup_stage, sequence: coordinated.sequence,
                        fresh: now.saturating_sub(coordinated.at_us) <= MOTION_TRACKING_TIMEOUT_US,
                        age_us: now.saturating_sub(coordinated.at_us),
                    }
                }),
            maze: (now.saturating_sub(maze.at_us) <= MOTION_TRACKING_TIMEOUT_US
                && maze.epoch == self.maze_epoch.load(Ordering::Acquire))
                .then_some(maze.params).flatten().map(|params| MazeSnapshot { params, sequence: maze.sequence }),
            enabled: self.enabled.load(Ordering::Relaxed),
            pose,
            mouth: self.mouth.load().value.open,
        }
    }
}

#[cfg(test)]
mod autonomous_timeout_tests {
    use super::*;
    use duck_ipc_proto::{MazeParams, MotionTrackingAction as Action, MotionTrackingParams};

    #[test]
    fn line_resumes_on_a_new_sample_after_the_previous_sample_expires() {
        let mut intents = Intents::new();
        intents.epoch = Instant::now() - Duration::from_secs(2);
        let mut params = MotionTrackingParams {
            action: Action::Start, initial_pitch: None, target: Some([0.2, 1.0]),
            line_speed: Some(0.15), line_target_x: None, reference_area: 0.01, deadzone: [0.0; 2],
            gains: [[0.0; 3]; 2],
        };
        assert!(intents.update_motion_tracking(1, params));
        let mut old = **intents.motion_tracking.load();
        old.at_us -= MOTION_TRACKING_TIMEOUT_US + 1;
        intents.motion_tracking.store(Arc::new(old));
        assert!(intents.snapshot().motion_tracking.is_none());
        assert!(intents.motion_tracking_active());

        params.action = Action::Update;
        params.initial_pitch = Some(1900);
        assert!(!intents.update_motion_tracking(2, params));
        assert!(intents.update_motion_tracking(1, params));
        let tracking = intents.snapshot().motion_tracking.unwrap();
        assert_eq!(tracking.target, params.target);
        assert_eq!(tracking.initial_pitch, Some(1900));
        assert_eq!(tracking.sequence, old.sequence);
        params.initial_pitch = None;
        assert!(intents.update_motion_tracking(1, params));
        assert_eq!(intents.snapshot().motion_tracking.unwrap().initial_pitch, Some(1900));
        intents.cancel_motion_tracking();
        assert!(!intents.update_motion_tracking(1, params));
    }

    #[test]
    fn maze_pauses_motion_before_it_expires_the_session() {
        let mut intents = Intents::new();
        intents.epoch = Instant::now() - Duration::from_secs(2);
        let mut params = MazeParams {
            action: Action::Start, threshold_mm: 400, head_step_deg: 10.0,
            settle_ms: 1000, sample_ms: 200, turn_initial_ms: 200,
            turn_final_ms: 200, restore_ms: 200, linear_speed: 0.15, angular_speed: 1.0,
        };
        assert!(intents.update_maze(1, params));
        let mut old = **intents.maze.load();
        old.at_us -= MOTION_TRACKING_TIMEOUT_US + 50_000;
        intents.maze.store(Arc::new(old));
        assert!(intents.snapshot().maze.is_none());
        assert!(intents.maze_active());

        params.action = Action::Update;
        assert!(!intents.update_maze(2, params));
        assert!(intents.update_maze(1, params));
        assert!(intents.snapshot().maze.is_some());

        let mut expired = **intents.maze.load();
        expired.at_us -= AUTONOMOUS_SESSION_TIMEOUT_US + 1;
        intents.maze.store(Arc::new(expired));
        assert!(!intents.maze_active());
        assert!(!intents.update_maze(1, params));
    }
}

#[cfg(test)]
mod manual_skill_tests {
    use super::{Intents, ManualSkillRequest};
    use duck_ipc_proto::IntentResult;
    use std::time::{Duration, Instant};
    use tokio::sync::oneshot;

    fn request(name: &str) -> (ManualSkillRequest, oneshot::Receiver<IntentResult>) {
        let (reply, response) = oneshot::channel();
        let request = ManualSkillRequest {
            name: name.to_owned(),
            deadline: Instant::now() + Duration::from_millis(350),
            reply,
        };
        (request, response)
    }

    #[test]
    fn repeated_skill_is_refused_instead_of_queued() {
        let intents = Intents::new();
        let (first, mut response) = request("kick_left");
        let (duplicate, _) = request("kick_left");
        assert!(intents.request_manual_skill(first).is_ok());
        assert_eq!(intents.request_manual_skill(duplicate),
            Err("the skill request is already pending"));
        assert!(matches!(response.try_recv(), Err(oneshot::error::TryRecvError::Empty)));

        let pending = intents.take_manual_skill().expect("first request");
        assert_eq!(pending.name, "kick_left");
        pending.reply.send(IntentResult::accepted()).expect("waiting client");
        assert!(response.try_recv().expect("control loop response").accepted);
        assert!(intents.take_manual_skill().is_none());
    }

    #[test]
    fn stop_refuses_the_pending_skill_before_it_can_start() {
        let intents = Intents::new();
        let (pending, mut response) = request("kick_right");
        assert!(intents.request_manual_skill(pending).is_ok());
        intents.stop();

        let result = response.try_recv().expect("stop response");
        assert!(!result.accepted);
        assert_eq!(result.reason.as_deref(), Some("the skill request was stopped"));
        assert!(intents.take_manual_skill().is_none());
    }
}

#[cfg(test)]
mod pickup_tests {
    use super::Intents;
    use duck_ipc_proto::{CoordinatedTrackingParams, MotionTrackingAction, PickupStage, TrackingCamera};

    #[test]
    fn pickup_switch_preserves_live_vision_and_rejects_an_expired_handoff() {
        let config = CoordinatedTrackingParams {
            action: MotionTrackingAction::Start, initial_pitch: Some(1400), target: None, camera: None,
            head_deadzone: [0.05; 2], head_gains: [[1.0, 0.0, 0.0]; 2], head_target_point: None, body_gains: [1.0, 0.0, 0.0],
            final_body_gains: [1.0, 0.0, 0.0], approach_linear_limit: None, final_linear_limit: 0.1,
            final_lateral_limit: 0.1, final_point: [160.0, 100.0], final_tolerance: [10.0; 2],
            stop_area_px: None, stop_area_deadzone_px: 0, stop_distance_mm: None, stop_distance_deadzone_mm: 0,
            arrival_delay_s: 0.0, arrival_skill: String::new(), mouth_before: Default::default(),
            mouth_after: Default::default(), pickup_after: None,
        };
        let mut first = config.clone();
        first.camera = Some(TrackingCamera {
            width: 1280, height: 720, rotate: 0, fx: 900.0, fy: 900.0, cx: 640.0, cy: 360.0,
            distortion: [0.0; 5],
        });
        first.initial_pitch = Some(2100);
        first.arrival_skill = "ground_pick".into();
        first.pickup_after = Some(Box::new(config));
        let intents = Intents::new();
        assert!(intents.update_coordinated_tracking(1, first.clone()));
        let first_snapshot = intents.snapshot();
        let sequence = first_snapshot.coordinated_tracking.unwrap().sequence;
        intents.stop();
        intents.advance_pickup(sequence, first_snapshot.stop_generation);
        assert_eq!(intents.snapshot().coordinated_tracking.unwrap().pickup_stage, Some(PickupStage::Before));

        let generation = intents.snapshot().stop_generation;
        intents.advance_pickup(sequence, generation);
        let second = intents.snapshot().coordinated_tracking.unwrap();
        assert_eq!(second.pickup_stage, Some(PickupStage::After));
        assert_eq!(second.initial_pitch, Some(1400));
        assert_eq!(second.params.camera, first.camera);
        intents.advance_pickup(second.sequence, generation);
        assert_eq!(intents.snapshot().coordinated_tracking.unwrap().sequence, second.sequence);
        first.action = MotionTrackingAction::Update;
        first.initial_pitch = None;
        first.pickup_after = None;
        assert!(!intents.update_coordinated_tracking(1, first));
        assert_eq!(intents.snapshot().coordinated_tracking.unwrap().pickup_stage, Some(PickupStage::After));
    }
}
