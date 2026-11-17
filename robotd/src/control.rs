//! Turning sensors and a command into joint targets — and scheduling the skills.
//!
//! Everything here is pure computation between [`duck_control::io::RobotIo::read`] and the
//! safety layer's `apply`. It holds no IO handle — by construction it cannot command a
//! motor, only propose targets.
//!
//! The tick, in order:
//!
//! ```text
//! skill windows ← advance / expire (roulade window, kick timer, ground-pick phase, sit↔stand rise)
//! command      ← the caller's smoothed command, re-encoded for the active skill
//! net          ← roulade > kick > ground pick > sit/rise > stand-by-magnitude > walk
//! action       ← ONNX
//! targets      ← home pose + action_scale × action
//! filters      ← optional first-order low-pass on head and legs
//! ```
//!
//! The priority chain and every numeric default come from `microduck_runtime`'s
//! `control_step`, which this replaces. Two of its subtleties are worth naming because they
//! are easy to "fix" by accident:
//!
//!  - **A kick window runs at standing tuning.** The kick's observation carries an all-zero
//!    command, and in the prototype the standing transition fires on exactly that — so a
//!    kick runs at `standing_action_scale` and the softened standing gain. Kept, because
//!    the kicks were tuned against it.
//!  - **The sitstand *rise* also runs at the standing gain** (its command is all-zero),
//!    while the *sit* does not (its posture flag makes the twist magnitude 1). Same
//!    mechanism, same reason.
//!
//! One deliberate divergence: the prototype tracks the standing action scale by
//! saving/restoring `action_scale` on transitions, which can leave a stale value behind
//! after a sit→stand cycle until the next walk. Here scale and gain are recomputed from
//! the active state every tick — same values on every path that matters, no leftovers.

use duck_control::model::NUM_JOINTS;
use duck_control::obs::{ACTION_LEN, OBS_LEN, Command, Observation};
use duck_control::policy::{Net, Policy, PolicyError, PrecisionPolicy};
use duck_ipc_proto::PrecisionAxis;
use std::time::Instant;

use crate::precision_motion::PrecisionMotion;

/// Joint indices the head low-pass covers: neck_pitch, head_pitch, head_yaw, head_roll.
const HEAD_JOINTS: std::ops::Range<usize> = 5..9;

/// How recently a request must have arrived, at the end of a chaining skill's window, for
/// another to start. The prototype chains roulade on "X still held at the window boundary";
/// here the client holds the button by re-sending the request every tick, so "held" is "a
/// request landed within the last few ticks". 150 ms is seven ticks — generous against a
/// dropped packet, far too short to mistake a fresh press for a hold.
const CHAIN_WINDOW: f64 = 0.15;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tuning {
    /// Scales raw policy output before it becomes a joint offset. The prototype's current
    /// alpha default.
    pub action_scale: f64,
    /// The standing policy is trained to be applied whole.
    pub standing_action_scale: f64,
    /// Standing runs softer, at this fraction of the running gain. `--standing-kp-ratio`.
    pub standing_gain_ratio: f64,
    pub gain: u16,
    /// First-order low-pass on the head joints. `None` is no filtering. The alpha policies
    /// are trained with 0.5 — it must match training or transfer degrades.
    pub head_lowpass: Option<f64>,
    /// Same, for the ten leg joints. Trained with 0.7.
    pub legs_lowpass: Option<f64>,
}

impl Default for Tuning {
    fn default() -> Self {
        Self {
            action_scale: 0.9,
            standing_action_scale: 1.0,
            standing_gain_ratio: 0.8,
            gain: 200,
            head_lowpass: Some(0.5),
            legs_lowpass: Some(0.7),
        }
    }
}

/// The scripted-skill numbers, resolved per mode by `params` — from the installed set's
/// manifest where it says, and from the prototype's literals where it does not.
#[derive(Debug, Clone, PartialEq)]
pub struct SkillTuning {
    /// One ground-pick cycle, seconds.
    pub ground_pick_period: f64,
    /// The ground pick hands back at this fraction of its cycle. The current pickup
    /// completes phase 1.0 so its return finishes before handing back.
    pub ground_pick_end_phase: f64,
    pub ground_pick_action_scale: f64,
    /// Gain multiplier while the pick runs.
    pub ground_pick_gain_ratio: f64,
    /// How long the sitstand network rises (posture flag 0) before the main policy takes over.
    /// 1 s is enough on the robot — velstand owns the tail of the rise fine.
    pub sitstand_rise_s: f64,
    /// How long the seat takes to settle after the posture flag flips: the ~2 s glide the
    /// network is trained on. The shutdown sit waits this plus a second before easing into the
    /// rest pose.
    pub sitstand_ramp_s: f64,
    /// The one-shot skills, in priority order — name, duration, whether holding chains, and
    /// what each changes about the robot while it runs. Config, resolved over the built-ins.
    pub skills: Vec<robotd_params::SkillDef>,
}

impl Default for SkillTuning {
    fn default() -> Self {
        Self {
            ground_pick_period: 4.0,
            ground_pick_end_phase: robotd_params::DEFAULT_GROUND_PICK_END_PHASE,
            ground_pick_action_scale: 1.0,
            ground_pick_gain_ratio: 1.0,
            sitstand_rise_s: robotd_params::DEFAULT_SITSTAND_RISE_S,
            sitstand_ramp_s: robotd_params::DEFAULT_SITSTAND_RAMP_S,
            skills: Vec::new(),
        }
    }
}

/// One tick's worth of decisions, for the caller to act on and report.
#[derive(Debug, Clone, PartialEq)]
pub struct Step {
    pub targets: [f64; NUM_JOINTS],
    /// Which network drove, as the wire label: `walk`, `stand`, `ground_pick`, `sit`, `rise`,
    /// or a configured skill's own name.
    ///
    /// Borrowed for the fixed set and owned for a skill, whose name comes from config rather
    /// than from this build — which is the whole point of a skill being config.
    pub label: std::borrow::Cow<'static, str>,
    /// What the gain should be for this tick.
    pub gain: u16,
    /// A scripted or precision move is mid-flight, regardless of the current twist.
    /// Unlike `Controller::busy()`, this includes precision motion.
    pub busy: bool,
}

/// The exact policy tensors assembled during one successful or attempted step.
///
/// This stays out of [`Step`] because motion control does not otherwise need to retain an
/// observation or raw action after deriving its joint targets. The optional caller-owned value
/// lets a recorder copy those tensors without making the normal 50 Hz path allocate.
#[derive(Debug, Clone, Copy)]
pub struct PolicyTrace {
    pub observation: [f32; OBS_LEN],
    pub observation_built: bool,
    /// `None` when inference failed after the observation was assembled.
    pub action: Option<[f32; ACTION_LEN]>,
}

impl Default for PolicyTrace {
    fn default() -> Self {
        Self {
            observation: [0.0; OBS_LEN],
            observation_built: false,
            action: None,
        }
    }
}

/// Where the robot is in the sit↔stand cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Sit {
    Up,
    /// The sitstand network holds the seat (posture flag 1).
    Sitting,
    /// Warm the sitstand network in the seat before an automatic rise.
    Settling { remaining: f64 },
    /// Rise automatically after the seat has settled.
    AutoRising { remaining: f64 },
    /// The sitstand network rises (posture flag 0) for the remaining seconds, then the
    /// main policy takes over.
    Rising {
        remaining: f64,
    },
}

/// A one-shot skill mid-flight.
#[derive(Debug, Clone, Copy)]
struct ActiveSkill {
    /// Index into the resolved skill list, which is also [`Net::Skill`]'s index.
    index: usize,
    /// Whether it is doing the thing or coming back from it.
    phase: SkillPhase,
    /// Seconds left in this phase.
    remaining: f64,
    /// Seconds left during which another request still counts as the button being held.
    /// Roulade's chaining, generalised: counted down every tick, refreshed by each request
    /// that lands while the skill runs, and positive at the end of a window means start
    /// another.
    chain: f64,
}

/// Which half of a skill is running.
///
/// Most skills only ever have the first. The second exists for a policy that does not end
/// itself — one that holds until told otherwise — where handing straight back to walk would
/// give it a robot mid-pose.
#[derive(Debug, Clone, Copy, PartialEq)]
enum SkillPhase {
    /// Driving `command`, for `duration`.
    Holding,
    /// Driving `unwind`, for `unwind_s`, before handing back.
    Unwinding,
}

/// Which network drove the robot on the last step, as a policy change needs to know it.
///
/// [`Net`] with the skill index replaced by the skill's name, because an index is only meaningful
/// against one skill list and a change may be about to install another.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Driving {
    Walk,
    Stand,
    /// The sitstand network, in motion: rising, or the tick the flag flipped.
    SitStand,
    /// The sitstand network holding the seat. Parked, not travelling — the one driving state a
    /// network can be swapped under without the robot noticing, which is why it is its own
    /// variant rather than a flag beside `SitStand`.
    Seated,
    GroundPick,
    /// A one-shot skill, by its config name.
    Skill(String),
}

impl std::fmt::Display for Driving {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Driving::Walk => f.write_str("walk"),
            Driving::Stand => f.write_str("stand"),
            Driving::SitStand => f.write_str("sitstand"),
            Driving::Seated => f.write_str("seated"),
            Driving::GroundPick => f.write_str("ground_pick"),
            Driving::Skill(name) => f.write_str(name),
        }
    }
}

pub struct Controller {
    policy: Policy,
    precision_policy: Option<PrecisionPolicy>,
    precision_motion: Option<PrecisionMotion>,
    /// Model reference used by observations and action decoding.
    home: [f64; NUM_JOINTS],
    /// Physical pose held when no loaded network owns the robot.
    home_target: [f64; NUM_JOINTS],
    tuning: Tuning,
    skills: SkillTuning,
    /// Raw previous policy output, which the observation feeds back. Raw, not scaled: the
    /// policy was trained observing its own output, not the actuator command derived from
    /// it. Shared across every network, as the prototype shares it.
    last_action: [f32; ACTION_LEN],
    /// Previous filtered targets, kept only for the low-pass. `None` until the first tick,
    /// so the filter starts from reality rather than dragging up from zero.
    previous: Option<[f64; NUM_JOINTS]>,
    /// Ground-pick phase, 0..`skills.ground_pick_end_phase`. `None` when inactive.
    ground_pick: Option<f64>,
    /// **A running skill switches the fall reflex off**, because `busy()` is what gates the
    /// limp-fall predictor and any active skill makes it true. That was uncontroversial when
    /// every one-shot was under a second; a skill configured to hold for ten is a robot with no
    /// fall reflex for ten seconds, which wants deciding rather than inheriting.
    ///
    /// The one-shot skill driving right now: which one, and how long it has left.
    ///
    /// One field where there were three, because a kick and a roulade were the same thing
    /// with different numbers.
    active: Option<ActiveSkill>,
    roll_phase: Option<duck_control::roll_phase::RollPhase>,
    roll_heading: duck_control::roll_phase::RollHeadingReference,
    sit: Sit,
    /// The ordinary network on the last step; `None` before the first or during precision.
    last_net: Option<Net>,
    force_walk: bool,
}

impl Controller {
    pub fn managed_models(&self, cfg: &robotd_params::ResolvedPolicy) -> Vec<duck_ipc_proto::ManagedModel> {
        let mut models = Vec::new();
        let mut add = |id: &str, net: Net| {
            if let Some((path, digest)) = self.policy.network_identity(net) {
                let sha256 = digest.iter().map(|byte| format!("{byte:02x}")).collect();
                models.push(duck_ipc_proto::ManagedModel {
                    id: id.to_owned(), path: path.display().to_string(), sha256, active: false,
                });
            }
        };
        add("walk", Net::Walk);
        add("stand", Net::Stand);
        add("sitstand", Net::SitStand);
        add("ground_pick", Net::GroundPick);
        for (index, skill) in cfg.skills.iter().enumerate() {
            add(&skill.name, Net::Skill(index));
        }
        models
    }

    pub fn new(
        policy: Policy,
        tuning: Tuning,
        skills: SkillTuning,
        home: [f64; NUM_JOINTS],
        home_target: [f64; NUM_JOINTS],
    ) -> Self {
        Self {
            policy,
            precision_policy: None,
            precision_motion: None,
            home,
            home_target,
            tuning,
            skills,
            last_action: [0.0; ACTION_LEN],
            previous: None,
            ground_pick: None,
            active: None,
            roll_phase: None,
            roll_heading: Default::default(),
            sit: Sit::Up,
            last_net: None,
            force_walk: false,
        }
    }

    pub fn home(&self) -> [f64; NUM_JOINTS] {
        self.home
    }

    pub fn home_target(&self) -> [f64; NUM_JOINTS] {
        self.home_target
    }

    /// Temporarily select the walking network for tracking or a manual stop handoff.
    pub fn set_force_walk(&mut self, active: bool) {
        self.force_walk = active;
    }

    /// When automatic stand selection is enabled, settle after it actually takes over.
    /// Walk-only and roller bundles must remain usable without automatic stand selection.
    pub fn target_correction_policy_ready(&self) -> bool {
        !self.policy.will_stand(0.0) || self.last_net == Some(Net::Stand)
    }

    /// Install the preloaded policy without loading files in the control loop. On failure
    /// the caller keeps ownership, so a rejected controller swap cannot discard the model.
    pub fn install_precision_policy(
        &mut self,
        policy: PrecisionPolicy,
    ) -> Result<(), (PrecisionPolicy, PolicyError)> {
        if self.precision_policy.is_some() {
            return Err((policy, PolicyError::Inference("precision policy already installed".into())));
        }
        if let Err(error) = check_precision_home(&self.home, &policy.home()) {
            return Err((policy, error));
        }
        self.precision_policy = Some(policy);
        Ok(())
    }

    /// Detach the policy for storage while no controller exists, ending any active motion.
    pub fn take_precision_policy(&mut self) -> Option<PrecisionPolicy> {
        self.cancel_precision();
        self.precision_policy.take()
    }

    /// Move the preloaded policy and any active motion to a replacement controller atomically.
    /// Call before `carry_over`; a failed HOME check leaves the source untouched.
    pub fn transfer_precision_from(&mut self, from: &mut Controller) -> Result<(), PolicyError> {
        self.check_precision_transfer_from(from)?;
        if from.precision_policy.is_none() {
            return Ok(());
        }
        self.precision_policy = from.precision_policy.take();
        self.precision_motion = from.precision_motion.take();
        if self.precision_motion.is_some() {
            self.policy.reset();
            self.last_action = from.last_action;
            self.previous = from.previous;
            self.last_net = None;
        }
        Ok(())
    }

    pub fn check_precision_transfer_from(&self, from: &Controller) -> Result<(), PolicyError> {
        let Some(policy) = from.precision_policy.as_ref() else { return Ok(()); };
        if self.precision_policy.is_some() {
            return Err(PolicyError::Inference("precision policy already installed".into()));
        }
        check_precision_home(&self.home, &policy.home())
    }

    /// Begin a displacement using the loaded policy and the caller's monotonic clock.
    pub fn start_precision(
        &mut self,
        axis: PrecisionAxis,
        value: f64,
        now: Instant,
    ) -> Result<(), &'static str> {
        if self.precision_motion.is_some() {
            return Err("precision motion already running");
        }
        if self.active.is_some() || self.ground_pick.is_some() || self.sit != Sit::Up {
            return Err("another scripted motion or seat posture is active");
        }
        let motion = PrecisionMotion::new(now, axis, value)?;
        let policy = self.precision_policy.as_mut().ok_or("no precision policy loaded")?;
        policy.reset();
        // The regular network has stopped driving. Its recurrent state must not survive
        // the excursion, but the physical last action and filter anchor must survive.
        self.policy.reset();
        self.precision_motion = Some(motion);
        Ok(())
    }

    /// Stop the sequence without changing the last action supplied to the next policy.
    pub fn cancel_precision(&mut self) {
        self.precision_motion = None;
    }

    /// Separate from `busy()`, which the main loop uses to cancel gamepad velocity.
    pub fn precision_active(&self) -> bool {
        self.precision_motion.is_some()
    }

    /// The ordinary network that drove on the last step, or `None` during precision.
    pub fn driving(&self) -> Option<Driving> {
        Some(match self.last_net? {
            Net::Walk => Driving::Walk,
            Net::Stand => Driving::Stand,
            Net::SitStand if self.is_sitting() => Driving::Seated,
            Net::SitStand => Driving::SitStand,
            Net::GroundPick => Driving::GroundPick,
            Net::Skill(index) => Driving::Skill(
                self.skills
                    .skills
                    .get(index)
                    .map_or_else(|| index.to_string(), |d| d.name.clone()),
            ),
        })
    }

    /// Pick up where `from` left off.
    ///
    /// For a controller built to replace one whose driving network it has *not* changed — a new
    /// `walk` under a robot that is sitting, a new `stand` under one that is walking. The seat,
    /// the skill mid-flight, the ground-pick phase, the last action the observation feeds back
    /// and the low-pass anchor all carry across, so the swap is invisible: the next tick runs the
    /// same network from the same state, and the replaced one is simply there when it is next
    /// selected. A fresh controller in its place would start `Sit::Up`, which under a seated
    /// robot is a stand-up nobody asked for.
    ///
    /// The caller has checked that the network driving is unchanged, and that the skill list is
    /// unchanged if a skill is what is driving — `active` addresses that list by index. The one
    /// exception is a seated robot, which carries across whatever changed: the seat is a static
    /// pose on a constant flag, and any sitstand network asked to hold it holds it.
    pub fn carry_over(&mut self, from: &Controller) {
        self.policy.carry_over(&from.policy);
        self.last_action = from.last_action;
        self.previous = from.previous;
        self.ground_pick = from.ground_pick;
        self.active = from.active;
        self.roll_phase = from.roll_phase;
        self.roll_heading = from.roll_heading;
        self.sit = from.sit;
        self.last_net = from.last_net;
    }

    /// Keep the policy's existing head filter anchored to externally driven joints.
    pub fn anchor_head_targets(&mut self, targets: [f64; 2]) {
        if let Some(previous) = self.previous.as_mut() {
            previous[6] = targets[0];
            previous[7] = targets[1];
        }
    }

    /// Reset the feedback state.
    ///
    /// Called when the policy is re-enabled, so a robot that sat disabled for a minute does
    /// not resume with a stale action in its observation and a filter anchored to wherever
    /// it was before. Recurrent networks also discard their episode memory. Cancel the
    /// one-shot selection with that memory, or Start replays an interrupted skill from
    /// its initial state without a new request. The latched seat posture stays unchanged.
    pub fn reset(&mut self) {
        self.policy.reset();
        self.roll_heading.clear();
        self.cancel_scripted_actions();
        self.last_action = [0.0; ACTION_LEN];
        self.previous = None;
    }

    /// A rest ended limp; discard the seat and scripted move before the next initialization.
    pub fn forget(&mut self) {
        self.reset();
        self.ground_pick = None;
        self.active = None;
        self.sit = Sit::Up;
    }

    /// An explicit stop ends a rise and cancels automatic settling while retaining a seat.
    pub fn stop_moves(&mut self) {
        self.reset();
        self.sit = match self.sit {
            Sit::Settling { .. } => Sit::Sitting,
            Sit::Rising { .. } | Sit::AutoRising { .. } => Sit::Up,
            other => other,
        };
    }

    /// Restart a known seated policy from measured joints, settle, then rise.
    pub fn settle_then_rise(&mut self, pose: &[f64; NUM_JOINTS]) {
        let offsets = std::array::from_fn(|joint| pose[joint] - self.home[joint]);
        self.last_action = Observation::gather_action(&offsets);
        self.previous = Some(*pose);
        self.sit = Sit::Settling { remaining: self.skills.sitstand_ramp_s };
    }

    /// Cancel transient skills while preserving the currently latched seat posture.
    pub fn cancel_scripted_actions(&mut self) {
        self.cancel_precision();
        self.ground_pick = None;
        self.active = None;
        self.roll_phase = None;
    }

    pub fn cancel_unwritten_sit(&mut self) {
        if self.sit == Sit::Sitting { self.sit = Sit::Up; }
    }

    pub fn has_sitstand(&self) -> bool {
        self.policy.has_sitstand()
    }

    /// How long the policy holds the shutdown sit before the joints ease into the rest pose:
    /// the seat's settle time, then a second sitting still.
    pub fn shutdown_sit_secs(&self) -> f64 {
        self.skills.sitstand_ramp_s + 1.0
    }

    pub fn is_sitting(&self) -> bool {
        matches!(self.sit, Sit::Sitting | Sit::Settling { .. })
    }

    /// Whether the unmodified command would select the walking network.
    pub fn walks_for(&self, command: &Command) -> bool {
        self.policy.has_walking()
            && !self.precision_active()
            && self.sit == Sit::Up
            && self.ground_pick.is_none()
            && self.active.is_none()
            && (self.force_walk || !self.policy.will_stand(command.twist_magnitude()))
    }

    /// A scripted move is mid-flight. Sitting itself is not busy — a seated robot is
    /// parked, not travelling.
    pub fn busy(&self) -> bool {
        self.ground_pick.is_some()
            || (self.active.is_some()
                && !self.roll_phase.is_some_and(|phase| phase.completed && !phase.holds_inverted()))
            || matches!(self.sit, Sit::Settling { .. } | Sit::Rising { .. } | Sit::AutoRising { .. })
    }

    /// A scripted policy may continue holding its final phase after `busy()` clears.
    pub fn scripted_active(&self) -> bool {
        self.ground_pick.is_some() || self.active.is_some()
            || matches!(self.sit, Sit::Settling { .. } | Sit::Rising { .. } | Sit::AutoRising { .. })
    }

    pub fn release_completed_roll(&mut self) {
        if self.roll_phase.is_some_and(|phase| phase.completed && !phase.holds_inverted()) {
            self.release_roll_hold();
        }
    }

    /// Start a one-shot ground pick. The prototype gates the trigger on nothing but the
    /// network existing and the move not already running — a pick can even preempt a kick's
    /// tail, and that stays as it was.
    pub fn start_ground_pick(&mut self) -> Result<(), &'static str> {
        if matches!(self.sit, Sit::Settling { .. } | Sit::AutoRising { .. }) {
            return Err("automatic seated recovery is running");
        }
        if !self.policy.has_ground_pick() {
            return Err("no ground-pick policy loaded");
        }
        if self.ground_pick.is_some() {
            return Err("ground pick already running");
        }
        if self.roll_phase.is_some_and(|phase| !phase.completed) {
            return Err("a roll is in progress");
        }
        self.cancel_precision();
        self.release_roll_hold();
        self.ground_pick = Some(0.0);
        Ok(())
    }

    /// Every one-shot skill this robot has, in priority order — what a client may ask for.
    pub fn skill_names(&self) -> Vec<String> {
        self.skills
            .skills
            .iter()
            .take(self.policy.skill_count())
            .map(|skill| skill.name.clone())
            .collect()
    }

    /// Start a one-shot skill, or — for a chaining one already running — keep the chain alive.
    ///
    /// `Ok(true)` started it; `Ok(false)` refreshed a running one, and the caller should stay
    /// quiet about that, because a held button lands here fifty times a second.
    ///
    /// The gating is the prototype's, generalised. A scripted move blocks another, except that
    /// a chaining skill may preempt one that is not chaining — which is how the X press could
    /// always roll out of a kick's tail or out of the seat. A ground pick blocks everything, as
    /// it always has.
    pub fn start_skill(&mut self, index: usize) -> Result<bool, &'static str> {
        if matches!(self.sit, Sit::Settling { .. } | Sit::AutoRising { .. }) {
            return Err("automatic seated recovery is running");
        }
        let Some(def) = self.skills.skills.get(index) else {
            return Err("no such skill on this robot");
        };
        let (duration, chains) = (def.duration, def.chain);
        let roll_phase = self.policy.skill_phase(index);
        if def.name == "headstand" && !roll_phase.is_some_and(|phase| phase.holds_inverted()) {
            return Err("headstand requires a headstand_phase policy");
        }
        if roll_phase.is_some_and(|phase| phase.holds_inverted()) && self.sit != Sit::Up {
            return Err("stand up before starting a headstand");
        }

        if let Some(active) = self.active.as_mut()
            && let Some(phase) = self.roll_phase
        {
            if active.index == index && (!phase.completed || active.chain > 0.0) {
                active.chain = CHAIN_WINDOW;
                return Ok(false);
            }
            if !phase.completed {
                return Err("a roll is in progress");
            }
        }
        self.release_roll_hold();

        if let Some(active) = &mut self.active {
            if active.index == index && chains {
                active.chain = CHAIN_WINDOW;
                return Ok(false);
            }
            if !chains {
                return Err("a scripted move is already running");
            }
        }
        if self.ground_pick.is_some() {
            return Err("a ground pick is running");
        }
        self.cancel_precision();
        self.active = Some(ActiveSkill {
            index,
            phase: SkillPhase::Holding,
            remaining: duration,
            chain: if roll_phase.is_some() { CHAIN_WINDOW } else { 0.0 },
        });
        self.roll_phase = roll_phase;
        self.roll_heading.clear();
        if roll_phase.is_some() || self.policy.skill_completion_controlled(index) {
            self.policy.reset();
            if roll_phase.is_some_and(|phase| !phase.holds_inverted()) {
                self.last_action.fill(0.0);
            }
        }
        if roll_phase.is_some_and(|phase| phase.holds_inverted()) {
            // Same settled-pose action history as the Python headstand trigger.
            let settled = [
                -0.019880, -0.084759, -0.389621, -0.041503, 0.356091,
                0.285178, 0.428532, -0.008422, 0.001141,
                0.031818, 0.087306, 0.393430, 0.010875, -0.382988,
            ];
            for (index, target) in settled.into_iter().enumerate() {
                let joint = if index < duck_control::model::MOUTH_INDEX {
                    index
                } else {
                    index + 1
                };
                self.last_action[index] = (target - self.home[joint]) as f32;
            }
        }
        Ok(true)
    }

    /// Sit if standing, stand if sitting. Refused mid-rise, as the prototype refuses it
    /// while a stand transition is in flight.
    pub fn sit_toggle(&mut self) -> Result<&'static str, &'static str> {
        if self.roll_phase.is_some_and(|phase| !phase.completed) {
            return Err("a roll is in progress");
        }
        self.release_roll_hold();
        match self.sit {
            Sit::Up => {
                if !self.policy.has_sitstand() {
                    return Err("no sitstand policy loaded");
                }
                self.cancel_precision();
                self.sit = Sit::Sitting;
                Ok("sit")
            }
            Sit::Sitting => {
                self.cancel_precision();
                self.sit = Sit::Rising {
                    remaining: self.skills.sitstand_rise_s,
                };
                Ok("stand")
            }
            Sit::Settling { .. } => Err("still settling in the seat"),
            Sit::Rising { .. } | Sit::AutoRising { .. } => Err("already standing up"),
        }
    }

    /// Engage the sit for the shutdown sequence. The caller owns the timing (sit for a few
    /// seconds, then cut torque and power off); this just puts the sitstand network in
    /// charge with the posture flag at 1.
    pub fn begin_shutdown_sit(&mut self) {
        self.cancel_precision();
        if self.roll_phase.take().is_some() {
            self.active = None;
        }
        self.sit = Sit::Sitting;
    }

    /// One tick, optionally preserving the exact ONNX input and output.
    ///
    /// `body_active` says a client is holding the body-pose mode: the twist is zeroed and
    /// the standing network drives (by magnitude where it is selectable, forced where it is
    /// reserved), exactly as the prototype's B-button mode behaves. `scale_mult` multiplies
    /// the action scale — voltage adaptation, 1.0 when off.
    pub fn step_with_trace(
        &mut self,
        sensors: &duck_control::Sensors,
        command: &Command,
        body_active: bool,
        dt: f64,
        scale_mult: f64,
        trace: Option<&mut PolicyTrace>,
    ) -> Result<Step, PolicyError> {
        self.step_with_observation_gravity(sensors, command, body_active, dt, scale_mult, None, trace)
    }

    /// Override only observation gravity, preserving orientation used by skill phases.
    #[allow(clippy::too_many_arguments)]
    pub fn step_with_observation_gravity(
        &mut self,
        sensors: &duck_control::Sensors,
        command: &Command,
        body_active: bool,
        dt: f64,
        scale_mult: f64,
        observation_gravity: Option<[f64; 3]>,
        trace: Option<&mut PolicyTrace>,
    ) -> Result<Step, PolicyError> {
        self.step_with_selection(
            sensors, command, command.twist, body_active, dt, scale_mult, observation_gravity, trace,
        )
    }

    /// Select the base network using the operator's command before auxiliary yaw is added.
    #[allow(clippy::too_many_arguments)]
    pub fn step_with_selection(
        &mut self,
        sensors: &duck_control::Sensors,
        command: &Command,
        selection_twist: [f64; 3],
        body_active: bool,
        dt: f64,
        scale_mult: f64,
        observation_gravity: Option<[f64; 3]>,
        trace: Option<&mut PolicyTrace>,
    ) -> Result<Step, PolicyError> {
        self.step_with_selection_at(
            sensors, command, selection_twist, body_active, dt, scale_mult,
            observation_gravity, trace, Instant::now(),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn step_with_selection_at(
        &mut self,
        sensors: &duck_control::Sensors,
        command: &Command,
        selection_twist: [f64; 3],
        body_active: bool,
        dt: f64,
        scale_mult: f64,
        observation_gravity: Option<[f64; 3]>,
        mut trace: Option<&mut PolicyTrace>,
        now: Instant,
    ) -> Result<Step, PolicyError> {
        if let Some(motion) = self.precision_motion {
            if let Some(frame) = motion.frame(now) {
                return self.step_precision(sensors, frame, observation_gravity, trace);
            }
            self.cancel_precision();
        }
        if body_active || command.twist_magnitude() > 0.0 {
            self.release_roll_hold();
        }
        if let Some(active) = self.active
            && self.policy.skill_completion_controlled(active.index)
            && self.policy.skill_completed(active.index)
        {
            self.active = None;
        }
        // Expire windows first, so a tick after the deadline runs the next thing rather
        // than one more frame of a finished move — the prototype checks its timers at the
        // same point relative to inference.
        // The end of a window is a fork, as the prototype forks a roll: the button still held
        // — a request landed within the chain window — restarts it, and released hands back.
        // Only a chaining skill can take the first branch, so a kick still ends when it ends.
        if self.roll_phase.is_none()
            && let Some(active) = self.active
            && active.remaining <= 0.0
            && !self.policy.skill_completion_controlled(active.index)
        {
            let def = self.skills.skills.get(active.index);
            self.active = match active.phase {
                // A skill that does not end itself comes back first, so walk never inherits a
                // robot mid-pose. `unwind_s` of zero — the common case — skips straight past.
                SkillPhase::Holding if def.is_some_and(|d| d.unwind_s > 0.0) => Some(ActiveSkill {
                    phase: SkillPhase::Unwinding,
                    remaining: def.map_or(0.0, |d| d.unwind_s),
                    ..active
                }),
                // The end of a window is a fork, as the prototype forks a roll: the button still
                // held — a request landed within the chain window — restarts it, and released
                // hands back. Only a chaining skill can take that branch, so a kick still ends
                // when it ends, and a skill that unwound is finished either way.
                SkillPhase::Holding | SkillPhase::Unwinding => {
                    let chains = active.phase == SkillPhase::Holding
                        && def.is_some_and(|d| d.chain)
                        && active.chain > 0.0;
                    if chains {
                        // A chained replay is a new episode even though Net is unchanged.
                        self.policy.reset();
                    }
                    chains.then(|| ActiveSkill {
                        phase: SkillPhase::Holding,
                        remaining: def.map_or(0.0, |d| d.duration),
                        chain: 0.0,
                        ..active
                    })
                }
            };
        }
        if let Sit::Rising { remaining } | Sit::AutoRising { remaining } = self.sit
            && remaining <= 0.0
        {
            self.sit = Sit::Up;
        }
        if let Sit::Settling { remaining } = self.sit
            && remaining <= 0.0
        {
            self.sit = Sit::AutoRising { remaining: self.skills.sitstand_rise_s };
        }

        // Re-encode the command for the active skill and pick the network. The priority chain
        // is the prototype's, with its three one-shots now one entry: skill > ground pick >
        // sit/rise > stand-by-magnitude > walk, and the skills themselves ordered by config.
        let (net, effective, label) = if let Some(active) = self.active {
            // Head and body are zeroed whatever the phase — every one-shot published so far
            // declares them unused, and a policy trained with `zero_command_padding` expects
            // exactly that. Only the twist differs, and for most skills it is zero too, which is
            // what made the kick and roulade arms the same arm.
            let def = self.skills.skills.get(active.index);
            let twist = if self.policy.skill_zero_command(active.index) {
                [0.0; 3]
            } else if let Some(phase) = self.roll_phase.as_mut() {
                phase.update(sensors.imu.gravity, sensors.imu.quat)
                    .map_err(|error| PolicyError::Inference(error.into()))?
            } else {
                def.map_or([0.0; 3], |definition| match active.phase {
                    SkillPhase::Holding => definition.command,
                    SkillPhase::Unwinding => definition.unwind,
                })
            };
            let label = def.map_or(std::borrow::Cow::Borrowed("skill"), |d| {
                std::borrow::Cow::Owned(match active.phase {
                    SkillPhase::Holding => d.name.clone(),
                    SkillPhase::Unwinding => format!("{}:unwind", d.name),
                })
            });
            let c = Command {
                twist,
                ..Command::default()
            };
            (Net::Skill(active.index), c, label)
        } else if let Some(phase) = self.ground_pick {
            // The twist slots carry the phase encoding; head and body are zero-padded,
            // mirroring the training env's `zero_command_padding`.
            let angle = std::f64::consts::TAU * phase;
            let c = Command {
                twist: [angle.cos(), angle.sin(), 0.0],
                ..Command::default()
            };
            (Net::GroundPick, c, "ground_pick".into())
        } else {
            // A posture-only bundle has no walking network. Ignore velocity, head and body
            // intents in that mode so a connected gamepad can select only standing or sit/rise.
            let mut c = if self.policy.has_walking() {
                *command
            } else {
                Command::default()
            };
            match self.sit {
                // The posture flag rides the twist vx slot: 1 = sit, 0 = stand. Head and
                // body slots stay live — the prototype keeps them in the buffer too.
                Sit::Sitting | Sit::Settling { .. } => {
                    c.twist = [1.0, 0.0, 0.0];
                    (Net::SitStand, c, "sit".into())
                }
                Sit::Rising { .. } | Sit::AutoRising { .. } => {
                    c.twist = [0.0; 3];
                    (Net::SitStand, c, "rise".into())
                }
                Sit::Up => {
                    if body_active {
                        c.twist = [0.0; 3];
                    }
                    let selection = Command {
                        twist: if body_active { [0.0; 3] } else { selection_twist },
                        ..Command::default()
                    };
                    let standing = (self.policy.will_stand(selection.twist_magnitude()) && !self.force_walk)
                        || (body_active && self.policy.has_standing());
                    let Some(net) = idle_net(
                        self.policy.has_walking(),
                        self.policy.has_standing(),
                        self.policy.has_sitstand(),
                        standing,
                    ) else {
                        // Nothing loaded can hold an upright robot — a scripted move on its
                        // own (roulade, a kick) is a legitimate set, and the honest answer
                        // between moves is the home pose, held here rather than asked of a
                        // network that is not there.
                        self.previous = Some(self.home_target);
                        return Ok(Step {
                            targets: self.home_target,
                            gain: self.tuning.gain,
                            label: "held".into(),
                            busy: false,
                        });
                    };
                    let label = match net {
                        Net::Walk => "walk",
                        Net::Stand | Net::SitStand => "stand",
                        _ => unreachable!("idle policy selection only returns a base network"),
                    };
                    (net, c, label.into())
                }
            }
        };

        self.last_net = Some(net);

        let mut observation_imu = sensors.imu;
        if let Some(gravity) = observation_gravity { observation_imu.gravity = gravity; }
        let mut observation = Observation::build(
            &observation_imu,
            &sensors.positions,
            &sensors.velocities,
            &self.home,
            &self.last_action,
            &effective,
        );
        if let Net::Skill(index) = net
            && self.policy.skill_relative_heading(index)
        {
            let heading = self.roll_heading.update(sensors.imu.quat)
                .map_err(|error| PolicyError::Inference(error.into()))?;
            observation.set_roll_heading(heading)
                .map_err(|error| PolicyError::Inference(error.into()))?;
        }

        if let Some(trace) = trace.as_deref_mut() {
            trace.observation.copy_from_slice(observation.as_slice());
            trace.observation_built = true;
            trace.action = None;
        }

        let action = self.policy.infer(&observation, net)?;
        if let Net::Skill(index) = net
            && self.policy.skill_completion_controlled(index)
            && self.policy.skill_completed(index)
        {
            // Completion belongs to this feedback sample. Reselect before publishing the
            // Kick action, changing history/filter anchors, or advancing any window: the
            // replacement policy alone owns this tick and advances its clocks once.
            self.active = None;
            return self.step_with_selection_at(
                sensors,
                command,
                selection_twist,
                body_active,
                dt,
                scale_mult,
                observation_gravity,
                trace,
                now,
            );
        }
        if let Some(trace) = trace.as_deref_mut() {
            trace.action = Some(action);
        }
        self.last_action = action;

        // Scale and gain follow the active state, recomputed every tick. "Standing tuning"
        // applies whenever the *effective* command is inside the standing threshold and the
        // standing network exists — which is how a kick window and the sitstand rise end up
        // at standing gain in the prototype, so they do here too.
        let standing_tuned = matches!(net, Net::Stand)
            || (matches!(net, Net::Skill(_) | Net::SitStand)
                && self.policy.will_stand(if self.roll_phase.is_some() { 0.0 } else { effective.twist_magnitude() }));
        let (scale, gain) = match net {
            // A skill's own overrides, falling back to the gait's — which is how a kick keeps
            // running at the standing tuning it was tuned against while a roulade can ask for
            // something else.
            Net::Skill(index) => {
                let overrides = self.skills.skills.get(index).map(|d| &d.params);
                let scale = overrides
                    .and_then(|o| o.action_scale)
                    .unwrap_or(if standing_tuned {
                        self.tuning.standing_action_scale
                    } else {
                        self.tuning.action_scale
                    });
                let ratio = overrides.and_then(|o| o.gain_ratio).unwrap_or({
                    if standing_tuned {
                        self.tuning.standing_gain_ratio
                    } else {
                        1.0
                    }
                });
                (scale, (self.tuning.gain as f64 * ratio).round() as u16)
            }
            Net::GroundPick => (
                self.skills.ground_pick_action_scale,
                (self.tuning.gain as f64 * self.skills.ground_pick_gain_ratio).round() as u16,
            ),
            Net::SitStand => (
                // The prototype's `start_sit_toggle` pins the scale at 1.0 for the whole
                // sit/rise cycle.
                1.0,
                if standing_tuned {
                    (self.tuning.gain as f64 * self.tuning.standing_gain_ratio).round() as u16
                } else {
                    self.tuning.gain
                },
            ),
            _ if standing_tuned => (
                self.tuning.standing_action_scale,
                (self.tuning.gain as f64 * self.tuning.standing_gain_ratio).round() as u16,
            ),
            _ => (self.tuning.action_scale, self.tuning.gain),
        };
        let scale = scale * scale_mult;

        let offsets = Observation::scatter_action(&action);
        let mut targets = [0.0; NUM_JOINTS];
        for joint in 0..NUM_JOINTS {
            targets[joint] = self.home[joint] + scale * offsets[joint];
        }

        if self.roll_phase.is_none()
            && let Some(previous) = self.previous
        {
            if let Some(alpha) = self.tuning.head_lowpass {
                for joint in HEAD_JOINTS {
                    targets[joint] = alpha * targets[joint] + (1.0 - alpha) * previous[joint];
                }
            }
            if let Some(alpha) = self.tuning.legs_lowpass {
                for (joint, target) in targets.iter_mut().enumerate() {
                    if HEAD_JOINTS.contains(&joint) || joint == duck_control::model::MOUTH_INDEX {
                        continue;
                    }
                    *target = alpha * *target + (1.0 - alpha) * previous[joint];
                }
            }
        }
        self.previous = Some(targets);

        // Advance the windows, after the tick that used them — the prototype advances its
        // phase after the motor write.
        if let Some(phase) = self.ground_pick.as_mut() {
            *phase += dt / self.skills.ground_pick_period;
            if *phase >= self.skills.ground_pick_end_phase {
                self.ground_pick = None;
            }
        }
        if let Some(active) = self.active.as_mut() {
            if self.roll_phase.is_none() {
                active.remaining -= dt;
            }
            active.chain = (active.chain - dt).max(0.0);
        }
        if let Sit::Settling { remaining } | Sit::Rising { remaining } | Sit::AutoRising { remaining } = &mut self.sit {
            *remaining -= dt;
        }

        Ok(Step {
            targets,
            label,
            gain,
            busy: self.busy(),
        })
    }

    fn release_roll_hold(&mut self) {
        if self.roll_phase.is_some_and(|phase| phase.completed) {
            self.active = None;
            self.roll_phase = None;
            self.roll_heading.clear();
        }
    }

    fn step_precision(
        &mut self,
        sensors: &duck_control::Sensors,
        frame: crate::precision_motion::Frame,
        observation_gravity: Option<[f64; 3]>,
        trace: Option<&mut PolicyTrace>,
    ) -> Result<Step, PolicyError> {
        let mut imu = sensors.imu;
        if let Some(gravity) = observation_gravity {
            imu.gravity = gravity;
        }
        let command = Command { twist: frame.command, ..Command::default() };
        let observation = Observation::build(
            &imu, &sensors.positions, &sensors.velocities, &self.home, &self.last_action, &command,
        );
        let mut trace = trace;
        if let Some(trace) = trace.as_deref_mut() {
            trace.observation.copy_from_slice(observation.as_slice());
            trace.observation_built = true;
            trace.action = None;
        }
        let policy = self.precision_policy.as_mut().ok_or_else(|| {
            PolicyError::Inference("active precision motion has no loaded policy".into())
        })?;
        let action = policy.infer(&observation)?;
        if let Some(trace) = trace {
            trace.action = Some(action);
        }
        self.last_action = action;
        self.last_net = None;
        let offsets = Observation::scatter_action(&action);
        let targets = std::array::from_fn(|joint| self.home[joint] + offsets[joint]);
        self.previous = Some(targets);
        Ok(Step { targets, label: "precision_student".into(), gain: self.tuning.gain, busy: true })
    }
}

/// Match the controller HOME to the displacement policy at the same precision used
/// by ordinary policy bundles.
fn check_precision_home(
    home: &[f64; NUM_JOINTS],
    precision_home: &[f64; NUM_JOINTS],
) -> Result<(), PolicyError> {
    for joint in 0..NUM_JOINTS {
        if joint == duck_control::model::MOUTH_INDEX {
            continue;
        }
        if !home[joint].is_finite()
            || !precision_home[joint].is_finite()
            || (home[joint] - precision_home[joint]).abs() > 1e-6
        {
            return Err(PolicyError::JointContract(format!(
                "precision HOME mismatch for {}: controller {}, precision {}",
                duck_control::model::JOINT_NAMES[joint], home[joint], precision_home[joint],
            )));
        }
    }
    Ok(())
}







/// Pick the network that holds an upright robot when no scripted move is active.
///
/// A lone sitstand network is a valid posture-only controller. Its zero posture flag is
/// the standing command; the `Sit::Sitting` branch above changes that flag to one.
///
/// `None` is a loaded set with no base network at all — a roulade or a kick on its own,
/// which is a legitimate way to run one scripted move against a robot somebody else is
/// holding up. The caller holds the home pose then. Asking for `Net::Walk` in that set is
/// what produced "inference failed: walking policy is not loaded" on every tick.
fn idle_net(
    has_walking: bool,
    has_standing: bool,
    has_sitstand: bool,
    standing: bool,
) -> Option<Net> {
    if !has_walking && !has_standing && has_sitstand {
        Some(Net::SitStand)
    } else if standing {
        has_standing
            .then_some(Net::Stand)
            .or(has_walking.then_some(Net::Walk))
    } else {
        has_walking.then_some(Net::Walk)
    }
}
