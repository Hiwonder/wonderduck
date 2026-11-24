//! The ONNX policies.
//!
//! Walking and standing are chosen by the magnitude of the velocity command, exactly as
//! `microduck_runtime` does; the skill networks — sit↔stand, ground pick, the two kicks —
//! are selected explicitly by the scheduler in `robotd`, which owns the priority rules.
//! Every network shares the one 61-D observation layout, so a skill is a session choice
//! plus a command-block encoding. LSTM exports additionally pass hidden/cell state,
//! owned by each network; sensor observations and joint actions are unchanged.
//!
//! **Everything is validated at load, not at inference.** A bundle with the wrong
//! observation width, the wrong action count, or a missing ONNX Runtime must fail while the
//! robot is standing still and the caller can be told why — not sixty ticks later, mid
//! stride. `robotd` turns a load failure into "hold the pose and report unhealthy", so the
//! updater rolls the release back instead of leaving a robot that cannot walk.

use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use ort::session::Session;
use ort::session::builder::GraphOptimizationLevel;
use ort::tensor::TensorElementType;
use ort::value::{Tensor, ValueType};
use sha2::{Digest, Sha256};

use crate::obs::{ACTION_LEN, COMMAND_LEN, OBS_LEN, Observation};

mod precision;
pub use precision::PrecisionPolicy;

/// Below this velocity magnitude the standing policy takes over. The prototype's value.
pub const DEFAULT_STANDING_THRESHOLD: f64 = 0.05;

/// Inference threads per session.
///
/// One, deliberately. The prototype uses two, which on a four-core A55 means the control
/// thread blocks on a pool it does not own — and for a network this small the pool costs
/// more in synchronisation than it recovers in parallelism. Worth re-measuring on the board
/// rather than trusting either number.
const INTRA_THREADS: usize = 1;

#[derive(Debug, thiserror::Error)]
pub enum PolicyError {
    #[error("reading {path}: {source}")]
    Read {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("loading {path}: {source}")]
    Load {
        path: PathBuf,
        #[source]
        source: ort::Error,
    },
    /// The bundle does not match what this build implements. Reported with both shapes
    /// because "wrong policy file" and "wrong daemon" look identical without them.
    #[error("{path}: {what} is {got}, expected {expected}")]
    Shape {
        path: PathBuf,
        what: &'static str,
        expected: String,
        got: String,
    },
    #[error("inference failed: {0}")]
    Inference(String),
    /// ONNX Runtime is not installed, or not where it is being looked for.
    ///
    /// Its own diagnosis, because it is an operator problem with an operator fix — install
    /// the library or set `ORT_DYLIB_PATH` — and not a broken policy bundle.
    #[error("ONNX Runtime not loadable ({searched}): {detail}")]
    RuntimeMissing { searched: String, detail: String },
    /// `ort` panicked instead of returning an error. See [`catching_ort_panics`].
    ///
    /// `detail` is the panic message, and carrying it is the point: the one panic we have
    /// actually seen on a board names the two version numbers that explain it.
    #[error("ort panicked loading the policy: {detail}")]
    RuntimePanic { detail: String },
    #[error("policy joint contract: {0}")]
    JointContract(String),
}

impl PolicyError {
    /// The file this error is about, when it is about one.
    ///
    /// `Read`, `Load` and `Shape` name a file; a missing runtime or an `ort` panic does not, and
    /// blaming whichever policy happened to be loading when the dylib turned out to be absent
    /// would send an operator to replace a file that is fine.
    pub fn path(&self) -> Option<&Path> {
        match self {
            PolicyError::Read { path, .. }
            | PolicyError::Load { path, .. }
            | PolicyError::Shape { path, .. } => Some(path),
            PolicyError::Inference(_)
            | PolicyError::JointContract(_)
            | PolicyError::RuntimeMissing { .. }
            | PolicyError::RuntimePanic { .. } => None,
        }
    }
}

/// Where `ort` will look for the runtime, replicating its own logic.
fn dylib_name() -> String {
    match std::env::var("ORT_DYLIB_PATH") {
        Ok(path) if !path.is_empty() => path,
        _ => {
            if cfg!(target_os = "windows") {
                "onnxruntime.dll".to_owned()
            } else if cfg!(any(target_os = "macos", target_os = "ios")) {
                "libonnxruntime.dylib".to_owned()
            } else {
                "libonnxruntime.so".to_owned()
            }
        }
    }
}

/// Confirm ONNX Runtime is loadable **before** calling into `ort`.
///
/// This exists because `ort` does not return an error when the dylib is missing — it
/// `expect`s inside `setup_api`, from a lazy path reachable through any API call, so a
/// missing library aborts the thread that touched it. In the control loop that means the
/// thread dies, no tick ever lands, and `robot.health` reports "the loop has not completed a
/// cycle" forever: the daemon looks wedged instead of saying ONNX Runtime is not installed.
///
/// Probing first turns the *missing library* case into an ordinary error the caller can
/// report, with the operator's fix in it. That is all it does.
///
/// It does **not** mean `ort` cannot then panic, and an earlier version of this comment
/// claimed it did. A board running ONNX Runtime 1.20.1 falsified that: the library loaded, so
/// the probe passed, and `ort` panicked in `setup_api` on its own version check
/// (`expected version >= '1.23.x', but got '1.20.1'`). The probe proves the file loads;
/// nothing more. [`catching_ort_panics`] covers the rest, including panics we have not seen.
fn ensure_runtime() -> Result<(), PolicyError> {
    static PROBE: OnceLock<Result<(), String>> = OnceLock::new();
    let outcome = PROBE.get_or_init(|| {
        let name = dylib_name();
        // Safety: loading a shared library runs its initialisers. This is the same library
        // `ort` is about to load itself, so the risk is not one this probe introduces.
        match unsafe { libloading::Library::new(&name) } {
            Ok(library) => {
                // Leak it: `ort` will dlopen the same file moments later and the OS
                // reference-counts the mapping. Dropping ours would be harmless but
                // pointless churn.
                std::mem::forget(library);
                Ok(())
            }
            Err(e) => Err(e.to_string()),
        }
    });

    outcome
        .clone()
        .map_err(|detail| PolicyError::RuntimeMissing {
            searched: dylib_name(),
            detail,
        })
}

/// Run the `ort` calls, turning a panic from inside them into a [`PolicyError`].
///
/// `ort` treats some initialisation failures as unrecoverable and panics rather than
/// returning `Err` — the version mismatch in [`ensure_runtime`]'s comment is the one a board
/// hit, and it fires from inside a lazy init reachable through any API call. In the control
/// thread a panic is worse than an error: the thread dies, no tick ever lands, and
/// `robot.health` answers "the loop has not completed a cycle yet" — the one message that
/// names no cause — while the daemon stays up serving its socket. The updater then rolls the
/// release back for a reason nobody can act on.
///
/// `robotd` already handles a policy that fails to load: hold the pose, keep ticking at rate,
/// report why, get rolled back. This makes a panic take that same path.
///
/// Deliberately wraps the `ort` work only, and not all of [`Policy::load`], so a genuine bug
/// of ours does not get relabelled "policy unavailable". Note that a caught panic has still
/// run the panic hook, so the backtrace is in the journal either way.
///
/// `AssertUnwindSafe` is needed because `Session` is not `UnwindSafe`. It is sound here
/// because nothing of ours is observed after a catch: the sessions being built are moved into
/// the `Policy` on success and dropped on failure, and the caller's answer is the error.
///
/// **`panic = "abort"` would defeat this.** The root `Cargo.toml` has no `[profile.release]`,
/// so the default unwind strategy applies; adding one would silently turn this back into a
/// dead control thread.
fn catching_ort_panics<T>(work: impl FnOnce() -> Result<T, PolicyError>) -> Result<T, PolicyError> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(work)).unwrap_or_else(|payload| {
        Err(PolicyError::RuntimePanic {
            detail: panic_message(payload),
        })
    })
}

/// The panic message, or a stand-in saying there wasn't one.
///
/// `panic!` with a literal produces `&'static str`; with arguments, `String`. `ort` uses both.
fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
    if let Some(s) = payload.downcast_ref::<&'static str>() {
        (*s).to_owned()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "panicked with no message; see the journal for the backtrace".to_owned()
    }
}

/// Which network drives a tick.
///
/// The choice is the caller's — the skill scheduler in `robotd` owns the priority rules —
/// and this enum is how it names its choice. Asking for a network that is not loaded falls
/// back to walking rather than panicking, but the scheduler is expected to check `has_*`
/// first; the fallback exists so a race cannot kill the control thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Net {
    Walk,
    Stand,
    /// Commanded sit↔stand: the twist `vx` slot carries a posture flag, 1 = sit, 0 = stand.
    SitStand,
    /// Phase-scripted ground pick; the twist slots carry `[cos φ, sin φ, 0]`.
    GroundPick,
    /// A one-shot skill, by its index in [`PolicyPaths::skills`].
    ///
    /// Kicks and roulade used to be variants here. They were the same thing three times over —
    /// a network trained on an all-zero command, driving for a fixed window, selected by an
    /// explicit request — differing only in duration and tuning, which is data. An index means
    /// a robot gains a skill by gaining a config entry rather than a release.
    Skill(usize),
}

/// Which policy files to load. `walk` may be absent for a posture-only robot; every other slot
/// is a capability the robot simply does not have when `None`.
#[derive(Debug, Clone, Default)]
pub struct PolicyPaths {
    pub walk: Option<PathBuf>,
    pub stand: Option<PathBuf>,
    pub sitstand: Option<PathBuf>,
    pub ground_pick: Option<PathBuf>,
    /// One-shot skills, in the priority order the caller wants them considered. Each is
    /// selected only by an explicit request, so an empty list is a robot with no tricks rather
    /// than a robot missing something.
    pub skills: Vec<PathBuf>,
}

/// The loaded networks.
///
/// A configured path that fails to load fails the whole load — the policies ship inside the
/// release, so a missing or corrupt file is a broken bundle, and the right outcome is
/// "unhealthy, roll it back", not a robot that silently lost its kick.
pub struct Policy {
    walk: Option<Network>,
    stand: Option<Network>,
    sitstand: Option<Network>,
    ground_pick: Option<Network>,
    skills: Vec<Network>,
    standing_threshold: f64,
    active: Option<Net>,
    /// Roller mode and fall-recovery mode reserve the standing network (roller has none;
    /// fall recovery keeps it for getting up), so command magnitude must never select it.
    standing_disabled: bool,
}

fn parse_joint_home(
    names: &str,
    encoded: &str,
) -> Result<[f64; crate::model::NUM_JOINTS], PolicyError> {
    use crate::model::{DEFAULT_POSITION, JOINT_NAMES, MOUTH_INDEX};
    let expected: Vec<&str> = JOINT_NAMES
        .iter()
        .enumerate()
        .filter_map(|(i, name)| (i != MOUTH_INDEX).then_some(*name))
        .collect();
    let actual: Vec<&str> = names.split(',').map(str::trim).collect();
    let values: Vec<f64> = encoded
        .split(',')
        .map(|v| v.trim().parse::<f64>())
        .collect::<Result<_, _>>()
        .map_err(|e| PolicyError::JointContract(format!("invalid HOME: {e}")))?;
    if values.len() != actual.len()
        || values
            .iter()
            .any(|v| !v.is_finite() || v.abs() > std::f64::consts::PI)
    {
        return Err(PolicyError::JointContract(
            "HOME must match joint_names and contain finite angles within +/-pi".into(),
        ));
    }
    let mut seen = std::collections::HashSet::new();
    for name in &actual {
        if !seen.insert(*name) {
            return Err(PolicyError::JointContract(format!(
                "duplicate joint {name}"
            )));
        }
        if !expected.contains(name)
            && !expected
                .iter()
                .any(|joint| *name == format!("passive_{joint}_backlash"))
        {
            return Err(PolicyError::JointContract(format!("unknown joint {name}")));
        }
    }
    let controlled: Vec<_> = actual
        .iter()
        .copied()
        .filter(|name| expected.contains(name))
        .collect();
    if controlled != expected {
        return Err(PolicyError::JointContract(format!(
            "controlled joint order {controlled:?}, expected {expected:?}"
        )));
    }
    // Passive simulation hinges are not control channels. Resolve the encoder
    // references by name, retaining the fixed physical action order.
    let values: Vec<f64> = expected
        .iter()
        .map(|name| {
            actual
                .iter()
                .position(|actual| actual == name)
                .map(|index| values[index])
                .ok_or_else(|| PolicyError::JointContract(format!("missing joint {name}")))
        })
        .collect::<Result<_, _>>()?;
    let mut home = DEFAULT_POSITION;
    let mut values = values.into_iter();
    for (i, angle) in home.iter_mut().enumerate() {
        if i != MOUTH_INDEX {
            *angle = values.next().expect("validated action count");
        }
    }
    Ok(home)
}

impl Policy {
    /// Identity of the exact bytes used to construct this network's live session.
    pub fn network_identity(&self, net: Net) -> Option<(&Path, [u8; 32])> {
        let network = match net {
            Net::Walk => self.walk.as_ref(),
            Net::Stand => self.stand.as_ref(),
            Net::SitStand => self.sitstand.as_ref(),
            Net::GroundPick => self.ground_pick.as_ref(),
            Net::Skill(index) => self.skills.get(index),
        }?;
        Some((&network.path, network.digest))
    }

    /// Read the named joint reference used by every network in a custom bundle.
    /// The caller opts in; the standard robot keeps its established calibration.
    pub fn joint_home(&self) -> Result<[f64; crate::model::NUM_JOINTS], PolicyError> {
        let mut home = None;
        let networks = self
            .walk
            .iter()
            .chain(self.stand.iter())
            .chain(self.sitstand.iter())
            .chain(self.ground_pick.iter())
            .chain(self.skills.iter());
        for network in networks {
            let metadata = network
                .session
                .metadata()
                .map_err(|e| PolicyError::JointContract(e.to_string()))?;
            let names = metadata
                .custom("joint_names")
                .ok_or_else(|| PolicyError::JointContract("missing joint_names".into()))?;
            let encoded = metadata
                .custom("default_joint_pos")
                .ok_or_else(|| PolicyError::JointContract("missing default_joint_pos".into()))?;
            let candidate = parse_joint_home(&names, &encoded)?;
            if home.is_some_and(|previous: [f64; crate::model::NUM_JOINTS]| {
                previous
                    .iter()
                    .zip(candidate)
                    .any(|(a, b)| (a - b).abs() > 1e-6)
            }) {
                return Err(PolicyError::JointContract(
                    "loaded networks use different HOME references".into(),
                ));
            }
            home = Some(candidate);
        }
        home.ok_or_else(|| PolicyError::JointContract("no network supplies HOME".into()))
    }

    /// Load, validate and warm up.
    ///
    /// `stand` is optional: without it the walking policy runs at every velocity. Conversely,
    /// a posture-only bundle may omit `walk` and then always selects `stand` outside a sit/rise.
    pub fn load(paths: &PolicyPaths, standing_threshold: f64) -> Result<Self, PolicyError> {
        Self::load_with_candidate(paths, standing_threshold, None)
    }

    /// Build the complete bundle using the exact candidate bytes read by the caller.
    pub fn load_with_candidate(
        paths: &PolicyPaths,
        standing_threshold: f64,
        candidate: Option<(&Path, &[u8])>,
    ) -> Result<Self, PolicyError> {
        ensure_runtime()?;

        // Everything below calls into `ort`, and `ort` panics on failures it considers
        // unrecoverable — so the whole of it, and nothing else, goes inside the catch.
        catching_ort_panics(move || {
            // Warm up before the loop ever calls this. The first inference is always an
            // outlier — lazy initialisation, cold pages, first-touch faults — and paying that
            // on tick one would look exactly like a control loop that missed its deadline.
            // It also proves ONNX Runtime is actually present and usable, which with
            // `load-dynamic` is not known until something is run.
            let zero = Observation::zeroed();
            fn open_warm(path: &Path, zero: &Observation, candidate: Option<(&Path, &[u8])>) -> Result<Network, PolicyError> {
                let mut network = match candidate {
                    Some((candidate_path, bytes)) if candidate_path == path => open_bytes(path, false, bytes)?,
                    _ => open(path)?,
                };
                network.run(zero)?;
                network.reset();
                Ok(network)
            }
            fn open_opt(
                path: &Option<PathBuf>,
                zero: &Observation,
                candidate: Option<(&Path, &[u8])>,
            ) -> Result<Option<Network>, PolicyError> {
                let network = path
                    .as_deref()
                    .map(|path| open_warm(path, zero, candidate))
                    .transpose()?;
                if network
                    .as_ref()
                    .is_some_and(|network| network.roll_phase.is_some() || network.zero_command)
                {
                    return Err(PolicyError::Inference(
                        "phase and zero-padding policies must be loaded as skills".into(),
                    ));
                }
                Ok(network)
            }

            Ok(Self {
                walk: open_opt(&paths.walk, &zero, candidate)?,
                stand: open_opt(&paths.stand, &zero, candidate)?,
                sitstand: open_opt(&paths.sitstand, &zero, candidate)?,
                ground_pick: open_opt(&paths.ground_pick, &zero, candidate)?,
                skills: paths
                    .skills
                    .iter()
                    .map(|path| open_warm(path, &zero, candidate))
                    .collect::<Result<Vec<_>, _>>()?,
                standing_threshold,
                active: None,
                standing_disabled: false,
            })
        })
    }

    /// Reserve the standing network: command magnitude no longer selects it, and only an
    /// explicit [`Net::Stand`] from the caller (fall recovery, body pose) reaches it.
    pub fn set_standing_disabled(&mut self, disabled: bool) {
        self.standing_disabled = disabled;
    }

    /// Whether the standing policy would be chosen for this command.
    ///
    /// Separate from [`Self::infer`] because the caller needs the same answer to decide
    /// gains and action scale, and asking twice must not be able to disagree.
    pub fn will_stand(&self, twist_magnitude: f64) -> bool {
        self.stand.is_some()
            && !self.standing_disabled
            && (self.walk.is_none() || twist_magnitude <= self.standing_threshold)
    }

    pub fn has_walking(&self) -> bool {
        self.walk.is_some()
    }

    pub fn has_standing(&self) -> bool {
        self.stand.is_some()
    }

    pub fn has_sitstand(&self) -> bool {
        self.sitstand.is_some()
    }

    pub fn has_ground_pick(&self) -> bool {
        self.ground_pick.is_some()
    }

    /// How many one-shot skills are loaded. A caller's index is valid below this.
    pub fn skill_count(&self) -> usize {
        self.skills.len()
    }

    pub fn skill_phase(&self, index: usize) -> Option<crate::roll_phase::RollPhase> {
        self.skills
            .get(index)
            .and_then(|network| network.roll_phase)
    }

    pub fn skill_relative_heading(&self, index: usize) -> bool {
        self.skills.get(index).is_some_and(|network| network.relative_heading)
    }

    /// Completion is exported by the motion policy; timers do not end these skills.
    pub fn skill_completion_controlled(&self, index: usize) -> bool {
        self.skills.get(index).is_some_and(|network| network.completion_controlled)
    }

    pub fn skill_completed(&self, index: usize) -> bool {
        self.skills.get(index).is_some_and(|network| network.completed)
    }

    pub fn skill_zero_command(&self, index: usize) -> bool {
        self.skills
            .get(index)
            .is_some_and(|network| network.zero_command)
    }

    /// One inference on the named network. A missing optional network falls back to
    /// walking — the scheduler checks `has_*` before asking, so reaching the fallback is a
    /// bug, but a wrong gait beats a dead control thread.
    pub fn infer(
        &mut self,
        observation: &Observation,
        net: Net,
    ) -> Result<[f32; ACTION_LEN], PolicyError> {
        // Resolve fallback before comparing: asking for an absent skill must not reset
        // the walking network on every tick.
        let net = match net {
            Net::Stand if self.stand.is_none() => Net::Walk,
            Net::SitStand if self.sitstand.is_none() => Net::Walk,
            Net::GroundPick if self.ground_pick.is_none() => Net::Walk,
            Net::Skill(i) if i >= self.skills.len() => Net::Walk,
            net => net,
        };
        let changed = self.active != Some(net);
        let network = match net {
            Net::Walk => self
                .walk
                .as_mut()
                .ok_or_else(|| PolicyError::Inference("walking policy is not loaded".to_owned()))?,
            Net::Stand => self.stand.as_mut().unwrap(),
            Net::SitStand => self.sitstand.as_mut().unwrap(),
            Net::GroundPick => self.ground_pick.as_mut().unwrap(),
            Net::Skill(i) => &mut self.skills[i],
        };
        if changed {
            network.reset();
        }
        let result = network.run(observation);
        // Never carry a failed inference's state into another control tick.
        if result.is_err() {
            network.reset();
            self.active = None;
        } else {
            self.active = Some(net);
        }
        result
    }

    /// Preserve the running network when only another slot changed. Compare model
    /// bytes, not paths: a reload may replace a file in place, and a seated swap may
    /// intentionally replace the active network. Those cases must start fresh.
    pub fn carry_over(&mut self, from: &Self) {
        self.reset();
        let Some(net) = from.active else {
            return;
        };
        let target = match net {
            Net::Walk => self.walk.as_mut(),
            Net::Stand => self.stand.as_mut(),
            Net::SitStand => self.sitstand.as_mut(),
            Net::GroundPick => self.ground_pick.as_mut(),
            Net::Skill(i) => self.skills.get_mut(i),
        };
        let source = match net {
            Net::Walk => from.walk.as_ref(),
            Net::Stand => from.stand.as_ref(),
            Net::SitStand => from.sitstand.as_ref(),
            Net::GroundPick => from.ground_pick.as_ref(),
            Net::Skill(i) => from.skills.get(i),
        };
        if let (Some(target), Some(source)) = (target, source)
            && target.digest == source.digest
        {
            if let (Some(dst), Some(src)) = (&mut target.state, &source.state) {
                dst.h
                    .try_extract_tensor_mut::<f32>()
                    .unwrap()
                    .1
                    .copy_from_slice(src.h.try_extract_tensor::<f32>().unwrap().1);
                dst.c
                    .try_extract_tensor_mut::<f32>()
                    .unwrap()
                    .1
                    .copy_from_slice(src.c.try_extract_tensor::<f32>().unwrap().1);
            }
            self.active = Some(net);
        }
    }

    /// Start a new episode, including when resuming after disable or fall recovery.
    /// A network also starts fresh whenever selection switches away and back to it.
    pub fn reset(&mut self) {
        self.active = None;
        for network in self
            .walk
            .iter_mut()
            .chain(self.stand.iter_mut())
            .chain(self.sitstand.iter_mut())
            .chain(self.ground_pick.iter_mut())
            .chain(self.skills.iter_mut())
        {
            network.reset();
        }
    }
}

/// Can this file be loaded as a policy, without committing to running it?
///
/// Opens the graph and checks both shapes, then throws the session away. Two callers, and they
/// want it for the same reason from opposite ends:
///
///  - `robot.loadPolicy` answers a client *synchronously*, while the real swap happens seconds
///    later at the home pose. Validating here is what turns "accepted" followed by a robot that
///    did not change into an immediate `observation width is 51, expected 61`.
///  - startup checks each overridden slot before building the controller, so one bad override
///    costs that slot rather than the whole policy.
///
/// **No warm-up inference**, unlike [`Policy::load`] — this is a question about a file, not a
/// session about to be driven, and the first-inference cost is the loading path's to pay. It
/// therefore proves less: a graph that opens and has the right shape can still fail to run.
/// Nothing downstream treats a pass as a guarantee, which is why a failed load at the home pose
/// still has to keep the controller it had.
///
/// Not from inside a tick. Opening a session is tens of milliseconds and the loop has 20 to
/// spend — so the IPC caller runs it on its own runtime, and the loop only ever calls it in its
/// preamble, before the first tick is due.
pub fn validate(path: &Path) -> Result<(), PolicyError> {
    ensure_runtime()?;
    catching_ort_panics(|| open(path).map(drop))
}

/// The mjlab/rsl_rl LSTM export passes state explicitly. Buffers belong to one
/// session, are allocated at load, and are never shared between policy slots.
struct Network {
    session: Session,
    /// Reused for each inference; loading allocates the observation tensor once.
    input: Tensor<f32>,
    state: Option<LstmState>,
    action_name: String,
    path: PathBuf,
    digest: [u8; 32],
    roll_phase: Option<crate::roll_phase::RollPhase>,
    relative_heading: bool,
    zero_command: bool,
    completion_controlled: bool,
    completed: bool,
}

fn policy_input(observation: &Observation, zero_command: bool, input: &mut [f32]) {
    input.copy_from_slice(observation.as_slice());
    if zero_command {
        input[OBS_LEN - COMMAND_LEN..].fill(0.0);
    }
}

struct LstmState {
    h: Tensor<f32>,
    c: Tensor<f32>,
}

impl Network {
    fn reset(&mut self) {
        self.completed = false;
        if let Some(state) = &mut self.state {
            state.h.try_extract_tensor_mut::<f32>().unwrap().1.fill(0.0);
            state.c.try_extract_tensor_mut::<f32>().unwrap().1.fill(0.0);
        }
    }

    fn run(&mut self, observation: &Observation) -> Result<[f32; ACTION_LEN], PolicyError> {
        let fail = |e: String| PolicyError::Inference(format!("{}: {e}", self.path.display()));
        if !observation.as_slice().iter().all(|v| v.is_finite()) {
            return Err(fail("non-finite observation".into()));
        }
        let (_, input) = self.input.try_extract_tensor_mut::<f32>()
            .map_err(|e| fail(format!("updating input: {e}")))?;
        policy_input(observation, self.zero_command, input);
        let outputs = match &self.state {
            Some(state) => self.session.run(ort::inputs![
                "obs" => &self.input, "h_in" => &state.h, "c_in" => &state.c
            ]),
            None => self.session.run(ort::inputs!["obs" => &self.input]),
        }
        .map_err(|e| fail(e.to_string()))?;
        let (_, actions) = outputs[self.action_name.as_str()]
            .try_extract_tensor::<f32>()
            .map_err(|e| fail(e.to_string()))?;
        if actions.len() != ACTION_LEN || !actions.iter().all(|v| v.is_finite()) {
            return Err(fail("expected 14 finite actions".into()));
        }
        let mut result = [0.0; ACTION_LEN];
        result.copy_from_slice(actions);
        if let Some(state) = &mut self.state {
            let (hs, h) = outputs["h_out"]
                .try_extract_tensor::<f32>()
                .map_err(|e| fail(e.to_string()))?;
            let (cs, c) = outputs["c_out"]
                .try_extract_tensor::<f32>()
                .map_err(|e| fail(e.to_string()))?;
            let (expected_h, h_in) = state
                .h
                .try_extract_tensor_mut::<f32>()
                .map_err(|e| fail(e.to_string()))?;
            let (expected_c, c_in) = state
                .c
                .try_extract_tensor_mut::<f32>()
                .map_err(|e| fail(e.to_string()))?;
            // Check both before updating either, including dynamic runtime output shapes.
            if hs != expected_h || cs != expected_c || !h.iter().chain(c).all(|v| v.is_finite()) {
                return Err(fail(
                    "invalid LSTM output state shape or non-finite state".into(),
                ));
            }
            if self.completion_controlled {
                if h.len() != 5 || (h[4] != 0.0 && h[4] != 1.0) {
                    return Err(fail("invalid motion-completion state".into()));
                }
                self.completed = h[4] == 1.0;
            }
            h_in.copy_from_slice(h);
            c_in.copy_from_slice(c);
        }
        Ok(result)
    }
}

fn shape_error(path: &Path, what: &'static str, expected: &str, got: String) -> PolicyError {
    PolicyError::Shape {
        path: path.to_owned(),
        what,
        expected: expected.into(),
        got,
    }
}

/// Require an exact rank and float32 type. Batch may be symbolic, but this runtime
/// always supplies batch one; state layers and hidden width must be known at load.
fn tensor_shape(path: &Path, outlet: &ort::value::Outlet) -> Result<Vec<i64>, PolicyError> {
    match outlet.dtype() {
        ValueType::Tensor {
            ty: TensorElementType::Float32,
            shape,
            ..
        } => Ok(shape.to_vec()),
        other => Err(shape_error(
            path,
            "tensor type",
            "float32",
            format!("{}: {other:?}", outlet.name()),
        )),
    }
}

fn outlet<'a>(
    path: &Path,
    outlets: &'a [ort::value::Outlet],
    name: &str,
) -> Result<&'a ort::value::Outlet, PolicyError> {
    outlets.iter().find(|o| o.name() == name).ok_or_else(|| {
        shape_error(
            path,
            "tensor names",
            name,
            format!("{:?}", outlets.iter().map(|o| o.name()).collect::<Vec<_>>()),
        )
    })
}

fn check_matrix(path: &Path, outlet: &ort::value::Outlet, width: usize) -> Result<(), PolicyError> {
    let shape = tensor_shape(path, outlet)?;
    if shape.len() != 2 || (shape[0] != 1 && shape[0] != -1) || shape[1] != width as i64 {
        return Err(shape_error(
            path,
            "tensor shape",
            &format!("[1 or dynamic, {width}]"),
            format!("{}: {shape:?}", outlet.name()),
        ));
    }
    Ok(())
}

fn open(path: &Path) -> Result<Network, PolicyError> {
    open_with_precision(path, false)
}

fn open_with_precision(path: &Path, precision: bool) -> Result<Network, PolicyError> {
    let bytes = std::fs::read(path).map_err(|source| PolicyError::Read {
        path: path.to_owned(),
        source,
    })?;
    open_bytes(path, precision, &bytes)
}

fn open_bytes(path: &Path, precision: bool, bytes: &[u8]) -> Result<Network, PolicyError> {
    let digest: [u8; 32] = Sha256::digest(bytes).into();
    let session = Session::builder()
        .and_then(|b| b.with_optimization_level(GraphOptimizationLevel::Level3))
        .and_then(|b| b.with_intra_threads(INTRA_THREADS))
        .and_then(|b| b.commit_from_memory(bytes))
        .map_err(|source| PolicyError::Load {
            path: path.to_owned(),
            source,
        })?;

    let inputs = session.inputs();
    let outputs = session.outputs();
    check_matrix(path, outlet(path, inputs, "obs")?, OBS_LEN)?;
    let recurrent = match (inputs.len(), outputs.len()) {
        (1, 1) => false,
        (3, 3) => true,
        counts => {
            return Err(shape_error(
                path,
                "input/output contract",
                "obs -> actions, or obs/h_in/c_in -> actions/h_out/c_out",
                format!("{counts:?} tensors"),
            ));
        }
    };
    // Preserve the existing feed-forward output naming contract (the sole output).
    let action = if recurrent {
        outlet(path, outputs, "actions")?
    } else {
        &outputs[0]
    };
    check_matrix(path, action, ACTION_LEN)?;
    let action_name = action.name().to_owned();
    let state = if recurrent {
        let mut shapes = Vec::new();
        for (outlets, name) in [
            (inputs, "h_in"),
            (inputs, "c_in"),
            (outputs, "h_out"),
            (outputs, "c_out"),
        ] {
            let shape = tensor_shape(path, outlet(path, outlets, name)?)?;
            if shape.len() != 3
                || shape[0] <= 0
                || (shape[1] != 1 && shape[1] != -1)
                || shape[2] <= 0
            {
                return Err(shape_error(
                    path,
                    "LSTM state shape",
                    "[positive layers, 1 or dynamic batch, positive hidden size]",
                    format!("{name}: {shape:?}"),
                ));
            }
            shapes.push(vec![shape[0], 1, shape[2]]);
        }
        if shapes.iter().any(|shape| shape != &shapes[0]) {
            return Err(shape_error(
                path,
                "LSTM state shapes",
                "matching h/c input and output shapes",
                format!("{shapes:?}"),
            ));
        }
        let shape = &shapes[0];
        let count = usize::try_from(shape[0])
            .ok()
            .and_then(|n| n.checked_mul(shape[2] as usize))
            .filter(|&n| n <= 1_048_576)
            .ok_or_else(|| {
                shape_error(
                    path,
                    "LSTM state size",
                    "at most 1048576 elements per state",
                    format!("{shape:?}"),
                )
            })?;
        let make = || {
            Tensor::from_array((shape.clone(), vec![0.0f32; count])).map_err(|source| {
                PolicyError::Load {
                    path: path.to_owned(),
                    source,
                }
            })
        };
        Some(LstmState {
            h: make()?,
            c: make()?,
        })
    } else {
        None
    };
    let encoding = session
        .metadata()
        .map(|metadata| metadata.custom("command_encoding"))
        .map_err(|source| PolicyError::Load {
            path: path.to_owned(),
            source,
        })?;
    let roll_phase = match encoding.as_deref() {
        None | Some("") | Some("constant") | Some("zero_padding") => None,
        Some("precision_displacement" | "null") if precision => None,
        Some("roll_phase") => Some(crate::roll_phase::RollPhase::default()),
        Some("headstand_phase") => Some(crate::roll_phase::RollPhase::headstand()),
        Some(encoding) => {
            return Err(shape_error(
                path,
                "command encoding",
                "constant, zero_padding, roll_phase or headstand_phase",
                encoding.to_owned(),
            ));
        }
    };
    let heading = session.metadata()
        .map(|metadata| metadata.custom("roulade_heading_observation"))
        .map_err(|source| PolicyError::Load { path: path.to_owned(), source })?;
    let relative_heading = crate::roll_phase::heading_contract(heading.as_deref(),
        roll_phase.is_some_and(|phase| !phase.holds_inverted()))
        .map_err(|error| shape_error(path, "roll heading observation", error,
                                   heading.unwrap_or_default()))?;
    let completion = session.metadata()
        .map(|metadata| metadata.custom("kick_completion"))
        .map_err(|source| PolicyError::Load { path: path.to_owned(), source })?;
    let completion_controlled = match completion.as_deref() {
        None | Some("") => false,
        Some("swing_home_hold") => {
            if encoding.as_deref() != Some("zero_padding") || !recurrent
                || tensor_shape(path, outlet(path, session.inputs(), "h_in")?)? != vec![1, 1, 5] {
                return Err(shape_error(path, "kick completion", "zero-padding skill with [1,1,5] state", "incompatible graph".into()));
            }
            true
        }
        Some(other) => return Err(shape_error(path, "kick completion", "swing_home_hold", other.into())),
    };
    let input = Tensor::from_array(([1usize, OBS_LEN], vec![0.0f32; OBS_LEN]))
        .map_err(|source| PolicyError::Load { path: path.to_owned(), source })?;
    Ok(Network {
        session,
        input,
        state,
        action_name,
        path: path.to_owned(),
        digest,
        roll_phase,
        relative_heading,
        zero_command: encoding.as_deref() == Some("zero_padding"),
        completion_controlled,
        completed: false,
    })
}
