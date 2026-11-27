//! `robotd` — the robot control daemon.
//!
//! The 50 Hz control loop, the `robot.*` socket, and the health the updater gates on.
//!
//! **Health is why the daemon has the shape it does.** The updater's auto-rollback is only
//! meaningful if `robot.health` means something, and it used to mean "the loop ticked once" —
//! so every rollback was tested against a placeholder. It now means **the loop is meeting its
//! deadline**: a loop running at 60% of target is alive, answers every request, and is badly
//! broken.
//!
//! Holding the pose with no policy (`policy.enabled = false`) stays the right thing to be doing
//! while someone deliberately breaks releases at a bench: the bus sees the real load at the
//! real rate, and nothing falls over when a bad build lands.
//!
//! Every method must be answerable *while the robot is in a bad state*,
//! since that is exactly when it is asked. So the IPC side reads atomics the control loop
//! publishes and never calls into the loop — a wedged loop reports itself unhealthy rather
//! than hanging the caller.

mod camera_process;
mod chorale;
mod control;
mod emergency_relax;
mod coordinated_mouth;
mod coordinated_tracking;
mod gamepad_process;
mod head_tracking;
mod human_control;
mod motion_cpu;
mod motion_tracking;
mod maze;
mod managed_models;






mod idle_yawn;
mod imu_calibration;
mod imu_input;
mod low_voltage;
mod low_voltage_lock;
mod obstacle;
mod policy_gravity;
mod precision_motion;
mod intents;
mod velocity_command;
mod web_process;
#[cfg(target_os = "linux")]
mod control_scheduling;






mod log_output;
mod params;
mod record;
mod rest_action;
mod soc;
mod sound;
mod studio_process;
mod status_bus;
mod status_stop;
mod target_follow_stability;
mod telemetry;
mod theremin;


use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};
use status_stop::{StatusMaintenanceStop, WALK_STAND_SETTLE};

use arc_swap::{ArcSwap, ArcSwapOption};
use clap::{Parser, Subcommand};
use duck_control::fall::{FallPredictor, FallPredictorConfig};
use duck_control::hx::ImuSource;
use duck_control::io::RobotIo;
use duck_control::obs::{BodyPose, Command as PolicyCommand};
use duck_control::policy::{DEFAULT_STANDING_THRESHOLD, Policy, PolicyError, PolicyPaths, PrecisionPolicy};
use duck_control::safety::{Safety, SafetyConfig};
use duck_control::{DEFAULT_POSITION, FakeIo, NUM_JOINTS};
use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};

use control::{Controller, Driving, SkillTuning, Tuning};
use intents::Intents;
use params::{BusProtocol, Mode, Params, Slot};

/// What to do when the shutdown sequence completes. Injected so the tests can observe the
/// call instead of powering off the machine running them.
type PowerOff = Arc<dyn Fn() + Send + Sync>;

/// Model API version this build implements (`updater-design.md` §5.5). Bump when the
/// sensor-input / actuator-output contract a model sees changes.
// API 2 adds explicit recurrent state; API 1 feed-forward models remain supported.
const MODEL_API: u32 = 2;

/// Socket mode. Same reasoning as `updaterd`'s: the group decides who may ask.
const SOCKET_MODE: u32 = 0o660;

const MAX_LINE: usize = 64 * 1024;

/// How often the loop logs a summary at `info`.
///
/// Per-tick logging would be ~4.3M lines a day at 50 Hz. That is not merely noise: under a
/// journal size cap it is what *evicts* the logs support needs.
const LOOP_SUMMARY_INTERVAL: Duration = Duration::from_secs(300);

/// How fast the beak follows the vowel being sung, as a time constant.
///
/// A vowel is a step — `ah` opens the mouth to 0.90 and `mm` to 0.02 — and a servo asked to jump
/// between them on a 20 ms tick twitches rather than sings. This is roughly how fast a jaw moves.
const CHORALE_MOUTH_TAU_S: f64 = 0.09;

/// How far a subscriber may fall behind before it starts losing frames.
///
/// Five seconds at 50 Hz. State is advisory: a client that cannot keep up gets a gap, never
/// backpressure onto the control loop. Same rule the updater applies to progress.
const STATE_BUFFER: usize = 256;

/// Window over which the achieved rate is measured, and therefore how quickly a degraded
/// loop becomes visible to the health gate. Doubles as the slow-sensor sampling interval
/// ([`publish_slow_sensors`]).
const RATE_WINDOW: Duration = Duration::from_secs(1);

/// How long the ramp to the home pose takes when the policy is first enabled.
///
/// The same half-second `robotd init` uses, and for the same reason: fast enough that nobody wonders
/// whether the button did anything, slow enough that a robot going from limp to standing does not
/// snap. Not a parameter yet — one number with no evidence that anyone wants a different one.
const HOME_RAMP: Duration = Duration::from_millis(500);

/// Smoothing on the reported battery voltage. At one sample per [`RATE_WINDOW`] this is a
/// ~10 s time constant, which is what makes the number readable: the raw voltage sags
/// several tenths of a volt on every step and recovers between them, so an unsmoothed
/// reading swings while the pack is doing nothing unusual. Borrowed, with the figure, from
/// `microduck_runtime`.
const BATTERY_EMA_ALPHA: f64 = 0.1;

/// How long the policy holds the shutdown sit before the rest ramp, when no controller is there
/// to say. The sitstand descent is a deliberate ~2 s glide and the robot then sits still for a
/// second; a running controller derives the same from the set's `ramp_s`
/// (`Controller::shutdown_sit_secs`).
const SHUTDOWN_SIT: Duration = Duration::from_secs(3);

/// The shutdown's last leg, after the sit: a linear ramp from the seat to
/// [`duck_control::REST_POSITION`], then a hold there, then torque off. Cutting torque straight
/// from the policy's seat let the robot sag forward; from the rest pose there is nowhere left to
/// sag to. Slow on purpose — the robot is on the floor and nothing is in a hurry.
const REST_RAMP: Duration = Duration::from_secs(1);
const REST_HOLD: Duration = Duration::from_secs(1);
const EXIT_SIT_TIMEOUT: Duration = Duration::from_secs(15);
const EXIT_SIT_REQUESTED: u8 = 1;
const EXIT_SIT_RUNNING: u8 = 2;
const EXIT_SIT_DONE: u8 = 3;
const EXIT_HEAD_30_MS: u64 = 1000;
const EXIT_HEAD_31_DELAY_MS: u64 = 1000;
const EXIT_HEAD_31_MS: u64 = 500;

/// How many consecutive failed bus reads the policy may drive through on the last good
/// sample before it stops.
///
/// **One dropped Dynamixel transaction is ordinary** — `robotd` says so itself when it logs
/// one — and it must therefore be *invisible*. It was not: a failed read used to stop the
/// policy for that tick, which commanded the hold pose and reset the controller, so every
/// ordinary dropped read produced a visible twitch. Measured on a bench robot at ~8 drops a
/// minute with a monitor attached, which is exactly the reported "random tiny spasms".
///
/// Coasting is safe because the observation is *already* a tick old by construction: at
/// 50 Hz the policy is trained on data of exactly this age, and a second tick of it is
/// inside that. Three ticks (60 ms) covers a drop, a retry and a slow tick; past that the
/// robot genuinely cannot see, and holding still is the honest answer.
const COAST_TICKS: u32 = 3;

/// How long driving must have been interrupted before the controller is reset.
///
/// The reset zeroes the action history the policy observes and drops the low-pass anchor,
/// which is right after a real pause and is itself a discontinuity after a 20 ms hiccup —
/// the very jolt it exists to prevent.
const RESET_AFTER_PAUSE: Duration = Duration::from_millis(200);

/// Where the limp-fall sequence is (`[safety] limp_fall`).
///
/// The daemon's only answer to a fall, and it runs *during* one rather than after it: the
/// trigger is [`duck_control::fall::FallPredictor`], not the `fallen` verdict, because a
/// verdict that waits for the robot to be down cannot make it soft before it lands.
///
/// The verdict is still tracked and published throughout — it just does not gate anything.
#[derive(Debug, Clone, Copy, PartialEq)]
enum LimpFall {
    Idle,
    /// Gains at `gain_limp`, joints commanded to wherever they already are — the softest
    /// thing the servos can do short of cutting torque, which would drop the head on the
    /// floor and lose the pose the next phase ramps from.
    Limp {
        since: Instant,
        landing: Landing,
    },
    /// Ramping from where the robot landed back to the standing pose, so the standing
    /// policy starts from the still, known posture it stands up from cleanly.
    Posing {
        from: [f64; NUM_JOINTS],
        since: Instant,
    },
}

/// Watching a limping robot for the moment it stops moving.
///
/// The end of the limp is read from the gyro rather than timed, because the falls are not
/// all the same length: a stumble is over in a few hundred milliseconds and a topple off a
/// table takes most of a second, and ending the limp early is landing stiff after all.
/// Debounced, because a tumbling robot passes through instants of near-zero rate on its
/// way over — one quiet sample is not a landing.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct Landing {
    still_for: Duration,
}

impl Landing {
    /// One tick. `rate` is the gyro magnitude in rad/s, or `None` for a tick with no fresh
    /// sample — which resets rather than accumulates: a robot nobody can read is not a
    /// robot anybody has seen go still.
    fn observe(&mut self, rate: Option<f64>, period: Duration, below: f64, held: Duration) -> bool {
        self.still_for = match rate {
            Some(rate) if rate < below => self.still_for.saturating_add(period),
            _ => Duration::ZERO,
        };
        self.still_for >= held
    }
}

impl LimpFall {
    fn position_p_gain(&self, gains: params::FallPositionGains) -> u8 {
        match self {
            Self::Idle => gains.running,
            Self::Limp { .. } => gains.limp,
            Self::Posing { .. } => gains.posing,
        }
    }

    fn apply_position_p_gain<T: RobotIo>(
        &self,
        safety: &mut Safety<T>,
        gains: Option<params::FallPositionGains>,
    ) -> duck_control::io::Result<()> {
        if let Some(gains) = gains {
            safety.set_position_p_gain(self.position_p_gain(gains))?;
        }
        Ok(())
    }

    /// The interpolated pose target for this tick, or `None` once the ramp is done.
    ///
    /// The same linear shape as [`Bringup::homing_target`], and separate from it on
    /// purpose: that ramp is bring-up from limp at the policy gain over a fixed two
    /// seconds, this one is a landed robot being put back into shape, and both its
    /// duration and its gain are tunable because both depend on how the robot lands.
    fn pose_target(
        &self,
        now: Instant,
        over: Duration,
        home: &[f64; NUM_JOINTS],
    ) -> Option<[f64; NUM_JOINTS]> {
        let LimpFall::Posing { from, since } = self else {
            return None;
        };
        let t = now.duration_since(*since).as_secs_f64() / over.as_secs_f64();
        if t >= 1.0 {
            return None;
        }
        let mut target = [0.0; NUM_JOINTS];
        for (i, slot) in target.iter_mut().enumerate() {
            *slot = from[i] + (home[i] - from[i]) * t;
        }
        Some(target)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ExitHead31Params {
    delay_ms: u64,
    duration_ms: u64,
}

fn parse_exit_head_31(value: &str) -> Result<ExitHead31Params, String> {
    let (delay, duration) = value.split_once(',')
        .ok_or_else(|| "expected DELAY_MS,DURATION_MS".to_owned())?;
    let delay_ms: u64 = delay.parse().map_err(|_| "invalid servo 31 delay".to_owned())?;
    let duration_ms: u64 = duration.parse().map_err(|_| "invalid servo 31 duration".to_owned())?;
    if delay_ms > 60000 || !(1..=60000).contains(&duration_ms) {
        return Err("delay must be 0–60000 ms and duration must be 1–60000 ms".to_owned());
    }
    Ok(ExitHead31Params { delay_ms, duration_ms })
}

fn parse_tof_signal_rate(value: &str) -> Result<f64, String> {
    let rate: f64 = value.parse().map_err(|_| "invalid ToF signal rate".to_owned())?;
    if !rate.is_finite() || !(0.01..=1.0).contains(&rate) {
        return Err("ToF signal rate must be 0.01–1.0 MCPS".to_owned());
    }
    Ok(rate)
}

#[derive(Parser, Debug)]
#[command(name = "robotd", about = "Robot control daemon", version)]
struct Args {
    /// Socket to serve the `robot.*` API on. `updaterd --robot-socket` must match.
    #[arg(long, default_value = "/run/robotd.sock")]
    socket: PathBuf,

    /// Full diagnostic log (INFO and HX DEBUG). Defaults to the socket path with .log.
    /// Rotates at 10 MiB with three backups; stderr shows concise summaries via RUST_LOG.
    #[arg(long, value_name = "PATH")]
    log_file: Option<PathBuf>,

    /// Params file. Defaults to `/etc/robot/robotd.toml`, which may be absent — an
    /// unprovisioned board comes up on defaults. A path given here must exist.
    #[arg(long)]
    params: Option<PathBuf>,

    /// Serial port override, for a board wired differently from the shipped default.
    #[arg(long)]
    port: Option<String>,

    /// IMU source; id200-host consumes raw 100 Hz batches for host gravity and gyro EMA.
    #[arg(long, value_name = "i2c|id200|id200-timing|id201|id200-host")]
    imu_source: Option<ImuSource>,

    /// Explicitly enable a secondary recording IMU; omitted means no secondary acquisition.
    /// Bus references require IMU_TIMING=1 firmware.
    #[arg(long, requires = "record", value_name = "i2c|id200|id201")]
    reference_imu_source: Option<ImuSource>,

    /// Read the selected shared-bus IMU after the servos and recording reference.
    #[arg(long)]
    imu_last: bool,

    /// On initialization, move to HOME before allowing the policy to drive.
    /// By default initialization enables torque at the current pose.
    #[arg(long, global = true, conflicts_with = "init_torque_only")]
    init_home: bool,

    /// Keep the current pose on initialization (the default; retained for older commands).
    #[arg(long, global = true, conflicts_with = "init_home")]
    init_torque_only: bool,

    /// Apply saved paired pose alignment (either source); gyro bias is owned by MCU.
    /// Disabling this does not disable calibration stored inside the I2C module.
    #[arg(long, default_value_t = true, value_name = "true|false", action = clap::ArgAction::Set)]
    use_imu_calibration: bool,

    /// Apply saved ID200 accelerometer correction before MCU fusion (requires runtime-switch firmware).
    #[arg(long, default_value_t = true, value_name = "true|false", action = clap::ArgAction::Set)]
    use_accel_calibration: bool,

    /// Start the adjacent padd binary with classic layout, this socket and params file.
    /// The daemon owns the client and stops it during shutdown.
    #[arg(long)]
    with_gamepad: bool,

    /// Interpret short stick gestures as precision movements and replay long gestures after 300 ms.
    #[arg(long)]
    gamepad_precision: bool,

    /// Start the installed camera launcher also with --fake or --sim.
    /// On physical hardware the installed launcher starts automatically.
    #[arg(long, conflicts_with = "no_camera")]
    with_camera: bool,

    /// Do not start the installed camera launcher.
    #[arg(long)]
    no_camera: bool,

    /// Disable automatic camera, distance sensor, and web control services.
    #[arg(long, conflicts_with = "with_camera")]
    motion_only: bool,

    /// Single-zone ToF target sample rate in Hz (default: 20).
    #[arg(long, default_value_t = 20, value_parser = clap::value_parser!(u32).range(2..=20))]
    tof_hz: u32,

    /// ToF measurement timing budget in milliseconds; omit to use the sensor default.
    #[arg(long, value_name = "MS", value_parser = clap::value_parser!(u32).range(20..=500))]
    tof_timing_budget_ms: Option<u32>,

    /// ToF return-signal threshold in MCPS; omit to use the driver default (0.25).
    #[arg(long, value_name = "MCPS", value_parser = parse_tof_signal_rate)]
    tof_signal_rate_limit: Option<f64>,

    /// Gamepad start delay and short-press threshold in milliseconds; 0 disables both (default: 0).
    #[arg(long, requires = "with_gamepad", value_name = "MS")]
    gamepad_start_delay_ms: Option<u64>,

    /// Short gamepad release command duration in milliseconds; 0 disables both (default: 0).
    #[arg(long, requires = "with_gamepad", value_name = "MS")]
    gamepad_short_duration_ms: Option<u64>,

    #[command(flatten)]
    gamepad_limits: gamepad_process::SpeedLimits,

    /// Skip servo torque release when exiting on Ctrl-C (SIGINT).
    /// By default Ctrl-C releases torque; gamepad failure still releases it.
    #[arg(long)]
    keep_torque_on_ctrl_c: bool,

    /// Exit positioning duration for servo 30, in milliseconds.
    #[arg(long, default_value_t = EXIT_HEAD_30_MS, value_name = "MS", value_parser = clap::value_parser!(u64).range(1..=60000))]
    exit_head_30_ms: u64,

    /// Servo 31 delay from servo 30 start and positioning duration, in milliseconds.
    #[arg(long, default_value = "1000,500", value_name = "DELAY_MS,DURATION_MS", value_parser = parse_exit_head_31)]
    exit_head_31_ms: ExitHead31Params,

    /// Run against a robot made of nothing. For laptop development and tests, and for the cases
    /// `--sim` does not cover: no physics, no falling over, positions that echo back perfectly.
    #[arg(long)]
    fake: bool,

    /// Run against a robot in MuJoCo, at `host:port`.
    ///
    /// **The same daemon, a simulated body.** Everything above `duck_control::io::RobotIo` — the
    /// loop, the policy, safety, fall detection, odometry, kinematics, every IPC call — is the code
    /// that runs on a robot, unchanged and unable to tell. `duck_control::sim` has the protocol.
    ///
    /// Nothing is connected until the first tick, and a simulator that goes away is retried on
    /// every tick after — restarting MuJoCo must not mean restarting the duck.
    #[arg(long, conflicts_with = "fake")]
    sim: Option<String>,

    /// Do not load a policy: run the loop and hold the startup pose.
    ///
    /// Distinct from a policy that failed to load, which is unhealthy. This is the
    /// configuration to use when the thing under test is the updater rather than the gait —
    /// nothing falls over when a deliberately broken release lands.
    #[arg(long, conflicts_with = "enable_policies")]
    no_policy: bool,

    /// Save one-second bus read and seven-stage timing summaries in the detailed log.
    /// Terminal output requires DEBUG logging.
    /// HX reads use the selected IMU source alongside the servo broadcast.
    #[arg(long)]
    measure_servo_read_time: bool,

    /// Normal Linux scheduler weight for the physical control thread; zero uses
    /// ordinary weight. Child threads/processes retain ordinary weight.
    #[arg(long, default_value_t = -10, allow_hyphen_values = true)]
    control_nice: i32,

    #[command(flatten)]
    policy_overrides: PolicyOverrides,

    /// Save control records from the first policy step for comparison with simulation.
    ///
    /// The path names one JSONL file, overwritten at startup; parent directories are created.
    /// Samples begin when the policy first drives and
    /// continue until shutdown. A dedicated writer drains a non-blocking bounded queue.
    #[arg(long, value_name = "FILE")]
    record: Option<PathBuf>,

    /// Record only while person following is active, including waiting and target search.
    #[arg(long, requires = "record", conflicts_with = "motion_only")]
    record_person_follow: bool,

    /// Record only while color-target tracking is active, including loss and arrival actions.
    #[arg(long, requires = "record", conflicts_with = "motion_only")]
    record_target_follow: bool,

    /// Read firmware counters at 0x80..0x8B after goal writes for these servo IDs.
    /// Requires the diagnostic firmware; zero values alone do not establish support.
    #[arg(long, value_delimiter = ',', requires = "record")]
    servo_diagnostics_ids: Vec<u8>,

    #[command(subcommand)]
    command: Option<Command>,
}

impl Args {
    fn hold_current_pose_on_init(&self) -> Result<bool, &'static str> {
        // Clap does not catch conflicting global flags placed on opposite sides of a subcommand.
        if self.init_home && self.init_torque_only {
            return Err("--init-home and --init-torque-only cannot be used together");
        }
        Ok(!self.init_home)
    }

    fn record_extended_servo_feedback(&self) -> bool {
        self.record.is_some()
            && (!(self.record_person_follow || self.record_target_follow) || self.reference_imu_source.is_some())
    }

    /// The raw host comparison owns its whole IMU pipeline; saved calibration stays intact.
    fn configure_imu_pipeline(&mut self, source: ImuSource) -> Result<(), String> {
        if source == ImuSource::Id200Host {
            if self.policy_overrides.imu_filter == Some(true)
                || self.policy_overrides.imu_gyro_filter == Some(true)
            {
                return Err("id200-host already applies gyro EMA; extra IMU median filters must be disabled".into());
            }
            self.policy_overrides.imu_filter = Some(false);
            self.policy_overrides.imu_gyro_filter = Some(false);
            self.use_imu_calibration = false;
            self.use_accel_calibration = false;
        }
        Ok(())
    }

    fn validate_policy_gravity(&self, protocol: BusProtocol, source: ImuSource, frozen: bool) -> Result<(), String> {
        if !self.policy_overrides.policy_gravity_from_reference { return Ok(()); }
        if self.record.is_none() || protocol != BusProtocol::Hx || self.fake || self.sim.is_some() {
            return Err("--policy-gravity-from-reference requires --record and physical bus.protocol = hx".into());
        }
        let reference = self.reference_imu_source.ok_or_else(||
            "--policy-gravity-from-reference requires explicit --reference-imu-source".to_string())?;
        if !matches!((source, reference),
            (ImuSource::Id200 | ImuSource::Id200Timing, ImuSource::Id201)
            | (ImuSource::Id201, ImuSource::Id200 | ImuSource::Id200Timing)) {
            return Err("--policy-gravity-from-reference requires the physical ID200/ID201 pair".into());
        }
        if self.policy_overrides.imu_filter != Some(false) {
            return Err("--policy-gravity-from-reference requires explicit --imu-filter false".into());
        }
        if frozen {
            return Err("--policy-gravity-from-reference conflicts with ROBOT_FREEZE_POLICY_IMU=1".into());
        }
        Ok(())
    }

    fn selected_imu_source(&self, protocol: BusProtocol) -> Result<ImuSource, String> {
        if (self.imu_source.is_some() || self.imu_last || self.reference_imu_source.is_some()) && protocol != BusProtocol::Hx {
            return Err("--imu-source requires bus.protocol = hx".into());
        }
        let source = self.imu_source.unwrap_or_default();
        if let Some(reference) = self.reference_imu_source {
            if reference == ImuSource::Id200Host || reference == source
                || reference.bus_id().is_some_and(|id| source.bus_id() == Some(id)) {
                return Err("--reference-imu-source must select a different physical IMU, without host fusion".into());
            }
        }
        if self.imu_last && !source.shared_bus() {
            return Err("--imu-last requires a shared-bus IMU source".into());
        }
        if !self.use_accel_calibration && !source.shared_bus() {
            return Err("--use-accel-calibration false requires --imu-source id200".into());
        }
        Ok(source)
    }
}

/// Process-local policy settings, reapplied whenever the config is reloaded.
#[derive(clap::Args, Debug, Clone, Default)]
struct PolicyOverrides {
    /// Use the other ID200/ID201 IMU only for policy observation gravity; requires --imu-filter false.
    #[arg(long, requires = "record")]
    policy_gravity_from_reference: bool,

    /// Immediately clear each velocity axis whose target is zero; nonzero targets still smooth.
    /// Without this flag, released axes retain the configured command smoothing.
    #[arg(long)]
    immediate_stop: bool,

    /// Load only these policy slots for this process. This does not edit `--params`.
    ///
    /// `velocity` loads `velocity.onnx` from the same directory as `[policy].walk`.
    /// `headstand` loads the phase-aware skill beside `[policy].stand`, triggered by
    /// `robotctl robot do headstand`; it holds inverted until explicitly switched away.
    /// When this leaves `sitstand` as the only base policy, it drives both the standing
    /// posture and the commanded sit/rise transition.
    #[arg(
        long,
        value_name = "SLOT[,SLOT...]",
        value_delimiter = ',',
        value_parser = parse_policy_slot,
    )]
    enable_policies: Option<Vec<EnableSlot>>,

    /// Head/neck action smoothing coefficient (0 < alpha <= 1); 1 disables smoothing.
    #[arg(long, value_name = "ALPHA", value_parser = parse_lowpass)]
    head_lowpass: Option<f64>,

    /// Leg action smoothing coefficient (0 < alpha <= 1); 1 disables smoothing.
    #[arg(long, value_name = "ALPHA", value_parser = parse_lowpass)]
    legs_lowpass: Option<f64>,

    /// Scale policy actions by nominal/measured voltage. Overrides the config for this run.
    #[arg(long, value_name = "true|false", action = clap::ArgAction::Set)]
    voltage_adapt: Option<bool>,

    /// Apply host three-sample medians to policy gyro and gravity (default: true).
    /// False preserves mounting, calibration, sensor fusion and safety filtering.
    #[arg(long, value_name = "true|false", action = clap::ArgAction::Set, conflicts_with = "imu_gyro_filter")]
    imu_filter: Option<bool>,

    /// Apply the three-sample median to policy gyro input (default: true).
    /// False keeps mounting and bias correction; gravity and safety keep their filters.
    #[arg(long, value_name = "true|false", action = clap::ArgAction::Set)]
    imu_gyro_filter: Option<bool>,
}

impl PolicyOverrides {
    fn velocity_command_safety(&self) -> bool {
        !self.immediate_stop
            && self.enable_policies.as_ref().is_some_and(|slots| {
                slots.len() == 1 && slots[0] == EnableSlot::Velocity
            })
    }

    fn apply(&self, policy: &mut params::PolicyParams) -> Result<(), String> {
        if let Some(selection) = self.enable_policies.as_deref() {
            let slots = resolve_enable_policies(policy, selection)?;
            policy
                .skills
                .retain(|skill| match Slot::parse(&skill.name) {
                    Some(slot) => slots.contains(&slot),
                    None => skill.name == "headstand" && selection.contains(&EnableSlot::Headstand),
                });
            apply_policy_selection(policy, &slots)?;
        }
        if let Some(alpha) = self.head_lowpass {
            policy.head_lowpass = Some(alpha);
        }
        if let Some(alpha) = self.legs_lowpass {
            policy.legs_lowpass = Some(alpha);
        }
        if let Some(enabled) = self.voltage_adapt {
            policy.voltage_adapt = enabled;
        }
        Ok(())
    }
}

fn parse_lowpass(value: &str) -> Result<f64, String> {
    let alpha = value
        .parse::<f64>()
        .map_err(|_| "expected 0 < alpha <= 1")?;
    if !alpha.is_finite() || alpha <= 0.0 || alpha > 1.0 {
        return Err("expected 0 < alpha <= 1; use 1 to disable smoothing".to_owned());
    }
    Ok(alpha)
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Enable torque at the current pose, then exit; use --init-home to move to HOME.
    ///
    /// Explicit, and separate from running the daemon, because the control loop must never
    /// move the robot on its own: `robotd` restarting during an update would otherwise make
    /// a standing robot lurch, which is both a fall risk and a confounder when the thing
    /// under test is the updater.
    Init {
        #[arg(long, default_value = "500ms", value_parser = parse_duration)]
        duration: Duration,
    },
}

fn parse_duration(raw: &str) -> Result<Duration, String> {
    let (value, scale) = match raw.strip_suffix("ms") {
        Some(v) => (v, 1u64),
        None => match raw.strip_suffix('s') {
            Some(v) => (v, 1000),
            None => (raw, 1000),
        },
    };
    value
        .parse::<u64>()
        .map(|n| Duration::from_millis(n * scale))
        .map_err(|_| format!("expected e.g. 500ms or 2s, got {raw:?}"))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EnableSlot {
    Slot(Slot),
    Velocity,
    Headstand,
}

impl From<Slot> for EnableSlot {
    fn from(slot: Slot) -> Self {
        EnableSlot::Slot(slot)
    }
}

fn parse_policy_slot(value: &str) -> Result<EnableSlot, String> {
    if value == "velocity" {
        return Ok(EnableSlot::Velocity);
    }
    if value == "headstand" {
        return Ok(EnableSlot::Headstand);
    }
    Slot::parse(value).map(EnableSlot::Slot).ok_or_else(|| {
        format!(
            "unknown policy slot {value:?}; expected one of {}, velocity, headstand",
            Slot::names()
        )
    })
}

/// `velocity` is the walk network loaded from `velocity.onnx` beside `[policy].walk`.
fn resolve_enable_policies(
    policy: &mut params::PolicyParams,
    selection: &[EnableSlot],
) -> Result<Vec<Slot>, String> {
    if selection.is_empty() {
        return Err("--enable-policies needs at least one slot".to_owned());
    }
    let mut slots = Vec::with_capacity(selection.len());
    for &item in selection {
        if selection.iter().filter(|&&other| other == item).count() > 1 {
            let name = match item {
                EnableSlot::Velocity => "velocity",
                EnableSlot::Headstand => "headstand",
                EnableSlot::Slot(slot) => slot.as_str(),
            };
            return Err(format!("--enable-policies names {name} more than once"));
        }
        match item {
            EnableSlot::Headstand => {
                if policy.mode != Mode::Walk {
                    return Err("headstand requires walk mode".into());
                }
                let configured = policy
                    .skills
                    .iter()
                    .find(|skill| skill.name == "headstand")
                    .and_then(|skill| skill.path.clone());
                let path = match configured {
                    Some(path) if !params::is_none_sentinel(&path) => path,
                    Some(_) => return Err("headstand is disabled in [policy.skill]".into()),
                    None => policy
                        .stand
                        .as_ref()
                        .filter(|path| !params::is_none_sentinel(path))
                        .ok_or("headstand requires [policy].stand to locate headstand.onnx")?
                        .with_file_name("headstand.onnx"),
                };
                policy.skills.retain(|skill| skill.name != "headstand");
                policy.skills.push(params::SkillDef {
                    name: "headstand".into(),
                    path: Some(path),
                    // The measured phase owns the lifetime, including the inverted hold.
                    duration: 0.0,
                    params: params::SkillOverrides {
                        action_scale: Some(1.0),
                        ..Default::default()
                    },
                    ..Default::default()
                });
            }
            EnableSlot::Velocity => {
                if selection.contains(&EnableSlot::Slot(Slot::Walk)) {
                    return Err("--enable-policies cannot combine walk and velocity".to_owned());
                }
                match policy.slot(Slot::Walk) {
                    Some(path) if !params::is_none_sentinel(path) => {
                        policy.set_slot(Slot::Walk, Some(path.with_file_name("velocity.onnx")));
                    }
                    _ => {
                        return Err(
                            "--enable-policies velocity requires [policy].walk to name a model file"
                                .to_owned(),
                        );
                    }
                }
                slots.push(Slot::Walk);
            }
            EnableSlot::Slot(slot) => slots.push(slot),
        }
    }
    Ok(slots)
}

/// Restrict this process to an explicit policy subset without changing its TOML file.
///
/// A selected slot must name an override rather than falling back to the board's bundled
/// default. Otherwise a typo or an unset key could make this option appear to load a requested
/// model while actually loading the official one.
fn apply_policy_selection(
    policy: &mut params::PolicyParams,
    selection: &[Slot],
) -> Result<(), String> {
    if selection.is_empty() && !policy.skills.iter().any(|skill| skill.name == "headstand") {
        return Err("--enable-policies needs at least one slot".to_owned());
    }
    for &slot in selection {
        if selection.iter().filter(|&&other| other == slot).count() > 1 {
            return Err(format!("--enable-policies names {slot} more than once"));
        }
        match policy.slot(slot) {
            Some(path) if !params::is_none_sentinel(path) => {}
            _ => {
                return Err(format!(
                    "--enable-policies requires [policy].{} to name a model file",
                    slot.as_str()
                ));
            }
        }
    }
    for slot in Slot::ALL {
        if !selection.contains(&slot) {
            policy.set_slot(slot, Some(PathBuf::from("none")));
        }
    }
    Ok(())
}

/// What the control loop publishes about itself.
///
/// Atomics rather than a mutex on purpose: the IPC side must never be able to block on the
/// control loop. A robot whose loop is wedged still has to be able to say "I am not
/// healthy" — if answering required the loop's lock, the one situation where `updaterd`
/// needs an answer is the situation it would hang in.
/// [`Mode`] as the byte [`RobotState::mode`] holds, and back.
///
/// Two tiny functions rather than a numeric cast, so the mapping is written down once: a mode
/// added later gets a code here and nowhere else, and an unknown byte reads as walking rather
/// than panicking on the IPC thread.
fn mode_code(mode: Mode) -> u8 {
    match mode {
        Mode::Walk => 0,
        Mode::Roller => 1,
    }
}

fn mode_of(code: u8) -> Mode {
    match code {
        1 => Mode::Roller,
        _ => Mode::Walk,
    }
}

/// The policy file names for one mode, as a set.
///
/// `None` in a skill slot doubles as "this robot cannot do that", which is what lets `dispatch`
/// refuse a `robot.do` for a skill this mode has no network for instead of queueing a request the
/// loop will drop — and roller mode is exactly that case: no standing network, a crouch where the
/// ground pick was.
#[derive(Debug, Clone, Default)]
struct PolicyNames {
    walk: Option<String>,
    stand: Option<String>,
    sitstand: Option<String>,
    ground_pick: Option<String>,
    /// The configurable one-shots this robot has, in priority order — the names a client may
    /// ask for, and the answer to "what can this robot do".
    skills: Vec<String>,
}

impl PolicyNames {
    /// The names for `policy`, or all-`None` when no policy is wanted.
    fn of(policy: &params::ResolvedPolicy) -> Self {
        if !policy.enabled {
            return Self::default();
        }
        let name = |path: &Option<std::path::PathBuf>| path.as_deref().and_then(file_name);
        Self {
            walk: policy.walk.as_deref().and_then(file_name),
            stand: name(&policy.stand),
            sitstand: name(&policy.sitstand),
            ground_pick: name(&policy.ground_pick),
            skills: policy.skills.iter().map(|s| s.name.clone()).collect(),
        }
    }
}

/// Why a slot is not running what was asked of it, for as long as that stays true.
///
/// A `Vec` rather than seven fields or a map: it is empty on nearly every robot, at most seven
/// long, and the linear scan is over a list shorter than the branch predictor's memory.
#[derive(Debug, Default, Clone)]
struct SlotErrors(Vec<(Slot, String)>);

impl SlotErrors {
    fn get(&self, slot: Slot) -> Option<&str> {
        self.0
            .iter()
            .find(|(s, _)| *s == slot)
            .map(|(_, reason)| reason.as_str())
    }

    fn set(&mut self, slot: Slot, reason: String) {
        self.clear(slot);
        self.0.push((slot, reason));
    }

    fn clear(&mut self, slot: Slot) {
        self.0.retain(|(s, _)| *s != slot);
    }
}

/// Where a policy file came from, for `robot.policies`.
///
/// Three origins, and the path is enough to tell them apart because the path is *made* of the
/// answer: the official set lives in its own directory, and a fetched policy lands under
/// `<library>/<org>/<name>/<revision>/`, so the org that published it is a path component. A
/// file anywhere else is one somebody put there by hand.
///
/// **Not a security boundary**, and nothing here pretends otherwise. Someone who can edit
/// `robotd.toml` can point a slot at a directory named to look official, and someone who can do
/// that can run whatever they like anyway. What stands between a bad policy and a broken robot
/// is the safety layer and the shape gate, not this label. This is here so a person can see what
/// they are running.
fn origin_of(path: &std::path::Path) -> &'static str {
    if path.starts_with(params::POLICY_DIR) {
        return "official";
    }
    if let Ok(rest) = path.strip_prefix(params::POLICY_LIBRARY)
        && let Some(org) = rest.components().next()
    {
        return if org.as_os_str() == OFFICIAL_ORG {
            "official"
        } else {
            "community"
        };
    }
    "local"
}

/// The org whose policies are official. One constant, matching `updater::policy::OFFICIAL_ORG`:
/// a robot that can be *told* which org to trust has a badge that means nothing.
const OFFICIAL_ORG: &str = "pollen-robotics";

/// The per-slot answer to `robot.policies`: what is loaded, from where, and why not.
///
/// Derived rather than stored, from the three things that between them decide it — what config
/// says, what that resolved to, and what failed. Storing it instead would be a fourth place for
/// the same fact, updated from three.
fn slot_report(
    policy_params: &params::PolicyParams,
    cfg: &params::ResolvedPolicy,
    errors: &SlotErrors,
) -> Vec<proto::PolicySlot> {
    Slot::ALL
        .into_iter()
        .map(|slot| {
            let path = cfg.slot(slot);
            proto::PolicySlot {
                slot: slot.as_str().to_owned(),
                path: path.map(|p| p.display().to_string()),
                origin: path.map(|p| origin_of(p).to_owned()),
                overridden: policy_params.slot(slot).is_some(),
                error: errors.get(slot).map(str::to_owned),
            }
        })
        .collect()
}

fn publish_managed_models(state: &RobotState, controller: &Option<Controller>, cfg: &params::ResolvedPolicy) {
    let models = controller.as_ref().map_or_else(Vec::new, |controller| controller.managed_models(cfg));
    state.managed_models.models.store(Arc::new(models));
}

/// Drop any *overridden* slot whose file will not load, before the controller is built.
///
/// The sharp edge of `robotctl policy load` writing config: a file that loaded when it was tried
/// can be gone by the next boot — deleted, on a wiped library, on a reflashed board whose config
/// was restored. Without this the daemon reports unhealthy on every start, and `updaterd`'s
/// health gate then rolls back daemon releases for a reason no release caused: an update loop
/// over a missing file no update could have supplied.
///
/// So an override that will not load costs its slot and nothing else. The slot falls back to the
/// mode's default, the reason is recorded, and health reports **degraded** — the state for a
/// fault that belongs to the board rather than to the release being gated, where reverting the
/// daemon cannot help and only churns the boot counter.
///
/// **Only overrides.** An official path that will not load is a broken bundle and must reach the
/// health gate as unhealthy, which is exactly what rolls it back.
///
/// **And only faults that name a file.** A missing ONNX Runtime fails every path equally; taking
/// it as evidence against each override in turn would silently strip a board's whole
/// configuration because a dylib was not installed.
fn drop_unloadable_overrides(policy_params: &mut params::PolicyParams, errors: &mut SlotErrors) {
    drop_unloadable_overrides_with(policy_params, errors, duck_control::policy::validate)
}

/// [`drop_unloadable_overrides`] with the check injected, so its two rules can be tested on a
/// machine that has no ONNX Runtime — which is every machine this repository's tests run on, and
/// is itself one of the cases the rules are about.
fn drop_unloadable_overrides_with(
    policy_params: &mut params::PolicyParams,
    errors: &mut SlotErrors,
    validate: impl Fn(&std::path::Path) -> Result<(), PolicyError>,
) {
    if !policy_params.enabled {
        return;
    }
    // A `walk = "none"` already in the file, from a `policy load walk none` this daemon used to
    // accept. `resolved` now falls back rather than panicking, so the robot walks — but it walks
    // with something the config does not name, which is exactly what this reports.
    for slot in Slot::ALL {
        if policy_params.slot(slot).is_none() {
            continue;
        }
        let cfg = policy_params.resolved();
        // A slot set to the literal "none" resolves to nothing, and disabling a capability on
        // purpose is not a fault to fall back from.
        let Some(path) = cfg.slot(slot) else {
            continue;
        };
        let Err(e) = validate(path) else {
            continue;
        };
        if e.path().is_none() {
            tracing::error!(error = %e, "cannot validate policies at all; leaving config alone");
            return;
        }
        tracing::error!(
            slot = slot.as_str(),
            path = %path.display(),
            error = %e,
            "override will not load; falling back to this slot's default"
        );
        errors.set(slot, e.to_string());
        policy_params.set_slot(slot, None);
    }
}

struct RobotState {
    status_journal: PathBuf,
    status_imu_length: u8,
    pose_calibration: Option<imu_calibration::PoseCalibration>,
    /// Epoch for every timestamp below. `Instant` so the clock cannot go backwards.
    started: Instant,
    ticks: AtomicU64,
    /// Ticks whose work overran the period. Cumulative, for diagnosis; the rate check is
    /// what catches a sustained problem.
    missed: AtomicU64,
    /// Microseconds since `started` at the last completed tick.
    last_tick_us: AtomicU64,
    /// Achieved rate over the last window, as `f64::to_bits`. Zero until the first window
    /// closes, which is why the stall check carries the first second.
    achieved_hz: AtomicU64,
    /// Consecutive failed bus reads. Reset by any success.
    consecutive_errors: AtomicU32,
    /// Control-loop bus reads that exhausted the receive timeout, cumulative.
    bus_read_timeouts: AtomicU64,
    /// The HX transport reports receive timeouts as a distinct error type.
    bus_timeout_supported: bool,
    /// Failed attempts to bring the bus up — opening it, verifying its registers, or
    /// reading the startup pose. Non-zero means the loop is still waiting for a robot to
    /// answer and has never commanded anything.
    startup_bus_failures: AtomicU32,
    startup_servo_configuration_failures: ArcSwapOption<Vec<StartupServoFailure>>,
    /// Motor-bus voltage, EMA-smoothed, as `f64::to_bits`. Zero means *not read yet* — a
    /// distinction that has to survive to the wire, since zero volts and unknown volts look
    /// nothing alike to whoever is deciding whether to charge the robot.
    battery_v: AtomicU64,
    /// Hottest servo of the last thermal sample: temperature as `f64::to_bits`, and which
    /// joint it was. Zero means *not read yet*, same as the battery.
    motor_max_c: AtomicU64,
    motor_mean_c: AtomicU64,
    /// Index into [`duck_control::JOINT_NAMES`] of the hottest joint.
    motor_hottest: AtomicU32,
    /// Hottest board thermal zone, as `f64::to_bits`. Zero means no reading — off Linux, or a
    /// kernel with no thermal sysfs ([`soc`]).
    cpu_temp_c: AtomicU64,
    /// What the board's clock is capped at, and how far the thermal governor wound it down.
    ///
    /// A swap rather than four atomics, because the four numbers are one reading: a level
    /// paired with the ceiling from a different sample would describe a board that never
    /// existed. `None` until the first sample, and forever on a board with no `cpufreq` sysfs.
    cpu_throttle: ArcSwapOption<proto::CpuThrottle>,
    mouth_sound_enabled: AtomicBool,
    /// Mirrors of the bus's own IMU diagnostics, refreshed with the thermal sample. Held here
    /// so the IPC side can report them without touching the loop's IO.
    imu_stale_blocks: AtomicU64,
    /// Stale reads in a row as of the last sample. Sampled rather than watched, which is fine
    /// for the fault it describes: a board that has stopped refreshing keeps repeating, so its
    /// run is still growing whenever the next sample lands.
    imu_stale_run: AtomicU64,
    imu_ready: AtomicBool,
    /// A shutdown was requested; startup must not continue while clients close.
    stopping: AtomicBool,
    shutdown: AtomicBool,
    release_torque_on_exit: AtomicBool,
    exit_sit: AtomicU8,
    exit_sit_succeeded: AtomicBool,
    exit_release_verified: AtomicBool,
    exit_head_timing: ExitHeadTiming,
    low_voltage: low_voltage::LowVoltage,
    status_bus_fault: AtomicBool,
    /// A startup recovery released the joints; the control loop must not
    /// send targets until the operator explicitly starts or initializes it.
    startup_release_verified: AtomicBool,
    low_voltage_lock: low_voltage_lock::LowVoltageLock,
    startup_voltage_ready: AtomicBool,
    startup_read_only: AtomicBool,
    human_control: human_control::Arbiter,
    emergency_relax: emergency_relax::EmergencyRelax,
    human_action_busy: AtomicBool,
    human_other_action_busy: AtomicBool,
    human_motion_busy: AtomicBool,
    active_skill: ArcSwapOption<String>,
    last_skill_timeout: ArcSwapOption<proto::SkillTimeout>,
    precision_policy: std::sync::Mutex<Option<PrecisionPolicy>>,
    precision_ready: AtomicBool,
    /// Fan-out for `robot.state`. Bounded and lossy by design — see [`STATE_BUFFER`].
    state_tx: tokio::sync::broadcast::Sender<proto::RobotState>,
    /// Independent per-device results; a slow subscriber cannot stall the bus owner.
    devices_tx: tokio::sync::broadcast::Sender<proto::DeviceObservation>,
    /// Last guarded, individually sampled servo states used to admit partial maintenance.
    partial_maintenance_observation: std::sync::Mutex<Option<proto::DeviceObservation>>,
    device_configuration_generation: AtomicU64,
    subscriptions: Arc<telemetry::Subscriptions>,
    device_subscriptions: Arc<telemetry::Subscriptions>,
    /// What `btd` should be advertising, published when it changes.
    ///
    /// A broadcast channel like the state stream, and for the same reason: `btd` subscribes, and a
    /// `robotd` with nobody subscribed must not be blocked by that. Small buffer — a subscriber
    /// that has fallen behind on beacons wants the newest one, not a backlog of beats that have
    /// already passed.
    chorale_tx: tokio::sync::broadcast::Sender<proto::ChoraleAdvertise>,
    /// Why the policy is not loaded, if it is not. Set once at startup; the loop keeps
    /// running and holds the pose, so a broken bundle is a rollback rather than a crash.
    policy_error: ArcSwapOption<String>,
    /// Why the last policy change failed, when it named no slot to blame it on.
    ///
    /// **Not [`RobotState::policy_error`]**, and the difference is the health verdict.
    /// That one is the release's own policy failing to load, which is unhealthy and rolls the
    /// release back. This is somebody's *change* failing — a reload after a skill was added, a
    /// whole-robot reset — where the robot is still running the policy it had, so it is degraded
    /// and a rollback would fix nothing.
    ///
    /// A change to one slot is reported on that slot instead, which says more.
    policy_change_error: ArcSwapOption<String>,
    /// Which policy files this process is running, as file names. `None` when the policy is
    /// disabled.
    ///
    /// From the params rather than from the loaded network, and therefore known before the
    /// control thread has finished loading anything — a client that subscribes during startup
    /// gets the answer rather than a race. What *failed* to load is `policy_error`; this is
    /// what was asked for, and the pair is what distinguishes "no policy wanted" from "the
    /// policy this release ships would not load".
    ///
    /// **One swap for the whole set rather than seven fields**, because a mode switch replaces
    /// all of them at once: a reader that caught `walk` from roller beside `stand` from walking
    /// would be told about a robot that does not exist. Swapped by the control loop, which is
    /// the only thing that loads a policy.
    policies: ArcSwap<PolicyNames>,
    /// The per-slot report `robot.policies` serves — paths, origins, and any slot whose override
    /// could not be loaded.
    ///
    /// Beside [`Self::policies`] rather than replacing it: that one is read every time a client
    /// asks which skills exist, on a hot path, and answers with file names alone. This is the
    /// whole picture and is read when somebody asks a question about policies, which is rare.
    /// Swapped together and by the same thread, so the two cannot disagree.
    policy_slots: ArcSwap<Vec<proto::PolicySlot>>,
    managed_models: managed_models::Shared,
    /// `[policy] enabled`. A startup constant — nothing switches it at runtime — and it is here
    /// because `robot.policies` has to distinguish a robot that is deliberately holding its pose
    /// from one whose slots merely look empty.
    policy_enabled: bool,
    /// The config file this robot was started with, for the two calls that write it.
    ///
    /// Everything else here reads config once at startup and keeps what it parsed. `pad.bind` has
    /// to write, because `padd` re-reads `[pad]` from the file every second — a binding held only
    /// in memory would be reverted before anybody let go of the phone.
    config_path: PathBuf,
    /// Whether this robot can make a sound at all: audio enabled, and a bank with wavs in
    /// it. Same job as the `policy_*` fields above — false is "this robot cannot do that",
    /// which is what lets `dispatch` refuse a `robot.sound` with a reason instead of
    /// accepting it into silence. Read once at startup, like the policies: the postinstall
    /// renders the bank and restarts robotd, so a bank cannot appear under a running one.
    has_voice: bool,
    /// Clips require the audio output even when the generated voice bank is absent.
    audio_enabled: bool,
    /// Print one-second timing summaries for the complete servo/IMU read and control tick.
    measure_servo_read_time: AtomicBool,
    /// Whether a theremin can be picked up: the params allow one, and the depth stream is
    /// actually delivering frames. Published by the loop rather than read once at startup,
    /// because unlike a voice bank the sensor comes and goes — `tofd` restarts, the ToF
    /// drops off the I²C bus — and an accepted theremin on a duck with no depth is a
    /// feature that silently does nothing.
    theremin_ready: AtomicBool,
    /// Whether `[theremin] enabled` allows an instrument at all, and there is a voice to play
    /// it in. Read once at startup, like the policies — and kept separate from
    /// [`State::theremin_ready`] so that the refusal can name the switch. Off is the default,
    /// so "the feature is not turned on" is the answer most of these refusals want, and
    /// sending that person to look at `tofd` wastes their evening.
    theremin_allowed: bool,
    /// Whether this robot's config allows it to sing with others. Read once at startup, like the
    /// policies: `[chorale] accept`, false by default.
    chorale_accepted: bool,
    /// `walk` or `roller`, as `robot.mode` reports it.
    ///
    /// Not constant any more: `robot.setMode` switches it while the robot runs, so this is stored
    /// rather than read from the params it started as. An `AtomicU8` over [`Mode`] because it is
    /// read on the IPC side and written by the control loop, and there is nothing to allocate.
    mode: AtomicU8,
    /// Published by the loop so the IPC side can answer without consulting it.
    fallen: AtomicBool,
    /// The policy is driving and has been asked for a non-zero velocity.
    moving: AtomicBool,
    /// The robot is ready to drive: holding its current pose, or homed when explicitly requested.
    ///
    /// False covers two different states on purpose — never brought up, and ramping — because a
    /// client can act on neither: the answer to both is "wait, or press Start". The journal
    /// distinguishes them.
    homed: AtomicBool,
    /// Whether initialization holds the current pose instead of ramping to HOME.
    init_torque_only: bool,

    /// Published by the bus owner after reading ID200's firmware information.
    imu_firmware_version: ArcSwapOption<String>,

    /// The duck is parked in its seat, latched there by `sit_toggle`.
    ///
    /// **Published because "stand up" means two different calls.** A duck on its feet stands with
    /// `robot.init`; a duck in the seat is *held* there by a latch this daemon drives, and `init`
    /// argues with it rather than winning. What ends a sit is `robot.do sit_toggle`, and a client
    /// cannot know which of the two to send without this — the playground sent `init` at a seated
    /// duck and watched the two fight.
    ///
    /// Not `busy`: a seated duck is parked, not travelling, which is the distinction
    /// [`control::Controller::busy`] already draws.
    sitting: AtomicBool,
    obstacle: Arc<obstacle::Shared>,

    period_us: u64,
    min_achieved_hz: f64,
    stall_periods: u32,
    max_consecutive_errors: u32,
}

impl RobotState {
    fn record_skill_timeout(&self, skill: String) {
        let sequence = self.last_skill_timeout.load().as_ref().map_or(1, |previous| previous.sequence.wrapping_add(1));
        self.last_skill_timeout.store(Some(Arc::new(proto::SkillTimeout { sequence, skill })));
    }

    fn publish_active_skill(&self, name: Option<&str>) {
        if self.active_skill.load().as_deref().map(String::as_str) != name {
            self.active_skill.store(name.map(|name| Arc::new(name.to_owned())));
        }
    }

    fn new(
        params: &Params,
        config_path: &std::path::Path,
        intents: Arc<Intents>,
    ) -> Self {
        Self {
            status_journal: PathBuf::new(),
            status_imu_length: 28,
            pose_calibration: None,
            started: Instant::now(),
            ticks: AtomicU64::new(0),
            missed: AtomicU64::new(0),
            last_tick_us: AtomicU64::new(0),
            achieved_hz: AtomicU64::new(0),
            consecutive_errors: AtomicU32::new(0),
            bus_read_timeouts: AtomicU64::new(0),
            bus_timeout_supported: params.bus.protocol == BusProtocol::Hx,
            startup_bus_failures: AtomicU32::new(0),
            startup_servo_configuration_failures: ArcSwapOption::empty(),
            battery_v: AtomicU64::new(0),
            motor_max_c: AtomicU64::new(0),
            motor_mean_c: AtomicU64::new(0),
            motor_hottest: AtomicU32::new(0),
            cpu_temp_c: AtomicU64::new(0),
            cpu_throttle: ArcSwapOption::empty(),
            mouth_sound_enabled: AtomicBool::new(true),
            imu_stale_blocks: AtomicU64::new(0),
            imu_stale_run: AtomicU64::new(0),
            imu_ready: AtomicBool::new(false),
            stopping: AtomicBool::new(false),
            shutdown: AtomicBool::new(false),
            release_torque_on_exit: AtomicBool::new(false),
            exit_sit: AtomicU8::new(0),
            exit_sit_succeeded: AtomicBool::new(false),
            exit_release_verified: AtomicBool::new(false),
            exit_head_timing: ExitHeadTiming::default(),
            low_voltage: low_voltage::LowVoltage::default(),
            status_bus_fault: AtomicBool::new(false),
            startup_release_verified: AtomicBool::new(false),
            low_voltage_lock: low_voltage_lock::LowVoltageLock::open(None).expect("in-memory interlock"),
            startup_voltage_ready: AtomicBool::new(true),
            startup_read_only: AtomicBool::new(false),
            human_control: human_control::Arbiter::new(intents),
            emergency_relax: emergency_relax::EmergencyRelax::default(),
            human_action_busy: AtomicBool::new(false),
            human_other_action_busy: AtomicBool::new(false),
            human_motion_busy: AtomicBool::new(false),
            active_skill: ArcSwapOption::empty(),
            last_skill_timeout: ArcSwapOption::empty(),
            precision_policy: std::sync::Mutex::new(None),
            precision_ready: AtomicBool::new(false),
            state_tx: tokio::sync::broadcast::Sender::new(STATE_BUFFER),
            devices_tx: tokio::sync::broadcast::Sender::new(STATE_BUFFER),
            partial_maintenance_observation: std::sync::Mutex::new(None),
            device_configuration_generation: AtomicU64::new(0),
            subscriptions: Arc::new(telemetry::Subscriptions::default()),
            device_subscriptions: Arc::new(telemetry::Subscriptions::default()),
            chorale_tx: tokio::sync::broadcast::Sender::new(8),
            policy_error: ArcSwapOption::empty(),
            policy_change_error: ArcSwapOption::empty(),
            policies: ArcSwap::from_pointee(PolicyNames::of(&params.policy.resolved())),
            policy_slots: ArcSwap::from_pointee(slot_report(
                &params.policy,
                &params.policy.resolved(),
                &SlotErrors::default(),
            )),
            managed_models: managed_models::Shared::default(),
            policy_enabled: params.policy.enabled,
            config_path: config_path.to_owned(),
            has_voice: params.audio.enabled && has_any_wav(&params.audio.bank),
            audio_enabled: params.audio.enabled,
            measure_servo_read_time: AtomicBool::new(false),
            theremin_ready: AtomicBool::new(false),
            theremin_allowed: params.theremin.enabled && params.audio.enabled,
            chorale_accepted: params.chorale.accept,
            mode: AtomicU8::new(mode_code(params.policy.mode)),
            fallen: AtomicBool::new(false),
            moving: AtomicBool::new(false),
            homed: AtomicBool::new(false),
            init_torque_only: true,
            imu_firmware_version: ArcSwapOption::empty(),
            sitting: AtomicBool::new(false),
            obstacle: Arc::new(obstacle::Shared::default()),
            period_us: params.period().as_micros() as u64,
            min_achieved_hz: params.update_gate.min_achieved_hz,
            stall_periods: params.update_gate.stall_periods,
            max_consecutive_errors: params.update_gate.max_consecutive_errors,
        }
    }

    fn health(&self) -> proto::HealthResult {
        // Everything the robot can say about itself, attached to every answer whatever the
        // verdict — and consulted by none of the checks below.
        //
        // Two separate jobs in one method, deliberately. `healthy`/`degraded` are the update
        // system's inputs and may only reflect what a *release* can be blamed for. The rest
        // is a description of the robot for whoever is looking at it, and it travels on the
        // same answer because a robot behaving oddly is asked exactly one question, once.
        let describe =
            |healthy: bool, degraded: bool, reason: Option<String>| proto::HealthResult {
                healthy,
                imu_firmware_version: self.imu_firmware_version.load_full().map(|version| (*version).clone()),
                degraded,
                reason,
                battery: self.battery(),
                motors: self.motor_thermal(),
                cpu_temp_c: {
                    // Zero is "never read", the same sentinel the battery uses: a board at
                    // 0 °C is a sensor that is not there, not a cold robot.
                    let c = f64::from_bits(self.cpu_temp_c.load(Ordering::Relaxed));
                    (c > 0.0).then_some(c)
                },
                cpu_throttle: self.cpu_throttle.load_full().map(|t| *t),
                control_loop: Some(self.loop_health()),
                bus: proto::BusHealth {
                    consecutive_errors: self.consecutive_errors.load(Ordering::Relaxed),
                    read_timeouts: self.bus_timeout_supported
                        .then(|| self.bus_read_timeouts.load(Ordering::Relaxed)),
                    startup_failures: self.startup_bus_failures.load(Ordering::Relaxed),
                },
                imu: Some(proto::ImuHealth {
                    ready: self.imu_ready.load(Ordering::Relaxed),
                    stale_blocks: self.imu_stale_blocks.load(Ordering::Relaxed),
                    consecutive_stale_blocks: self.imu_stale_run.load(Ordering::Relaxed),
                }),
            };

        let unhealthy = |reason: String| describe(false, false, Some(reason));
        // Not healthy, but not the release's fault either — see `HealthResult::degraded`.
        let degraded = |reason: String| describe(false, true, Some(reason));

        if self.status_bus_fault.load(Ordering::Acquire) {
            return degraded("设备状态回复未确认；串口已隔离，控制和低压监测暂停".into());
        }

        // "Starting" is not "started". The gate polls, so it will see the transition.
        if self.ticks.load(Ordering::Relaxed) == 0 {
            if let Some(failures) = self.startup_servo_configuration_failures.load_full() {
                return degraded(format!("舵机启动参数未确认：{}", startup_servo_failure_detail(&failures)));
            }
            // Distinguish "starting" from "cannot see a robot". Both mean no ticks, but only
            // one of them is going to resolve on its own, and the update system quotes this
            // string as the reason it rolled a release back — so it has to name the cause.
            let waiting = self.startup_bus_failures.load(Ordering::Relaxed);
            if waiting > 0 {
                // Degraded, not unhealthy: an unpowered bench board must not roll back every
                // release shipped to it. The bus not answering is the same before and after.
                return degraded(format!(
                    "motor bus startup checks failed after {waiting} attempts; see detailed log"
                ));
            }
            return unhealthy("control loop has not completed a cycle yet".into());
        }

        // A daemon that came up but cannot run its policy is not healthy, however well the
        // loop is ticking. This is what makes the updater roll back a release whose bundle
        // is wrong, instead of leaving a robot that holds a pose and never walks again.
        if let Some(reason) = self.policy_error.load_full() {
            return unhealthy(format!("policy unavailable: {reason}"));
        }

        // A slot running its default because an override would not load. Degraded rather than
        // unhealthy, and the distinction is load-bearing: this is a property of the *board* —
        // somebody's file is missing — so reverting the daemon cannot fix it, and gating on it
        // would make every subsequent release fail its health check over a stale config line.
        // The robot walks; it walks with the policy it shipped with, and says so.
        let fell_back = self.policy_slots.load();
        let mut fell_back = fell_back
            .iter()
            .filter_map(|slot| slot.error.as_ref().map(|e| format!("{}: {e}", slot.slot)))
            .peekable();
        if fell_back.peek().is_some() {
            return degraded(format!(
                "using the default policy where an override would not load — {}",
                fell_back.collect::<Vec<_>>().join("; ")
            ));
        }

        // A reload or a reset that would not build. Same verdict and the same reasoning as the
        // slots above: the robot kept the policy it had, so it walks, and no release the update
        // gate could revert to would change the outcome.
        if let Some(reason) = self.policy_change_error.load_full() {
            return degraded(format!("the last policy change did not take — {reason}"));
        }

        if matches!(self.human_control.lock().maintenance_phase(),
            Some(human_control::MaintenancePhase::Active | human_control::MaintenancePhase::Returning))
        {
            return degraded("motor bus is in maintenance".into());
        }

        let errors = self.consecutive_errors.load(Ordering::Relaxed);
        if errors >= self.max_consecutive_errors {
            return unhealthy(format!("{errors} consecutive bus read failures"));
        }

        // A wedged loop stops stamping. This is what turns a hung control thread into an
        // honest answer instead of a socket that keeps saying "healthy" forever.
        let now_us = self.started.elapsed().as_micros() as u64;
        let stale_us = now_us.saturating_sub(self.last_tick_us.load(Ordering::Relaxed));
        let stall_limit_us = self.period_us.saturating_mul(self.stall_periods as u64);
        if stale_us > stall_limit_us {
            return unhealthy(format!("control loop stalled for {} ms", stale_us / 1000));
        }

        let hz = f64::from_bits(self.achieved_hz.load(Ordering::Relaxed));
        if hz > 0.0 && hz < self.min_achieved_hz {
            return unhealthy(format!(
                "control loop at {hz:.1} Hz, below the {:.1} Hz floor",
                self.min_achieved_hz
            ));
        }

        describe(true, false, None)
    }

    /// The loop's own numbers, as the readout reports them.
    fn loop_health(&self) -> proto::LoopHealth {
        let hz = f64::from_bits(self.achieved_hz.load(Ordering::Relaxed));
        let now_us = self.started.elapsed().as_micros() as u64;
        proto::LoopHealth {
            target_hz: 1_000_000.0 / self.period_us as f64,
            // Zero is the "no window has closed yet" sentinel, not a measured rate.
            achieved_hz: (hz > 0.0).then_some(hz),
            ticks: self.ticks.load(Ordering::Relaxed),
            missed: self.missed.load(Ordering::Relaxed),
            last_tick_age_ms: now_us.saturating_sub(self.last_tick_us.load(Ordering::Relaxed))
                / 1000,
        }
    }

    /// The hottest servo of the last thermal sample, or `None` before the first one.
    fn motor_thermal(&self) -> Option<proto::MotorThermal> {
        let max_c = f64::from_bits(self.motor_max_c.load(Ordering::Relaxed));
        if max_c <= 0.0 {
            return None;
        }
        let hottest = self.motor_hottest.load(Ordering::Relaxed) as usize;
        Some(proto::MotorThermal {
            hottest: duck_control::JOINT_NAMES
                .get(hottest)
                .unwrap_or(&"unknown")
                .to_string(),
            max_c,
            mean_c: f64::from_bits(self.motor_mean_c.load(Ordering::Relaxed)),
        })
    }

    /// The last battery reading, mapped to a percentage — or `None` if there has not been
    /// one.
    ///
    /// Zero is the "never read" sentinel rather than a measurement: the atomic starts there,
    /// and a robot whose bus cannot answer never leaves it. Reporting that as `0.00 V, 0%`
    /// would put a flat-battery warning in front of anyone whose robot has been up for less
    /// than a second.
    fn battery(&self) -> Option<proto::Battery> {
        let volts = f64::from_bits(self.battery_v.load(Ordering::Relaxed));
        (volts > 0.0).then(|| proto::Battery {
            volts,
            percent: duck_control::battery_percent(volts),
        })
    }

    fn safe_to_restart(&self) -> proto::SafeToRestartResult {
        if self.human_control.lock().maintenance_phase().is_some() {
            return proto::SafeToRestartResult {
                safe: false,
                reason: Some("motor bus is in maintenance".into()),
            };
        }
        // Restarting motor control mid-stride is how a robot falls over
        // (`updater-design.md` §7.2). A robot that is merely standing, or already down, is
        // safe to interrupt — it is going nowhere either way.
        if self.moving.load(Ordering::Relaxed) && !self.fallen.load(Ordering::Relaxed) {
            return proto::SafeToRestartResult {
                safe: false,
                reason: Some("the robot is walking".into()),
            };
        }
        proto::SafeToRestartResult {
            safe: true,
            reason: None,
        }
    }
}

#[tokio::main]
async fn main() -> ExitCode {
    // Rust ignores SIGPIPE, which turns `robotd ... | head` into a panic.
    unsafe { libc::signal(libc::SIGPIPE, libc::SIG_DFL) };

    let mut args = Args::parse();
    let hold_current_pose_on_init = match args.hold_current_pose_on_init() {
        Ok(hold) => hold,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::FAILURE;
        }
    };

    let log_path = args.log_file.clone().unwrap_or_else(|| args.socket.with_extension("log"));
    let _log_guard = match log_output::init(&log_path) {
        Ok(guard) => guard,
        Err(error) => {
            eprintln!("cannot initialize logging at {}: {error}", log_path.display());
            return ExitCode::FAILURE;
        }
    };
    tracing::info!(path = %log_path.display(), "detailed log");

    // An offline model update also holds this lock while changing the managed paths.
    // Take it before reading either the params or the managed model manifest, so a
    // daemon cannot retain paths from before an update while starting afterwards.
    let instance_lock = match claim_lock(&args.socket) {
        Ok(lock) => lock,
        Err(e) => {
            tracing::error!(path = %args.socket.display(), error = %e, "cannot claim robot IPC socket");
            return ExitCode::FAILURE;
        }
    };

    let explicit = args.params.is_some();
    let params_path = args
        .params
        .clone()
        .unwrap_or_else(|| PathBuf::from(params::DEFAULT_PATH));
    let mut params = match Params::load(&params_path, explicit) {
        Ok(params) => params,
        Err(e) => {
            tracing::error!(error = %e, "bad params");
            return ExitCode::FAILURE;
        }
    };
    if let Some(port) = args.port.clone() {
        params.bus.port = port;
    }
    let physical_status_bus = !args.fake && args.sim.is_none() && params.bus.protocol == BusProtocol::Hx;
    let status_journal = status_bus::journal_path(&args.socket, physical_status_bus);
    // Do not clear an interrupted session until recovery has also verified that
    // the servos are released. The control thread retries this with IPC available.
    let status_fault_at_startup = physical_status_bus && match status_bus::pending(&status_journal) {
        Ok(pending) => pending,
        Err(error) => { tracing::error!(%error, "cannot inspect status journal"); true }
    };
    if args.no_policy {
        params.policy.enabled = false;
    }
    let tof_enabled = std::env::var_os("WONDERDUCK_UNOQ_TOF_ENABLED")
        .is_none_or(|value| value == "true");
    if !tof_enabled {
        tracing::info!("UNO Q ToF polling is disabled");
    }
    if args.motion_only || !tof_enabled {
        params.theremin.enabled = false;
    }
    if let Err(error) = args.policy_overrides.apply(&mut params.policy) {
        tracing::error!(%error, "invalid policy startup overrides");
        return ExitCode::FAILURE;
    }
    match managed_models::load(&params_path) {
        Ok(paths) => managed_models::apply_paths(&mut params.policy, &paths),
        Err(error) => {
            tracing::error!(%error, "cannot load managed model paths");
            return ExitCode::FAILURE;
        }
    }
    let imu_source = match args.selected_imu_source(params.bus.protocol) {
        Ok(source) => source,
        Err(error) => {
            tracing::error!(%error, "invalid IMU source");
            return ExitCode::FAILURE;
        }
    };
    if let Err(error) = args.validate_policy_gravity(params.bus.protocol, imu_source,
        std::env::var_os("ROBOT_FREEZE_POLICY_IMU").is_some_and(|value| value == "1")) {
        tracing::error!(%error, "invalid policy gravity source");
        return ExitCode::FAILURE;
    }
    if let Err(error) = args.configure_imu_pipeline(imu_source) {
        tracing::error!(%error, "invalid IMU pipeline options");
        return ExitCode::FAILURE;
    }
    if imu_source == ImuSource::Id200Host {
        tracing::info!(sample_hz = 100, fusion_tau_s = 0.3, gyro_ema_previous_weight = 0.5,
            calibration = "raw_no_bias_or_scale_or_pose_correction", extra_median = false,
            "host IMU gravity pipeline selected; saved calibration is not applied");
    }
    if !args.fake && args.sim.is_none() && params.bus.protocol == BusProtocol::Hx {
        tracing::info!(source = imu_source.as_str(), imu_last = args.imu_last,
            same_snapshot_timing = imu_source.timed(), "IMU source selected");
    }
    if let Err(error) = duck_control::servo_diagnostics::validate_ids(&args.servo_diagnostics_ids) {
        tracing::error!(%error, "invalid servo diagnostic IDs");
        return ExitCode::FAILURE;
    }
    if !args.servo_diagnostics_ids.is_empty() && !args.fake && (args.sim.is_some() || params.bus.protocol != BusProtocol::Hx) {
        tracing::error!("servo diagnostics require the HX backend");
        return ExitCode::FAILURE;
    }

    let low_voltage_marker = physical_status_bus.then(|| PathBuf::from(low_voltage_lock::PHYSICAL_MARKER));
    let low_voltage_lock = match low_voltage_lock::LowVoltageLock::open(low_voltage_marker) {
        Ok(lock) => lock,
        Err(error) => {
            tracing::error!(%error, "cannot arm the persistent low-voltage interlock");
            return ExitCode::FAILURE;
        }
    };

    if let Some(Command::Init { duration }) = args.command {
        if status_fault_at_startup {
            tracing::error!("status reply unconfirmed; standalone init refused");
            return ExitCode::FAILURE;
        }
        if low_voltage_lock.locked() || low_voltage_lock.requires_rearm() {
            tracing::error!("initialization refused while low-voltage recovery is pending");
            return ExitCode::FAILURE;
        }
        if args.with_gamepad {
            tracing::error!("--with-gamepad is only available when running the daemon");
            return ExitCode::FAILURE;
        }
        if args.with_camera {
            tracing::error!("--with-camera is only available when running the daemon");
            return ExitCode::FAILURE;
        }
        if args.record.is_some() {
            tracing::error!("--record collects daemon control ticks and cannot be used with init");
            return ExitCode::FAILURE;
        }
        // Keep ownership until initialization completes, so no other process joins the bus.
        let _instance_lock = instance_lock;
        duck_ipc_proto::log_startup_identity!("robotd");
        let result = run_init(&params, duration, imu_source, args.imu_last, args.use_accel_calibration, hold_current_pose_on_init);
        return result;
    }

    // QMI firmware owns one frozen bias for both fusion and policy angular rate.
    // Historical gyro files remain only as provenance for paired tilt alignment.
    let pose_calibration = if !args.fake && args.sim.is_none() && params.bus.protocol == BusProtocol::Hx {
        match imu_calibration::PoseCalibration::load(
            Path::new(imu_calibration::PAIR_PATH), Path::new(imu_calibration::ACTIVE_PATH),
            &params.bus.port, imu_source, args.use_imu_calibration,
        ) {
            Ok(calibration) => {
                if calibration.is_some() {
                    tracing::info!(source = imu_source.as_str(), path = imu_calibration::PAIR_PATH,
                        "paired IMU pose calibration loaded");
                }
                calibration
            }
            Err(error) => {
                tracing::error!(%error, "cannot load paired IMU calibration");
                return ExitCode::FAILURE;
            }
        }
    } else { None };

    let precision_policy = if args.gamepad_precision {
        let resolved = params.policy.resolved();
        let Some(directory) = resolved.walk.as_deref().and_then(Path::parent) else {
            tracing::error!("--gamepad-precision requires a configured walking policy");
            return ExitCode::FAILURE;
        };
        let student = directory.join("precision.onnx");
        match PrecisionPolicy::load(&student) {
            Ok(policy) => {
                tracing::info!(student = %student.display(), "precision policy loaded and validated");
                Some(policy)
            }
            Err(error) => {
                tracing::error!(student = %student.display(), %error, "cannot load precision policy");
                return ExitCode::FAILURE;
            }
        }
    } else {
        None
    };

    // Own the endpoint before publishing an identity or starting a thread that can touch
    // the motors. Keep the lock through control-thread shutdown and socket cleanup.
    let (_instance_lock, listener) = match claim_socket(&args.socket, instance_lock).await {
        Ok(owned) => owned,
        Err(e) => {
            tracing::error!(path = %args.socket.display(), error = %e, "cannot claim robot IPC socket");
            return ExitCode::FAILURE;
        }
    };
    duck_ipc_proto::log_startup_identity!("robotd");

    let mut gamepad = if args.with_gamepad {
        match gamepad_process::GamepadProcess::start(&args.socket, &params_path, &args.gamepad_limits, args.gamepad_precision) {
            Ok(client) => Some(client),
            Err(error) => {
                tracing::error!(%error, "cannot start gamepad control");
                drop(listener);
                let _ = std::fs::remove_file(&args.socket);
                return ExitCode::FAILURE;
            }
        }
    } else {
        None
    };

    let (max_linear, max_lateral, max_angular) = args.gamepad_limits.motion_caps();
    if args.gamepad_precision && (args.gamepad_start_delay_ms.is_some() || args.gamepad_short_duration_ms.is_some()) {
        tracing::warn!("--gamepad-precision uses its own timing; legacy gamepad timing options are inactive");
    }
    let gamepad_timing = if args.gamepad_precision {
        velocity_command::GamepadTiming::default()
    } else {
        velocity_command::GamepadTiming {
            start_delay: args.gamepad_start_delay_ms
                .map(Duration::from_millis)
                .unwrap_or(velocity_command::START_DELAY),
            short_duration: args.gamepad_short_duration_ms
                .map(Duration::from_millis)
                .unwrap_or(velocity_command::SHORT_DURATION),
        }
    };
    let intents = Arc::new(Intents::new_with_axis_limits(
        max_linear,
        max_lateral,
        args.gamepad_limits.backward_cap(),
        max_angular,
        gamepad_timing,
    ));

    let requires_voltage_recovery = physical_status_bus && low_voltage_lock.requires_rearm();
    let mut state = RobotState::new(&params, &params_path, Arc::clone(&intents));
    state.obstacle = Arc::new(obstacle::Shared::new(tof_enabled));
    state.status_bus_fault.store(status_fault_at_startup, Ordering::Release);
    state.low_voltage_lock = low_voltage_lock;
    state.low_voltage.use_prompt_marker = physical_status_bus;
    state.startup_voltage_ready.store(!requires_voltage_recovery, Ordering::Relaxed);
    state.startup_read_only.store(requires_voltage_recovery, Ordering::Relaxed);
    state.status_journal = status_journal.clone();
    state.status_imu_length = if args.record_extended_servo_feedback() { 12 } else { 28 };
    state.human_control.set_status_journal(status_journal);
    state.init_torque_only = hold_current_pose_on_init;
    *state.precision_policy.get_mut().expect("precision policy mutex is not poisoned") = precision_policy;
    state
        .measure_servo_read_time
        .store(args.measure_servo_read_time, Ordering::Relaxed);
    state.pose_calibration = pose_calibration.clone();
    state.exit_head_timing = ExitHeadTiming {
        first: Duration::from_millis(args.exit_head_30_ms),
        delay: Duration::from_millis(args.exit_head_31_ms.delay_ms),
        second: Duration::from_millis(args.exit_head_31_ms.duration_ms),
    };
    let state = Arc::new(state);

    if args.init_home {
        tracing::warn!("--init-home: initialization will ramp to HOME");
    } else {
        tracing::info!("initialization will enable torque at the current pose");
    }

    let (record_sender, recorder) = match args.record.as_deref() {
        Some(path) => match record::Recorder::start(
            path,
            record::Metadata::from_params(
                &params_path,
                &params,
                matches!(params.bus.protocol, BusProtocol::Hx)
                    && std::env::var_os("ROBOT_FREEZE_POLICY_IMU")
                        .is_some_and(|value| value == "1"),
            )
            .with_imu_source(imu_source)
            .with_reference_imu_source(args.reference_imu_source)
            .with_policy_gravity_from_reference(args.policy_overrides.policy_gravity_from_reference)
            .with_imu_last(args.imu_last)
            .with_immediate_stop(args.policy_overrides.immediate_stop)
            .with_person_follow_only(args.record_person_follow)
            .with_target_follow_only(args.record_target_follow)
            .with_extended_servo_feedback(args.record_extended_servo_feedback())
            .with_servo_diagnostics(&args.servo_diagnostics_ids)
            .with_accel_calibration(args.use_accel_calibration)
            .with_pose_calibration(pose_calibration)
            .with_imu_filter(args.policy_overrides.imu_filter.unwrap_or(true))
            .with_imu_gyro_filter(
                args.policy_overrides.imu_filter.unwrap_or(true)
                    && args.policy_overrides.imu_gyro_filter.unwrap_or(true),
            ),
        ) {
            Ok((sender, recorder)) => {
                tracing::info!(path = %recorder.path().display(), person_follow_only = args.record_person_follow,
                    target_follow_only = args.record_target_follow,
                    "comparison recording ready");
                (Some(sender.with_person_follow_only(args.record_person_follow)
                    .with_target_follow_only(args.record_target_follow)), Some(recorder))
            }
            Err(error) => {
                tracing::error!(path = %path.display(), error = %error, "cannot start comparison recording");
                drop(listener);
                let _ = std::fs::remove_file(&args.socket);
                return ExitCode::FAILURE;
            }
        },
        None => (None, None),
    };

    // The real thing. `setsid` detaches the command from this process's cgroup, so the
    // poweroff proceeds while systemd is busy killing robotd itself.
    let poweroff: PowerOff = Arc::new(|| {
        let result = std::process::Command::new("setsid")
            .args(["sh", "-c", "systemctl poweroff"])
            .spawn();
        if let Err(e) = result {
            tracing::error!(error = %e, "cannot power off");
        }
    });

    let control = match spawn_control_thread(
        &args,
        &params,
        Arc::clone(&state),
        Arc::clone(&intents),
        poweroff,
        record_sender,
    ) {
        Ok(handle) => handle,
        Err(e) => {
            tracing::error!(error = %e, "cannot start the control loop");
            if let Some(recorder) = recorder {
                match recorder.finish() {
                    Ok(summary) => tracing::info!(
                        path = %summary.path.display(),
                        samples_written = summary.samples_written,
                        samples_dropped = summary.samples_dropped,
                        "comparison recording saved"
                    ),
                    Err(error) => {
                        tracing::error!(error = %error, "comparison recording was not saved completely")
                    }
                }
            }
            return ExitCode::FAILURE;
        }
    };

    let (camera_enabled, sensor_enabled, web_enabled) = auxiliary_services(&args);
    let camera = camera_enabled.then(|| camera_process::CameraSupervisor::start(args.with_camera));
    let sensor = obstacle::start_sensor(sensor_enabled && tof_enabled, Arc::clone(&state.obstacle), obstacle::SensorConfig {
        hz: args.tof_hz, timing_budget_ms: args.tof_timing_budget_ms,
        signal_rate_limit: args.tof_signal_rate_limit,
    });
    let start_web = || {
        match web_process::WebProcess::start(&args.socket, &params_path, args.record.as_deref(),
            args.record_person_follow, args.record_target_follow) {
            Ok(web_process::WebStart::Started(client)) => Ok(Some(client)),
            Ok(web_process::WebStart::NotInstalled) => {
                tracing::info!("web control is not installed; autostart skipped");
                Ok(None)
            }
            Err(error) => Err(error),
        }
    };
    let start_studio = || studio_process::StudioProcess::start(&args.socket, &params.bus.port);
    let studio_enabled = web_enabled && params.bus.protocol == BusProtocol::Hx && imu_source == ImuSource::Id200;
    let mut web = None;
    let mut studio = None;
    let mut web_retry = web_enabled.then(ServiceRestart::new);
    let mut studio_retry = studio_enabled.then(ServiceRestart::new);
    let mut service_tick = tokio::time::interval(Duration::from_millis(250));
    service_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    let mut serving = Box::pin(serve(
        Arc::clone(&state),
        Arc::clone(&intents),
        listener,
        args.socket.clone(),
        gamepad.as_ref().map(|client| client.id()),
    ));
    let mut shutdown_signal = Box::pin(shutdown());
    let mut code = ExitCode::SUCCESS;
    let mut sit_on_exit = false;
    let mut serving_stopped = false;
    loop {
        tokio::select! {
            result = &mut serving => {
                serving_stopped = true;
                if let Err(e) = result {
                    tracing::error!(error = %e, "IPC server stopped");
                    code = ExitCode::FAILURE;
                }
                break;
            }
            status = gamepad_process::wait_for_exit(&mut gamepad) => {
                match status {
                    Ok(status) => tracing::error!(%status, "gamepad control exited; stopping robotd"),
                    Err(error) => tracing::error!(%error, "gamepad control monitoring failed; stopping robotd"),
                }
                state.release_torque_on_exit.store(true, Ordering::Relaxed);
                code = ExitCode::FAILURE;
                break;
            }
            status = web_process::wait_for_exit(&mut web) => {
                match status {
                    Ok(status) => tracing::error!(%status, "web control exited; robot control remains active"),
                    Err(error) => tracing::error!(%error, "web control monitoring failed; robot control remains active"),
                }
                drop(web.take());
                if let Some(retry) = &mut web_retry {
                    retry.exited(Instant::now());
                }
            }
            status = studio_process::wait_for_exit(&mut studio) => {
                match status {
                    Ok(status) => tracing::error!(%status, "Studio exited; robot control remains active"),
                    Err(error) => tracing::error!(%error, "Studio monitoring failed; robot control remains active"),
                }
                drop(studio.take());
                if let Some(retry) = &mut studio_retry {
                    retry.exited(Instant::now());
                }
            }
            _ = service_tick.tick(), if web_retry.is_some() || studio_retry.is_some() => {
                restart_auxiliary("web control", &mut web_retry, &mut web, start_web);
                restart_auxiliary("Studio", &mut studio_retry, &mut studio, start_studio);
            }
            _ = state.low_voltage.triggered.notified() => {
                tracing::warn!("continuous low servo voltage; sitting and exiting");
                sit_on_exit = true;
                break;
            }
            signal = &mut shutdown_signal => {
                let release_torque = signal.releases_torque(args.keep_torque_on_ctrl_c);
                tracing::info!(release_torque, "shutting down");
                if release_torque {
                    state.release_torque_on_exit.store(true, Ordering::Relaxed);
                    state.exit_sit.store(EXIT_SIT_REQUESTED, Ordering::Release);
                    sit_on_exit = true;
                }
                break;
            }
        }
    }
    // A simultaneous signal or child exit must not bypass an already-triggered low-voltage sit.
    if state.exit_sit.load(Ordering::Acquire) != 0
        && state.release_torque_on_exit.load(Ordering::Acquire)
    {
        sit_on_exit = true;
    }
    state.stopping.store(true, Ordering::Release);
    if let Some(studio) = studio {
        if serving_stopped {
            studio.stop().await;
        } else {
            let stop_studio = studio.stop();
            tokio::pin!(stop_studio);
            tokio::select! {
                () = &mut stop_studio => {}
                result = &mut serving => {
                    if let Err(error) = result {
                        tracing::error!(%error, "IPC server stopped during Studio shutdown");
                        code = ExitCode::FAILURE;
                    }
                    stop_studio.await;
                }
            }
        }
    }
    drop(serving);
    drop(shutdown_signal);

    // Stop this process's input client before shutting down the control loop.
    drop(gamepad);
    drop(web);

    if sit_on_exit {
        if !wait_for_exit_sit(&state, &intents, EXIT_SIT_TIMEOUT).await {
            tracing::warn!("exit sit timed out; releasing torque");
        }
    }

    // Ask the loop to stop and let it finish the tick it is in, rather than aborting
    // mid-transaction and leaving a half-written packet on the bus.
    state.shutdown.store(true, Ordering::Release);
    if control.join().is_err() {
        tracing::error!("control thread failed");
        code = ExitCode::FAILURE;
    }
    if state.low_voltage.status().triggered {
        if !state.low_voltage_lock.persistence_confirmed() {
            tracing::error!("low-voltage lock durability was not confirmed");
            code = ExitCode::FAILURE;
        }
        if !state.exit_sit_succeeded.load(Ordering::Acquire)
            || !state.exit_release_verified.load(Ordering::Acquire)
        {
            tracing::error!("low-voltage sit or torque release not fully verified");
            code = ExitCode::FAILURE;
        }
    }
    if let Some(sensor) = sensor {
        sensor.abort();
        let _ = sensor.await;
    }
    if let Some(supervisor) = camera {
        supervisor.stop().await;
    }
    if let Some(recorder) = recorder {
        match recorder.finish() {
            Ok(summary) => tracing::info!(
                path = %summary.path.display(),
                samples_written = summary.samples_written,
                samples_dropped = summary.samples_dropped,
                "comparison recording saved"
            ),
            Err(error) => {
                tracing::error!(error = %error, "comparison recording was not saved completely")
            }
        }
    }
    let _ = std::fs::remove_file(&args.socket);
    code
}

struct ServiceRestart {
    next_attempt: Instant,
    started_at: Option<Instant>,
    delay: Duration,
}

impl ServiceRestart {
    fn new() -> Self {
        Self { next_attempt: Instant::now(), started_at: None, delay: Duration::from_secs(1) }
    }

    fn exited(&mut self, now: Instant) {
        if self.started_at.take().is_some_and(|start| now.duration_since(start) >= Duration::from_secs(30)) {
            self.delay = Duration::from_secs(1);
        }
        self.next_attempt = now + self.delay;
        self.delay = (self.delay * 2).min(Duration::from_secs(30));
    }
}

#[cfg(test)]
#[test]
fn auxiliary_retry_caps_delay_and_resets_after_a_stable_run() {
    let mut retry = ServiceRestart::new();
    let now = Instant::now();
    for seconds in [1, 2, 4, 8, 16, 30, 30] {
        retry.exited(now);
        assert_eq!(retry.next_attempt, now + Duration::from_secs(seconds));
    }
    retry.started_at = Some(now);
    let later = now + Duration::from_secs(30);
    retry.exited(later);
    assert_eq!(retry.next_attempt, later + Duration::from_secs(1));
}

fn restart_auxiliary<T>(
    service: &str,
    restart: &mut Option<ServiceRestart>,
    process: &mut Option<T>,
    start: impl FnOnce() -> std::io::Result<Option<T>>,
) {
    let Some(retry) = restart.as_mut().filter(|retry| process.is_none() && Instant::now() >= retry.next_attempt) else {
        return;
    };
    match start() {
        Ok(Some(child)) => {
            retry.started_at = Some(Instant::now());
            *process = Some(child);
        }
        Ok(None) => *restart = None,
        Err(error) => {
            tracing::warn!(service, %error, retry_secs = retry.delay.as_secs(), "auxiliary service startup failed");
            retry.exited(Instant::now());
        }
    }
}

fn auxiliary_services(args: &Args) -> (bool, bool, bool) {
    if args.motion_only {
        return (false, false, false);
    }
    let sim = args.sim.is_some();
    (
        camera_process::enabled(args.fake, sim, args.with_camera, args.no_camera),
        !args.fake && !sim,
        web_process::enabled(args.fake, sim),
    )
}

/// Set and verify the measured goal before enabling torque in standalone init.
fn initialize_current_pose<T: RobotIo>(io: &mut T, gain: u16) -> duck_control::io::Result<()> {
    let positions = io.read()?.positions;
    if positions.iter().any(|value| !value.is_finite()) {
        return Err(duck_control::io::IoError::Bus(
            "measured position is not finite; cannot enable torque at current pose".into(),
        ));
    }
    io.prepare_torque_only(&positions)?;
    io.set_gain(gain)?;
    io.set_torque(true)
}

/// Enable torque at the current pose, or explicitly ramp to HOME.
#[cfg(target_os = "linux")]
fn run_init(params: &Params, duration: Duration, imu_source: ImuSource, imu_last: bool, use_accel_calibration: bool, hold_current_pose: bool) -> ExitCode {
    // HOME is needed only for the explicit ramp; resolve it before enabling torque.
    let home = if hold_current_pose {
        None
    } else {
        Some(if params.bus.protocol == BusProtocol::Hx {
            match try_controller(&params.policy.resolved(), false, BusProtocol::Hx, None) {
                Ok(Some(controller)) => controller.home_target(),
                Ok(None) => {
                    tracing::error!("cannot initialize custom joints without a policy HOME");
                    return ExitCode::FAILURE;
                }
                Err(error) => {
                    tracing::error!(%error, "cannot resolve joint HOME");
                    return ExitCode::FAILURE;
                }
            }
        } else {
            DEFAULT_POSITION
        })
    };
    // The same open as the daemon's, replacement adoption included: `init` is what someone
    // reaches for right after a motor swap, and it must not be the one path that refuses the
    // new servo.
    let mut calibration_restore = duck_control::hx::CalibrationRestoreAttempts::default();
    let Some(mut io) = open_bus(&params.bus, 0, imu_source, imu_last, use_accel_calibration, None, &mut calibration_restore) else {
        return ExitCode::FAILURE;
    };
    let Some(home) = home else {
        if let Err(error) = initialize_current_pose(&mut io, params.policy.gain) {
            tracing::error!(%error, "cannot initialize torque at current pose");
            return ExitCode::FAILURE;
        }
        tracing::warn!("holding current pose, torque enabled");
        return ExitCode::SUCCESS;
    };
    if let Err(e) = io.set_torque(true) {
        tracing::error!(error = %e, "cannot enable torque");
        return ExitCode::FAILURE;
    }
    // Before the ramp, not after: position_p_gain is a RAM register that survives this
    // process, so whatever was written last is what the robot stands up with. A previous fall
    // leaves `gain_limp` (50) there, and `init` would then take the robot to its home pose at
    // a third of the intended stiffness — soft enough to be a fall risk in the one command
    // whose whole job is establishing a known state.
    //
    // `robotd`'s control loop sets its own gain on the first tick, so this only governs the
    // ramp and the window before the daemon starts — which is exactly the window where the
    // robot is standing up unsupported.
    if let Err(e) = io.set_gain(params.policy.gain) {
        tracing::error!(error = %e, gain = params.policy.gain, "cannot set the position gain");
        return ExitCode::FAILURE;
    }
    if let Err(e) = io.interpolate_to(&home, duration, Duration::from_millis(20)) {
        tracing::error!(error = %e, "interpolation to the home pose failed");
        return ExitCode::FAILURE;
    }
    tracing::warn!(?duration, "at home pose, torque enabled");
    ExitCode::SUCCESS
}

#[cfg(not(target_os = "linux"))]
fn run_init(_params: &Params, _duration: Duration, _imu_source: ImuSource, _imu_last: bool, _use_accel_calibration: bool, _hold_current_pose: bool) -> ExitCode {
    tracing::error!("init needs a real bus; this build is not on the robot");
    ExitCode::FAILURE
}

/// Start the control loop on its own OS thread, with its own current-thread runtime.
///
/// Its own thread because the bus read is *blocking* serial I/O — on a shared runtime it
/// would occupy a worker for the duration of every transaction. Its own runtime so IPC work
/// can never be scheduled in front of a tick. This mirrors the prototype, where the loop
/// likewise had a runtime to itself and everything else lived on threads.
fn spawn_control_thread(
    args: &Args,
    params: &Params,
    state: Arc<RobotState>,
    intents: Arc<Intents>,
    poweroff: PowerOff,
    recorder: Option<record::RecordSender>,
) -> std::io::Result<std::thread::JoinHandle<()>> {
    let period = params.period();
    let fake = args.fake;
    let sim = args.sim.clone();
    #[cfg(target_os = "linux")]
    let control_nice = args.control_nice;
    let bus = params.bus.clone();
    let imu_source = args.imu_source.unwrap_or_default();
    let imu_last = args.imu_last;
    let reference_imu_source = args.reference_imu_source;
    let record_extended_servo_feedback = args.record_extended_servo_feedback();
    let use_accel_calibration = args.use_accel_calibration;
    let servo_diagnostics_ids = args.servo_diagnostics_ids.clone();
    let params = params.clone();
    let policy_overrides = args.policy_overrides.clone();
    // So a reload can re-read `[policy]` without a restart. The path rather than the loaded
    // params, because the point is to pick up what has been written since.
    let params_path = args
        .params
        .clone()
        .unwrap_or_else(|| PathBuf::from(params::DEFAULT_PATH));

    std::thread::Builder::new()
        .name("control".into())
        .spawn(move || {
            #[cfg(target_os = "linux")]
            if !fake && sim.is_none() && bus.protocol == BusProtocol::Hx {
                configure_startup_servos(&bus, &state, imu_source, imu_last);
            }
            if !wait_for_startup_status_bus_recovery(&bus, &state, &intents, imu_source, imu_last, period) {
                return;
            }
            #[cfg(target_os = "linux")]
            if !fake && sim.is_none() {
                if let Err(error) = control_scheduling::configure(control_nice) {
                    tracing::error!(%error, control_nice,
                        "cannot configure physical control thread scheduling");
                    return;
                }
            }
            let runtime = match tokio::runtime::Builder::new_current_thread()
                .enable_time()
                .build()
            {
                Ok(runtime) => runtime,
                Err(e) => {
                    tracing::error!(error = %e, "cannot build the control runtime");
                    return;
                }
            };

            if fake {
                tracing::warn!("--fake: no bus, no robot");
                runtime.block_on(control_loop(
                    FakeIo::at(DEFAULT_POSITION),
                    state,
                    intents,
                    params,
                    params_path,
                    policy_overrides,
                    period,
                    poweroff,
                    recorder.clone(),
                    None,
                ));
                return;
            }

            // No `open_bus_waiting` equivalent, deliberately: `RemoteIo` connects lazily and
            // reconnects on every tick, so a duck started before its simulator simply reports
            // unhealthy until the simulator answers — which is what a robot with no power on the
            // bus does too, by a different route.
            if let Some(addr) = sim {
                tracing::warn!(%addr, "--sim: the body is in MuJoCo");
                runtime.block_on(control_loop(
                    duck_control::sim::RemoteIo::at(addr),
                    state,
                    intents,
                    params,
                    params_path,
                    policy_overrides,
                    period,
                    poweroff,
                    recorder.clone(),
                    None,
                ));
                return;
            }

            // Waiting, not one shot. `open_bus` verifies motor registers, which means it
            // talks to the servos — so on an unpowered board it fails and this used to fall
            // straight off the end of the thread. No control loop was ever created, and
            // because nothing had been *attempted* the health reason was the bland "control
            // loop has not completed a cycle yet", forever, whatever happened to the robot
            // afterwards. Retrying the read alone was not enough: execution never got there.
            runtime.block_on(async move {
                #[cfg(target_os = "linux")]
                while bus.protocol == BusProtocol::Hx && state.low_voltage_lock.requires_rearm() {
                    if wait_for_low_voltage_recovery(&bus, &state, &intents, imu_source, imu_last).await {
                        break;
                    }
                    if !state.status_bus_fault.load(Ordering::Acquire)
                        || !wait_for_startup_status_bus_recovery(&bus, &state, &intents, imu_source, imu_last, period)
                    {
                        return;
                    }
                }
                if let Some(mut io) = open_bus_waiting(&bus, &state, &intents, imu_source, imu_last, use_accel_calibration).await {
                    #[cfg(target_os = "linux")]
                    if state.stopping.load(Ordering::Acquire) {
                        finish_startup_bus(Some(io), &bus, &state, imu_source, imu_last);
                        return;
                    }
                    #[cfg(target_os = "linux")]
                    if let BusIo::Hx(hx) = &mut io {
                        hx.set_record_feedback(record_extended_servo_feedback);
                        if let Some(source) = reference_imu_source {
                            if let Err(error) = hx.set_reference_imu_source(source) {
                                tracing::error!(%error, "cannot configure reference IMU");
                                return;
                            }
                        }
                        if let Err(error) = hx.prepare_imu_reference() {
                            tracing::error!(%error, "cannot prepare selected recording reference");
                            return;
                        }
                        if let Err(error) = hx.set_servo_diagnostics_ids(servo_diagnostics_ids) {
                            tracing::error!(%error, "cannot enable servo diagnostics");
                            return;
                        }
                    }
                    control_loop(
                        io,
                        state,
                        intents,
                        params,
                        params_path,
                        policy_overrides,
                        period,
                        poweroff,
                        recorder,
                        motion_cpu::MotionCpuFloor::configured(),
                    )
                    .await;
                }
            });
        })
}

#[derive(Clone)]
struct StartupServoFailure {
    id: u8,
    remaining: u8,
    reason: String,
}

#[cfg(target_os = "linux")]
fn configure_startup_servos(bus: &params::Bus, state: &RobotState, imu_source: ImuSource, imu_last: bool) {
    const ATTEMPTS: u8 = 3;
    let mut pending: Vec<_> = duck_control::model::JOINT_IDS.into_iter().map(|id| StartupServoFailure {
        id, remaining: ATTEMPTS, reason: String::new(),
    }).collect();
    let mut corrected = 0;
    for attempt in 1..=ATTEMPTS {
        if state.stopping.load(Ordering::Acquire) || state.shutdown.load(Ordering::Acquire) { return; }
        let opened = open_backend(bus, imu_source, imu_last).and_then(|mut io| {
            if attempt > 1 || state.status_bus_fault.load(Ordering::Acquire) {
                io.prepare_individual_device_observation()?;
            }
            Ok(io)
        });
        match opened {
            Ok(BusIo::Hx(mut io)) => {
                pending.retain_mut(|servo| {
                    match io.check_released_servo_at_startup(servo.id, low_voltage_lock::RECOVER_VOLTAGE, &mut servo.remaining) {
                        Ok(count) => { corrected += count; false }
                        Err(error) => {
                            servo.reason = error.to_string();
                            tracing::warn!(id = servo.id, attempt, %error, "startup servo parameter check failed");
                            true
                        }
                    }
                });
                // Publish firmware information while the bus is still owned,
                // even if motion startup will wait for a low-voltage rearm.
                if pending.is_empty() || attempt == ATTEMPTS {
                    refresh_imu_firmware_version(&mut io, state, true);
                }
            }
            Ok(BusIo::Dynamixel(_)) => unreachable!("startup configuration requires HX"),
            Err(error) => {
                tracing::warn!(attempt, %error, "cannot open HX bus for startup parameters");
                for servo in &mut pending { servo.reason = error.to_string(); }
            }
        }
        if pending.is_empty() {
            tracing::info!(corrected, "HX startup configuration verified before motion startup");
            return;
        }
        if attempt < ATTEMPTS {
            std::thread::sleep(Duration::from_millis(200));
        }
    }
    let detail = startup_servo_failure_detail(&pending);
    tracing::warn!(corrected, %detail, "HX startup servo parameters unverified after three checks");
    state.startup_servo_configuration_failures.store(Some(Arc::new(pending)));
}

fn startup_servo_failure_detail(failures: &[StartupServoFailure]) -> String {
    failures.iter().map(|servo| format!("ID {}: {}", servo.id, servo.reason)).collect::<Vec<_>>().join(" | ")
}

#[cfg(target_os = "linux")]
fn check_startup_servo_recovery(hx: &mut duck_control::hx::HxIo, state: &RobotState) -> duck_control::io::Result<usize> {
    // The normal control read must see every required joint before doing
    // addressed readbacks of the joints that failed configuration.
    hx.check_control_stream()?;
    let Some(failures) = state.startup_servo_configuration_failures.load_full() else { return Ok(0); };
    let mut pending = (*failures).clone();
    let mut corrected = 0;
    pending.retain_mut(|servo| {
        match hx.check_released_servo_at_startup(servo.id, low_voltage_lock::RECOVER_VOLTAGE, &mut servo.remaining) {
            Ok(count) => { corrected += count; false }
            Err(error) => { servo.reason = error.to_string(); true }
        }
    });
    if !pending.is_empty() {
        let reason = startup_servo_failure_detail(&pending);
        state.startup_servo_configuration_failures.store(Some(Arc::new(pending)));
        return Err(duck_control::io::IoError::Bus(reason));
    }
    tracing::info!(corrected, "HX startup servo parameters verified after recovery");
    state.startup_servo_configuration_failures.store(None);
    Ok(corrected)
}

#[cfg(target_os = "linux")]
fn refresh_imu_firmware_version(io: &mut duck_control::hx::HxIo, state: &RobotState, log_unavailable: bool) {
    if state.imu_firmware_version.load().is_none() {
        match io.imu_firmware_version() {
            Ok(version) => state.imu_firmware_version.store(Some(Arc::new(version))),
            Err(error) if log_unavailable => tracing::info!(%error, "ID200 firmware version unavailable"),
            Err(_) => {}
        }
    }
}

fn wait_for_startup_status_bus_recovery(
    bus: &params::Bus, state: &RobotState, intents: &Intents,
    imu_source: ImuSource, imu_last: bool, period: Duration,
) -> bool {
    if !state.status_bus_fault.load(Ordering::Acquire) { return true; }
    // An interrupted status read might have left torque enabled.
    // Drain late replies, then individually release and verify the
    // servos before clearing the durable marker or starting control.
    intents.set_enabled(false);
    #[cfg(target_os = "linux")]
    let mut status_io: Option<BusIo> = None;
    #[cfg(target_os = "linux")]
    let mut status_only = ReadOnlyStatusSession::default();
    #[cfg(target_os = "linux")]
    let (mut maintenance_parked, mut maintenance_error) = (false, None);
    let mut retry_at = Instant::now();
    while !state.shutdown.load(Ordering::Acquire) && !state.stopping.load(Ordering::Acquire) {
        #[cfg(target_os = "linux")]
        if service_startup_maintenance(state, &mut maintenance_parked, &mut maintenance_error, |release| {
            if release {
                let io = status_io.as_mut().ok_or_else(|| {
                    duck_control::io::IoError::Bus("individual status bus is unavailable".into())
                })?;
                status_only.close(io, state).map_err(duck_control::io::IoError::Bus)?;
                verify_faulted_partial_maintenance(state).map_err(duck_control::io::IoError::Bus)?;
                io.maintenance_handoff(true)?;
            } else {
                status_io = None;
            }
            Ok(())
        }) {
            std::thread::sleep(period);
            continue;
        }
        #[cfg(target_os = "linux")]
        if service_startup_emergency(state, intents, || {
            if let Some(io) = status_io.as_mut() { status_only.close(io, state)?; }
            let mut io = match status_io.take() {
                Some(io) => io,
                None => open_backend(bus, imu_source, imu_last).map_err(|error| error.to_string())?,
            };
            io.recover_device_bus().map_err(|error| error.to_string())?;
            status_bus::complete_recovery(&state.status_journal).map_err(|error| error.to_string())?;
            state.status_bus_fault.store(false, Ordering::Release);
            state.homed.store(false, Ordering::Release);
            Ok(())
        }) {
            if !state.status_bus_fault.load(Ordering::Acquire) { break; }
            std::thread::sleep(period);
            continue;
        }
        #[cfg(target_os = "linux")]
        if status_only_requested(state) {
            if state.devices_tx.receiver_count() > 0 && Instant::now() >= retry_at {
                if status_io.is_none() {
                    match open_backend(bus, imu_source, imu_last) {
                        Ok(io) => status_io = Some(io),
                        Err(error) => {
                            tracing::warn!(%error, "cannot open startup status bus");
                            retry_at = Instant::now() + Duration::from_secs(2);
                        }
                    }
                }
                if let Some(io) = status_io.as_mut() {
                    if io.supports_individual_device_observation() {
                        status_only.sample(io, state, period, false);
                    } else {
                        publish_status_fault(state, None);
                    }
                }
            }
            std::thread::sleep(period);
            continue;
        }
        #[cfg(target_os = "linux")]
        if let Some(mut io) = status_io.take() {
            let _ = status_only.close(&mut io, state);
        }
        publish_status_fault(state, None);
        if Instant::now() < retry_at {
            std::thread::sleep(period);
            continue;
        }
        #[cfg(target_os = "linux")]
        let recovered = open_backend(bus, imu_source, imu_last)
            .and_then(|mut io| io.recover_device_bus())
            .map_err(|error| error.to_string())
            .and_then(|()| status_bus::complete_recovery(&state.status_journal)
                .map_err(|error| error.to_string()));
        #[cfg(not(target_os = "linux"))]
        let recovered: Result<(), String> = Err("status bus recovery is unavailable".into());
        match recovered {
            Ok(()) => {
                state.startup_release_verified.store(true, Ordering::Release);
                let mut guard = state.human_control.lock();
                if guard.emergency_release_completed() {
                    guard.finish_emergency_release();
                }
                state.status_bus_fault.store(false, Ordering::Release);
                state.homed.store(false, Ordering::Release);
                tracing::warn!("status bus recovered before startup; policy disabled and joints released");
                break;
            }
            Err(error) => tracing::warn!(%error, "startup status bus recovery failed; retrying"),
        }
        retry_at = Instant::now() + Duration::from_secs(2);
        std::thread::sleep(period);
    }
    !state.status_bus_fault.load(Ordering::Acquire)
}

#[cfg(target_os = "linux")]
fn minimum_recovery_voltage(blocks: &[Vec<u8>]) -> Option<f64> {
    if blocks.len() != duck_control::model::JOINT_IDS.len() { return None; }
    blocks.iter().map(|block| block.get(6).copied().filter(|raw| *raw > 0))
        .collect::<Option<Vec<_>>>()?
        .into_iter().min().map(|raw| f64::from(raw) / 10.0)
}

/// Validate power without sending target, torque or configuration writes.
/// One durable marker covers the entire preflight; an unanswered transaction
/// must never be retried or forgotten merely because the line is quiet.
#[cfg(target_os = "linux")]
async fn wait_for_low_voltage_recovery(bus: &params::Bus, state: &RobotState, intents: &Intents,
    imu_source: ImuSource, imu_last: bool) -> bool {
    let mut io = loop {
        if state.stopping.load(Ordering::Acquire) || state.shutdown.load(Ordering::Acquire) { return false; }
        match duck_control::hx::HxIo::open_read_only(&bus.port) {
            Ok(mut io) => {
                io.set_imu_source(imu_source);
                if let Err(error) = io.set_imu_last(imu_last) {
                    tracing::warn!(%error, "cannot configure read-only voltage preflight IMU");
                    return false;
                }
                break io;
            }
            Err(error) => {
                tracing::warn!(%error, "cannot open read-only voltage preflight bus");
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    };
    if let Err(error) = status_bus::begin(&state.status_journal, 254, 56, 12) {
        tracing::error!(%error, "cannot journal voltage preflight; motor bus remains locked");
        state.status_bus_fault.store(true, Ordering::Release);
        publish_status_fault(state, None);
        return false;
    }
    let mut admitted = false;
    let mut status_only = ReadOnlyStatusSession::default();
    let (mut maintenance_parked, mut maintenance_error) = (false, None);
    loop {
        if service_startup_maintenance(state, &mut maintenance_parked, &mut maintenance_error, |release| {
            if release {
                status_only.close(&mut io, state).map_err(duck_control::io::IoError::Bus)?;
                match io.read_maintenance_readiness(low_voltage_lock::RECOVER_VOLTAGE) {
                    Ok(None) => {}
                    Ok(Some(reason)) => return Err(duck_control::io::IoError::Bus(reason)),
                    Err(error) => {
                        state.status_bus_fault.store(true, Ordering::Release);
                        return Err(error);
                    }
                }
                io.maintenance_handoff(true)?;
                state.startup_release_verified.store(true, Ordering::Release);
                state.startup_voltage_ready.store(false, Ordering::Release);
                state.low_voltage_lock.observe_recovery(None, Instant::now());
                admitted = false;
            } else {
                // Maintenance can leave torque enabled. Restore a verified release
                // before the read-only preflight takes ownership again.
                let result = release_voltage_preflight_bus(bus, imu_source, imu_last)
                    .and_then(|()| io.maintenance_reclaim_for_shutdown());
                if let Err(error) = result {
                    tracing::warn!(%error, "cannot verify torque release after voltage-preflight maintenance");
                    state.status_bus_fault.store(true, Ordering::Release);
                }
            }
            Ok(())
        }) {
            if state.status_bus_fault.load(Ordering::Acquire) { return false; }
            if maintenance_parked && (state.stopping.load(Ordering::Acquire) || state.shutdown.load(Ordering::Acquire)) {
                // Studio still owns the serial port. Keep the recovery journal for restart.
                state.release_torque_on_exit.store(false, Ordering::Release);
                return false;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
            continue;
        }
        if state.stopping.load(Ordering::Acquire) || state.shutdown.load(Ordering::Acquire) {
            if let Err(error) = status_only.close(&mut io, state) {
                tracing::error!(%error, "cannot close voltage preflight status session");
                state.status_bus_fault.store(true, Ordering::Release);
                return false;
            }
            // The last request was completely answered. No next request was sent.
            if let Err(error) = status_bus::complete(&state.status_journal) {
                tracing::error!(%error, "cannot complete voltage preflight journal");
                state.status_bus_fault.store(true, Ordering::Release);
            }
            state.release_torque_on_exit.store(false, Ordering::Release);
            if state.exit_sit.load(Ordering::Acquire) != 0 {
                state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
            }
            return false;
        }
        if service_startup_emergency(state, intents, || {
            status_only.close(&mut io, state)?;
            state.startup_voltage_ready.store(false, Ordering::Release);
            state.low_voltage_lock.observe_recovery(None, Instant::now());
            admitted = false;
            let result = io.maintenance_handoff(true)
                .and_then(|()| release_voltage_preflight_bus(bus, imu_source, imu_last))
                .and_then(|()| io.maintenance_reclaim_for_shutdown());
            if result.is_err() {
                state.status_bus_fault.store(true, Ordering::Release);
            }
            result.map_err(|error| error.to_string())
        }) {
            if state.status_bus_fault.load(Ordering::Acquire) { return false; }
            continue;
        }
        if status_only_requested(state) {
            // Preflight is read-only and has its own durable marker. Suspend
            // its synchronous sample while the status page owns the bus.
            status_only.sample(&mut io, state, Duration::from_millis(20), true);
            if state.status_bus_fault.load(Ordering::Acquire) { return false; }
            tokio::time::sleep(Duration::from_millis(20)).await;
            continue;
        }
        if let Err(error) = status_only.close(&mut io, state) {
            tracing::error!(%error, "cannot close voltage preflight status session");
            state.status_bus_fault.store(true, Ordering::Release);
            return false;
        }
        let sample = match io.diagnostic_read_state_blocks(&duck_control::model::JOINT_IDS) {
            Ok(blocks) => minimum_recovery_voltage(&blocks),
            Err(error) => {
                // Do not clear the journal or send another instruction: the
                // unanswered reply can still arrive after an arbitrary delay.
                state.startup_voltage_ready.store(false, Ordering::Release);
                state.low_voltage.observe_recovery(None, Instant::now());
                state.status_bus_fault.store(true, Ordering::Release);
                publish_status_fault(state, None);
                tracing::error!(%error, "voltage preflight reply unconfirmed; motor bus remains locked");
                return false;
            }
        };
        let now = Instant::now();
        let voltage_recovered = sample.is_some_and(|v| v >= low_voltage_lock::RECOVER_VOLTAGE);
        if let Some(voltage) = sample { state.battery_v.store(voltage.to_bits(), Ordering::Relaxed); }
        state.low_voltage_lock.observe_recovery(sample, now);
        if state.low_voltage_lock.locked() && admitted {
            // Restart the continuous voltage check after a sampling gap.
            // A normal reading after a status-page pause keeps the recovered marker.
            let mut guard = state.human_control.lock();
            state.startup_voltage_ready.store(false, Ordering::Release);
            guard.emergency_stop();
            clear_human_intents(intents);
            intents.set_enabled(false);
            let _ = intents.take_power_request();
            if !voltage_recovered &&
                let Err(error) = state.low_voltage_lock.trip()
            {
                tracing::error!(%error, "cannot restore the persistent low-voltage lock");
                state.status_bus_fault.store(true, Ordering::Release);
                return false;
            }
            admitted = false;
        }
        if !admitted && !state.low_voltage_lock.locked() && state.low_voltage_lock.requires_rearm() {
            if let Err(error) = state.low_voltage_lock.mark_recovered() {
                state.low_voltage.observe_recovery(sample, now);
                tracing::error!(%error, "cannot persist recovered voltage; startup remains blocked");
                tokio::time::sleep(Duration::from_millis(100)).await;
                continue;
            }
            // No request received before this point may run afterwards.
            let mut guard = state.human_control.lock();
            guard.emergency_stop();
            clear_human_intents(intents);
            intents.cancel_motion_tracking();
            intents.cancel_maze();
            intents.set_enabled(false);
            let _ = intents.take_power_request();
            let _ = intents.take_reboot_motors();
            let _ = intents.take_mode_switch();
            admitted = true;
            state.startup_voltage_ready.store(true, Ordering::Release);
            tracing::warn!("voltage recovered; awaiting an explicit start");
        }
        if admitted && !state.low_voltage_lock.requires_rearm()
            && !voltage_recovered
        {
            let mut guard = state.human_control.lock();
            state.startup_voltage_ready.store(false, Ordering::Release);
            guard.emergency_stop();
            clear_human_intents(intents);
            intents.set_enabled(false);
            let _ = intents.take_power_request();
            if let Err(error) = state.low_voltage_lock.trip() {
                tracing::error!(%error, "cannot persist low-voltage interlock after rearm");
                state.status_bus_fault.store(true, Ordering::Release);
                publish_status_fault(state, None);
                return false;
            }
            admitted = false;
        }
        state.low_voltage.observe_recovery(sample, now);
        if admitted && !state.low_voltage_lock.requires_rearm()
            && voltage_recovered
        {
            if let Err(error) = status_bus::complete(&state.status_journal) {
                state.startup_voltage_ready.store(false, Ordering::Release);
                state.status_bus_fault.store(true, Ordering::Release);
                publish_status_fault(state, None);
                tracing::error!(%error, "cannot complete voltage preflight journal");
                return false;
            }
            state.startup_voltage_ready.store(true, Ordering::Release);
            state.startup_read_only.store(false, Ordering::Release);
            return true;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

#[cfg(target_os = "linux")]
fn release_voltage_preflight_bus(
    bus: &params::Bus, imu_source: ImuSource, imu_last: bool,
) -> duck_control::io::Result<()> {
    let mut io = open_backend(bus, imu_source, imu_last)?;
    io.recover_device_bus()
}

/// The real bus on the board; a fake elsewhere, so `open_bus_waiting` has one signature.
#[cfg(target_os = "linux")]
enum BusIo {
    Dynamixel(duck_control::bus::DynamixelIo),
    Hx(duck_control::hx::HxIo),
}

#[cfg(target_os = "linux")]
impl BusIo {
    fn check_registers(&mut self) -> duck_control::io::Result<usize> {
        match self {
            Self::Dynamixel(io) => io.check_registers(),
            Self::Hx(io) => io.check_registers(),
        }
    }

    /// The factory-default replacement protocol is a Dynamixel-only recovery path. HX devices
    /// use a different wire protocol and are still checked normally by `check_registers`.
    fn missing_servos(&mut self) -> duck_control::io::Result<Vec<u8>> {
        match self {
            Self::Dynamixel(io) => io.missing_servos(),
            Self::Hx(_) => Ok(Vec::new()),
        }
    }

    fn adopt_replacement(&mut self, id: u8) -> duck_control::io::Result<bool> {
        match self {
            Self::Dynamixel(io) => io.adopt_replacement(id),
            Self::Hx(_) => Ok(false),
        }
    }

    fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.interpolate_to(target, duration, step),
            Self::Hx(io) => io.interpolate_to(target, duration, step),
        }
    }

    fn set_torque(&mut self, on: bool) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.set_torque(on),
            Self::Hx(io) => io.set_torque(on),
        }
    }

    fn set_gain(&mut self, kp: u16) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.set_gain(kp),
            Self::Hx(io) => io.set_gain(kp),
        }
    }
}

#[cfg(target_os = "linux")]
impl RobotIo for BusIo {
    fn maintenance_handoff(&mut self, release: bool) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.maintenance_handoff(release),
            Self::Hx(io) => io.maintenance_handoff(release),
        }
    }

    fn maintenance_reclaim_for_shutdown(&mut self) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.maintenance_reclaim_for_shutdown(),
            Self::Hx(io) => io.maintenance_reclaim_for_shutdown(),
        }
    }

    fn measures_load(&self) -> bool {
        match self {
            Self::Dynamixel(io) => io.measures_load(),
            Self::Hx(io) => io.measures_load(),
        }
    }

    fn read(&mut self) -> duck_control::io::Result<duck_control::io::Sensors> {
        match self {
            Self::Dynamixel(io) => io.read(),
            Self::Hx(io) => io.read(),
        }
    }

    fn read_with_cycle_deadline(
        &mut self,
        deadline: Instant,
    ) -> duck_control::io::Result<duck_control::io::Sensors> {
        match self {
            Self::Dynamixel(io) => io.read(),
            Self::Hx(io) => io.read_with_cycle_deadline(deadline),
        }
    }

    fn sample_device_observation(&mut self, deadline: Instant) -> Option<proto::DeviceObservation> {
        match self {
            Self::Dynamixel(io) => io.sample_device_observation(deadline),
            Self::Hx(io) => io.sample_device_observation(deadline),
        }
    }

    fn sample_individual_device_observation(&mut self, deadline: Instant) -> duck_control::io::Result<Option<proto::DeviceObservation>> {
        match self {
            Self::Dynamixel(io) => io.sample_individual_device_observation(deadline),
            Self::Hx(io) => io.sample_individual_device_observation(deadline),
        }
    }

    fn set_individual_device_observation(&mut self, active: bool) {
        match self {
            Self::Dynamixel(io) => io.set_individual_device_observation(active),
            Self::Hx(io) => io.set_individual_device_observation(active),
        }
    }

    fn supports_individual_device_observation(&self) -> bool {
        match self {
            Self::Dynamixel(io) => io.supports_individual_device_observation(),
            Self::Hx(io) => io.supports_individual_device_observation(),
        }
    }

    fn prepare_individual_device_observation(&mut self) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.prepare_individual_device_observation(),
            Self::Hx(io) => io.prepare_individual_device_observation(),
        }
    }

    fn quarantine_device_bus(&mut self) {
        match self {
            Self::Dynamixel(io) => io.quarantine_device_bus(),
            Self::Hx(io) => io.quarantine_device_bus(),
        }
    }

    fn torque_release_verified(&self) -> bool {
        match self {
            Self::Dynamixel(io) => io.torque_release_verified(),
            Self::Hx(io) => io.torque_release_verified(),
        }
    }

    fn recover_device_bus(&mut self) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.recover_device_bus(),
            Self::Hx(io) => io.recover_device_bus(),
        }
    }

    fn begin_device_observation(&mut self, generation: u64) {
        match self {
            Self::Dynamixel(io) => io.begin_device_observation(generation),
            Self::Hx(io) => io.begin_device_observation(generation),
        }
    }

    fn end_device_observation(&mut self) {
        match self {
            Self::Dynamixel(io) => io.end_device_observation(),
            Self::Hx(io) => io.end_device_observation(),
        }
    }

    fn write(&mut self, targets: &duck_control::io::JointTargets) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.write(targets),
            Self::Hx(io) => io.write(targets),
        }
    }

    fn write_timed_positions(&mut self, positions: &[Option<f64>; NUM_JOINTS], duration_ms: u16) -> duck_control::io::Result<bool> {
        match self {
            Self::Dynamixel(io) => io.write_timed_positions(positions, duration_ms),
            Self::Hx(io) => io.write_timed_positions(positions, duration_ms),
        }
    }

    fn set_gain(&mut self, kp: u16) -> duck_control::io::Result<()> {
        BusIo::set_gain(self, kp)
    }

    fn set_position_p_gain(&mut self, kp: u8) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.set_position_p_gain(kp),
            Self::Hx(io) => io.set_position_p_gain(kp),
        }
    }

    fn set_torque(&mut self, on: bool) -> duck_control::io::Result<()> {
        BusIo::set_torque(self, on)
    }

    fn prepare_torque_only(&mut self, positions: &[f64; duck_control::NUM_JOINTS]) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.prepare_torque_only(positions),
            Self::Hx(io) => io.prepare_torque_only(positions),
        }
    }

    fn read_positions_raw(&mut self, ids: &[u8]) -> duck_control::io::Result<Vec<u16>> {
        match self {
            Self::Dynamixel(io) => RobotIo::read_positions_raw(io, ids),
            Self::Hx(io) => RobotIo::read_positions_raw(io, ids),
        }
    }

    fn write_positions_raw(&mut self, ids: &[u8], positions: &[u16]) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => RobotIo::write_positions_raw(io, ids, positions),
            Self::Hx(io) => RobotIo::write_positions_raw(io, ids, positions),
        }
    }

    fn write_limp_mouth(&mut self, opening: Option<f64>) -> duck_control::io::Result<()> {
        match self {
            Self::Hx(io) => io.write_limp_mouth(opening),
            Self::Dynamixel(_) => Ok(()),
        }
    }

    fn write_accessory_trigger(&mut self, pressure: u8) -> duck_control::io::Result<()> {
        match self {
            Self::Hx(io) => io.write_accessory_trigger(pressure),
            Self::Dynamixel(_) => Ok(()),
        }
    }

    fn reboot(&mut self, id: u8) -> duck_control::io::Result<()> {
        match self {
            Self::Dynamixel(io) => io.reboot(id),
            Self::Hx(io) => io.reboot(id),
        }
    }

    fn slow_sensors(&mut self) -> duck_control::io::Result<duck_control::io::SlowSensors> {
        match self {
            Self::Dynamixel(io) => io.slow_sensors(),
            Self::Hx(io) => io.slow_sensors(),
        }
    }

    fn poll_diagnostics(&mut self, budget: Duration) {
        match self {
            Self::Dynamixel(io) => io.poll_diagnostics(budget),
            Self::Hx(io) => io.poll_diagnostics(budget),
        }
    }

    fn record_servo_diagnostics(&mut self, budget: Duration) -> Option<duck_control::servo_diagnostics::Capture> {
        match self {
            Self::Dynamixel(io) => io.record_servo_diagnostics(budget),
            Self::Hx(io) => io.record_servo_diagnostics(budget),
        }
    }

    fn imu_stale(&self) -> duck_control::io::ImuStale {
        match self {
            Self::Dynamixel(io) => io.imu_stale(),
            Self::Hx(io) => io.imu_stale(),
        }
    }

    fn imu_ready(&self) -> bool {
        match self {
            Self::Dynamixel(io) => io.imu_ready(),
            Self::Hx(io) => io.imu_ready(),
        }
    }

    fn restart_imu_stream(&mut self) {
        match self {
            Self::Dynamixel(io) => io.restart_imu_stream(),
            Self::Hx(io) => io.restart_imu_stream(),
        }
    }
}

#[cfg(not(target_os = "linux"))]
type BusIo = FakeIo;

#[cfg(target_os = "linux")]
fn open_backend(bus: &params::Bus, imu_source: ImuSource, imu_last: bool) -> duck_control::io::Result<BusIo> {
    match bus.protocol {
        BusProtocol::Dynamixel => {
            duck_control::bus::DynamixelIo::open(&bus.port, bus.fast_sync_read)
                .map(BusIo::Dynamixel)
        }
        BusProtocol::Hx => {
            let mut io = duck_control::hx::HxIo::open(&bus.port)?;
            io.set_imu_source(imu_source);
            io.set_imu_last(imu_last)?;
            Ok(BusIo::Hx(io))
        }
    }
}

/// Open and verify the bus, waiting for a robot to answer.
///
/// Same reasoning as [`adopt_startup_pose`], one step earlier: an unpowered board cannot
/// pass `check_registers`, and that is a condition someone fixes by flipping a switch, not
/// one to abandon the control loop over.
///
/// Returns `None` only if shutdown is requested while waiting.
#[cfg(target_os = "linux")]
async fn open_bus_waiting(
    bus: &params::Bus,
    state: &RobotState,
    intents: &Intents,
    imu_source: ImuSource,
    imu_last: bool,
    use_accel_calibration: bool,
) -> Option<BusIo> {
    let mut attempt = 0u32;
    let mut io: Option<BusIo> = None;
    let mut next_check = Instant::now();
    let mut maintenance_parked = false;
    let mut maintenance_error = None;
    let mut observing = false;
    let mut device_schedule = telemetry::DeviceSchedule::default();
    let mut status_only = ReadOnlyStatusSession::default();
    let mut calibration_restore = duck_control::hx::CalibrationRestoreAttempts::default();
    let sample_period = Duration::from_micros(state.period_us);

    while !state.shutdown.load(Ordering::Relaxed) {
        let status_requested = status_only_requested(state);
        if !status_requested && let Some(opened) = io.as_mut()
            && let Err(error) = status_only.close(opened, state)
        {
            tracing::error!(%error, "cannot close startup status session");
            state.human_control.lock().finish_maintenance_handoff(
                human_control::MaintenancePhase::Failed, Some(error));
            tokio::time::sleep(sample_period).await;
            continue;
        }
        if service_startup_maintenance(state, &mut maintenance_parked, &mut maintenance_error, |release| {
            if release && state.status_bus_fault.load(Ordering::Acquire) {
                verify_faulted_partial_maintenance(state).map_err(duck_control::io::IoError::Bus)?;
                if io.is_none() {
                    return Err(duck_control::io::IoError::Bus("individual status bus is unavailable".into()));
                }
            }
            match io.as_mut() {
                Some(io) if !release && state.stopping.load(Ordering::Acquire) => {
                    io.maintenance_reclaim_for_shutdown()
                }
                Some(io) => io.maintenance_handoff(release),
                None => Ok(()),
            }
        }) {
            // Recheck registers and calibration after the bus returns from maintenance.
            next_check = Instant::now();
            tokio::time::sleep(sample_period).await;
            continue;
        }

        if service_startup_emergency(state, intents, || {
            if let Some(opened) = io.as_mut() {
                status_only.close(opened, state)?;
            }
            if state.status_bus_fault.load(Ordering::Acquire) {
                if io.is_none() {
                    io = Some(open_backend(bus, imu_source, imu_last).map_err(|error| error.to_string())?);
                }
                io.as_mut().expect("bus opened").recover_device_bus().map_err(|error| error.to_string())?;
                status_bus::complete_recovery(&state.status_journal).map_err(|error| error.to_string())?;
                state.status_bus_fault.store(false, Ordering::Release);
                state.homed.store(false, Ordering::Release);
                return Ok(());
            }
            if io.is_none() {
                io = Some(open_backend(bus, imu_source, imu_last).map_err(|error| error.to_string())?);
            }
            match io.as_mut().expect("bus opened") {
                BusIo::Hx(hx) => hx.set_torque(false).map_err(|error| error.to_string()),
                _ => Err("该舵机总线不支持卸力回读，结果未确认".into()),
            }
        }) {
            next_check = Instant::now();
            continue;
        }

        if state.stopping.load(Ordering::Acquire) {
            if let Some(opened) = io.as_mut() { let _ = status_only.close(opened, state); }
            finish_startup_bus(io.take(), bus, state, imu_source, imu_last);
            return None;
        }

        if status_requested {
            // Once the page guard is held, finish the current transaction and
            // stop every startup sync check, even before a subscriber arrives.
            intents.set_enabled(false);
            if state.devices_tx.receiver_count() > 0 && Instant::now() >= next_check {
                if io.is_none() {
                    match open_backend(bus, imu_source, imu_last) {
                        Ok(opened) => io = Some(opened),
                        Err(error) => {
                            tracing::warn!(%error, "cannot open startup status bus");
                            next_check = Instant::now() + STARTUP_RETRY_INTERVAL;
                        }
                    }
                }
                if let Some(opened) = io.as_mut() {
                    if opened.supports_individual_device_observation() {
                        status_only.sample(opened, state, sample_period, false);
                    } else {
                        publish_status_fault(state, None);
                    }
                }
            }
            tokio::time::sleep(sample_period).await;
            continue;
        }

        if state.status_bus_fault.load(Ordering::Acquire) {
            // Only a verified release may clear the durable fault. Do not run
            // startup register checks while recovering an ambiguous reply.
            intents.set_enabled(false);
            if Instant::now() >= next_check {
                let recovered = match io.take() {
                    Some(mut opened) => opened.recover_device_bus(),
                    None => open_backend(bus, imu_source, imu_last)
                        .and_then(|mut opened| opened.recover_device_bus()),
                }.map_err(|error| error.to_string())
                    .and_then(|()| status_bus::complete_recovery(&state.status_journal).map_err(|error| error.to_string()));
                match recovered {
                    Ok(()) => {
                        state.startup_release_verified.store(true, Ordering::Release);
                        state.status_bus_fault.store(false, Ordering::Release);
                        state.homed.store(false, Ordering::Release);
                        next_check = Instant::now();
                    }
                    Err(error) => {
                        tracing::warn!(%error, "startup status bus recovery failed; retrying");
                        next_check = Instant::now() + Duration::from_secs(2);
                    }
                }
            }
            publish_status_fault(state, None);
            tokio::time::sleep(sample_period).await;
            continue;
        }

        let emergency_active = state.human_control.lock().emergency_release_active();
        if emergency_active {
            // A failed emergency release stays latched until a verified retry.
            tokio::time::sleep(sample_period).await;
            continue;
        }

        let requested = state.devices_tx.receiver_count() > 0;
        if requested != observing {
            observing = requested;
            next_check = Instant::now();
        }
        if !requested {
            device_schedule.reset();
            // Without a device-status subscriber, retain the original open/check/retry path.
            io = None;
            if Instant::now() >= next_check {
                if let Some(opened) = open_bus(bus, attempt, imu_source, imu_last, use_accel_calibration, Some(state), &mut calibration_restore) {
                    if state.stopping.load(Ordering::Acquire) {
                        finish_startup_bus(Some(opened), bus, state, imu_source, imu_last);
                        return None;
                    }
                    state.startup_bus_failures.store(0, Ordering::Relaxed);
                    return Some(opened);
                }
                attempt += 1;
                state.startup_bus_failures.store(attempt, Ordering::Relaxed);
                next_check = Instant::now() + STARTUP_RETRY_INTERVAL;
            }
            tokio::time::sleep(sample_period.min(next_check.saturating_duration_since(Instant::now()))).await;
            continue;
        }

        if Instant::now() >= next_check {
            if io.is_none() {
                match open_backend(bus, imu_source, imu_last) {
                    Ok(opened) => io = Some(opened),
                    Err(error) => {
                        if attempt == 0 || attempt.is_multiple_of(STARTUP_READ_LOG_EVERY) {
                            tracing::error!(%error, port = %bus.port, attempt, "cannot open the bus; waiting");
                        }
                    }
                }
            }
            if let Some(opened) = io.as_mut()
                && check_open_bus(opened, bus, attempt, imu_source, use_accel_calibration, Some(state), &mut calibration_restore)
            {
                if state.stopping.load(Ordering::Acquire) {
                    finish_startup_bus(io.take(), bus, state, imu_source, imu_last);
                    return None;
                }
                state.startup_bus_failures.store(0, Ordering::Relaxed);
                return io;
            }
            attempt += 1;
            state.startup_bus_failures.store(attempt, Ordering::Relaxed);
            next_check = Instant::now() + STARTUP_RETRY_INTERVAL;
        }

        if let Some(opened) = io.as_mut() {
            // The startup checks still gate all motion. Use the already-owned bus to
            // publish whatever each device actually answered while a check is failing.
            let started = Instant::now();
            sample_waiting_bus(opened, state, &mut device_schedule, sample_period, started + sample_period);
            tokio::time::sleep(sample_period.saturating_sub(started.elapsed())).await;
        } else {
            tokio::time::sleep(STARTUP_RETRY_INTERVAL).await;
        }
    }

    if state.stopping.load(Ordering::Acquire) {
        if let Some(opened) = io.as_mut() { let _ = status_only.close(opened, state); }
        if maintenance_parked {
            if state.release_torque_on_exit.load(Ordering::Acquire) {
                tracing::error!("shutdown torque off not attempted: motor bus is still owned by maintenance");
            }
        } else {
            finish_startup_bus(io.take(), bus, state, imu_source, imu_last);
        }
    }
    None
}

#[cfg(target_os = "linux")]
fn finish_startup_bus(
    io: Option<BusIo>, bus: &params::Bus, state: &RobotState,
    imu_source: ImuSource, imu_last: bool,
) {
    let locked = state.status_bus_fault.load(Ordering::Acquire)
        || state.human_control.lock().emergency_release_active();
    if locked {
        tracing::error!("shutdown bus write skipped: motor status or torque release is unconfirmed");
    } else if state.release_torque_on_exit.load(Ordering::Acquire) {
        let result = match io {
            Some(mut io) => io.set_torque(false),
            None => open_backend(bus, imu_source, imu_last).and_then(|mut io| io.set_torque(false)),
        };
        if let Err(error) = result {
            tracing::error!(%error, "shutdown torque off failed during startup");
        }
    }
    if state.exit_sit.load(Ordering::Acquire) != 0 {
        state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
    }
}

#[cfg(not(target_os = "linux"))]
async fn open_bus_waiting(
    bus: &params::Bus,
    state: &RobotState,
    _intents: &Intents,
    imu_source: ImuSource,
    imu_last: bool,
    use_accel_calibration: bool,
) -> Option<BusIo> {
    let mut calibration_restore = duck_control::hx::CalibrationRestoreAttempts::default();
    if let Some(io) = open_bus(bus, 0, imu_source, imu_last, use_accel_calibration, Some(state), &mut calibration_restore) {
        return Some(io);
    }
    state.startup_bus_failures.store(1, Ordering::Relaxed);
    None
}

/// Open and verify the bus, or explain why not.
fn startup_check_reason(error: &duck_control::io::IoError) -> &'static str {
    use duck_control::io::IoError;
    use duck_control::read_diagnostic::ReadFailureReason;
    match error {
        IoError::ReadFailure(failure) => match &failure.reason {
            ReadFailureReason::Status { status, .. } if *status != 0 => "device_alarm",
            ReadFailureReason::Status { .. } => "reply_length_mismatch",
            ReadFailureReason::Timeout => "reply_timeout",
            ReadFailureReason::Response(_) => "serial_read_failure",
        },
        _ => "device_check_failure",
    }
}

#[cfg(target_os = "linux")]
fn open_bus(bus: &params::Bus, attempt: u32, imu_source: ImuSource, imu_last: bool, use_accel_calibration: bool, state: Option<&RobotState>, calibration_restore: &mut duck_control::hx::CalibrationRestoreAttempts) -> Option<BusIo> {
    // First attempt and every thirtieth — about one line per 30 s while waiting.
    let loud = attempt == 0 || attempt.is_multiple_of(STARTUP_READ_LOG_EVERY);
    let port = bus.port.as_str();

    let mut io = match open_backend(bus, imu_source, imu_last) {
        Ok(io) => io,
        Err(e) => {
            if loud {
                tracing::error!(error = %e, port, attempt, "cannot open the bus; waiting");
            }
            return None;
        }
    };
    check_open_bus(&mut io, bus, attempt, imu_source, use_accel_calibration, state, calibration_restore).then_some(io)
}

#[cfg(target_os = "linux")]
fn check_open_bus(io: &mut BusIo, bus: &params::Bus, attempt: u32, imu_source: ImuSource, use_accel_calibration: bool, state: Option<&RobotState>, calibration_restore: &mut duck_control::hx::CalibrationRestoreAttempts) -> bool {
    let loud = attempt == 0 || attempt.is_multiple_of(STARTUP_READ_LOG_EVERY);
    let port = bus.port.as_str();
    if let (Some(state), BusIo::Hx(hx)) = (state, &mut *io) {
        refresh_imu_firmware_version(hx, state, loud);
    }
    // Under the same `loud` rule as everything else here — a board waiting on servo power
    // retries this forever. Worth saying at all because the whole tick budget hangs off it,
    // and "turned off in robotd.toml" is otherwise indistinguishable from "this board is slow".
    if bus.protocol == BusProtocol::Dynamixel && !bus.fast_sync_read && loud {
        tracing::warn!("bus.fast_sync_read is off; every sync read is a plain one");
    }
    // A missing or faulty servo must not stop the settings of the other joints
    // from being checked. The result still gates motion until every check and
    // any replacement-device adoption succeed.
    let registers = match (&mut *io, state) {
        (BusIo::Hx(hx), Some(state)) => check_startup_servo_recovery(hx, state),
        _ => io.check_registers(),
    };
    let adoption_ok = adopt_missing_servo(io, loud);
    match registers {
        Ok(0) if loud => tracing::info!("motor registers already correct"),
        Ok(0) => {}
        Ok(n) => tracing::warn!(corrected = n, "motor registers corrected"),
        Err(e) => {
            if loud {
                tracing::error!(
                    error = %e,
                    attempt,
                    reason = startup_check_reason(&e),
                    "startup device check failed; waiting"
                );
            }
            return false;
        }
    }
    if !adoption_ok { return false; }
    if let BusIo::Hx(hx) = io
        && imu_source != ImuSource::Id200Host
    {
        if imu_source.shared_bus() {
            if let Err(error) = hx.restore_missing_persistent_calibration(port, use_accel_calibration, calibration_restore) {
                if loud { tracing::error!(%error, "cannot restore missing MCU calibration"); }
                return false;
            }
            if use_accel_calibration && let Err(error) = hx.load_fixed_accel_calibration(port) {
                if loud { tracing::error!(%error, "cannot load manually saved MCU accelerometer calibration"); }
                return false;
            }
            if let Err(error) = hx.load_fixed_gyro_calibration(port) {
                if loud { tracing::error!(%error, "cannot load manually saved MCU gyro calibration"); }
                return false;
            }
        }
        if let Err(error) = hx.report_gyro_bias() {
            if loud { tracing::error!(%error, "cannot confirm unified MCU gyro bias"); }
            return false;
        }
        if imu_source.shared_bus() {
            if let Err(error) = hx.configure_accel_calibration(use_accel_calibration) {
                tracing::error!(%error, "cannot configure MCU acceleration calibration");
                return false;
            }
        }
        if let Err(error) = hx.report_accel_calibration() {
            tracing::error!(%error, "cannot confirm MCU calibration and gyro wire contract");
            return false;
        }
    }
    true
}

/// The motor-swap path: if exactly one expected servo is silent and a factory-fresh one
/// answers instead, flash the new one as the missing joint.
///
/// A ping census of the fifteen expected IDs is all a complete bus pays for this. The
/// factory-defaults probe — which reopens the port at 57 600 baud — only runs once a single
/// servo is known to be missing, so an ordinary boot never scans for anything.
///
/// Returns whether the bus is worth checking further. `false` is "keep waiting": every servo
/// unpowered, a servo missing with nothing fresh to replace it, or two missing at once, which
/// cannot be told apart and is left to a human.
#[cfg(target_os = "linux")]
fn adopt_missing_servo(io: &mut BusIo, loud: bool) -> bool {
    use duck_control::bus::replacement_target;

    let missing = match io.missing_servos() {
        Ok(missing) => missing,
        Err(e) => {
            if loud {
                tracing::error!(error = %e, "cannot ping the servos; waiting");
            }
            return false;
        }
    };
    if missing.is_empty() {
        return true;
    }
    if missing.len() == duck_control::NUM_JOINTS {
        // Not a swap, just no power yet; `check_registers` below says so in the words people
        // already know.
        return true;
    }
    let Some(id) = replacement_target(&missing) else {
        if loud {
            tracing::error!(
                ?missing,
                "several servos are missing; a replacement can only be adopted one at a time, waiting"
            );
        }
        return false;
    };
    match io.adopt_replacement(id) {
        Ok(true) => true,
        Ok(false) => {
            if loud {
                tracing::error!(
                    id,
                    "servo missing and nothing answers at factory defaults (id 1, 57600 baud); \
                     is it plugged in? waiting"
                );
            }
            false
        }
        Err(e) => {
            if loud {
                tracing::error!(error = %e, id, "adopting the replacement servo failed; waiting");
            }
            false
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn open_bus(_bus: &params::Bus, _attempt: u32, _imu_source: ImuSource, _imu_last: bool, _use_accel_calibration: bool, _state: Option<&RobotState>, _calibration_restore: &mut duck_control::hx::CalibrationRestoreAttempts) -> Option<BusIo> {
    tracing::error!("no bus on this platform; use --fake");
    None
}

/// How long to wait between attempts to read the startup pose.
///
/// A second, not a control period: the read itself already carries a 30 ms bus timeout, and
/// a board waiting for someone to switch servo power on is not in a hurry. Fast enough that
/// powering the robot brings it up while your hand is still on the switch.
const STARTUP_RETRY_INTERVAL: Duration = Duration::from_secs(1);

/// Log one waiting line, then one every this many attempts — about one per 30 s.
const STARTUP_READ_LOG_EVERY: u32 = 30;

/// Adopt the pose the robot is already in, waiting for the bus to answer.
///
/// Never move on start: the servos hold their last commanded goal while this process is
/// dead, so a restart mid-update leaves a standing robot standing, with no gap. That
/// requires a successful read, so there is nothing to command until one lands.
///
/// This used to be a single read that logged and returned on failure — which killed the
/// control thread for the life of the process. `robotd` stayed up and kept answering the
/// socket, so a board booted before its servos were powered was permanently inert: powering
/// them changed nothing and only `systemctl restart robotd` helped, with no hint anywhere
/// that a restart was what was needed. Retrying makes the ordinary order of operations —
/// power the board, then power the servos — just work.
///
/// Read through `Safety` rather than the bus directly: safety owns the only `RobotIo`, so
/// this is the only way to reach it, and going through it keeps that invariant intact even
/// for the one read that happens before the loop starts.
///
/// Returns `None` only if shutdown is requested while waiting.
/// How far the robot has got towards being drivable.
///
/// This is what made pressing Start on a fresh robot do nothing visible: the policy ran, the loop
/// wrote positions, and the servos ignored them because they had no torque. Torque came from
/// `robotd init` — a separate subcommand that opens the motor bus itself, so it needs the daemon
/// stopped, and which appeared in no documentation.
///
/// The invariant the old design protected is narrower than "never touch torque", and it survives
/// intact: **nothing here runs because a process started.** A `robotd` restarted by an update finds
/// `Limp`, writes nothing new, and leaves a standing robot standing. Only an explicit
/// `robot.enable` moves it on, which is a human pressing Start.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Bringup {
    /// No torque asked for yet. The loop still reads, publishes and holds — it just cannot make the
    /// robot do anything, which is the correct state for a robot nobody has asked to move.
    Limp,
    /// Torque is on and the joints are ramping to the home pose. The policy does not drive yet: it
    /// would be stepping from wherever the robot was slumped, which is exactly the lurch the ramp
    /// exists to avoid.
    Homing {
        from: [f64; NUM_JOINTS],
        since: Instant,
    },
    /// At the home pose with torque on. The policy drives when enabled.
    Ready,
}

impl Bringup {
    /// The interpolated target for this tick, or `None` once the ramp is done.
    ///
    /// Linear, like `DynamixelIo::interpolate_to` which `robotd init` uses — same shape, except this
    /// one is computed per tick instead of blocking the thread, because here the loop is running.
    fn homing_target(&self, now: Instant, home: &[f64; NUM_JOINTS]) -> Option<[f64; NUM_JOINTS]> {
        let Bringup::Homing { from, since } = self else {
            return None;
        };
        let t = now.duration_since(*since).as_secs_f64() / HOME_RAMP.as_secs_f64();
        if t >= 1.0 {
            return None;
        }
        let mut target = [0.0; NUM_JOINTS];
        for (i, slot) in target.iter_mut().enumerate() {
            *slot = from[i] + (home[i] - from[i]) * t;
        }
        Some(target)
    }
}

/// The shutdown's ramp from the seat to the rest pose — see [`REST_RAMP`].
struct RestRamp {
    /// The last targets the sit commanded, so the ramp starts where the servos were being held
    /// rather than where gravity had them: no step on the first tick.
    from: [f64; NUM_JOINTS],
    /// The gain the sit was running at, kept through the ramp and the hold.
    gain: u16,
    since: Instant,
}

impl RestRamp {
    /// This tick's target: along the ramp, then the rest pose itself for the hold.
    fn target(&self, now: Instant) -> [f64; NUM_JOINTS] {
        let t = (now.duration_since(self.since).as_secs_f64() / REST_RAMP.as_secs_f64()).min(1.0);
        let mut target = [0.0; NUM_JOINTS];
        for (i, slot) in target.iter_mut().enumerate() {
            *slot = self.from[i] + (duck_control::REST_POSITION[i] - self.from[i]) * t;
        }
        target
    }

    /// Ramped and held: torque can go.
    fn done(&self, now: Instant) -> bool {
        now.duration_since(self.since) >= REST_RAMP + REST_HOLD
    }
}

enum RestMotion {
    Ramp(RestRamp),
    Action { playback: rest_action::Playback, gain: u16 },
}

impl RestMotion {
    fn targets(&self, now: Instant) -> ([f64; NUM_JOINTS], u16) {
        match self {
            Self::Ramp(ramp) => (ramp.target(now), ramp.gain),
            Self::Action { playback, gain } => (playback.target(), *gain),
        }
    }

    fn done(&self, now: Instant) -> bool {
        match self {
            Self::Ramp(ramp) => ramp.done(now),
            Self::Action { playback, .. } => playback.done(),
        }
    }
}

/// One low-pass step toward `target`.
///
/// A non-finite target is dropped, not folded in: `ema += α·(inf − ema)` is `inf` on this
/// tick and on every tick after, because nothing finite can climb back out of it. The wire
/// can produce one — JSON parses `1e400` as infinity — and the safety layer below refuses
/// non-finite joint targets rather than clamping them, so a single bad `robot.move` would
/// otherwise freeze the robot on its hold pose until reboot.
fn slew(ema: &mut f64, target: f64, alpha: f64) {
    if target.is_finite() {
        *ema += alpha * (target - *ema);
    }
}

// The existing velocity-idle threshold also terminates a released EMA at exact zero.
const TWIST_IDLE_EPSILON: f64 = 1e-3;

/// Keep the walk policy at zero briefly after an ordinary manual release has decelerated.
#[derive(Default)]
struct WalkStandHandoff {
    manual_motion_requested: bool,
    stopped_at: Option<Instant>,
}

impl WalkStandHandoff {
    fn update(&mut self, now: Instant, eligible: bool, walking: bool,
              requested: [f64; 3], smoothed: [f64; 3]) -> bool {
        if !eligible {
            *self = Self::default();
            return false;
        }
        if requested != [0.0; 3] {
            self.manual_motion_requested = true;
            self.stopped_at = None;
            return false;
        }
        if !self.manual_motion_requested || !walking {
            *self = Self::default();
            return false;
        }
        if smoothed != [0.0; 3] {
            self.stopped_at = None;
            return true;
        }
        let stopped_at = *self.stopped_at.get_or_insert(now);
        if now.saturating_duration_since(stopped_at) < WALK_STAND_SETTLE {
            true
        } else {
            *self = Self::default();
            false
        }
    }
}

/// Select the release behavior without changing acceleration or head/body smoothing.
fn smooth_twist(command: &mut [f64; 3], target: [f64; 3], alpha: f64, immediate_stop: bool) {
    for (ema, target) in command.iter_mut().zip(target) {
        if immediate_stop && target == 0.0 {
            *ema = 0.0;
        } else {
            slew(ema, target, alpha);
            if target == 0.0 && ema.abs() < TWIST_IDLE_EPSILON {
                *ema = 0.0;
            }
        }
    }
}

fn apply_tracking_deadzone(command: &mut [f64; 3], deadzone: [bool; 3], person_follow: bool, target_follow: bool) {
    for axis in 0..3 {
        // Normal coordinated-follow stops decelerate through the existing command filter.
        // Lateral motion is always disabled; safety and explicit stops clear the filter earlier.
        if deadzone[axis] && ((!person_follow && !target_follow) || (person_follow && axis == 1)) {
            command[axis] = 0.0;
        }
    }
}

fn coordinated_smoothed_twist(smoothed: [f64; 3], target: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|axis| {
        let value = smoothed[axis];
        let minimum = coordinated_tracking::MIN_BODY_SPEED[axis];
        if target[axis].abs() < minimum || value == 0.0 || value.abs() >= minimum {
            value
        } else if value.signum() == target[axis].signum() {
            minimum.copysign(value)
        } else {
            0.0
        }
    })
}

fn coordinated_limit_twist(mut twist: [f64; 3], limits: [Option<f64>; 2]) -> [f64; 3] {
    for (value, limit) in twist[..2].iter_mut().zip(limits) {
        if let Some(limit) = limit { *value = value.clamp(-limit, limit); }
    }
    twist
}

/// Require a released velocity command and bounded measured body rotation before
/// target arrival can accumulate its independent 500 ms visual confirmation.
fn target_follow_motion_settled(twist: [f64; 3], gyro: [f64; 3]) -> bool {
    twist.iter().all(|velocity| velocity.is_finite() && velocity.abs() < 0.005)
        && gyro.iter().all(|rate| rate.is_finite())
        && gyro[0].hypot(gyro[1]) < 0.2
        && gyro[2].abs() < 0.25
}

/// Keep walking during actual movement and head homing. Merely entering the visual
/// confirmation window must not switch a stopped robot back to the walking policy:
/// a target near the boundary can alternate between confirmation and alignment.
fn coordinated_tracking_force_walk(
    tracking_present: bool,
    tracking_active: bool,
    tracker: &coordinated_tracking::Tracker,
    target_keep_walking: bool,
    lateral_release: bool,
    twist: [f64; 3],
) -> bool {
    (tracking_present && tracker.homing()) || (target_keep_walking && !lateral_release)
        || (tracking_active && !lateral_release && twist.iter().any(|value| *value != 0.0))
}

/// Publish each device's own read time and validity, including when no control frame exists.
fn publish_device_observation(state: &RobotState, observation: Option<proto::DeviceObservation>) {
    let Some(mut observation) = observation else { return };
    if state.devices_tx.receiver_count() == 0 { return; }
    if let (Some(calibration), true, Some(quat), Some(gyro), Some(gravity)) = (
        &state.pose_calibration,
        observation.imu.valid,
        observation.imu.quaternion_wxyz,
        observation.imu.gyro_rad_s,
        observation.imu.gravity,
    ) {
        let mut sensors = duck_control::Sensors::default();
        sensors.imu.quat = quat;
        sensors.imu.gyro = gyro;
        sensors.imu.gyro_unfiltered = gyro;
        sensors.imu.gravity = gravity;
        let corrected = calibration.correct(sensors).imu;
        observation.imu.quaternion_wxyz = Some(corrected.quat);
        observation.imu.gyro_rad_s = Some(corrected.gyro);
        observation.imu.gravity = Some(corrected.gravity);
    }
    // Bounded broadcast only wakes subscribers; serialization and socket writes happen there.
    let _ = state.devices_tx.send(observation);
}

fn partial_maintenance_ready(observation: &proto::DeviceObservation, now_ns: u64) -> bool {
    const MAX_AGE_NS: u64 = 10_000_000_000;
    let expected = duck_control::model::JOINT_IDS;
    if observation.bus_error.is_some() || observation.servos.len() != expected.len() { return false; }
    let mut available = 0;
    let mut missing = 0;
    for id in expected {
        let mut matches = observation.servos.iter().filter(|servo| servo.id == id);
        let Some(servo) = matches.next() else { return false };
        if matches.next().is_some() { return false; }
        let Some(observed) = servo.observed_t_ns else { return false };
        if observed > now_ns || now_ns - observed > MAX_AGE_NS { return false; }
        if servo.valid {
            if servo.t_ns.is_none() || servo.error.is_some() || servo.torque != Some(0) { return false; }
            available += 1;
        } else if servo.error.as_deref() == Some("未连接") {
            missing += 1;
        } else { return false; }
    }
    available > 0 && missing > 0
}

fn verify_faulted_partial_maintenance(state: &RobotState) -> Result<(), String> {
    if state.low_voltage_lock.requires_rearm() && !state.low_voltage_lock.awaiting_start() {
        return Err("low_voltage_rearm_required".into());
    }
    let observation = state.partial_maintenance_observation.lock().expect("partial observation mutex");
    let ready = observation.as_ref().is_some_and(|observation| {
        partial_maintenance_ready(observation, proto::clock::monotonic_ns())
            && observation.servos.iter().filter(|servo| servo.valid).all(|servo| {
                servo.voltage.is_some_and(|voltage| (low_voltage_lock::RECOVER_VOLTAGE..=25.5).contains(&voltage))
            })
            && observation.optional_servos.iter().all(|servo| servo.valid && servo.torque == Some(0))
    });
    if ready { Ok(()) } else { Err("partial_servo_release_or_voltage_not_verified".into()) }
}

#[cfg(test)]
mod partial_maintenance_tests {
    use super::*;

    #[test]
    fn recovered_voltage_allows_maintenance_to_verify_bus_before_manual_start() {
        let intents = Arc::new(Intents::new());
        let state = RobotState::new(&Params::default(), Path::new("/test/robotd.toml"), intents);
        state.startup_read_only.store(true, Ordering::Relaxed);
        state.startup_voltage_ready.store(false, Ordering::Relaxed);
        state.low_voltage_lock.trip().unwrap();
        assert!(!maintenance_voltage_admitted(&state));

        let started = Instant::now();
        for step in 0..=20 {
            state.low_voltage_lock.observe_recovery(Some(7.8), started + Duration::from_millis(100 * step));
        }
        state.low_voltage_lock.mark_recovered().unwrap();
        assert!(maintenance_voltage_admitted(&state));
        assert!(state.low_voltage_lock.requires_rearm());

        state.low_voltage_lock.trip().unwrap();
        assert!(!maintenance_voltage_admitted(&state));
    }

    fn observations(now: u64) -> proto::DeviceObservation {
        let servos = duck_control::model::JOINT_IDS.into_iter().map(|id| {
            if id == 34 {
                proto::ServoObservation { id, observed_t_ns: Some(now), valid: false,
                    error: Some("未连接".into()), ..Default::default() }
            } else {
                proto::ServoObservation { id, t_ns: Some(now), observed_t_ns: Some(now),
                    valid: true, torque: Some(0), ..Default::default() }
            }
        }).collect();
        proto::DeviceObservation { servos, ..Default::default() }
    }

    #[test]
    fn partial_release_requires_each_reachable_servo_verified_and_a_clean_bus() {
        let now = 20_000_000_000;
        let mut state = observations(now);
        assert!(partial_maintenance_ready(&state, now));
        state.servos[0].torque = Some(1);
        assert!(!partial_maintenance_ready(&state, now));
        state.servos[0].torque = Some(0);
        state.servos[1].observed_t_ns = Some(now - 10_000_000_001);
        assert!(!partial_maintenance_ready(&state, now));
        state.servos[1].observed_t_ns = Some(now);
        state.bus_error = Some("unconfirmed frame".into());
        assert!(!partial_maintenance_ready(&state, now));
        state.bus_error = None;
        let missing = state.servos.iter_mut().find(|servo| servo.id == 34).unwrap();
        missing.error = Some("读取超时".into());
        assert!(!partial_maintenance_ready(&state, now));
        state.servos.iter_mut().find(|servo| servo.id == 34).unwrap().error = Some("invalid frame".into());
        assert!(!partial_maintenance_ready(&state, now));
    }

    #[test]
    fn unverified_release_keeps_movement_blocked_during_partial_maintenance() {
        let arbiter = human_control::Arbiter::new(Arc::new(Intents::new()));
        let session = human_control::Session::new();
        let mut guard = arbiter.lock();
        assert!(guard.begin_partial_maintenance(&session).is_err());
        guard.begin_emergency_release().unwrap();
        assert!(guard.begin_partial_maintenance(&session).is_err());
        guard.mark_emergency_release_unverified();
        guard.begin_partial_maintenance(&session).unwrap();
        assert!(guard.partial_maintenance());
        assert!(guard.emergency_release_active());
    }

    #[test]
    fn faulted_partial_maintenance_checks_voltage_on_each_reachable_servo() {
        let intents = Arc::new(Intents::new());
        let state = RobotState::new(&Params::default(), Path::new("/test/robotd.toml"), intents);
        let mut scan = observations(proto::clock::monotonic_ns());
        for servo in scan.servos.iter_mut().filter(|servo| servo.valid) {
            servo.voltage = Some(7.9);
        }
        *state.partial_maintenance_observation.lock().unwrap() = Some(scan.clone());
        assert!(verify_faulted_partial_maintenance(&state).is_ok());
        scan.servos[0].voltage = Some(7.1);
        *state.partial_maintenance_observation.lock().unwrap() = Some(scan);
        assert!(verify_faulted_partial_maintenance(&state).is_err());
    }

    #[test]
    fn failed_partial_handoff_and_disconnect_keep_motion_locked() {
        use proto::HumanControlSource;
        let arbiter = human_control::Arbiter::new(Arc::new(Intents::new()));
        let mut maintenance = human_control::Session::new();
        let mut gamepad = human_control::Session::new();
        let mut guard = arbiter.lock();
        assert!(guard.register(&mut gamepad, HumanControlSource::Gamepad).accepted);
        guard.begin_emergency_release().unwrap();
        guard.mark_emergency_release_unverified();
        guard.begin_partial_maintenance(&maintenance).unwrap();
        guard.finish_maintenance_handoff(human_control::MaintenancePhase::Failed, Some("handoff failed".into()));
        assert!(!guard.maintenance_result(&maintenance).accepted);
        assert!(!guard.acquire(&gamepad, HumanControlSource::Gamepad).accepted);
        guard.disconnect(&mut maintenance);
        assert_eq!(guard.maintenance_phase(), Some(human_control::MaintenancePhase::Returning));
        guard.finish_maintenance_return();
        assert_eq!(guard.maintenance_phase(), None);
        assert!(guard.emergency_release_active());
        assert!(!guard.acquire(&gamepad, HumanControlSource::Gamepad).accepted);
    }
}

fn publish_status_fault(state: &RobotState, cached: Option<proto::DeviceObservation>) {
    if state.devices_tx.receiver_count() == 0 { return; }
    let mut observation = cached.unwrap_or_else(|| proto::DeviceObservation {
        servos: duck_control::model::JOINT_IDS.into_iter().map(|id|
            proto::ServoObservation { id, ..Default::default() }).collect(),
        ..Default::default()
    });
    observation.bus_error = Some("设备状态回复未确认；串口已隔离，控制和低压监测暂停".into());
    let _ = state.devices_tx.send(observation);
}

/// Assemble display-only telemetry from individually timestamped status reads.
fn status_sensors(observation: &proto::DeviceObservation, now_ns: u64) -> Option<(duck_control::Sensors, u64)> {
    let imu = &observation.imu;
    if !imu.valid || observation.servos.len() != NUM_JOINTS { return None; }
    let mut oldest = imu.t_ns?;
    let gyro = imu.gyro_rad_s?;
    let quat = imu.quaternion_wxyz?;
    let gravity = imu.gravity?;
    if gyro.iter().chain(quat.iter()).chain(gravity.iter()).any(|value| !value.is_finite()) { return None; }
    let mut sensors = duck_control::Sensors::default();
    sensors.imu = duck_control::imu::ImuData { gyro, gyro_unfiltered: gyro, gravity, quat };
    let mut servo_status = [proto::ServoStatus::default(); NUM_JOINTS];
    let mut minimum: Option<f64> = None;
    for (joint, id) in duck_control::model::JOINT_IDS.iter().copied().enumerate() {
        let servo = observation.servos.iter().find(|servo| servo.id == id && servo.valid)?;
        oldest = oldest.min(servo.t_ns?);
        let raw = servo.raw?;
        let voltage = servo.voltage?;
        let temperature = servo.temperature?;
        let load = servo.load_percent?;
        let speed = servo.speed_rad_s?;
        if !voltage.is_finite() || voltage <= 0.0 || !temperature.is_finite()
            || !load.is_finite() || !speed.is_finite() { return None; }
        sensors.positions[joint] = duck_control::hx::decode_position(joint, &raw.to_le_bytes()).ok()?;
        sensors.velocities[joint] = speed;
        sensors.currents_ma[joint] = load.abs() * 10.0;
        servo_status[joint] = proto::ServoStatus {
            raw, load_percent: load, voltage, temperature,
            packet_status: servo.packet_status?, register_status: servo.register_status?, moving: servo.moving?,
        };
        minimum = Some(minimum.map_or(voltage, |value| value.min(voltage)));
    }
    if oldest == 0 || now_ns < oldest || now_ns - oldest > 500_000_000 { return None; }
    sensors.servo_status = Some(servo_status);
    sensors.min_servo_voltage = minimum;
    Some((sensors, oldest))
}

fn status_voltage(observation: &proto::DeviceObservation, now_ns: u64) -> Option<f64> {
    if observation.servos.len() != NUM_JOINTS { return None; }
    let mut minimum: Option<f64> = None;
    for id in duck_control::model::JOINT_IDS {
        let servo = observation.servos.iter().find(|servo| servo.id == id && servo.valid)?;
        let timestamp = servo.t_ns?;
        let voltage = servo.voltage?;
        if timestamp == 0 || now_ns < timestamp || now_ns - timestamp > 250_000_000
            || !voltage.is_finite() || voltage <= 0.0 { return None; }
        minimum = Some(minimum.map_or(voltage, |value| value.min(voltage)));
    }
    minimum
}

/// Poll only page-specific reads while the control loop is stopped.
fn sample_status_devices<T: RobotIo>(
    state: &RobotState,
    intents: &Intents,
    safety: &mut Safety<T>,
    schedule: &mut telemetry::DeviceSchedule,
    period: Duration,
    deadline: Instant,
) -> duck_control::io::Result<Option<proto::DeviceObservation>> {
    // Safety voltage sampling must continue even if a viewer asks for a slow refresh.
    let interval = state.device_subscriptions.period(period).min(Duration::from_millis(150));
    let mut latest = None;
    for _ in 0..2 {
        let guard = state.human_control.lock();
        let active = guard.status_blocked() && guard.maintenance_phase().is_none()
            && !guard.emergency_release_active();
        drop(guard);
        if !active || intents.enabled() || state.exit_sit.load(Ordering::Acquire) != 0
            || state.stopping.load(Ordering::Acquire)
            || !schedule.due(Instant::now(), proto::clock::monotonic_ns(), interval) { break; }
        match safety.sample_individual_device_observation(deadline)? {
            Some(observation) => {
                schedule.observe(&observation);
                latest = Some(observation);
            }
            None => break,
        }
    }
    Ok(latest)
}

fn sample_devices(
    state: &RobotState,
    schedule: &mut telemetry::DeviceSchedule,
    period: Duration,
    deadline: Instant,
    sample: impl FnOnce(Instant, Option<u64>) -> Option<proto::DeviceObservation>,
) {
    if state.devices_tx.receiver_count() == 0 {
        schedule.reset();
        return;
    }
    let interval = state.device_subscriptions.period(period);
    if schedule.due(Instant::now(), proto::clock::monotonic_ns(), interval)
        && let Some(observation) = {
            let generation = state.device_configuration_generation.load(Ordering::Acquire);
            sample(deadline, schedule.refresh_configuration(generation).then_some(generation))
        }
    {
        schedule.observe(&observation);
        publish_device_observation(state, Some(observation));
    }
}

fn sample_waiting_bus<T: RobotIo>(
    io: &mut T,
    state: &RobotState,
    schedule: &mut telemetry::DeviceSchedule,
    period: Duration,
    deadline: Instant,
) {
    if state.devices_tx.receiver_count() == 0 {
        io.end_device_observation();
    }
    sample_devices(state, schedule, period, deadline, |deadline, generation| {
        if let Some(generation) = generation { io.begin_device_observation(generation); }
        io.sample_device_observation(deadline)
    });
}

/// Access used by the status-only path while startup or emergency release is
/// blocked. The path never requests a control read or writes to an actuator.
trait StatusObservationBus {
    fn prepare_status(&mut self) -> duck_control::io::Result<()>;
    fn set_status_individual(&mut self, active: bool);
    fn begin_status(&mut self, generation: u64);
    fn sample_status(&mut self, deadline: Instant) -> duck_control::io::Result<Option<proto::DeviceObservation>>;
    fn quarantine_status(&mut self);
}

impl<T: RobotIo> StatusObservationBus for Safety<T> {
    fn prepare_status(&mut self) -> duck_control::io::Result<()> { self.prepare_individual_device_observation() }
    fn set_status_individual(&mut self, active: bool) { self.set_individual_device_observation(active); }
    fn begin_status(&mut self, generation: u64) { self.begin_device_observation(generation); }
    fn sample_status(&mut self, deadline: Instant) -> duck_control::io::Result<Option<proto::DeviceObservation>> {
        self.sample_individual_device_observation(deadline)
    }
    fn quarantine_status(&mut self) { self.quarantine_device_bus(); }
}

#[cfg(target_os = "linux")]
impl StatusObservationBus for BusIo {
    fn prepare_status(&mut self) -> duck_control::io::Result<()> { self.prepare_individual_device_observation() }
    fn set_status_individual(&mut self, active: bool) { self.set_individual_device_observation(active); }
    fn begin_status(&mut self, generation: u64) { self.begin_device_observation(generation); }
    fn sample_status(&mut self, deadline: Instant) -> duck_control::io::Result<Option<proto::DeviceObservation>> {
        self.sample_individual_device_observation(deadline)
    }
    fn quarantine_status(&mut self) { self.quarantine_device_bus(); }
}

#[cfg(target_os = "linux")]
impl StatusObservationBus for duck_control::hx::HxIo {
    fn prepare_status(&mut self) -> duck_control::io::Result<()> { self.prepare_individual_device_observation() }
    fn set_status_individual(&mut self, active: bool) { self.set_individual_device_observation(active); }
    fn begin_status(&mut self, generation: u64) { self.begin_device_observation(generation); }
    fn sample_status(&mut self, deadline: Instant) -> duck_control::io::Result<Option<proto::DeviceObservation>> {
        self.sample_individual_device_observation(deadline)
    }
    fn quarantine_status(&mut self) { self.quarantine_device_bus(); }
}

struct ReadOnlyStatusSession {
    active: bool,
    owns_marker: bool,
    retry_after: Instant,
    schedule: telemetry::DeviceSchedule,
}

impl Default for ReadOnlyStatusSession {
    fn default() -> Self {
        Self { active: false, owns_marker: false, retry_after: Instant::now(),
            schedule: telemetry::DeviceSchedule::default() }
    }
}

impl ReadOnlyStatusSession {
    /// `existing_marker_owned` is used only by voltage preflight, whose journal
    /// already protects its own read-only session and must not be removed here.
    fn sample<B: StatusObservationBus>(&mut self, io: &mut B, state: &RobotState,
        period: Duration, existing_marker_owned: bool)
    {
        if state.devices_tx.receiver_count() == 0 || Instant::now() < self.retry_after { return; }
        let result = self.sample_inner(io, state, period, existing_marker_owned);
        if let Err(error) = result {
            tracing::error!(%error, "individual status read unconfirmed; motor bus remains locked");
            *state.partial_maintenance_observation.lock().expect("partial observation mutex") = None;
            io.set_status_individual(false);
            io.quarantine_status();
            self.active = false;
            self.owns_marker = false;
            self.schedule.reset();
            self.retry_after = Instant::now() + Duration::from_secs(2);
            state.status_bus_fault.store(true, Ordering::Release);
            publish_status_fault(state, None);
        }
    }

    fn sample_inner<B: StatusObservationBus>(&mut self, io: &mut B, state: &RobotState,
        period: Duration, existing_marker_owned: bool) -> Result<(), String>
    {
        if !self.active {
            *state.partial_maintenance_observation.lock().expect("partial observation mutex") = None;
            let pending = status_bus::pending(&state.status_journal).map_err(|error| error.to_string())?;
            if pending {
                if !existing_marker_owned && !state.status_bus_fault.load(Ordering::Acquire) {
                    state.status_bus_fault.store(true, Ordering::Release);
                }
                self.owns_marker = false;
            } else {
                status_bus::begin_session(&state.status_journal).map_err(|error| error.to_string())?;
                self.owns_marker = !state.status_bus_fault.load(Ordering::Acquire);
            }
            // Wait for late replies while still holding the same exclusive port.
            // A pre-existing fault and its marker remain until torque is verified.
            io.prepare_status().map_err(|error| error.to_string())?;
            io.set_status_individual(true);
            io.begin_status(state.device_configuration_generation.load(Ordering::Acquire));
            self.active = true;
        }
        let interval = state.device_subscriptions.period(period).min(Duration::from_millis(150));
        if self.schedule.due(Instant::now(), proto::clock::monotonic_ns(), interval)
            && let Some(observation) = io.sample_status(Instant::now() + period.max(Duration::from_millis(3)))
                .map_err(|error| error.to_string())?
        {
            self.schedule.observe(&observation);
            *state.partial_maintenance_observation.lock().expect("partial observation mutex") = Some(observation.clone());
            if !state.status_bus_fault.load(Ordering::Acquire) || verify_faulted_partial_maintenance(state).is_ok() {
                publish_device_observation(state, Some(observation));
            } else {
                publish_status_fault(state, Some(observation));
            }
        }
        Ok(())
    }

    fn close<B: StatusObservationBus>(&mut self, io: &mut B, state: &RobotState) -> Result<(), String> {
        if !self.active { return Ok(()); }
        io.set_status_individual(false);
        self.active = false;
        self.schedule.reset();
        if self.owns_marker && !state.status_bus_fault.load(Ordering::Acquire) {
            if let Err(error) = status_bus::complete(&state.status_journal) {
                io.quarantine_status();
                state.status_bus_fault.store(true, Ordering::Release);
                self.owns_marker = false;
                return Err(error.to_string());
            }
        }
        self.owns_marker = false;
        Ok(())
    }
}

fn status_only_requested(state: &RobotState) -> bool {
    let guard = state.human_control.lock();
    (guard.status_blocked() || guard.maintenance_transition_blocked())
        && guard.maintenance_phase().is_none()
}

/// Honour maintenance handoff while startup is still waiting for a complete device set.
fn service_startup_maintenance(
    state: &RobotState,
    parked: &mut bool,
    last_error: &mut Option<String>,
    mut handoff: impl FnMut(bool) -> duck_control::io::Result<()>,
) -> bool {
    use human_control::MaintenancePhase;
    let phase = state.human_control.lock().maintenance_phase();
    let Some(phase) = phase else { return false };
    match phase {
        MaintenancePhase::Requested => {
            if state.human_control.lock().status_read_session_requested() {
                // A failed startup probe may itself have a late reply. Until startup
                // succeeds, it is not safe to begin independent status reads.
                state.human_control.lock().finish_maintenance_handoff(
                    MaintenancePhase::Failed, Some("启动设备检查尚未完成，无法安全开始状态采集".into()));
                return true;
            }
            let result = handoff(true);
            *parked = result.is_ok();
            let (next, error) = match result {
                Ok(()) => (MaintenancePhase::Active, None),
                Err(error) => (MaintenancePhase::Failed, Some(error.to_string())),
            };
            state.human_control.lock().finish_maintenance_handoff(next, error);
        }
        MaintenancePhase::Returning => {
            if state.human_control.lock().status_read_pending() {
                if last_error.is_none() {
                    tracing::error!("status reply unconfirmed; motor bus remains parked");
                    *last_error = Some("status read unconfirmed".into());
                }
                return true;
            }
            let result = if *parked { handoff(false) } else { Ok(()) };
            match result {
                Ok(()) => {
                    *parked = false;
                    *last_error = None;
                    state.human_control.lock().finish_maintenance_return();
                }
                Err(error) => {
                    let error = error.to_string();
                    if last_error.as_ref() != Some(&error) {
                        tracing::error!(%error, "cannot reclaim motor bus after startup maintenance");
                        *last_error = Some(error);
                    }
                }
            }
        }
        MaintenancePhase::Active | MaintenancePhase::Failed => {}
    }
    true
}

/// Startup checks can wait indefinitely; service torque release while they wait.
fn service_startup_emergency(
    state: &RobotState,
    intents: &Intents,
    release: impl FnOnce() -> Result<(), String>,
) -> bool {
    let waiters = {
        let guard = state.human_control.lock();
        if guard.maintenance_phase().is_some() { None } else { state.emergency_relax.take() }
    };
    let Some(waiters) = waiters else { return false };
    intents.cancel_pending_rest();
    intents.finish_rest(false);
    intents.set_enabled(false);
    let result = release();
    match &result {
        Ok(()) => {
            state.startup_release_verified.store(true, Ordering::Release);
            tracing::warn!("startup emergency torque release verified");
        }
        Err(error) => tracing::error!(%error, "startup emergency torque release not verified"),
    }
    let mut guard = state.human_control.lock();
    match state.emergency_relax.complete(waiters, result) {
        emergency_relax::CompletionState::Verified => guard.finish_emergency_release(),
        emergency_relax::CompletionState::Unverified => guard.mark_emergency_release_unverified(),
        emergency_relax::CompletionState::Pending => {},
    }
    true
}

async fn adopt_startup_pose<T: RobotIo>(
    safety: &mut Safety<T>,
    state: &RobotState,
    intents: &Intents,
    verified_torque_release: bool,
    period: Duration,
) -> Option<[f64; NUM_JOINTS]> {
    let mut attempt = 0u32;
    let mut last_logged = None;
    let mut maintenance_parked = false;
    let mut maintenance_error = None;
    let mut device_schedule = telemetry::DeviceSchedule::default();
    let mut status_only = ReadOnlyStatusSession::default();
    let mut fault_retry_at = Instant::now();

    while !state.shutdown.load(Ordering::Relaxed) {
        let status_requested = status_only_requested(state);
        if !status_requested && let Err(error) = status_only.close(safety, state) {
            tracing::error!(%error, "cannot close startup status session");
            state.human_control.lock().finish_maintenance_handoff(
                human_control::MaintenancePhase::Failed, Some(error));
            tokio::time::sleep(period).await;
            continue;
        }
        if service_startup_maintenance(state, &mut maintenance_parked, &mut maintenance_error, |release| {
            if !release && state.stopping.load(Ordering::Acquire) {
                safety.maintenance_reclaim_for_shutdown()
            } else {
                safety.maintenance_handoff(release)
            }
        }) {
            tokio::time::sleep(period).await;
            continue;
        }
        if service_startup_emergency(state, intents, || {
            status_only.close(safety, state)?;
            if state.status_bus_fault.load(Ordering::Acquire) {
                safety.recover_device_bus().map_err(|error| error.to_string())?;
                status_bus::complete_recovery(&state.status_journal).map_err(|error| error.to_string())?;
                state.status_bus_fault.store(false, Ordering::Release);
                state.homed.store(false, Ordering::Release);
                return Ok(());
            }
            if !verified_torque_release {
                return Err("该舵机总线不支持卸力回读，结果未确认".into());
            }
            safety.set_torque(false).map_err(|error| error.to_string())
        }) {
            continue;
        }
        if state.stopping.load(Ordering::Acquire) {
            let _ = status_only.close(safety, state);
            finish_startup_exit(safety, state).await;
            return None;
        }
        if status_requested {
            intents.set_enabled(false);
            if safety.supports_individual_device_observation() {
                status_only.sample(safety, state, period, false);
            } else {
                publish_status_fault(state, None);
            }
            tokio::time::sleep(period).await;
            continue;
        }
        if state.status_bus_fault.load(Ordering::Acquire) {
            intents.set_enabled(false);
            if Instant::now() >= fault_retry_at {
                let recovered = safety.recover_device_bus().map_err(|error| error.to_string())
                    .and_then(|()| status_bus::complete_recovery(&state.status_journal).map_err(|error| error.to_string()));
                match recovered {
                    Ok(()) => {
                        state.startup_release_verified.store(true, Ordering::Release);
                        state.status_bus_fault.store(false, Ordering::Release);
                        state.homed.store(false, Ordering::Release);
                    }
                    Err(error) => {
                        tracing::warn!(%error, "startup status bus recovery failed; retrying");
                        fault_retry_at = Instant::now() + Duration::from_secs(2);
                    }
                }
            }
            publish_status_fault(state, None);
            tokio::time::sleep(period).await;
            continue;
        }
        let emergency_active = state.human_control.lock().emergency_release_active();
        if emergency_active {
            tokio::time::sleep(period).await;
            continue;
        }
        let read = safety.read();
        match read {
            Ok(sensors) => {
                state.startup_bus_failures.store(0, Ordering::Relaxed);
                if attempt > 0 {
                    // Only worth a line if it had to wait — otherwise "control loop running"
                    // below already says everything, and this would just double it.
                    tracing::warn!(
                        attempt,
                        hz = 1.0 / period.as_secs_f64(),
                        "the motor bus answered; holding the pose found at startup"
                    );
                }
                return Some(sensors.positions);
            }
            Err(e) => {
                attempt += 1;
                // Published before sleeping, so `robot.health` can name the cause on the
                // very first failure rather than after the first log line.
                state.startup_bus_failures.store(attempt, Ordering::Relaxed);
                if last_logged.is_none_or(|at: Instant| at.elapsed() >= STARTUP_RETRY_INTERVAL * STARTUP_READ_LOG_EVERY) {
                    tracing::warn!(
                        error = %e,
                        attempt,
                        "no answer from the motor bus; waiting, not commanding anything"
                    );
                    last_logged = Some(Instant::now());
                }
                let retry_at = Instant::now() + STARTUP_RETRY_INTERVAL;
                while Instant::now() < retry_at && !state.shutdown.load(Ordering::Relaxed) {
                    if status_only_requested(state) || state.status_bus_fault.load(Ordering::Acquire)
                        || state.human_control.lock().emergency_release_active()
                    {
                        break;
                    }
                    if service_startup_maintenance(state, &mut maintenance_parked, &mut maintenance_error, |release| {
                        if !release && state.stopping.load(Ordering::Acquire) {
                            safety.maintenance_reclaim_for_shutdown()
                        } else {
                            safety.maintenance_handoff(release)
                        }
                    }) {
                        tokio::time::sleep(period).await;
                        continue;
                    }
                    if service_startup_emergency(state, intents, || {
                        if !verified_torque_release {
                            return Err("该舵机总线不支持卸力回读，结果未确认".into());
                        }
                        safety.set_torque(false).map_err(|error| error.to_string())
                    }) {
                        continue;
                    }
                    if state.stopping.load(Ordering::Acquire) {
                        finish_startup_exit(safety, state).await;
                        return None;
                    }
                    let sample_start = Instant::now();
                    if state.devices_tx.receiver_count() == 0 { safety.end_device_observation(); }
                    sample_devices(state, &mut device_schedule, period, sample_start + period,
                        |deadline, generation| {
                            if let Some(generation) = generation { safety.begin_device_observation(generation); }
                            safety.sample_device_observation(deadline)
                        });
                    tokio::time::sleep(period.saturating_sub(sample_start.elapsed())
                        .min(retry_at.saturating_duration_since(Instant::now()))).await;
                }
            }
        }
    }

    if state.stopping.load(Ordering::Acquire) {
        let _ = status_only.close(safety, state);
        if maintenance_parked {
            if state.release_torque_on_exit.load(Ordering::Acquire) {
                tracing::error!("shutdown torque off not attempted: motor bus is still owned by maintenance");
            }
        } else {
            finish_startup_exit(safety, state).await;
        }
    }
    None
}

async fn finish_startup_exit<T: RobotIo>(safety: &mut Safety<T>, state: &RobotState) {
    let locked = state.status_bus_fault.load(Ordering::Acquire)
        || state.human_control.lock().emergency_release_active();
    if locked {
        tracing::error!("shutdown bus write skipped: motor status or torque release is unconfirmed");
    } else if state.release_torque_on_exit.load(Ordering::Acquire) {
        let released = finish_exit(safety, false, state.exit_head_timing).await.is_ok();
        state.exit_release_verified.store(released && state.bus_timeout_supported, Ordering::Release);
    }
    if state.exit_sit.load(Ordering::Acquire) != 0 {
        state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
    }
}

/// The tick.
///
/// ```text
/// read → observe (fall) → gate (deadman) → policy → safety.apply
/// ```
///
/// Safety holds the only `RobotIo`, so everything above it can propose targets and nothing
/// above it can command a motor.
///
/// A policy that failed to load is survivable, and deliberately so: the loop keeps running
/// at rate, holds its pose, and `robot.health` says why. The updater then rolls the release
/// back. The alternative — refusing to start — becomes a crashloop under
/// `Restart=always` and reaches the health gate as `Unreachable`, which blames the wrong
/// thing in the journal.
/// Load one mode's policy bundle, reporting why not.
///
/// `Ok(None)` is "no policy was wanted", which is healthy; `Err` is "one was wanted and could not
/// be loaded", which is not. Collapsing those two would either make a bench robot look broken or
/// let a release with an unusable bundle pass the health gate.
///
/// Separate from [`build_controller`] because two callers want opposite things from a failure.
/// Startup and a mode switch want it recorded as *the* policy error, which is unhealthy and gets
/// the release rolled back. A `robot.loadPolicy` wants the reason handed back so it can keep the
/// controller it already had — a robot must not lose its gait because somebody tried a file.
/// A policy change between being asked for and being applied.
///
/// The networks load on a thread of their own. `Policy::load` validates and warms up every
/// session, which is most of a second on the board, and this loop cannot stop for that: the
/// swap used to happen only at the home pose, where a stalled command stream cost nothing, but
/// a change to a network that is not driving now happens wherever the robot is — mid-sit,
/// mid-stride — and a walking robot whose targets stop arriving for a second falls over.
struct PendingSwap {
    change: intents::PolicyChange,
    /// The config the change asks for, from the one that is running.
    candidate: params::PolicyParams,
    cfg: params::ResolvedPolicy,
    /// The network that was driving when the request landed is one this change replaces. The
    /// robot goes home first and the new controller starts fresh there, as every change used to.
    /// Otherwise the new controller is swapped in wherever the robot is and continues from the
    /// old one's state — see [`Controller::carry_over`].
    disturbs: bool,
    loading: std::sync::mpsc::Receiver<Result<Option<Controller>, String>>,
    /// What the thread came back with, once it has.
    built: Option<Result<Option<Controller>, String>>,
}

struct ManagedPrepared {
    controller: Option<Controller>,
    staged: PathBuf,
    destination: PathBuf,
}

struct ManagedSwap {
    request: Arc<managed_models::Request>,
    candidate: params::PolicyParams,
    cfg: params::ResolvedPolicy,
    loading: Option<std::sync::mpsc::Receiver<Result<Option<ManagedPrepared>, String>>>,
    prepared: Option<ManagedPrepared>,
    committing: Option<std::sync::mpsc::Receiver<Result<(), managed_models::CommitError>>>,
    committed: bool,
    disturbs: bool,
    home_started: bool,
}

fn managed_candidate(
    current: &params::PolicyParams,
    model: &str,
    path: &Path,
) -> Result<params::PolicyParams, String> {
    let mut candidate = current.clone();
    if let Some(slot) = Slot::parse(model) {
        if candidate.resolved().slot(slot).is_none() {
            return Err(format!("{model} is not enabled"));
        }
        candidate.set_slot(slot, Some(path.to_owned()));
    } else if let Some(skill) = candidate.skills.iter_mut().find(|skill| skill.name == model) {
        skill.path = Some(path.to_owned());
    } else if let Some(mut skill) = candidate.resolved_skills().into_iter().find(|skill| skill.name == model) {
        skill.path = Some(path.to_owned());
        candidate.skills.push(skill);
    } else {
        return Err(format!("{model} is not enabled"));
    }
    Ok(candidate)
}

fn managed_disturbs(model: &str, driving: Option<&Driving>) -> bool {
    match driving {
        Some(Driving::Walk) => model == "walk",
        Some(Driving::Stand) => model == "stand",
        Some(Driving::SitStand) => model == "sitstand",
        Some(Driving::Seated) | None => false,
        Some(Driving::GroundPick) => model == "ground_pick",
        Some(Driving::Skill(name)) => model == name,
    }
}

impl PendingSwap {
    /// Collect the thread's answer if it has one. Called once a tick; never blocks.
    fn poll(&mut self) {
        if self.built.is_some() {
            return;
        }
        match self.loading.try_recv() {
            Ok(result) => self.built = Some(result),
            Err(std::sync::mpsc::TryRecvError::Empty) => {}
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.built = Some(Err(
                    "the policy loading thread ended without a result".to_owned()
                ))
            }
        }
    }

    fn ready(&self) -> bool {
        self.built.is_some()
    }
}

/// Does a change replace the network that is driving the robot right now?
///
/// The report that made this a question: `policy load walk <file>`, a walk, a sit, then `policy
/// reset walk` — and the robot ramped to home and stood up. The reset was right about the config
/// and wrong about the robot: nothing it changed was running. The `sitstand` network had the
/// robot, and it was the same file before and after.
///
/// So a change is compared against what is driving, by the resolved path of that one network — or
/// the whole skill list when a skill is driving, because a skill is addressed by its index in it.
/// `None` driving, or a network that resolves to the same file, is a change the robot need not
/// move for.
///
/// A reload is the exception, and disturbs whatever is in motion: it exists for the case where
/// every path is unchanged and the bytes behind them are not, so the paths cannot say whether the
/// running network is affected. It is also the one change that is never issued while somebody is
/// trying a gait.
///
/// **A seated robot is disturbed by nothing.** The second report: `robotctl policy update` on a
/// robot sitting on the bench, and it ramped to home and stood up — the update ends in a reload,
/// and the reload took the mode switch's path because the sitstand network was driving. But the
/// home-first ramp exists to stop a network being swapped under a *moving* gait, and a seated
/// robot is parked: the network holds a static pose on a constant flag, and `busy()` already says
/// sitting is not travelling. Swapping bytes under that is exactly what `carry_over` was built
/// for — the seat, the flag and the filter anchor go across, and the next tick holds the same
/// pose from the same place. Standing it up to change a file it was not using was the surprise.
fn change_disturbs(
    driving: Option<&Driving>,
    change: &intents::PolicyChange,
    before: &params::ResolvedPolicy,
    after: &params::ResolvedPolicy,
) -> bool {
    let Some(driving) = driving else {
        return false;
    };
    if matches!(driving, Driving::Seated) {
        return false;
    }
    if matches!(change, intents::PolicyChange::Reload) {
        return true;
    }
    match driving {
        Driving::Walk => before.walk != after.walk,
        Driving::Stand => before.stand != after.stand,
        Driving::SitStand => before.sitstand != after.sitstand,
        Driving::Seated => false,
        Driving::GroundPick => before.ground_pick != after.ground_pick,
        Driving::Skill(_) => before.skills != after.skills,
    }
}

/// The config a change asks for, from the one that is running.
///
/// `None` when it cannot be read — a reload of a file that will not parse — which leaves the robot
/// exactly as it is. It is the caller's file and their mistake to fix, and dropping a working
/// policy set over a syntax error would be the worse of the two outcomes.
fn candidate_params(
    change: &intents::PolicyChange,
    current: &params::PolicyParams,
    params_path: &std::path::Path,
    slot_errors: &mut SlotErrors,
    overrides: &PolicyOverrides,
) -> Option<params::PolicyParams> {
    let mut candidate = current.clone();
    match change {
        intents::PolicyChange::Slot { slot, path } => candidate.set_slot(*slot, path.clone()),
        intents::PolicyChange::ResetAll => {
            for slot in Slot::ALL {
                candidate.set_slot(slot, None);
            }
        }
        // Re-read `[policy]` from disk, so a skill added since startup is one this robot has
        // without a restart.
        //
        // **Only `[policy]`, and the mode is kept.** The daemons read their config once at
        // startup and that stays true of everything else here — re-reading `[safety]` or
        // `[control]` under a running loop is a different and much larger promise. The mode is
        // carried over because `robot.setMode` deliberately does not write config: adopting the
        // file's mode here would undo a live switch as a side effect of adding a skill.
        intents::PolicyChange::Reload => match params::Params::load(params_path, false) {
            Ok(fresh) => {
                let mode = candidate.mode;
                candidate = fresh.policy;
                candidate.mode = mode;
                // A file may name a skill this board cannot load; normal startup drops it and
                // reports degraded. A selected process keeps that override so it fails instead
                // of silently falling back to an unselected bundled model.
                if overrides.enable_policies.is_none() {
                    drop_unloadable_overrides(&mut candidate, slot_errors);
                }
            }
            Err(e) => {
                tracing::error!(
                    error = %e,
                    path = %params_path.display(),
                    "reload: config will not parse; keeping what is running"
                );
                return None;
            }
        },
    }
    if let Err(error) = overrides.apply(&mut candidate) {
        tracing::error!(%error, change = describe_change(change), "policy overrides reject change");
        return None;
    }
    match managed_models::load(params_path) {
        Ok(paths) => managed_models::apply_paths(&mut candidate, &paths),
        Err(error) => {
            tracing::error!(%error, "cannot reload managed model paths");
            return None;
        }
    }
    Some(candidate)
}

fn try_controller(
    policy_cfg: &params::ResolvedPolicy,
    limp_fall: bool,
    protocol: BusProtocol,
    expected_home: Option<[f64; NUM_JOINTS]>,
) -> Result<Option<Controller>, PolicyError> {
    try_controller_with_candidate(policy_cfg, limp_fall, protocol, expected_home, None)
}

fn try_controller_with_candidate(
    policy_cfg: &params::ResolvedPolicy,
    limp_fall: bool,
    protocol: BusProtocol,
    expected_home: Option<[f64; NUM_JOINTS]>,
    candidate: Option<(&Path, &[u8])>,
) -> Result<Option<Controller>, PolicyError> {
    if !policy_cfg.enabled {
        tracing::warn!("policy disabled; holding the startup pose");
        return Ok(None);
    }
    {
        let tuning = Tuning {
            action_scale: policy_cfg.action_scale,
            standing_action_scale: policy_cfg.standing_action_scale,
            standing_gain_ratio: policy_cfg.standing_gain_ratio,
            gain: policy_cfg.gain,
            head_lowpass: policy_cfg.head_lowpass,
            legs_lowpass: policy_cfg.legs_lowpass,
        };
        let skills = SkillTuning {
            ground_pick_period: policy_cfg.ground_pick_period,
            ground_pick_end_phase: policy_cfg.ground_pick_end_phase,
            ground_pick_action_scale: policy_cfg.ground_pick_action_scale,
            ground_pick_gain_ratio: policy_cfg.ground_pick_gain_ratio,
            sitstand_rise_s: policy_cfg.sitstand_rise_s,
            sitstand_ramp_s: policy_cfg.sitstand_ramp_s,
            skills: policy_cfg.skills.clone(),
        };
        let paths = PolicyPaths {
            walk: policy_cfg.walk.clone(),
            stand: policy_cfg.stand.clone(),
            sitstand: policy_cfg.sitstand.clone(),
            ground_pick: policy_cfg.ground_pick.clone(),
            // Only the skills whose file resolves — a slot switched off with `"none"` is
            // already filtered out by `resolved_skills`.
            skills: policy_cfg
                .skills
                .iter()
                .filter_map(|skill| skill.resolved_path())
                .collect(),
        };
        match Policy::load_with_candidate(&paths, DEFAULT_STANDING_THRESHOLD, candidate) {
            Ok(mut policy) => {
                for (index, skill) in policy_cfg.skills.iter().enumerate() {
                    if skill.name == "headstand"
                        && !policy
                            .skill_phase(index)
                            .is_some_and(|phase| phase.holds_inverted())
                    {
                        return Err(PolicyError::Inference(
                            "headstand requires command_encoding=headstand_phase".into(),
                        ));
                    }
                }
                let home = if protocol == BusProtocol::Hx {
                    let home = policy.joint_home()?;
                    if expected_home.is_some_and(|expected| expected != home) {
                        return Err(PolicyError::JointContract(
                            "HOME changed; stop and restart robotd before using this bundle".into(),
                        ));
                    }
                    tracing::info!(home = ?home, "joint HOME loaded from policy metadata");
                    home
                } else {
                    DEFAULT_POSITION
                };
                // Roller mode has no standing network — command magnitude stops selecting
                // it. Nothing else reserves it: limp-fall hands back by *letting* the
                // standing network be selected, which is what stands the robot up.
                if policy_cfg.mode == Mode::Roller {
                    policy.set_standing_disabled(true);
                }
                tracing::warn!(
                    mode = policy_cfg.mode.as_str(),
                    walk = ?policy_cfg.walk.as_ref().map(|p| p.display().to_string()),
                    stand = ?policy_cfg.stand.as_ref().map(|p| p.display().to_string()),
                    sitstand = ?policy_cfg.sitstand.as_ref().map(|p| p.display().to_string()),
                    ground_pick = ?policy_cfg.ground_pick.as_ref().map(|p| p.display().to_string()),
                    kicks = policy_cfg.kick_left.is_some() || policy_cfg.kick_right.is_some(),
                    roulade = ?policy_cfg.roulade.as_ref().map(|p| p.display().to_string()),
                    headstand = ?policy_cfg.skills.iter().find(|skill| skill.name == "headstand").and_then(|skill| skill.resolved_path()),
                    limp_fall,
                    head_lowpass = policy_cfg.head_lowpass.unwrap_or(1.0),
                    legs_lowpass = policy_cfg.legs_lowpass.unwrap_or(1.0),
                    voltage_adapt = policy_cfg.voltage_adapt,
                    nominal_voltage = policy_cfg.nominal_voltage,
                    "policy loaded"
                );
                let home_target = if protocol == BusProtocol::Hx {
                    duck_control::hx::home_target()
                } else {
                    home
                };
                tracing::info!(home = ?home_target, "physical HOME target configured");
                Ok(Some(Controller::new(policy, tuning, skills, home, home_target)))
            }
            Err(e) => Err(e),
        }
    }
}

/// How many times the torque cut on the way to a power-off is tried before giving up.
const POWEROFF_TORQUE_ATTEMPTS: u32 = 3;

/// Cut torque before a power-off, and keep asking until every servo has heard.
///
/// This is the one bus write that has to land. The poweroff it precedes kills this process, so a
/// servo the cut did not reach stays stiff with nothing left running to tell it otherwise — and
/// a robot that sat down and switched off with its legs locked is exactly how a single dropped
/// ack presented, reported in the journal by the last tick that could have retried and acted on by
/// nobody. Torque writes are idempotent, so asking three times costs a few milliseconds on a tick
/// where the robot is sitting still, and it is the last thing this loop does that matters.
fn cut_torque_before_poweroff<T: RobotIo>(safety: &mut Safety<T>) {
    for attempt in 1..=POWEROFF_TORQUE_ATTEMPTS {
        match safety.set_torque(false) {
            Ok(()) => return,
            Err(e) => {
                tracing::warn!(error = %e, attempt, "cannot cut torque before power off")
            }
        }
    }
    tracing::error!(
        attempts = POWEROFF_TORQUE_ATTEMPTS,
        "torque may still be on; powering off anyway"
    );
}

/// Load the other mode's bundle, and only then say the mode changed.
///
/// `Policy::load` validates and warms up each network, so this blocks the loop for a moment.
/// Both callers are places where that costs nothing: a robot holding the home pose, or one with
/// no torque at all.
fn load_mode(
    target: Mode,
    policy_params: &mut params::PolicyParams,
    policy_cfg: &mut params::ResolvedPolicy,
    controller: &mut Option<Controller>,
    limp_fall: bool,
    state: &RobotState,
    slot_errors: &SlotErrors,
    protocol: BusProtocol,
    expected_home: Option<[f64; NUM_JOINTS]>,
) {
    let previous_mode = policy_params.mode;
    policy_params.mode = target;
    let cfg = policy_params.resolved();
    state.policy_error.store(None);
    let mut replacement = build_controller(&cfg, limp_fall, state, protocol, expected_home);
    if let Err(error) = transfer_precision(state, controller, &mut replacement) {
        policy_params.mode = previous_mode;
        tracing::error!(%error, "mode switch cannot preserve the precision policy");
        state.policy_change_error.store(Some(Arc::new(error.to_string())));
        return;
    }
    *controller = replacement;
    publish_managed_models(state, controller, &cfg);
    // Published together with the mode, and only after the load: a client that reads
    // `robot.mode` and gets the new one must not then be told the old mode's networks.
    state.policies.store(Arc::new(PolicyNames::of(&cfg)));
    state.mode.store(mode_code(target), Ordering::Relaxed);
    state
        .policy_slots
        .store(Arc::new(slot_report(policy_params, &cfg, slot_errors)));
    *policy_cfg = cfg;
    tracing::warn!(
        mode = target.as_str(),
        loaded = controller.is_some(),
        "mode switch complete"
    );
}

fn transfer_precision(
    state: &RobotState,
    current: &mut Option<Controller>,
    replacement: &mut Option<Controller>,
) -> Result<(), PolicyError> {
    match (current.as_mut(), replacement.as_mut()) {
        (Some(old), Some(new)) => new.transfer_precision_from(old)?,
        (Some(old), None) => {
            if let Some(policy) = old.take_precision_policy() {
                *state.precision_policy.lock().expect("precision policy mutex is not poisoned") = Some(policy);
            }
            state.precision_ready.store(false, Ordering::Release);
        }
        (None, Some(new)) => {
            let policy = state.precision_policy.lock().expect("precision policy mutex is not poisoned").take();
            if let Some(policy) = policy {
                if let Err((policy, error)) = new.install_precision_policy(policy) {
                    *state.precision_policy.lock().expect("precision policy mutex is not poisoned") = Some(policy);
                    return Err(error);
                }
                state.precision_ready.store(true, Ordering::Release);
            }
        }
        (None, None) => {}
    }
    Ok(())
}

/// [`try_controller`], with a failure recorded as *the* policy error.
///
/// The startup and mode-switch path: a policy that was wanted and would not load makes the robot
/// unhealthy, which is what gets a bad release rolled back rather than leaving a duck that holds
/// a pose and never walks again.
fn build_controller(
    policy_cfg: &params::ResolvedPolicy,
    limp_fall: bool,
    state: &RobotState,
    protocol: BusProtocol,
    expected_home: Option<[f64; NUM_JOINTS]>,
) -> Option<Controller> {
    match try_controller(policy_cfg, limp_fall, protocol, expected_home) {
        Ok(controller) => controller,
        Err(e) => {
            tracing::error!(error = %e, "policy unavailable; holding the pose");
            state.policy_error.store(Some(Arc::new(e.to_string())));
            None
        }
    }
}

async fn control_loop<T: RobotIo>(
    io: T,
    state: Arc<RobotState>,
    intents: Arc<Intents>,
    params: Params,
    // Where `[policy]` is re-read from on a reload, so adding a skill needs no restart.
    params_path: PathBuf,
    // Startup overrides stay in force for `robot.loadPolicy` and config reloads.
    policy_overrides: PolicyOverrides,
    period: Duration,
    poweroff: PowerOff,
    recorder: Option<record::RecordSender>,
    mut motion_cpu: Option<motion_cpu::MotionCpuFloor>,
) {
    // Owned rather than borrowed from `params`, because neither the mode nor the slots stay
    // what the file said: `robot.setMode` replaces the mode, `robot.loadPolicy` replaces one
    // slot, and the line below drops any override this board cannot actually load.
    let mut policy_params = params.policy.clone();
    // Slots running their default because an override would not load. Empty on nearly every
    // robot; anything in it makes health degraded and shows up in `robot.policies`.
    let mut slot_errors = SlotErrors::default();
    if policy_overrides.enable_policies.is_none() {
        drop_unloadable_overrides(&mut policy_params, &mut slot_errors);
    }
    // `mut` because a mode switch replaces it: the resolved policy *is* the mode, once the
    // per-mode defaults have been applied.
    let mut policy_cfg = policy_params.resolved();
    let fall_position_gains = match params.fall_position_gains() {
        Ok(gains) => gains,
        Err(error) => {
            tracing::error!(%error, "invalid native fall gains");
            return;
        }
    };
    // Republished, because `RobotState::new` built both from the file and the check above may
    // have dropped an override since. A client that asks before the first tick gets what this
    // loop is actually going to load, not what the file asked for.
    state.policies.store(Arc::new(PolicyNames::of(&policy_cfg)));
    state.policy_slots.store(Arc::new(slot_report(
        &policy_params,
        &policy_cfg,
        &slot_errors,
    )));
    let mut safety = Safety::new(
        io,
        SafetyConfig {
            fall_gravity_z: params.safety.fall_gravity_z,
            fall_debounce: Duration::from_millis(params.safety.fall_debounce_ms),
            deadman: Duration::from_millis(params.safety.deadman_ms),
            gain_running: policy_cfg.gain,
            gain_limp: params.safety.gain_limp,
        },
    );

    let Some(mut hold) = adopt_startup_pose(&mut safety, &state, &intents,
                                            params.bus.protocol == BusProtocol::Hx, period).await else {
        return;
    };
    if state.stopping.load(Ordering::Acquire) {
        finish_startup_exit(&mut safety, &state).await;
        return;
    }

    let mut controller = build_controller(
        &policy_cfg,
        params.safety.limp_fall,
        &state,
        params.bus.protocol,
        None,
    );
    if let Some(loaded) = controller.as_ref() {
        let check = managed_models::load(&params_path)
            .and_then(|paths| managed_models::verify_loaded(&loaded.managed_models(&policy_cfg), &paths));
        if let Err(error) = check {
            tracing::error!(%error, "managed model integrity check failed");
            state.policy_error.store(Some(Arc::new(error)));
            controller = None;
        }
    }
    let pending_precision = state.precision_policy.lock().expect("precision policy mutex is not poisoned").take();
    if let Some(policy) = pending_precision {
        match controller.as_mut() {
            Some(controller) => match controller.install_precision_policy(policy) {
                Ok(()) => state.precision_ready.store(true, Ordering::Release),
                Err((policy, error)) => {
                    *state.precision_policy.lock().expect("precision policy mutex is not poisoned") = Some(policy);
                    tracing::error!(%error, "precision policy does not match the controller");
                    state.policy_error.store(Some(Arc::new(error.to_string())));
                    return;
                }
            },
            None => {
                *state.precision_policy.lock().expect("precision policy mutex is not poisoned") = Some(policy);
                tracing::error!("precision policy requires a loaded controller");
                return;
            }
        }
    }
    publish_managed_models(&state, &controller, &policy_cfg);
    // Model references stay immutable for hot-loaded networks; physical Home is separate.
    let policy_home = controller
        .as_ref()
        .map_or(DEFAULT_POSITION, Controller::home);
    let home = match controller.as_ref() {
        Some(controller) => controller.home_target(),
        None if params.bus.protocol == BusProtocol::Hx => duck_control::hx::home_target(),
        None => DEFAULT_POSITION,
    };

    if state.stopping.load(Ordering::Acquire) {
        finish_startup_exit(&mut safety, &state).await;
        return;
    }

    tracing::warn!(
        joints = NUM_JOINTS,
        hz = 1.0 / period.as_secs_f64(),
        driving = controller.is_some(),
        "control loop running"
    );

    let mut ticker = tokio::time::interval(period);
    // `Skip`, not `Burst` and not `Delay`.
    //
    // `Burst` replays a backlog back to back, stacking motor commands — clearly wrong. But
    // `Delay` is wrong too, and less obviously: it schedules the next tick at *now + period*
    // after each one, so every tick's wakeup latency is added to the period rather than
    // absorbed. A few milliseconds of scheduler jitter becomes a permanent rate loss.
    //
    // Measured, not reasoned about: with `Delay` this loop reported 43.1 Hz against a 50 Hz
    // target and `missed = 0` — not overrunning its work, just being rescheduled late every
    // time. With a real bus read costing 3–8 ms it would have been nearer 35 Hz, and it
    // would have looked like a hardware problem.
    //
    // `Skip` keeps the original schedule and drops missed ticks, which is what a control
    // loop wants: no backlog, no drift.
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    let mut window_start = Instant::now();
    let mut window_ticks = 0u64;
    let mut last_summary = Instant::now();
    let measure_servo_read_time = state.measure_servo_read_time.load(Ordering::Relaxed);
    let mut read_timing = ServoReadTiming::new(measure_servo_read_time);
    // Every failed read is logged; this counter also feeds the loop summary.
    let mut bus_drops_window = 0u32;
    // Ticks this window whose sample came from the coast rather than the bus.
    let mut stale_steps_window = 0u32;
    // The run of failed reads in progress, reported as one line when it ends. See [`ReadRun`].
    let mut read_run: Option<ReadRun> = None;
    let mut was_driving = false;
    let mut policy_was_enabled = false;
    let mut resume_seated_policy = false;
    let mut bringup = Bringup::Limp;
    // A mode switch in flight: the mode to end up in, once the robot is home. `None` the rest of
    // the time, which is nearly always.
    let mut mode_change: Option<Mode> = None;
    // A policy load in flight, for the same reason and on the same path as `mode_change`: the
    // robot goes home first, and the swap happens there.
    let mut pending_swap: Option<PendingSwap> = None;
    let mut managed_swap: Option<ManagedSwap> = None;

    // Command smoothing, per the prototype: `cmd += α × (target − cmd)` at the tick rate.
    // A stick snap becomes a ramp the gait can follow; the state lives here because it is
    // per-tick, which the intent slots must not be.
    let dt = period.as_secs_f64();
    let cmd_alpha = params.control.cmd_alpha.clamp(0.0, 1.0);
    let velocity_command_safety = policy_overrides.velocity_command_safety();
    tracing::info!(
        cmd_alpha,
        immediate_stop = policy_overrides.immediate_stop,
        gamepad_start_delay_ms = intents.gamepad_timing().start_delay.as_millis() as u64,
        gamepad_short_command_ms = intents.gamepad_timing().short_duration.as_millis() as u64,
        "velocity command smoothing configured"
    );
    let head_alpha = params.control.head_alpha.clamp(0.0, 1.0);
    // Whether measured velocity and load go on the state stream. Read once: it is a startup
    // setting like the rest of `[control]`, and `robotctl configure` writing the file takes
    // effect on the next start of the daemon, not mid-tick.
    let publish_measurements = params.control.publish_velocity_and_load;
    let mut twist_ema = [0.0f64; 3];
    let mut walk_stand_handoff = WalkStandHandoff::default();
    let mut obstacle_assistance = obstacle::Assistance::default();
    let max_angular = intents.max_angular();
    let tracking_min = intents.limit_twist([params.pad_drive.vx_min, params.pad_drive.vy_min, params.pad_drive.vyaw_min]);
    let tracking_max = intents.limit_twist([params.pad_drive.vx_max, params.pad_drive.vy_max, params.pad_drive.vyaw_max]);
    let tracking_limits = coordinated_tracking::MotionLimits {
        forward: [tracking_min[0], tracking_max[0]],
        lateral: [tracking_min[1], tracking_max[1]],
        yaw: [tracking_min[2], tracking_max[2]],
    };
    let mut previous_stop_generation = 0;
    let mut previous_abandon_generation = 0;
    let mut previous_gamepad_twist = false;
    let mut previous_gamepad_ready = false;
    let mut current_precision: Option<Arc<intents::PrecisionRequest>> = None;
    let mut head_ema = [0.0f64; 4];
    let mut head_tracker = head_tracking::HeadTracker::default();
    let mut target_head_tracker = head_tracking::HeadTracker::default();
    let mut head_positioning: Option<(u16, head_tracking::HeadReturn)> = None;
    let mut head_handoff = head_tracking::HeadHandoff::default();
    let mut head_tracking_sequence = None;
    let mut motion_tracker = motion_tracking::MotionTracker::default();
    let mut motion_tracking_sequence = None;
    let mut coordinated_tracker = coordinated_tracking::Tracker::with_home([home[6], home[7]]);
    let mut target_follow_stability = target_follow_stability::TargetFollowStability::default();
    let mut target_walk_grace = target_follow_stability::WalkGrace::default();
    let mut target_head_rate = target_follow_stability::HeadRate::default();
    let mut target_geometry_missing = false;
    let mut coordinated_mouth = coordinated_mouth::Sequence::default();
    let mut person_loss_mouth: Option<(u64, Option<Instant>)> = None;
    let mut mouth_close_until = None;
    let mut coordinated_initial_head = coordinated_tracking::InitialHead::default();
    let mut coordinated_tracking_sequence = None;
    let mut pickup_action_completed = None;
    let mut coordinated_head_only_mode = None;
    let mut coordinated_feedback = coordinated_tracking::FeedbackHistory::default();
    let mut coordinated_tick: Option<Instant> = None;
    let mut coordinated_feedback_missing = false;
    let mut coordinated_paused = false;
    let mut coordinated_previous_phase = None;
    let mut coordinated_unwritten_sit = false;
    let mut maze_controller: Option<(u64, maze::Maze)> = None;
    let mut autonomous_fall_recovery = maze::FallRecovery::new();
    let mut maze_last_head = 0.0;
    let mut maze_previous_phase = None;
    let mut body_ema = [0.0f64; 3];

    // Coasting over a dropped bus read, and where the robot actually is.
    //
    // `known_positions` is updated from every sample that arrives and is what a fallback
    // holds. It used to be `hold`, which is only assigned when driving *stops* — and the
    // assignment needed a sample, which is the one thing a failed read does not have. So a
    // drop mid-stride held whatever `hold` was last set to, which after bring-up is the home
    // pose: one tick of "snap to standing" at full gain, mid-motion.
    let mut coast = Coast::new();
    let mut imu_input_monitor = imu_input::Monitor::new(period);
    let mut gravity_selector = policy_gravity::Selector::new(period);
    let mut gravity_holding = false;
    let mut stopped_driving_at: Option<Instant> = None;

    // Contact odometry, ticked at the loop's own rate on the sample the loop already
    // read — the prototype ran it at 100 Hz on extra bus reads, which is exactly the
    // bus pressure the spasms investigation taught this loop not to add.
    let mut odometry = odometry::Odometry::alpha();

    // The sit-then-power-off sequence: the policy sits, then `shutdown_rest` eases the joints
    // into the rest pose before torque is cut.
    let mut shutdown_sit: Option<Instant> = None;
    let mut shutdown_power_off = true;
    let mut shutdown_rest: Option<RestMotion> = None;
    let mut rest_action = None;
    let mut rest_sit_remaining = Duration::ZERO;
    let mut exit_sit_started: Option<Instant> = None;
    let mut seat_transition_until: Option<Instant> = None;
    // What the previous tick commanded, which is where the rest ramp starts.
    let mut last_targets = DEFAULT_POSITION;
    let mut maintenance_parked = false;
    // Voltage recovery does not itself authorize target writes: HX can re-enable
    // torque on a goal-position command even with the policy disabled.
    let mut maintenance_holding = state.low_voltage_lock.requires_rearm()
        || state.startup_release_verified.load(Ordering::Acquire);
    let mut maintenance_reopen_error: Option<String> = None;
    let mut next_low_lock_attempt = Instant::now();
    let mut powered_off = false;
    let mut warned_imu_warming = false;

    // Limp-fall (`[safety] limp_fall`): the predictor that sees a fall start, and where the
    // sequence it drives has got to. Built whether or not the mode is on — it costs a
    // multiply-subtract per tick, and having the numbers computed on every robot is what
    // makes the thresholds tunable against a recording from a robot that was not running
    // the mode.
    let mut falling = FallPredictor::new(FallPredictorConfig {
        tilt_z: params.safety.limp_fall_tilt_z,
        predicted_z: params.safety.limp_fall_predict_z,
        lookahead: Duration::from_millis(params.safety.limp_fall_lookahead_ms),
        debounce: Duration::from_millis(params.safety.limp_fall_debounce_ms),
    });
    let limp_fall_still = Duration::from_millis(params.safety.limp_fall_still_ms);
    let limp_fall_max = Duration::from_millis(params.safety.limp_fall_max_ms);
    let limp_fall_pose = Duration::from_millis(params.safety.limp_fall_pose_ms);
    let mut limp_fall = LimpFall::Idle;

    // The voice, and the ear. Both optional equipment: a robot without a codec or a bank
    // walks identically — the player degrades to a debug line, and the mic worker is only
    // spawned when configured, with its own retry loop when arecord flaps.
    let mut voice = params.audio.enabled.then(|| {
        sound::Sound::new(
            params.audio.bank.clone(),
            params.audio.device.clone(),
            params.audio.player.clone(),
        )
    });
    let pet: Option<pet_detect::worker::PetHandle> = if params.audio.enabled
        && params.audio.pet_detect_resolved(params.policy.mode)
        && let Some(model) = params.audio.pet_model_resolved()
        && model.exists()
    {
        match pet_detect::worker::PetHandle::spawn(pet_detect::worker::PetConfig {
            alsa_device: params.audio.capture_device(),
            model_path: model.clone(),
            enter_threshold: params.audio.pet_enter_threshold,
            exit_threshold: params.audio.pet_exit_threshold,
        }) {
            Ok(handle) => {
                tracing::warn!(model = %model.display(), "petting detection listening");
                Some(handle)
            }
            Err(e) => {
                // Not unhealthy: the classifier is a feature, not the robot. A missing
                // model on a release that ships one is caught by the packaging tripwires.
                tracing::warn!(error = %e, "petting detection unavailable");
                None
            }
        }
    } else {
        // Configured on but nothing to run: a bench robotd, a board that predates the
        // model, or audio off. Quietly so on purpose — the mic is optional equipment.
        None
    };

    // The theremin. `[theremin] enabled` is off by default, so on most ducks this is `None`
    // and nothing here subscribes to depth at all — `tofd` keeps its own counsel, and
    // `robotctl monitor` is unaffected either way, since it subscribes to `tofd` itself.
    //
    // On a duck that has turned it on, the depth reader starts *now* and parks on `tofd`'s
    // socket whether or not anyone ever asks for an instrument: one blocked read costs
    // nothing, and connecting lazily would make the first arming window wait for a
    // connection as well as for frames — a second of silence that reads as a broken feature.
    // Also off when audio is off, since a theremin with no voice is a mouth opening for no
    // reason.
    let mut theremin = (params.theremin.enabled && params.audio.enabled)
        .then(|| theremin::Theremin::spawn(params.theremin.socket.clone(), params.theremin.hand()));
    // How far the beak is open for the chorale, slewed across ticks — see where it is written.
    let mut chorale_mouth = 0.0f64;
    // And how the head sways while singing, applied to the next tick's command.
    let mut chorale_head = [0.0f64; 4];
    let mut idle_yawn = idle_yawn::IdleYawn::new(
        params.idle,
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system clock precedes Unix epoch")
            .as_nanos() as u64,
    );

    // The note the theremin is holding, kept across ticks so a hand leaving the frame fades
    // the note at its own pitch instead of gliding to the bottom of the range on its way out.
    let mut theremin_hz = 0.0f64;

    // The chorale. Off unless the config opted in — see `[chorale] accept`, and note that off
    // means putting nothing on the air rather than declining politely.
    let mut chorale = (params.chorale.accept && params.audio.enabled)
        .then(|| {
            let personality = voice.as_mut().and_then(|v| v.personality())?;
            Some(chorale::Chorale::new(
                personality.pitch_center_hz,
                personality.seed,
            ))
        })
        .flatten();
    if chorale.is_some() {
        tracing::warn!("chorale: this robot will sing with others");
    }

    let freeze_policy_imu = matches!(params.bus.protocol, BusProtocol::Hx)
        && std::env::var_os("ROBOT_FREEZE_POLICY_IMU").is_some_and(|value| value == "1");
    let imu_filter = policy_overrides.imu_filter.unwrap_or(true);
    let imu_gyro_filter = imu_filter && policy_overrides.imu_gyro_filter.unwrap_or(true);
    tracing::info!(
        gyro_enabled = imu_gyro_filter,
        gravity_enabled = imu_filter,
        "policy IMU host median filters"
    );
    if freeze_policy_imu {
        tracing::warn!(
            "diagnostic: policy IMU fixed upright; real IMU remains active for telemetry and safety"
        );
    }

    // When the joints in `sensors` were actually read, `CLOCK_MONOTONIC` ns — the clock
    // `tof.frame` and a mapper share. Stamped at the read, not at publish, because
    // `controller.step()` (an ONNX forward pass) sits between the two and would bias every
    // `robot.state` timestamp by one inference otherwise. It rides the coast: a coasted tick
    // republishes the last fresh sample, so it must republish that sample's read time too.
    let mut sensors_read_ns = 0u64;
    let mut telemetry = match telemetry::Publisher::start(Arc::clone(&state), period) {
        Ok(publisher) => publisher,
        Err(error) => {
            tracing::error!(%error, "cannot start state output worker");
            return;
        }
    };
    let mut device_schedule = telemetry::DeviceSchedule::default();
    let mut status_maintenance_stop: Option<StatusMaintenanceStop> = None;
    let mut status_measurement_still = false;
    let mut status_session_open = false;
    let mut status_hold_pose = None;
    let mut status_page_was_ready = false;
    let mut status_pose_latched = false;
    let mut next_status_recovery = Instant::now();
    let mut status_only = ReadOnlyStatusSession::default();
    // The startup pose read precedes blocking model/audio initialization. Begin
    // the host estimator's continuous stream here; runtime gap checks stay strict.
    safety.restart_imu_stream();
    let mut shutdown_after_maintenance = false;
    while !state.shutdown.load(Ordering::Acquire) {
        ticker.tick().await;
        let tick_start = Instant::now();
        if let Some(floor) = motion_cpu.as_mut() {
            // Standing under policy control still needs timely feedback with zero twist.
            // The moving flag includes homing, scripted motion and fall recovery.
            let active = intents.enabled() || state.moving.load(Ordering::Relaxed)
                || intents.rest_phase() == proto::RestPhase::Resting
                || shutdown_sit.is_some() || shutdown_rest.is_some()
                || matches!(state.exit_sit.load(Ordering::Acquire), EXIT_SIT_REQUESTED | EXIT_SIT_RUNNING);
            floor.update(active && !powered_off && !maintenance_parked && !status_session_open
                && !state.status_bus_fault.load(Ordering::Acquire), tick_start);
        }
        read_timing.begin_tick(tick_start);

        if state.status_bus_fault.load(Ordering::Acquire) {
            let partial_maintenance = state.human_control.lock().partial_maintenance();
            if partial_maintenance
                && service_startup_maintenance(&state, &mut maintenance_parked, &mut maintenance_reopen_error, |release| {
                    if release {
                        status_only.close(&mut safety, &state).map_err(duck_control::io::IoError::Bus)?;
                        verify_faulted_partial_maintenance(&state).map_err(duck_control::io::IoError::Bus)?;
                    }
                    safety.maintenance_handoff(release)
                })
            {
                continue;
            }
            let mut requested_recovery = None;
            service_startup_emergency(&state, &intents, || {
                let result = status_only.close(&mut safety, &state)
                    .and_then(|()| safety.recover_device_bus().map_err(|error| error.to_string()))
                    .and_then(|()| status_bus::complete_recovery(&state.status_journal)
                        .map_err(|error| error.to_string()));
                requested_recovery = Some(result.clone());
                result
            });
            if requested_recovery.is_none() && status_only_requested(&state)
                && !state.stopping.load(Ordering::Acquire)
            {
                intents.set_enabled(false);
                if safety.supports_individual_device_observation() {
                    status_only.sample(&mut safety, &state, period, false);
                } else {
                    publish_status_fault(&state, None);
                }
                continue;
            }
            if let Err(error) = status_only.close(&mut safety, &state) {
                tracing::error!(%error, "cannot close faulted status session");
            }
            publish_status_fault(&state, safety.sample_device_observation(tick_start));
            let phase = state.human_control.lock().maintenance_phase();
            let recovered = if let Some(result) = requested_recovery {
                Some(result)
            } else if tick_start >= next_status_recovery && phase != Some(human_control::MaintenancePhase::Active) {
                intents.set_enabled(false);
                Some(safety.recover_device_bus().map_err(|error| error.to_string())
                    .and_then(|()| status_bus::complete_recovery(&state.status_journal).map_err(|error| error.to_string())))
            } else { None };
            if let Some(recovered) = recovered {
                match recovered {
                    Ok(()) => {
                        let mut guard = state.human_control.lock();
                        if guard.maintenance_phase() == Some(human_control::MaintenancePhase::Returning) {
                            guard.finish_status_recovery();
                            guard.finish_maintenance_return();
                            maintenance_parked = false;
                        }
                        if guard.emergency_release_completed() {
                            guard.finish_emergency_release();
                        }
                        drop(guard);
                        state.status_bus_fault.store(false, Ordering::Release);
                        state.homed.store(false, Ordering::Release);
                        clear_human_intents(&intents);
                        intents.cancel_motion_tracking();
                        intents.cancel_maze();
                        let _ = intents.take_power_request();
                        if let Some(controller) = controller.as_mut() { controller.forget(); }
                        maintenance_holding = true;
                        bringup = Bringup::Limp;
                        was_driving = false;
                        policy_was_enabled = false;
                        resume_seated_policy = false;
                        status_session_open = false;
                        status_page_was_ready = false;
                        status_pose_latched = false;
                        status_maintenance_stop = None;
                        sensors_read_ns = 0;
                        device_schedule.reset();
                        tracing::warn!("status bus recovered; policy remains disabled and joints released");
                    }
                    Err(error) => {
                        tracing::warn!(%error, "status bus recovery failed; retrying");
                        next_status_recovery = Instant::now() + Duration::from_secs(2);
                    }
                }
            }
            continue;
        }

        // The bus belongs to this loop unless maintenance has already started. The
        // arbiter prevents a new handoff while this request is queued or executing.
        let emergency = {
            let guard = state.human_control.lock();
            if guard.maintenance_phase().is_none() {
                state.emergency_relax.take()
            } else {
                None
            }
        };
        if let Some(waiters) = emergency {
            if let Err(error) = status_only.close(&mut safety, &state) {
                let mut guard = state.human_control.lock();
                match state.emergency_relax.complete(waiters, Err(format!("串口交接未确认：{error}"))) {
                    emergency_relax::CompletionState::Verified => guard.finish_emergency_release(),
                    emergency_relax::CompletionState::Unverified => guard.mark_emergency_release_unverified(),
                    emergency_relax::CompletionState::Pending => {},
                }
                continue;
            }
            if status_session_open {
                safety.set_individual_device_observation(false);
                if let Err(error) = status_bus::complete(&state.status_journal) {
                    safety.quarantine_device_bus();
                    state.status_bus_fault.store(true, Ordering::Release);
                    let mut guard = state.human_control.lock();
                    match state.emergency_relax.complete(waiters, Err(format!("串口交接未确认：{error}"))) {
                        emergency_relax::CompletionState::Verified => guard.finish_emergency_release(),
                        emergency_relax::CompletionState::Unverified => guard.mark_emergency_release_unverified(),
                        emergency_relax::CompletionState::Pending => {},
                    }
                    continue;
                }
                status_session_open = false;
            }
            intents.cancel_pending_rest();
            intents.finish_rest(false);
            rest_action = None;
            maintenance_holding = true;
            hold = last_targets;
            bringup = Bringup::Limp;
            was_driving = false;
            policy_was_enabled = false;
            resume_seated_policy = false;
            shutdown_sit = None;
            shutdown_rest = None;
            exit_sit_started = None;
            seat_transition_until = None;
            limp_fall = LimpFall::Idle;
            falling.reset();
            if let Some(controller) = controller.as_mut() {
                controller.forget();
            }
            if state.exit_sit.load(Ordering::Acquire) != 0 {
                state.exit_sit_succeeded.store(false, Ordering::Release);
                state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
            }
            let result = safety.set_torque(false).map_err(|error| error.to_string())
                .and_then(|()| if params.bus.protocol == BusProtocol::Hx {
                    Ok(())
                } else {
                    Err("该舵机总线不支持卸力回读，结果未确认".to_owned())
                });
            match &result {
                Ok(()) => tracing::warn!("emergency torque release verified"),
                Err(error) => tracing::error!(%error, "emergency torque release not verified"),
            }
            let mut guard = state.human_control.lock();
            match state.emergency_relax.complete(waiters, result) {
                emergency_relax::CompletionState::Verified => guard.finish_emergency_release(),
                emergency_relax::CompletionState::Unverified => guard.mark_emergency_release_unverified(),
                emergency_relax::CompletionState::Pending => {},
            }
            // Do not use an earlier snapshot or write a target in the release tick.
            continue;
        }
        let (emergency_active, partial_maintenance) = {
            let guard = state.human_control.lock();
            (guard.emergency_release_active(), guard.partial_maintenance())
        };
        if emergency_active && partial_maintenance {
            if let Err(error) = status_only.close(&mut safety, &state) {
                state.human_control.lock().finish_maintenance_handoff(
                    human_control::MaintenancePhase::Failed, Some(error));
                continue;
            }
        }
        if emergency_active && !partial_maintenance {
            // A failed release stays locked until an explicit retry succeeds.
            // The status page may still inspect the exclusively owned bus one
            // device at a time; no normal read or actuator write is permitted.
            intents.set_enabled(false);
            if status_only_requested(&state) && !state.stopping.load(Ordering::Acquire) {
                if safety.supports_individual_device_observation() {
                    status_only.sample(&mut safety, &state, period, false);
                } else {
                    publish_status_fault(&state, None);
                }
            } else if let Err(error) = status_only.close(&mut safety, &state) {
                tracing::error!(%error, "cannot close emergency status session");
            }
            continue;
        }

        let (maintenance, maintenance_stop_requested, status_page_active) = {
            let guard = state.human_control.lock();
            (guard.maintenance_phase(), guard.status_stop_requested(),
                guard.status_blocked() || guard.maintenance_transition_blocked())
        };
        let status_stop_requested = maintenance_stop_requested || status_page_active;
        // A maintenance page must not command a fallen robot to stand up. If a
        // fall was already in progress when the page opened, abandon that recovery
        // and hold the measured pose; keep following it while the robot is down.
        if status_stop_requested && (safety.fallen() || limp_fall != LimpFall::Idle) {
            intents.set_enabled(false);
            hold = coast.known_positions(hold);
            if status_page_active { status_hold_pose = Some(hold); }
            limp_fall = LimpFall::Idle;
            falling.reset();
        }
        if status_page_active && status_hold_pose.is_none() {
            status_hold_pose = Some(coast.known_positions(hold));
        } else if !status_page_active {
            status_hold_pose = None;
            status_pose_latched = false;
        }
        if status_session_open && (!status_page_active || maintenance.is_some()
            || state.exit_sit.load(Ordering::Acquire) != 0 || state.stopping.load(Ordering::Acquire))
        {
            safety.set_individual_device_observation(false);
            if let Err(error) = status_bus::complete(&state.status_journal) {
                tracing::error!(%error, "cannot close status session; motor bus remains quarantined");
                safety.quarantine_device_bus();
                state.status_bus_fault.store(true, Ordering::Release);
                continue;
            }
            status_session_open = false;
        }
        if status_session_open && status_page_active && maintenance.is_none()
            && state.exit_sit.load(Ordering::Acquire) == 0 && !state.stopping.load(Ordering::Acquire)
            && shutdown_sit.is_none() && shutdown_rest.is_none() && !powered_off
        {
            // The status page owns the bus while its guard is active. Do not run
            // the normal control read or issue policy targets in this branch.
            intents.set_enabled(false);
            safety.reset_fall();
            state.fallen.store(false, Ordering::Relaxed);
            if let Some(voice) = voice.as_mut() {
                voice.set_mouth_enabled(state.mouth_sound_enabled.load(Ordering::Relaxed));
                voice.theremin_settle();
                voice.update_hold(intents.sound_hold());
                for tag in intents.take_sounds() { voice.play(tag.as_str(), false); }
                if let Some(name) = intents.take_sound_clip() { voice.play_clip(Arc::from(name.as_str())); }
                voice.poll_clip();
                if let Some(pet) = pet.as_ref() {
                    while let Some(event) = pet.try_recv_event() {
                        if matches!(event, pet_detect::PettingEvent::Start)
                            && !safety.fallen() && controller.as_ref().is_none_or(|controller| !controller.busy())
                        { voice.play("coo", false); }
                    }
                    while pet.try_recv_sound().is_some() {}
                }
            }
            let observation = match sample_status_devices(&state, &intents, &mut safety,
                                                          &mut device_schedule, period, tick_start + period) {
                Ok(observation) => observation,
                Err(error) => {
                    tracing::error!(%error, "status reply unconfirmed; resynchronizing motor bus");
                    safety.quarantine_device_bus();
                    state.status_bus_fault.store(true, Ordering::Release);
                    publish_status_fault(&state, safety.sample_device_observation(tick_start));
                    continue;
                }
            };
            if let Some(observation) = observation {
                let now_ns = proto::clock::monotonic_ns();
                let voltage = status_voltage(&observation, now_ns);
                let low_triggered = state.low_voltage.observe(voltage, tick_start);
                if low_triggered || (state.low_voltage.status().triggered
                    && !state.low_voltage_lock.persistence_confirmed() && tick_start >= next_low_lock_attempt)
                {
                    safety.set_individual_device_observation(false);
                    if let Err(error) = status_bus::complete(&state.status_journal) {
                        tracing::error!(%error, "cannot close status session for low-voltage protection");
                        safety.quarantine_device_bus();
                        state.status_bus_fault.store(true, Ordering::Release);
                        continue;
                    }
                    status_session_open = false;
                    let mut guard = state.human_control.lock();
                    guard.emergency_stop();
                    clear_human_intents(&intents);
                    intents.cancel_motion_tracking();
                    intents.cancel_maze();
                    match state.low_voltage_lock.trip() {
                        Ok(()) => {
                            state.release_torque_on_exit.store(true, Ordering::Release);
                            state.exit_sit.store(EXIT_SIT_REQUESTED, Ordering::Release);
                            state.low_voltage.triggered.notify_one();
                            tracing::error!(?voltage, "low voltage protection triggered");
                        }
                        Err(error) => {
                            next_low_lock_attempt = tick_start + Duration::from_secs(1);
                            tracing::error!(%error, "low-voltage lock not saved; stopping motion and retrying");
                        }
                    }
                    continue;
                }
                if let Some(voltage) = voltage {
                    let previous = f64::from_bits(state.battery_v.load(Ordering::Relaxed));
                    let smoothed = if previous > 0.0 {
                        BATTERY_EMA_ALPHA * voltage + (1.0 - BATTERY_EMA_ALPHA) * previous
                    } else { voltage };
                    state.battery_v.store(smoothed.to_bits(), Ordering::Relaxed);
                }
                if let Some((sensors, sampled_ns)) = status_sensors(&observation, now_ns) {
                    let sensors = match &state.pose_calibration {
                        Some(calibration) => calibration.correct(sensors),
                        None => sensors,
                    };
                    sensors_read_ns = sampled_ns;
                    hold = sensors.positions;
                    coast.sample(Some(sensors));
                    if state.state_tx.receiver_count() > 0 {
                        let policy = telemetry.label("held");
                        telemetry.publish(telemetry::Snapshot {
                            captured_at: Instant::now(), t: state.started.elapsed().as_secs_f64(),
                            t_ns: sampled_ns, sensors, targets: hold, requested: [0.0; 3], applied: [0.0; 3],
                            limited: [false; 3], head: [0.0; 4], policy, tracking_phase: None, pickup_stage: None,
                            safety: proto::SafetyState { fallen: safety.fallen(),
                                limp: safety.torque_release_verified() || intents.rest_phase() == proto::RestPhase::Rested,
                                gravity: sensors.imu.gravity, gain: safety.gain() },
                            control_loop: proto::LoopState {
                                hz: f64::from_bits(state.achieved_hz.load(Ordering::Relaxed)),
                                missed: state.missed.load(Ordering::Relaxed),
                            },
                            publish_velocity: publish_measurements, publish_load: false,
                            odom: proto::OdomState { position: odometry.position(), yaw: odometry.yaw() },
                            theremin: None, chorale: None,
                        });
                    }
                }
                publish_device_observation(&state, Some(observation));
            }
            state.ticks.fetch_add(1, Ordering::Relaxed);
            state.last_tick_us.store(state.started.elapsed().as_micros() as u64, Ordering::Relaxed);
            window_ticks += 1;
            let window = window_start.elapsed();
            if window >= RATE_WINDOW {
                state.achieved_hz.store((window_ticks as f64 / window.as_secs_f64()).to_bits(), Ordering::Relaxed);
                window_start = Instant::now();
                window_ticks = 0;
            }
            continue;
        }
        let mut status_quiescing = false;
        let mut status_page_ready = false;
        if status_stop_requested {
            let now = Instant::now();
            let posture = controller.as_ref().and_then(Controller::driving);
            let stop = status_maintenance_stop.get_or_insert_with(|| {
                StatusMaintenanceStop::new(now, posture == Some(Driving::Walk))
            });
            let age_ns = proto::clock::monotonic_ns().saturating_sub(sensors_read_ns);
            let fresh = sensors_read_ns != 0 && age_ns <= (period * 2).as_nanos() as u64
                && state.consecutive_errors.load(Ordering::Relaxed) == 0;
            // A missing policy state is not proof of rest: startup, a scripted action,
            // precision motion, homing and shutdown can all be actively moving the robot.
            let transitions_active = matches!(bringup, Bringup::Homing { .. })
                || mode_change.is_some() || pending_swap.is_some()
                || shutdown_sit.is_some() || shutdown_rest.is_some()
                || limp_fall != LimpFall::Idle
                || controller.as_ref().is_some_and(|controller|
                    controller.busy() || controller.scripted_active() || controller.precision_active());
            // Finish the normal zero-velocity walk handoff before disabling the
            // policy. A standstill can be disabled immediately; the measured-pose
            // check below still gates starting individual device reads.
            if intents.enabled() && !transitions_active && stop.policy_can_stop(twist_ema) {
                intents.set_enabled(false);
                hold = coast.known_positions(hold);
                if status_page_active { status_hold_pose = Some(hold); }
            }
            let enabled = intents.enabled();
            let posture_stable = !enabled || matches!(posture, Some(Driving::Stand | Driving::Seated));
            // Once a status session is safely established with the policy off,
            // external handling must not suspend the individual servo scan for
            // another full stillness dwell on every moving sample.
            let established_status = status_page_active && status_pose_latched && !enabled
                && !transitions_active && fresh;
            // A completed rest or individually verified emergency release has
            // already stopped the policy and released the joints. Do not wait
            // for another synchronous control read before yielding the bus.
            let released = !enabled && !transitions_active
                && (partial_maintenance || safety.torque_release_verified() || intents.rest_phase() == proto::RestPhase::Rested);
            if !established_status && !released && !stop.ready(now, fresh && status_measurement_still,
                                                   transitions_active, posture_stable, twist_ema) {
                if stop.timed_out(now) && maintenance_stop_requested {
                    state.human_control.lock().finish_maintenance_handoff(
                        human_control::MaintenancePhase::Failed,
                        Some("无法确认机器人已经停止，请检查设备后重试".into()));
                    continue;
                }
                status_quiescing = true;
            } else if status_page_active {
                status_page_ready = true;
                if !status_page_was_ready && !status_pose_latched {
                    status_hold_pose = Some(coast.known_positions(hold));
                    status_pose_latched = true;
                }
            }
        } else {
            status_maintenance_stop = None;
        }
        status_page_was_ready = status_page_active && status_page_ready;
        if status_page_ready && maintenance.is_none()
            && !state.human_control.lock().emergency_release_active()
            && state.exit_sit.load(Ordering::Acquire) == 0
            && !state.stopping.load(Ordering::Acquire)
            && shutdown_sit.is_none() && shutdown_rest.is_none() && !powered_off
            && safety.supports_individual_device_observation()
        {
            if let Err(error) = safety.write_limp_mouth(None) {
                tracing::warn!(%error, "cannot release jaw before observing the motor bus");
                continue;
            }
            // Stop before the normal control read in this very tick. The next
            // tick will read only individual devices while the guard is active.
            if let Err(error) = status_bus::begin_session(&state.status_journal) {
                tracing::error!(%error, "cannot persist status session; motor bus quarantined");
                safety.quarantine_device_bus();
                state.status_bus_fault.store(true, Ordering::Release);
                continue;
            }
            status_session_open = true;
            safety.set_individual_device_observation(true);
            safety.begin_device_observation(state.device_configuration_generation.load(Ordering::Acquire));
            continue;
        }
        // An active or returning maintenance lease may already own the bus, so
        // waiting for a fresh control read here would prevent its return forever.
        if let Some(phase) = maintenance.filter(|phase| (!status_quiescing
            || matches!(phase, human_control::MaintenancePhase::Active | human_control::MaintenancePhase::Returning))
            && !(*phase == human_control::MaintenancePhase::Failed && !maintenance_holding && !maintenance_parked)) {
            use human_control::MaintenancePhase;
            // A status request can be cancelled or fail while it is still waiting
            // for a scripted move to finish. In that case the bus was never parked;
            // returning the lease must not freeze the move or disable its policy.
            if (phase == MaintenancePhase::Requested
                || (phase == MaintenancePhase::Returning && maintenance_parked))
                && !maintenance_holding
            {
                intents.cancel_pending_rest();
                intents.finish_rest(false);
                rest_action = None;
                intents.set_enabled(false);
                maintenance_holding = true;
                hold = last_targets;
                was_driving = false;
                policy_was_enabled = false;
                resume_seated_policy = false;
                bringup = Bringup::Limp;
                shutdown_sit = None;
                shutdown_rest = None;
                exit_sit_started = None;
                seat_transition_until = None;
                limp_fall = LimpFall::Idle;
                falling.reset();
                if let Some(controller) = controller.as_mut() {
                    controller.cancel_scripted_actions();
                    controller.reset();
                }
            }
            match phase {
                MaintenancePhase::Requested => {
                    let result = safety.maintenance_handoff(true);
                    maintenance_parked = result.is_ok();
                    let (next, error) = match result {
                        Ok(()) => (MaintenancePhase::Active, None),
                        Err(error) => (MaintenancePhase::Failed, Some(error.to_string())),
                    };
                    state.human_control.lock().finish_maintenance_handoff(next, error);
                }
                MaintenancePhase::Returning => {
                    if state.human_control.lock().status_read_pending() {
                        match status_bus::recover_inflight(&state.status_journal, &params.bus.port) {
                            Ok(()) => {
                                state.human_control.lock().finish_status_recovery();
                                maintenance_reopen_error = None;
                            }
                            Err(error) => {
                                let error = error.to_string();
                                if maintenance_reopen_error.as_ref() != Some(&error) {
                                    tracing::error!(%error, "status reply unconfirmed; motor control remains stopped");
                                    maintenance_reopen_error = Some(error);
                                }
                                continue;
                            }
                        }
                    }
                    let result = if maintenance_parked {
                        if state.stopping.load(Ordering::Acquire) {
                            safety.maintenance_reclaim_for_shutdown()
                        } else {
                            safety.maintenance_handoff(false)
                        }
                    } else {
                        Ok(())
                    };
                    match result {
                        Ok(()) => {
                            maintenance_parked = false;
                            maintenance_reopen_error = None;
                            state.human_control.lock().finish_maintenance_return();
                            if state.stopping.load(Ordering::Acquire) {
                                // The shutdown-only handoff has not validated the IMU.
                                // Leave the loop without another read or motion command.
                                shutdown_after_maintenance = true;
                                break;
                            }
                        }
                        Err(error) => {
                            let error = error.to_string();
                            if maintenance_reopen_error.as_ref() != Some(&error) {
                                tracing::error!(%error, "cannot reclaim motor bus after maintenance");
                                maintenance_reopen_error = Some(error);
                            }
                        }
                    }
                }
                MaintenancePhase::Active | MaintenancePhase::Failed => {}
            }
            continue;
        }

        // Partial maintenance never clears the emergency interlock. In particular,
        // a failed or still-quiescing handoff must not fall through to the normal
        // synchronous read and its subsequent policy/actuator writes.
        if emergency_active {
            continue;
        }

        let read_at = proto::clock::monotonic_ns();
        let read_started = Instant::now();
        let read_result = safety.read_with_cycle_deadline(tick_start + period);
        let read_elapsed = read_started.elapsed();
        read_timing.mark_stage(0);
        read_timing.observe(read_result.is_ok(), Some(read_elapsed));
        let fresh = match read_result {
            Ok(sensors) => {
                let read_finished = read_started + read_elapsed;
                if read_finished > tick_start + period {
                    tracing::warn!(read_ms = read_elapsed.as_secs_f64() * 1000.0,
                        over_ms = read_finished.duration_since(tick_start + period).as_secs_f64() * 1000.0,
                        "bus read completed after cycle deadline");
                }
                let sensors = match &state.pose_calibration {
                    Some(calibration) => calibration.correct(sensors),
                    None => sensors,
                };
                status_measurement_still = sensors.velocities.iter().all(|v| v.is_finite() && v.abs() < 0.35)
                    && sensors.imu.gyro.iter().all(|v| v.is_finite() && v.abs() < 0.3);
                state.consecutive_errors.store(0, Ordering::Relaxed);
                sensors_read_ns = read_at;
                if let Some(run) = read_run.as_mut() {
                    run.recovered = true;
                }
                Some(sensors)
            }
            Err(e) => {
                let n = state.consecutive_errors.fetch_add(1, Ordering::Relaxed) + 1;
                if matches!(
                    &e,
                    duck_control::IoError::ReadFailure(failure)
                        if matches!(&failure.reason, duck_control::read_diagnostic::ReadFailureReason::Timeout)
                ) {
                    state.bus_read_timeouts.fetch_add(1, Ordering::Relaxed);
                }
                bus_drops_window += 1;
                let error = Arc::new(e);
                read_run
                    .get_or_insert_with(|| ReadRun::new(Instant::now(), Arc::clone(&error)))
                    .failed(Arc::clone(&error));

                tracing::warn!(error = error.as_ref() as &(dyn std::error::Error + 'static), consecutive = n,
                    read_ms = read_elapsed.as_secs_f64() * 1000.0, "bus read failed");
                None
            }
        };
        if status_session_open && fresh.is_none() {
            // A control read can also leave a delayed reply while status scanning.
            // The session marker remains and no subsequent instruction may be sent.
            safety.quarantine_device_bus();
            state.status_bus_fault.store(true, Ordering::Release);
            publish_status_fault(&state, safety.sample_device_observation(tick_start));
            continue;
        }
        let low_triggered = state.low_voltage.observe(fresh.as_ref().and_then(|s| s.min_servo_voltage), tick_start);
        if low_triggered || (state.low_voltage.status().triggered
            && !state.low_voltage_lock.persistence_confirmed() && tick_start >= next_low_lock_attempt)
        {
            // No status transaction is in flight here: both the control read and
            // the preceding individual read completed on this same thread.
            // Close the session before beginning any low-voltage sit motion.
            if status_session_open {
                safety.set_individual_device_observation(false);
                if let Err(error) = status_bus::complete(&state.status_journal) {
                    tracing::error!(%error, "cannot close status session for low-voltage protection");
                    safety.quarantine_device_bus();
                    state.status_bus_fault.store(true, Ordering::Release);
                    publish_status_fault(&state, safety.sample_device_observation(tick_start));
                    continue;
                }
                status_session_open = false;
            }
            let mut guard = state.human_control.lock();
            guard.emergency_stop();
            clear_human_intents(&intents);
            intents.cancel_motion_tracking();
            intents.cancel_maze();
            match state.low_voltage_lock.trip() {
                Ok(()) => {
                    // Never begin sit/release until its restart lock is durable.
                    state.release_torque_on_exit.store(true, Ordering::Release);
                    state.exit_sit.store(EXIT_SIT_REQUESTED, Ordering::Release);
                    state.low_voltage.triggered.notify_one();
                    tracing::error!(voltage = ?fresh.as_ref().and_then(|s| s.min_servo_voltage), "low voltage protection triggered");
                }
                Err(error) => {
                    next_low_lock_attempt = tick_start + Duration::from_secs(1);
                    tracing::error!(%error, "low-voltage lock not saved; stopping motion and retrying");
                }
            }
        }
        // Coast over a dropped read on the last good sample — see [`COAST_TICKS`]. The
        // distinction between "fresh" and "what the policy steps from" matters twice below:
        // safety only observes fresh samples, so a repeat cannot feed the fall debounce, and
        // `known_positions` only advances on fresh ones, so a coasted tick cannot pretend to
        // know where the joints have moved to.
        let read_failed = fresh.is_none();
        let sensors = coast.sample(fresh);
        if read_failed && let Some(run) = read_run.as_mut() {
            if sensors.is_some() {
                run.coasted_ticks += 1;
                stale_steps_window += 1;
            } else if !run.ran_out {
                // Past the coast window there is no sample left to drive from. `driving` is
                // resolved a few hundred lines below, so the flag here is last tick's; the
                // run's own summary carries the accurate one.
                run.ran_out = true;
                tracing::warn!(
                    failures = run.failures,
                    stale_ticks = COAST_TICKS,
                    last_error = run.last_error.as_ref() as &(dyn std::error::Error + 'static),
                    "no fresh sensor data: the loop is holding the last pose"
                );
            }
        }

        let coordinated_active = intents.coordinated_tracking_active();
        if !coordinated_active || fresh.is_none() {
            coordinated_feedback.clear();
        }
        // The automatic sit still uses a policy. After the handoff, the
        // action owns the joints and has no fall verdict.
        let policy_owns_motion = shutdown_rest.is_none()
            && (intents.enabled() || shutdown_sit.is_some());
        if !policy_owns_motion {
            safety.reset_fall();
        }
        if let Some(fresh) = fresh.as_ref() {
            if policy_owns_motion {
                safety.observe(fresh, period);
            }
            if coordinated_active {
                let now_mono = proto::clock::monotonic_ns();
                let now_real = proto::clock::realtime_ns();
                if let Some(read_real) = now_mono.checked_sub(sensors_read_ns)
                    .and_then(|age| now_real.checked_sub(age))
                {
                    coordinated_feedback.push_with_orientation(read_real / 1000, [
                        fresh.positions[5], fresh.positions[6], fresh.positions[7], fresh.positions[8],
                    ], if safety.imu_ready() { fresh.imu.quat } else { [0.0; 4] });
                }
            }
            // Only once the orientation filter has converged: seeding the anchor from a
            // quaternion that is still swinging would put the world origin somewhere the
            // robot never was. A coasted tick is skipped too — repeating a stale sample
            // into the estimator would tell it the robot froze, which it did not.
            if safety.imu_ready() {
                odometry.update(&fresh.positions, fresh.imu.quat);
            }
        }
        state.fallen.store(safety.fallen(), Ordering::Relaxed);
        read_timing.mark_stage(1);

        let mut snapshot = intents.snapshot();
        if snapshot.abandon_generation != previous_abandon_generation {
            // Do not combine tracking sampled before cancellation with its new generation.
            snapshot = intents.snapshot();
        }
        let now = Instant::now();
        if let Some(controller) = controller.as_mut().filter(|controller| controller.skill_timed_out(now)) {
            let _guard = state.human_control.lock();
            snapshot = intents.snapshot();
            if snapshot.enabled && state.exit_sit.load(Ordering::Acquire) == 0
                && snapshot.stop_generation == previous_stop_generation
                && let Some(skill) = controller.expire_skill(now, coast.known_positions(hold))
            {
                // Clear motion before any intent in this tick can use the old snapshot.
                intents.stop();
                intents.set_pose(intents::PoseIntent::default());
                intents.clear_head();
                intents.cancel_head_tracking();
                intents.cancel_coordinated_tracking();
                intents.cancel_motion_tracking();
                intents.cancel_maze();
                snapshot = intents.snapshot();
                hold = coast.known_positions(hold);
                state.human_other_action_busy.store(true, Ordering::Release);
                state.human_action_busy.store(true, Ordering::Release);
                state.publish_active_skill(Some(&skill));
                state.record_skill_timeout(skill.clone());
                tracing::warn!(%skill, "skill timed out after 5 seconds; recovering with zero motion");
            }
        }
        if snapshot.enabled {
            maintenance_holding = false;
        }
        let explicit_stop = policy_was_enabled && !snapshot.enabled;
        if !policy_was_enabled && snapshot.enabled {
            resume_seated_policy = bringup == Bringup::Ready
                && shutdown_sit.is_none() && state.exit_sit.load(Ordering::Acquire) == 0
                && !powered_off && controller.as_ref().is_some_and(Controller::is_sitting);
        }
        policy_was_enabled = snapshot.enabled;
        let mut requested = snapshot.command;
        let mut tracking_stopped = false;
        let mut coordinated_action_cancelled = false;
        let mut tracking_deadzone = [false; 3];
        let mut coordinated_head = None;
        let mut coordinated_eligible = false;
        let mut maze_head = None;
        let mut maze_immediate_stop = false;
        let mut maze_wait_reason = None;
        let autonomous_ready = snapshot.enabled && was_driving && bringup == Bringup::Ready
            && safety.imu_ready() && !powered_off && !snapshot.pose.active
            && shutdown_sit.is_none() && shutdown_rest.is_none()
            && mode_change.is_none() && pending_swap.is_none()
            && state.exit_sit.load(Ordering::Acquire) == 0
            && snapshot.stop_generation == previous_stop_generation
            && controller.as_ref().is_some_and(|controller| !controller.busy() && !controller.is_sitting());
        let recovery_imu = fresh.as_ref().filter(|_| autonomous_ready).map(|sensors| &sensors.imu);
        let autonomous_recovery = autonomous_fall_recovery.observe(
            safety.fallen() || limp_fall != LimpFall::Idle, recovery_imu, period,
        );
        let autonomous_paused = autonomous_recovery == maze::RecoveryState::Paused;
        if let Some(maze) = snapshot.maze {
            let params = maze.params;
            let settings = maze::Settings {
                threshold_mm: params.threshold_mm,
                head_step_deg: params.head_step_deg,
                settle: Duration::from_millis(params.settle_ms),
                sample: Duration::from_millis(params.sample_ms),
                restore: Duration::from_millis(params.restore_ms),
                turn_initial: Duration::from_millis(params.turn_initial_ms),
                turn_final: Duration::from_millis(params.turn_final_ms),
                linear_speed: params.linear_speed,
                angular_speed: params.angular_speed,
            };
            let unavailable = !snapshot.enabled || shutdown_sit.is_some() || shutdown_rest.is_some()
                || snapshot.pose.active || powered_off || state.exit_sit.load(Ordering::Acquire) != 0
                || (!autonomous_paused && controller.as_ref()
                    .is_none_or(|controller| controller.busy() || controller.is_sitting()));
            if unavailable {
                intents.cancel_maze();
                maze_controller = None;
                requested.twist = [0.0; 3];
                maze_immediate_stop = true;
            } else if autonomous_paused {
                // The standing controller can be busy getting up. Keep the
                // maze intent and restart its scan after standing is stable.
                maze_wait_reason = Some("fall_recovery");
                requested.twist = [0.0; 3];
                maze_immediate_stop = true;
            } else {
                let ready = autonomous_ready;
                if let Some(sensors) = sensors.as_ref().filter(|sensors| sensors.positions[7].is_finite()) {
                    maze_last_head = sensors.positions[7];
                    if maze_controller.as_ref().is_none_or(|(sequence, _)| *sequence != maze.sequence) {
                        maze_controller = Some((maze.sequence, maze::Maze::new(sensors.positions[7])));
                    }
                    let controller = &mut maze_controller.as_mut().expect("maze created").1;
                    if autonomous_recovery == maze::RecoveryState::Resumed {
                        controller.resume_after_fall(sensors.positions[7]);
                    }
                    let reading = if ready && fresh.is_some() {
                        state.obstacle.frame(proto::clock::monotonic_ns())
                            .map(|(distance_mm, stamp)| maze::Reading { distance_mm, stamp })
                    } else { None };
                    if !ready { maze_wait_reason = Some("control_not_ready"); }
                    else if fresh.is_none() { maze_wait_reason = Some("motor_feedback_unavailable"); }
                    else if reading.and_then(|frame| frame.distance_mm).is_none() { maze_wait_reason = Some("tof_invalid_or_stale"); }
                    let output = if ready && fresh.is_some() {
                        controller.tick(
                            tick_start, &settings, reading, sensors.positions[7],
                            twist_ema.iter().all(|value| *value == 0.0),
                        )
                    } else {
                        controller.hold_on_failure(sensors.positions[7])
                    };
                    requested.twist = intents.limit_twist(output.twist);
                    maze_head = if ready { output.head } else { None };
                    maze_immediate_stop = output.immediate_stop;
                } else {
                    maze_wait_reason = Some("motor_feedback_unavailable");
                    if let Some((_, controller)) = maze_controller.as_mut() {
                        controller.hold_on_failure(maze_last_head);
                    }
                    requested.twist = [0.0; 3];
                    maze_immediate_stop = true;
                }
            }
        } else if maze_controller.take().is_some() {
            requested.twist = [0.0; 3];
            maze_immediate_stop = true;
        }
        let phase = if maze_wait_reason == Some("fall_recovery") {
            Some("fall_recovery")
        } else {
            maze_controller.as_ref().map(|(_, controller)| controller.phase())
        };
        let maze_state = phase.map(|phase| (phase, maze_wait_reason));
        if maze_state != maze_previous_phase {
            tracing::info!(phase = phase.unwrap_or("stopped"), reason = maze_wait_reason, "maze state changed");
            maze_previous_phase = maze_state;
        }
        if let Some(tracking) = snapshot.coordinated_tracking.as_ref() {
            let head_only = tracking.params.head_only();
            if coordinated_tracking_sequence != Some(tracking.sequence)
                || coordinated_head_only_mode != Some(head_only)
            {
                if coordinated_tracker.arrival_action_running() {
                    cancel_arrival_skill(controller.as_mut(), &mut coordinated_unwritten_sit);
                    coordinated_action_cancelled = true;
                }
                coordinated_tracker.reset();
                target_head_tracker.reset();
                let pickup_handoff = tracking.pickup_stage == Some(proto::PickupStage::After)
                    && coordinated_tracking_sequence.is_some_and(|sequence|
                        pickup_action_completed == Some((sequence, snapshot.stop_generation)));
                pickup_action_completed = None;
                target_follow_stability.reset();
                target_head_rate.reset();
                target_geometry_missing = false;
                if !pickup_handoff { coordinated_mouth.reset(); }
                let staged_target = !head_only && tracking.params.stop_area_px.is_none();
                let initial_pitch = tracking.initial_pitch
                    .or(staged_target.then_some(coordinated_tracking::INITIAL_PITCH));
                coordinated_initial_head = coordinated_tracking::InitialHead::new(initial_pitch);
                tracing::info!(servo = 31, target = ?initial_pitch, "coordinated tracking initializing head");
                coordinated_tracking_sequence = Some(tracking.sequence);
                coordinated_head_only_mode = Some(head_only);
                coordinated_tick = None;
            }
            if coordinated_tracker.arrival_action_running()
                && let Some(controller) = controller.as_mut()
            {
                controller.release_completed_roll();
            }
            let eligible = autonomous_ready && !autonomous_paused && fresh.is_some()
                && controller.as_ref().is_some_and(|controller|
                    !coordinated_tracker.arrival_action_running() || !controller.scripted_active());
            let paused = !tracking.fresh || tracking.params.camera.is_none();
            coordinated_eligible = eligible && !paused;
            if paused != coordinated_paused {
                if paused {
                    tracing::warn!(stale = !tracking.fresh, camera = tracking.params.camera.is_some(),
                        "coordinated tracking paused; awaiting fresh vision");
                } else {
                    tracing::info!("coordinated tracking resumed with fresh vision");
                }
                coordinated_paused = paused;
            }
            if paused {
                target_head_tracker.reset();
                target_follow_stability.reset();
                target_geometry_missing = false;
                coordinated_mouth.reset();
                if coordinated_tracker.arrival_action_running() {
                    cancel_arrival_skill(controller.as_mut(), &mut coordinated_unwritten_sit);
                    coordinated_action_cancelled = true;
                }
                if let Some(sensors) = fresh.as_ref().filter(|_| eligible) {
                    let joints = [sensors.positions[5], sensors.positions[6], sensors.positions[7], sensors.positions[8]];
                    let paused = match (tracking.params.stop_area_px, coordinated_initial_head.fixed_head()) {
                        (Some(_), Some(head)) => coordinated_tracker.pause_fixed(head, false),
                        _ => coordinated_tracker.pause(joints),
                    };
                    if let coordinated_tracking::Step::Holding { head, .. } = paused {
                        coordinated_head = Some(head);
                    }
                } else {
                    coordinated_tracker.reset();
                }
                coordinated_initial_head.pause();
                coordinated_tick = None;
                coordinated_feedback_missing = false;
                requested.twist = [0.0; 3];
                tracking_stopped = true;
            } else if coordinated_eligible && !coordinated_tracker.arrived()
                && pickup_action_completed == Some((tracking.sequence, snapshot.stop_generation))
            {
                // Continue the handoff if a vision or feedback interruption cleared the arrival.
                requested.twist = [0.0; 3];
                tracking_stopped = true;
            } else if let Some(sensors) = fresh.as_ref().filter(|_| eligible) {
                let joints = [sensors.positions[5], sensors.positions[6], sensors.positions[7], sensors.positions[8]];
                if coordinated_tracker.arrival_action_running()
                    && sensors.positions[duck_control::model::MOUTH_INDEX].is_finite()
                {
                    coordinated_mouth.action_finished(tracking.params.mouth_after,
                        sensors.positions[duck_control::model::MOUTH_INDEX]);
                    coordinated_tracker.finish_arrival_action([joints[1], joints[2]]);
                    if tracking.params.pickup_after.is_some() {
                        pickup_action_completed = Some((tracking.sequence, snapshot.stop_generation));
                    }
                }
                let elapsed = coordinated_tick
                    .map(|previous| tick_start.saturating_duration_since(previous).as_secs_f64())
                    .unwrap_or(dt);
                coordinated_tick = Some(tick_start);
                if tracking.params.stop_area_px.is_none() {
                    coordinated_tracker.set_target_motion_settled(
                        target_follow_motion_settled(twist_ema, sensors.imu.gyro));
                }
                let output = coordinated_initial_head.step(joints, elapsed)
                    .map_err(|error| error.to_string())
                    .and_then(|initial| match initial {
                        Some(hold) => Ok(hold),
                        None => {
                            if head_only {
                                let _ = coordinated_initial_head.take_handoff();
                                let target_point = tracking.params.head_target_point.unwrap_or(proto::HEAD_TRACKING_DEFAULT_POINT);
                                let error = tracking.params.target.map(|target| [
                                    target.point[0] - (target_point[0] / 160.0 - 1.0),
                                    target.point[1] - (target_point[1] / 90.0 - 1.0),
                                ]);
                                target_head_tracker.step([joints[1], joints[2]], error,
                                    tracking.params.head_deadzone, tracking.params.head_gains, elapsed.min(0.1))
                                    .map(|head| coordinated_tracking::Step::Following {
                                        head, twist: [0.0; 3], deadzone: [true; 3],
                                    }).ok_or_else(|| "invalid head tracking state".to_owned())
                            } else {
                                if let Some(head) = coordinated_initial_head.take_handoff() {
                                    coordinated_tracker.seed_head(head);
                                } else if tracking.params.stop_area_px.is_some()
                                    && let Some(head) = coordinated_initial_head.fixed_head()
                                {
                                    coordinated_tracker.fix_head(head);
                                }
                                coordinated_tracking::geometry().map_err(|error| error.to_string())
                                .and_then(|geometry| coordinated_tracker.step(
                                    geometry, &coordinated_feedback, joints, tracking.params.clone(), elapsed,
                                    tracking_limits,
                                ).map_err(|error| error.to_string()))
                            }
                        },
                    });
                match output {
                    Ok(coordinated_tracking::Step::Following { mut head, mut twist, mut deadzone }) => {
                        if coordinated_feedback_missing {
                            tracing::info!("coordinated tracking feedback aligned");
                            coordinated_feedback_missing = false;
                        }
                        if head_only {
                            // This mode owns only the head, including after a manual move or loss.
                            tracking_stopped = true;
                            target_follow_stability.reset();
                        } else if tracking.params.stop_area_px.is_none() {
                            let phase = coordinated_tracker.phase();
                            let needs_geometry = tracking.params.target.is_some()
                                && matches!(phase, proto::CoordinatedTrackingPhase::Approach
                                    | proto::CoordinatedTrackingPhase::FinalAlignment
                                    | proto::CoordinatedTrackingPhase::Confirming);
                            let target_geometry = if needs_geometry {
                                match coordinated_tracking::geometry()
                                    .map_err(|error| error.to_string())
                                    .and_then(|geometry| coordinated_tracker.target_geometry(geometry, joints)
                                        .map_err(|error| error.to_string()))
                                {
                                    Ok(Some(frame)) if phase != proto::CoordinatedTrackingPhase::Approach
                                        || frame.bearing_rad.is_some() => {
                                        if target_geometry_missing {
                                            tracing::info!("target tracking geometry recovered");
                                            target_geometry_missing = false;
                                        }
                                        Some(frame)
                                    }
                                    Ok(Some(_)) | Ok(None) | Err(_) => {
                                        if !target_geometry_missing {
                                            tracing::warn!("target tracking is waiting for valid target geometry");
                                            target_geometry_missing = true;
                                        }
                                        None
                                    }
                                }
                            } else { None };
                            if needs_geometry && target_geometry.is_none() {
                                // Unknown target direction is a safety stop, not a normal
                                // deadzone release that should coast through the EMA.
                                tracking_stopped = true;
                            }
                            (twist, deadzone) = target_follow_stability.apply_with_dt(
                                phase, target_geometry, &tracking.params, twist, tracking_limits,
                                Duration::try_from_secs_f64(elapsed).unwrap_or(Duration::MAX),
                                target_follow_motion_settled(twist_ema, sensors.imu.gyro)
                                    && controller.as_ref().is_some_and(Controller::target_correction_policy_ready));
                            if coordinated_initial_head.arrived() && !coordinated_tracker.arrived() {
                                head = target_head_rate.limit(head, [joints[1], joints[2]], elapsed);
                            }
                        }
                        coordinated_head = Some(head);
                        tracking_deadzone = deadzone;
                        requested.twist = twist;
                    }
                    Ok(coordinated_tracking::Step::Holding { head, missing_feedback }) => {
                        target_follow_stability.reset();
                        if missing_feedback && !coordinated_feedback_missing {
                            tracing::warn!("coordinated tracking is waiting for capture-time head feedback");
                        }
                        coordinated_feedback_missing = missing_feedback;
                        coordinated_head = Some(head);
                        requested.twist = [0.0; 3];
                        tracking_stopped = true;
                    }
                    Err(error) => {
                        tracing::error!(%error, "coordinated tracking stopped");
                        coordinated_tracker.reset();
                        target_head_tracker.reset();
                        target_follow_stability.reset();
                        coordinated_mouth.reset();
                        intents.cancel_coordinated_tracking();
                        requested.twist = [0.0; 3];
                        tracking_stopped = true;
                    }
                }
            } else {
                target_head_tracker.reset();
                target_follow_stability.reset();
                target_head_rate.reset();
                let arrival_action_owns_motion = coordinated_tracker.arrival_action_running()
                    && snapshot.enabled && !autonomous_paused && limp_fall == LimpFall::Idle
                    && !snapshot.pose.active && bringup == Bringup::Ready
                    && shutdown_sit.is_none() && shutdown_rest.is_none() && !powered_off
                    && state.exit_sit.load(Ordering::Acquire) == 0
                    && mode_change.is_none() && pending_swap.is_none()
                    && snapshot.stop_generation == previous_stop_generation;
                if !arrival_action_owns_motion && (fresh.is_some() || !coordinated_tracker.arrived()) {
                    if coordinated_tracker.arrival_action_running() {
                        cancel_arrival_skill(controller.as_mut(), &mut coordinated_unwritten_sit);
                        coordinated_action_cancelled = true;
                    }
                    coordinated_tracker.reset();
                    coordinated_mouth.reset();
                }
                coordinated_tick = None;
                requested.twist = [0.0; 3];
                tracking_stopped = true;
            }
        } else if coordinated_tracking_sequence.take().is_some() {
            if coordinated_tracker.arrival_action_running() {
                cancel_arrival_skill(controller.as_mut(), &mut coordinated_unwritten_sit);
                coordinated_action_cancelled = true;
                tracing::warn!("coordinated arrival action cancelled after tracking ended");
            }
            coordinated_tracker.reset();
            pickup_action_completed = None;
            target_head_tracker.reset();
            coordinated_head_only_mode = None;
            target_follow_stability.reset();
            target_head_rate.reset();
            target_geometry_missing = false;
            coordinated_mouth.reset();
            coordinated_tick = None;
            coordinated_feedback_missing = false;
            coordinated_paused = false;
            requested.twist = [0.0; 3];
            tracking_stopped = true;
        }
        if let Some(tracking) = snapshot.motion_tracking {
            if motion_tracking_sequence != Some(tracking.sequence) {
                motion_tracker.reset();
                motion_tracking_sequence = Some(tracking.sequence);
            }
            let eligible = autonomous_ready && !autonomous_paused && fresh.is_some();
            let target = if eligible { tracking.target } else { None };
            let twist = if let Some(speed) = tracking.line_speed {
                motion_tracker.follow_line(target, speed, tracking.line_target_x.unwrap_or(160.0),
                    tracking.deadzone[1], tracking.gains[1], dt)
            } else {
                motion_tracker.step(target, tracking.reference_area, tracking.deadzone, tracking.gains, dt)
            };
            if let Some(output) = twist {
                if output.twist.iter().all(|value| value.is_finite()) {
                    tracking_deadzone = [output.in_deadzone[0], false, output.in_deadzone[1]];
                    requested.twist = intents.limit_twist(output.twist);
                } else {
                    tracing::error!("non-finite movement tracking output");
                    motion_tracker.reset();
                    requested.twist = [0.0; 3];
                    tracking_stopped = true;
                }
            } else {
                motion_tracker.reset();
                requested.twist = [0.0; 3];
                tracking_stopped = true;
            }
        } else if motion_tracking_sequence.take().is_some() {
            motion_tracker.reset();
            requested.twist = [0.0; 3];
            tracking_stopped = true;
        }
        // Person following uses only forward/backward and body yaw, including during
        // initial positioning, target loss and transitions from manual movement.
        let person_follow = snapshot.coordinated_tracking.as_ref()
            .is_some_and(|tracking| tracking.params.stop_area_px.is_some());
        let target_follow = snapshot.coordinated_tracking.as_ref()
            .is_some_and(|tracking| tracking.params.stop_area_px.is_none());
        let recording = recorder.as_ref().is_some_and(|recorder| recorder.should_record(person_follow, target_follow));
        if person_follow {
            requested.twist[1] = 0.0;
            tracking_deadzone[1] = true;
        }
        let twist_age = if snapshot.motion_tracking.is_some() || snapshot.coordinated_tracking.is_some()
            || snapshot.maze.is_some() { Duration::ZERO } else { snapshot.twist_age };
        let (gated, deadman) = safety.gate(requested, twist_age);
        let mut limits: Vec<duck_control::safety::Limit> = deadman.into_iter().collect();

        if snapshot.stop_generation != previous_stop_generation
            && let Some(controller) = controller.as_mut()
        {
            controller.cancel_scripted_actions();
        }

        // An explicit `robot.init` / `robot.relax`, taken once.
        //
        // Before the enable-driven bring-up below, so a `relax` that arrives in the same tick as a
        // still-set `enabled` flag wins — `request_relax` clears that flag, and reading the request
        // first means the order cannot invert.
        let power_request = intents.take_power_request().filter(|_| state.exit_sit.load(Ordering::Acquire) == 0);
        let mut torque_only_initialized = false;
        match power_request {
            // The power-off has been asked for and torque is off. Standing the robot up now would
            // hand the poweroff a robot mid-ramp, stiff, which is the outcome the sit exists to
            // avoid.
            Some(intents::PowerRequest::Init) if powered_off => {
                tracing::warn!("robot.init ignored: the robot is powering off")
            }
            Some(intents::PowerRequest::Init)
                if params.bus.protocol == BusProtocol::Hx && controller.is_none() =>
            {
                tracing::error!("robot.init refused: no validated policy joint HOME");
            }
            Some(intents::PowerRequest::Init) => {
                let guard = state.human_control.lock();
                if guard.emergency_release_active() {
                    tracing::warn!("robot.init ignored: torque release is in progress");
                } else {
                    maintenance_holding = false;
                    match (bringup, sensors.as_ref()) {
                        // A bench robot can be initialized without a walking policy.
                        (Bringup::Limp, Some(sensors)) => match (|| {
                            if state.init_torque_only {
                                let current = fresh.as_ref().ok_or_else(|| duck_control::io::IoError::Bus(
                                    "current-pose initialization requires fresh joint readings".into()))?;
                                safety.prepare_torque_only(&current.positions)?;
                            }
                            safety.set_torque(true)
                        })() {
                            Ok(()) => {
                                if state.init_torque_only {
                                    hold = sensors.positions;
                                    bringup = Bringup::Ready;
                                    torque_only_initialized = true;
                                    if let Some(controller) = controller.as_mut() { controller.forget(); }
                                    tracing::warn!("robot.init: torque on, holding current pose");
                                } else {
                                    tracing::warn!(?HOME_RAMP, "robot.init: torque on, ramping to home");
                                    bringup = Bringup::Homing {
                                        from: sensors.positions,
                                        since: tick_start,
                                    };
                                }
                            }
                            Err(e) => tracing::warn!(error = %e, "cannot enable torque"),
                        },
                        (Bringup::Ready, Some(_))
                            if controller.as_ref().is_some_and(Controller::is_sitting) =>
                        {
                            tracing::warn!("robot.init: holding the seated pose")
                        }
                        // Return to home before handing a stopped robot back to a policy.
                        (Bringup::Ready, Some(sensors)) => {
                            if state.init_torque_only {
                                hold = sensors.positions;
                                torque_only_initialized = true;
                                tracing::info!("robot.init: already powered, holding current pose");
                            } else {
                                tracing::warn!(?HOME_RAMP, "robot.init: re-homing from the current pose");
                                bringup = Bringup::Homing {
                                    from: sensors.positions,
                                    since: tick_start,
                                };
                            }
                        }
                        // Mid-ramp or without a sample, there is nothing to bring up.
                        (state, _) => tracing::info!(?state, "robot.init: nothing to bring up"),
                    }
                }
            }
            Some(intents::PowerRequest::Relax) => {
                intents.cancel_pending_rest();
                if !shutdown_power_off {
                    intents.finish_rest(false);
                    shutdown_sit = None;
                    shutdown_rest = None;
                    shutdown_power_off = true;
                    rest_action = None;
                }
                match safety.set_torque(false) {
                    Ok(()) => {
                        tracing::warn!("robot.relax: torque off");
                        // Back to the start, so the next `init` or Start ramps from wherever the robot
                        // ends up rather than assuming it is still at the home pose.
                        bringup = Bringup::Limp;
                        was_driving = false;
                        if let Some(controller) = controller.as_mut() {
                            controller.forget();
                        }
                    }
                    Err(e) => {
                        tracing::warn!(error = %e, "cannot cut torque; the robot is still powered")
                    }
                }
            }
            None => {}
        }

        // `robot.rebootMotors`: torque off everywhere, then REBOOT the named servos (every servo
        // when none are named). The rebooted servos are off the bus for a few hundred milliseconds
        // — the reads fail and the loop coasts through it — and come back with torque off and
        // EEPROM gains; the forgotten gain cache makes the next write restore the gains, and the
        // robot is back at limp, so the next `init` or Start brings it up like a fresh boot. Torque
        // off first on purpose: a tripped servo is usually a leg, and a robot standing on the other
        // leg while one reboots is not a robot to leave standing.
        if let Some(ids) = intents.take_reboot_motors() {
            let ids: Vec<u8> = if ids.is_empty() {
                duck_control::model::JOINT_IDS.to_vec()
            } else {
                ids
            };
            if let Err(error) = safety.set_torque(false) {
                tracing::warn!(%error, "robot.rebootMotors: torque off failed on a servo");
            }
            if let Err(error) = safety.reboot_motors(&ids) {
                tracing::warn!(%error, ?ids, "robot.rebootMotors: failed part-way");
            } else {
                tracing::warn!(?ids, "robot.rebootMotors: rebooted; init or Start brings the robot up");
            }
            bringup = Bringup::Limp;
            state.homed.store(false, Ordering::Release);
            intents.cancel_pending_rest();
            intents.finish_rest(false);
            rest_action = None;
            was_driving = false;
            if let Some(controller) = controller.as_mut() {
                controller.forget();
            }
            if !shutdown_power_off {
                shutdown_sit = None;
                shutdown_rest = None;
                shutdown_power_off = true;
            }
        }

        // One-shot skill requests, taken once per tick like the power request. They need a
        // driving robot — the prototype's buttons likewise did nothing until the policy ran.
        let mut manual_action_started = false;
        {
            let guard = state.human_control.lock();
            if let Some(request) = intents.take_manual_skill()
                && !request.reply.is_closed()
            {
                let result = if Instant::now() >= request.deadline {
                    Err("the skill request timed out before it could start")
                } else if state.exit_sit.load(Ordering::Acquire) != 0
                    || guard.maintenance_phase().is_some() || guard.emergency_release_active()
                    || guard.status_blocked() || guard.maintenance_transition_blocked()
                    || intents.stop_generation() != snapshot.stop_generation
                {
                    Err("the skill request was stopped")
                } else if !intents.enabled() || bringup != Bringup::Ready || shutdown_sit.is_some()
                    || resume_seated_policy || mode_change.is_some() || pending_swap.is_some()
                    || !was_driving || fresh.is_none() || !safety.imu_ready()
                    || limp_fall != LimpFall::Idle || powered_off
                {
                    Err("the policy is not ready")
                } else if seat_transition_until.is_some_and(|until| tick_start < until) {
                    Err("a scripted move is already running")
                } else if let Some(controller) = controller.as_mut() {
                    start_requested_skill(controller, &request.name, tick_start, &mut seat_transition_until)
                } else {
                    Err("the policy is not ready")
                };
                let result = match result {
                    Ok(()) => {
                        manual_action_started = true;
                        state.publish_active_skill(Some(&request.name));
                        tracing::info!(skill = request.name, "skill started");
                        proto::IntentResult::accepted()
                    }
                    Err(reason) => proto::IntentResult::refused(reason),
                };
                let _ = request.reply.send(result);
            }
        }
        let requests = intents.take_skills();
        if !manual_action_started && requests.any() && state.exit_sit.load(Ordering::Acquire) == 0 {
            match controller.as_mut() {
                Some(controller)
                    if snapshot.enabled && bringup == Bringup::Ready && shutdown_sit.is_none()
                        && !resume_seated_policy =>
                {
                    let outcome = |what: &str, result: Result<(), &'static str>| match result {
                        Ok(()) => tracing::warn!(skill = what, "skill started"),
                        Err(reason) => tracing::warn!(skill = what, reason, "skill refused"),
                    };
                    if requests.ground_pick {
                        let result = controller.start_ground_pick();
                        manual_action_started |= result.is_ok();
                        outcome("ground_pick", result);
                    }
                    if requests.sit_toggle {
                        match controller.sit_toggle() {
                            Ok(direction) => {
                                manual_action_started = true;
                                seat_transition_until = Some(tick_start + Duration::from_secs_f64(controller.shutdown_sit_secs()));
                                tracing::warn!(direction, "sit toggle");
                            }
                            Err(reason) => tracing::warn!(reason, "sit toggle refused"),
                        }
                    }
                    // The configurable one-shots, in priority order. A held button lands here
                    // every tick, so only the *start* of one is journal-worthy: `Ok(false)` is a
                    // chain refresh and stays quiet.
                    for index in requests.requested() {
                        let name = controller
                            .skill_names()
                            .get(index)
                            .cloned()
                            .unwrap_or_else(|| index.to_string());
                        match controller.start_skill(index) {
                            Ok(true) => {
                                manual_action_started = true;
                                tracing::warn!(skill = %name, "skill started");
                            }
                            Ok(false) => {}
                            Err(reason) => {
                                tracing::debug!(skill = %name, reason, "skill refused");
                            }
                        }
                    }
                }
                _ => tracing::info!("skill request ignored: the policy is not driving"),
            }
        }

        // Held sounds own the PCM ahead of one-shots, including custom clips.
        if let Some(voice) = voice.as_mut() {
            voice.set_mouth_enabled(state.mouth_sound_enabled.load(Ordering::Relaxed));
            let hold = if powered_off {
                intents::SoundHold::Wheee(intents::WheeeHold::Released)
            } else {
                intents.sound_hold()
            };
            voice.theremin_settle();
            voice.update_hold(hold);
            for tag in intents.take_sounds() {
                voice.play(tag.as_str(), false);
            }
            if let Some(name) = intents.take_sound_clip() {
                voice.play_clip(Arc::from(name.as_str()));
            }
            voice.poll_clip();
            voice.repeat_mouth_if_idle(
                !powered_off && !status_page_active && maintenance.is_none()
                    && shutdown_sit.is_none() && shutdown_rest.is_none()
                    && !state.stopping.load(Ordering::Acquire) && !state.low_voltage_lock.requires_rearm()
                    && intents.mouth_sound_repeating(),
                tick_start,
            );

            // Petting: coo, exactly when the prototype coos — not fallen, no scripted move
            // in flight. The verdict is used bare here (not the armed fall gate): this is a
            // sound cue, and cooing while face-down would be worse than staying quiet.
            if let Some(pet) = pet.as_ref() {
                while let Some(ev) = pet.try_recv_event() {
                    match ev {
                        pet_detect::PettingEvent::Start => {
                            let calm =
                                !safety.fallen() && controller.as_ref().is_none_or(|c| !c.busy());
                            if calm {
                                tracing::info!("petting started");
                                voice.play("coo", false);
                            } else {
                                tracing::debug!("petting detected (ignored: busy or down)");
                            }
                        }
                        pet_detect::PettingEvent::End => tracing::debug!("petting ended"),
                    }
                }
                // Ambient sound events have no consumer until the autonomous brain arrives;
                // surfaced at debug so mic tuning on a bench has data to look at.
                while let Some(ev) = pet.try_recv_sound() {
                    tracing::debug!(event = ?ev, "ambient sound");
                }
            }
        }

        // A drive-mode switch: `robot.setMode`, which the pad's held D-pad up sends.
        //
        // The robot goes back to its home pose first and the policies load there, for the reason
        // the prototype relaunched into pose 0: swapping the network under a moving gait means the
        // next tick is a different policy's idea of what the legs were doing. Torque stays on
        // throughout — the robot holds itself up across the swap rather than sitting down.
        if let Some(code) = intents.take_mode_switch() {
            let target = mode_of(code);
            if target == policy_params.mode {
                tracing::info!(
                    mode = target.as_str(),
                    "already in that mode; nothing to switch"
                );
            } else if mode_change.is_some() {
                tracing::warn!(mode = target.as_str(), "a mode switch is already in flight");
            } else if pending_swap.is_some() || state.managed_models.unresolved() {
                // The other half of the guard on the policy change below. A change is derived
                // from the params that are running, so a switch accepted now would rebuild
                // everything at the home pose and then have the load land on top of it, putting
                // the old mode's params, networks and slot report back with `robot.mode` still
                // saying the new one.
                tracing::warn!(
                    mode = target.as_str(),
                    "mode switch refused: a policy change is still loading; ask again when it lands"
                );
            } else if shutdown_sit.is_some() || powered_off {
                // The robot is on its way down. Homing from inside the sit would stand it back
                // up at gain, and the sit would then cut the torque out from under it: the shape
                // of #159, by a third door after `robot.init` and the enable-driven bring-up.
                tracing::warn!(
                    mode = target.as_str(),
                    "mode switch refused: the robot is shutting down"
                );
            } else {
                // What happens next depends on where the robot is, so the line a human reads off
                // it should too: a limp robot swaps where it stands and does not move, and saying
                // "going home" about that describes a ramp nobody is going to see.
                if bringup == Bringup::Limp {
                    tracing::warn!(
                        from = policy_params.mode.as_str(),
                        to = target.as_str(),
                        "mode switch: swapping the policies now; the robot is limp and stays limp"
                    );
                } else {
                    tracing::warn!(
                        from = policy_params.mode.as_str(),
                        to = target.as_str(),
                        "mode switch: going home before loading the other policies"
                    );
                }
                // The prototype's cue, and worth keeping: one quack for walking, two for roller,
                // so the robot says which mode it is going to without anybody reading a log.
                if let Some(voice) = voice.as_mut() {
                    voice.play("chirp", false);
                    if target == Mode::Roller {
                        voice.play("chirp", false);
                    }
                }
                mode_change = Some(target);
            }
        }

        // A switch in flight, taken from whatever state the robot is in. Every tick rather than
        // only on the tick the request landed, because the two things it needs are not always
        // there on that tick, and a request that could not be acted on must wait rather than
        // vanish: it used to arm `mode_change` and do nothing else on a blind tick, and nothing
        // looked at `mode_change` again until the robot next homed for some other reason, with
        // every later request refused as "already in flight".
        //
        // Waiting is what makes "the robot went down while a switch was pending" a state of its
        // own, and the gate for it belongs here rather than in the arms below: every state the
        // robot can reach while waiting gets the same answer, and none of them can be written
        // without one. The request site refuses a switch on a robot that is shutting down; a
        // switch that was already waiting when the shutdown began is that same request arriving
        // late, and `robot.init` is turned away on this path for the same reason.
        if let Some(target) = mode_change {
            if shutdown_sit.is_some() || powered_off {
                mode_change = None;
                tracing::warn!(
                    mode = target.as_str(),
                    "mode switch dropped: the robot is shutting down"
                );
            } else {
                match bringup {
                    // Ramping. The end of the ramp does the swap, with the robot still at home.
                    Bringup::Homing { .. } => {}
                    // Nothing is moving, so there is no gait to protect and no home to ramp to.
                    // Swap now and stay limp: `Ready` means torque on and at home, and a ramp
                    // written to servos nobody powered is neither. Going through `Homing` from
                    // here used to leave the robot `Ready` with no torque write, so the next
                    // Start found no bring-up to do and the policy drove a robot that could
                    // not move.
                    Bringup::Limp => {
                        mode_change = None;
                        load_mode(
                            target,
                            &mut policy_params,
                            &mut policy_cfg,
                            &mut controller,
                            params.safety.limp_fall,
                            &state,
                            &slot_errors,
                            params.bus.protocol,
                            Some(policy_home),
                        );
                    }
                    // Home the robot with the machinery `init` and a fall recovery already use: it
                    // ramps per tick, and `driving` is false until it reaches Ready. From a sample,
                    // which is where the ramp starts, so a blind tick waits for the next one.
                    Bringup::Ready => {
                        if let Some(sensors) = sensors.as_ref() {
                            bringup = Bringup::Homing {
                                from: sensors.positions,
                                since: tick_start,
                            };
                        }
                    }
                }
            }
        }

        // A policy change: `robot.loadPolicy`. The networks load on their own thread, and where
        // the swap then happens depends on what is driving — see `change_disturbs`.
        //
        // A change to the network that has the robot takes the mode switch's path and for the
        // same reason: swapping a network under a moving gait means the next tick is a different
        // policy's idea of what the legs were doing. So the robot goes home first and the swap
        // happens there, torque on throughout. A change to any other network is swapped in
        // wherever the robot is, carrying the running state across, so a `walk` reset under a
        // seated robot is a config change and not a stand-up.
        if let Some(change) = intents.take_policy_change() {
            if mode_change.is_some() || pending_swap.is_some() {
                tracing::warn!("a policy change is already in flight; ignoring this one");
            } else if shutdown_sit.is_some() {
                tracing::warn!(
                    change = describe_change(&change),
                    "policy change refused: the robot is shutting down"
                );
            } else if let Some(candidate) = candidate_params(
                &change,
                &policy_params,
                &params_path,
                &mut slot_errors,
                &policy_overrides,
            ) {
                let cfg = candidate.resolved();
                // Only a network that stepped last tick counts as driving: a robot that is
                // disabled, limp or mid-ramp has nothing running that a swap could interrupt.
                let driving = if was_driving {
                    controller.as_ref().and_then(|c| c.driving())
                } else {
                    None
                };
                let disturbs = controller.as_ref().is_some_and(Controller::precision_active)
                    || change_disturbs(driving.as_ref(), &change, &policy_cfg, &cfg);
                let (tx, rx) = std::sync::mpsc::sync_channel(1);
                let load_cfg = cfg.clone();
                let limp_fall = params.safety.limp_fall;
                let protocol = params.bus.protocol;
                let spawned = std::thread::Builder::new()
                    .name("policy-load".to_owned())
                    .spawn(move || {
                        // A receiver that has gone away is a loop that has stopped; nothing
                        // to do with the result then.
                        let _ = tx.send(
                            try_controller(&load_cfg, limp_fall, protocol, Some(policy_home))
                                .map_err(|e| e.to_string()),
                        );
                    });
                match spawned {
                    Err(e) => tracing::error!(
                        change = describe_change(&change),
                        error = %e,
                        "cannot start loading the policy; keeping what is running"
                    ),
                    Ok(_) => {
                        if disturbs {
                            tracing::warn!(
                                change = describe_change(&change),
                                driving = %driving.as_ref().map_or_else(String::new, |d| d.to_string()),
                                "policy change replaces the network driving: going home before swapping"
                            );
                            // Home first only when the robot is actually up. The ramp exists to
                            // stop a network being swapped under a moving gait, and a robot
                            // lying limp on a bench has no gait to protect — while standing one
                            // up because somebody loaded a file would be a surprise of its own,
                            // and on a table a nasty one. So the swap below waits for the ramp
                            // when there is one and happens as soon as the load lands when
                            // there is not.
                            if bringup == Bringup::Ready
                                && let Some(sensors) = sensors.as_ref()
                            {
                                bringup = Bringup::Homing {
                                    from: sensors.positions,
                                    since: tick_start,
                                };
                            }
                        } else {
                            tracing::warn!(
                                change = describe_change(&change),
                                driving = %driving.as_ref().map_or_else(|| "nothing".to_owned(), |d| d.to_string()),
                                "policy change leaves the network driving alone: swapping in place"
                            );
                        }
                        pending_swap = Some(PendingSwap {
                            change,
                            candidate,
                            cfg,
                            disturbs,
                            loading: rx,
                            built: None,
                        });
                    }
                }
            }
        }
        if let Some(pending) = pending_swap.as_mut() {
            pending.poll();
        }

        // The complete replacement is built and warmed on a worker. The loop never opens a
        // model or writes configuration, and does not start a home ramp on a rejected file.
        if let Some(request) = state.managed_models.take() {
            if mode_change.is_some() || pending_swap.is_some() || managed_swap.is_some()
                || shutdown_sit.is_some() || powered_off
            {
                state.managed_models.finish(&request.operation, "failed", Some("另一个策略操作正在进行或机器人正在关闭".into()));
            } else {
                match managed_candidate(&policy_params, &request.model, &request.path) {
                    Err(error) => state.managed_models.finish(&request.operation, "failed", Some(error)),
                    Ok(candidate) => {
                        let cfg = candidate.resolved();
                        let load_cfg = cfg.clone();
                        let file = request.path.clone();
                        let model = request.model.clone();
                        let operation_id = request.operation.id.clone();
                        let expected_candidate_digest = request.candidate_sha256.clone();
                        let config_path = params_path.clone();
                        let current_digest = state.managed_models.models.load().iter()
                            .find(|item| item.id == model).map(|item| item.sha256.clone());
                        if current_digest.as_deref() != Some(request.expected_sha256.as_str()) {
                            state.managed_models.finish(&request.operation, "failed", Some("模型已变化，请刷新后重试".into()));
                        } else {
                        let limp_fall = params.safety.limp_fall;
                        let protocol = params.bus.protocol;
                        let (tx, rx) = std::sync::mpsc::sync_channel(1);
                        let started = std::thread::Builder::new().name("model-prepare".into()).spawn(move || {
                            use sha2::{Digest, Sha256};
                            let result = (|| {
                                let bytes = std::fs::read(&file).map_err(|e| format!("{}: {e}", file.display()))?;
                                let digest = Sha256::digest(&bytes).iter().map(|b| format!("{b:02x}")).collect::<String>();
                                if digest != expected_candidate_digest {
                                    return Err("模型文件摘要与上传内容不一致".to_owned());
                                }
                                if current_digest.as_deref() == Some(digest.as_str()) {
                                    return Ok(None);
                                }
                                let controller = try_controller_with_candidate(&load_cfg, limp_fall, protocol,
                                    Some(policy_home), Some((&file, &bytes))).map_err(|e| e.to_string())?;
                                if controller.is_none() { return Err("策略未启用".to_owned()); }
                                let (staged, destination) = managed_models::stage(&config_path, &operation_id, &model, &file, &digest)?;
                                Ok(Some(ManagedPrepared { controller, staged, destination }))
                            })();
                            let _ = tx.send(result);
                        });
                        match started {
                            Ok(_) => managed_swap = Some(ManagedSwap {
                                request, candidate, cfg, loading: Some(rx), prepared: None,
                                committing: None, committed: false, disturbs: false, home_started: false,
                            }),
                            Err(error) => state.managed_models.finish(&request.operation, "failed", Some(error.to_string())),
                        }
                        }
                    }
                }
            }
        }
        if let Some(mut pending) = managed_swap.take() {
            let mut done = false;
            if !done && let Some(rx) = pending.loading.as_ref() {
                match rx.try_recv() {
                    Ok(Ok(None)) => {
                        state.managed_models.finish(&pending.request.operation, "unchanged", None);
                        done = true;
                    }
                    Ok(Ok(prepared @ Some(_))) => { pending.prepared = prepared; pending.loading = None; }
                    Ok(Err(error)) => {
                        state.managed_models.finish(&pending.request.operation, "failed", Some(error));
                        done = true;
                    }
                    Err(std::sync::mpsc::TryRecvError::Empty) => {}
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                        state.managed_models.finish(&pending.request.operation, "failed", Some("模型准备线程异常退出".into()));
                        done = true;
                    }
                }
            }
            if !done && !pending.committed && pending.loading.is_none() && pending.committing.is_none() && pending.prepared.is_some() {
                let prepared = pending.prepared.as_ref().expect("checked above");
                let transfer_check = match (prepared.controller.as_ref(), controller.as_ref()) {
                    (Some(new), Some(old)) => new.check_precision_transfer_from(old).map_err(|error| error.to_string()),
                    _ => Ok(()),
                };
                let current_matches = state.managed_models.models.load().iter().any(|model|
                    model.id == pending.request.model && model.sha256 == pending.request.expected_sha256);
                if !current_matches {
                    state.managed_models.finish(&pending.request.operation, "failed", Some("模型已变化，请刷新后重试".into()));
                    done = true;
                } else if shutdown_sit.is_some() || powered_off || state.exit_sit.load(Ordering::Acquire) != 0
                    || state.consecutive_errors.load(Ordering::Relaxed) >= state.max_consecutive_errors {
                    let _ = std::fs::remove_file(&prepared.staged);
                    state.managed_models.finish(&pending.request.operation, "failed", Some("机器人正在关闭或设备采样中断".into()));
                    done = true;
                } else if let Err(error) = transfer_check {
                    let _ = std::fs::remove_file(&prepared.staged);
                    state.managed_models.finish(&pending.request.operation, "failed", Some(error));
                    done = true;
                } else {
                    let staged = prepared.staged.clone();
                    let destination = prepared.destination.clone();
                    let candidate = pending.request.path.clone();
                    let expected_digest = pending.request.candidate_sha256.clone();
                    let (tx, rx) = std::sync::mpsc::sync_channel(1);
                    match std::thread::Builder::new().name("model-commit".into()).spawn(move || {
                        let result = managed_models::commit_checked(&staged, &destination, &candidate, &expected_digest);
                        let _ = tx.send(result);
                    }) {
                        Ok(_) => pending.committing = Some(rx),
                        Err(error) => {
                            state.managed_models.finish(&pending.request.operation, "failed", Some(error.to_string()));
                            done = true;
                        }
                    }
                }
            }
            if !done && let Some(rx) = pending.committing.as_ref() {
                match rx.try_recv() {
                    Ok(Ok(())) => {
                        pending.committing = None;
                        pending.committed = true;
                    }
                    Ok(Err(managed_models::CommitError::NotApplied(error))) => {
                        state.managed_models.finish(&pending.request.operation, "failed", Some(error));
                        done = true;
                    }
                    Ok(Err(managed_models::CommitError::Uncertain(error))) => {
                        state.managed_models.finish(&pending.request.operation, "uncertain", Some(error));
                        done = true;
                    }
                    Err(std::sync::mpsc::TryRecvError::Empty) => {}
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                        state.managed_models.finish(&pending.request.operation, "uncertain", Some("模型配置提交结果未知；请检查当前模型配置".into()));
                        done = true;
                    }
                }
            }
            if !done && pending.committed {
                if shutdown_sit.is_some() || powered_off || state.exit_sit.load(Ordering::Acquire) != 0
                    || state.consecutive_errors.load(Ordering::Relaxed) >= state.max_consecutive_errors
                {
                    if pending.home_started && matches!(bringup, Bringup::Homing { .. }) {
                        hold = last_targets;
                        bringup = Bringup::Ready;
                    }
                    state.managed_models.finish(&pending.request.operation, "uncertain",
                        Some("服务正在关闭或设备采样中断，模型已写入配置但尚未确认切换".into()));
                    done = true;
                }
            }
            if !done && pending.committed {
                let driving = if was_driving { controller.as_ref().and_then(Controller::driving) } else { None };
                pending.disturbs |= managed_disturbs(&pending.request.model, driving.as_ref());
                if pending.disturbs && bringup == Bringup::Ready && !pending.home_started {
                    if let Some(sensors) = sensors.as_ref() {
                        bringup = Bringup::Homing { from: sensors.positions, since: tick_start };
                        pending.home_started = true;
                    }
                } else if !matches!(bringup, Bringup::Homing { .. }) {
                    let mut prepared = pending.prepared.take().expect("committed model exists");
                    let result = transfer_precision(&state, &mut controller, &mut prepared.controller);
                    match result {
                        Err(error) => state.managed_models.finish(&pending.request.operation, "uncertain", Some(error.to_string())),
                        Ok(()) => {
                            if !pending.disturbs && let (Some(new), Some(old)) = (prepared.controller.as_mut(), controller.as_ref()) {
                                new.carry_over(old);
                            }
                            controller = prepared.controller;
                            policy_params = pending.candidate.clone();
                            policy_cfg = pending.cfg.clone();
                            state.policies.store(Arc::new(PolicyNames::of(&policy_cfg)));
                            state.policy_slots.store(Arc::new(slot_report(&policy_params, &policy_cfg, &slot_errors)));
                            publish_managed_models(&state, &controller, &policy_cfg);
                            state.managed_models.finish(&pending.request.operation, "applied", None);
                        }
                    }
                    done = true;
                }
            }
            if !done { managed_swap = Some(pending); }
        }

        // The sit-then-power-off sequence: `robot.shutdown`, or a genuinely empty pack.
        // The battery reading is a ~10 s EMA refreshed once a second, so a load sag cannot
        // reach the floor — a pack that gets there is spent.
        let battery_v = f64::from_bits(state.battery_v.load(Ordering::Relaxed));
        let battery_empty = params.bus.protocol != BusProtocol::Hx
            && params.safety.battery_empty_shutdown
            && battery_v > 0.0
            && battery_v <= duck_control::model::BATTERY_EMPTY_V;
        //
        // Latch a power-off requested during a rest until the rest has ended.
        let (shutdown, rest) = if !powered_off && shutdown_sit.is_none()
            && state.exit_sit.load(Ordering::Acquire) == 0 {
            (intents.take_shutdown() || battery_empty, intents.take_rest())
        } else {
            (false, false)
        };
        if coordinated_tracker.arrival_action_running()
            && (shutdown || rest || state.exit_sit.load(Ordering::Acquire) != 0)
        {
            cancel_arrival_skill(controller.as_mut(), &mut coordinated_unwritten_sit);
            coordinated_tracker.reset();
            coordinated_action_cancelled = true;
            tracing::warn!("coordinated arrival action cancelled for shutdown or rest");
        }
        let mut release_rest = None;
        if rest && shutdown {
            intents.finish_rest(false);
        } else if rest {
            if bringup == Bringup::Limp {
                release_rest = Some(true);
            } else if bringup == Bringup::Ready && !safety.fallen() && fresh.is_some()
                && safety.imu_ready() && limp_fall == LimpFall::Idle
                && let Some(controller) = controller.as_mut().filter(|controller| controller.has_sitstand())
            {
                let action = if params.bus.protocol == BusProtocol::Hx {
                    match rest_action::RestAction::load() {
                        Ok(action) => Ok(Some(action)),
                        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                            tracing::info!("rest: no Studio rest.csv, using the built-in rest pose");
                            Ok(None)
                        }
                        Err(error) => Err(error),
                    }
                } else {
                    Ok(None)
                };
                match action {
                    Ok(action) => {
                        let seated = controller.is_sitting()
                            && seat_transition_until.is_none_or(|until| tick_start >= until);
                        rest_sit_remaining = if seated {
                            Duration::ZERO
                        } else if action.is_some() {
                            Duration::from_secs_f64(controller.sit_ramp_secs())
                        } else {
                            Duration::from_secs_f64(controller.shutdown_sit_secs())
                        };
                        rest_action = action;
                        controller.cancel_scripted_actions();
                        controller.begin_shutdown_sit();
                        shutdown_sit = Some(tick_start);
                        shutdown_power_off = false;
                        tracing::info!("rest: entering seated pose");
                    }
                    Err(error) => {
                        tracing::error!(%error, "rest: cannot load Studio rest.csv");
                        intents.finish_rest(false);
                    }
                }
            } else {
                tracing::warn!("rest: seated pose is unavailable");
                intents.finish_rest(false);
            }
        }
        if shutdown {
            // A shutdown can arrive on IPC after this tick's session check.  No
            // individual status reply is outstanding here: finish the journal
            // before the shutdown can issue torque or position commands.
            if status_session_open {
                safety.set_individual_device_observation(false);
                if let Err(error) = status_bus::complete(&state.status_journal) {
                    tracing::error!(%error, "cannot close status session for shutdown");
                    safety.quarantine_device_bus();
                    state.status_bus_fault.store(true, Ordering::Release);
                    publish_status_fault(&state, safety.sample_device_observation(tick_start));
                    continue;
                }
                status_session_open = false;
            }
            shutdown_power_off = true;
            let can_sit = snapshot.enabled
                && bringup == Bringup::Ready
                && !safety.fallen()
                && controller.as_ref().is_some_and(|c| c.has_sitstand());
            if can_sit {
                tracing::warn!(battery_empty, "shutdown: sitting down before power off");
                // Goodbye peck; non-blocking — it plays out during the sit.
                if let Some(voice) = voice.as_mut() {
                    voice.play("peck", false);
                }
                controller
                    .as_mut()
                    .expect("can_sit checked the controller")
                    .begin_shutdown_sit();
                shutdown_sit = Some(tick_start);
            } else {
                tracing::warn!(battery_empty, "shutdown: cutting torque and powering off");
                cut_torque_before_poweroff(&mut safety);
                // Goodbye peck; blocking, so it is heard before the poweroff kills this
                // process's cgroup. The one deliberate block in the loop, on its last tick.
                if let Some(voice) = voice.as_mut() {
                    voice.play("peck", true);
                }
                intents.set_enabled(false);
                bringup = Bringup::Limp;
                powered_off = true;
                poweroff();
            }
        }
        let shutdown_sit_hold = controller.as_ref().map_or(SHUTDOWN_SIT, |c| {
            Duration::from_secs_f64(c.shutdown_sit_secs())
        });
        if state.exit_sit.load(Ordering::Acquire) == EXIT_SIT_REQUESTED {
            let can_sit = bringup == Bringup::Ready
                && !safety.fallen() && sensors.is_some() && safety.imu_ready()
                && limp_fall == LimpFall::Idle
                && !powered_off && shutdown_sit.is_none()
                && controller.as_ref().is_some_and(|c| c.has_sitstand());
            if !can_sit {
                tracing::warn!(enabled = snapshot.enabled, ?bringup, fallen = safety.fallen(),
                    sensors_available = sensors.is_some(), imu_ready = safety.imu_ready(),
                    limp_fall_active = limp_fall != LimpFall::Idle, powered_off,
                    shutdown_in_progress = shutdown_sit.is_some(),
                    sit_policy_loaded = controller.as_ref().is_some_and(|c| c.has_sitstand()),
                    "exit sit skipped: policy or robot state unavailable");
                state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
            } else if snapshot.enabled && controller.as_ref().is_some_and(|c| c.is_sitting()) {
                if seat_transition_until.is_some_and(|until| tick_start < until) {
                    tracing::info!("exit: waiting for the current sit to settle");
                    shutdown_sit = Some(tick_start);
                    exit_sit_started = Some(tick_start);
                    state.exit_sit.store(EXIT_SIT_RUNNING, Ordering::Release);
                } else {
                    tracing::info!("exit sit complete: already seated");
                    state.exit_sit_succeeded.store(true, Ordering::Release);
                    state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
                }
            } else if controller.as_ref().is_some_and(|c| !c.busy()) {
                tracing::info!("exit: sitting before torque release");
                controller.as_mut().expect("sit policy checked").begin_shutdown_sit();
                shutdown_sit = Some(tick_start);
                exit_sit_started = Some(tick_start);
                state.exit_sit.store(EXIT_SIT_RUNNING, Ordering::Release);
            }
        }
        if let Some(started) = exit_sit_started
            && state.exit_sit.load(Ordering::Acquire) == EXIT_SIT_RUNNING
        {
            if safety.fallen() || sensors.is_none() || !safety.imu_ready() {
                tracing::warn!("exit sit interrupted by unsafe state; releasing torque");
                state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
            } else if tick_start.duration_since(started) >= shutdown_sit_hold {
                tracing::info!("exit sit complete; stopping policy");
                state.exit_sit_succeeded.store(true, Ordering::Release);
                state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
            }
        }
        // Preserve the seated targets before ordinary policy-stop handling can command HOME.
        if state.exit_sit.load(Ordering::Acquire) == EXIT_SIT_DONE {
            break;
        }
        if let Some(started) = shutdown_sit
            && !powered_off && exit_sit_started.is_none()
        {
            if let Some(RestMotion::Action { playback, .. }) = shutdown_rest.as_mut() {
                playback.advance(tick_start);
            }
            let interruption = match shutdown_rest.as_ref() {
                _ if shutdown_power_off => None,
                _ if bringup != Bringup::Ready => Some("robot is no longer ready"),
                _ if sensors.is_none() => Some("lost control feedback"),
                Some(_) => None,
                None if !safety.imu_ready() => Some("IMU is no longer ready"),
                None if safety.fallen() => Some("standing fall threshold exceeded"),
                None => None,
            };
            if let Some(reason) = interruption {
                let phase = match shutdown_rest.as_ref() {
                    None => "sit",
                    Some(RestMotion::Action { .. }) => "action",
                    Some(RestMotion::Ramp(_)) => "ramp",
                };
                tracing::warn!(phase, reason, ?bringup, "rest: motion interrupted");
                release_rest = Some(false);
            } else {
                let sit_complete = if shutdown_power_off {
                    tick_start.duration_since(started) >= shutdown_sit_hold
                } else {
                    rest_sit_remaining.is_zero()
                };
                match shutdown_rest.as_ref() {
                    None if sit_complete => {
                        let gain = safety.gain().unwrap_or(policy_cfg.gain);
                        if !shutdown_power_off {
                            // Hand off at the last sit target while the servos still have torque.
                            intents.set_enabled(false);
                            snapshot.enabled = false;
                            policy_was_enabled = false;
                        }
                        shutdown_rest = Some(if let Some(action) = rest_action.take() {
                            tracing::info!("rest: seated pose complete, policy disabled, starting rest.csv");
                            RestMotion::Action { playback: action.start(last_targets), gain }
                        } else {
                            RestMotion::Ramp(RestRamp { from: last_targets, gain, since: tick_start })
                        });
                    }
                    Some(rest) if rest.done(tick_start) && !shutdown_power_off => {
                        tracing::info!("rest: movement complete, releasing all servos");
                        release_rest = Some(true);
                    }
                    Some(rest) if rest.done(tick_start) => {
                        tracing::warn!("at rest: cutting torque and powering off");
                        cut_torque_before_poweroff(&mut safety);
                        intents.set_enabled(false);
                        bringup = Bringup::Limp;
                        powered_off = true;
                        poweroff();
                    }
                    _ => {}
                }
            }
        }
        if let Some(completed) = release_rest {
            intents.set_enabled(false);
            let mut torque_released = false;
            for attempt in 1..=POWEROFF_TORQUE_ATTEMPTS {
                match safety.set_torque(false) {
                    Ok(()) => {
                        torque_released = true;
                        break;
                    }
                    Err(error) => tracing::warn!(%error, attempt, "rest: torque release failed"),
                }
            }
            intents.finish_rest(completed && torque_released);
            if !torque_released {
                maintenance_holding = true;
            }
            bringup = Bringup::Limp;
            state.homed.store(false, Ordering::Release);
            was_driving = false;
            shutdown_sit = None;
            shutdown_rest = None;
            shutdown_power_off = true;
            rest_action = None;
            if let Some(controller) = controller.as_mut() {
                controller.forget();
            }
        }

        // Limp-fall (`[safety] limp_fall`): catch the fall on the way down.
        //
        // The standing policy is a good stand-up-er and a bad faller. It stands a still
        // robot up cleanly from face-down or face-up; asked to do it out of a dynamic fall
        // it tries, fails, tries again — at walking gain, against the floor — and the
        // motors pay for every attempt. So take the fall away from it: go soft before the
        // landing, let the robot arrive limp, put it back into the standing pose, and only
        // then hand it over. What the policy then sees is the case it is good at.
        //
        // Everything here is off the policy's path by construction: `driving` is false for
        // the whole sequence, so the controller is not stepped and the targets below come
        // from this block. Nothing in `safety` needs telling: the fall verdict gates
        // nothing, so the pose ramp moves a robot lying on the floor like any other tick.
        if params.safety.limp_fall {
            // Abandoned mid-sequence: someone cut the torque, disabled the policy, or the
            // shutdown sit started. Whatever took over owns the robot now — drop out rather
            // than finishing a pose ramp nobody asked for, and hold where the robot is
            // rather than snapping back to a pose it collapsed out of a second ago.
            if limp_fall != LimpFall::Idle
                && !(snapshot.enabled
                    && bringup == Bringup::Ready
                    && shutdown_sit.is_none()
                    && !powered_off)
            {
                tracing::warn!("limp-fall abandoned — something else took the robot");
                limp_fall = LimpFall::Idle;
                falling.reset();
                hold = coast.known_positions(hold);
            }
            match limp_fall {
                LimpFall::Idle => {
                    // Only on a fresh sample: a coasted one is the same reading twice, and
                    // feeding it to a debounce would let one bad tick count three times.
                    let eligible = snapshot.enabled && !status_stop_requested
                        && bringup == Bringup::Ready
                        && was_driving
                        // Already down. There is no fall left to catch, and this is what
                        // stops the sequence firing at a robot on the floor working its way
                        // upright: a stand-up rocks, and a rock is a tilt with a rate on it.
                        // Limping through that would be an endless limp-pose-retry loop.
                        && !safety.fallen()
                        && shutdown_sit.is_none()
                        && !powered_off
                        // A roulade tips the robot over on purpose, a ground pick pitches it
                        // forward, and a sit is not a fall. Limping through any of them would
                        // break a move that was working.
                        && controller
                            .as_ref()
                            .is_some_and(|c| (!c.busy() || c.skill_recovering()) && !c.is_sitting());
                    match fresh.as_ref() {
                        Some(fresh) if eligible && safety.imu_ready() => {
                            if falling.observe(&fresh.imu, period) {
                                tracing::warn!(
                                    gravity_z = format!("{:.2}", fresh.imu.gravity[2]),
                                    rate =
                                        format!("{:.2}", FallPredictor::gravity_z_rate(&fresh.imu)),
                                    gain = params.safety.gain_limp,
                                    "falling — going limp to land soft"
                                );
                                limp_fall = LimpFall::Limp {
                                    since: tick_start,
                                    landing: Landing::default(),
                                };
                            }
                        }
                        // Not eligible, or blind for this tick. Drop any debounce in
                        // progress rather than resuming it later against a robot that has
                        // moved on.
                        _ => falling.reset(),
                    }
                }
                LimpFall::Limp { since, mut landing } => {
                    let rate = fresh
                        .as_ref()
                        .map(|s| s.imu.gyro.iter().map(|w| w * w).sum::<f64>().sqrt());
                    let landed = landing.observe(
                        rate,
                        period,
                        params.safety.limp_fall_still_rate,
                        limp_fall_still,
                    );
                    // Whatever the gyro says: a robot held in someone's hands, or resting
                    // against something that keeps nudging it, must not stay limp forever.
                    let timed_out = tick_start.duration_since(since) >= limp_fall_max;
                    if landed || timed_out {
                        // From where the robot actually is. `hold` is where it was when
                        // driving stopped, which is a pose it has since collapsed out of —
                        // ramping from that would start with a jump.
                        let from = coast.known_positions(hold);
                        tracing::warn!(
                            limp_ms = tick_start.duration_since(since).as_millis(),
                            timed_out,
                            pose_ms = limp_fall_pose.as_millis(),
                            "landed — posing back to standing"
                        );
                        limp_fall = LimpFall::Posing {
                            from,
                            since: tick_start,
                        };
                    } else {
                        limp_fall = LimpFall::Limp { since, landing };
                    }
                }
                LimpFall::Posing { since, .. } => {
                    if tick_start.duration_since(since) >= limp_fall_pose {
                        tracing::warn!("posed — handing back to the standing policy");
                        limp_fall = LimpFall::Idle;
                        falling.reset();
                        hold = home;
                        // Hand back to the policy, and let it choose. The twist has been
                        // held at zero through the whole sequence, so with nobody driving
                        // the standing network is what command magnitude selects — which
                        // is the stand-up. A client that *is* driving gets its walk back;
                        // the humans stay in charge, here as everywhere else.
                        if let Some(controller) = controller.as_mut() {
                            controller.reset();
                        }
                    }
                }
            }
        }
        let in_limp_fall = limp_fall != LimpFall::Idle;

        // Smooth the command. The limp-fall sequence holds the twist at zero outright, so
        // the robot is not handed back mid-command; and leaving body-pose mode snaps the
        // body back to nominal rather than gliding, which is its B-button exit.
        let stop_changed = snapshot.stop_generation != previous_stop_generation;
        let other_unsafe_motion = deadman.is_some() || safety.fallen() || in_limp_fall
                || !snapshot.enabled || bringup != Bringup::Ready || sensors.is_none()
                || snapshot.pose.active || shutdown_sit.is_some()
                || mode_change.is_some() || pending_swap.is_some()
                || controller.as_ref().is_none_or(|controller| controller.busy());
        let unsafe_motion = stop_changed || other_unsafe_motion;
        let gamepad_ready = deadman.is_none() && !safety.fallen() && !in_limp_fall
            && snapshot.enabled && bringup == Bringup::Ready && sensors.is_some()
            && shutdown_sit.is_none() && mode_change.is_none()
            && pending_swap.is_none() && safety.imu_ready() && !powered_off;
        if state.precision_ready.load(Ordering::Acquire) && previous_gamepad_ready && !gamepad_ready {
            let mut guard = state.human_control.lock();
            if guard.revoke_gamepad() {
                clear_human_intents(&intents);
                tracing::warn!("gamepad control revoked after motion became unavailable");
            }
        }
        previous_gamepad_ready = gamepad_ready;
        let stop_requires_cancel = stop_changed
            && snapshot.gamepad_rearmed_stop_generation != Some(snapshot.stop_generation);
        if snapshot.gamepad_twist && (other_unsafe_motion || stop_requires_cancel) {
            intents.cancel_gamepad_twist(snapshot.gamepad_rearmed_stop_generation);
        }
        if unsafe_motion || !safety.imu_ready() || powered_off {
            if let Some(controller) = controller.as_mut() {
                controller.cancel_precision();
            }
        }
        if let Some(request) = intents.take_precision() {
            match request.params {
                proto::PrecisionMoveParams::Cancel => {
                    if let Some(controller) = controller.as_mut() { controller.cancel_precision(); }
                    request.finish();
                }
                proto::PrecisionMoveParams::Start { axis, value } => {
                    if request.interrupt_precision {
                        if let Some(controller) = controller.as_mut() { controller.cancel_precision(); }
                        if let Some(previous) = current_precision.take() { previous.finish(); }
                        intents.set_precision_running(false);
                    }
                    if !unsafe_motion && safety.imu_ready() && !powered_off
                        && request.stop_generation == snapshot.stop_generation
                        && intents.snapshot().stop_generation == snapshot.stop_generation
                    {
                        if let Some(controller) = controller.as_mut() {
                            match controller.start_precision(axis, value, tick_start) {
                                Ok(()) => {
                                    current_precision = Some(Arc::clone(&request));
                                    intents.set_precision_running(true);
                                    tracing::info!(?axis, value, "precision movement started");
                                }
                                Err(reason) => {
                                    request.finish();
                                    tracing::warn!(reason, "precision movement refused by controller");
                                }
                            }
                        } else {
                            request.finish();
                        }
                    } else {
                        request.finish();
                        tracing::warn!("precision movement cancelled before execution");
                    }
                }
                proto::PrecisionMoveParams::Status => request.finish(),
            }
        }
        let precision_active = controller.as_ref().is_some_and(Controller::precision_active);
        let cancel_velocity_motion = unsafe_motion
            && (velocity_command_safety || snapshot.gamepad_twist || previous_gamepad_twist
                || snapshot.motion_tracking.is_some() || snapshot.coordinated_tracking.is_some() || snapshot.maze.is_some());
        previous_stop_generation = snapshot.stop_generation;
        previous_gamepad_twist = snapshot.gamepad_twist;
        let skill_recovering = controller.as_ref().is_some_and(Controller::skill_recovering);
        let twist_target = if in_limp_fall || cancel_velocity_motion || precision_active || skill_recovering {
            [0.0; 3]
        } else {
            gated.twist
        };
        if in_limp_fall || cancel_velocity_motion || (tracking_stopped && !status_quiescing)
            || (maze_immediate_stop && !status_quiescing) || precision_active || skill_recovering {
            twist_ema = [0.0; 3];
        }
        smooth_twist(
            &mut twist_ema,
            twist_target,
            cmd_alpha,
            policy_overrides.immediate_stop && !status_quiescing,
        );
        apply_tracking_deadzone(&mut twist_ema, tracking_deadzone, person_follow, target_follow);
        for (ema, target) in head_ema.iter_mut().zip(gated.head) {
            slew(ema, target, head_alpha);
        }
        if snapshot.pose.active {
            for (ema, target) in body_ema.iter_mut().zip(snapshot.pose.body) {
                slew(ema, target, cmd_alpha);
            }
        } else {
            body_ema = [0.0; 3];
        }
        // Match upstream idle eligibility: policy running, zero motion, no recent head
        // intent, no scripted skill or singing. Only the yawn is added here.
        let still = was_driving
            && !status_page_active
            && snapshot.enabled
            && bringup == Bringup::Ready
            && sensors.is_some()
            && !in_limp_fall
            && shutdown_sit.is_none()
            && twist_target == [0.0; 3]
            && twist_ema.iter().all(|v| v.abs() < TWIST_IDLE_EPSILON)
            && snapshot.head_age >= idle_yawn.after()
            && chorale_head.iter().all(|v| v.abs() < 1e-3)
            && controller.as_ref().is_some_and(|c| !c.busy());
        let idle = idle_yawn.tick(tick_start, still, period);
        // Bring the robot up when someone asks it to drive and it has no torque yet.
        //
        // Not gated on the fall: a fallen robot ramps like any other, as the prototype
        // does. Being down is a report, and the humans stay in charge.
        //
        // Needs a sample: `from` is where the joints actually are, and starting a ramp from a
        // position nobody read would be the lurch this exists to avoid.
        // `controller.is_some()` as well, because this call means "enable the policy": powering the
        // joints to run a policy that is disabled or would not load is work towards nothing, and it
        // would make a release whose bundle is broken stand the robot up and then hold. A robot that
        // should stand without a policy is what `robotd init` is for.
        //
        // Read the flag again here instead of using the tick-start snapshot. `robot.relax` can
        // clear it after that snapshot was taken; using the old value would cut torque and then
        // immediately enable it again in this same tick.
        let enable_guard = state.human_control.lock();
        let enable_requested = intents.enabled() && !enable_guard.emergency_release_active();
        if !powered_off
            && let (Bringup::Limp, true, true, Some(sensors)) = (
                bringup,
                enable_requested,
                controller.is_some(),
                sensors.as_ref(),
            )
        {
            match (|| {
                if state.init_torque_only {
                    let current = fresh.as_ref().ok_or_else(|| duck_control::io::IoError::Bus(
                        "current-pose initialization requires fresh joint readings".into()))?;
                    safety.prepare_torque_only(&current.positions)?;
                }
                safety.set_torque(true)
            })() {
                Ok(()) => {
                    if state.init_torque_only {
                        hold = sensors.positions;
                        bringup = Bringup::Ready;
                        if let Some(controller) = controller.as_mut() { controller.forget(); }
                        tracing::warn!("enabling the policy: torque on at current pose");
                    } else {
                        tracing::warn!(?HOME_RAMP, "enabling the policy: torque on, ramping to home");
                        bringup = Bringup::Homing {
                            from: sensors.positions,
                            since: tick_start,
                        };
                    }
                }
                // Reported, not fatal, and it stays `Limp` so the next tick tries again: a bus that
                // dropped one transaction is ordinary, and a robot that refused to ever come up
                // because of it would be worse than one that keeps asking.
                Err(e) => tracing::warn!(error = %e, "cannot enable torque; the robot stays limp"),
            }
        }
        drop(enable_guard);

        // The ramp finishing is what makes the policy eligible to drive.
        //
        // Unless the robot went home for a swap whose networks are still loading. Then it holds
        // here, at the pose the ramp reached, until they land: the ramp exists so the swap happens
        // at rest, and going `Ready` with the old controller for the second the load still needs
        // would hand it the robot for exactly that second.
        if let Bringup::Homing { .. } = bringup
            && bringup.homing_target(tick_start, &home).is_none()
            && pending_swap
                .as_ref()
                .is_some_and(|p| p.disturbs && !p.ready())
        {
            hold = home;
        } else if let Bringup::Homing { .. } = bringup
            && bringup.homing_target(tick_start, &home).is_none()
        {
            // Home, and a switch waiting: load the other mode's bundle here, where the robot is
            // standing still at a known pose with torque on. `Policy::load` validates and warms
            // up each network, so this blocks the loop for a moment — deliberately, and in the one
            // place where a stalled command stream costs nothing, because the robot is holding a
            // pose rather than mid-stride. Missed ticks in that window are expected.
            if let Some(target) = mode_change.take() {
                load_mode(
                    target,
                    &mut policy_params,
                    &mut policy_cfg,
                    &mut controller,
                    params.safety.limp_fall,
                    &state,
                    &slot_errors,
                    params.bus.protocol,
                    Some(policy_home),
                );
            }

            tracing::warn!("at the home pose; the policy has the robot");
            bringup = Bringup::Ready;
            hold = home;
            if let Some(controller) = controller.as_mut() {
                controller.forget();
            }
            resume_seated_policy = false;
        }
        // A loaded policy change, applied where it is safe to: at the home pose the ramp above
        // just reached when the change replaces the network driving, or wherever the robot is
        // when it does not. Never mid-ramp, which is the one state where the joints are moving
        // somewhere they have not arrived.
        //
        // What differs from a mode switch is the failure. A mode switch that will not load
        // leaves the robot unhealthy, because the release it shipped in is broken. A load is
        // somebody *trying* a file, and losing the gait you had over it would be a worse answer
        // than the file being refused — so nothing is swapped unless the new controller built.
        if !matches!(bringup, Bringup::Homing { .. })
            && let Some(swap) = pending_swap.take_if(|p| p.ready())
        {
            let PendingSwap {
                change,
                candidate,
                cfg,
                disturbs,
                built,
                ..
            } = swap;
            let built = built.expect("take_if checked the load has finished").and_then(|mut built| {
                transfer_precision(&state, &mut controller, &mut built).map_err(|error| error.to_string())?;
                Ok(built)
            });
            match built {
                Ok(mut built) => {
                    // The network driving is unchanged, so the robot need not notice: the
                    // new controller takes over the old one's seat, skill and filter state
                    // and the next tick is the same network from the same place.
                    if !disturbs
                        && let (Some(new), Some(old)) = (built.as_mut(), controller.as_ref())
                    {
                        new.carry_over(old);
                    }
                    controller = built;
                    policy_params = candidate;
                    publish_managed_models(&state, &controller, &cfg);
                    // Whatever this slot could not do before, it is not doing it now: either
                    // it loaded, or it went back to a default that is known to.
                    match &change {
                        intents::PolicyChange::Slot { slot, .. } => slot_errors.clear(*slot),
                        // A reload re-reads every file, so every slot's verdict is fresh again.
                        intents::PolicyChange::ResetAll | intents::PolicyChange::Reload => {
                            slot_errors = SlotErrors::default()
                        }
                    }
                    state.policy_error.store(None);
                    state.policy_change_error.store(None);
                    state.policies.store(Arc::new(PolicyNames::of(&cfg)));
                    state.policy_slots.store(Arc::new(slot_report(
                        &policy_params,
                        &cfg,
                        &slot_errors,
                    )));
                    policy_cfg = cfg;
                    tracing::warn!(
                        change = describe_change(&change),
                        loaded = controller.is_some(),
                        in_place = !disturbs,
                        "policy change complete"
                    );
                }
                Err(e) => {
                    // `candidate` is dropped, so config, the resolved paths and the running
                    // networks are all still the ones that were working a tick ago.
                    tracing::error!(
                        change = describe_change(&change),
                        error = %e,
                        "policy change failed; keeping the policy that was running"
                    );
                    // Recorded against the slot so the failure outlives the log line: it is
                    // what `robot.policies` shows the caller polling for an outcome, and what
                    // makes health degraded until the slot is loaded or reset. It does not
                    // make the robot unhealthy — nothing about the release is wrong.
                    match change {
                        intents::PolicyChange::Slot { slot, .. } => {
                            slot_errors.set(slot, e);
                            state.policy_slots.store(Arc::new(slot_report(
                                &policy_params,
                                &policy_cfg,
                                &slot_errors,
                            )));
                        }
                        // A reload or a whole-robot reset names no slot to hang this on, and
                        // without somewhere to put it the failure was a log line on the robot
                        // and silence on the wire — while `robot.setSkill` answered "accepted"
                        // and triggered exactly this reload. Degraded, not unhealthy: the robot
                        // is still running what it had, so there is nothing for a rollback to
                        // repair.
                        intents::PolicyChange::Reload | intents::PolicyChange::ResetAll => {
                            state.policy_change_error.store(Some(Arc::new(e)));
                        }
                    }
                }
            }
        }

        state
            .homed
            .store(bringup == Bringup::Ready, Ordering::Relaxed);
        // The policy must not start on an unconverged orientation filter — the first
        // seconds of projected gravity are whatever the filter is mid-way through deciding,
        // and a gait stepping against that horizon is the prototype's "crazy start".
        let imu_warm = safety.imu_ready();
        if snapshot.enabled && !imu_warm && !warned_imu_warming {
            tracing::warn!(
                "IMU converging (keep the robot still) — the policy starts when it is ready"
            );
            warned_imu_warming = true;
        }

        // Drive only with a sample to drive from: a tick whose read failed has no
        // observation to build, and inventing one would feed the policy a stale robot.
        //
        // And only once the ramp is done, or the policy's first step would come from wherever the
        // robot was slumped. A fall does not stop the driving, as the prototype does not
        // stop it: the policy keeps going and the humans stay in charge.
        let exit_policy_driving = exit_sit_started.is_some()
            && state.exit_sit.load(Ordering::Acquire) == EXIT_SIT_RUNNING;
        let rest_sitting = shutdown_sit.is_some() && !shutdown_power_off && shutdown_rest.is_none();
        let policy_enabled_for_tick = snapshot.enabled || exit_policy_driving || rest_sitting;
        if shutdown_sit.is_some() || shutdown_rest.is_some() || powered_off
            || state.exit_sit.load(Ordering::Acquire) != 0
        {
            resume_seated_policy = false;
        }
        let requested_driving = policy_enabled_for_tick
            && state.exit_sit.load(Ordering::Acquire) != EXIT_SIT_DONE
            && bringup == Bringup::Ready
            && controller.is_some()
            // The limp-fall sequence owns the robot for its duration: the whole point is
            // that the policy is *not* driving while the robot falls, lands and is posed.
            && !in_limp_fall
            // The rest ramp has the robot for the end of a shutdown.
            && shutdown_rest.is_none()
            && sensors.is_some()
            && imu_warm
            && !powered_off;

        let mut policy_gravity_input = None;
        if policy_overrides.policy_gravity_from_reference {
            if requested_driving {
                policy_gravity_input = Some(gravity_selector.select(
                    sensors.as_ref().expect("requested driving has sensors"),
                    proto::clock::monotonic_ns(), fresh.is_some(),
                ));
            } else if !policy_enabled_for_tick || bringup != Bringup::Ready || in_limp_fall || powered_off {
                gravity_selector.pause();
                gravity_holding = false;
            }
        }
        let mut driving = requested_driving
            && policy_gravity_input.as_ref().is_none_or(|input| input.error.is_none());
        if !driving && let Some(controller) = controller.as_mut() {
            controller.cancel_precision();
        }
        if policy_gravity_input.as_ref().is_some_and(|input| input.error.is_some()) && !gravity_holding {
            hold = coast.known_positions(hold);
            stopped_driving_at = Some(tick_start);
            gravity_holding = true;
        }

        if driving && !was_driving {
            // Starting fresh: a stale previous action in the observation, or a filter
            // anchored to where the robot was a minute ago, would both show up as a lurch.
            //
            // Only after a real pause, though. Past `COAST_TICKS` of a bad bus this edge
            // fires again within a few ticks, and resetting then re-introduces exactly the
            // discontinuity the reset is for: a zeroed action the policy observes, and a
            // low-pass filter with no anchor.
            if reset_on_resume(stopped_driving_at, tick_start)
                && let Some(controller) = controller.as_mut()
            {
                controller.reset();
            }
        }
        if was_driving && !driving {
            stopped_driving_at = Some(tick_start);
            if policy_enabled_for_tick {
                // Any other stop — IMU cooling, a blind bus, the armed fall gate — freezes
                // where the robot *is*, from the last sample that arrived. Captured once,
                // not re-read each tick, or the hold target would sag under gravity.
                //
                // Deliberately not gated on a sample arriving this tick: the stop that
                // matters most is the one caused by a read failing, and requiring a reading
                // then is requiring the thing that just failed.
                hold = coast.known_positions(hold);
            }
        }
        if explicit_stop && !torque_only_initialized && !policy_enabled_for_tick && bringup == Bringup::Ready
            && shutdown_sit.is_none() && !powered_off && !maintenance_holding
        {
            resume_seated_policy = false;
            stopped_driving_at = Some(tick_start);
            let seated = controller.as_mut().is_some_and(|controller| {
                controller.stop_moves();
                controller.is_sitting()
            });
            hold = if seated || status_stop_requested { coast.known_positions(hold) } else { home };
            seat_transition_until = None;
        }
        if driving && resume_seated_policy {
            let controller = controller.as_mut().expect("driving has a controller");
            let pose = sensors.as_ref().expect("driving has sensors").positions;
            controller.settle_then_rise(&pose);
            resume_seated_policy = false;
        }
        // Report the posture after applying enable and stop transitions.
        state.sitting.store(
            controller.as_ref().is_some_and(Controller::is_sitting),
            Ordering::Relaxed,
        );
        was_driving = driving;
        let mut pickup_seated = false;
        if let (Some(tracking), Some(sensors)) = (snapshot.coordinated_tracking.as_ref(), fresh.as_ref())
            && tracking.fresh && tracking.params.camera.is_some()
            && sensors.positions[duck_control::model::MOUTH_INDEX].is_finite()
        {
            let mouth = sensors.positions[duck_control::model::MOUTH_INDEX];
            if coordinated_tracker.arrival_action_preparing() {
                coordinated_mouth.prepare(tracking.params.mouth_before, mouth);
            } else {
                coordinated_mouth.cancel_preparation();
            }
            if coordinated_mouth.sitting_action_running()
                && coordinated_tracker.arrival_action_running() && !coordinated_unwritten_sit
                && controller.as_ref().is_some_and(Controller::is_sitting)
                && seat_transition_until.is_some_and(|until| tick_start >= until)
            {
                coordinated_mouth.action_finished(tracking.params.mouth_after, mouth);
                if tracking.params.pickup_after.is_some() {
                    pickup_action_completed = Some((tracking.sequence, snapshot.stop_generation));
                }
            }
            pickup_seated = controller.as_ref().is_some_and(Controller::is_sitting);
            coordinated_mouth.update_after(tracking.params.mouth_after);
        } else if snapshot.coordinated_tracking.as_ref().is_none_or(|tracking|
            !tracking.fresh || tracking.params.camera.is_none())
        {
            coordinated_mouth.reset();
        }
        if coordinated_tracker.arrival_action_ready()
            && (coordinated_mouth.before_ready() || requests.any())
        {
            if requests.any() {
                coordinated_tracker.resolve_arrival_action(false);
                coordinated_mouth.reset();
                tracing::info!("coordinated arrival action cancelled by another action request");
            } else if let Some(tracking) = snapshot.coordinated_tracking.as_ref()
                && driving && fresh.is_some() && !unsafe_motion && safety.imu_ready()
                && shutdown_sit.is_none() && shutdown_rest.is_none() && !powered_off
                && state.exit_sit.load(Ordering::Acquire) == 0
                && controller.as_ref().is_some_and(|controller| {
                    !controller.busy() && !controller.is_sitting()
                        && controller.driving() == Some(Driving::Stand)
                })
            {
                let current = intents.snapshot();
                let same_arrival = current.enabled && current.stop_generation == snapshot.stop_generation
                    && current.coordinated_tracking.as_ref().is_some_and(|current| {
                        current.sequence == tracking.sequence
                            && current.fresh && current.params.camera.is_some()
                            && current.params.arrival_skill == tracking.params.arrival_skill
                            && current.params.mouth_before == tracking.params.mouth_before
                    });
                if same_arrival {
                    let name = &tracking.params.arrival_skill;
                    if name.is_empty() {
                        coordinated_tracker.resolve_arrival_action(false);
                        coordinated_mouth.reset();
                    } else {
                        let result = start_arrival_skill(
                            controller.as_mut().expect("standing controller exists"), name,
                            tick_start, &mut seat_transition_until,
                        );
                        coordinated_unwritten_sit = result.is_ok() && name == "sit_toggle";
                        coordinated_tracker.resolve_arrival_action(result.is_ok());
                        if result.is_ok() { coordinated_mouth.action_started(name == "sit_toggle"); }
                        else { coordinated_mouth.reset(); }
                        match result {
                            Ok(()) => tracing::warn!(skill = %name, "coordinated arrival action started"),
                            Err(reason) => tracing::warn!(skill = %name, reason, "coordinated arrival action refused"),
                        }
                    }
                }
            }
        }
        let tracking_walk = snapshot.coordinated_tracking.is_some() && !tracking_stopped;
        let tracking_present = snapshot.coordinated_tracking.is_some();
        let tracking_motion = tracking_walk && !coordinated_tracker.arrived()
            && snapshot.coordinated_tracking.as_ref().is_some_and(|tracking| tracking.params.target.is_some());
        let mut policy_twist = if tracking_motion && person_follow {
            coordinated_smoothed_twist(twist_ema, twist_target)
        } else {
            twist_ema
        };
        if tracking_walk && let Some(tracking) = snapshot.coordinated_tracking.as_ref() {
            policy_twist = coordinated_limit_twist(policy_twist, coordinated_tracker.linear_limits(&tracking.params));
        }
        let target_lateral_release = target_follow && tracking_walk
            && target_follow_stability.lateral_release() && twist_target == [0.0; 3];
        let target_keep_walking = target_walk_grace.update(
            target_follow && tracking_walk && !coordinated_tracker.arrived() && !target_lateral_release,
            policy_twist.iter().any(|velocity| velocity.abs() >= 0.005),
            period,
        );
        let manual_handoff_eligible = driving && !unsafe_motion && !explicit_stop && !precision_active
            && !tracking_stopped && !maze_immediate_stop && shutdown_rest.is_none() && !powered_off
            && state.exit_sit.load(Ordering::Acquire) == 0 && !state.stopping.load(Ordering::Acquire)
            && snapshot.head_tracking.is_none() && snapshot.motion_tracking.is_none()
            && snapshot.coordinated_tracking.is_none() && snapshot.maze.is_none()
            && !snapshot.pose.active
            && controller.as_ref().is_some_and(|controller|
                !controller.is_sitting() && !controller.scripted_active());
        let manual_hold_walk = walk_stand_handoff.update(
            Instant::now(), manual_handoff_eligible,
            controller.as_ref().is_some_and(|controller| controller.driving() == Some(Driving::Walk)),
            twist_target, policy_twist,
        );
        let status_hold_walk = status_quiescing && status_maintenance_stop.as_mut()
            .is_some_and(|stop| stop.force_walk(tick_start, policy_twist));
        if let Some(controller) = controller.as_mut() {
            controller.set_force_walk(status_hold_walk || manual_hold_walk || coordinated_tracking_force_walk(
                tracking_present, tracking_walk, &coordinated_tracker,
                target_keep_walking, target_lateral_release, policy_twist));
        }
        let manual_command = PolicyCommand { twist: policy_twist, ..PolicyCommand::default() };
        let avoidance_eligible = !unsafe_motion && !person_follow && snapshot.maze.is_none()
            && driving
            && controller.as_ref().is_some_and(|controller| controller.walks_for(&manual_command));
        let mut assisted_twist = obstacle_assistance.apply(
            &state.obstacle,
            proto::clock::monotonic_ns(),
            avoidance_eligible,
            twist_target,
            policy_twist,
            max_angular,
        );
        if person_follow { assisted_twist[1] = 0.0; }
        let mut command = PolicyCommand {
            twist: assisted_twist,
            // The chorale's sway rides on top of the head command from the preceding tick.
            head: [
                head_ema[0] + chorale_head[0],
                head_ema[1] + chorale_head[1] + idle.pitch,
                head_ema[2] + chorale_head[2],
                head_ema[3] + chorale_head[3],
            ],
            body: BodyPose {
                z: body_ema[0],
                roll: body_ema[1],
                pitch: body_ema[2],
            },
        };
        // Coordinated tracking owns these two physical head targets. Inform the policy of
        // the same pose before inference, in its HOME-relative command coordinates. The
        // tracked targets still go to the servos below: the policy alone may not reach the
        // precise position required by the camera and initial-position handoff.
        let manual_head = if params.bus.protocol == BusProtocol::Hx && driving
            && !snapshot.pose.active && !safety.fallen() && !in_limp_fall
            && shutdown_sit.is_none() && shutdown_rest.is_none()
            && controller.as_ref().is_some_and(|controller| !controller.busy() && !controller.is_sitting())
            && (snapshot.head_position_held || snapshot.head_age <= Duration::from_millis(250))
            && snapshot.head_tracking.is_none() && snapshot.coordinated_tracking.is_none()
            && snapshot.motion_tracking.is_none() && snapshot.maze.is_none()
            && snapshot.head_position_handoff.is_none()
            && let Some([pitch, yaw]) = snapshot.head_position_raw
        {
            duck_control::hx::decode_position(6, &pitch.to_le_bytes()).ok()
                .zip(duck_control::hx::decode_position(7, &yaw.to_le_bytes()).ok())
                .map(|(pitch, yaw)| {
                    let (pitch_min, pitch_max) = duck_control::model::JOINT_LIMITS[6];
                    let (yaw_min, yaw_max) = duck_control::model::JOINT_LIMITS[7];
                    [pitch.clamp(pitch_min, pitch_max), yaw.clamp(yaw_min, yaw_max)]
                })
        } else {
            None
        };
        if let Some([pitch, yaw]) = coordinated_head.or(manual_head) {
            command.head[1] = pitch - policy_home[6];
            command.head[2] = yaw - policy_home[7];
        }
        if let Some(yaw) = maze_head {
            // Ask for a physical head pose through the policy command. The actor's
            // servo target may differ to compensate for actuator tracking error.
            command.head[2] = yaw - policy_home[7];
        }
        // Voltage adaptation: the servos' effective kP tracks their supply, so scaling the
        // action by (nominal / measured) holds the robot's response steady as the pack
        // sags. The EMA is clamped to a plausible band so a bad reading cannot become a
        // wild scale.
        let scale_mult = if policy_cfg.voltage_adapt && battery_v > 0.0 {
            policy_cfg.nominal_voltage / battery_v.clamp(6.0, 9.5)
        } else {
            1.0
        };
        read_timing.mark_stage(2);

        let mut policy_trace = (recording || policy_overrides.policy_gravity_from_reference)
            .then(control::PolicyTrace::default);
        let mut policy_imu_input = None;
        if !driving || freeze_policy_imu {
            imu_input_monitor.pause();
        }
        let mut inference_failed = false;
        let (mut targets, gain, moving, policy_label) = match (driving, sensors.as_ref()) {
            // The limp-fall sequence, before anything else — `driving` is false throughout,
            // so without this it would fall through to the hold branch and the robot would
            // be commanded its pre-fall pose at walking gain, which is precisely the thing
            // the mode exists to stop.
            //
            // `moving` stays true for the whole sequence: the joints are travelling (down,
            // then back to the pose), and `safeToRestart` must not say yes in the middle of
            // a fall.
            _ if in_limp_fall => match limp_fall {
                // Command the joints where they already are, at limp gain. Following the
                // measurement rather than holding a fixed pose is what makes it soft: a
                // fixed target grows an error as the robot collapses, and an error at any
                // gain is a motor pushing back against the floor.
                LimpFall::Limp { .. } => (
                    coast.known_positions(hold),
                    params.safety.gain_limp,
                    true,
                    "limp_fall".into(),
                ),
                LimpFall::Posing { .. } => (
                    // Past the end of the ramp the target is the pose itself — the state
                    // machine above clears `Posing` on the same tick, so this is the one
                    // frame where the two can disagree.
                    limp_fall
                        .pose_target(tick_start, limp_fall_pose, &home)
                        .unwrap_or(home),
                    params.safety.limp_fall_pose_gain,
                    true,
                    "limp_pose".into(),
                ),
                LimpFall::Idle => unreachable!("in_limp_fall excludes Idle"),
            },
            // The end of a shutdown, from the seat to the rest pose. `driving` is false while it
            // runs, so the policy is not stepped under it.
            _ if !powered_off && shutdown_rest.is_some() => {
                let rest = shutdown_rest.as_ref().expect("just checked it is Some");
                let (targets, gain) = rest.targets(tick_start);
                (targets, gain, true, "rest".into())
            }
            (true, Some(sensors)) => 'policy: {
                let controller = controller.as_mut().expect("driving implies a controller");
                let mut policy_sensors = *sensors;
                if !imu_filter {
                    policy_sensors.imu = sensors.imu.without_host_filter();
                } else if !imu_gyro_filter {
                    policy_sensors.imu.gyro = sensors.imu.gyro_unfiltered;
                }
                if freeze_policy_imu {
                    policy_sensors.imu.gyro = [0.0; 3];
                    policy_sensors.imu.gravity = [0.0, 0.0, -1.0];
                }
                if !freeze_policy_imu {
                    policy_imu_input = imu_input_monitor.consume(
                        policy_sensors.imu_timing,
                        proto::clock::monotonic_ns(),
                        fresh.is_some(),
                    );
                }
                // The drive gate may precede a scheduler delay. Check again at consumption,
                // retaining the same sequence evidence instead of counting this sample twice.
                let observation_gravity = if let Some(input) = policy_gravity_input.as_mut() {
                    match input.at_consumption(proto::clock::monotonic_ns()) {
                        Ok(gravity) => Some(gravity),
                        Err(_) => {
                            gravity_selector.report(input);
                            driving = false;
                            was_driving = false;
                            controller.cancel_precision();
                            if !gravity_holding {
                                stopped_driving_at = Some(tick_start);
                                hold = coast.known_positions(hold);
                                gravity_holding = true;
                            }
                            imu_input_monitor.pause();
                            policy_trace = None;
                            policy_imu_input = None;
                            break 'policy (hold, policy_cfg.gain, false, "held".into());
                        }
                    }
                } else { None };
                controller.set_force_walk(status_hold_walk || manual_hold_walk || coordinated_tracking_force_walk(
                    tracking_present, tracking_walk, &coordinated_tracker,
                    target_keep_walking, target_lateral_release, command.twist));
                let result = if command.twist != policy_twist {
                    controller.step_with_selection(
                        &policy_sensors, &command, policy_twist, snapshot.pose.active, dt, scale_mult,
                        observation_gravity, policy_trace.as_mut(),
                    )
                } else if observation_gravity.is_some() {
                    controller.step_with_observation_gravity(
                        &policy_sensors, &command, snapshot.pose.active, dt, scale_mult,
                        observation_gravity, policy_trace.as_mut(),
                    )
                } else {
                    controller.step_with_trace(
                        &policy_sensors, &command, snapshot.pose.active, dt, scale_mult,
                        policy_trace.as_mut(),
                    )
                };
                if let Some(input) = policy_gravity_input.as_ref() { gravity_selector.report(input); }
                match result {
                    Ok(step) => (
                        step.targets,
                        step.gain,
                        // A scripted move is motion whatever the twist says; so is walking.
                        step.busy || command.twist_magnitude() > 0.0,
                        step.label,
                    ),
                    Err(e) => {
                        tracing::warn!(error = %e, "inference failed; holding");
                        controller.cancel_precision();
                        inference_failed = true;
                        (hold, policy_cfg.gain, false, "held".into())
                    }
                }
            }
            // Ramping to the home pose. `moving` is true, because it is: the joints are travelling,
            // and `safeToRestart` must not say yes in the middle of it.
            _ if bringup.homing_target(tick_start, &home).is_some() => (
                bringup
                    .homing_target(tick_start, &home)
                    .expect("just checked it is Some"),
                policy_cfg.gain,
                true,
                "homing".into(),
            ),
            _ => (hold, policy_cfg.gain, false, "held".into()),
        };
        if inference_failed {
            walk_stand_handoff = WalkStandHandoff::default();
        }
        let active_name = match policy_label.as_ref() {
            "sit" | "rise" | "seated" => Some("sitstand"),
            label if state.managed_models.models.load().iter().any(|model| model.id == label) => Some(label),
            _ => None,
        };
        if state.managed_models.active.load().as_deref().map(String::as_str) != active_name {
            state.managed_models.active.store(active_name.map(|name| Arc::new(name.to_owned())));
        }
        if controller.as_ref().is_none_or(|controller| !controller.precision_active())
            && let Some(request) = current_precision.take()
        {
            request.finish();
            intents.set_precision_running(false);
        }
        if policy_overrides.policy_gravity_from_reference {
            if !policy_trace.as_ref().is_some_and(|trace| trace.observation_built) {
                policy_trace = None;
                if let Some(input) = policy_gravity_input.as_mut() { input.used_gravity = None; }
            }
            if driving { gravity_holding = false; }
        }
        // A reference rejected at the last consumption check did not drive this read.
        if driving && let Some(run) = read_run.as_mut() {
            run.driving = true;
        }
        if exit_sit_started.is_some() && inference_failed {
            tracing::warn!("exit sit inference failed; releasing torque");
            state.exit_sit_succeeded.store(false, Ordering::Release);
            state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
            break;
        }
        read_timing.mark_stage(3);
        let policy_output_t_ns =
            (recording && driving && !inference_failed && sensors.is_some()
                && (!policy_overrides.policy_gravity_from_reference || policy_trace.is_some()))
                .then(proto::clock::monotonic_ns);
        if let Some(controller) = controller.as_mut() {
            let recovery_imu = fresh.as_ref().filter(|_| {
                driving && safety.imu_ready() && !inference_failed && !safety.fallen() && !in_limp_fall
            }).map(|sensors| &sensors.imu);
            controller.observe_skill_recovery(recovery_imu, period);
        }
        state.moving.store(moving, Ordering::Relaxed);
        let other_action_busy = controller.as_ref().is_some_and(Controller::busy)
            || matches!(bringup, Bringup::Homing { .. })
            || shutdown_sit.is_some() || mode_change.is_some() || pending_swap.is_some()
            || seat_transition_until.is_some_and(|until| tick_start < until);
        state.human_other_action_busy.store(other_action_busy, Ordering::Release);
        state.human_action_busy.store(
            other_action_busy || controller.as_ref().is_some_and(Controller::precision_active),
            Ordering::Release,
        );
        state.human_motion_busy.store(
            twist_ema.iter().any(|velocity| velocity.abs() > 0.005), Ordering::Release,
        );
        let active_skill = controller.as_ref().and_then(Controller::active_skill_name)
            .or(seat_transition_until.filter(|until| tick_start < *until).map(|_| "sit_toggle"));
        let timeout = state.last_skill_timeout.load();
        let recovery_skill = controller.as_ref().filter(|controller| controller.skill_recovering())
            .and_then(|_| timeout.as_ref().map(|timeout| timeout.skill.as_str()));
        state.publish_active_skill(active_skill.or(recovery_skill));

        // Explicit mouth intent also applies during startup hold and the home ramp.
        // An active instrument can replace it below; the safety layer bounds the travel.
        targets[duck_control::model::MOUTH_INDEX] =
            idle.mouth_target(snapshot.mouth, driving);

        // The theremin: a hand's distance in front of the beak, turned into a note and a
        // mouth opening. Before the mouth is written, because while an instrument is up it
        // *is* what the mouth is doing — the intent from a client is not competing with it.
        let mut theremin_state = None;
        if let Some(instrument) = theremin.as_mut() {
            // Whether an instrument could be picked up right now, for the IPC side to
            // refuse on. Republished every tick because the sensor can go away under a
            // running daemon.
            state.theremin_ready.store(
                instrument.has_frames() && voice.is_some(),
                Ordering::Relaxed,
            );
            if let Some(active) = intents.take_theremin_request() {
                instrument.set_active(active);
            }
            // Nothing takes the instrument away but asking. Walking used to drop it, because
            // a captured background is a picture of one spot — with no background there is
            // nothing to invalidate, and a duck that plays while it walks is a feature.
            let note = instrument.tick(tick_start);
            match note {
                Some(note) => {
                    let mut block = note.state;
                    // Silence keeps the last pitch rather than gliding to the bottom of the
                    // range: a fade at the note you were playing is a note ending, and a fade
                    // on the way down is a note falling over.
                    let level = match note.closeness {
                        Some(closeness) => {
                            if let Some(voice) = voice.as_mut()
                                && let Some(hz) = voice.theremin_hz_at(closeness)
                            {
                                theremin_hz = hz;
                            }
                            block.note_hz = Some(theremin_hz);
                            1.0
                        }
                        None => 0.0,
                    };
                    if let Some(voice) = voice.as_mut() {
                        // Idempotent, so this is also what picks the instrument up on the
                        // first tick after the request — there is no separate start edge to
                        // get wrong.
                        if voice.theremin_start() {
                            voice.theremin_set(theremin_hz, level, note.mouth);
                        }
                    }
                    // Whatever the policy is doing, unlike the mouth *intent* below. The
                    // mouth is not part of any policy, and a duck playing a theremin while
                    // sitting — which is how anyone will first try this — has to be able to
                    // open its beak: gating on `driving` made the visible half of the whole
                    // gesture silently absent on a sitting robot.
                    if snapshot.enabled && bringup == Bringup::Ready {
                        targets[duck_control::model::MOUTH_INDEX] =
                            duck_control::model::mouth_target(note.mouth);
                    }
                    theremin_state = Some(block);
                }
                // The instrument is down. Putting the voice down too is idempotent, so this
                // covers both "it was just dropped" and "there has never been one".
                None => {
                    if let Some(voice) = voice.as_mut() {
                        voice.theremin_stop();
                    }
                }
            }
        }

        // The chorale: where in the piece the ensemble is, and this duck's line of it. Before the
        // mouth, like the theremin, because while a duck is singing its beak is doing that.
        let mut chorale_state = None;
        if let Some(ensemble) = chorale.as_mut() {
            if let Some((active, piece_pin)) = intents.take_chorale_request() {
                ensemble.set_active(active, tick_start, piece_pin);
            }
            for heard in intents.take_chorale_heard() {
                ensemble.heard(&heard, tick_start);
            }
            let tick = ensemble.tick(tick_start);
            if let Some(advertise) = tick.advertise {
                // Handed to whatever `btd` connection is subscribed. Only when it changes, which
                // is about once a beat rather than fifty times a second.
                let _ = state.chorale_tx.send(advertise);
            }
            // The mouth: a target from the vowel being sung, written on every tick **while a
            // chorale is up** — which is the bug this replaced. Writing it only while a note
            // actually sounded left the target at whatever the hold pose had put there between
            // notes, so a beak could simply stay open.
            //
            // "While a chorale is up" and not "while this robot may sing", which was the *second*
            // bug: owning the mouth whenever `[chorale] accept` was true meant the trigger could
            // not open it any more, on a robot that was not singing and might never sing. Opting in
            // grants the chorale nothing until it is actually running.
            let mut head_target = [0.0f64; 4];
            let mouth_target = match tick.singing {
                Some((part, beats)) => {
                    if let Some(voice) = voice.as_mut()
                        && voice.sing_start(ensemble.score(), part)
                    {
                        voice.sing_at(beats, true);
                    }
                    let line = ensemble.score().line(part);
                    let (mut low, mut high) = (127.0f64, 0.0f64);
                    let mut current = None;
                    for note in line {
                        low = low.min(f64::from(note.midi));
                        high = high.max(f64::from(note.midi));
                        if beats >= note.start_beat && beats < note.end_beat() {
                            current = Some(*note);
                        }
                    }
                    match current {
                        Some(note) => {
                            // Where this note sits in the duck's own line, for the head lift.
                            let reach = ((f64::from(note.midi) - low) / (high - low).max(1.0))
                                .clamp(0.0, 1.0);
                            head_target = chorale::head_expression(beats, reach);
                            // The audio releases the last 8% of a note so a repeated pitch
                            // re-articulates; the beak does the same, visibly — without this a
                            // run of same-vowel notes reads as one long weird note.
                            let sung = (beats - note.start_beat) / note.beats;
                            if sung < 0.92 {
                                note.vowel.open()
                            } else {
                                note.vowel.open() * 0.2
                            }
                        }
                        // Between notes: beak closed, but keep swaying — a singer breathing is
                        // still part of the choir.
                        None => {
                            head_target = chorale::head_expression(beats, 0.0);
                            0.0
                        }
                    }
                }
                None => {
                    if let Some(voice) = voice.as_mut() {
                        voice.sing_stop();
                    }
                    0.0
                }
            };
            if ensemble.active() {
                // Slewed, not snapped. A vowel is a step — `ah` is 0.90 and `mm` is 0.05 — and a
                // servo asked to jump between them on a 20 ms tick twitches rather than sings.
                // The head rides the same slew: the sway targets are already smooth sinusoids,
                // and the slew is what makes the *start and stop* of singing gentle.
                let alpha = (period.as_secs_f64() / CHORALE_MOUTH_TAU_S).clamp(0.0, 1.0);
                chorale_mouth += (mouth_target - chorale_mouth) * alpha;
                for (offset, target) in chorale_head.iter_mut().zip(head_target) {
                    *offset += (target - *offset) * alpha;
                }
                if snapshot.enabled && bringup == Bringup::Ready {
                    targets[duck_control::model::MOUTH_INDEX] =
                        duck_control::model::mouth_target(chorale_mouth);
                }
                chorale_state = Some(proto::ChoraleState {
                    listening: true,
                    part: tick.singing.map(|(part, _)| part.as_str().to_owned()),
                    beats: tick.singing.map(|(_, beats)| beats),
                    joining: tick.joining,
                    voices: tick.voices as u32,
                });
            } else {
                // Nothing on the wire and nothing on the servo: the mouth belongs to whoever else
                // wants it, which is the trigger — and the head settles back to where it was
                // asked to look, through the same slew rather than a snap.
                chorale_mouth = 0.0;
                let alpha = (period.as_secs_f64() / CHORALE_MOUTH_TAU_S).clamp(0.0, 1.0);
                for offset in chorale_head.iter_mut() {
                    *offset += (0.0 - *offset) * alpha;
                }
            }
        }

        let head_takeover = snapshot.maze.is_some() || !snapshot.enabled || snapshot.pose.active
            || safety.fallen() || in_limp_fall || shutdown_sit.is_some() || shutdown_rest.is_some()
            || state.exit_sit.load(Ordering::Acquire) != 0 || powered_off
            || bringup != Bringup::Ready || mode_change.is_some() || pending_swap.is_some()
            || controller.as_ref().is_none_or(|controller| controller.busy() || controller.is_sitting())
            || (driving && !matches!(policy_label.as_ref(), "stand" | "walk"));
        let head_available = !head_takeover && driving && !inference_failed && bringup == Bringup::Ready;
        let measured_head = fresh.as_ref().map(|sensors| [sensors.positions[6], sensors.positions[7]])
            .filter(|positions| positions.iter().all(|position| position.is_finite()));
        if snapshot.abandon_generation != previous_abandon_generation {
            if snapshot.head_generation == snapshot.abandon_head_generation {
                head_handoff.hold_current(measured_head, snapshot.head_generation);
            }
            previous_abandon_generation = snapshot.abandon_generation;
        }
        if !head_available && snapshot.coordinated_tracking.is_some() {
            target_head_tracker.reset();
            let arrival_action_owns_motion = coordinated_tracker.arrival_action_running()
                && snapshot.enabled && !autonomous_paused && !in_limp_fall && !snapshot.pose.active
                && shutdown_sit.is_none() && shutdown_rest.is_none() && !powered_off
                && state.exit_sit.load(Ordering::Acquire) == 0 && bringup == Bringup::Ready
                && mode_change.is_none() && pending_swap.is_none()
                && snapshot.stop_generation == intents.snapshot().stop_generation;
            let failed_arrival_step = inference_failed && coordinated_tracker.arrived()
                && snapshot.enabled && !safety.fallen() && !in_limp_fall && !snapshot.pose.active
                && shutdown_sit.is_none() && shutdown_rest.is_none() && !powered_off
                && state.exit_sit.load(Ordering::Acquire) == 0 && bringup == Bringup::Ready
                && mode_change.is_none() && pending_swap.is_none()
                && snapshot.stop_generation == intents.snapshot().stop_generation;
            if !arrival_action_owns_motion && !failed_arrival_step
                && (head_takeover || !coordinated_tracker.arrived())
            {
                if coordinated_tracker.arrival_action_running() {
                    cancel_arrival_skill(controller.as_mut(), &mut coordinated_unwritten_sit);
                    coordinated_action_cancelled = true;
                }
                coordinated_tracker.reset();
            }
            coordinated_initial_head.pause();
            coordinated_tick = None;
            coordinated_head = None;
        }
        let mut tracked_head = coordinated_head;
        if let (Some(tracking), Some(positions)) = (snapshot.head_tracking.filter(|tracking| tracking.pitch_position.is_none()), measured_head)
            && head_available
        {
            if head_tracking_sequence != Some(tracking.sequence) {
                head_tracker.reset();
                head_tracking_sequence = Some(tracking.sequence);
            }
            if let Some(head) = head_tracker.step(
                positions, tracking.target, tracking.deadzone, tracking.gains, dt,
            ) {
                tracked_head = Some(head);
            } else {
                tracing::error!("invalid head tracking state");
                head_tracker.reset();
                intents.cancel_head_tracking();
            }
        } else if snapshot.head_tracking.is_none() || head_takeover || !head_available || measured_head.is_none() {
            head_tracker.reset();
            head_tracking_sequence = None;
        }
        let preview_position = snapshot.head_tracking.and_then(|tracking| tracking.pitch_position)
            .or_else(|| {
                if snapshot.head_tracking.is_none() && snapshot.coordinated_tracking.is_none() {
                    snapshot.motion_tracking.and_then(|tracking| tracking.initial_pitch)
                        .or(snapshot.head_position_handoff.filter(|_| !snapshot.head_return_home))
                } else {
                    None
                }
            });
        let mut head_override = head_handoff.update(
            snapshot.head_tracking.is_some() || snapshot.coordinated_tracking.is_some() || preview_position.is_some(),
            snapshot.motion_tracking.is_some(), tracked_head, measured_head, [home[6], home[7]],
            snapshot.head_generation, head_takeover, snapshot.head_return_home, dt)
            .filter(|_| head_available);
        if preview_position.is_some() {
            head_override = None;
        }
        if head_available && let Some(head) = manual_head {
            head_override = Some(head);
        }
        if let Some(head) = head_override {
            targets[6] = head[0];
            targets[7] = head[1];
        }
        if let (Some(raw), Some(measured)) = (preview_position, measured_head) {
            if head_available {
                if head_positioning.as_ref().is_none_or(|(previous, _)| *previous != raw) {
                    let position = duck_control::hx::decode_position(6, &raw.to_le_bytes())
                        .expect("validated head position");
                    let target = [position, measured[1]];
                    let motion = if raw == duck_control::hx::PHYSICAL_HOME_ENCODER_TARGETS[6] {
                        head_tracking::HeadReturn::gentle_home(measured, target)
                    } else { head_tracking::HeadReturn::new(measured, target) };
                    head_positioning = Some((raw, motion));
                }
                if let Some((_, motion)) = &head_positioning {
                    targets[6] = motion.next(dt)[0];
                }
            } else {
                head_positioning = None;
            }
        } else {
            head_positioning = None;
        }

        if coordinated_action_cancelled || coordinated_tracker.arrival_action_running() {
            let current = intents.snapshot();
            let still_owned = !coordinated_action_cancelled && current.enabled
                && current.stop_generation == snapshot.stop_generation
                && state.exit_sit.load(Ordering::Acquire) == 0
                && current.coordinated_tracking.as_ref().is_some_and(|tracking| {
                    Some(tracking.sequence) == coordinated_tracking_sequence
                        && tracking.fresh && tracking.params.camera.is_some()
                });
            if !still_owned {
                if !coordinated_action_cancelled {
                    cancel_arrival_skill(controller.as_mut(), &mut coordinated_unwritten_sit);
                }
                coordinated_tracker.reset();
                coordinated_mouth.reset();
                coordinated_head = None;
                head_override = None;
                let manual_takeover = coordinated_action_cancelled && manual_action_started
                    && current.enabled && current.stop_generation == snapshot.stop_generation
                    && !stop_changed && !safety.fallen() && !in_limp_fall && bringup == Bringup::Ready
                    && shutdown_sit.is_none() && shutdown_rest.is_none() && !powered_off
                    && state.exit_sit.load(Ordering::Acquire) == 0;
                if !manual_takeover { targets = coast.known_positions(hold); }
                tracing::warn!("coordinated arrival action cancelled before actuator write");
            }
        }

        let mut coordinated_mouth_command = None;
        if coordinated_mouth.active() {
            let current = intents.snapshot();
            let mouth_owned = !coordinated_action_cancelled && !manual_action_started
                && current.enabled && current.stop_generation == snapshot.stop_generation
                && state.exit_sit.load(Ordering::Acquire) == 0 && !safety.fallen() && !in_limp_fall
                && !current.pose.active && bringup == Bringup::Ready && !powered_off
                && shutdown_sit.is_none() && shutdown_rest.is_none() && mode_change.is_none()
                && pending_swap.is_none() && theremin_state.is_none() && chorale_state.is_none()
                && current.coordinated_tracking.as_ref().is_some_and(|tracking|
                    Some(tracking.sequence) == coordinated_tracking_sequence
                        && tracking.fresh && tracking.params.camera.is_some())
                && controller.as_ref().is_some_and(|controller|
                    coordinated_tracker.arrival_action_running()
                        || (!controller.busy() && !controller.is_sitting()));
            if !mouth_owned {
                coordinated_mouth.reset();
            } else if !driving || inference_failed || fresh.as_ref().is_none_or(|sensors|
                !sensors.positions[duck_control::model::MOUTH_INDEX].is_finite())
            {
                if driving && !inference_failed {
                    if let Some(position) = coordinated_mouth.pause() {
                        targets[duck_control::model::MOUTH_INDEX] = position;
                    }
                } else {
                    coordinated_mouth.pause();
                }
            } else if let Some(position) = coordinated_mouth.command(tick_start, snapshot.mouth) {
                targets[duck_control::model::MOUTH_INDEX] = position;
                coordinated_mouth_command = Some(position);
            }
        }

        // A manual jaw request can operate without the walking policy. After the
        // operator releases it, allow time to close before releasing its torque.
        let jaw_available = state.startup_voltage_ready.load(Ordering::Acquire)
            && !state.low_voltage_lock.requires_rearm()
            && !status_page_active && maintenance.is_none()
            && shutdown_sit.is_none() && shutdown_rest.is_none() && !powered_off
            && !state.stopping.load(Ordering::Acquire) && state.exit_sit.load(Ordering::Acquire) == 0;
        let jaw = if !jaw_available {
            mouth_close_until = None;
            None
        } else if snapshot.mouth >= duck_ipc_proto::mouth_gesture::PRESSURE_START {
            mouth_close_until = Some(tick_start + Duration::from_millis(450));
            Some(snapshot.mouth)
        } else if mouth_close_until.is_some_and(|deadline| tick_start < deadline) {
            Some(0.0)
        } else {
            None
        };
        // An idle mouth default must not turn a current-pose hold into a new target.
        let torque_only_passive_hold = state.init_torque_only && bringup == Bringup::Ready
            && !policy_enabled_for_tick && shutdown_sit.is_none() && shutdown_rest.is_none()
            && !powered_off && targets.iter().zip(hold.iter()).enumerate().all(|(index, (target, current))|
                index == duck_control::model::MOUTH_INDEX || target == current);
        if torque_only_passive_hold {
            let index = duck_control::model::MOUTH_INDEX;
            if let Some(open) = jaw {
                targets[index] = duck_control::model::mouth_target(open);
                if open == 0.0 { hold[index] = targets[index]; }
            } else {
                targets[index] = hold[index];
            }
        }

        let status_actively_holding = status_page_active && state.exit_sit.load(Ordering::Acquire) == 0
            && !state.stopping.load(Ordering::Acquire) && !safety.fallen() && !in_limp_fall
            && shutdown_sit.is_none() && shutdown_rest.is_none() && !powered_off;
        if status_actively_holding && let Some(pose) = status_hold_pose
        {
            if status_pose_latched { targets = pose; }
            else { targets[5..10].copy_from_slice(&pose[5..10]); }
        }

        if let Some(RestMotion::Action { playback, .. }) = shutdown_rest.as_ref() {
            targets = playback.target();
        }

        let write_guard = state.human_control.lock();
        let mut person_loss_mouth_starting = false;
        if let Some((sequence, opened_at)) = person_loss_mouth {
            let current = intents.snapshot();
            let can_open = coordinated_eligible && head_available && !unsafe_motion
                && !manual_action_started && !coordinated_action_cancelled
                && jaw_available && jaw.is_none() && !coordinated_mouth.active()
                && !maintenance_holding && !write_guard.emergency_release_active()
                && !write_guard.maintenance_transition_blocked() && !write_guard.status_blocked()
                && theremin_state.is_none() && chorale_state.is_none()
                && fresh.as_ref().is_some_and(|sample| sample.positions[duck_control::model::MOUTH_INDEX].is_finite())
                && current.enabled && current.stop_generation == snapshot.stop_generation
                && current.mouth < proto::mouth_gesture::PRESSURE_START
                && current.coordinated_tracking.as_ref().is_some_and(|tracking| {
                    tracking.sequence == sequence && tracking.fresh && tracking.params.camera.is_some()
                        && tracking.params.stop_area_px.is_some()
                        && (opened_at.is_some() || tracking.params.target.is_none())
                });
            if can_open && opened_at.is_none_or(|at| tick_start.duration_since(at) < Duration::from_millis(350)) {
                targets[duck_control::model::MOUTH_INDEX] = duck_control::model::mouth_target(1.0);
                person_loss_mouth_starting = opened_at.is_none();
            } else {
                person_loss_mouth = None;
            }
        }
        read_timing.mark_stage(4);
        last_targets = targets;
        let apply_start_t_ns = proto::clock::monotonic_ns();
        let limp_jaw = jaw.filter(|_| bringup == Bringup::Limp && !policy_enabled_for_tick
            && !write_guard.emergency_release_active() && !write_guard.maintenance_transition_blocked()
            && !write_guard.status_blocked() && fresh.is_some());
        let wrote_targets = !maintenance_holding && !write_guard.emergency_release_active()
            && limp_jaw.is_none();
        let write_ok = if !wrote_targets {
            true
        } else {
            let mut timing = duck_control::execution_timing::StageTimer::<2>::new(
                "action_apply", "safety_apply",
            );
            // Confirm the native gain before sending targets for the new phase. On a
            // partial write, skip those targets and retry confirmation on the next tick.
            let applied = limp_fall
                .apply_position_p_gain(&mut safety, fall_position_gains)
                .and_then(|()| {
                    if let Some(RestMotion::Action { playback, .. }) = shutdown_rest.as_mut() {
                        let Some((positions, duration_ms)) = playback.pending_frame() else {
                            return Ok(duck_control::safety::Applied::default());
                        };
                        let applied = safety.apply_timed_frame(positions, duration_ms, targets, hold, gain)?;
                        if !applied.limited_by(duck_control::safety::Limit::NotFinite) {
                            playback.confirm_sent(Instant::now());
                        }
                        Ok(applied)
                    } else if status_pose_latched && status_actively_holding {
                        safety.hold_torque_only(targets, gain)
                    } else if torque_only_passive_hold {
                        safety.hold_torque_only(targets, gain)
                    } else {
                        safety.apply(targets, hold, gain)
                    }
                });
            match applied {
                Ok(applied) => {
                    timing.advance("limit_merge");
                    limits.extend(applied.limits);
                    true
                }
                Err(e) => {
                    timing.advance("write_error_log");
                    tracing::warn!(error = %e, "bus write failed");
                    false
                }
            }
        };
        if let Err(error) = safety.write_limp_mouth(limp_jaw) {
            tracing::warn!(%error, "isolated jaw write failed");
        }
        // A due R2 reversal is a single write behind the main motor command.
        // Skipping it when a tick has little spare time leaves a pause at the endpoint.
        if wrote_targets && write_ok && gamepad_ready && !stop_changed && fresh.is_some()
            && !status_page_active && !write_guard.maintenance_transition_blocked()
            && shutdown_rest.is_none() && !state.stopping.load(Ordering::Acquire)
            && state.exit_sit.load(Ordering::Acquire) == 0
            && let Err(error) = safety.write_accessory_trigger(intents.accessory_trigger())
        {
            tracing::warn!(%error, "accessory servo write failed");
        }
        drop(write_guard);
        if person_follow && coordinated_eligible && !unsafe_motion && !tracking_stopped
            && driving && !inference_failed
            && wrote_targets && write_ok && !limits.contains(&duck_control::safety::Limit::NotFinite)
        {
            if snapshot.coordinated_tracking.as_ref().is_some_and(|tracking| tracking.params.target.is_some()) {
                if coordinated_tracker.confirm_person_follow_motion(requested.twist, command.twist)
                    && intents.snapshot().coordinated_tracking.is_some_and(|current| {
                        current.fresh && Some(current.sequence) == coordinated_tracking_sequence
                            && current.params.target.is_some()
                    })
                    && let Some(voice) = voice.as_mut()
                {
                    voice.play(proto::SoundTag::Mouth.as_str(), false);
                    tracing::info!("person follow mouth sound requested at the target distance");
                }
            } else if requested.twist[2] < 0.0 && command.twist[2] < 0.0
                && coordinated_tracker.confirm_person_search_turn() && person_loss_mouth.is_none()
            {
                person_loss_mouth = coordinated_tracking_sequence.map(|sequence| (sequence, None));
            }
        }
        if person_loss_mouth_starting && wrote_targets && write_ok
            && !limits.contains(&duck_control::safety::Limit::NotFinite)
        {
            if let Some((_, opened_at)) = &mut person_loss_mouth {
                *opened_at = Some(tick_start);
            }
            if let Some(voice) = voice.as_mut() {
                voice.play(proto::SoundTag::Mouth.as_str(), false);
                tracing::info!("person search mouth sound requested after two turns");
            }
        }
        if wrote_targets && write_ok && !limits.contains(&duck_control::safety::Limit::NotFinite)
            && rest_sitting && driving && !inference_failed && fresh.is_some() && policy_label.as_ref() == "sit"
        {
            rest_sit_remaining = rest_sit_remaining.saturating_sub(period);
        }
        if let Some(position) = coordinated_mouth_command {
            if wrote_targets && write_ok && !limits.contains(&duck_control::safety::Limit::NotFinite) {
                coordinated_mouth.confirm(position, tick_start, snapshot.mouth);
            } else {
                coordinated_mouth.pause();
            }
        }
        if wrote_targets && write_ok && !limits.contains(&duck_control::safety::Limit::NotFinite)
            && let Some((_, motion)) = &mut head_positioning
        {
            motion.confirm([targets[6], targets[7]]);
            head_handoff.confirm([targets[6], targets[7]]);
            if let Some(controller) = controller.as_mut() {
                controller.anchor_head_targets([targets[6], targets[7]]);
            }
        }
        if write_ok && !limits.contains(&duck_control::safety::Limit::NotFinite)
            && let Some(head) = head_override
        {
            head_handoff.confirm(head);
            if coordinated_head.is_some() {
                if wrote_targets { coordinated_initial_head.confirm(head); }
                coordinated_tracker.confirm_head(head);
                if target_follow && wrote_targets {
                    target_head_rate.confirm(head);
                }
            }
            if let Some(controller) = controller.as_mut() {
                controller.anchor_head_targets(head);
            }
        }
        if write_ok && !limits.contains(&duck_control::safety::Limit::NotFinite)
            && !inference_failed && driving && policy_label.as_ref() == "sit"
            && coordinated_tracker.arrival_action_running()
        {
            coordinated_unwritten_sit = false;
        }
        if write_ok && !limits.contains(&duck_control::safety::Limit::NotFinite)
            && fresh.is_some() && head_available && !inference_failed && driving
            && snapshot.coordinated_tracking.is_some() && policy_label.as_ref() == "stand"
            && controller.as_ref().is_some_and(|controller| controller.driving() == Some(Driving::Stand))
            && intents.snapshot().stop_generation == snapshot.stop_generation
        {
            coordinated_tracker.confirm_stand_written();
        }
        let apply_end_t_ns = proto::clock::monotonic_ns();
        read_timing.mark_stage(5);

        if let Some(tracking) = snapshot.coordinated_tracking.as_ref()
            && pickup_action_completed == Some((tracking.sequence, snapshot.stop_generation))
            && (coordinated_eligible && head_available || pickup_seated)
            && !unsafe_motion && !manual_action_started && !coordinated_action_cancelled
            && state.exit_sit.load(Ordering::Acquire) == 0
            && wrote_targets && write_ok && !inference_failed && driving
            && !limits.contains(&duck_control::safety::Limit::NotFinite)
            && !coordinated_mouth.finishing()
            && (pickup_seated || !coordinated_tracker.arrived() || coordinated_tracker.arrival_action_finished())
        {
            intents.advance_pickup(tracking.sequence, snapshot.stop_generation);
        }
        use proto::CoordinatedTrackingPhase as Phase;
        let recovering_autonomous_motion = (autonomous_paused || safety.fallen() || in_limp_fall)
            && (snapshot.coordinated_tracking.is_some()
                || snapshot.motion_tracking.is_some() || snapshot.maze.is_some());
        let tracking_phase = if recovering_autonomous_motion {
            Some(Phase::Fallen)
        } else {
            snapshot.coordinated_tracking.as_ref().map(|tracking| {
                if !tracking.fresh || tracking.params.camera.is_none() { Phase::PausedVision }
                else if coordinated_mouth.finishing() { Phase::MouthAfterAction }
                else if coordinated_mouth.preparing() { Phase::MouthBeforeAction }
                else if coordinated_tracker.arrival_action_running() { Phase::RunningAction }
                else if fresh.is_none() || coordinated_feedback_missing { Phase::WaitingFeedback }
                else if !snapshot.enabled || !driving || bringup != Bringup::Ready { Phase::WaitingRobot }
                else if !coordinated_initial_head.arrived() { Phase::InitialHead }
                // Reuse existing wire phases without changing the shared IPC version.
                else if tracking.params.head_only() {
                    if tracking.params.target.is_some() { Phase::Approach } else { Phase::WaitingTarget }
                }
                else { coordinated_tracker.phase() }
            })
        };
        if let Some(recorder) = recorder.as_ref().filter(|_| recording) {
            let servo_diagnostics = safety.record_servo_diagnostics(period.saturating_sub(tick_start.elapsed()));
            let mut sample = record::Sample::new(
                state.ticks.load(Ordering::Relaxed) + 1,
                read_at,
                sensors.as_ref().map(|_| sensors_read_ns),
                state.started.elapsed().as_secs_f64(),
                fresh.is_some(),
                sensors.as_ref(),
                policy_trace.as_ref().map(|trace| trace.observation),
                policy_trace.as_ref().and_then(|trace| trace.action),
                policy_output_t_ns,
                apply_start_t_ns,
                apply_end_t_ns,
                record::CommandSample::new(
                    requested.twist,
                    snapshot.command.head,
                    snapshot.pose.body,
                    snapshot.pose.active,
                ),
                record::CommandSample::new(
                    command.twist,
                    command.head,
                    [command.body.z, command.body.roll, command.body.pitch],
                    snapshot.pose.active,
                ),
                policy_label.as_ref().to_owned(),
                driving,
                inference_failed,
                safety.fallen(),
                matches!(limp_fall, LimpFall::Limp { .. }),
                gain,
                safety.gain(),
                targets,
                record::LimitFlags::from_limits(&limits),
                write_ok,
                tick_start.elapsed().as_secs_f64() * 1000.0,
            );
            sample.policy_imu_input = policy_imu_input;
            sample.policy_gravity_input = policy_gravity_input;
            sample.servo_diagnostics = servo_diagnostics;
            if let Some(tracking) = snapshot.coordinated_tracking.as_ref()
                .filter(|tracking| tracking.params.stop_area_px.is_some())
            {
                sample.person_follow = Some(record::PersonFollowSample {
                    realtime_ns: proto::clock::realtime_ns(),
                    sequence: tracking.sequence,
                    phase: tracking_phase,
                    update_age_us: tracking.age_us,
                    vision_fresh: tracking.fresh,
                    camera: tracking.params.camera,
                    target: tracking.params.target,
                    head_position_raw: tracking.initial_pitch,
                    head_arrived: coordinated_initial_head.arrived(),
                    control_eligible: coordinated_eligible,
                    capture_feedback_missing: coordinated_feedback_missing,
                    tracking_stopped,
                    deadzone: tracking_deadzone,
                    settings: (&tracking.params).into(),
                    decision: coordinated_tracker.person_follow_diagnostic(&tracking.params),
                });
            }
            if let Some(tracking) = snapshot.coordinated_tracking.as_ref()
                .filter(|tracking| tracking.params.stop_area_px.is_none())
            {
                let head_only = tracking.params.head_only();
                sample.target_follow = Some(record::TargetFollowSample {
                    realtime_ns: proto::clock::realtime_ns(), sequence: tracking.sequence,
                    mode: if head_only { "head_only" } else { "coordinated" },
                    phase: tracking_phase, update_age_us: tracking.age_us, vision_fresh: tracking.fresh,
                    camera: tracking.params.camera, target: tracking.params.target,
                    head_position_raw: tracking.initial_pitch, head_arrived: coordinated_initial_head.arrived(),
                    head_target_rad: head_override,
                    control_eligible: coordinated_eligible, capture_feedback_missing: coordinated_feedback_missing,
                    tracking_stopped, deadzone: tracking_deadzone, settings: (&tracking.params).into(),
                    decision: (!head_only).then(|| coordinated_tracker.target_follow_diagnostic()),
                    stability: (!head_only).then(|| target_follow_stability.diagnostic()),
                });
            }
            recorder.record(sample);
        }

        if tracking_phase != coordinated_previous_phase {
            tracing::info!(phase = ?tracking_phase, "coordinated tracking phase changed");
            coordinated_previous_phase = tracking_phase;
        }

        // Only assemble a frame when somebody is subscribed. On a robot nobody usually is,
        // and this would otherwise be a per-tick allocation on the thread that should not
        // be visiting the allocator without a reason.
        if state.state_tx.receiver_count() > 0
            && let Some(sensors) = sensors.as_ref()
        {
            tracing::debug!(
                target: "control_sample",
                t = state.started.elapsed().as_secs_f64(),
                fresh = fresh.is_some(),
                freeze_policy_imu,
                read_errors = state.consecutive_errors.load(Ordering::Relaxed),
                elapsed_ms = tick_start.elapsed().as_secs_f64() * 1000.0,
                driving,
                policy = %policy_label,
                gyro = ?sensors.imu.gyro,
                gravity = ?sensors.imu.gravity,
                positions = ?sensors.positions,
                velocities = ?sensors.velocities,
                targets = ?targets,
                "control sample"
            );
            let policy = telemetry.label(policy_label.as_ref());
            telemetry.publish(telemetry::Snapshot {
                captured_at: Instant::now(),
                t: state.started.elapsed().as_secs_f64(),
                t_ns: sensors_read_ns,
                sensors: *sensors,
                targets,
                requested: requested.twist,
                applied: command.twist,
                limited: [duck_control::safety::Limit::Deadman, duck_control::safety::Limit::Range,
                    duck_control::safety::Limit::NotFinite].map(|limit| limits.contains(&limit)),
                head: command.head,
                policy,
                tracking_phase,
                pickup_stage: snapshot.coordinated_tracking.as_ref().and_then(|tracking| tracking.pickup_stage),
                safety: proto::SafetyState {
                    fallen: safety.fallen(),
                    // Limp means "the robot is actually at limp gain" — which now happens
                    // in exactly one place, the limp-fall sequence riding a fall down. A
                    // bare fall verdict is a report, not a state.
                    limp: matches!(limp_fall, LimpFall::Limp { .. }),
                    gravity: sensors.imu.gravity,
                    gain: safety.gain(),
                },
                control_loop: proto::LoopState {
                    hz: f64::from_bits(state.achieved_hz.load(Ordering::Relaxed)),
                    missed: state.missed.load(Ordering::Relaxed),
                },
                // Empty is *not reported* on this wire, and both of these have two ways of
                // being that: `[control] publish_velocity_and_load = false` on a robot whose
                // operator does not want the bytes, and a backend with nothing to report —
                // `--fake` has no servos, and a simulator may send no load. Neither may become
                // a block of zeros, which reads as a robot at rest under no load.
                publish_velocity: publish_measurements && safety.measures_velocity(),
                publish_load: publish_measurements && safety.measures_load(),
                odom: proto::OdomState {
                    position: odometry.position(),
                    yaw: odometry.yaw(),
                },
                theremin: theremin_state.clone(),
                chorale: chorale_state.clone(),
            });
        }

        let ticks = state.ticks.fetch_add(1, Ordering::Relaxed) + 1;
        state.last_tick_us.store(
            state.started.elapsed().as_micros() as u64,
            Ordering::Relaxed,
        );
        window_ticks += 1;
        let window = window_start.elapsed();
        if window >= RATE_WINDOW {
            if let Some((_, maze)) = maze_controller.as_ref() {
                tracing::info!(phase = maze.phase(), reason = maze_wait_reason,
                    head_actual = ?sensors.as_ref().map(|s| s.positions[7]),
                    head_target = ?maze_head, head_servo_target = targets[7],
                    requested = ?requested.twist,
                    applied = ?command.twist, driving, limited_by = ?limits,
                    tof = ?state.obstacle.frame(proto::clock::monotonic_ns()),
                    "maze control");
            }
            let hz = window_ticks as f64 / window.as_secs_f64();
            state.achieved_hz.store(hz.to_bits(), Ordering::Relaxed);
            window_start = Instant::now();
            window_ticks = 0;

            publish_slow_sensors(&mut safety, &state);
            safety.poll_diagnostics(period.saturating_sub(tick_start.elapsed()));

            if last_summary.elapsed() >= LOOP_SUMMARY_INTERVAL {
                tracing::info!(
                    total = ticks,
                    hz = format!("{hz:.1}"),
                    missed = state.missed.load(Ordering::Relaxed),
                    // Every dropped read is also logged individually.
                    bus_drops = bus_drops_window,
                    // How many of those ticks the policy actually stepped on stale data.
                    stale_steps = stale_steps_window,
                    driving,
                    fallen = safety.fallen(),
                    battery_v = format!(
                        "{:.2}",
                        f64::from_bits(state.battery_v.load(Ordering::Relaxed))
                    ),
                    motor_max_c = format!(
                        "{:.0}",
                        f64::from_bits(state.motor_max_c.load(Ordering::Relaxed))
                    ),
                    cpu_c = format!(
                        "{:.0}",
                        f64::from_bits(state.cpu_temp_c.load(Ordering::Relaxed))
                    ),
                    "control loop"
                );
                last_summary = Instant::now();
                bus_drops_window = 0;
                stale_steps_window = 0;
            }
        }
        if state.devices_tx.receiver_count() == 0 {
            device_schedule.reset();
            safety.end_device_observation();
        } else {
            // The HX backend publishes cached replies from the primary control read.
            sample_devices(&state, &mut device_schedule, period, tick_start + period,
                |deadline, generation| {
                    if let Some(generation) = generation { safety.begin_device_observation(generation); }
                    safety.sample_device_observation(deadline)
                });
        }
        read_timing.mark_stage(6);
        let tick_elapsed = tick_start.elapsed();
        if tick_elapsed > period {
            state.missed.fetch_add(1, Ordering::Relaxed);
            if let Some(run) = read_run.as_mut() {
                run.overran_ticks += 1;
            }
        }
        read_timing.observe_tick(tick_elapsed, period);

        // Report after counting this tick's overrun, including periodic diagnostics.
        if read_run.as_ref().is_some_and(|run| run.recovered)
            && let Some(run) = read_run.take()
        {
            run.report();
        }
    }
    if let Some(pending) = managed_swap.take() {
        let status = if pending.committed || pending.committing.is_some() { "uncertain" } else { "failed" };
        state.managed_models.finish(&pending.request.operation, status, Some("控制服务已停止".into()));
    }
    if let Some(request) = state.managed_models.take() {
        state.managed_models.finish(&request.operation, "failed", Some("控制服务已停止".into()));
    }
    read_timing.report();
    if let Err(error) = status_only.close(&mut safety, &state) {
        tracing::error!(%error, "cannot close read-only status session on shutdown");
    }
    if state.status_bus_fault.load(Ordering::Acquire)
        || state.human_control.lock().emergency_release_active()
    {
        tracing::error!("control loop stops without bus writes; status or torque release remains unconfirmed");
        if state.exit_sit.load(Ordering::Acquire) != 0 {
            state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
        }
        return;
    }
    if status_session_open {
        if let Err(error) = status_bus::complete(&state.status_journal) {
            tracing::error!(%error, "cannot close status session on shutdown");
            return;
        }
    }
    if maintenance_parked {
        tracing::error!("control loop stopped while the motor bus was owned by maintenance");
        return;
    }
    if state.release_torque_on_exit.load(Ordering::Relaxed)
        && (!state.low_voltage_lock.requires_rearm() || state.low_voltage.status().triggered)
    {
        let safe_to_position = !shutdown_after_maintenance
            && state.exit_sit_succeeded.load(Ordering::Acquire)
            && !safety.fallen() && safety.imu_ready()
            && state.consecutive_errors.load(Ordering::Relaxed) == 0;
        let released = finish_exit(&mut safety, safe_to_position, state.exit_head_timing).await.is_ok();
        state.exit_release_verified.store(released && state.bus_timeout_supported, Ordering::Release);
    }
    if !shutdown_after_maintenance && !state.low_voltage_lock.requires_rearm() {
        if let Err(error) = LimpFall::Idle.apply_position_p_gain(&mut safety, fall_position_gains) {
            tracing::error!(%error, "shutdown normal position P restoration failed");
        }
    } else if state.exit_sit.load(Ordering::Acquire) != 0 {
        state.exit_sit.store(EXIT_SIT_DONE, Ordering::Release);
    }
    tracing::info!("control loop stopped");
}

#[derive(Clone, Copy)]
struct ExitHeadTiming {
    first: Duration,
    delay: Duration,
    second: Duration,
}

impl Default for ExitHeadTiming {
    fn default() -> Self {
        Self {
            first: Duration::from_millis(EXIT_HEAD_30_MS),
            delay: Duration::from_millis(EXIT_HEAD_31_DELAY_MS),
            second: Duration::from_millis(EXIT_HEAD_31_MS),
        }
    }
}

async fn finish_exit<T: RobotIo>(safety: &mut Safety<T>, seated: bool, timing: ExitHeadTiming) -> duck_control::io::Result<()> {
    if seated {
        if let Err(error) = position_head_for_exit(safety, timing).await {
            tracing::error!(%error, "exit head positioning failed; releasing torque");
        }
    } else {
        tracing::info!("exit head positioning skipped: sit or sensor state unavailable");
    }
    safety.set_torque(false).inspect_err(|error| tracing::error!(%error, "shutdown torque off failed"))
}

async fn position_head_for_exit<T: RobotIo>(safety: &mut Safety<T>, timing: ExitHeadTiming) -> duck_control::io::Result<()> {
    const IDS: [u8; 2] = [30, 31];
    const TARGET: [u16; 2] = [1500, 1300];
    let mut initial = safety.read_positions_raw(&IDS)?;
    if initial.len() != IDS.len() {
        return Err(duck_control::io::IoError::Bus("incomplete exit head position read".into()));
    }
    tracing::info!(?initial, ?TARGET, first_ms = timing.first.as_millis() as u64,
        delay_ms = timing.delay.as_millis() as u64, second_ms = timing.second.as_millis() as u64,
        "exit head positioning started");
    let started = Instant::now();
    let mut ticker = tokio::time::interval(Duration::from_millis(20));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut second_started_at = None;
    let mut first_done = false;
    let mut second_done = false;
    loop {
        ticker.tick().await;
        if !first_done {
            let elapsed = started.elapsed().min(timing.first);
            let fraction = elapsed.as_secs_f64() / timing.first.as_secs_f64();
            let first = (f64::from(initial[0])
                + (f64::from(TARGET[0]) - f64::from(initial[0])) * fraction).round() as u16;
            safety.write_positions_raw(&IDS[..1], &[first])?;
            first_done = elapsed >= timing.first;
        }
        if started.elapsed() >= timing.delay && second_started_at.is_none()
            && (timing.delay < timing.first || first_done)
        {
            let current = safety.read_positions_raw(&IDS[1..])?;
            if current.len() != 1 {
                return Err(duck_control::io::IoError::Bus("incomplete second head position read".into()));
            }
            initial[1] = current[0];
            second_started_at = Some(Instant::now());
        }
        if let Some(second_started_at) = second_started_at && !second_done {
            let second_elapsed = second_started_at.elapsed().min(timing.second);
            let second_fraction = second_elapsed.as_secs_f64() / timing.second.as_secs_f64();
            let second = (f64::from(initial[1])
                + (f64::from(TARGET[1]) - f64::from(initial[1])) * second_fraction).round() as u16;
            safety.write_positions_raw(&IDS[1..], &[second])?;
            second_done = second_elapsed >= timing.second;
        }
        if first_done && second_done { break; }
    }
    let measured = safety.read_positions_raw(&IDS)?;
    if measured.len() != IDS.len() {
        return Err(duck_control::io::IoError::Bus("incomplete final head position read".into()));
    }
    if measured.iter().zip(TARGET).any(|(&actual, target)| actual.abs_diff(target) > 20) {
        tracing::warn!(?measured, ?TARGET, "exit head has not reached its target; releasing torque");
    } else {
        tracing::info!(?measured, "exit head positioning complete");
    }
    Ok(())
}

/// Stable wire names for the reasons a command was altered.
///
/// Spelled out rather than `Debug`-formatted: this goes over the wire, and a client
/// branching on it must not break because a variant was renamed in Rust.
/// A path's file name, for reporting which policy is loaded. `None` for a path that ends in
/// something that is not a file name — reported as unknown rather than as an empty string,
/// which would read as "no policy".
fn file_name(path: &std::path::Path) -> Option<String> {
    Some(path.file_name()?.to_string_lossy().into_owned())
}

/// The sample a tick steps from, and how stale it may get.
///
/// Exists as a type rather than three variables in the loop because it *was* three variables
/// in the loop, and the bug lived in the gap between them: a failed read stopped the policy
/// for one tick, which commanded `hold` — a pose only ever assigned when driving stops, and
/// the assignment needed a sample, which is the one thing a failed read does not have. Mid
/// stride that meant one tick of "snap to the home pose" at full gain, then a controller
/// reset on the way back in. Roughly eight times a minute on a loaded board.
struct Coast {
    last: Option<duck_control::Sensors>,
    ticks: u32,
}

impl Coast {
    fn new() -> Self {
        Self {
            last: None,
            ticks: 0,
        }
    }

    /// Feed a tick's read result; get back what the policy may step from.
    fn sample(&mut self, fresh: Option<duck_control::Sensors>) -> Option<duck_control::Sensors> {
        match fresh {
            Some(sensors) => {
                self.ticks = 0;
                self.last = Some(sensors);
                Some(sensors)
            }
            None if self.ticks < COAST_TICKS => {
                self.ticks += 1;
                self.last
            }
            None => None,
        }
    }

    /// Where the joints were the last time anything was actually read — what a fallback
    /// holds. `fallback` covers the ticks before the first successful read.
    fn known_positions(&self, fallback: [f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
        self.last.map_or(fallback, |sensors| sensors.positions)
    }
}

struct ServoReadTiming {
    enabled: bool,
    samples_ns: Vec<u64>,
    tick_samples_ns: Vec<u64>,
    stage_samples_ns: [Vec<u64>; 7],
    stage_tick_ns: [u64; 7],
    stage_last_mark: Instant,
    max_tick_ns: u64,
    max_tick_stages_ns: [u64; 7],
    failures: u64,
    tick_over_period: u64,
    last_report: Instant,
    last_tick_ms: Option<f64>,
}

impl ServoReadTiming {
    fn new(enabled: bool) -> Self {
        Self {
            enabled,
            samples_ns: Vec::new(),
            tick_samples_ns: Vec::new(),
            stage_samples_ns: std::array::from_fn(|_| Vec::new()),
            stage_tick_ns: [0; 7],
            stage_last_mark: Instant::now(),
            max_tick_ns: 0,
            max_tick_stages_ns: [0; 7],
            failures: 0,
            tick_over_period: 0,
            last_report: Instant::now(),
            last_tick_ms: None,
        }
    }

    fn begin_tick(&mut self, started: Instant) {
        if self.enabled {
            self.stage_last_mark = started;
        }
    }

    fn mark_stage(&mut self, index: usize) {
        if !self.enabled {
            return;
        }
        let now = Instant::now();
        self.stage_tick_ns[index] = now
            .duration_since(self.stage_last_mark)
            .as_nanos()
            .min(u128::from(u64::MAX)) as u64;
        self.stage_last_mark = now;
    }

    fn observe(&mut self, success: bool, elapsed: Option<Duration>) {
        if !self.enabled {
            return;
        }
        match (success, elapsed) {
            (true, Some(elapsed)) => {
                let ns = elapsed.as_nanos().min(u128::from(u64::MAX)) as u64;
                self.samples_ns.push(ns);
            }
            _ => self.failures += 1,
        }
    }

    fn observe_tick(&mut self, elapsed: Duration, period: Duration) {
        if !self.enabled {
            return;
        }
        let ns = elapsed.as_nanos().min(u128::from(u64::MAX)) as u64;
        self.tick_samples_ns.push(ns);
        for (samples, stage_ns) in self.stage_samples_ns.iter_mut().zip(self.stage_tick_ns) {
            samples.push(stage_ns);
        }
        if ns > self.max_tick_ns {
            self.max_tick_ns = ns;
            self.max_tick_stages_ns = self.stage_tick_ns;
        }
        self.last_tick_ms = Some(ns as f64 / 1_000_000.0);
        if elapsed > period {
            self.tick_over_period += 1;
        }
        if self.last_report.elapsed() >= Duration::from_secs(1) {
            tracing::debug!(target: "robotd::control_timing",
                "timing {:>5} | 1 read   {:>6.2} | 2 sens   {:>6.2} | 3 state  {:>6.2} | 4 policy {:>6.2}",
                self.tick_samples_ns.len(),
                self.stage_tick_ns[0] as f64 / 1_000_000.0,
                self.stage_tick_ns[1] as f64 / 1_000_000.0,
                self.stage_tick_ns[2] as f64 / 1_000_000.0,
                self.stage_tick_ns[3] as f64 / 1_000_000.0,
            );
            tracing::debug!(target: "robotd::control_timing",
                "timing {:>5} | 5 target {:>6.2} | 6 write  {:>6.2} | 7 pub    {:>6.2} | total    {:>6.2} | >20 {:>2} | err {:>2}",
                self.tick_samples_ns.len(),
                self.stage_tick_ns[4] as f64 / 1_000_000.0,
                self.stage_tick_ns[5] as f64 / 1_000_000.0,
                self.stage_tick_ns[6] as f64 / 1_000_000.0,
                self.last_tick_ms.unwrap_or(0.0),
                self.tick_over_period,
                self.failures,
            );
            self.last_report = Instant::now();
        }
    }

    fn report(&mut self) {
        if !self.enabled {
            return;
        }
        if self.samples_ns.is_empty() {
            tracing::warn!(
                failures = self.failures,
                "diagnostic: no full bus read timing samples"
            );
            return;
        }
        self.samples_ns.sort_unstable();
        let percentile = |p: f64| {
            let index = ((self.samples_ns.len() - 1) as f64 * p).round() as usize;
            self.samples_ns[index] as f64 / 1_000_000.0
        };
        tracing::debug!(target: "robotd::control_timing",
            "timing total | n {:>5} | err {:>2} | >20 {:>2} | max {:>6.2} ms",
            self.tick_samples_ns.len(),
            self.failures,
            self.tick_over_period,
            self.max_tick_ns as f64 / 1_000_000.0,
        );
        tracing::debug!(target: "robotd::control_timing",
            "timing read  | n {:>5} | min {:>6.2} | p50 {:>6.2} | p99 {:>6.2} ms",
            self.samples_ns.len(),
            self.samples_ns[0] as f64 / 1_000_000.0,
            percentile(0.50),
            percentile(0.99),
        );
        const STAGES: [&str; 7] = [
            "1 read", "2 sens", "3 state", "4 policy", "5 target", "6 write", "7 pub",
        ];
        tracing::debug!(target: "robotd::control_timing",
            "timing stats | {:<8} {:>6} {:>6} {:>6} {:>6}",
            "stage",
            "mean",
            "p95",
            "max",
            "at_max"
        );
        for (index, (name, samples)) in STAGES
            .into_iter()
            .zip(&mut self.stage_samples_ns)
            .enumerate()
        {
            samples.sort_unstable();
            let n = samples.len();
            let p95 = ((n - 1) as f64 * 0.95).round() as usize;
            tracing::debug!(target: "robotd::control_timing",
                "timing stats | {:<8} {:>6.2} {:>6.2} {:>6.2} {:>6.2}",
                name,
                samples.iter().copied().sum::<u64>() as f64 / n as f64 / 1_000_000.0,
                samples[p95] as f64 / 1_000_000.0,
                samples[n - 1] as f64 / 1_000_000.0,
                self.max_tick_stages_ns[index] as f64 / 1_000_000.0,
            );
        }
    }
}

/// One run of dropped bus reads, for the line that says how bad the spasm got.
///
/// Every drop gets its own diagnostic line. What a *run* costs is different in kind:
/// every failed read makes
/// the next tick step on the sample before it, so for as long as the run lasts the network's
/// observation is no longer the robot it is commanding. Three in a row is what turned a
/// stand-up into a "do nothing" crouch, and the journal said nothing about it.
///
/// So the run is accumulated here and summarised once, on the tick that recovers — never
/// rate-limited, because a spasm is news — and a run that outlives the coast window is called
/// out when the policy loses its sensors entirely.
struct ReadRun {
    started: Instant,
    failures: u32,
    last_error: Arc<duck_control::IoError>,
    /// Ticks the loop stepped from a coasted sample rather than a fresh one.
    coasted_ticks: u32,
    /// Coasted ticks whose work overran the period, so the schedule lost a slot too.
    overran_ticks: u32,
    /// True once the policy drove during the run — that is when the stale sample is acted on.
    driving: bool,
    /// The run outlived [`COAST_TICKS`] and the loop stopped stepping the policy.
    ran_out: bool,
    recovered: bool,
}

impl ReadRun {
    fn new(now: Instant, error: Arc<duck_control::IoError>) -> Self {
        Self {
            started: now,
            failures: 0,
            last_error: error,
            coasted_ticks: 0,
            overran_ticks: 0,
            driving: false,
            ran_out: false,
            recovered: false,
        }
    }

    fn failed(&mut self, error: Arc<duck_control::IoError>) {
        self.failures += 1;
        self.last_error = error;
    }

    /// Whether a finished run is worth a line of its own. A drop that recovered before the
    /// policy ever stepped on it is already covered by the per-failure line.
    fn reports(&self) -> bool {
        self.coasted_ticks > 0 || self.ran_out
    }

    /// The line for a finished run.
    fn report(&self) {
        if !self.reports() {
            return;
        }
        tracing::warn!(
            failures = self.failures,
            blind_ms = self.started.elapsed().as_millis() as u64,
            coasted_ticks = self.coasted_ticks,
            overran_ticks = self.overran_ticks,
            driving = self.driving,
            stale = self.coasted_ticks > 0,
            policy_holds = self.ran_out,
            "bus reads recovered after a run"
        );
    }
}

/// Whether resuming from `stopped_at` warrants resetting the controller.
///
/// The reset zeroes the action history the policy observes and drops the low-pass anchor.
/// After a real pause that is right — the robot may be somewhere else entirely. After a
/// 20 ms bus hiccup it is itself the discontinuity the reset exists to prevent.
fn reset_on_resume(stopped_at: Option<Instant>, now: Instant) -> bool {
    stopped_at.is_none_or(|since| now.duration_since(since) >= RESET_AFTER_PAUSE)
}

/// Whether a voice bank has anything to play. One directory walk at startup: the bank is a
/// tree of per-tag directories, and an empty one (postinstall's `ensure-bank` failed, which
/// the hook deliberately downgrades to a warning) is exactly the case a `robot.sound` must
/// not answer "accepted" to.
fn has_any_wav(bank: &std::path::Path) -> bool {
    let Ok(tags) = std::fs::read_dir(bank) else {
        return false;
    };
    tags.filter_map(Result::ok).any(|tag| {
        std::fs::read_dir(tag.path()).is_ok_and(|mut wavs| {
            wavs.any(|w| w.is_ok_and(|w| w.path().extension().is_some_and(|ext| ext == "wav")))
        })
    })
}

fn limit_name(limit: duck_control::safety::Limit) -> &'static str {
    use duck_control::safety::Limit;
    match limit {
        Limit::Deadman => "deadman",
        Limit::Range => "joint_range",
        Limit::NotFinite => "not_finite",
    }
}

/// Sample and publish everything that does not need sampling every tick, once per
/// [`RATE_WINDOW`].
///
/// HX returns voltage/temperature from its complete main-read cache. Board temperature
/// is sampled independently by the state-output worker.
///
/// Called from the loop thread because that thread owns the IO, and nothing else may touch
/// the bus: a transaction issued from the IPC side would interleave bytes with a tick and
/// corrupt both. The IMU counters come from the same `io`, so they are mirrored here rather
/// than reached for from the socket.
fn publish_slow_sensors<T: RobotIo>(io: &mut Safety<T>, state: &RobotState) {
    let stale = io.imu_stale();
    state.imu_stale_blocks.store(stale.total, Ordering::Relaxed);
    state.imu_stale_run.store(stale.run, Ordering::Relaxed);
    state.imu_ready.store(io.imu_ready(), Ordering::Relaxed);

    match io.slow_sensors() {
        Ok(slow) => {
            let previous = f64::from_bits(state.battery_v.load(Ordering::Relaxed));
            // Seed from the first reading rather than blending up from zero, which would
            // spend ten seconds reporting a battery flatter than it is.
            let smoothed = if previous > 0.0 {
                BATTERY_EMA_ALPHA * slow.volts + (1.0 - BATTERY_EMA_ALPHA) * previous
            } else {
                slow.volts
            };
            state.battery_v.store(smoothed.to_bits(), Ordering::Relaxed);

            // Temperature is not smoothed: a servo's case is already a slow signal, and an
            // EMA would only delay the one reading anybody cares about — the joint climbing
            // towards its overheat shutdown.
            let (hottest, max_c) = slow.temps_c.iter().enumerate().fold(
                (0usize, f64::MIN),
                |(best, high), (joint, &t)| {
                    if t > high { (joint, t) } else { (best, high) }
                },
            );
            let mean_c = slow.temps_c.iter().sum::<f64>() / slow.temps_c.len() as f64;
            state.motor_max_c.store(max_c.to_bits(), Ordering::Relaxed);
            state
                .motor_mean_c
                .store(mean_c.to_bits(), Ordering::Relaxed);
            state.motor_hottest.store(hottest as u32, Ordering::Relaxed);
        }
        // Keep the last sample. A single failed transaction is ordinary on a serial bus, and
        // dropping to "unknown" over one would make the reported battery flicker. A bus that
        // is really gone already shows up in the verdict and in `bus.consecutive_errors`.
        Err(e) => tracing::debug!(error = %e, "slow-sensor read failed; keeping the last sample"),
    }
}

/// Own an endpoint even when no listener exists, as during startup or standalone init.
fn claim_lock(socket_path: &Path) -> std::io::Result<std::fs::File> {
    use std::fs::{OpenOptions, TryLockError};
    use std::io::{Error, ErrorKind};

    if let Some(parent) = socket_path.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }

    // Append, rather than replace the extension: distinct --socket paths need distinct
    // locks. Never unlink this file, even at shutdown — a waiter could already have the
    // old inode open. The kernel releases the lock on exit, including SIGKILL.
    let mut lock_path = socket_path.as_os_str().to_os_string();
    lock_path.push(".lock");
    let lock = OpenOptions::new()
        .create(true)
        .read(true)
        .write(true)
        .truncate(false)
        .open(lock_path)?;
    match lock.try_lock() {
        Ok(()) => (),
        Err(TryLockError::WouldBlock) => {
            return Err(Error::new(
                ErrorKind::AddrInUse,
                "another robotd owns this socket",
            ));
        }
        Err(TryLockError::Error(e)) => return Err(e),
    }
    Ok(lock)
}

/// Claim one daemon endpoint, including the window before there is a listener to probe.
async fn claim_socket(socket_path: &Path, lock: std::fs::File) -> std::io::Result<(std::fs::File, UnixListener)> {
    use std::io::ErrorKind;
    use std::os::unix::fs::{FileTypeExt, PermissionsExt};

    // Before bind, not after it. A regular file or a symlink at the path is a typo, and it is
    // refused here on every platform rather than left to the kernel: Linux fails to bind over
    // either, which is what made this look enforced, but macOS follows a dangling symlink and
    // creates the socket at its target, so a daemon there came up serving through a path nobody
    // asked for. That is the one platform `cargo test --workspace` is promised on.
    match std::fs::symlink_metadata(socket_path) {
        Ok(existing) if !existing.file_type().is_socket() => {
            return Err(std::io::Error::new(
                ErrorKind::AddrInUse,
                "the socket path exists and is not a socket; refusing to bind over it",
            ));
        }
        Ok(_) => (),
        Err(e) if e.kind() == ErrorKind::NotFound => (),
        Err(e) => return Err(e),
    }
    let listener = match UnixListener::bind(socket_path) {
        Ok(listener) => listener,
        Err(e) if e.kind() == ErrorKind::AddrInUse => {
            // An older daemon may own the socket without holding our new lock. Only a
            // real socket that refuses connections is stale; a timeout or a permission error
            // is not permission to remove somebody else's path.
            match tokio::time::timeout(Duration::from_secs(1), UnixStream::connect(socket_path))
                .await
            {
                Ok(Err(probe)) if probe.kind() == ErrorKind::ConnectionRefused => {
                    tracing::warn!(path = %socket_path.display(), "removing stale socket");
                    std::fs::remove_file(socket_path)?;
                    UnixListener::bind(socket_path)?
                }
                Ok(Err(probe)) => return Err(probe),
                _ => return Err(e),
            }
        }
        Err(e) => return Err(e),
    };
    std::fs::set_permissions(socket_path, std::fs::Permissions::from_mode(SOCKET_MODE))?;
    Ok((lock, listener))
}

async fn serve(
    state: Arc<RobotState>,
    intents: Arc<Intents>,
    listener: UnixListener,
    socket_path: PathBuf,
    gamepad_pid: Option<u32>,
) -> std::io::Result<()> {
    tracing::info!(
        path = %socket_path.display(),
        mode = format!("{SOCKET_MODE:o}"),
        model_api = MODEL_API,
        "serving robot IPC"
    );

    let mut lease_tick = tokio::time::interval(Duration::from_millis(50));
    loop {
        let accepted = tokio::select! {
            accepted = listener.accept() => accepted,
            _ = lease_tick.tick() => {
                drop(state.human_control.lock());
                continue;
            }
        };
        let (stream, _) = match accepted {
            Ok(pair) => pair,
            Err(e) => {
                tracing::warn!(error = %e, "accept failed");
                continue;
            }
        };
        let is_gamepad = if let Some(pid) = gamepad_pid {
            match stream.peer_cred().ok().and_then(|cred| cred.pid()) {
                Some(peer) => peer == pid as i32,
                None => {
                    tracing::error!("cannot identify IPC peer; rejecting connection");
                    continue;
                }
            }
        } else {
            false
        };
        let state = Arc::clone(&state);
        let intents = Arc::clone(&intents);
        tokio::spawn(async move {
            let result = handle(state, intents, stream, is_gamepad).await;
            if let Err(e) = result {
                tracing::debug!(error = %e, "connection ended");
            }
        });
    }
}

fn clear_human_intents(intents: &Intents) {
    clear_human_intents_with_stop(intents, false);
}

fn clear_human_intents_with_stop(intents: &Intents, smooth: bool) {
    clear_human_motion(intents, smooth);
    intents.clear_head();
}

fn clear_human_motion(intents: &Intents, smooth: bool) {
    intents.cancel_head_tracking();
    intents.cancel_coordinated_tracking();
    if smooth {
        // Do not raise stop_generation: that is the immediate-stop signal that
        // cancels an already executing skill. Let the skill finish while new
        // requests are refused by the maintenance lease.
        intents.stop_for_maintenance();
    } else {
        intents.stop();
    }
    intents.set_pose(intents::PoseIntent::default());
    intents.set_mouth(proto::MouthParams::default());
    let _ = intents.take_skills();
    let _ = intents.take_sounds();
    let _ = intents.take_sound_clip();
    intents.cancel_pending_rest();
    intents.cancel_pending_shutdown();
    if matches!(intents.take_power_request(), Some(intents::PowerRequest::Relax)) {
        intents.request_relax();
    }
    intents.clear_sound_hold();
}

fn is_motion_intent(call: &proto::Call) -> bool {
    matches!(call,
        proto::Call::RobotMove(_) | proto::Call::RobotPrecisionMove(_) | proto::Call::RobotSetObstacle(_)
        | proto::Call::RobotHead(_) | proto::Call::RobotHeadTracking(_) | proto::Call::RobotMotionTracking(_) | proto::Call::RobotCoordinatedTracking(_) | proto::Call::RobotMaze(_) | proto::Call::RobotLook(_) | proto::Call::RobotStop
        | proto::Call::RobotEnable(_) | proto::Call::RobotInit | proto::Call::RobotRelax
        | proto::Call::RobotRebootMotors(_) | proto::Call::RobotRest | proto::Call::RobotDo(_)
        | proto::Call::RobotPose(_) | proto::Call::RobotMouth(_) | proto::Call::RobotAccessoryTrigger(_)
        | proto::Call::RobotSound(_)
        | proto::Call::RobotSoundClip(_)
        | proto::Call::RobotTheremin(_) | proto::Call::RobotChorale(_)
        | proto::Call::RobotShutdown | proto::Call::RobotSetMode(_)
        | proto::Call::RobotLoadPolicy(_) | proto::Call::RobotReloadPolicies
        | proto::Call::RobotSetSkill(_) | proto::Call::RobotRemoveSkill(_))
}

/// Movement initiated by a person, as opposed to the page's tracking and maze commands.
fn is_manual_movement(call: &proto::Call) -> bool {
    matches!(call,
        proto::Call::RobotMove(_) | proto::Call::RobotHead(_) | proto::Call::RobotLook(_)
        | proto::Call::RobotPose(_) | proto::Call::RobotDo(_) | proto::Call::RobotSetMode(_)
        | proto::Call::RobotPrecisionMove(proto::PrecisionMoveParams::Start { .. }))
}

async fn handle(
    state: Arc<RobotState>,
    intents: Arc<Intents>,
    stream: UnixStream,
    is_gamepad: bool,
) -> std::io::Result<()> {
    let mut session = human_control::Session::new();
    let result = handle_session(Arc::clone(&state), Arc::clone(&intents), stream, is_gamepad, &mut session).await;
    intents.update_head_tracking(session.id(), proto::HeadTrackingParams { pitch_position: None, action: proto::MotionTrackingAction::Stop, target: None, deadzone: [0.0; 2], gains: [[0.0; 3]; 2] });
    intents.update_motion_tracking(session.id(), proto::MotionTrackingParams { initial_pitch: None, action: proto::MotionTrackingAction::Stop, target: None, line_speed: None, line_target_x: None, reference_area: 1.0, deadzone: [0.0; 2], gains: [[0.0; 3]; 2] });
    intents.update_coordinated_tracking(session.id(), proto::CoordinatedTrackingParams { initial_pitch: None,
        action: proto::MotionTrackingAction::Stop, target: None, camera: None,
        head_deadzone: [0.0; 2], head_gains: [[0.0; 3]; 2], head_target_point: None, body_gains: [0.0; 3],
        final_body_gains: [0.0; 3], approach_linear_limit: None,
        final_linear_limit: proto::COORDINATED_TRACKING_DEFAULT_FINAL_LINEAR_LIMIT,
        final_lateral_limit: proto::COORDINATED_TRACKING_DEFAULT_FINAL_LINEAR_LIMIT,
        final_point: proto::COORDINATED_TRACKING_DEFAULT_FINAL_POINT,
        final_tolerance: proto::COORDINATED_TRACKING_DEFAULT_FINAL_TOLERANCE,
        stop_area_px: None, stop_area_deadzone_px: 0, stop_distance_mm: None, stop_distance_deadzone_mm: 0, arrival_delay_s: proto::COORDINATED_TRACKING_DEFAULT_ARRIVAL_DELAY_S,
        arrival_skill: String::new(),
        mouth_before: proto::TimedMouthPosition::default(),
        mouth_after: proto::TimedMouthPosition::default(),
        pickup_after: None,
    });
    intents.update_maze(session.id(), proto::MazeParams { action: proto::MotionTrackingAction::Stop, threshold_mm: 1,
        head_step_deg: 0.0, settle_ms: 0, sample_ms: 0, restore_ms: 0, turn_initial_ms: 0, turn_final_ms: 0, linear_speed: 0.0, angular_speed: 0.0 });
    {
        let mut guard = state.human_control.lock();
        if guard.disconnect(&mut session) {
            clear_human_intents(&intents);
            tracing::warn!("human control disconnected; motion stopped");
        }
    }
    result
}

async fn handle_session(
    state: Arc<RobotState>,
    intents: Arc<Intents>,
    stream: UnixStream,
    is_gamepad: bool,
    session: &mut human_control::Session,
) -> std::io::Result<()> {
    let (read_half, mut write_half) = stream.into_split();
    let mut lines = BufReader::new(read_half).lines();

    // `None` until the client subscribes. Once set, the connection is both a request
    // channel and a state stream, so the loop below waits on whichever speaks first.
    let mut states: Option<tokio::sync::broadcast::Receiver<proto::RobotState>> = None;
    let mut devices: Option<tokio::sync::broadcast::Receiver<proto::DeviceObservation>> = None;
    // What to advertise, once `btd` has subscribed. A second stream on the same connection, which
    // is why the read loop selects over a pair of options rather than one receiver.
    let mut beacons: Option<tokio::sync::broadcast::Receiver<proto::ChoraleAdvertise>> = None;
    let mut decimate = Duration::ZERO;
    let mut _subscription: Option<telemetry::Subscription> = None;
    let mut _device_subscription: Option<telemetry::Subscription> = None;
    let mut last_sent: Option<Instant> = None;
    let mut next_device_due: Option<Instant> = None;

    loop {
        // Three things can happen: a request arrives, a state frame is due for a subscriber, or a
        // beacon is due for `btd`. Written as nested selects rather than one, because the two
        // streams are independent options and a client normally has neither.
        let line = match (states.as_mut(), beacons.as_mut()) {
            (None, None) => lines.next_line().await?,
            (None, Some(rx)) => {
                tokio::select! {
                    line = lines.next_line() => line?,
                    received = rx.recv() => {
                        match received {
                            Ok(advertise) => {
                                write_line(
                                    &mut write_half,
                                    &proto::Request::notify(&proto::Call::ChoraleBeaconSet(advertise)),
                                )
                                .await?;
                            }
                            // Lagged: `btd` fell behind on beats. The newest beacon is the only
                            // one worth having — an old beat is a beat that has already passed.
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                                tracing::debug!(dropped = n, "chorale subscriber fell behind");
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Closed) => return Ok(()),
                        }
                        continue;
                    }
                }
            }
            (Some(rx), beacon_rx) => {
                let mut pending = beacon_rx;
                let mut pending_devices = devices.as_mut();
                tokio::select! {
                    line = lines.next_line() => line?,
                    received = async {
                        match pending.as_mut() {
                            Some(rx) => rx.recv().await,
                            // Never resolves, so the select falls to the other arms.
                            None => std::future::pending().await,
                        }
                    } => {
                        match received {
                            Ok(advertise) => {
                                write_line(
                                    &mut write_half,
                                    &proto::Request::notify(&proto::Call::ChoraleBeaconSet(advertise)),
                                )
                                .await?;
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                                tracing::debug!(dropped = n, "chorale subscriber fell behind");
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Closed) => return Ok(()),
                        }
                        continue;
                    }
                    received = rx.recv() => {
                        match received {
                            Ok(frame) => {
                                // Decimate per subscriber: a dashboard asking for 10 Hz
                                // should not cost what a digital twin asking for 50 does.
                                let due = decimate <= state.subscriptions.period(Duration::from_micros(state.period_us))
                                    || last_sent
                                    .map(|at| at.elapsed() >= decimate)
                                    .unwrap_or(true);
                                if due {
                                    last_sent = Some(Instant::now());
                                    write_line(&mut write_half, &proto::Request::notify_state(&frame))
                                        .await?;
                                }
                            }
                            // Lagged: the client fell behind and lost frames. That is the
                            // designed behaviour — state is advisory and must never apply
                            // backpressure to the control loop — so carry on from the newest.
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                                tracing::debug!(dropped = n, "state subscriber fell behind");
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Closed) => return Ok(()),
                        }
                        continue;
                    }
                    received = async {
                        match pending_devices.as_mut() {
                            Some(rx) => rx.recv().await,
                            None => std::future::pending().await,
                        }
                    } => {
                        match received {
                            Ok(frame) => {
                                let now = Instant::now();
                                if next_device_due.is_none_or(|due| now >= due) {
                                    next_device_due = Some(match next_device_due {
                                        Some(due) if due + decimate > now => due + decimate,
                                        _ => now + decimate,
                                    });
                                    write_line(&mut write_half, &proto::Request::notify_devices(&frame)).await?;
                                }
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                                tracing::debug!(dropped = n, "device subscriber fell behind");
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Closed) => return Ok(()),
                        }
                        continue;
                    }
                }
            }
        };
        let Some(line) = line else { return Ok(()) };

        if line.trim().is_empty() {
            continue;
        }
        if line.len() > MAX_LINE {
            let response = proto::Response::err(
                None,
                proto::Error::new(proto::code::INVALID_REQUEST, "request too large"),
            );
            write_line(&mut write_half, &response).await?;
            continue;
        }

        let request: proto::Request = match serde_json::from_str(&line) {
            Ok(request) => request,
            Err(e) => {
                let response = proto::Response::err(
                    None,
                    proto::Error::new(proto::code::PARSE_ERROR, e.to_string()),
                );
                write_line(&mut write_half, &response).await?;
                continue;
            }
        };

        let call = request.as_call();

        let transition_blocked = state.human_control.lock().maintenance_transition_blocked();
        if transition_blocked && call.as_ref().is_ok_and(|call| {
            (is_motion_intent(call) && !matches!(call, proto::Call::RobotStop
                | proto::Call::RobotShutdown | proto::Call::RobotSound(_) | proto::Call::RobotSoundClip(_)))
                || matches!(call, proto::Call::RobotLowVoltageRearm | proto::Call::RobotApplyModel(_))
                || matches!(call, proto::Call::RobotHumanControl(params)
                    if matches!(params.action, proto::HumanControlAction::PersonFollowHome
                        | proto::HumanControlAction::CenterHead))
        }) {
            if let Some(id) = request.id.clone() {
                let response = proto::Response::err(Some(id), proto::Error::new(
                    proto::code::INVALID_REQUEST, "维护页面正在接管，运动控制不可用"));
                write_line(&mut write_half, &response).await?;
            }
            continue;
        }

        if state.status_bus_fault.load(Ordering::Acquire) {
            let blocked = call.as_ref().is_ok_and(|call| {
                (is_motion_intent(call) && !matches!(call, proto::Call::RobotStop
                    | proto::Call::RobotSound(_) | proto::Call::RobotSoundClip(_)))
                    || matches!(call, proto::Call::RobotLowVoltageRearm | proto::Call::RobotApplyModel(_))
                    || matches!(call, proto::Call::RobotMaintenance(params)
                        if !matches!(params.action, proto::MaintenanceAction::Status
                            | proto::MaintenanceAction::AcquirePartial | proto::MaintenanceAction::Release
                            | proto::MaintenanceAction::InvalidateRest))
                    || matches!(call, proto::Call::RobotHumanControl(params)
                        if matches!(params.action, proto::HumanControlAction::PersonFollowHome
                            | proto::HumanControlAction::CenterHead))
            });
            if blocked {
                if let Some(id) = request.id.clone() {
                    let response = proto::Response::err(Some(id), proto::Error::new(
                        proto::code::INVALID_REQUEST, "设备状态回复未确认，串口已隔离"));
                    write_line(&mut write_half, &response).await?;
                }
                continue;
            }
        }

        if let Ok(proto::Call::RobotLowVoltageRearm) = &call {
            if let Some(id) = request.id.clone() {
                let result = {
                    let mut guard = state.human_control.lock();
                    if guard.access(session) != human_control::Access::Owner {
                        proto::IntentResult::refused("需要新的控制权才能启动")
                    } else if guard.manual_blocked() || guard.status_blocked() || guard.maintenance_transition_blocked() {
                        proto::IntentResult::refused("当前页面禁止手动启动")
                    } else if !state.startup_voltage_ready.load(Ordering::Acquire) {
                        proto::IntentResult::refused("电压尚未恢复或采样不可用")
                    } else if !state.low_voltage_lock.requires_rearm() {
                        proto::IntentResult::accepted()
                    } else {
                        match state.low_voltage_lock.rearm(Instant::now()) {
                            Ok(true) => {
                                // Revoke the old lease and all queued intents. The
                                // caller must acquire a fresh lease after this reply.
                                guard.emergency_stop();
                                clear_human_intents(&intents);
                                intents.cancel_motion_tracking();
                                intents.cancel_maze();
                                intents.set_enabled(false);
                                let _ = intents.take_power_request();
                                let _ = intents.take_reboot_motors();
                                let _ = intents.take_mode_switch();
                                proto::IntentResult::already(proto::LOW_VOLTAGE_REARMED)
                            }
                            Ok(false) => proto::IntentResult::refused("电压尚未稳定恢复"),
                            Err(error) => {
                                tracing::error!(%error, "cannot persist low-voltage rearm");
                                proto::IntentResult::refused("无法保存电压保护恢复状态")
                            }
                        }
                    }
                };
                write_line(&mut write_half, &proto::Response::ok(Some(id), &result)).await?;
            }
            continue;
        }

        // An operator must explicitly rearm after recovery. Reject requests
        // before they can be queued and accidentally run when voltage returns.
        // After confirmed voltage recovery an explicit release may turn torque off
        // during startup. Motion still requires a separate, freshly verified Start.
        if state.startup_read_only.load(Ordering::Acquire)
            && !state.low_voltage_lock.awaiting_start()
            && matches!(&call, Ok(proto::Call::RobotEmergencyRelax))
        {
            if let Some(id) = request.id {
                let result = proto::IntentResult::refused("电压恢复期间总线只读，紧急卸力无法确认");
                write_line(&mut write_half, &proto::Response::ok(Some(id), &result)).await?;
            }
            continue;
        }
        if (!state.startup_voltage_ready.load(Ordering::Acquire) || state.low_voltage_lock.requires_rearm())
            && call.as_ref().is_ok_and(|call| {
                is_motion_intent(call) && !matches!(call, proto::Call::RobotStop | proto::Call::RobotSetObstacle(_))
            })
        {
            if let Some(id) = request.id {
                let result = proto::IntentResult::refused("电压检测或保护恢复中，请等待后手动启动");
                write_line(&mut write_half, &proto::Response::ok(Some(id), &result)).await?;
            }
            continue;
        }

        if matches!(&call, Ok(proto::Call::RobotInit | proto::Call::RobotEnable(proto::EnableParams { on: true, .. })))
            && let Some(failures) = state.startup_servo_configuration_failures.load_full()
        {
            if let Some(id) = request.id {
                let reason = format!("舵机启动参数未确认：{}", startup_servo_failure_detail(&failures));
                write_line(&mut write_half, &proto::Response::ok(Some(id), &proto::IntentResult::refused(reason))).await?;
            }
            continue;
        }

        if let Ok(proto::Call::RobotEmergencyRelax) = &call {
            let queued = {
                let mut guard = state.human_control.lock();
                guard.begin_emergency_release().map(|()| {
                    intents.cancel_motion_tracking();
                    intents.cancel_maze();
                    clear_human_intents(&intents);
                    intents.resume_after_rest();
                    intents.set_enabled(false);
                    let _ = intents.take_power_request();
                    let _ = intents.take_reboot_motors();
                    let _ = intents.take_mode_switch();
                    state.emergency_relax.request(request.id.is_some())
                })
            };
            if let Some(id) = request.id {
                let result = match queued {
                    Err(reason) => proto::IntentResult::refused(reason),
                    Ok(Some(reply)) => match tokio::time::timeout(Duration::from_secs(25), reply).await {
                        Ok(Ok(Ok(()))) => proto::IntentResult::accepted(),
                        Ok(Ok(Err(reason))) => proto::IntentResult::refused(reason),
                        Ok(Err(_)) | Err(_) => proto::IntentResult::refused("卸力结果未确认"),
                    },
                    Ok(None) => unreachable!("request with id has a completion"),
                };
                write_line(&mut write_half, &proto::Response::ok(Some(id), &result)).await?;
            } else if let Err(reason) = queued {
                tracing::warn!(reason, "emergency torque release refused");
            }
            continue;
        }

        if let Ok(proto::Call::RobotMaintenance(params)) = &call {
            if let Some(id) = request.id.clone() {
                let result = maintenance_request(&state, &intents, session, *params).await;
                write_line(&mut write_half, &proto::Response::ok(Some(id), &result)).await?;
            }
            continue;
        }

        if state.exit_sit.load(Ordering::Acquire) != 0
            && call.as_ref().is_ok_and(is_motion_intent)
        {
            if let Some(id) = request.id {
                let response = proto::Response::err(Some(id), proto::Error::new(
                    proto::code::INVALID_REQUEST, "robot is exiting"));
                write_line(&mut write_half, &response).await?;
            }
            continue;
        }

        if let Ok(proto::Call::RobotHeadTracking(params)) = &call {
            let result = {
                let mut guard = state.human_control.lock();
                let web = session.source() == Some(proto::HumanControlSource::Web);
                match params.action {
                    proto::MotionTrackingAction::Start => {
                        let snapshot = intents.snapshot();
                        let reason = if !web { Some("网页控制连接未注册") }
                            else if guard.status_blocked() || guard.maintenance_transition_blocked() { Some("维护页面禁止运动") }
                            else if !params.valid() { Some("目标或头部 PID 参数无效") }
                            else if intents.maze_active() { Some("迷宫模式正在运行") }
                            else if intents.coordinated_tracking_active() { Some("联合追踪正在运行") }
                            else if !snapshot.enabled { Some("机器人策略未启用") }
                            else if snapshot.pose.active { Some("机器人处于姿态控制模式") }
                            else if state.fallen.load(Ordering::Acquire) { Some("机器人处于摔倒状态") }
                            else if state.sitting.load(Ordering::Acquire) { Some("机器人处于坐下状态") }
                            else if state.human_action_busy.load(Ordering::Acquire) { Some("机器人正在执行动作或尚未准备好") }
                            else { None };
                        if let Some(reason) = reason { Err(reason.to_owned()) }
                        else {
                            let already_owner = guard.access(session) == human_control::Access::Owner;
                            if !guard.acquire_if_free(session, proto::HumanControlSource::Web).accepted {
                                Err("网页无法取得控制权".to_owned())
                            } else {
                                if !already_owner { clear_human_intents(&intents); }
                                if intents.update_head_tracking(session.id(), *params) { Ok(()) }
                                else { Err("头部追踪意图被拒绝".to_owned()) }
                            }
                        }
                    }
                    proto::MotionTrackingAction::Update => {
                        if web && guard.access(session) == human_control::Access::Owner
                            && intents.update_head_tracking(session.id(), *params)
                            && guard.refresh(session, proto::HumanControlSource::Web).accepted
                        { Ok(()) } else { Err("头部追踪已失效或网页已失去控制权".to_owned()) }
                    }
                    proto::MotionTrackingAction::Stop => {
                        if web && guard.access(session) == human_control::Access::Owner
                            && intents.update_head_tracking(session.id(), *params)
                        {
                            if params.pitch_position.is_some() {
                                guard.refresh(session, proto::HumanControlSource::Web);
                            } else if !intents.motion_tracking_active() {
                                intents.request_head_home();
                                guard.abandon(session, proto::HumanControlSource::Web);
                            }
                            Ok(())
                        } else { Err("头部追踪已停止或网页已失去控制权".to_owned()) }
                    }
                }
            };
            if let Some(id) = request.id {
                let result = match result { Ok(()) => proto::IntentResult::accepted(),
                    Err(reason) => proto::IntentResult::refused(&reason) };
                write_line(&mut write_half, &proto::Response::ok(Some(id), &result)).await?;
            }
            continue;
        }

        if let Ok(proto::Call::RobotMotionTracking(params)) = &call {
            let result = {
                let mut guard = state.human_control.lock();
                let web = session.source() == Some(proto::HumanControlSource::Web);
                match params.action {
                    proto::MotionTrackingAction::Start => {
                        let snapshot = intents.snapshot();
                        let reason = if !web { Some("网页控制连接未注册") }
                            else if guard.status_blocked() || guard.maintenance_transition_blocked() { Some("维护页面禁止运动") }
                            else if !params.valid() { Some("目标或追踪参数无效") }
                            else if intents.maze_active() { Some("迷宫模式正在运行") }
                            else if intents.coordinated_tracking_active() { Some("联合追踪正在运行") }
                            else if !snapshot.enabled { Some("机器人策略未启用") }
                            else if snapshot.pose.active { Some("机器人处于姿态控制模式") }
                            else if state.policies.load().walk.is_none() { Some("未加载行走策略") }
                            else if state.human_action_busy.load(Ordering::Acquire) { Some("机器人正在执行动作或尚未准备好") }
                            else { None };
                        if let Some(reason) = reason { Err(reason.to_owned()) }
                        else if !guard.acquire_if_free(session, proto::HumanControlSource::Web).accepted {
                            Err("网页无法取得控制权".to_owned())
                        } else {
                            intents.stop();
                            if intents.update_motion_tracking(session.id(), *params) { Ok(()) }
                            else { Err("追踪意图被拒绝".to_owned()) }
                        }
                    }
                    proto::MotionTrackingAction::Update => {
                        if web && guard.access(session) == human_control::Access::Owner
                            && intents.update_motion_tracking(session.id(), *params)
                            && guard.refresh(session, proto::HumanControlSource::Web).accepted
                        { Ok(()) } else { Err("追踪已失效或网页已失去控制权".to_owned()) }
                    }
                    proto::MotionTrackingAction::Stop => {
                        if web && guard.access(session) == human_control::Access::Owner
                            && intents.update_motion_tracking(session.id(), *params)
                        {
                            intents.stop();
                            if !intents.head_tracking_active() {
                                intents.request_head_home();
                                guard.abandon(session, proto::HumanControlSource::Web);
                            }
                            Ok(())
                        } else { Err("追踪已停止或网页已失去控制权".to_owned()) }
                    }
                }
            };
            match (&params.action, &result) {
                (proto::MotionTrackingAction::Start, Ok(())) => tracing::info!(session = session.id(), "movement tracking started"),
                (proto::MotionTrackingAction::Stop, Ok(())) => tracing::info!(session = session.id(), "movement tracking stopped"),
                (_, Err(reason)) => tracing::warn!(session = session.id(), action = ?params.action, %reason, "movement tracking request refused"),
                _ => {}
            }
            if let Some(id) = request.id {
                let result = match result { Ok(()) => proto::IntentResult::accepted(),
                    Err(reason) => proto::IntentResult::refused(&reason) };
                write_line(&mut write_half, &proto::Response::ok(Some(id), &result)).await?;
            }
            continue;
        }

        if let Ok(proto::Call::RobotCoordinatedTracking(params)) = &call {
            let result = {
                let mut guard = state.human_control.lock();
                let web = session.source() == Some(proto::HumanControlSource::Web);
                match params.action {
                    proto::MotionTrackingAction::Start => {
                        let snapshot = intents.snapshot();
                        let reason = if !web { Some("网页控制连接未注册") }
                            else if guard.status_blocked() || guard.maintenance_transition_blocked() { Some("维护页面禁止运动") }
                            else if !params.valid() { Some("目标、相机或联合追踪参数无效") }
                            else if coordinated_tracking::geometry().is_err() { Some("公共头部运动学模型不可用") }
                            else if intents.maze_active() { Some("迷宫模式正在运行") }
                            else if intents.motion_tracking_active() || intents.head_tracking_active() {
                                Some("其他视觉追踪正在运行")
                            }
                            else if !snapshot.enabled { Some("机器人策略未启用") }
                            else if snapshot.pose.active { Some("机器人处于姿态控制模式") }
                            else if state.policies.load().walk.is_none() { Some("未加载行走策略") }
                            else if let Some(reason) = arrival_skill_error(&state.policies.load(), params) { Some(reason) }
                            else if state.sitting.load(Ordering::Acquire) { Some("机器人处于坐下状态") }
                            else if state.human_action_busy.load(Ordering::Acquire) {
                                Some("机器人正在执行动作或尚未准备好")
                            }
                            else { None };
                        if let Some(reason) = reason { Err(reason.to_owned()) }
                        else {
                            let already_owner = guard.access(session) == human_control::Access::Owner;
                            if !guard.acquire_if_free(session, proto::HumanControlSource::Web).accepted {
                                Err("网页无法取得控制权".to_owned())
                            } else {
                                if !already_owner { clear_human_intents(&intents); }
                                intents.stop();
                                if intents.update_coordinated_tracking(session.id(), params.clone()) { Ok(()) }
                                else { Err("联合追踪意图被拒绝".to_owned()) }
                            }
                        }
                    }
                    proto::MotionTrackingAction::Update => {
                        if let Some(reason) = arrival_skill_error(&state.policies.load(), params) {
                            Err(reason.to_owned())
                        }
                        else if !web || guard.access(session) != human_control::Access::Owner {
                            Err("网页已失去控制权".to_owned())
                        } else if !intents.update_coordinated_tracking(session.id(), params.clone()) {
                            Err("联合追踪会话已停止或被接管".to_owned())
                        } else if !guard.refresh(session, proto::HumanControlSource::Web).accepted {
                            Err("网页控制权续期失败".to_owned())
                        } else { Ok(()) }
                    }
                    proto::MotionTrackingAction::Stop => {
                        if web && guard.access(session) == human_control::Access::Owner
                            && intents.update_coordinated_tracking(session.id(), params.clone())
                        {
                            intents.stop();
                            intents.request_head_home();
                            guard.abandon(session, proto::HumanControlSource::Web);
                            Ok(())
                        } else { Err("联合追踪已停止或网页已失去控制权".to_owned()) }
                    }
                }
            };
            match (&params.action, &result) {
                (proto::MotionTrackingAction::Start, Ok(())) => tracing::info!(session = session.id(), "coordinated tracking started"),
                (proto::MotionTrackingAction::Stop, Ok(())) => tracing::info!(session = session.id(), "coordinated tracking stopped"),
                (_, Err(reason)) => tracing::warn!(session = session.id(), action = ?params.action, %reason, "coordinated tracking request refused"),
                _ => {}
            }
            if let Some(id) = request.id {
                let result = match result { Ok(()) => proto::IntentResult::accepted(),
                    Err(reason) => proto::IntentResult::refused(&reason) };
                write_line(&mut write_half, &proto::Response::ok(Some(id), &result)).await?;
            }
            continue;
        }

        if let Ok(proto::Call::RobotMaze(params)) = &call {
            let result = {
                let mut guard = state.human_control.lock();
                let web = session.source() == Some(proto::HumanControlSource::Web);
                match params.action {
                    proto::MotionTrackingAction::Start => {
                        let snapshot = intents.snapshot();
                        let reason = if !web { Some("网页控制连接未注册") }
                            else if guard.status_blocked() || guard.maintenance_transition_blocked() { Some("维护页面禁止运动") }
                            else if !params.valid() { Some("迷宫参数无效") }
                            else if !state.obstacle.sensor_enabled() { Some("ToF 测距已关闭，无法开始迷宫模式") }
                            else if intents.motion_tracking_active() || intents.head_tracking_active()
                                || intents.coordinated_tracking_active() { Some("视觉追踪正在运行") }
                            else if !snapshot.enabled { Some("机器人策略未启用") }
                            else if snapshot.pose.active { Some("机器人处于姿态控制模式") }
                            else if state.policies.load().walk.is_none() { Some("未加载行走策略") }
                            else if state.sitting.load(Ordering::Acquire) { Some("机器人处于坐下状态") }
                            else if state.human_action_busy.load(Ordering::Acquire) { Some("机器人正在执行动作或尚未准备好") }
                            else { None };
                        if let Some(reason) = reason { Err(reason.to_owned()) }
                        else if !guard.acquire_if_free(session, proto::HumanControlSource::Web).accepted {
                            Err("网页无法取得控制权".to_owned())
                        } else {
                            intents.stop();
                            if intents.update_maze(session.id(), *params) { Ok(()) }
                            else { Err("迷宫控制意图被拒绝".to_owned()) }
                        }
                    }
                    proto::MotionTrackingAction::Update => {
                        if web && guard.access(session) == human_control::Access::Owner
                            && intents.update_maze(session.id(), *params)
                            && guard.refresh(session, proto::HumanControlSource::Web).accepted
                        { Ok(()) } else { Err("迷宫模式已失效或网页已失去控制权".to_owned()) }
                    }
                    proto::MotionTrackingAction::Stop => {
                        if web && guard.access(session) == human_control::Access::Owner
                            && intents.update_maze(session.id(), *params)
                        {
                            intents.stop();
                            intents.request_head_home();
                            guard.abandon(session, proto::HumanControlSource::Web);
                            Ok(())
                        } else { Err("迷宫模式已停止或网页已失去控制权".to_owned()) }
                    }
                }
            };
            match (&params.action, &result) {
                (proto::MotionTrackingAction::Start, Ok(())) => tracing::info!(session = session.id(), "maze started"),
                (proto::MotionTrackingAction::Stop, Ok(())) => tracing::info!(session = session.id(), "maze stopped"),
                (_, Err(reason)) => tracing::warn!(session = session.id(), action = ?params.action, %reason, "maze request refused"),
                _ => {}
            }
            if let Some(id) = request.id {
                let result = match result { Ok(()) => proto::IntentResult::accepted(),
                    Err(reason) => proto::IntentResult::refused(reason) };
                write_line(&mut write_half, &proto::Response::ok(Some(id), &result)).await?;
            }
            continue;
        }

        if let Ok(proto::Call::RobotHumanControl(params)) = &call {
            if let Some(id) = request.id.clone() {
                let result = {
                    let mut guard = state.human_control.lock();
                    use proto::HumanControlAction as Action;
                    match params.action {
                        Action::Register => guard.register(session, params.source),
                        Action::BlockManual => {
                            let result = guard.block_manual(session);
                            if result.accepted {
                                let automatic = intents.head_tracking_active() || intents.motion_tracking_active()
                                    || intents.coordinated_tracking_active() || intents.maze_active();
                                if automatic {
                                    guard.revoke_gamepad();
                                } else {
                                    guard.emergency_stop();
                                    intents.clear_head();
                                    intents.set_pose(intents::PoseIntent::default());
                                    let _ = intents.take_skills();
                                }
                                // An ordinary zero command preserves the walking policy's
                                // normal deceleration and stand handoff.
                                intents.set_twist([0.0; 3]);
                                if intents.precision_active() {
                                    intents.request_precision(proto::PrecisionMoveParams::Cancel);
                                }
                            }
                            result
                        }
                        Action::UnblockManual => guard.unblock_manual(session),
                        Action::BlockStatus => {
                            let result = guard.block_status(session);
                            if result.accepted {
                                guard.emergency_stop();
                                intents.cancel_head_tracking();
                                intents.cancel_motion_tracking();
                                intents.cancel_coordinated_tracking();
                                intents.cancel_maze();
                                intents.stop_for_maintenance();
                            }
                            result
                        },
                        Action::UnblockStatus => guard.unblock_status(session),
                        Action::BlockMaintenanceTransition => {
                            let result = guard.block_maintenance_transition(session);
                            if result.accepted {
                                guard.emergency_stop();
                                intents.cancel_head_tracking();
                                intents.cancel_motion_tracking();
                                intents.cancel_coordinated_tracking();
                                intents.cancel_maze();
                                intents.stop_for_maintenance();
                            }
                            result
                        },
                        Action::UnblockMaintenanceTransition => guard.unblock_maintenance_transition(session),
                        Action::Acquire | Action::AcquireIfFree
                            if guard.status_blocked() || guard.maintenance_transition_blocked() => {
                            proto::HumanControlResult {
                                accepted: false, owner: None, reason: Some("设备状态页面禁止运动".into()),
                            }
                        }
                        Action::Acquire | Action::AcquireIfFree if (intents.head_tracking_active() || intents.motion_tracking_active()
                            || intents.coordinated_tracking_active() || intents.maze_active())
                            && guard.access(session) != human_control::Access::Owner =>
                        {
                            proto::HumanControlResult {
                                accepted: false, owner: Some(proto::HumanControlSource::Web), reason: Some("tracking_active".into()),
                            }
                        }
                        Action::Acquire | Action::AcquireIfFree => {
                            let snapshot = intents.snapshot();
                            let other_motion = !guard.has_owner()
                                && (state.human_action_busy.load(Ordering::Acquire)
                                    || state.human_motion_busy.load(Ordering::Acquire)
                                    || (snapshot.twist_age < Duration::from_millis(500)
                                        && snapshot.command.twist.iter().any(|value| *value != 0.0)));
                            if other_motion {
                                proto::HumanControlResult {
                                    accepted: false, owner: None, reason: Some("robot_busy".into()),
                                }
                            } else {
                                let already_owner = guard.access(session) == human_control::Access::Owner;
                                let replacing_owner = guard.has_owner() && !already_owner;
                                let result = if params.action == Action::AcquireIfFree {
                                    guard.acquire_if_free(session, params.source)
                                } else {
                                    guard.acquire(session, params.source)
                                };
                                if result.accepted && replacing_owner {
                                    clear_human_intents(&intents);
                                    let _ = intents.take_mode_switch();
                                    if intents.take_theremin_request() == Some(false) {
                                        intents.request_theremin(false);
                                    }
                                    if matches!(intents.take_chorale_request(), Some((false, _))) {
                                        intents.request_chorale(false, None);
                                    }
                                }
                                if result.accepted && !already_owner
                                    && params.source == proto::HumanControlSource::Gamepad
                                {
                                    intents.rearm_gamepad_twist();
                                }
                                result
                            }
                        }
                        Action::Refresh => guard.refresh(session, params.source),
                        Action::Release if intents.head_tracking_active() || intents.motion_tracking_active()
                            || intents.coordinated_tracking_active() || intents.maze_active() => {
                            proto::HumanControlResult { accepted: false, owner: Some(proto::HumanControlSource::Web), reason: Some("tracking_active".into()) }
                        }
                        Action::Release => {
                            let busy = state.human_action_busy.load(Ordering::Acquire)
                                || state.human_motion_busy.load(Ordering::Acquire)
                                || intents.snapshot().command.twist.iter().any(|value| *value != 0.0);
                            let result = guard.release(session, params.source, busy);
                            if result.accepted {
                                clear_human_motion(&intents, false);
                                intents.hold_manual_head_on_release();
                            }
                            result
                        }
                        Action::Abandon => {
                            let owned = guard.access(session) == human_control::Access::Owner;
                            let result = guard.abandon(session, params.source);
                            if result.accepted {
                                if owned {
                                    intents.cancel_motion_tracking();
                                    intents.cancel_maze();
                                    clear_human_intents(&intents);
                                }
                                intents.abandon_head_return();
                            }
                            result
                        }
                        Action::PersonFollowHome | Action::CenterHead => {
                            let leaving = params.action == Action::PersonFollowHome;
                            if !state.startup_voltage_ready.load(Ordering::Acquire) || state.low_voltage_lock.requires_rearm() {
                                proto::HumanControlResult { accepted: false, owner: None, reason: Some("低电压保护中，请充电后手动启动".into()) }
                            } else if guard.status_blocked() || guard.maintenance_transition_blocked() {
                                proto::HumanControlResult { accepted: false, owner: None, reason: Some("设备状态页面禁止运动".into()) }
                            } else if session.source() != Some(params.source)
                                || (leaving && params.source != proto::HumanControlSource::Web)
                                || (guard.access(session) != human_control::Access::Owner
                                    && (!leaving || guard.has_owner()))
                            {
                                proto::HumanControlResult { accepted: false, owner: None, reason: Some("not_owner".into()) }
                            } else if !intents.enabled()
                                || intents.rest_phase() == proto::RestPhase::Resting
                                || (leaving && (state.fallen.load(Ordering::Acquire)
                                    || (state.sitting.load(Ordering::Acquire)
                                        && !intents.coordinated_tracking_active())))
                                || (!leaving && (intents.head_tracking_active() || intents.motion_tracking_active()
                                    || intents.coordinated_tracking_active() || intents.maze_active()))
                            {
                                proto::HumanControlResult { accepted: false, owner: Some(params.source),
                                    reason: Some("机器人尚未准备好，无法让头部回位".into()) }
                            } else {
                                if !guard.has_owner() {
                                    guard.acquire(session, proto::HumanControlSource::Web);
                                }
                                let result = guard.refresh(session, params.source);
                                if result.accepted && leaving {
                                    intents.cancel_head_tracking();
                                    intents.cancel_coordinated_tracking();
                                    intents.cancel_motion_tracking();
                                    intents.cancel_maze();
                                    intents.stop();
                                }
                                if result.accepted {
                                    intents.request_head_home();
                                }
                                result
                            }
                        }
                        Action::Status => guard.status(session, params.source),
                    }
                };
                write_line(&mut write_half, &proto::Response::ok(Some(id), &result)).await?;
            }
            continue;
        }

        if let Ok(call) = &call
            && is_motion_intent(call)
            && !matches!(call, proto::Call::RobotSetObstacle(_))
        {
            let mut skill_reply = None;
            let response = {
                let mut guard = state.human_control.lock();
                let access = guard.access(session);
                let allowed = access == human_control::Access::Owner
                    || (access == human_control::Access::Unregistered && !guard.has_owner());
                let manual_disabled = guard.manual_blocked()
                    && (is_gamepad || session.source() == Some(proto::HumanControlSource::Gamepad)
                        || is_manual_movement(call));
                if state.exit_sit.load(Ordering::Acquire) != 0 {
                    request.id.clone().map(|id| proto::Response::ok(Some(id), &proto::IntentResult::refused("robot is exiting")))
                } else if matches!(call, proto::Call::RobotStop) {
                    guard.emergency_stop();
                    intents.cancel_head_tracking();
                    intents.cancel_motion_tracking();
                    intents.cancel_coordinated_tracking();
                    intents.cancel_maze();
                    clear_human_intents(&intents);
                    request.id.clone().map(|id| proto::Response::ok(Some(id), &proto::IntentResult::accepted()))
                } else if matches!(call, proto::Call::RobotShutdown) {
                    // A confirmed power-off is global, including while tracking owns motion.
                    guard.emergency_stop();
                    intents.cancel_head_tracking();
                    intents.cancel_motion_tracking();
                    intents.cancel_coordinated_tracking();
                    intents.cancel_maze();
                    clear_human_intents(&intents);
                    intents.request_confirmed_shutdown();
                    request.id.clone().map(|id| proto::Response::ok(Some(id), &proto::IntentResult::accepted()))
                } else if guard.status_blocked() || guard.maintenance_transition_blocked() {
                    request.id.clone().map(|id| proto::Response::ok(Some(id), &proto::IntentResult::refused("设备状态页面禁止运动")))
                } else if intents.motion_tracking_active() || intents.coordinated_tracking_active() || intents.maze_active() {
                    request.id.clone().map(|id| proto::Response::ok(Some(id), &proto::IntentResult::refused("autonomous control owns the robot")))
                } else if manual_disabled {
                    request.id.clone().map(|id| proto::Response::ok(Some(id), &proto::IntentResult::refused("manual movement is disabled on the active page")))
                } else if !allowed {
                    request.id.clone().map(|id| proto::Response::ok(Some(id), &proto::IntentResult::refused("another control source owns the robot")))
                } else if let proto::Call::RobotAccessoryTrigger(params) = call {
                    let result = if access == human_control::Access::Owner
                        && session.source() == Some(proto::HumanControlSource::Gamepad)
                        && params.pressure <= 100
                    {
                        intents.set_accessory_trigger(params.pressure);
                        proto::IntentResult::accepted()
                    } else {
                        proto::IntentResult::refused("accessory trigger requires gamepad control and a pressure of 0–100")
                    };
                    request.id.clone().map(|id| proto::Response::ok(Some(id), &result))
                } else {
                    if access == human_control::Access::Owner
                        && !matches!(call, proto::Call::RobotMove(_) | proto::Call::RobotHead(_)
                            | proto::Call::RobotPrecisionMove(proto::PrecisionMoveParams::Status)
                            | proto::Call::RobotPose(_) | proto::Call::RobotMouth(_)
                            | proto::Call::RobotSound(_) | proto::Call::RobotSoundClip(_)) {
                        guard.mark_action();
                    }
                    let gamepad = is_gamepad || session.source() == Some(proto::HumanControlSource::Gamepad);
                    if let proto::Call::RobotMove(p) = call && gamepad {
                        intents.set_gamepad_twist([p.vx, p.vy, p.vyaw]);
                        request.id.clone().map(|id| proto::Response::ok(Some(id), &proto::IntentResult::accepted()))
                    } else if let Some(id) = request.id.clone() {
                        if let proto::Call::RobotDo(params) = call {
                            match queue_manual_skill(&state, &intents, &params.skill) {
                                Ok(reply) => {
                                    skill_reply = Some((id, reply));
                                    None
                                }
                                Err(reason) => Some(proto::Response::ok(Some(id), &proto::IntentResult::refused(reason))),
                            }
                        } else {
                            Some(dispatch(&state, &intents, id, call))
                        }
                    } else {
                        apply_intent(&state, &intents, call);
                        None
                    }
                }
            };
            let response = if let Some((id, reply)) = skill_reply {
                let result = match tokio::time::timeout(SKILL_ADMISSION_TIMEOUT, reply).await {
                    Ok(Ok(result)) => result,
                    Ok(Err(_)) => proto::IntentResult::refused("the skill request was stopped"),
                    Err(_) => proto::IntentResult::refused("the skill request timed out before it could start"),
                };
                Some(proto::Response::ok(Some(id), &result))
            } else {
                response
            };
            if let Some(response) = response { write_line(&mut write_half, &response).await?; }
            continue;
        }

        // Notifications get no reply, per the spec. Continuous intents arrive this way —
        // at 50 Hz a response per message would be pure overhead, and there is nothing
        // useful to say about a velocity that is superseded 20 ms later.
        let Some(id) = request.id.clone() else {
            if let Ok(call) = call {
                if let (true, proto::Call::RobotMove(p)) = (is_gamepad, &call) {
                    intents.set_gamepad_twist([p.vx, p.vy, p.vyaw]);
                } else {
                    apply_intent(&state, &intents, &call);
                }
            }
            continue;
        };

        if let Ok(proto::Call::ChoraleSubscribe) = &call {
            // `btd` asking what to put on the air. One connection carries both directions: this
            // stream down, and `chorale.heard` notifications up.
            beacons = Some(state.chorale_tx.subscribe());
        }

        if let Ok(proto::Call::RobotSubscribe(params)) = &call {
            decimate = params
                .hz
                .filter(|hz| *hz > 0)
                .map(|hz| Duration::from_secs_f64(1.0 / hz as f64))
                .unwrap_or(Duration::ZERO);
            _subscription = Some(state.subscriptions.register(
                decimate.max(Duration::from_micros(state.period_us))));
            _device_subscription = params.devices.then(|| state.device_subscriptions.register(
                decimate.max(Duration::from_micros(state.period_us))));
            // Subscribing again replaces the rate rather than opening a second stream.
            states = Some(state.state_tx.subscribe());
            if params.devices {
                state.device_configuration_generation.fetch_add(1, Ordering::Release);
            }
            devices = params.devices.then(|| state.devices_tx.subscribe());
            last_sent = None;
            next_device_due = None;
        }

        let response = match call {
            Ok(proto::Call::RobotMove(p)) if is_gamepad => {
                intents.set_gamepad_twist([p.vx, p.vy, p.vyaw]);
                proto::Response::ok(Some(id), &proto::IntentResult::accepted())
            }
            Ok(call) => dispatch(&state, &intents, id, &call),
            Err(e) => proto::Response::err(Some(id), e),
        };
        write_line(&mut write_half, &response).await?;
    }
}

fn maintenance_voltage_admitted(state: &RobotState) -> bool {
    if state.startup_read_only.load(Ordering::Acquire) && state.low_voltage_lock.awaiting_start() {
        // The voltage preflight verifies fresh voltage and torque before yielding
        // the port. The recovery marker is consumed only by a separate Start.
        return true;
    }
    state.startup_voltage_ready.load(Ordering::Acquire) && !state.low_voltage_lock.requires_rearm()
}

async fn maintenance_request(
    state: &RobotState,
    intents: &Intents,
    session: &human_control::Session,
    params: proto::MaintenanceParams,
) -> proto::MaintenanceResult {
    use proto::MaintenanceAction;
    let action = params.action;
    let started = {
        let _policy_gate = state.managed_models.gate.lock().expect("policy admission mutex");
        let mut guard = state.human_control.lock();
        match action {
            MaintenanceAction::Status => return guard.maintenance_result(session),
            MaintenanceAction::StatusReadBegin | MaintenanceAction::StatusReadComplete | MaintenanceAction::StatusReadTimeout => {
                let result = match (params.read_id, params.read_address, params.read_length) {
                    (Some(id), Some(address), Some(length)) => {
                        if action == MaintenanceAction::StatusReadBegin {
                            guard.begin_status_read(session, id, address, length)
                        } else if action == MaintenanceAction::StatusReadTimeout {
                            // A timeout or a quiet interval is not a device reply.
                            Err("status_read_unconfirmed".into())
                        } else {
                            guard.complete_status_read(session, id, address, length)
                        }
                    }
                    _ => Err("invalid_status_read".into()),
                };
                return match result {
                    Ok(()) => guard.maintenance_result(session),
                    Err(reason) => proto::MaintenanceResult { accepted: false, phase: "rejected".into(), reason: Some(reason), status_imu_length: None },
                };
            }
            MaintenanceAction::InvalidateRest => {
                if guard.holds_active_maintenance(session) {
                    intents.resume_after_rest();
                    Ok(())
                } else {
                    Err("not_active_maintenance_owner")
                }
            }
            MaintenanceAction::Acquire | MaintenanceAction::AcquirePartial | MaintenanceAction::AcquireStatus if state.managed_models.busy.load(Ordering::Acquire) => Err("model_update_in_progress"),
            MaintenanceAction::Acquire | MaintenanceAction::AcquirePartial if guard.status_blocked() => Err("status_page_active"),
            MaintenanceAction::Acquire if !maintenance_voltage_admitted(state) => Err("low_voltage_rearm_required"),
            MaintenanceAction::AcquireStatus if !state.startup_voltage_ready.load(Ordering::Acquire) || state.low_voltage_lock.requires_rearm() => Err("low_voltage_rearm_required"),
            MaintenanceAction::Acquire | MaintenanceAction::AcquirePartial | MaintenanceAction::AcquireStatus if state.stopping.load(Ordering::Acquire) => Err("robot_exiting"),
            MaintenanceAction::Acquire | MaintenanceAction::AcquirePartial | MaintenanceAction::AcquireStatus if state.exit_sit.load(Ordering::Acquire) != 0 => Err("robot_exiting"),
            MaintenanceAction::AcquirePartial if !guard.maintenance_transition_blocked() => Err("partial_transition_guard_required"),
            MaintenanceAction::AcquirePartial => {
                let faulted = state.status_bus_fault.load(Ordering::Acquire);
                let ready = if faulted {
                    verify_faulted_partial_maintenance(state)
                        .map_err(|_| "partial_servo_release_or_voltage_not_verified")
                } else if !state.startup_voltage_ready.load(Ordering::Acquire) || state.low_voltage_lock.requires_rearm() {
                    Err("low_voltage_rearm_required")
                } else if !state.partial_maintenance_observation.lock().expect("partial observation mutex")
                    .as_ref().is_some_and(|observation| partial_maintenance_ready(observation, proto::clock::monotonic_ns()))
                {
                    Err("partial_servo_release_not_verified")
                } else {
                    Ok(())
                };
                ready.and_then(|()| {
                    if faulted && !guard.emergency_release_active() {
                        // The guarded scan verified release for reachable servos.
                        // Keep the full release gate latched across maintenance.
                        guard.begin_emergency_release()?;
                        guard.mark_emergency_release_unverified();
                    }
                    guard.begin_partial_maintenance(session)
                }).map(|()| {
                    clear_human_intents_with_stop(intents, true);
                    intents.cancel_motion_tracking();
                    intents.cancel_maze();
                    let _ = intents.take_power_request();
                })
            }
            MaintenanceAction::Acquire | MaintenanceAction::AcquireStatus => guard.begin_maintenance(session, action == MaintenanceAction::AcquireStatus).map(|()| {
                clear_human_intents_with_stop(intents, true);
                intents.cancel_motion_tracking();
                intents.cancel_maze();
                let _ = intents.take_power_request();
            }),
            MaintenanceAction::Release => guard.return_maintenance(session),
        }
    };
    if let Err(reason) = started {
        return proto::MaintenanceResult { accepted: false, phase: "rejected".into(), reason: Some(reason.into()), status_imu_length: None };
    }
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let mut result = state.human_control.lock().maintenance_result(session);
        let waiting = match action {
            MaintenanceAction::Acquire | MaintenanceAction::AcquirePartial | MaintenanceAction::AcquireStatus => result.phase == "requested",
            MaintenanceAction::Release => result.phase == "returning",
            MaintenanceAction::Status => false,
            MaintenanceAction::StatusReadBegin | MaintenanceAction::StatusReadComplete
                | MaintenanceAction::StatusReadTimeout | MaintenanceAction::InvalidateRest => false,
        };
        if !waiting {
            if action == MaintenanceAction::AcquireStatus && result.accepted && result.phase == "active" {
                result.status_imu_length = Some(state.status_imu_length);
            }
            return result;
        }
        if Instant::now() >= deadline {
            return proto::MaintenanceResult {
                accepted: false, phase: result.phase,
                reason: Some("motor bus handoff timed out".into()),
                status_imu_length: None,
            };
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

/// Answer one request.
///
/// Synchronous and allocation-light on purpose: these answers must be available even when
/// everything else is broken.
/// Apply a continuous intent. Shared by the notification path and the request path, so a
/// client that sends `robot.move` with an `id` is not silently ignored — the spec permits
/// either, and refusing one because of a framing choice would be a surprise.
fn apply_intent(state: &RobotState, intents: &Intents, call: &proto::Call) -> bool {
    match call {
        proto::Call::RobotPrecisionMove(params) => {
            if matches!(params, proto::PrecisionMoveParams::Status) { return true; }
            let result = queue_precision(state, intents, *params);
            if !result.accepted {
                tracing::warn!(reason = ?result.reason, "precision movement rejected");
            }
            true
        }
        proto::Call::RobotMove(p) => {
            intents.set_twist([p.vx, p.vy, p.vyaw]);
            true
        }
        proto::Call::RobotSetObstacle(p) => {
            if let Err(error) = state.obstacle.set(*p, proto::clock::monotonic_ns()) {
                tracing::warn!(%error, "invalid obstacle assistance notification");
            }
            true
        }
        proto::Call::RobotHead(p) => {
            intents.set_head(*p);
            true
        }
        proto::Call::RobotPose(p) => {
            intents.set_pose(intents::PoseIntent {
                body: [p.z, p.roll, p.pitch],
                active: p.active,
            });
            true
        }
        proto::Call::RobotMouth(p) => {
            intents.set_mouth(*p);
            true
        }
        // A beacon `btd` heard. A notification because it is one-way and frequent, and because
        // there is nothing to say back about a beat.
        proto::Call::ChoraleHeard(p) => {
            intents.heard_chorale(p.clone());
            true
        }
        // A skill as a notification: how a client spells "the button is held" — `padd`
        // resends `roulade` every tick to keep a chain alive, and an answer per resend
        // would be pure overhead. One-shot requests wait for the control loop's decision.
        proto::Call::RobotDo(p) => {
            queue_skill(state, intents, &p.skill);
            true
        }
        // Same story for the wheee hold, which arrives per tick while the trigger is down.
        proto::Call::RobotSound(p) => {
            intents.request_sound(*p);
            true
        }
        proto::Call::RobotSoundClip(p) => {
            if !p.valid() {
                tracing::warn!(name = %p.name, "invalid audio clip name");
            } else if !state.audio_enabled {
                tracing::warn!("audio clip rejected: audio is disabled");
            } else {
                intents.request_sound_clip(p);
            }
            true
        }
        _ => false,
    }
}

/// One phrase for the journal: which of the three things a pending change is.
fn describe_change(change: &intents::PolicyChange) -> String {
    match change {
        intents::PolicyChange::Slot {
            slot,
            path: Some(path),
        } => format!("{slot} <- {}", path.display()),
        intents::PolicyChange::Slot { slot, path: None } => format!("{slot} reset"),
        intents::PolicyChange::ResetAll => "all slots reset".to_owned(),
        intents::PolicyChange::Reload => "reload".to_owned(),
    }
}

/// Is the robot already in the state a `robot.loadPolicy` is asking for?
///
/// `Some(reason)` when the answer is yes and there is nothing to queue. The alternative is a
/// visible ten-second non-event: the robot goes home, reloads seven networks, and comes back to
/// exactly what it was running. `robot.setMode` has treated "already in that mode" as an
/// acceptance since it existed, and for the same reason — the caller asked for a state and that
/// state is what they have.
///
/// **A slot carrying an error is never already-done, whatever its path says.** An override that
/// failed at boot has been dropped in memory, so the slot reads as not-overridden and running the
/// default — which looks exactly like a slot that was never touched. Resetting it is how the
/// error and the config line that caused it get cleared, so that request has to go through.
fn already_loaded(
    state: &RobotState,
    slot: Option<Slot>,
    path: Option<&std::path::Path>,
) -> Option<String> {
    let slots = state.policy_slots.load();
    let settled = |reported: &proto::PolicySlot| -> bool {
        if reported.error.is_some() {
            return false;
        }
        match path {
            Some(path) => reported.path.as_deref() == Some(path.display().to_string().as_str()),
            None => !reported.overridden,
        }
    };

    match slot {
        Some(slot) => {
            let reported = slots.iter().find(|s| s.slot == slot.as_str())?;
            settled(reported).then(|| match path {
                Some(path) => format!("{slot} is already running {}", path.display()),
                None => format!("{slot} is already this robot's own policy"),
            })
        }
        None => slots
            .iter()
            .all(settled)
            .then(|| "every slot is already this robot's own policy".to_owned()),
    }
}

/// Validate a `robot.loadPolicy` and, if it holds up, queue it for the loop.
///
/// **The file is opened here**, on the IPC runtime rather than the control thread. The swap
/// itself happens at the home pose seconds later, so without this the answer to
/// `robotctl policy load walk some-51d-thing.onnx` would be "accepted" followed by a robot that
/// did not change, and the reason would be a journal line. Validating first turns that into
/// `observation width is 51, expected 61` before the caller's command returns.
///
/// It is not a guarantee — the file can still be replaced between here and the home pose — which
/// is why the loop keeps the controller it has when the real load fails.
/// Every skill this robot has, as the table a client edits and sends back.
///
/// Read from the config file rather than from the running snapshot, because the two answer
/// different questions: the snapshot says what is *loaded*, and this says what is *configured* —
/// which is what an editor needs, including an entry whose file has gone missing.
///
/// `built_in` carries the two names `robot.do` answers to that are not table entries at all:
/// `ground_pick` writes a scripted phase and `sit_toggle` is latched. A client offering "what can
/// this robot do" needs both lists, and would otherwise conclude those two do not exist.
fn skills_report(state: &RobotState) -> proto::SkillsResult {
    let configured = params::Params::load(&state.config_path, false)
        .map(|p| p.policy.clone())
        .unwrap_or_default();
    let overridden: std::collections::HashSet<String> =
        configured.skills.iter().map(|s| s.name.clone()).collect();

    let names = state.policies.load();
    proto::SkillsResult {
        skills: configured
            .resolved_skills()
            .iter()
            .map(|skill| proto::SkillParams {
                name: skill.name.clone(),
                path: skill.path.as_ref().map(|p| p.display().to_string()),
                duration: Some(skill.duration),
                command: Some(skill.command),
                unwind: Some(skill.unwind),
                unwind_s: Some(skill.unwind_s),
                chain: Some(skill.chain),
                action_scale: skill.params.action_scale,
                gain_ratio: skill.params.gain_ratio,
                overridden: overridden.contains(&skill.name),
            })
            .collect(),
        built_in: ["ground_pick", "sit_toggle"]
            .into_iter()
            .filter(|name| {
                names.skills.iter().all(|s| s != name)
                    && match *name {
                        "ground_pick" => names.ground_pick.is_some(),
                        _ => names.sitstand.is_some(),
                    }
            })
            .map(str::to_owned)
            .collect(),
    }
}

/// Add a skill, or change one.
///
/// **Written to the config file and reloaded here**, so one call is the whole operation. The
/// alternative was a client writing and then remembering to call `robot.reloadPolicies`, and a
/// client that forgot would leave a robot whose file and behaviour disagree until the next
/// restart — the exact state `robotctl policy load` exists to reconcile.
///
/// The reload is what checks the policy: it opens the file and refuses the shape, reporting the
/// slot as degraded rather than taking the robot down. So this validates what it cheaply can —
/// a name, a length, a path that exists — and leaves the network to the loader.
fn set_skill_request(
    p: &proto::SkillParams,
    state: &RobotState,
    intents: &Intents,
) -> proto::IntentResult {
    if p.name.trim().is_empty() {
        return proto::IntentResult::refused(
            "a skill needs a name; it is what `robot.do` asks for",
        );
    }
    // The two the daemon drives itself. Checked here so the refusal arrives before the file is
    // read and the answer names the caller's own request; `edit::set_skill` refuses the same
    // names for whoever comes the other way, which is the writer `robotctl policy add` uses.
    if params::DAEMON_OWNED_SKILLS.contains(&p.name.as_str()) {
        return proto::IntentResult::refused(format!(
            "{} is driven by the robot itself and cannot be a table entry",
            p.name
        ));
    }

    // An existing entry supplies whatever this call leaves out, so a client may send one field.
    let existing = params::Params::load(&state.config_path, false)
        .ok()
        .and_then(|params| {
            params
                .policy
                .resolved_skills()
                .into_iter()
                .find(|s| s.name == p.name)
        });

    let duration = match p.duration.or(existing.as_ref().map(|s| s.duration)) {
        Some(duration) if duration > 0.0 => duration,
        Some(_) => {
            return proto::IntentResult::refused("a skill's duration must be more than zero");
        }
        None => {
            return proto::IntentResult::refused(format!(
                "{:?} is new here, so it needs a duration — how long it runs, in seconds",
                p.name
            ));
        }
    };

    let path = match &p.path {
        Some(path) => Some(std::path::PathBuf::from(path)),
        None => existing.as_ref().and_then(|s| s.path.clone()),
    };
    // **Through the same gate `robot.loadPolicy` puts a slot's file through**, and for a sharper
    // reason here: a skill is not one of `Slot::ALL`, so the startup check that drops an
    // unloadable *slot* override never looks at it. Without this, a skill pointed at something
    // that is not a policy is accepted, the reload it triggers fails building the controller,
    // the robot keeps what it had, and the only trace is a log line — the caller is told the
    // skill was added and finds out by pressing the button.
    if let Some(path) = &path
        && !params::is_none_sentinel(path)
    {
        if !path.exists() {
            return proto::IntentResult::refused(format!("no such file: {}", path.display()));
        }
        // Only when the error is about the *file*. A board with no ONNX runtime cannot validate
        // anything, and refusing a config edit on those grounds would send somebody to replace a
        // policy that is fine — the same distinction `drop_unloadable_overrides` draws at
        // startup, and what `PolicyError::path` exists for.
        if let Err(e) = duck_control::policy::validate(path)
            && e.path().is_some()
        {
            return proto::IntentResult::refused(e.to_string());
        }
    }

    let skill = params::SkillDef {
        name: p.name.clone(),
        path,
        duration,
        command: p
            .command
            .or(existing.as_ref().map(|s| s.command))
            .unwrap_or_default(),
        unwind: p
            .unwind
            .or(existing.as_ref().map(|s| s.unwind))
            .unwrap_or_default(),
        unwind_s: p
            .unwind_s
            .or(existing.as_ref().map(|s| s.unwind_s))
            .unwrap_or(0.0),
        chain: p
            .chain
            .or(existing.as_ref().map(|s| s.chain))
            .unwrap_or(false),
        params: params::SkillOverrides {
            action_scale: p
                .action_scale
                .or(existing.as_ref().and_then(|s| s.params.action_scale)),
            gain_ratio: p
                .gain_ratio
                .or(existing.as_ref().and_then(|s| s.params.gain_ratio)),
        },
    };

    if let Err(e) = params::edit::set_skill(&state.config_path, &skill) {
        return proto::IntentResult::refused(e);
    }
    intents.request_policy_change(intents::PolicyChange::Reload);
    proto::IntentResult::accepted()
}

/// Take a skill out of the table.
///
/// A skill this robot's release ships comes back when the override is removed, which is why this
/// says which happened rather than only that it worked.
fn remove_skill_request(
    p: &proto::SkillNameParams,
    state: &RobotState,
    intents: &Intents,
) -> proto::IntentResult {
    match params::edit::remove_skill(&state.config_path, &p.name) {
        Ok(false) => {
            proto::IntentResult::already(format!("no configured skill called {:?}", p.name))
        }
        Err(e) => proto::IntentResult::refused(e),
        Ok(true) => {
            intents.request_policy_change(intents::PolicyChange::Reload);
            proto::IntentResult::accepted()
        }
    }
}

/// What each bindable pad button runs, with the two things a client cannot work out alone.
///
/// `overridden` saves it knowing the defaults, and `error` names a binding that will do nothing
/// when pressed — a skill somebody removed, or a name typed wrong before this daemon could check
/// it. Pressing the button is otherwise the only way to discover that, and a button that silently
/// does nothing reads as a broken robot rather than a stale config.
fn pad_bindings_report(config: &std::path::Path, state: &RobotState) -> proto::PadBindingsResult {
    let bindings = params::edit::pad_bindings(config).unwrap_or_default();
    let defaults = params::PadParams::default();
    let known = do_names(&state.policies.load());

    proto::PadBindingsResult {
        bindings: params::PadParams::BUTTONS
            .iter()
            .filter_map(|button| {
                let skill = bindings.skill(button)?;
                Some(proto::PadBinding {
                    button: (*button).to_owned(),
                    skill: skill.to_owned(),
                    overridden: Some(skill) != defaults.skill(button),
                    // An empty binding is a button switched off on purpose, not a fault.
                    error: (!skill.is_empty() && !known.iter().any(|s| s == skill))
                        .then(|| format!("this robot has no skill called {skill:?}")),
                })
            })
            .collect(),
    }
}

/// Put a skill on a button, or clear it.
///
/// **Written to the config file, because there is nowhere else it could go.** `padd` re-reads
/// `[pad]` every second, so a binding held in memory here would be reverted before the caller let
/// go of the phone. That also means this needs no reload call and no restart: the change is live
/// within a second of the write.
///
/// The name is checked against the skills this robot has, which is the reason this call is served
/// here rather than beside the rest of `pad.*` in `configd`. A refusal naming the real skills is
/// worth much more than a button that turns out to do nothing.
fn bind_pad_request(p: &proto::PadBindParams, state: &RobotState) -> proto::IntentResult {
    let defaults = params::PadParams::default();
    if defaults.skill(&p.button).is_none() {
        return proto::IntentResult::refused(format!(
            "no bindable button called {:?}; expected one of {}",
            p.button,
            params::PadParams::names()
        ));
    }

    // Absent is "put it back", exactly as an absent path is for `robot.loadPolicy`.
    let wanted = match p.skill.as_deref() {
        Some(skill) => skill,
        None => defaults.skill(&p.button).unwrap_or_default(),
    };

    if !wanted.is_empty() {
        let known = do_names(&state.policies.load());
        if !known.iter().any(|s| s == wanted) {
            return proto::IntentResult::refused(format!(
                "this robot has no skill called {wanted:?} — it has {}",
                if known.is_empty() {
                    "none".to_owned()
                } else {
                    known.join(", ")
                }
            ));
        }
    }

    let current = params::edit::pad_bindings(&state.config_path).unwrap_or_default();
    if current.skill(&p.button) == Some(wanted) {
        return proto::IntentResult::already(format!("{} already runs {wanted:?}", p.button));
    }

    match params::edit::bind_pad(&state.config_path, &p.button, wanted) {
        Ok(()) => proto::IntentResult::accepted(),
        Err(e) => proto::IntentResult::refused(e),
    }
}

fn load_policy_request(
    p: &proto::LoadPolicyParams,
    state: &RobotState,
    intents: &Intents,
) -> proto::IntentResult {
    // The one combination the wire type allows and nothing means: there is no such thing as
    // loading one file into every slot.
    if p.slot.is_none() && p.path.is_some() {
        return proto::IntentResult::refused(
            "a path needs a slot; omit both to reset every slot to its default",
        );
    }

    let slot = match p.slot.as_deref() {
        None => None,
        Some(name) => match Slot::parse(name) {
            Some(slot) => Some(slot),
            None => {
                return proto::IntentResult::refused(format!(
                    "no policy slot named {name:?}; expected one of {}",
                    Slot::names()
                ));
            }
        },
    };

    // Writing an override a disabled loop will never read is a config edit that silently does
    // nothing, which is the failure `robotctl policy load` exists to remove.
    if !state.policy_enabled {
        return proto::IntentResult::refused(
            "policies are disabled on this robot; set [policy] enabled = true first",
        );
    }

    let path = match p.path.as_deref() {
        None => None,
        // The literal that turns a slot off, exactly as `[policy] <slot> = "none"` does. It names
        // no file, so there is nothing to make absolute and nothing to open — and it has to be
        // reachable from here, because the one published community policy that needs a slot
        // disabled is the reason a person would ever want to.
        Some(path) if params::is_none_sentinel(std::path::Path::new(path)) => {
            match slot {
                None => {
                    return proto::IntentResult::refused(
                        "a slot to disable, or omit both to reset",
                    );
                }
                // The one slot that cannot be empty: a robot with no walking network has nothing
                // to run, and every other slot resolving to it is what makes the rest optional.
                // Refused here rather than left to the config layer, which now falls back
                // instead of honouring it — a request that would be quietly ignored is worse
                // than one that is answered.
                Some(Slot::Walk) => {
                    return proto::IntentResult::refused(
                        "the walking policy cannot be switched off — it is what every other \
                         slot falls back to. `policy reset walk` returns it to this robot's own",
                    );
                }
                Some(_) => {}
            }
            Some(PathBuf::from("none"))
        }
        Some(path) => {
            let path = PathBuf::from(path);
            // `robotd`'s working directory is not the caller's, so a relative path would name a
            // different file at each end — a refusal is better than loading the wrong one.
            if !path.is_absolute() {
                return proto::IntentResult::refused(format!(
                    "{} is not an absolute path",
                    path.display()
                ));
            }
            if let Err(e) = duck_control::policy::validate(&path) {
                return proto::IntentResult::refused(e.to_string());
            }
            Some(path)
        }
    };

    // **Written down before it is asked for.** A slot is a config key and nothing else, so a
    // load the file does not record is one the next restart undoes — which is the ephemeral
    // "try it until reboot" mode §3 of the policy-channel design considered and rejected. It
    // was that mode by accident for as long as `robotctl` wrote the file and the method did
    // not, which made `policy load` and `robot.loadPolicy` the same words with different
    // durability depending on who asked.
    //
    // Before the request rather than after, because the write is the half that can still
    // refuse: a file this daemon cannot write is a refusal the caller can act on, where a
    // robot that swapped and then failed to record it is a surprise saved up for the next boot.
    let slots = match slot {
        Some(slot) => vec![slot],
        None => Slot::ALL.to_vec(),
    };
    let recorded = match params::edit::set_slots(&state.config_path, &slots, path.as_deref()) {
        Ok(recorded) => recorded,
        Err(e) => return proto::IntentResult::refused(e),
    };
    if let Err(error) = managed_models::record_legacy(&state.config_path, &slots, path.as_deref()) {
        return proto::IntentResult::refused(error);
    }

    // Last, because it is the only check that needs the file to have been resolved and found
    // valid: "already running this" is a claim about a policy that exists.
    if let Some(reason) = already_loaded(state, slot, path.as_deref()) {
        tracing::info!(reason, recorded, "policy load: nothing to do");
        // A robot already running it whose file said otherwise is not a robot with nothing to
        // do — the file has just stopped disagreeing, and the caller is the only one who can
        // see that this is why their next reboot will behave differently.
        return proto::IntentResult::already(match recorded {
            true => format!("{reason}; {} now says so too", state.config_path.display()),
            false => reason,
        });
    }

    intents.request_policy_change(match slot {
        Some(slot) => intents::PolicyChange::Slot { slot, path },
        None => intents::PolicyChange::ResetAll,
    });
    proto::IntentResult::accepted()
}

/// Turn a skill name into a request the loop will see, or `false` if this robot has no such
/// skill.
///
/// The two that are not in the configurable list keep their names: `ground_pick` writes a
/// scripted phase rather than a constant, and `sit_toggle` is latched and is driven internally
/// by the shutdown sit as well as by a button. Everything else is an
/// index into what config says this robot can do.
///
/// One load of the published names for the whole decision: they are the *current* mode's, and
/// reading them twice could straddle a mode switch.
/// Every name `robot.do` answers to on this robot, in the order [`queue_skill`] tries them.
///
/// **`policies.skills` is not that list**, and the difference is a real trap: `ground_pick` and
/// `sit_toggle` are driven by their own arm of the cascade rather than being config entries, so
/// they are absent from it while being perfectly good things to ask for. A caller validating a
/// skill name against `skills` alone rejects two of the five the pad ships bound to — which is
/// exactly what `pad.bindings` did until a test caught it.
///
/// Both are conditional on the policy behind them being loaded, because a robot whose `sitstand`
/// slot is switched off genuinely cannot sit.
fn do_names(policies: &PolicyNames) -> Vec<String> {
    let mut names: Vec<String> = Vec::new();
    if policies.ground_pick.is_some() {
        names.push("ground_pick".to_owned());
    }
    if policies.sitstand.is_some() {
        names.push("sit_toggle".to_owned());
    }
    names.extend(policies.skills.iter().cloned());
    names
}

fn has_do_name(policies: &PolicyNames, name: &str) -> bool {
    (name == "ground_pick" && policies.ground_pick.is_some())
        || (name == "sit_toggle" && policies.sitstand.is_some())
        || policies.skills.iter().any(|skill| skill == name)
}

fn arrival_skill_error(policies: &PolicyNames, params: &proto::CoordinatedTrackingParams) -> Option<&'static str> {
    [Some(params), params.pickup_after.as_deref()].into_iter().flatten()
        .filter(|stage| !stage.arrival_skill.is_empty())
        .find_map(|stage| {
            if policies.stand.is_none() { Some("到位动作需要已加载的站立策略") }
            else if !has_do_name(policies, &stage.arrival_skill) { Some("到位动作未加载") }
            else { None }
        })
}

fn cancel_arrival_skill(controller: Option<&mut Controller>, unwritten_sit: &mut bool) {
    if let Some(controller) = controller {
        if *unwritten_sit { controller.cancel_unwritten_sit(); }
        controller.cancel_scripted_actions();
    }
    *unwritten_sit = false;
}

fn start_arrival_skill(
    controller: &mut Controller,
    name: &str,
    now: Instant,
    seat_transition_until: &mut Option<Instant>,
) -> Result<(), &'static str> {
    match name {
        "ground_pick" => controller.start_ground_pick(),
        "sit_toggle" => {
            controller.sit_toggle()?;
            *seat_transition_until = Some(now + Duration::from_secs_f64(controller.shutdown_sit_secs()));
            Ok(())
        }
        _ => {
            let index = controller.skill_names().iter().position(|skill| skill == name)
                .ok_or("skill is no longer loaded")?;
            match controller.start_skill(index)? {
                true => Ok(()),
                false => Err("skill was already running"),
            }
        }
    }
}

const SKILL_ADMISSION_TIMEOUT: Duration = Duration::from_millis(350);

fn queue_manual_skill(
    state: &RobotState,
    intents: &Intents,
    skill: &str,
) -> Result<tokio::sync::oneshot::Receiver<proto::IntentResult>, String> {
    if !intents.enabled() {
        return Err("the policy is not driving — press Start on the pad".into());
    }
    if !state.homed.load(Ordering::Acquire) {
        return Err("the robot is still going to its home pose; try again in a moment".into());
    }
    if state.active_skill.load().as_deref().map(String::as_str) == Some(skill) {
        return Err("the skill is already running".into());
    }
    let known = do_names(&state.policies.load());
    if !known.iter().any(|name| name == skill) {
        return Err(if known.is_empty() {
            "this robot has no skills configured".into()
        } else {
            format!("no skill named {:?}; this robot has {}", skill, known.join(", "))
        });
    }
    let (reply, response) = tokio::sync::oneshot::channel();
    intents.request_manual_skill(intents::ManualSkillRequest {
        name: skill.to_owned(),
        deadline: Instant::now() + SKILL_ADMISSION_TIMEOUT,
        reply,
    })?;
    Ok(response)
}

fn start_requested_skill(
    controller: &mut Controller,
    name: &str,
    now: Instant,
    seat_transition_until: &mut Option<Instant>,
) -> Result<(), &'static str> {
    if controller.skill_recovering() {
        return Err("the robot is recovering after a skill timeout");
    }
    if controller.busy() {
        return Err(if controller.active_skill_name() == Some(name) {
            "the skill is already running"
        } else {
            "a scripted move is already running"
        });
    }
    if name == "sit_toggle" {
        let direction = controller.sit_toggle()?;
        *seat_transition_until = Some(now + Duration::from_secs_f64(controller.shutdown_sit_secs()));
        tracing::info!(direction, "sit toggle");
        return Ok(());
    }
    if name == "ground_pick" {
        return controller.start_ground_pick();
    }
    let index = controller.skill_names().iter().position(|skill| skill == name)
        .ok_or("no such skill on this robot")?;
    if controller.start_skill(index)? {
        Ok(())
    } else {
        Err("the skill is already running")
    }
}

fn queue_skill(state: &RobotState, intents: &Intents, skill: &str) -> bool {
    let policies = state.policies.load();
    match skill {
        "ground_pick" if policies.ground_pick.is_some() => {
            intents.request_ground_pick();
            true
        }
        "sit_toggle" if policies.sitstand.is_some() => {
            intents.request_sit_toggle();
            true
        }
        name => match policies.skills.iter().position(|s| s == name) {
            Some(index) => {
                intents.request_skill(index);
                true
            }
            None => false,
        },
    }
}

fn queue_precision(
    state: &RobotState,
    intents: &Intents,
    params: proto::PrecisionMoveParams,
) -> proto::IntentResult {
    if let proto::PrecisionMoveParams::Start { axis, value } = params {
        if let Err(reason) = precision_motion::PrecisionMotion::new(Instant::now(), axis, value) {
            return proto::IntentResult::refused(reason);
        }
        if !state.precision_ready.load(Ordering::Acquire) {
            return proto::IntentResult::refused("precision policy is not available");
        }
        let interrupt = intents.precision_cancel_requested();
        if intents.precision_active() && !interrupt {
            return proto::IntentResult::refused("a precision movement is already active");
        }
        if !intents.enabled() || !state.homed.load(Ordering::Acquire) {
            return proto::IntentResult::refused("the policy is not ready");
        }
        if state.fallen.load(Ordering::Acquire) || state.sitting.load(Ordering::Acquire) {
            return proto::IntentResult::refused("the robot is fallen or seated");
        }
        if state.human_other_action_busy.load(Ordering::Acquire)
            || (state.human_action_busy.load(Ordering::Acquire) && !interrupt)
        {
            return proto::IntentResult::refused("another action is in progress");
        }
    }
    intents.request_precision(params);
    proto::IntentResult::accepted()
}

fn dispatch(
    state: &RobotState,
    intents: &Intents,
    id: proto::Id,
    call: &proto::Call,
) -> proto::Response {
    // Serialize admission and legacy config writes, without taking a lock on the control loop.
    let _policy_gate = if matches!(call, proto::Call::RobotApplyModel(_) | proto::Call::RobotSetMode(_)
        | proto::Call::RobotLoadPolicy(_) | proto::Call::RobotReloadPolicies
        | proto::Call::RobotSetSkill(_) | proto::Call::RobotRemoveSkill(_)) {
        match state.managed_models.gate.try_lock() {
            Ok(guard) => Some(guard),
            Err(std::sync::TryLockError::WouldBlock) => {
                return if matches!(call, proto::Call::RobotApplyModel(_)) {
                    proto::Response::ok(Some(id), &proto::ApplyModelResult {
                        accepted: false, operation_id: None, reason: Some("另一项策略操作正在进行".into()),
                    })
                } else {
                    proto::Response::ok(Some(id), &proto::IntentResult::refused("另一项策略操作正在进行"))
                };
            }
            Err(std::sync::TryLockError::Poisoned(_)) => panic!("policy admission mutex poisoned"),
        }
    } else { None };
    if state.managed_models.unresolved() && matches!(call,
        proto::Call::RobotSetMode(_) | proto::Call::RobotLoadPolicy(_) |
        proto::Call::RobotReloadPolicies | proto::Call::RobotSetSkill(_) | proto::Call::RobotRemoveSkill(_)) {
        return proto::Response::ok(Some(id), &proto::IntentResult::refused("模型更新正在进行"));
    }
    match call {
        proto::Call::RobotManagedModels => {
            let active = state.managed_models.active.load();
            let mut models = state.managed_models.models.load().as_ref().clone();
            for model in &mut models {
                model.active = active.as_deref().is_some_and(|id| id.as_str() == model.id);
            }
            proto::Response::ok(Some(id), &proto::ManagedModelsResult { enabled: state.policy_enabled, models })
        }
        proto::Call::RobotApplyModel(params) => {
            let result = if !state.startup_voltage_ready.load(Ordering::Acquire) || state.low_voltage_lock.requires_rearm() {
                proto::ApplyModelResult { accepted: false, operation_id: None, reason: Some("低电压保护中，请充电后手动启动".into()) }
            } else if state.human_control.lock().maintenance_phase().is_some() {
                proto::ApplyModelResult { accepted: false, operation_id: None, reason: Some("舵机维护正在进行".into()) }
            } else if state.exit_sit.load(Ordering::Acquire) != 0 || state.shutdown.load(Ordering::Acquire) {
                proto::ApplyModelResult { accepted: false, operation_id: None, reason: Some("机器人正在关闭".into()) }
            } else {
                state.managed_models.submit(params)
            };
            proto::Response::ok(Some(id), &result)
        }
        proto::Call::RobotModelUpdateStatus(params) => match state.managed_models.status(&params.operation_id) {
            Some(result) => proto::Response::ok(Some(id), &result),
            None => proto::Response::err(Some(id), proto::Error::new(proto::code::INVALID_PARAMS, "unknown model operation")),
        },
        proto::Call::RobotPrecisionMove(params) => {
            if matches!(params, proto::PrecisionMoveParams::Status) {
                proto::Response::ok(Some(id), &proto::PrecisionStatus { active: intents.precision_active() })
            } else {
                proto::Response::ok(Some(id), &queue_precision(state, intents, *params))
            }
        }
        proto::Call::RobotLowVoltageStatus => proto::Response::ok(
            Some(id), &state.low_voltage.client_status(),
        ),
        proto::Call::RobotLowVoltageRearm => proto::Response::ok(
            Some(id), &proto::IntentResult::refused("需要已登记的控制连接"),
        ),
        proto::Call::RobotSetLowVoltage(params) => {
            let result = match state.low_voltage.set(*params) {
                Ok(()) => proto::IntentResult::accepted(),
                Err(reason) => proto::IntentResult::refused(reason),
            };
            proto::Response::ok(Some(id), &result)
        }
        proto::Call::RobotObstacleStatus => proto::Response::ok(
            Some(id), &state.obstacle.status(proto::clock::monotonic_ns()),
        ),
        proto::Call::RobotSetObstacle(params) => {
            let result = match state.obstacle.set(*params, proto::clock::monotonic_ns()) {
                Ok(()) => proto::IntentResult::accepted(),
                Err(reason) => proto::IntentResult::refused(reason),
            };
            proto::Response::ok(Some(id), &result)
        }
        proto::Call::RobotMove(_)
        | proto::Call::RobotHead(_)
        | proto::Call::RobotPose(_)
        | proto::Call::RobotMouth(_) => {
            apply_intent(state, intents, call);
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
        }
        proto::Call::RobotAccessoryTrigger(_) => proto::Response::ok(
            Some(id), &proto::IntentResult::refused("accessory trigger requires gamepad control"),
        ),

        // Gaze as a point: the IK runs here, against the same MJCF the policies train on,
        // and the answer is the joints the head was sent to — so a client can hold the gaze
        // by resending them as `robot.head`, or notice `clamped` and move the robot instead.
        // Never refused: an aim is an intent like `robot.head`, and the closest-possible
        // gaze at a clamped target is still the most useful thing the head can do.
        proto::Call::RobotLook(p) => {
            static HEAD_FK: std::sync::LazyLock<kinematics::head::HeadFk> =
                std::sync::LazyLock::new(kinematics::head::HeadFk::alpha);
            let gaze = HEAD_FK.look_at([p.x, p.y, p.z], p.neck_pitch);
            let [neck_pitch, head_pitch, head_yaw, head_roll] = gaze.joints;
            let head = proto::HeadParams { neck_pitch, head_pitch, head_yaw, head_roll, raw_position: None };
            intents.set_head(head);
            proto::Response::ok(
                Some(id),
                &proto::LookResult {
                    head,
                    clamped: gaze.clamped,
                },
            )
        }

        // Picking the theremin up, or putting it down. Refused here for what this side can
        // already know — no voice, no depth frames, the feature switched off — and *not* for
        // circumstance: the loop puts the instrument down by itself when the robot starts
        // walking or goes over, which is a thing that happens after the answer.
        //
        // Note what the answer does not promise: arming. That takes about half a second of
        // depth frames, because the theremin's zero is whatever is in front of the duck at
        // that moment, and one frame is not a background. Whether it took, and the one
        // refusal a player will actually hit — a hand already in front of the beak — arrive
        // in `robot.state`'s theremin block.
        proto::Call::RobotTheremin(p) => {
            let result = if !state.has_voice {
                proto::ThereminResult {
                    accepted: false,
                    reason: Some("this robot has no voice to play a theremin in".to_owned()),
                }
            } else if p.active && !state.theremin_allowed {
                proto::ThereminResult {
                    accepted: false,
                    reason: Some(
                        "the theremin is off by default — turn `[theremin] enabled` on with \
                         `robotctl configure`, which offers the robotd restart it needs"
                            .to_owned(),
                    ),
                }
            } else if p.active && !state.theremin_ready.load(Ordering::Relaxed) {
                proto::ThereminResult {
                    accepted: false,
                    reason: Some(
                        "no depth frames — the ToF sensor is not delivering (is tofd running?)"
                            .to_owned(),
                    ),
                }
            } else {
                intents.request_theremin(p.active);
                proto::ThereminResult {
                    accepted: true,
                    reason: None,
                }
            };
            proto::Response::ok(Some(id), &result)
        }

        // A sound is queued for the loop's next tick. Never refused for *circumstance* —
        // unlike a skill it moves nothing, and a chirp out of a fallen robot is
        // diagnostics, not danger — but refused when this robot has no voice at all, the
        // same way `robot.do` refuses a skill that was never configured.
        //
        // That refusal is the whole value of `robotctl quack`: its job is to tell you which
        // duck you are talking to, and an accepted-then-silent quack inverts the answer —
        // you hear nothing, conclude you are on the wrong duck, and go looking.
        proto::Call::RobotSound(p) => {
            let result = if state.has_voice {
                intents.request_sound(*p);
                proto::IntentResult::accepted()
            } else {
                proto::IntentResult::refused(
                    "this robot has no voice: audio is disabled, or its bank is empty \
                     (run `sounds ensure-bank`)",
                )
            };
            proto::Response::ok(Some(id), &result)
        }
        proto::Call::RobotSoundClip(p) => {
            let result = if !p.valid() {
                proto::IntentResult::refused("invalid audio clip name")
            } else if !state.audio_enabled {
                proto::IntentResult::refused("audio is disabled")
            } else {
                intents.request_sound_clip(p);
                proto::IntentResult::accepted()
            };
            proto::Response::ok(Some(id), &result)
        }

        // Sit, then power the machine off. Never refused for being inconvenient — a robot
        // that cannot sit (no sitstand policy, not driving) cuts torque and powers off
        // directly, which is still what was asked for.
        proto::Call::RobotShutdown => {
            intents.request_shutdown();
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
        }

        // Switch drive mode. Refused here for the two things this side knows: a mode nobody has
        // heard of, and a robot with no policy to switch between — the loop arbitrates the rest.
        //
        // "Already in that mode" is an acceptance rather than a refusal: the caller asked for a
        // state and that state is what they get, and a pad held a beat too long should not report
        // an error.
        proto::Call::RobotSetMode(p) => {
            let target = match p.mode.as_str() {
                "walk" => Some(Mode::Walk),
                "roller" => Some(Mode::Roller),
                _ => None,
            };
            let result = match target {
                None => proto::IntentResult::refused("mode must be \"walk\" or \"roller\""),
                Some(_) if state.policies.load().walk.is_none() => proto::IntentResult::refused(
                    "no policy on this robot, so there is nothing to switch between",
                ),
                Some(mode) => {
                    intents.request_mode_switch(mode_code(mode));
                    proto::IntentResult::accepted()
                }
            };
            proto::Response::ok(Some(id), &result)
        }

        // What each slot is running, from where, and why it is not running what was asked. Served
        // from the snapshot the control loop publishes, so it answers during startup too rather
        // than racing the first load.
        // The geometry the mapping fields of `robot.state` are stated in. Constant for the
        // life of the process, read from the same asset the FK runs on.
        proto::Call::RobotModel => proto::Response::ok(Some(id), &mapping::model()),

        proto::Call::RobotPolicies => proto::Response::ok(
            Some(id),
            &proto::PoliciesResult {
                mode: mode_of(state.mode.load(Ordering::Relaxed))
                    .as_str()
                    .to_owned(),
                enabled: state.policy_enabled,
                slots: state.policy_slots.load().as_ref().clone(),
                skills: state.policies.load().skills.clone(),
                active_skill: state.active_skill.load_full().as_deref().cloned(),
                last_skill_timeout: state.last_skill_timeout.load_full().as_deref().cloned(),
                // The same flag `robot.do` refuses on, so a client can wait for it rather than
                // be told no and guess whether asking again would help.
                homed: state.startup_voltage_ready.load(Ordering::Acquire).then(|| state.homed.load(Ordering::Relaxed)),
                rest_phase: Some(if state.low_voltage_lock.requires_rearm() {
                    proto::RestPhase::Rested
                } else {
                    intents.rest_phase()
                }),
                motion_enabled: Some(intents.enabled()),
                sitting: Some(state.sitting.load(Ordering::Relaxed)),
                change_error: state
                    .policy_change_error
                    .load_full()
                    .map(|e| e.as_ref().clone()),
            },
        ),

        // What each pad button runs. `robotd` answers rather than `configd`, which owns the rest
        // of `pad.*`, because this is the daemon that knows which skills exist — see the routing
        // comment on `Call::PadBindings`.
        proto::Call::PadBindings => {
            proto::Response::ok(Some(id), &pad_bindings_report(&state.config_path, state))
        }

        proto::Call::PadBind(p) => proto::Response::ok(Some(id), &bind_pad_request(p, state)),

        proto::Call::RobotSkills => proto::Response::ok(Some(id), &skills_report(state)),

        proto::Call::RobotSetSkill(p) => {
            proto::Response::ok(Some(id), &set_skill_request(p, state, intents))
        }

        proto::Call::RobotRemoveSkill(p) => {
            proto::Response::ok(Some(id), &remove_skill_request(p, state, intents))
        }

        proto::Call::RobotLoadPolicy(p) => {
            proto::Response::ok(Some(id), &load_policy_request(p, state, intents))
        }

        // Re-read every slot, whatever the paths say.
        //
        // The one case `robot.loadPolicy` cannot serve, and not for want of trying: installing a
        // newer official set swaps `/opt/robot/policies/current` underneath the slots, so every
        // path still resolves to the same string and `already_loaded` correctly concludes there
        // is nothing to do. It is right about the paths and wrong about the bytes. This is how
        // whatever moved the symlink says so.
        //
        // No short-circuit for the same reason: "already loaded" is the answer this exists to
        // disbelieve.
        proto::Call::RobotReloadPolicies => {
            let result = if !state.policy_enabled {
                proto::IntentResult::refused(
                    "policies are disabled on this robot; set [policy] enabled = true first",
                )
            } else {
                intents.request_policy_change(intents::PolicyChange::Reload);
                proto::IntentResult::accepted()
            };
            proto::Response::ok(Some(id), &result)
        }

        proto::Call::RobotMode => proto::Response::ok(
            Some(id),
            &proto::ModeResult {
                mode: mode_of(state.mode.load(Ordering::Relaxed))
                    .as_str()
                    .to_owned(),
            },
        ),

        // Handled by the caller, which owns the connection; answering here keeps the
        // request/response pairing in one place.
        // The acknowledgement carries the policy identity: it is constant for the life of the
        // process, so sending it once here costs nothing, where putting it on every frame
        // would allocate two strings per tick on the control thread.
        // Start or stop looking for other ducks. Refused for what this side can know: no voice to
        // sing with, and — the one that matters — a robot whose config has not opted in. A chorale
        // moves the mouth and the head, so an un-opted-in robot does not merely decline, it never
        // goes on the air at all.
        proto::Call::RobotChorale(p) => {
            let result = if !state.chorale_accepted {
                proto::ChoraleResult {
                    accepted: false,
                    reason: Some(
                        "this robot has not opted in to singing with others (`[chorale] accept` \
                         in robotd.toml)"
                            .to_owned(),
                    ),
                }
            } else if !state.has_voice {
                proto::ChoraleResult {
                    accepted: false,
                    reason: Some("this robot has no voice to sing with".to_owned()),
                }
            } else if let Some(id) = p.piece.filter(|id| !chorale::known_piece(*id)) {
                // Refused at the door with the catalogue, rather than accepted into the coin:
                // a pin that silently did not pin is exactly the confusion it exists to end.
                proto::ChoraleResult {
                    accepted: false,
                    reason: Some(format!(
                        "piece {id} is not on this robot — it has {}",
                        chorale::piece_catalogue()
                    )),
                }
            } else {
                intents.request_chorale(p.active, p.piece);
                proto::ChoraleResult {
                    accepted: true,
                    reason: None,
                }
            };
            proto::Response::ok(Some(id), &result)
        }

        // `btd` subscribing to what to advertise. The stream itself is set up by the caller; this
        // only acknowledges, and is deliberately not refused for a robot that has not opted in —
        // `btd` may subscribe at boot and the answer can change without it reconnecting.
        proto::Call::ChoraleSubscribe => proto::Response::ok(
            Some(id),
            &proto::ChoraleResult {
                accepted: true,
                reason: None,
            },
        ),

        proto::Call::RobotSubscribe(_) => {
            let policies = state.policies.load();
            proto::Response::ok(
                Some(id),
                &proto::SubscribeResult {
                    accepted: true,
                    walk: policies.walk.clone(),
                    stand: policies.stand.clone(),
                    sitstand: policies.sitstand.clone(),
                    ground_pick: policies.ground_pick.clone(),
                    skills: policies.skills.clone(),
                    unavailable: state.policy_error.load_full().map_or_else(
                        || {
                            policies.walk.is_none().then(|| {
                                "no policy configured; holding the startup pose".to_owned()
                            })
                        },
                        |e| Some(format!("policy would not load: {e}")),
                    ),
                },
            )
        }

        proto::Call::RobotStop => {
            intents.stop();
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
        }

        // A refusal here is a normal answer with a reason, not an error: the client asked
        // something reasonable and the daemon declined. Gravity is never one of those
        // reasons — see below.
        proto::Call::RobotEnable(p) => {
            // `toggle` flips the robot's own state — the pad's Start. Evaluated here, not
            // in the client, because a client-side belief drifts (relax, shutdown, either
            // side restarting) and a stale one turns Start into a no-op every other press.
            let on = if p.toggle { !intents.enabled() } else { p.on };
            // Never refused for being down. Start on a robot lying on the floor is exactly
            // how someone asks it to stand back up, and it brings the robot up and hands it
            // to the standing policy like any other enable.
            let result = if on && intents.rest_phase() == proto::RestPhase::Resting {
                proto::IntentResult::refused("rest is still in progress")
            } else {
                intents.set_enabled(on);
                if on { intents.resume_after_rest(); }
                // No init here: stopping holds a seated robot where it is and returns
                // other poses to home. A seated restart settles before rising; from
                // limp, the enable-triggered bring-up still ramps torque on as ever.
                proto::IntentResult {
                    accepted: true,
                    reason: Some(
                        if on {
                            "enabled — driving"
                        } else if state.sitting.load(Ordering::Relaxed) {
                            "disabled — holding the seated pose"
                        } else {
                            "disabled — returning to the home pose"
                        }
                        .to_owned(),
                    ),
                }
            };
            proto::Response::ok(Some(id), &result)
        }

        // Power to the joints, which is the pair `robot.enable` is not: enabling asks the *policy*
        // to drive and brings a limp robot up on the way, while these two are the decision itself.
        //
        // Both only *ask*. The control loop owns the only `RobotIo` handle, so nothing here touches
        // the bus — which is also why `robotd init` needs the daemon stopped and these do not.
        proto::Call::RobotInit => {
            // Never refused for gravity: init works whatever the robot is lying on, which
            // is the prototype's behaviour and what a bench actually needs.
            let result = if intents.rest_phase() == proto::RestPhase::Resting {
                proto::IntentResult::refused("rest is still in progress")
            } else {
                state.homed.store(false, Ordering::Release);
                intents.resume_after_rest();
                intents.request_init();
                proto::IntentResult::accepted()
            };
            proto::Response::ok(Some(id), &result)
        }

        // Never refused. Going limp is always safe *for the robot* — it is the people around it who
        // need to know, which is why `robotctl` asks for `--yes` and BLE cannot reach this at all.
        proto::Call::RobotRelax => {
            intents.request_relax();
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
        }

        // This call is handled asynchronously by the session, which waits for the bus result.
        proto::Call::RobotEmergencyRelax => proto::Response::ok(Some(id),
            &proto::IntentResult::refused("卸力需要等待控制循环确认")),

        // The control loop owns the seated pose, action playback and torque release.
        proto::Call::RobotRest => {
            let result = if intents.request_rest() {
                proto::IntentResult::accepted()
            } else {
                proto::IntentResult::refused("rest is already in progress")
            };
            proto::Response::ok(Some(id), &result)
        }

        // Never refused: a robot with a tripped servo is exactly the one that needs it.
        proto::Call::RobotRebootMotors(p) => {
            intents.request_reboot_motors(p.ids.clone());
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
        }

        proto::Call::RobotHealth => proto::Response::ok(Some(id), &state.health()),
        proto::Call::RobotSafeToRestart => proto::Response::ok(Some(id), &state.safe_to_restart()),
        proto::Call::RobotModelApi => proto::Response::ok(
            Some(id),
            &proto::ModelApiResult {
                model_api: MODEL_API,
            },
        ),
        // No media stack, so no session can be live. `mediad` owns the real answer
        // (architecture.md §5.2); reporting `false` here is honest for now, and the updater
        // treats unknown as false anyway.
        proto::Call::RobotRemoteSessionActive => {
            proto::Response::ok(Some(id), &proto::SessionActiveResult { active: false })
        }
        proto::Call::Hello(_) => proto::Response::ok(
            Some(id),
            &proto::HelloResult {
                api_version: proto::API_VERSION,
                daemon_version: proto::semver::Version::parse(env!("CARGO_PKG_VERSION")).ok(),
                revision: proto::build_info!().revision.map(str::to_owned),
            },
        ),
        // `update.*` is `updaterd`'s namespace. A client reaching here aimed at the wrong
        // socket, so say that rather than report a generic failure.
        other => proto::Response::err(
            Some(id),
            proto::Error::new(
                proto::code::METHOD_NOT_FOUND,
                format!("{} is not served by robotd", other.method()),
            ),
        ),
    }
}

async fn write_line<T: serde::Serialize>(
    out: &mut tokio::net::unix::OwnedWriteHalf,
    message: &T,
) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(message)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    line.push(b'\n');
    out.write_all(&line).await?;
    out.flush().await
}

enum ShutdownSignal {
    Terminate,
    Interrupt,
}

async fn wait_for_exit_sit(state: &RobotState, intents: &Intents, timeout: Duration) -> bool {
    intents.stop();
    intents.set_pose(intents::PoseIntent::default());
    intents.set_mouth(proto::MouthParams::default());
    let _ = intents.take_skills();
    tokio::time::timeout(timeout, async {
        while state.exit_sit.load(Ordering::Acquire) != EXIT_SIT_DONE {
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    }).await.is_ok()
}

impl ShutdownSignal {
    fn releases_torque(&self, keep_torque_on_ctrl_c: bool) -> bool {
        matches!(self, Self::Interrupt) && !keep_torque_on_ctrl_c
    }
}

/// Resolve on SIGTERM (systemd stop) or SIGINT (Ctrl-C).
async fn shutdown() -> ShutdownSignal {
    use tokio::signal::unix::{SignalKind, signal};
    let mut term = match signal(SignalKind::terminate()) {
        Ok(s) => s,
        Err(e) => {
            tracing::warn!(error = %e, "cannot listen for SIGTERM");
            return std::future::pending().await;
        }
    };
    let mut int = match signal(SignalKind::interrupt()) {
        Ok(s) => s,
        Err(_) => return std::future::pending().await,
    };
    tokio::select! {
        _ = term.recv() => ShutdownSignal::Terminate,
        _ = int.recv() => ShutdownSignal::Interrupt,
    }
}

/// What `robot.state` carries for a mapper, and what `robot.model` answers: sensor poses from
/// the head FK, and the static geometry they are stated in. One place, the `kinematics` crate,
/// so a laptop or a server that pairs a picture with a pose asks rather than transcribes.
mod mapping {
    use std::sync::LazyLock;

    use duck_ipc_proto as proto;

    static FK: LazyLock<kinematics::head::HeadFk> = LazyLock::new(kinematics::head::HeadFk::alpha);
    static TOF: LazyLock<kinematics::tof::Reprojector> =
        LazyLock::new(kinematics::tof::Reprojector::alpha);

    /// The head joints, in [`kinematics::head::HeadFk`]'s order, out of a full joint vector.
    const HEAD: [&str; 4] = ["neck_pitch", "head_pitch", "head_yaw", "head_roll"];

    pub fn head_joints_of(positions: &[f64]) -> [f64; 4] {
        let mut out = [0.0; 4];
        for (slot, name) in out.iter_mut().zip(HEAD) {
            if let Some(i) = proto::JOINT_NAMES.iter().position(|n| *n == name) {
                *slot = positions.get(i).copied().unwrap_or(0.0);
            }
        }
        out
    }

    fn pose(p: kinematics::Pose) -> proto::PoseState {
        proto::PoseState {
            pos: p.pos,
            quat: p.quat.wxyz(),
        }
    }

    /// Every body's pose in the trunk frame, in [`Model::body_names`] order, from the full measured
    /// joint vector — the whole skeleton for a viewer. The model's joint order is the MJCF's, not
    /// the wire's, so the angles are gathered by name from [`proto::JOINT_NAMES`]-ordered positions.
    pub fn skeleton_at(positions: &[f64]) -> Vec<proto::PoseState> {
        let model = kinematics::Model::alpha();
        let angles: Vec<f64> = model
            .joint_names()
            .map(|n| {
                proto::JOINT_NAMES
                    .iter()
                    .position(|w| *w == n)
                    .and_then(|i| positions.get(i).copied())
                    .unwrap_or(0.0)
            })
            .collect();
        model.body_poses(&angles).into_iter().map(pose).collect()
    }

    fn skeleton_links() -> Vec<proto::SkeletonLink> {
        let model = kinematics::Model::alpha();
        let parents = model.body_parents();
        model
            .body_names()
            .enumerate()
            .map(|(i, name)| proto::SkeletonLink {
                name: name.to_owned(),
                parent: parents[i],
            })
            .collect()
    }

    pub fn frames_at(head: [f64; 4]) -> proto::FramesState {
        proto::FramesState {
            camera: pose(FK.camera_in_trunk_cv2(head)),
            tof: pose(FK.tof_in_trunk(head)),
            head_imu: FK.head_imu_in_trunk(head).map(pose),
        }
    }

    pub fn model() -> proto::ModelResult {
        let model = kinematics::Model::alpha();
        proto::ModelResult {
            asset: "alpha".to_owned(),
            trunk_height_m: model.trunk_height_m(),
            joint_names: proto::JOINT_NAMES.iter().map(|s| (*s).to_owned()).collect(),
            head_joints: HEAD.iter().map(|s| (*s).to_owned()).collect(),
            tof_beams: TOF.beams().to_vec(),
            tof_fov_deg: kinematics::tof::FOV_DEG,
            frames_at_zero: frames_at([0.0; 4]),
            skeleton: skeleton_links(),
        }
    }


}
