//! The safety authority.
//!
//! **It owns the only write handle to the robot.** Nothing above it — not the policy, not
//! the arbiter, not a client — holds a [`RobotIo`], so nothing above it *can* command a
//! motor. The invariant is enforced by the borrow checker rather than by a rule someone has
//! to remember when adding the eighth skill the night before a demo.
//!
//! That is the same argument the updater makes about its recovery path: code that only runs
//! once something has already gone wrong is the code most likely to be quietly broken, so
//! make the broken state unrepresentable instead.
//!
//! Two rules, unconditional:
//!
//!  - **Non-finite rejection.** A `NaN` target is not clamped, it is refused outright.
//!  - **Range clamp.** Targets are held inside the actuator's travel.
//!
//! Plus a deadman on the command itself: if intents stop arriving, the velocity goes to
//! zero. **Stop is not limp** — losing comms makes the robot *stand still*, because standing
//! is the safe state for a biped; losing balance is a different event, and it is not this
//! layer's to answer.
//!
//! **The fall verdict is a report, not a rule.** [`Safety::fallen`] is tracked every tick
//! and published, and it preempts nothing: a fallen robot keeps being driven and the
//! humans stay in charge. That is the prototype's behaviour, and it is the only one — a
//! robot that yields the moment gravity misreads a lean is a robot that keeps sitting down
//! while someone handles it.
//!
//! What to *do* about a fall is a control decision, and it lives above this layer:
//! `robotd`'s limp-fall mode drops the gain and rides the robot down, on
//! [`crate::fall::FallPredictor`] rather than on this verdict. It commands through `apply`
//! like anything else, with no exemption and no back door — which is the point of the
//! write handle living here.

use std::time::{Duration, Instant};

use crate::execution_timing::StageTimer;
use crate::io::{IoError, JointTargets, RobotIo, Sensors};
use crate::model::{JOINT_IDS, JOINT_LIMITS, MOUTH_CLOSED, MOUTH_INDEX, NUM_JOINTS};
use crate::obs::Command;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SafetyConfig {
    /// Projected-gravity z above which the robot counts as falling. Upright reads about
    /// -1.0 and a robot on its side reads near 0.
    pub fall_gravity_z: f64,
    /// How long that must hold before it counts. Debounced so a hard footfall is not a
    /// fall.
    pub fall_debounce: Duration,
    /// Intent age past which the velocity command is zeroed.
    pub deadman: Duration,
    /// Gain while running.
    pub gain_running: u16,
    /// Gain to yield at rather than fight the floor. Nothing here applies it — `robotd`
    /// commands it during limp-fall — but it lives with the other safety numbers because
    /// it is one: the gain at which the robot stops pushing back.
    pub gain_limp: u16,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            // The prototype's numbers.
            fall_gravity_z: -0.5,
            fall_debounce: Duration::from_millis(200),
            deadman: Duration::from_millis(500),
            gain_running: 200,
            gain_limp: 50,
        }
    }
}

/// Why a commanded value was not applied as asked. Surfaced so a client can be told,
/// rather than watching the robot ignore it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Limit {
    /// Intents went stale; the velocity was zeroed.
    Deadman,
    /// A target was outside its joint's mechanical travel.
    Range,
    /// A target was `NaN` or infinite.
    NotFinite,
}

/// What safety did with a tick's worth of targets.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Applied {
    pub limits: Vec<Limit>,
}

impl Applied {
    pub fn limited_by(&self, limit: Limit) -> bool {
        self.limits.contains(&limit)
    }
}

/// How often a limit that keeps firing is said again.
///
/// The first tick a limit trips always reports — a clamp that happens once is news, and
/// before this none of it was visible at all. After that, one line per fifty: at 50 Hz that
/// is one a second, which shows a fault is still going on without writing a line per tick
/// into the journal. The same shape as the bus-read warning in `robotd`, which reports the
/// first failure and then every tenth.
const LIMIT_LOG_EVERY: u64 = 50;

/// How many ticks running each limit has tripped, which is what the journal is keyed off.
///
/// Three counters rather than one because the three limits are independent events: a
/// deadman that fires for a second says nothing about whether a joint is being clamped, and
/// one shared counter would let whichever fires most starve the other two of a line.
///
/// Each is cleared by a tick that reaches that limit's check and does *not* trip it, so the
/// number reads "how long has this been going on" — which is the number worth saying, and
/// the one a reader has to be able to distinguish from "how many times has it ever
/// happened".
///
/// A tick refused as non-finite returns before the clamp is ever reached, so it leaves
/// `range` standing rather than clearing it. That is deliberate, and the alternative is
/// worse: a target alternating between `NaN` and out-of-range would end the range run on
/// every second tick, and both limits would then report on every tick they tripped — the
/// line per tick this counting exists to prevent. So the number is trips in a run, not
/// strictly consecutive ticks.
#[derive(Debug, Default, PartialEq)]
struct LimitRuns {
    deadman: u64,
    range: u64,
    not_finite: u64,
}

impl LimitRuns {
    /// Whether a run of `n` has reached a point worth writing down.
    ///
    /// Kept here rather than at each call site so the three limits cannot drift into
    /// three different ideas of "often enough".
    fn worth_logging(n: u64) -> bool {
        n == 1 || n.is_multiple_of(LIMIT_LOG_EVERY)
    }
}

pub struct Safety<T: RobotIo> {
    io: T,
    config: SafetyConfig,
    /// How long gravity has been past the threshold. Reset by any upright sample.
    falling_for: Duration,
    fallen: bool,
    /// Tracks the last gain written so an unchanged one is not rewritten every tick — that
    /// would be fifteen bus writes per tick for no reason.
    gain: Option<u16>,
    /// Separate units and confirmation from the legacy Dynamixel gain cache.
    position_p_gain: Option<u8>,
    /// Trip counts, for rate-limiting what the layer says about itself.
    runs: LimitRuns,
    /// Whether an intent has ever arrived inside the deadman, which is what makes a stale
    /// one news. Until one has, the robot has no driver to have lost. See
    /// [`Self::deadman_is_news`].
    deadman_armed: bool,
}

impl<T: RobotIo> Safety<T> {
    pub fn new(io: T, config: SafetyConfig) -> Self {
        Self {
            io,
            config,
            falling_for: Duration::ZERO,
            fallen: false,
            gain: None,
            position_p_gain: None,
            runs: LimitRuns::default(),
            deadman_armed: false,
        }
    }

    pub fn read(&mut self) -> Result<Sensors, IoError> {
        self.io.read()
    }

    pub fn maintenance_handoff(&mut self, release: bool) -> Result<(), IoError> {
        self.io.maintenance_handoff(release)?;
        if !release {
            self.gain = None;
            self.position_p_gain = None;
            self.io.restart_imu_stream();
        }
        Ok(())
    }

    /// Reclaim the bus solely to release torque during shutdown.
    pub fn maintenance_reclaim_for_shutdown(&mut self) -> Result<(), IoError> {
        self.io.maintenance_reclaim_for_shutdown()
    }

    pub fn read_with_cycle_deadline(&mut self, deadline: Instant) -> Result<Sensors, IoError> {
        self.io.read_with_cycle_deadline(deadline)
    }

    pub fn sample_device_observation(&mut self, deadline: Instant) -> Option<crate::io::DeviceObservation> {
        self.io.sample_device_observation(deadline)
    }

    pub fn sample_individual_device_observation(&mut self, deadline: Instant) -> Result<Option<crate::io::DeviceObservation>, IoError> {
        self.io.sample_individual_device_observation(deadline)
    }

    pub fn set_individual_device_observation(&mut self, active: bool) {
        self.io.set_individual_device_observation(active);
    }

    pub fn supports_individual_device_observation(&self) -> bool {
        self.io.supports_individual_device_observation()
    }

    pub fn quarantine_device_bus(&mut self) { self.io.quarantine_device_bus(); }

    pub fn torque_release_verified(&self) -> bool { self.io.torque_release_verified() }

    pub fn recover_device_bus(&mut self) -> Result<(), IoError> {
        self.io.recover_device_bus()?;
        self.gain = None;
        self.position_p_gain = None;
        self.io.restart_imu_stream();
        Ok(())
    }

    pub fn begin_device_observation(&mut self, generation: u64) {
        self.io.begin_device_observation(generation);
    }

    pub fn end_device_observation(&mut self) {
        self.io.end_device_observation();
    }

    /// Bus reads that are not part of the tick, passed through rather than exposing the IO.
    ///
    /// Safety owns the only `RobotIo` handle — that is what makes "nothing commands a motor
    /// except through here" a fact the borrow checker enforces rather than a convention. Slow
    /// sensors and IMU diagnostics read rather than write, so they are no threat to that, but
    /// handing out the handle to reach them would be.
    pub fn slow_sensors(&mut self) -> Result<crate::io::SlowSensors, IoError> {
        self.io.slow_sensors()
    }

    pub fn poll_diagnostics(&mut self, budget: std::time::Duration) {
        self.io.poll_diagnostics(budget);
    }

    pub fn record_servo_diagnostics(&mut self, budget: std::time::Duration) -> Option<crate::servo_diagnostics::Capture> {
        self.io.record_servo_diagnostics(budget)
    }

    pub fn imu_stale(&self) -> crate::io::ImuStale {
        self.io.imu_stale()
    }

    pub fn imu_ready(&self) -> bool {
        self.io.imu_ready()
    }

    /// Explicit sampling lifecycle boundary, without hardware writes or target changes.
    pub fn restart_imu_stream(&mut self) {
        self.io.restart_imu_stream();
    }

    pub fn fallen(&self) -> bool {
        self.fallen
    }

    /// Power the joints, so the positions this writes can actually be held.
    ///
    /// Through here because [`Safety`] owns the only [`RobotIo`] handle — the same reason
    /// [`Self::slow_sensors`] is a passthrough. Unlike that one this *does* affect the robot, so it
    /// is worth being explicit about what it is and is not:
    ///
    ///  - It is called when a human enables the policy on a limp robot, once, not per tick.
    ///  - It is **not** called at startup. A `robotd` restarted by an update must leave a standing
    ///    robot standing, and nothing here changes that.
    ///  - It does not bypass anything. Torque decides whether the motors hold what
    ///    [`Self::apply`] writes; every clamp, the fall gate and the limp gain still apply to *what*
    ///    gets written. A fallen robot with torque on is still commanded `hold` at `gain_limp`.
    pub fn set_torque(&mut self, on: bool) -> Result<(), IoError> {
        tracing::warn!(on, "torque");
        self.io.set_torque(on)
    }

    /// Prime the current measured pose while torque is off; refuse non-finite readings.
    pub fn prepare_torque_only(&mut self, positions: &[f64; NUM_JOINTS]) -> Result<(), IoError> {
        if positions.iter().any(|value| !value.is_finite()) {
            return Err(IoError::Bus("measured position is not finite; cannot enable torque at current pose".into()));
        }
        self.io.prepare_torque_only(positions)
    }

    /// Hold an already measured pose without pulling joints to software limits.
    pub fn hold_torque_only(&mut self, positions: [f64; NUM_JOINTS], gain: u16) -> Result<Applied, IoError> {
        if positions.iter().any(|value| !value.is_finite()) {
            return Err(IoError::Bus("current-pose hold is not finite".into()));
        }
        self.set_gain(gain)?;
        self.io.write(&JointTargets::new(positions))?;
        Ok(Applied::default())
    }

    pub fn read_positions_raw(&mut self, ids: &[u8]) -> Result<Vec<u16>, IoError> {
        self.io.read_positions_raw(ids)
    }

    pub fn write_positions_raw(&mut self, ids: &[u8], positions: &[u16]) -> Result<(), IoError> {
        self.io.write_positions_raw(ids, positions)
    }

    /// Reboot these servos, and forget the gain cache: a rebooted servo comes back at its EEPROM
    /// gains, and a cache that still says the running gain is written would leave it there. The
    /// next [`Self::apply`] rewrites the gains on every servo.
    pub fn reboot_motors(&mut self, ids: &[u8]) -> Result<(), IoError> {
        self.position_p_gain = None;
        for &id in ids {
            tracing::warn!(id, "rebooting servo");
            self.io.reboot(id)?;
        }
        self.gain = None;
        Ok(())
    }

    /// The gain last written to the servos, or `None` before the first write. This is what
    /// the robot is running at, which is not always what the caller asked for.
    pub fn gain(&self) -> Option<u16> {
        self.gain
    }

    /// Change native P only at phase boundaries. A partial bus write invalidates the
    /// previous value too, so an interrupted transition cannot skip the restoration.
    pub fn set_position_p_gain(&mut self, kp: u8) -> Result<(), IoError> {
        if self.position_p_gain == Some(kp) {
            return Ok(());
        }
        self.position_p_gain = None;
        self.io.set_position_p_gain(kp)?;
        self.position_p_gain = Some(kp);
        tracing::info!(position_p_gain = kp, "temporary position P gain confirmed");
        Ok(())
    }

    /// Update fall state from a fresh sample. Call every tick, before [`Self::apply`].
    ///
    /// Debounced in both directions: a robot has to be down for `fall_debounce` to count as
    /// fallen, and one upright sample clears the accumulator. Without the debounce, the
    /// impulse from a firm footfall reads as a fall.
    pub fn observe(&mut self, sensors: &Sensors, dt: Duration) {
        // An orientation filter that has not converged does not get a vote.
        //
        // The SFLP filter needs a few seconds of samples before its quaternion means
        // anything, and until then projected gravity is not `[0, 0, -1]` — it is whatever the
        // filter is mid-way through deciding, which reads as "above the fall threshold", which
        // reads as "on its side". Two hundred milliseconds of that and an upright robot on a
        // bench latches `fallen` at startup: `apply` writes `gain_limp`, the policy is refused
        // with "the robot is down; stand it up first", and it clears itself a few seconds
        // later leaving the gain behind at 50 with nothing to explain it.
        //
        // Observed on a board: a joint set to 137 by hand read back 50 after five seconds of
        // robotd, while `robotctl monitor` — sampled later, after convergence — reported `ok`.
        //
        // Holding the previous verdict is the safe default in both directions. At startup that
        // is "not fallen", which is what the robot standing on the bench actually is; and a
        // filter that stops being ready mid-run leaves a fallen robot fallen.
        if !self.io.imu_ready() {
            return;
        }

        let was_fallen = self.fallen;
        let down = sensors.imu.gravity[2] > self.config.fall_gravity_z;
        if down {
            self.falling_for = self.falling_for.saturating_add(dt);
            if self.falling_for >= self.config.fall_debounce {
                self.fallen = true;
            }
        } else {
            self.falling_for = Duration::ZERO;
            self.fallen = false;
        }

        // Not rate-limited, unlike the limits below — though a flip is less rare than the
        // debounce suggests. Latching takes `fall_debounce` of down, but one upright sample
        // clears it, so a robot held near the threshold on a noisy IMU can cycle in roughly
        // that period and report a few times a second. Still not rate-limited, because what
        // is being said here is the flip itself: a limit reports "this is still wrong",
        // where this reports "this just changed", and dropping all but one in fifty of those
        // would drop the one that says when.
        //
        // It is also the only *event* there is. `fallen` otherwise shows up in two places
        // and neither is a record of the fall: a subscribed client's frame — which on a
        // robot is usually nobody, since `robotd` only assembles one when it has a
        // receiver — and the loop summary, which samples the boolean every five minutes.
        // A fall that comes and goes inside one of those windows leaves nothing at all,
        // and one that persists has no time on it. This fires on the flip, so it has both.
        if self.fallen != was_fallen {
            tracing::warn!(
                fallen = self.fallen,
                gravity_z = sensors.imu.gravity[2],
                "the fall verdict changed"
            );
        }
    }

    /// Apply the deadman to a command.
    ///
    /// Zeroes the *twist* only. Head targets are left alone deliberately: a stale head pose
    /// is harmless, while a stale velocity walks the robot into a wall.
    ///
    /// Takes `&mut self` because how long the intents have been stale is this layer's to
    /// remember: a robot standing still because comms dropped is a state, not a moment.
    /// Report the transition once and count the duration without repeating the warning.
    pub fn gate(&mut self, command: Command, intent_age: Duration) -> (Command, Option<Limit>) {
        if intent_age <= self.config.deadman {
            self.runs.deadman = 0;
            self.deadman_armed = true;
            return (command, None);
        }

        self.runs.deadman += 1;
        if self.deadman_is_news() {
            tracing::warn!(
                intent_age_ms = intent_age.as_millis(),
                deadman_ms = self.config.deadman.as_millis(),
                tripped = self.runs.deadman,
                "intents went stale — the velocity command is zeroed"
            );
        }

        let mut stopped = command;
        stopped.twist = [0.0; 3];
        (stopped, Some(Limit::Deadman))
    }

    /// Whether a stale intent this tick is worth saying anything about.
    ///
    /// Never, on a robot that has not had a driver since it started, however long it has been
    /// idle. This runs every tick whether or not anything is driving the robot, and an intent
    /// that has never been written reads as maximally stale from the moment it boots.
    ///
    /// A robot with no driver is not being ignored, and that is not news. Losing one is.
    fn deadman_is_news(&self) -> bool {
        self.deadman_armed && self.runs.deadman <= 1
    }

    /// The only path to the motors.
    ///
    /// `hold` is what to command when the policy must not drive — normally the pose the
    /// robot is already in.
    /// `running_gain` is what the caller wants — the standing policy runs softer than the
    /// walking one, and limp-fall softer still. All of those are control decisions, not
    /// safety ones, so they are passed in rather than second-guessed here.
    pub fn apply(
        &mut self,
        targets: [f64; NUM_JOINTS],
        mut hold: [f64; NUM_JOINTS],
        running_gain: u16,
    ) -> Result<Applied, IoError> {
        let mut timing = StageTimer::<6>::new("control_apply", "gain");
        let mut applied = Applied::default();

        // Note what is *not* here: a fall gate. Being down does not stop the caller
        // driving, because the verdict is a report (see the module docs). What a fall is
        // worth doing about is decided above, and arrives as ordinary targets and an
        // ordinary gain.
        self.set_gain(running_gain)?;
        timing.advance("target_validation");

        // Non-finite is refused, not clamped. Clamping `NaN` silently produces a boundary
        // value, which is a plausible-looking joint angle — far worse than declining to
        // move, because the robot would lurch to a limit rather than hold still.
        if targets.iter().any(|v| !v.is_finite()) {
            // Returned before the clamp below is reached, so `range` is left standing — see
            // [`LimitRuns`] for why that is the right way round.
            self.runs.not_finite += 1;
            timing.advance("invalid_target_log");
            if LimitRuns::worth_logging(self.runs.not_finite) {
                tracing::warn!(
                    tripped = self.runs.not_finite,
                    "targets refused: not finite — holding the pose the robot is already in"
                );
            }
            applied.limits.push(Limit::NotFinite);
            // Close the jaw if an invalid command falls back to the measured hold pose.
            hold[MOUTH_INDEX] = MOUTH_CLOSED;
            timing.advance("bus_write");
            self.io.write(&JointTargets::new(hold))?;
            return Ok(applied);
        }
        self.runs.not_finite = 0;

        // Counted, not merely detected: one joint at its limit and nine are different
        // faults wearing the same `Limit::Range` on the wire, and the lowest index out is
        // what someone goes to look up. `first_clamped` doubles as "any were clamped",
        // which is what keeps the run counter honest on a tick that clamps nothing.
        let mut safe = targets;
        let mut clamped_joints = 0u32;
        let mut first_clamped = None;
        for (index, value) in safe.iter_mut().enumerate() {
            let (lower, upper) = JOINT_LIMITS[index];
            let clamped = value.clamp(lower, upper);
            if clamped != *value {
                clamped_joints += 1;
                first_clamped.get_or_insert(index);
                tracing::debug!(
                    "joint clamp: id={:>3}  send={:>6.1}° -> {:>6.1}°  range=[{:>6.1}, {:>6.1}]°",
                    JOINT_IDS[index],
                    value.to_degrees(),
                    clamped.to_degrees(),
                    lower.to_degrees(),
                    upper.to_degrees(),
                );
                *value = clamped;
            }
        }

        timing.advance("clamp_bookkeeping");
        if let Some(first) = first_clamped {
            self.runs.range += 1;
            applied.limits.push(Limit::Range);
            if LimitRuns::worth_logging(self.runs.range) {
                timing.advance("limit_log");
                tracing::warn!(
                    joints = clamped_joints,
                    first_joint = first,
                    tripped = self.runs.range,
                    "joint targets clamped to the joint travel"
                );
            }
        } else {
            self.runs.range = 0;
        }

        timing.advance("bus_write");
        self.io.write(&JointTargets::new(safe))?;
        Ok(applied)
    }

    fn set_gain(&mut self, kp: u16) -> Result<(), IoError> {
        if self.gain == Some(kp) {
            return Ok(());
        }
        self.io.set_gain(kp)?;
        self.gain = Some(kp);
        Ok(())
    }

    /// Whether the backend measures joint velocity, and whether it measures load.
    ///
    /// Forwarded one bool at a time rather than by handing out the IO, which stays private for
    /// the reason [`Self::io`] gives. The control loop needs the answer because it decides
    /// whether those blocks go on the state stream at all, and a fabricated zero there is
    /// indistinguishable from a robot at rest.
    pub fn measures_velocity(&self) -> bool {
        self.io.measures_velocity()
    }

    pub fn measures_load(&self) -> bool {
        self.io.measures_load()
    }


}
