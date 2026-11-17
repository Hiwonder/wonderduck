//! The engine's view of `robotd`.
//!
//! A trait rather than a concrete client for two reasons: the engine is built
//! before `robotd` exists, and the degraded-mode
//! paths — `robotd` dead, crash-looping, or hung — must be testable without
//! staging a real crash.
//!
//! **Every method here is allowed to fail and must be timeout-bounded.** A dead
//! or silent `robotd` is a normal, expected answer. `updaterd` is the recovery path, so it cannot
//! require the thing it is recovering.

use std::time::Duration;

/// Can the robot tolerate a restart of its control loop right now?
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SafeToRestart {
    Yes,
    /// Actively moving or otherwise mid-task. Carries a displayable reason.
    No(String),
    /// Answered, in a shape this `updaterd` cannot read. Does **not** permit a restart.
    ///
    /// The distinction from [`Self::Unreachable`] is the whole point, and getting it wrong is
    /// what this variant exists to fix: a reply that *arrived* is evidence of a `robotd` whose
    /// control loop is running, and reading its "no, I am mid-task" as "sure, go ahead" because
    /// a field was renamed is how an update restarts a walking robot. Silence is safe; an
    /// unreadable answer is not, and the two used to collapse into one variant that permitted
    /// the restart either way — against what that variant's own comment promised.
    ///
    /// Same split, for the same reason, as [`Health::Incompatible`] against
    /// [`Health::Unreachable`]. Carries serde's message, which names the field.
    ///
    /// The escape, when this blocks an update that needs to happen, is to make the robot
    /// genuinely silent — `systemctl stop robotd` — which is honest rather than a bypass: a
    /// stopped robot cannot be moving, and cannot be misread about it either. The refusal says
    /// so.
    Incompatible(String),
    /// `robotd` did not answer.
    ///
    /// Treated as **safe**: if the control loop isn't running, nothing is moving,
    /// and this is exactly the case where an update is the fix. Making this an
    /// error would block recovery on precisely the robots that need it.
    Unreachable,
}

impl SafeToRestart {
    pub fn permits_restart(&self) -> bool {
        !matches!(self, SafeToRestart::No(_) | SafeToRestart::Incompatible(_))
    }
}

/// Result of asking the new release whether it came up correctly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Health {
    Healthy,
    /// Came up and reported a problem, but one belonging to the board rather than to the
    /// release — no servo power, no motor bus. Passes the gate: see
    /// [`crate::proto::HealthResult::degraded`].
    Degraded(String),
    /// Came up and reported a problem.
    Unhealthy(String),
    /// Answered, in a shape this `updaterd` cannot read. Fails the gate — an unreadable
    /// verdict is not a healthy one — but says so in different words, which is the point.
    ///
    /// Distinct from [`Self::Unreachable`] because the two ask for opposite things from
    /// whoever reads the outcome. "Unreachable" sends you to look at a daemon that is
    /// probably dead; this sends you to look at a *contract*, and the robot is likely fine.
    /// Carries serde's own message. It names the missing or unexpected field, which is the
    /// single most useful string available at that moment.
    Incompatible(String),
    /// Did not answer within the timeout — includes crash-looping and hung
    /// (socket open, no reply). Fails the gate: unproven is not healthy.
    Unreachable,
}

/// A [`Health`] verdict in the three fields [`crate::proto::ComponentStatus`] carries it in.
///
/// Five verdicts into two booleans and a string, and the lossy part is deliberate: the booleans
/// answer the only question the gate asks — commit or revert, and whose fault — while the string
/// keeps the distinction a reader needs and the gate does not.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HealthReport {
    pub healthy: bool,
    pub degraded: bool,
    pub reason: Option<String>,
}

impl Health {
    pub fn is_healthy(&self) -> bool {
        matches!(self, Health::Healthy)
    }

    /// How this verdict is reported to whoever asked for status.
    ///
    /// Not `is_healthy`, which is the gate's question and answers `false` for four different
    /// situations: a board with no servo power (which the gate *commits* onto), a broken control
    /// loop, a `robotd` that said nothing, and one that said something unreadable. A status view
    /// built on `is_healthy` alone prints the same word for all four.
    ///
    /// `degraded` is set for exactly the verdict the gate commits on and blames the board for.
    /// The other three carry their reason in words instead: the two booleans genuinely cannot
    /// tell "did not answer" from "answered unreadably", and conflating *those* two is what
    /// `Health::Incompatible` exists to stop, so the string has to say it.
    pub fn report(&self) -> HealthReport {
        let (degraded, reason) = match self {
            Health::Healthy => (false, None),
            Health::Degraded(reason) => (true, Some(reason.clone())),
            Health::Unhealthy(reason) => (false, Some(reason.clone())),
            // The gate's own words for this, and for its reason: "unreachable" about a robotd
            // that is serving its socket sends the reader to the wrong half of the system.
            Health::Incompatible(detail) => (
                false,
                Some(format!(
                    "answered in a shape this updaterd cannot read ({detail}) — the robot may be \
                     fine and the contract is what disagrees"
                )),
            ),
            Health::Unreachable => (false, Some("unreachable: robotd did not answer".to_owned())),
        };
        HealthReport {
            healthy: self.is_healthy(),
            degraded,
            reason,
        }
    }
}

/// Every method is timeout-bounded and allowed to fail. Wrap the underlying IO in
/// [`tokio::time::timeout`] and map elapsed-time to the `Unreachable` variants —
/// a hung peer (socket open, no reply) must look the same as a dead one.
#[async_trait::async_trait]
pub trait RobotClient: Send + Sync {
    /// Refuse to restart motor control mid-motion.
    ///
    async fn safe_to_restart(&self, timeout: Duration) -> SafeToRestart;

    /// The post-apply health gate. Must return within `timeout` even if the peer
    /// holds the socket open and never replies.
    async fn health(&self, timeout: Duration) -> Health;

    /// Model API version the running daemon implements, for model compatibility
    /// checks. `None` when unreachable.
    async fn model_api(&self, timeout: Duration) -> Option<u32>;

    /// Is a telepresence/WebRTC session live? Restarting mid-session is a bad
    /// surprise.
    ///
    /// Defaults to `false` when unknown: this check is a courtesy, and must never
    /// be the reason a recovery update is refused.
    async fn remote_session_active(&self, timeout: Duration) -> bool;

    /// Tell `robotd` to re-read its policy slots from disk.
    ///
    /// Called after a policy set is swapped underneath it. `false` when the robot could not be
    /// reached or refused — the files are in place and it is still running the old ones, which is
    /// worth reporting rather than presenting as a success it has not acted on. A reboot fixes it
    /// either way, so this is never a reason to fail an install.
    async fn reload_policies(&self, timeout: Duration) -> bool;

    /// Every policy file the robot is currently pointed at — slots and skills both.
    ///
    /// `None` when it could not be asked, which is not the same as "none in use" and must never
    /// be read as one: it is what stops the library prune deleting the gait a silent robot is
    /// about to come back up on.
    ///
    /// Defaults to `None` for the same reason — a client that cannot answer this question causes
    /// nothing to be deleted, which is the failure this wants.
    async fn policy_paths(&self, _timeout: Duration) -> Option<Vec<String>> {
        None
    }
}

/// Talks to `robotd` over its unix socket.
pub struct SocketRobotClient {
    path: std::path::PathBuf,
}

impl SocketRobotClient {
    pub fn new(path: std::path::PathBuf) -> Self {
        Self { path }
    }
}

impl SocketRobotClient {
    /// One request/response exchange, entirely inside `timeout`.
    ///
    /// Every failure — connect refused, no reply, malformed reply — collapses to
    /// `None`, which callers map to their `Unreachable` variant. A wedged peer
    /// (socket open, silent) must be indistinguishable from a dead one, or the
    /// engine would hang on exactly the robot it is trying to repair.
    async fn ask(&self, call: &crate::proto::Call, timeout: Duration) -> Option<serde_json::Value> {
        let exchange = async {
            let stream = tokio::net::UnixStream::connect(&self.path).await.ok()?;
            let (read_half, mut write_half) = stream.into_split();

            let request = crate::proto::Request::call(crate::proto::Id::Number(1), call);
            let mut line = serde_json::to_vec(&request).ok()?;
            line.push(b'\n');

            use tokio::io::{AsyncBufReadExt, AsyncWriteExt};
            write_half.write_all(&line).await.ok()?;
            write_half.flush().await.ok()?;

            let mut reply = String::new();
            tokio::io::BufReader::new(read_half)
                .read_line(&mut reply)
                .await
                .ok()?;

            let response: crate::proto::Response = serde_json::from_str(reply.trim()).ok()?;
            response.result
        };

        match tokio::time::timeout(timeout, exchange).await {
            Ok(result) => result,
            Err(_elapsed) => {
                tracing::debug!(
                    method = call.method(),
                    "robotd did not answer within the timeout"
                );
                None
            }
        }
    }
}

#[async_trait::async_trait]
impl RobotClient for SocketRobotClient {
    async fn safe_to_restart(&self, timeout: Duration) -> SafeToRestart {
        let call = crate::proto::Call::RobotSafeToRestart;
        let Some(result) = self.ask(&call, timeout).await else {
            return SafeToRestart::Unreachable;
        };
        // An answer we cannot parse is not guessed at, and specifically is not read as safe:
        // guessing "safe" is what could restart a walking robot.
        match serde_json::from_value::<crate::proto::SafeToRestartResult>(result) {
            Ok(answer) if answer.safe => SafeToRestart::Yes,
            Ok(answer) => SafeToRestart::No(
                answer
                    .reason
                    .unwrap_or_else(|| "robot reports it is not safe to restart".into()),
            ),
            Err(e) => {
                tracing::warn!(error = %e, "robotd answered safeToRestart in an unexpected shape");
                SafeToRestart::Incompatible(e.to_string())
            }
        }
    }

    async fn health(&self, timeout: Duration) -> Health {
        let Some(result) = self.ask(&crate::proto::Call::RobotHealth, timeout).await else {
            return Health::Unreachable;
        };
        match serde_json::from_value::<crate::proto::HealthResult>(result) {
            Ok(answer) if answer.healthy => Health::Healthy,
            Ok(answer) if answer.degraded => Health::Degraded(
                answer
                    .reason
                    .unwrap_or_else(|| "robot reports degraded".into()),
            ),
            Ok(answer) => Health::Unhealthy(
                answer
                    .reason
                    .unwrap_or_else(|| "robot reports unhealthy".into()),
            ),
            Err(e) => {
                tracing::warn!(error = %e, "robotd answered health in an unexpected shape");
                Health::Incompatible(e.to_string())
            }
        }
    }

    async fn model_api(&self, timeout: Duration) -> Option<u32> {
        let result = self
            .ask(&crate::proto::Call::RobotModelApi, timeout)
            .await?;
        serde_json::from_value::<crate::proto::ModelApiResult>(result)
            .ok()
            .map(|answer| answer.model_api)
    }

    async fn reload_policies(&self, timeout: Duration) -> bool {
        self.ask(&crate::proto::Call::RobotReloadPolicies, timeout)
            .await
            .and_then(|v| serde_json::from_value::<crate::proto::IntentResult>(v).ok())
            .is_some_and(|result| result.accepted)
    }

    async fn policy_paths(&self, timeout: Duration) -> Option<Vec<String>> {
        let policies: crate::proto::PoliciesResult = serde_json::from_value(
            self.ask(&crate::proto::Call::RobotPolicies, timeout)
                .await?,
        )
        .ok()?;
        // The skills too: a fetched policy can fill a slot or answer to a name, and a prune that
        // only looked at slots would delete the bow somebody put on a button.
        let skills: Option<crate::proto::SkillsResult> =
            match self.ask(&crate::proto::Call::RobotSkills, timeout).await {
                Some(value) => Some(serde_json::from_value(value).ok()?),
                None => None,
            };
        Some(
            policies
                .slots
                .into_iter()
                .filter_map(|slot| slot.path)
                .chain(
                    skills
                        .into_iter()
                        .flat_map(|s| s.skills)
                        .filter_map(|skill| skill.path),
                )
                .collect(),
        )
    }

    async fn remote_session_active(&self, timeout: Duration) -> bool {
        // Defaults to false when unknown: this check is a courtesy and must never be
        // the reason a recovery update is refused.
        self.ask(&crate::proto::Call::RobotRemoteSessionActive, timeout)
            .await
            .and_then(|r| serde_json::from_value::<crate::proto::SessionActiveResult>(r).ok())
            .is_some_and(|answer| answer.active)
    }
}

/// A `robotd` that isn't there.
///
/// Not only a test double: it's the correct client for a component whose
/// `health` probe is `None`, and it documents the intended degraded behaviour.
pub struct AbsentRobot;

#[async_trait::async_trait]
impl RobotClient for AbsentRobot {
    async fn safe_to_restart(&self, _timeout: Duration) -> SafeToRestart {
        SafeToRestart::Unreachable
    }

    async fn health(&self, _timeout: Duration) -> Health {
        Health::Unreachable
    }

    async fn model_api(&self, _timeout: Duration) -> Option<u32> {
        None
    }

    async fn reload_policies(&self, _timeout: Duration) -> bool {
        false
    }

    async fn policy_paths(&self, _timeout: Duration) -> Option<Vec<String>> {
        None
    }

    async fn remote_session_active(&self, _timeout: Duration) -> bool {
        false
    }
}
