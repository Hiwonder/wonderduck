//! Gamepad precision movement scheduling and IPC.

use std::collections::VecDeque;
use std::os::unix::net::UnixStream;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;

use crate::{Mode, connection, control, notify, precision_input};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PrecisionMapping {
    Full { roller: bool },
    ForwardTurn { roller: bool },
    Stationary,
}

pub(super) fn precision_axes(
    mode: Mode,
    imu_drives_head: bool,
    roller: bool,
    left_y: f64,
    left_x: f64,
    right_x: f64,
) -> (PrecisionMapping, Option<[f64; 3]>) {
    match mode {
        Mode::Drive | Mode::HeadDrive if mode == Mode::Drive || imu_drives_head => (
            PrecisionMapping::Full { roller },
            Some([left_y, if roller { 0.0 } else { -left_x }, -right_x]),
        ),
        Mode::HeadDrive => (
            PrecisionMapping::ForwardTurn { roller },
            Some([left_y, 0.0, -left_x]),
        ),
        Mode::Head | Mode::BodyPose => (PrecisionMapping::Stationary, None),
        Mode::Drive => unreachable!("drive mode always uses the full mapping"),
    }
}

pub(super) struct PrecisionDrive {
    pub(super) gesture: precision_input::GestureInput,
    pub(super) epoch: Instant,
    pub(super) heartbeat_period: Duration,
    pub(super) mapping: Option<PrecisionMapping>,
    pub(super) next_deadline: Option<Duration>,
    // A successful start stays uncertain until a status query reports completion.
    pub(super) awaiting_completion: bool,
    pub(super) next_status_check: Option<Instant>,
    replay: VecDeque<precision_input::ReplayedVelocity>,
    replay_velocity: [f64; 3],
    last_replay: Option<(Duration, Duration)>,
    discard_short: bool,
}

pub(super) struct PrecisionStep {
    pub(super) started: bool,
    pub(super) movement: proto::MoveParams,
    pub(super) replayed: bool,
    pub(super) request: Option<proto::PrecisionMoveParams>,
}

impl PrecisionDrive {
    pub(super) fn new(heartbeat_period: Duration) -> Self {
        Self {
            gesture: precision_input::GestureInput::new(),
            epoch: Instant::now(),
            heartbeat_period,
            mapping: None,
            next_deadline: None,
            awaiting_completion: false,
            next_status_check: None,
            replay: VecDeque::new(),
            replay_velocity: [0.0; 3],
            last_replay: None,
            discard_short: false,
        }
    }

    pub(super) fn sample(
        &mut self,
        now: Duration,
        axes: [f64; 3],
        movement: proto::MoveParams,
    ) -> PrecisionStep {
        let mut out = self
            .gesture
            .step(now, axes, [movement.vx, movement.vy, movement.vyaw]);
        self.replay.extend(out.replayed.drain(..));
        let mut replayed = false;
        let replay_at = |sample: &precision_input::ReplayedVelocity,
                         last: Option<(Duration, Duration)>| {
            last.map_or(now, |(played, due)| {
                played.saturating_add(sample.due_at.saturating_sub(due))
            })
        };
        if self
            .replay
            .front()
            .is_some_and(|sample| now >= replay_at(sample, self.last_replay))
        {
            let sample = self
                .replay
                .pop_front()
                .expect("the replay queue has a sample");
            self.replay_velocity = sample.velocity;
            self.last_replay = Some((now, sample.due_at));
            replayed = true;
            if sample.velocity == [0.0; 3] && self.replay.is_empty() {
                self.last_replay = None;
            }
        }
        let replay_deadline = self
            .replay
            .front()
            .map(|sample| replay_at(sample, self.last_replay));
        self.next_deadline = match (out.next_deadline, replay_deadline) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        };
        if out.started {
            self.discard_short = !self.replay.is_empty() || self.replay_velocity != [0.0; 3];
        }
        if axes == [0.0; 3] && self.discard_short {
            out.short_target = None;
            self.discard_short = false;
        }
        let [vx, vy, vyaw] = self.replay_velocity;
        let request = out.short_target.map(|target| {
            let axis = match target.axis {
                0 => proto::PrecisionAxis::X,
                1 => proto::PrecisionAxis::Y,
                2 => proto::PrecisionAxis::Yaw,
                _ => unreachable!("gesture axis is always x, y or yaw"),
            };
            proto::PrecisionMoveParams::Start {
                axis,
                value: target.displacement,
            }
        });
        PrecisionStep {
            started: out.started,
            movement: proto::MoveParams { vx, vy, vyaw },
            replayed,
            request,
        }
    }

    pub(super) fn reset(&mut self) {
        self.gesture.cancel_until_neutral();
        self.mapping = None;
        self.next_deadline = None;
        self.next_status_check = None;
        self.replay.clear();
        self.replay_velocity = [0.0; 3];
        self.last_replay = None;
        self.discard_short = false;
    }

    pub(super) fn accepted(&mut self, now: Instant) {
        const EXPECTED_DURATION: Duration = Duration::from_millis(2300);
        self.awaiting_completion = true;
        self.next_status_check = Some(now + EXPECTED_DURATION);
    }

    pub(super) fn observe_status(&mut self, now: Instant, status: Option<proto::PrecisionStatus>) {
        if matches!(status, Some(proto::PrecisionStatus { active: false })) {
            self.awaiting_completion = false;
            self.next_status_check = None;
        } else {
            self.next_status_check = Some(now + control::REFRESH_INTERVAL);
        }
    }
}

pub(super) fn sample_precision(
    precision: &mut PrecisionDrive,
    stream: &mut UnixStream,
    now: Duration,
    axes: [f64; 3],
    movement: proto::MoveParams,
) -> std::io::Result<PrecisionStep> {
    let out = precision.sample(now, axes, movement);
    if !out.started || !precision.awaiting_completion {
        return Ok(out);
    }

    notify(
        stream,
        &proto::Call::RobotPrecisionMove(proto::PrecisionMoveParams::Cancel),
    )?;
    precision.awaiting_completion = false;
    precision.next_status_check = None;
    Ok(out)
}

pub(super) fn poll_precision_status(
    precision: &mut PrecisionDrive,
    stream: &mut UnixStream,
    next_id: &mut u64,
    now: Instant,
) -> std::io::Result<()> {
    if precision
        .next_status_check
        .is_none_or(|deadline| now < deadline)
    {
        return Ok(());
    }
    let status = precision_status(stream, next_id, precision.heartbeat_period)?;
    precision.observe_status(Instant::now(), status);
    Ok(())
}

// Discrete precision requests keep the existing movement heartbeat alive while awaiting a reply.
fn precision_status(
    stream: &mut UnixStream,
    next_id: &mut u64,
    heartbeat_period: Duration,
) -> std::io::Result<Option<proto::PrecisionStatus>> {
    let response = precision_exchange(
        stream,
        next_id,
        proto::PrecisionMoveParams::Status,
        heartbeat_period,
    )?;
    if let Some(error) = &response.error {
        tracing::warn!(code = error.code, message = %error.message, "precision status refused");
        return Ok(None);
    }
    match response.result_as::<proto::PrecisionStatus>() {
        Ok(status) => Ok(Some(status)),
        Err(error) => {
            if let Ok(result) = response.result_as::<proto::IntentResult>() {
                tracing::warn!(reason = ?result.reason, "precision status not accepted");
            } else {
                tracing::warn!(%error, "invalid precision status response");
            }
            Ok(None)
        }
    }
}

fn precision_exchange(
    stream: &mut UnixStream,
    next_id: &mut u64,
    params: proto::PrecisionMoveParams,
    heartbeat_period: Duration,
) -> std::io::Result<proto::Response> {
    let id = proto::Id::Number(*next_id);
    *next_id += 1;
    let request = proto::Request::call(id, &proto::Call::RobotPrecisionMove(params));
    let heartbeat = proto::Request::notify(&proto::Call::RobotMove(proto::MoveParams::default()));
    connection::exchange_with_heartbeat(stream, &request, &heartbeat, heartbeat_period)
}

pub(super) fn precision_request(
    stream: &mut UnixStream,
    next_id: &mut u64,
    params: proto::PrecisionMoveParams,
    heartbeat_period: Duration,
) -> std::io::Result<proto::IntentResult> {
    let response = precision_exchange(stream, next_id, params, heartbeat_period)?;
    if let Some(error) = &response.error {
        return Err(std::io::Error::other(format!(
            "precision request refused: {}",
            error.message
        )));
    }
    let result: proto::IntentResult = response
        .result_as()
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidData, error))?;
    if !result.accepted {
        tracing::warn!(?params, reason = ?result.reason, "precision movement not accepted");
    }
    Ok(result)
}

pub(super) fn reset_precision(
    precision: &mut Option<PrecisionDrive>,
    stream: &mut UnixStream,
    next_id: &mut u64,
    can_send: bool,
) -> std::io::Result<()> {
    let Some(precision) = precision.as_mut() else {
        return Ok(());
    };
    precision.reset();
    if can_send && precision.awaiting_completion {
        let result = precision_request(
            stream,
            next_id,
            proto::PrecisionMoveParams::Cancel,
            precision.heartbeat_period,
        )?;
        if !result.accepted {
            return Err(std::io::Error::other(format!(
                "precision cancellation was refused: {}",
                result.reason.as_deref().unwrap_or("no reason provided")
            )));
        }
    } else if can_send {
        notify(
            stream,
            &proto::Call::RobotMove(proto::MoveParams::default()),
        )?;
    }
    precision.awaiting_completion = false;
    Ok(())
}
