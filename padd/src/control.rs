//! Gamepad ownership and the shared human-control RPC.

use std::io;
use std::os::unix::net::UnixStream;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;

pub const REFRESH_INTERVAL: Duration = Duration::from_millis(100);
pub const IDLE_RELEASE: Duration = Duration::from_millis(250);
const LEASE_TTL: Duration = Duration::from_millis(750);

pub fn stop_on_pad_loss(present: bool, owns: bool) -> bool {
    present && owns
}

const START_TAP_MAX: Duration = Duration::from_millis(1500);
const START_REST_HOLD: Duration = Duration::from_secs(3);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartAction {
    Initialize,
    Rest,
    Select,
    ToggleHeadDrive,
}

#[derive(Default)]
enum StartGesture {
    // Also used on connection to ignore buttons already down.
    #[default]
    AwaitRelease,
    Ready,
    Holding(Instant),
    SelectHeld,
    Chord,
    RestRequested,
}

#[derive(Default)]
pub struct StartControls {
    start: bool,
    select: bool,
    gesture: StartGesture,
    pending: Option<StartAction>,
}

impl StartControls {
    fn update(&mut self, start: bool, select: bool, now: Instant) {
        let select_pressed = select && !self.select;
        self.start = start;
        self.select = select;
        match self.gesture {
            StartGesture::Ready | StartGesture::Holding(_) | StartGesture::SelectHeld
                if start && select =>
            {
                self.gesture = StartGesture::Chord;
                self.pending = Some(StartAction::ToggleHeadDrive);
            }
            StartGesture::Ready if start => self.gesture = StartGesture::Holding(now),
            StartGesture::Ready if select_pressed => self.gesture = StartGesture::SelectHeld,
            StartGesture::SelectHeld if !select => {
                self.gesture = StartGesture::Ready;
                self.pending = Some(StartAction::Select);
            }
            StartGesture::Holding(since) if !start => {
                let elapsed = now.saturating_duration_since(since);
                if elapsed >= START_REST_HOLD {
                    self.request_rest();
                } else {
                    self.gesture = StartGesture::Ready;
                    if elapsed < START_TAP_MAX {
                        self.pending = Some(StartAction::Initialize);
                    }
                }
            }
            _ => {}
        }
    }

    fn request_rest(&mut self) {
        self.gesture = StartGesture::RestRequested;
        self.pending = Some(StartAction::Rest);
    }

    pub fn button(&mut self, button: gilrs::Button, pressed: bool, now: Instant) {
        match button {
            gilrs::Button::Start => self.update(pressed, self.select, now),
            gilrs::Button::Select => self.update(self.start, pressed, now),
            _ => {}
        }
    }

    pub fn finish(
        &mut self,
        start: bool,
        select: bool,
        now: Instant,
    ) -> (Option<StartAction>, bool) {
        self.update(start, select, now);
        if let StartGesture::Holding(since) = self.gesture
            && now.saturating_duration_since(since) >= START_REST_HOLD
        {
            self.request_rest();
        }
        let action = self.pending.take();
        if !start && (!select || matches!(self.gesture, StartGesture::Chord)) {
            self.gesture = StartGesture::Ready;
        }
        let suppress_motion = action == Some(StartAction::Rest)
            || matches!(self.gesture, StartGesture::RestRequested);
        (action, suppress_motion)
    }
}

#[cfg(test)]
mod start_controls_tests {
    use super::*;
    use gilrs::Button;

    #[test]
    fn select_fires_on_each_release_after_becoming_neutral() {
        let now = Instant::now();
        let held = now + START_REST_HOLD;
        let mut keys = StartControls::default();
        assert_eq!(keys.finish(false, true, now), (None, false));
        assert_eq!(keys.finish(false, false, now), (None, false));
        assert_eq!(keys.finish(false, true, now), (None, false));
        assert_eq!(keys.finish(false, true, held), (None, false));
        assert_eq!(keys.finish(false, false, held), (Some(StartAction::Select), false));
        assert_eq!(keys.finish(false, false, held), (None, false));
        keys.button(Button::Select, true, held);
        keys.button(Button::Select, false, held);
        assert_eq!(keys.finish(false, false, held), (Some(StartAction::Select), false));
    }

    #[test]
    fn releasing_select_after_a_chord_does_not_fire_select() {
        let now = Instant::now();
        let mut keys = StartControls::default();
        keys.finish(false, false, now);
        keys.finish(false, true, now);
        assert_eq!(keys.finish(true, true, now), (Some(StartAction::ToggleHeadDrive), false));
        assert_eq!(keys.finish(false, true, now), (None, false));
        assert_eq!(keys.finish(false, true, now), (None, false));
        assert_eq!(keys.finish(false, false, now), (None, false));
    }

    #[test]
    fn start_requests_rest_once_at_three_seconds() {
        let now = Instant::now();
        let held = now + START_REST_HOLD;
        let mut keys = StartControls::default();
        keys.finish(false, false, now);
        assert_eq!(keys.finish(true, false, now), (None, false));
        assert_eq!(
            keys.finish(true, false, held - Duration::from_millis(1)),
            (None, false)
        );
        assert_eq!(
            keys.finish(true, false, held),
            (Some(StartAction::Rest), true)
        );
        assert_eq!(keys.finish(true, true, held), (None, true));
        assert_eq!(keys.finish(false, true, held), (None, true));
        assert_eq!(keys.finish(false, false, held), (None, false));
    }

    #[test]
    fn chord_toggles_in_both_press_orders_and_consumes_start_release() {
        let now = Instant::now();
        let later = now + START_REST_HOLD;
        let mut keys = StartControls::default();
        keys.finish(false, false, now);
        for (first, second) in [
            (Button::Select, Button::Start),
            (Button::Start, Button::Select),
        ] {
            keys.button(first, true, now);
            keys.button(second, true, now);
            keys.button(second, false, now);
            assert_eq!(
                keys.finish(first == Button::Start, first == Button::Select, now),
                (Some(StartAction::ToggleHeadDrive), false)
            );
            if first == Button::Select {
                assert_eq!(
                    keys.finish(true, true, later),
                    (Some(StartAction::ToggleHeadDrive), false)
                );
            } else {
                assert_eq!(keys.finish(true, false, later), (None, false));
                assert_eq!(keys.finish(true, true, later), (None, false));
            }
            assert_eq!(keys.finish(true, true, later), (None, false));
            assert_eq!(keys.finish(false, false, later), (None, false));
        }
    }

    #[test]
    fn short_press_initializes_and_reconnect_requires_release() {
        let now = Instant::now();
        let mut keys = StartControls::default();
        assert_eq!(keys.finish(true, true, now), (None, false));
        assert_eq!(
            keys.finish(true, false, now + START_REST_HOLD),
            (None, false)
        );
        assert_eq!(
            keys.finish(false, false, now + START_REST_HOLD),
            (None, false)
        );
        let later = now + START_REST_HOLD;
        keys.button(Button::Start, true, later);
        keys.button(Button::Start, false, later);
        assert_eq!(
            keys.finish(false, false, later),
            (Some(StartAction::Initialize), false)
        );
        keys.finish(true, false, later);
        assert_eq!(
            keys.finish(false, false, later + START_TAP_MAX),
            (None, false)
        );
        keys.finish(true, false, later + START_TAP_MAX);
        assert_eq!(
            keys.finish(false, false, later + START_TAP_MAX + START_REST_HOLD),
            (Some(StartAction::Rest), true)
        );
    }
}

pub fn call(
    stream: &mut UnixStream,
    next_id: &mut u64,
    action: proto::HumanControlAction,
) -> io::Result<proto::HumanControlResult> {
    let id = proto::Id::Number(*next_id);
    *next_id += 1;
    let request = proto::Request::call(
        id.clone(),
        &proto::Call::RobotHumanControl(proto::HumanControlParams {
            action,
            source: proto::HumanControlSource::Gamepad,
        }),
    );
    let response = crate::connection::exchange(stream, &request)?;
    if response.error.is_some() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("human control response: {response:?}"),
        ));
    }
    response.result_as().map_err(Into::into)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    NeutralRequired,
    Armed,
    Owned,
}

pub struct Gate {
    state: State,
    refreshed: Option<Instant>,
}

impl Default for Gate {
    fn default() -> Self {
        Self {
            state: State::NeutralRequired,
            refreshed: None,
        }
    }
}

impl Gate {
    pub fn observe(&mut self, neutral: bool, activity: bool) -> bool {
        match self.state {
            State::NeutralRequired if neutral && !activity => self.state = State::Armed,
            State::Armed if activity => return true,
            _ => {}
        }
        false
    }

    pub fn acquired(&mut self, result: &proto::HumanControlResult, now: Instant) {
        if result.accepted && result.owner == Some(proto::HumanControlSource::Gamepad) {
            self.state = State::Owned;
            self.refreshed = Some(now);
        } else {
            self.lost();
        }
    }

    pub fn owns(&self) -> bool {
        self.state == State::Owned
    }
    pub fn driving(&self) -> bool {
        self.state == State::Owned
    }
    pub fn refresh_due(&self, now: Instant) -> bool {
        self.owns()
            && self
                .refreshed
                .is_some_and(|at| now.saturating_duration_since(at) >= REFRESH_INTERVAL)
    }
    pub fn expire(&mut self, now: Instant) {
        if self.owns()
            && self
                .refreshed
                .is_some_and(|at| now.saturating_duration_since(at) >= LEASE_TTL)
        {
            self.lost();
        }
    }
    pub fn refreshed(&mut self, result: &proto::HumanControlResult, now: Instant) {
        if result.accepted && result.owner == Some(proto::HumanControlSource::Gamepad) {
            self.refreshed = Some(now);
        } else {
            self.lost();
        }
    }
    pub fn lost(&mut self) {
        self.state = State::NeutralRequired;
        self.refreshed = None;
    }
}
