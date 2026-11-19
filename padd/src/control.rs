//! Gamepad ownership and the shared human-control RPC.

use std::io;
use std::os::unix::net::UnixStream;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;

pub const REFRESH_INTERVAL: Duration = Duration::from_millis(100);
const LEASE_TTL: Duration = Duration::from_millis(750);

pub fn stop_on_pad_loss(present: bool, owns: bool) -> bool {
    present && owns
}

#[derive(Default)]
pub struct RestChord {
    start: bool,
    select: bool,
    latched: bool,
    pending_rest: bool,
}

pub struct RestChordAction {
    pub rest: bool,
    pub suppress: bool,
}

impl RestChord {
    fn update(&mut self, start: bool, select: bool) {
        self.start = start;
        self.select = select;
        if start && select && !self.latched {
            self.latched = true;
            self.pending_rest = true;
        }
    }

    pub fn button(&mut self, button: gilrs::Button, pressed: bool) {
        match button {
            gilrs::Button::Start => self.update(pressed, self.select),
            gilrs::Button::Select => self.update(self.start, pressed),
            _ => {}
        }
    }

    pub fn finish(&mut self, start: bool, select: bool) -> RestChordAction {
        self.update(start, select);
        let action = RestChordAction {
            rest: self.pending_rest,
            suppress: self.latched,
        };
        self.pending_rest = false;
        if !start && !select {
            self.latched = false;
        }
        action
    }

    pub fn disconnected(&mut self) {
        self.start = false;
        self.select = false;
        self.pending_rest = false;
        self.latched = false;
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
