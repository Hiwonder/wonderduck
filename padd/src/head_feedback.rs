//! Read head feedback on its own connection so it cannot stall the gamepad loop.

use std::io::{self, BufRead, BufReader, Read};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, Weak};
use std::time::{Duration, Instant};

use duck_control::manual_head::{FEEDBACK_TTL, measured_position};
use duck_ipc_proto as proto;

type Sample = Option<(Instant, [u16; 2])>;

pub struct HeadFeedback(Arc<Mutex<Sample>>);

impl HeadFeedback {
    pub fn start(socket: PathBuf) -> Self {
        let latest = Arc::new(Mutex::new(None));
        let reader = Arc::downgrade(&latest);
        if let Err(error) = std::thread::Builder::new()
            .name("pad-head-feedback".into())
            .spawn(move || {
                while reader.strong_count() > 0 {
                    if let Err(error) = subscribe(&socket, &reader) {
                        tracing::debug!(%error, "head feedback interrupted");
                    }
                    if let Some(latest) = reader.upgrade() {
                        *latest.lock().expect("head feedback lock") = None;
                    }
                    std::thread::sleep(Duration::from_millis(250));
                }
            })
        {
            tracing::warn!(%error, "could not start head feedback reader");
        }
        Self(latest)
    }

    pub fn position(&self, now: Instant) -> Option<[u16; 2]> {
        let (at, position) = (*self.0.lock().expect("head feedback lock"))?;
        (now.saturating_duration_since(at) <= FEEDBACK_TTL).then_some(position)
    }
}

fn subscribe(socket: &Path, latest: &Weak<Mutex<Sample>>) -> io::Result<()> {
    let mut stream = crate::connection::connect(socket)?;
    let response = crate::connection::exchange(
        &mut stream,
        &proto::Request::call(
            proto::Id::Number(1),
            &proto::Call::RobotSubscribe(proto::SubscribeParams {
                hz: Some(20),
                devices: false,
            }),
        ),
    )?;
    if !response.result_as::<proto::SubscribeResult>()?.accepted {
        return Err(io::Error::other(
            "robotd refused head feedback subscription",
        ));
    }
    stream.set_read_timeout(Some(FEEDBACK_TTL))?;

    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    while latest.strong_count() > 0 {
        line.clear();
        (&mut reader).take(1024 * 1024).read_line(&mut line)?;
        if !line.ends_with('\n') {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "incomplete robot state",
            ));
        }
        let Some(state) = serde_json::from_str::<proto::Request>(&line)
            .ok()
            .and_then(|request| request.as_state())
        else {
            continue;
        };
        let position = (!state.safety.limp
            && !state.safety.fallen
            && matches!(state.policy.as_str(), "walk" | "stand"))
        .then(|| measured_position(&state))
        .flatten();
        if let Some(latest) = latest.upgrade() {
            *latest.lock().expect("head feedback lock") =
                position.map(|position| (Instant::now(), position));
        }
    }
    Ok(())
}
