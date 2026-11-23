//! Playback of the Studio rest action through the robot control loop.

use duck_control::model::{JOINT_IDS, JOINT_LIMITS, NUM_JOINTS};
use std::io::{self, Read};
use std::path::Path;
use std::time::{Duration, Instant};

type Pose = [f64; NUM_JOINTS];
const MAX_BYTES: usize = 8 * 1024 * 1024;
const MAX_FRAMES: usize = 4096;

pub(crate) struct RestAction(Vec<Frame>);

struct Frame {
    duration_ms: u16,
    targets: [Option<f64>; NUM_JOINTS],
}

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}

impl RestAction {
    pub(crate) fn load() -> io::Result<Self> {
        let executable = std::env::current_exe()?;
        let script = crate::studio_process::script_path(&executable)
            .ok_or_else(|| io::Error::other("cannot locate Studio action groups"))?;
        let directory = script
            .parent()
            .expect("Studio script has a parent directory");
        Self::read(&directory.join("action-groups/rest.csv"))
    }

    fn read(path: &Path) -> io::Result<Self> {
        let mut document = String::new();
        std::fs::File::open(path)?
            .take((MAX_BYTES + 1) as u64)
            .read_to_string(&mut document)?;
        if document.len() > MAX_BYTES {
            return Err(invalid("rest.csv is too large"));
        }
        Self::parse(&document)
    }

    fn parse(document: &str) -> io::Result<Self> {
        let mut lines = document
            .trim_start_matches('\u{feff}')
            .lines()
            .map(str::trim)
            .filter(|line| !line.is_empty() && !line.starts_with('#'));
        let header = lines
            .next()
            .ok_or_else(|| invalid("rest.csv has no header"))?;
        let mut headings = header.split(',').map(str::trim);
        if headings.next() != Some("duration_ms") {
            return Err(invalid("rest.csv must start with duration_ms"));
        }
        let mut columns = Vec::new();
        for heading in headings {
            let id = heading
                .strip_prefix("servo:")
                .and_then(|value| value.parse::<u8>().ok())
                .filter(|id| (1..=36).contains(id))
                .ok_or_else(|| invalid(format!("invalid rest.csv column {heading}")))?;
            if columns.contains(&id) {
                return Err(invalid(format!("duplicate servo ID {id} in rest.csv")));
            }
            columns.push(id);
        }
        let mut frames = Vec::new();
        for line in lines {
            if frames.len() == MAX_FRAMES {
                return Err(invalid("rest.csv has too many frames"));
            }
            let mut fields = line.split(',').map(str::trim);
            let duration = fields
                .next()
                .and_then(|value| value.parse::<u16>().ok())
                .filter(|value| (1..=30_000).contains(value))
                .ok_or_else(|| invalid("invalid rest.csv frame duration"))?;
            let mut targets = [None; NUM_JOINTS];
            for id in &columns {
                let value = fields
                    .next()
                    .ok_or_else(|| invalid("incomplete rest.csv frame"))?;
                if value.is_empty() {
                    continue;
                }
                let joint = JOINT_IDS
                    .iter()
                    .position(|servo| servo == id)
                    .ok_or_else(|| invalid(format!("unknown servo ID {id} in rest.csv")))?;
                let raw = value
                    .parse::<u16>()
                    .map_err(|_| invalid(format!("invalid position for servo ID {id}")))?;
                let angle = duck_control::hx::decode_position(joint, &raw.to_le_bytes())
                    .map_err(|error| invalid(error.to_string()))?;
                let (lower, upper) = JOINT_LIMITS[joint];
                if !(lower..=upper).contains(&angle) {
                    return Err(invalid(format!(
                        "rest.csv position exceeds travel for servo ID {id}"
                    )));
                }
                targets[joint] = Some(angle);
            }
            if fields.next().is_some() {
                return Err(invalid("extra columns in rest.csv frame"));
            }
            frames.push(Frame {
                duration_ms: duration,
                targets,
            });
        }
        if !frames
            .iter()
            .any(|frame| frame.targets.iter().any(Option::is_some))
        {
            return Err(invalid("rest.csv contains no servo targets"));
        }
        Ok(Self(frames))
    }

    pub(crate) fn start(self, from: Pose) -> Playback {
        let frames = self.0.into_iter();
        let mut playback = Playback {
            frames,
            frame: None,
            goal: from,
            since: None,
        };
        playback.next_frame();
        playback
    }
}

pub(crate) struct Playback {
    frames: std::vec::IntoIter<Frame>,
    frame: Option<Frame>,
    goal: Pose,
    since: Option<Instant>,
}

impl Playback {
    fn next_frame(&mut self) {
        self.since = None;
        self.frame = self.frames.next();
        if let Some(frame) = &self.frame {
            for (goal, target) in self.goal.iter_mut().zip(frame.targets) {
                if let Some(target) = target {
                    *goal = target;
                }
            }
        }
    }

    pub(crate) fn target(&self) -> Pose {
        self.goal
    }

    pub(crate) fn pending_frame(&self) -> Option<(&[Option<f64>; NUM_JOINTS], u16)> {
        let frame = self.frame.as_ref()?;
        self.since
            .is_none()
            .then_some((&frame.targets, frame.duration_ms))
    }

    pub(crate) fn confirm_sent(&mut self, now: Instant) {
        self.since = Some(now);
    }

    pub(crate) fn advance(&mut self, now: Instant) {
        if let (Some(started), Some(frame)) = (self.since, &self.frame)
            && now.duration_since(started) >= Duration::from_millis(u64::from(frame.duration_ms))
        {
            self.next_frame();
        }
    }

    pub(crate) fn done(&self) -> bool {
        self.frame.is_none()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_frame_dispatches_once_and_runs_for_its_full_duration() {
        let action = RestAction::parse(
            "# format=hiwonder-servo-studio-action-csv-v3\nduration_ms,servo:31,servo:30\n2000,1336,2691\n2000,1820,\n"
        ).unwrap();
        let start = Instant::now();
        let mut playback = action.start([0.0; NUM_JOINTS]);
        let elapsed = |ms| start + Duration::from_millis(ms);
        let encode = |index, angle| duck_control::hx::encode_position(index, angle).unwrap();
        playback.advance(elapsed(2000));
        let (first, duration) = playback.pending_frame().unwrap();
        assert_eq!((encode(6, first[6].unwrap()), duration), (1336, 2000));
        playback.confirm_sent(elapsed(2000));
        playback.advance(elapsed(3999));
        assert!(playback.pending_frame().is_none());
        playback.advance(elapsed(4000));
        let (last, duration) = playback.pending_frame().unwrap();
        assert_eq!((encode(6, last[6].unwrap()), duration), (1820, 2000));
        assert_eq!(last[5], None);
        assert_eq!(encode(5, playback.target()[5]), 2691);
        playback.confirm_sent(elapsed(4020));
        playback.advance(elapsed(6019));
        assert!(!playback.done());
        playback.advance(elapsed(6020));
        assert!(playback.done());
    }

    #[test]
    fn rejects_out_of_travel_and_unknown_targets() {
        assert!(RestAction::parse("duration_ms,servo:31\n50,4095\n").is_err());
        assert!(RestAction::parse("duration_ms,servo:1\n50,2048\n").is_err());
    }
}
