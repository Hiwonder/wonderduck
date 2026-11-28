//! Playback of the Studio rest action through the robot control loop.

use duck_control::io::AccessoryPositions;
use duck_control::model::{ACCESSORY_SERVO_IDS, ACCESSORY_SERVO_LIMITS, JOINT_IDS, NUM_JOINTS};
use std::io::{self, Read};
use std::path::Path;
use std::time::{Duration, Instant};

type Pose = [f64; NUM_JOINTS];
const MAX_BYTES: usize = 8 * 1024 * 1024;
const MAX_FRAMES: usize = 4096;

pub(crate) struct RestAction(Vec<Frame>);

pub(crate) struct Frame {
    pub(crate) duration_ms: u16,
    pub(crate) targets: [Option<f64>; NUM_JOINTS],
    pub(crate) accessories: AccessoryPositions,
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
            let mut accessories = [None; ACCESSORY_SERVO_IDS.len()];
            for id in &columns {
                let value = fields
                    .next()
                    .ok_or_else(|| invalid("incomplete rest.csv frame"))?;
                if value.is_empty() {
                    continue;
                }
                let raw = value
                    .parse::<u16>()
                    .map_err(|_| invalid(format!("invalid position for servo ID {id}")))?;
                if let Some(wing) = ACCESSORY_SERVO_IDS.iter().position(|servo| servo == id) {
                    let (lower, upper) = ACCESSORY_SERVO_LIMITS[wing];
                    if !(lower..=upper).contains(&raw) {
                        return Err(invalid(format!(
                            "rest.csv position exceeds travel for servo ID {id}"
                        )));
                    }
                    accessories[wing] = Some(raw);
                    continue;
                }
                let joint = JOINT_IDS
                    .iter()
                    .position(|servo| servo == id)
                    .ok_or_else(|| invalid(format!("unknown servo ID {id} in rest.csv")))?;
                let (position, limited_raw) = rest_joint_position(joint, raw)?;
                if limited_raw != raw {
                    tracing::warn!(frame = frames.len() + 1, servo_id = *id,
                        requested_raw = raw, applied_raw = limited_raw,
                        "rest.csv joint target limited to travel");
                }
                targets[joint] = Some(position);
            }
            if fields.next().is_some() {
                return Err(invalid("extra columns in rest.csv frame"));
            }
            frames.push(Frame {
                duration_ms: duration,
                targets,
                accessories,
            });
        }
        if !frames
            .iter()
            .any(|frame| frame.targets.iter().any(Option::is_some)
                || frame.accessories.iter().any(Option::is_some))
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

fn rest_joint_position(joint: usize, raw: u16) -> io::Result<(f64, u16)> {
    use duck_control::hx::{decode_position, joint_position_bounds};

    let angle = decode_position(joint, &raw.to_le_bytes())
        .map_err(|error| invalid(error.to_string()))?;
    let [lower, upper] = joint_position_bounds(joint)
        .map_err(|error| invalid(error.to_string()))?;
    let count = raw.clamp(lower, upper);
    if count == raw {
        return Ok((angle, raw));
    }
    let angle = decode_position(joint, &count.to_le_bytes())
        .map_err(|error| invalid(error.to_string()))?;
    Ok((angle, count))
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

    pub(crate) fn pending_frame(&self) -> Option<&Frame> {
        self.frame.as_ref().filter(|_| self.since.is_none())
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
    use duck_control::model::JOINT_LIMITS;

    #[test]
    fn each_frame_dispatches_once_and_runs_for_its_full_duration() {
        let action = RestAction::parse(
            "# format=hiwonder-servo-studio-action-csv-v3\nduration_ms,servo:31,servo:30,servo:35,servo:36\n2000,1336,2691,2040,2051\n2000,1820,,,\n"
        ).unwrap();
        let start = Instant::now();
        let mut playback = action.start([0.0; NUM_JOINTS]);
        let elapsed = |ms| start + Duration::from_millis(ms);
        let encode = |index, angle| duck_control::hx::encode_position(index, angle).unwrap();
        playback.advance(elapsed(2000));
        let first = playback.pending_frame().unwrap();
        assert_eq!((encode(6, first.targets[6].unwrap()), first.duration_ms), (1336, 2000));
        assert_eq!(first.accessories, [Some(2040), Some(2051)]);
        playback.confirm_sent(elapsed(2000));
        playback.advance(elapsed(3999));
        assert!(playback.pending_frame().is_none());
        playback.advance(elapsed(4000));
        let last = playback.pending_frame().unwrap();
        assert_eq!((encode(6, last.targets[6].unwrap()), last.duration_ms), (1820, 2000));
        assert_eq!(last.targets[5], None);
        assert_eq!(last.accessories, [None; 2]);
        assert_eq!(encode(5, playback.target()[5]), 2691);
        playback.confirm_sent(elapsed(4020));
        playback.advance(elapsed(6019));
        assert!(!playback.done());
        playback.advance(elapsed(6020));
        assert!(playback.done());
    }

    #[test]
    fn limits_recorded_knee_targets_to_encoder_counts_inside_travel() {
        let action = RestAction::parse(
            "duration_ms,servo:13,servo:23,servo:30\n1000,2893,1193,1711\n",
        ).unwrap();
        let playback = action.start([0.0; NUM_JOINTS]);
        let frame = playback.pending_frame().unwrap();
        for (id, expected) in [(13, 2889), (23, 1207), (30, 1711)] {
            let joint = JOINT_IDS.iter().position(|servo| *servo == id).unwrap();
            let angle = frame.targets[joint].unwrap();
            let (lower, upper) = JOINT_LIMITS[joint];
            assert!((lower..=upper).contains(&angle));
            assert_eq!(duck_control::hx::encode_position(joint, angle).unwrap(), expected);
        }
    }

    #[test]
    fn rejects_unknown_targets_and_invalid_raw_positions() {
        assert!(RestAction::parse("duration_ms,servo:31\n50,4096\n").is_err());
        assert!(RestAction::parse("duration_ms,servo:1\n50,2048\n").is_err());
        assert!(RestAction::parse("duration_ms,servo:35,servo:36\n50,683,3413\n").is_ok());
        assert!(RestAction::parse("duration_ms,servo:35\n50,2049\n").is_err());
        assert!(RestAction::parse("duration_ms,servo:36\n50,3414\n").is_err());
    }
}
