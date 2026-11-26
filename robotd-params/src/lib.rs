//! `robotd`'s startup parameters: the schema, the defaults, and the validation.
//!
//! A file rather than a wall of CLI flags — the prototype grew 142 of them and most were
//! variants, dead skills and dead sensors, all of which are gone. **Read once at startup,
//! not watched**; live reload is deferred. That fact
//! is load-bearing for tooling: *any* change to the file requires a `robotd` restart, so an
//! editor never has to ask which keys are live.
//!
//! It lives outside `releases/<ver>/` so it survives an update *and* a rollback: this is
//! per-robot configuration, not shipped defaults.
//!
//! A crate of its own rather than a module of `robotd`, for one consumer: `robotctl
//! configure` edits the file interactively, and doing that against a copied schema is how a
//! copied schema drifts. [`registry`] is the machine-readable index of every key — what it
//! is, what it defaults to, what values it takes — and its completeness is enforced by a
//! test that walks [`Params`]'s own serialization, so a new section cannot be added without
//! the registry (and therefore the editor) learning about it.

pub mod edit;
mod idle;
pub mod registry;

pub use idle::IdleParams;

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// Where a release is mounted.
pub const RELEASE_DIR: &str = "/opt/robot/daemon/current";

/// Where the official policy set lives — **outside the release directory**, on purpose.
///
/// A gait retrain should not need a daemon release, and a daemon fix should not re-download six
/// megabytes of unchanged weights. So the two version independently, and the daemon reads its
/// policies from one place regardless of what put them there.
///
/// Today what puts them there is the daemon's own postinstall hook, which seeds this directory
/// from the copies the release still carries. That is a bootstrap, not the destination: it stops
/// the moment anything installs a real set here and repoints `current`, and `current` being a
/// symlink beside a `releases/` directory is exactly the shape the updater already swaps
/// atomically.
pub const POLICY_DIR: &str = "/opt/robot/policies/current";

/// Where the duck detector lives — outside the release, the way [`POLICY_DIR`] is, and for the
/// same reason: the model is trained and published elsewhere (`pollen-robotics/duck_detector`,
/// on the Hub as `pollen-robotics/microduck-duck-detector`), a retrain is a tag rather than a
/// daemon release, and a daemon fix should not re-ship fourteen megabytes of unchanged weights.
///
/// Seeded by the release's own `scripts/seed-detector.sh`, moved past with
/// `robotctl duck-detector update`; `current` is a symlink beside a `releases/` directory, the layout
/// the policies use and the updater swaps atomically.
pub const DETECTOR_DIR: &str = "/opt/robot/detector/current";

/// Where policies fetched from the Hub one at a time live — `robotctl policy load <slot> <repo>`.
///
/// Outside every release directory: a policy somebody chose has to
/// survive an update and a rollback. The layout mirrors the repo — `<org>/<name>/<revision>/` —
/// which is what makes the path in `robotd.toml` say where a policy came from without anything
/// having to look it up. `updater::policy::LIBRARY_ROOT` is the same string on the writing side.
pub const POLICY_LIBRARY: &str = "/var/lib/robot/policies";

/// Where a provisioned robot keeps it, alongside the updater's own config.
pub const DEFAULT_PATH: &str = "/etc/robot/robotd.toml";

pub mod gamepad_idle;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Params {
    pub bus: Bus,
    pub control: Control,
    pub update_gate: UpdateGate,
    pub policy: PolicyParams,
    pub safety: SafetyParams,
    pub audio: AudioParams,
    pub theremin: ThereminParams,
    pub head_imu: HeadImuParams,
    pub chorale: ChoraleParams,
    pub idle: IdleParams,
    pub media: MediaParams,
    ///
    /// `[detect]` until 2026-09: in `robotctl configure` that read as "detect what?", one line
    /// from `chorale.accept`. Named for what it detects. The alias keeps a file written under the
    /// old name loading; the editor renames the section the next time it saves that file.
    #[serde(alias = "detect")]
    pub duck_detector: DuckDetectorParams,
    /// Which pad button runs which skill. `padd` reads this, not `robotd`.
    pub pad: PadParams,
    /// Posing the head from the pad's own IMU. `padd` reads this too.
    ///
    /// Named for what it is, in full, because `[head_imu]` is two sections up and is the IMU *in*
    /// the robot's head: the two were `imu_head` and `head_imu` for a while, and that is a name
    /// apart, not a difference. The alias keeps a file written under the old name loading; the
    /// editor renames the section the next time it saves that file.
    #[serde(alias = "imu_head")]
    pub pad_imu_head_control: PadImuHeadControlParams,
    /// How fast full stick deflection drives the robot. `padd` reads this as well.
    pub pad_drive: PadDriveParams,
}

/// The pad's walking speed limits: what full stick deflection asks for, per axis and per
/// direction.
///
/// Each axis has a bound per direction, signed in the robot's own frame — `vx` forward, `vy` to
/// the left, `vyaw` counter-clockwise seen from above — so a `_min` is the bound in the negative
/// direction and is itself negative (or zero, to forbid that direction). The stick is linear
/// between centre and either bound. Walk mode only: roller mode keeps its own shaping.
///
/// This is what the *pad* asks for, not a limit on the robot. A `robot.move` from anywhere else
/// is not bounded by it, and the policy only follows commands inside the range it was trained
/// on — asking for more than that is asking, not getting.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PadDriveParams {
    /// Full stick forward, m/s.
    pub vx_max: f64,
    /// Full stick back, m/s — negative.
    pub vx_min: f64,
    /// Full stick left, m/s.
    pub vy_max: f64,
    /// Full stick right, m/s — negative.
    pub vy_min: f64,
    /// Full right stick left (turn left), rad/s.
    pub vyaw_max: f64,
    /// Full right stick right (turn right), rad/s — negative.
    pub vyaw_min: f64,
}

impl Default for PadDriveParams {
    /// Full-deflection walking commands, constrained by the pad client's startup limits.
    fn default() -> Self {
        Self {
            vx_max: 0.4,
            vx_min: -0.4,
            vy_max: 0.4,
            vy_min: -0.4,
            vyaw_max: 1.5,
            vyaw_min: -1.5,
        }
    }
}

impl PadDriveParams {
    /// Each axis as `(name, min, max)`, for validation and logging.
    pub fn axes(&self) -> [(&'static str, f64, f64); 3] {
        [
            ("vx", self.vx_min, self.vx_max),
            ("vy", self.vy_min, self.vy_max),
            ("vyaw", self.vyaw_min, self.vyaw_max),
        ]
    }

    /// A stick deflection in `[-1, 1]` scaled onto `[min, max]`: positive deflection towards
    /// `max`, negative towards `min`, linear on each side of centre.
    pub fn scale(deflection: f64, min: f64, max: f64) -> f64 {
        if deflection >= 0.0 {
            deflection * max
        } else {
            -deflection * min
        }
    }
}

/// Legacy controller-IMU head control settings, retained to read older configuration files.
/// The current gamepad layout does not use the controller IMU for head control.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PadImuHeadControlParams {
    /// Whether head + drive follows the pad IMU when available.
    pub enabled: bool,
    /// Head radians per pad radian. One is "the head turns as far as the pad did"; more makes a
    /// small wrist movement a large head movement. The head's own travel limit still applies.
    pub gain: f64,
}

impl Default for PadImuHeadControlParams {
    fn default() -> Self {
        Self {
            enabled: false,
            gain: 1.0,
        }
    }
}

/// Which pad button runs which skill.
///
/// A and the bumpers are configurable skill buttons. Direction keys and B/X are reserved for
/// movement, Y is ground pick, and Start/Select control initialization and rest.
/// The old B/X/Y fields remain readable so that existing configurations do not need migration.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PadParams {
    /// A (South). Sit ↔ stand, by default.
    pub a: String,
    /// Legacy B binding, ignored because B is now turn right.
    pub b: String,
    /// Legacy X binding, ignored because X is now turn left.
    pub x: String,
    /// Legacy Y binding, ignored because Y now runs ground pick.
    pub y: String,
    /// The left bumper — `LeftTrigger` in gilrs, which names the *analog* trigger
    /// `LeftTrigger2`. Getting that backwards binds a skill to a control nobody presses.
    pub lb: String,
    /// The right bumper, likewise.
    pub rb: String,
}

impl Default for PadParams {
    fn default() -> Self {
        Self {
            a: "sit_toggle".to_owned(),
            b: String::new(),
            x: String::new(),
            y: String::new(),
            lb: "kick_left".to_owned(),
            rb: "kick_right".to_owned(),
        }
    }
}

impl PadParams {
    /// The bindable buttons, in the order a listing should print them.
    pub const BUTTONS: [&'static str; 3] = ["a", "lb", "rb"];

    /// What a button runs, or `None` for a name this build has no button for.
    pub fn skill(&self, button: &str) -> Option<&str> {
        Some(match button {
            "a" => &self.a,
            "lb" => &self.lb,
            "rb" => &self.rb,
            _ => return None,
        })
    }

    /// Bind a button to a skill. `false` for a button this build does not have.
    pub fn bind(&mut self, button: &str, skill: &str) -> bool {
        let slot = match button {
            "a" => &mut self.a,
            "lb" => &mut self.lb,
            "rb" => &mut self.rb,
            _ => return false,
        };
        *slot = skill.to_owned();
        true
    }

    /// Every button name, for the "expected one of" half of a refusal.
    pub fn names() -> String {
        Self::BUTTONS.join(", ")
    }
}

/// The one video mode a robot streams in, as a name rather than four numbers.
///
/// **Frame size, rate and a matching bitrate move together or not at all.** They are not
/// independent settings: 1080p at the 2 Mb/s that suits 720p is a smear, and 720p at 6 Mb/s
/// spends a link's headroom on nothing. Offering `width`, `height`, `fps` and `bitrate` as four
/// keys would make every wrong combination of them expressible — including the ones the capture
/// path cannot produce at all, and a pipeline that will not start costs the WebRTC *control*
/// channel along with the video, because the two are bundled (`remote-webrtc.md`).
///
/// Each mode specifies both the delivery geometry and its capture rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum Quality {
    /// The sensor's full frame. The most detail, and the rung least likely to hold 30 fps on
    /// this ISP path — [`MediaParams`] says what is measured and what is not.
    #[serde(rename = "1080p30")]
    Q1080p30,
    /// Full-field 4:3 output from the IMX219 binned readout on RK3566.
    #[serde(rename = "960p30")]
    Q960p30,
    /// Full-width 4:3 output from the IMX219 full-array readout on RK3566.
    #[serde(rename = "960p21")]
    Q960p21,
    /// What every measurement in `mediad` was taken at, and the default.
    #[default]
    #[serde(rename = "720p30")]
    Q720p30,
    /// Same picture, half the frames: the rung for a link that cannot carry 30.
    #[serde(rename = "720p15")]
    Q720p15,
    /// Small and cheap, for a bad link or a busy CPU.
    #[serde(rename = "360p30")]
    Q360p30,
}

/// Every mode, in the order an editor cycles them — and the strings the file uses.
///
/// One list, so the registry's choices, the file's values and [`Quality`] itself cannot disagree;
/// [`tests::every_quality_label_round_trips`] pins it to the enum in both directions.
pub const QUALITY_LABELS: &[&str] = &["1080p30", "960p30", "960p21", "720p30", "720p15", "360p30"];

impl Quality {
    /// The modes, in [`QUALITY_LABELS`] order.
    pub const ALL: [Quality; 6] = [
        Quality::Q1080p30,
        Quality::Q960p30,
        Quality::Q960p21,
        Quality::Q720p30,
        Quality::Q720p15,
        Quality::Q360p30,
    ];

    /// The name this mode has in the file.
    pub fn label(self) -> &'static str {
        match self {
            Quality::Q1080p30 => "1080p30",
            Quality::Q960p30 => "960p30",
            Quality::Q960p21 => "960p21",
            Quality::Q720p30 => "720p30",
            Quality::Q720p15 => "720p15",
            Quality::Q360p30 => "360p30",
        }
    }

    /// Frame size in pixels aligned for the ISP and encoder.
    pub fn size(self) -> (u32, u32) {
        match self {
            Quality::Q1080p30 => (1920, 1080),
            Quality::Q960p30 | Quality::Q960p21 => (1280, 960),
            Quality::Q720p30 | Quality::Q720p15 => (1280, 720),
            Quality::Q360p30 => (640, 360),
        }
    }

    pub fn width(self) -> u32 {
        self.size().0
    }

    pub fn height(self) -> u32 {
        self.size().1
    }

    pub fn fps(self) -> u32 {
        match self {
            Quality::Q960p21 => 21,
            Quality::Q720p15 => 15,
            _ => 30,
        }
    }

    /// What this mode streams at when `[media] bitrate` is unset — bits per second.
    ///
    /// Scaled with the pixel rate rather than picked per rung: 720p30 is the measured 2 Mb/s
    /// `mediad` has always used, and the others are that number times their share of the pixels
    /// per second, rounded to something a human can read. Congestion control moves from here, so
    /// this is a starting point rather than a cap.
    pub fn default_bitrate(self) -> u32 {
        match self {
            Quality::Q1080p30 => 4_000_000,
            Quality::Q960p30 => 2_700_000,
            Quality::Q960p21 => 2_000_000,
            Quality::Q720p30 => 2_000_000,
            Quality::Q720p15 => 1_000_000,
            Quality::Q360p30 => 800_000,
        }
    }
}

/// How `mediad` decides what bitrate to actually send at.
///
/// **This is a CPU setting as much as a network one.** The estimator is not free: on the board,
/// with one peer connected, `rtpgccbwe` is the single largest consumer in the process — 7.6% of a
/// core against `v4l2src`'s 0.3% — because it works per packet while capture works per DMABuf
/// handle. Turning it off deletes that thread.
///
/// What it costs is adaptivity, and that is not a small thing: adapting the rate to the link is
/// the whole reason `webrtcsink` is handed raw video rather than pre-encoded H.264
/// (`mediad::pipeline`). On a link that stays good — a robot one hop away on its own LAN — the
/// estimator spends CPU discovering a ceiling it will never hit. On a link that degrades, it is
/// what keeps a picture rather than a stall.
///
/// **It also decides what `bitrate` means.** With an estimator running, `bitrate` is a starting
/// point it ramps away from within seconds. Disabled, nothing moves it, and `bitrate` is the rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CongestionControl {
    /// Nothing adapts. `bitrate` is exactly what is sent, and a link that degrades degrades the
    /// picture rather than the rate.
    Disabled,
    /// `webrtcsink`'s own sender-side heuristic. Cheaper than the estimator, blunter than it.
    Homegrown,
    /// Google Congestion Control, and `webrtcsink`'s own default — so this is what every robot has
    /// been running, and naming it here changes nothing.
    #[default]
    Gcc,
}

/// Every mode, in the order an editor cycles them.
///
/// These are `webrtcsink`'s own property nicknames rather than names of ours: they are what gets
/// set on the element, and a second vocabulary in between would be one more thing to get wrong.
/// Note `gcc`, not `googcc` — [`tests::every_congestion_label_round_trips`] pins the spelling.
pub const CONGESTION_LABELS: &[&str] = &["disabled", "homegrown", "gcc"];

impl CongestionControl {
    pub const ALL: [CongestionControl; 3] = [
        CongestionControl::Disabled,
        CongestionControl::Homegrown,
        CongestionControl::Gcc,
    ];

    /// The `congestion-control` nickname `webrtcsink` knows this by.
    pub fn nick(self) -> &'static str {
        match self {
            CongestionControl::Disabled => "disabled",
            CongestionControl::Homegrown => "homegrown",
            CongestionControl::Gcc => "gcc",
        }
    }
}

/// Where `mediad`'s video comes from.
///
/// **This was `camera: bool`, and the boolean was the problem.** `camera = false` does not mean
/// "no video" — the WebRTC control channel rides the video track, so a pipeline that never starts
/// costs a robot its control surface along with its picture (`remote-webrtc.md`). What it means is
/// "a test pattern instead", and nothing in the key said so: a board set up with no camera streamed
/// 720p30 of synthetic video, which is how the pattern came to cost five times what the camera it
/// stands in for does. A name that says what you get is the fix for the next person reading it.
///
/// `--sim-camera` is not a value here. It carries an address, so it is a flag, and it overrides
/// this the way it always has.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MediaSource {
    /// The head camera, at [`Quality`]. What every robot with a camera wants, and the default.
    #[default]
    Camera,
    /// A test pattern at [`TEST_PATTERN_GEOMETRY`], for a board with no camera: the session
    /// exists, so signalling, negotiation, the datachannel and the control API are all reachable.
    Test,
}

/// Every source, in the order an editor cycles them — and the strings the file uses.
///
/// One list, for [`QUALITY_LABELS`]'s reason; [`tests::every_media_source_label_round_trips`]
/// pins it to the enum in both directions.
pub const MEDIA_SOURCE_LABELS: &[&str] = &["camera", "test"];

impl MediaSource {
    /// The sources, in [`MEDIA_SOURCE_LABELS`] order.
    pub const ALL: [MediaSource; 2] = [MediaSource::Camera, MediaSource::Test];

    /// The name this source has in the file.
    pub fn label(self) -> &'static str {
        match self {
            MediaSource::Camera => "camera",
            MediaSource::Test => "test",
        }
    }
}

/// What a test pattern runs at, whatever `[media] quality` says — width, height, frames a second.
///
/// **A test pattern is not video anybody watches.** It exists so a board with no camera still has
/// a whole session — signalling, negotiation, the datachannel, the control API — and none of that
/// needs the rung a camera streams at. Running it at the configured rung instead is what an idle
/// radxa-zero3 was measured doing: **29.4% of a core against the real camera's 6.1%** on the same
/// board, with nothing connected. A camera's frames arrive off the ISP in hardware, while a test
/// pattern is drawn by the CPU — at 720p30, 1.84 MB of packed UYVY thirty times a second, every
/// frame discarded unread. This is 73 KB five times a second: 0.7% of that pixel rate.
///
/// 16:9 with both axes a multiple of 8, for [`Quality::size`]'s reason. Five frames a second is
/// still a live stream — enough to prove the path end to end, which is all it is for.
pub const TEST_PATTERN_GEOMETRY: (u32, u32, u32) = (256, 144, 5);

/// `[media]` — what `mediad` streams.
///
/// **These were command-line flags in `mediad.service`, and that is why this section exists.**
/// The release installer rewrites that unit file, so the only supported way to change a flag was
/// a systemd drop-in — a mechanism nobody reaches for to answer "why is the video soft?". Here
/// they are three keys in the file `robotctl configure` already edits.
///
/// `mediad` reads this file at startup and nothing else does anything to it, so a change needs
/// `systemctl restart mediad` — not `robotd`. The editor offers the right one.
///
/// **What is measured and what is not.** 720p30 is the rung every number in `mediad::pipeline`
/// comes from: 29.3 fps off the ISP main path, with the capture format and buffer depth that took
/// three bench sessions to find. The sensor is pinned to a 1920x1080 mode that runs at 30 and the
/// ISP scales down from it, so 1080p30 asks for no scaling at all — what is unmeasured there is
/// whether the capture path and the encoder hold 30 fps at 2.25x the pixels. A rung that does not
/// hold runs slower; it is not a pipeline that fails to start.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct MediaParams {
    /// Where the video comes from: the head camera, or a test pattern for a board without one.
    /// [`MediaSource`] says why this is a name rather than a boolean.
    pub source: MediaSource,
    /// Frame size and rate, as one name. [`Quality`] says why it is one key and not four.
    pub quality: Quality,
    /// Starting video bitrate, bits per second. Unset follows the quality —
    /// [`Quality::default_bitrate`] — which is what almost every robot wants.
    ///
    /// A *starting* point unless `congestion_control` is `disabled`, which is the one setting that
    /// makes this the rate.
    pub bitrate: Option<u32>,
    /// Whether the send rate adapts to the link, and by what. [`CongestionControl`] has the
    /// trade — it is the largest single CPU consumer in this process.
    pub congestion_control: CongestionControl,
    /// This robot's own measured camera geometry, when somebody has written a `[media.intrinsics]`
    /// table for it — and only then.
    ///
    /// Absent is the common case and not a gap: `mediad` falls back to the hardware family's
    /// calibration ([`CameraIntrinsics::alpha`] — the camera and lens are one part across a
    /// revision, so it is a real solve of the same optics) and publishes it tagged `source:
    /// "family"`, with the module's design figures (`source: "nominal"`) below that for a camera
    /// nobody has solved at all. So this stays `Some` only for a per-robot solve, which is exactly
    /// what lets `media.video` distinguish *this* robot's calibration from the family's. See
    /// [`CameraIntrinsics`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intrinsics: Option<CameraIntrinsics>,
}

/// A camera calibration, as OpenCV's `calibrateCamera` produces one.
///
/// **The resolution it was measured at is part of the record**, because intrinsics are in pixels:
/// the same camera calibrated at 1280×720 and at 640×360 yields numbers that differ by a factor of
/// two and describe the same optics. `mediad` scales them to whatever is being streamed, and
/// refuses to when the aspect ratio differs — a frame of another shape is not a resize of this
/// one, and which part was cropped is not recoverable from its size.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraIntrinsics {
    /// The frame size these were measured at, in pixels.
    pub width: u32,
    pub height: u32,
    /// Focal length in pixels, per axis. Equal for square pixels, which these are.
    pub fx: f64,
    pub fy: f64,
    /// Where the optical axis crosses the image, in pixels from the top-left of the **delivered,
    /// unrotated** frame. A few pixels off centre on a real module.
    pub cx: f64,
    pub cy: f64,
    /// `k1 k2 p1 p2 k3`, OpenCV's order. Empty for a calibration that did not model distortion.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub distortion: Vec<f64>,
}

impl CameraIntrinsics {
    /// The alpha family's camera calibration: the head camera module and M12 lens every alpha unit
    /// carries — one part across the family, so one solve is every unit's calibration.
    ///
    /// Calibration applies to the unrotated 1280×720 output and the approximately 62° horizontal
    /// field of view of the scaled full-frame sensor mode.
    pub fn alpha() -> Self {
        Self {
            width: 1280,
            height: 720,
            fx: 1061.8060025020175,
            fy: 1062.193713957031,
            cx: 596.7776848217006,
            cy: 474.52348043048636,
            distortion: vec![
                -0.34695388691888,
                0.14615866153945595,
                -0.0007449157712424215,
                -0.0022167170528743837,
                -0.016437266141048814,
            ],
        }
    }
}

// Derivable since `camera: bool` became `source: MediaSource`, and still written out: every line
// of it is a decision with a reason beside it, and `#[derive(Default)]` would delete the reasons
// while keeping the values. The next field added here should have to say what its default is for.
#[allow(clippy::derivable_impls)]
impl Default for MediaParams {
    fn default() -> Self {
        Self {
            // The camera, because a robot with a camera is the case. A board without one shows a
            // test pattern instead, and only if somebody names it.
            source: MediaSource::Camera,
            quality: Quality::default(),
            bitrate: None,
            // Only a per-robot solve goes here; the family calibration is `mediad`'s fallback.
            intrinsics: None,
            // `webrtcsink`'s own default, named rather than inherited: what the element defaults
            // to is a fact about a plugin we ship from a pinned release, and the day it changes
            // should not be the day every robot's send rate changes with it.
            congestion_control: CongestionControl::default(),
        }
    }
}

/// `[duck_detector]` — finding other ducks in the camera.
///
/// **Read by `mediad`, not by `robotd`**, which is a first for this file: the frames are on
/// `mediad`'s tee and perception belongs next to the sensor. It lives here anyway, because this is
/// the file `robotctl configure` edits and a robot has one place where its switches are — a second
/// config file for the second daemon that wants one is how a fleet ends up with settings nobody can
/// find.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct DuckDetectorParams {
    /// Off by default. The detector costs ~50 ms of CPU per frame and some heat; a robot that
    /// nothing asks to look for ducks should not be paying for it.
    pub enabled: bool,
    /// Where to look, and therefore *what runs it*: a `.rknn` goes to the NPU, an `.onnx` runs on
    /// the CPU. Absent means the installed set under [`DETECTOR_DIR`], NPU first — see
    /// [`DuckDetectorParams::models`].
    pub model: Option<PathBuf>,
    /// Frames per second to run the detector at.
    ///
    /// **2 Hz is a thermal number, not a taste.** Flat out on a Radxa Zero 3 this reaches 95 °C and
    /// the CPU throttles to 408 MHz, which is a robot that walks badly to see well. Two looks a
    /// second is plenty for "is there a duck over there", and costs about a tenth of one core.
    pub hz: f64,
    /// Confidence a detection needs, against **this** model.
    ///
    /// A quantised model's output tensor carries its own scale, so the number that means 0.9 on the
    /// float model does not mean 0.9 here — the INT8 scores of the shipped model saturate around
    /// 1.4. Tuned on the board, not inherited from training.
    pub threshold: f32,
}

impl Default for DuckDetectorParams {
    fn default() -> Self {
        Self {
            enabled: false,
            model: None,
            hz: 2.0,
            threshold: 0.35,
        }
    }
}

impl MediaParams {
    /// The bitrate the daemon will actually start at.
    pub fn bitrate_resolved(&self) -> u32 {
        self.bitrate
            .unwrap_or_else(|| self.quality.default_bitrate())
    }

    /// Frame size and rate the pipeline will actually run at — width, height, frames a second.
    ///
    /// [`Quality`], except for the test pattern, which gets [`TEST_PATTERN_GEOMETRY`] instead.
    /// A simulated camera is a camera: `--sim-camera` renders the configured rung, and `mediad`
    /// applies that precedence where it picks the source.
    pub fn geometry(&self) -> (u32, u32, u32) {
        match self.source {
            MediaSource::Camera => (
                self.quality.width(),
                self.quality.height(),
                self.quality.fps(),
            ),
            MediaSource::Test => TEST_PATTERN_GEOMETRY,
        }
    }
}

impl DuckDetectorParams {
    /// The models to try, best first. Empty when the detector is off.
    ///
    /// **A list, not a choice**, because whether the NPU works is not something this file can know.
    /// The `.rknn` is preferred — it is why the detector is cheap — but a board whose NPU is
    /// disabled in its device tree (which is how Armbian ships the Radxa Zero 3) or which never ran
    /// `setup-npu.sh` has no runtime to load it with. Falling through to the `.onnx` means such a
    /// board still sees, on the CPU, instead of logging one warning and doing nothing for ever.
    ///
    /// An explicit `model` is the operator being specific, so it is tried alone.
    ///
    /// **Not filtered on existence.** A board whose set was never seeded — first install with no
    /// network — has neither file, and an empty list here would read as "detector off" in
    /// `mediad`'s journal when the truth is "detector on, model missing". Letting the loader fail
    /// on the path names the directory and what fills it.
    pub fn models(&self) -> Vec<PathBuf> {
        if !self.enabled {
            return Vec::new();
        }
        if let Some(path) = &self.model {
            if is_none_sentinel(path) {
                return Vec::new();
            }
            return vec![path.clone()];
        }
        let set = PathBuf::from(DETECTOR_DIR);
        DETECTOR_FILES.iter().map(|name| set.join(name)).collect()
    }
}

/// The two files a detector set holds, NPU first. `scripts/seed-detector.sh` and
/// `updater::policy::DETECTOR_FILES` download exactly this list, and a test in each place holds
/// them together.
pub const DETECTOR_FILES: [&str; 2] = ["duck_detect.rknn", "duck_detect.onnx"];

/// `[chorale]` — several ducks singing one piece.
///
/// `accept` is **false by default, and that is the whole section.** A chorale is not only a sound:
/// it moves the mouth and it moves the head. A robot that began animating because another robot
/// walked into the room would be doing motion nobody asked for, in someone's living room, and two
/// people's ducks in a café have no business pairing up. Off also means *invisible* rather than
/// visibly declining — a duck that has not opted in puts nothing on the air at all.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ChoraleParams {
    /// Whether this robot may sing with others at all. `false` — and it is derived rather than
    /// written out, so that the default cannot be changed by editing one word.
    pub accept: bool,
}

/// `[theremin]` — the ToF theremin: what counts as a hand, and where the depth frames come
/// from.
///
/// The interesting field is `statuses`, and it is the reason this section exists at all. ST
/// documents 5 and 9 as "range valid", and a build that believes only those stops seeing a
/// hand at about 30 cm on this sensor — past that a moving hand comes back as 4 or 13,
/// *consistency failed*, carrying a distance that is fine for a pitch. That took a bench
/// session to find, so the set is configurable: a duck whose theremin has a short reach wants
/// more codes in, and one that plays phantom notes at nothing wants fewer.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ThereminParams {
    /// Master switch. **Off by default**: the theremin is a party trick, and a duck that
    /// nobody asked to play one should not be reaching for the depth stream at all. Turning
    /// it on grants the *ability* to pick the instrument up — that still takes
    /// `robot.theremin` — on a duck where somebody wants it and the ToF is known good.
    pub enabled: bool,
    /// `tofd`'s depth stream.
    pub socket: PathBuf,
    /// Nearest playable range, metres.
    pub near_m: f64,
    /// Farthest playable range, metres.
    pub far_m: f64,
    /// Fewest zones that make a hand.
    pub min_zones: usize,
    /// ST status bytes whose distance is believed. See the section docs — this is the one
    /// that decides how far the instrument reaches.
    pub statuses: Vec<u8>,
    /// How long a note is held through a sensor dropout, milliseconds. This is what keeps a
    /// flickering zone from chopping a note into gravel.
    pub hold_ms: u64,
}

impl Default for ThereminParams {
    fn default() -> Self {
        let hand = kinematics::hand::Config::default();
        Self {
            enabled: false,
            socket: PathBuf::from(duck_ipc_proto::socket::TOF),
            near_m: hand.near_m,
            far_m: hand.far_m,
            min_zones: hand.min_zones,
            statuses: hand.statuses,
            hold_ms: hand.hold.as_millis() as u64,
        }
    }
}

impl ThereminParams {
    /// The hand-detection config these params describe.
    pub fn hand(&self) -> kinematics::hand::Config {
        kinematics::hand::Config {
            near_m: self.near_m,
            far_m: self.far_m,
            min_zones: self.min_zones,
            statuses: self.statuses.clone(),
            hold: std::time::Duration::from_millis(self.hold_ms),
        }
    }
}

/// `[head_imu]` — the BMI088 on the head module, read by `tofd` and served as
/// `head_imu.stream`.
///
/// **One switch, and it is off.** Reading this chip at 100 Hz costs ~3.5–4.5% of a core on an
/// RK3566, and a bench that isolates the parts says none of it is fixable in the loop: being
/// woken a hundred times a second is 0.7 points of it, the Madgwick fusion 0.3, and the rest is
/// the two I²C transactions a sample takes. Fewer bytes is not on offer — a gyro and an
/// accelerometer sample *is* twelve bytes — so what is left is not reading it, which is this
/// key, or reading it less often, which is `tofd --imu-hz`.
///
/// It stays off until something subscribes to the stream, because for now nothing does: it was
/// added for the mapping work, and a duck that is not mapping was paying for it from boot.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct HeadImuParams {
    /// Read the head IMU at all. `false` — and derived rather than written out, so the default
    /// cannot be changed by editing one word. `tofd --imu` overrides it for a session.
    pub enabled: bool,
}

/// `[audio]` — the voice and the microphone. All optional equipment: a robot without a
/// codec (or a bank) walks identically and stays quiet, so nothing here reaches a health
/// verdict.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct AudioParams {
    /// Master switch: no sounds, no mic worker.
    pub enabled: bool,
    /// ALSA playback device — the TLV320AIC3104 codec.
    pub device: String,
    /// ALSA capture PCM when the microphone uses a separate device.
    pub capture_device: Option<String>,
    /// Executable with the `aplay` argument contract. A board adapter may hold an amplifier GPIO.
    pub player: PathBuf,
    /// Where the per-robot voice bank lives. The release's postinstall renders it there
    /// (`sounds ensure-bank`), seeded from the SoC serial.
    pub bank: PathBuf,
    /// Quack once as the control loop comes up. On by default because on a headless board
    /// it is the audible "robotd is running"; off for anyone who restarts the daemon all
    /// day and would rather it did so quietly.
    pub greet: bool,
    /// Listen for petting on the onboard mic and coo about it. Absent means **off**: the
    /// per-mode resolution the prototype shipped (on for walking) cooed at every incidental
    /// head scratch, which wore thin fast. Set `true` to opt in.
    pub pet_detect: Option<bool>,
    /// The petting classifier. Absent means the release's copy; the literal `"none"`
    /// disables it outright.
    pub pet_model: Option<PathBuf>,
    /// Probability above which petting starts, and below which it ends (hysteresis).
    pub pet_enter_threshold: f32,
    pub pet_exit_threshold: f32,
}

impl Default for AudioParams {
    fn default() -> Self {
        Self {
            enabled: true,
            device: "plughw:aic3104".to_owned(),
            capture_device: None,
            player: PathBuf::from("aplay"),
            bank: PathBuf::from("/var/lib/robot/sounds"),
            greet: true,
            pet_detect: None,
            pet_model: None,
            pet_enter_threshold: 0.95,
            pet_exit_threshold: 0.85,
        }
    }
}

impl AudioParams {
    /// Whether the mic worker runs, resolved against the drive mode.
    pub fn pet_detect_resolved(&self, _mode: Mode) -> bool {
        // Off unless asked for, in either mode. It used to resolve per mode as the prototype's
        // launcher did (on for walking, off for the roller) — and cooing at every incidental
        // head scratch turned out to be more annoying than charming in daily use. The mode is
        // still passed so flipping this back is a one-line change, not a signature change.
        self.pet_detect.unwrap_or(false)
    }

    /// Preserve named ALSA PCMs and use device 0 for existing hardware card settings.
    pub fn capture_device(&self) -> String {
        if let Some(device) = &self.capture_device {
            device.clone()
        } else if (self.device.starts_with("plughw:") || self.device.starts_with("hw:"))
            && !self.device.contains(',')
        {
            format!("{},0", self.device)
        } else {
            self.device.clone()
        }
    }

    /// The classifier path, or `None` when disabled with the `"none"` sentinel.
    pub fn pet_model_resolved(&self) -> Option<PathBuf> {
        match &self.pet_model {
            Some(p) if is_none_sentinel(p) => None,
            Some(p) => Some(p.clone()),
            None => Some(PathBuf::from(RELEASE_DIR).join("models/pet_detect.onnx")),
        }
    }
}

/// Which drive configuration this robot runs. One robot, two personalities: legs, or the
/// roller. They differ in policies *and* tuning, so the mode is one switch here rather than
/// six paths an operator has to keep consistent — the prototype's launcher kept two whole
/// command lines for the same reason. Switching is an edit plus `systemctl restart robotd`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    #[default]
    Walk,
    Roller,
}

impl Mode {
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Walk => "walk",
            Mode::Roller => "roller",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PolicyParams {
    /// Whether to load a policy at all.
    ///
    /// False means slice 1's behaviour: run the loop, hold the pose, stay healthy. That is a
    /// legitimate configuration — it is the safest thing to be doing while hammering
    /// install/rollback cycles at a bench — and it is distinct from a policy that was wanted
    /// and could not be loaded, which is unhealthy.
    pub enabled: bool,
    /// `walk` (default) or `roller`. Changes which policies load *and* the tuning defaults
    /// below — every unset field resolves per mode, so a roller robot needs one line.
    pub mode: Mode,
    /// Policy paths. Absent means the mode's default inside the release directory, so a
    /// normal update ships them; point one elsewhere to try a build without cutting a
    /// release. The literal `"none"` disables a slot outright — the prototype's convention.
    pub walk: Option<PathBuf>,
    /// Standing policy. Without one the walking policy runs at every velocity.
    pub stand: Option<PathBuf>,
    /// Commanded sit↔stand (posture flag in the twist `vx` slot). Sit toggle, shutdown sit
    /// and the seated-boot rise all need it.
    pub sitstand: Option<PathBuf>,
    /// Phase-scripted ground pick. In roller mode this slot holds the crouch.
    pub ground_pick: Option<PathBuf>,
    pub kick_left: Option<PathBuf>,
    pub kick_right: Option<PathBuf>,
    /// Episodic forward roll. Ships by default in both modes, as the prototype now does.
    pub roulade: Option<PathBuf>,
    /// Scales raw policy output into a joint offset. Absent resolves per mode: 0.9 walking
    /// (the prototype's alpha default), 0.8 roller.
    pub action_scale: Option<f64>,
    pub standing_action_scale: f64,
    /// Standing runs softer, at this fraction of `gain`.
    pub standing_gain_ratio: f64,
    /// Position P gain while running.
    pub gain: u16,
    /// First-order low-pass on the head joint targets, `1.0` = pass-through. Default 0.5
    /// in both modes — the value the alpha policies are *trained* with, so it must match
    /// or transfer degrades. (The roller preset used to ship it off; the prototype rebased
    /// its roller line on the alpha defaults, and this follows.)
    pub head_lowpass: Option<f64>,
    /// Same, for the ten leg joints. Walking default 0.7.
    pub legs_lowpass: Option<f64>,
    /// One ground-pick cycle, seconds. The move ends at the set's `end_phase`.
    /// Absent resolves from the installed set's phase-encoded
    /// entry for this mode, else 4.0 walking, 3.0 roller (the crouch).
    pub ground_pick_period: Option<f64>,
    /// Phase at which the loaded pickup hands back. An explicit value overrides the
    /// installed set, allowing a custom policy to complete its own return window.
    pub ground_pick_end_phase: Option<f64>,
    /// Action scale while the ground pick runs. Absent: the set's entry, else 1.0 walking,
    /// 0.8 roller.
    pub ground_pick_action_scale: Option<f64>,
    /// Gain multiplier while the ground pick runs.
    pub ground_pick_gain_ratio: f64,
    /// The one-shot skills this robot has, in priority order.
    ///
    /// Empty means the built-in three — kicks and roulade, with the numbers they have always
    /// had — so a board that updates onto this keeps working with nothing written. An entry
    /// merges by name: naming `roulade` changes that one, naming anything else adds a skill.
    /// The file stays a list of decisions rather than a copy of the defaults, which is what
    /// `robotctl configure` promises about every other key here.
    #[serde(default, rename = "skill", skip_serializing_if = "Vec::is_empty")]
    pub skills: Vec<SkillDef>,
    /// Scale actions with battery voltage: effective scale × (nominal / measured). The
    /// servos' effective kP tracks their supply, so this holds the robot's response steady
    /// as the pack sags.
    pub voltage_adapt: bool,
    /// Reference voltage for `voltage_adapt` — the supply the gains were identified at.
    pub nominal_voltage: f64,
}

/// The literal that disables an optional policy slot, per the prototype's `--x-policy None`.
///
/// Public because three places need to agree on it: this crate resolving config, `robotctl`
/// accepting `policy load <slot> none`, and `robotd` recognising it on the wire. A second spelling
/// of it somewhere would be a slot that looks disabled in a file and is not.
pub fn is_none_sentinel(path: &std::path::Path) -> bool {
    path.as_os_str().eq_ignore_ascii_case("none")
}

/// A one-shot skill: a network, how long it drives, and what it changes while it does.
///
/// **The thing this replaces was three hardcoded arms that differed in four numbers.** Kicks and
/// roulade wrote the same all-zero command into the same kind of window; they differed in
/// duration, action scale, gain ratio, and whether holding the button chained another. A
/// community policy like `polite-bow` — zero command, four seconds, selecting it is the trigger —
/// is a fifth set of the same four numbers, and could not be added without a daemon release.
///
/// Deliberately *only* the zero-command family. `walk` and `stand` are the fallback pair chosen
/// by command magnitude, `sitstand` is latched and driven internally by the shutdown sit and the
/// seated-boot rise, and `ground_pick` writes a scripted phase rather than a constant. Those stay
/// where they are until something needs them not to; see `docs/ideas/policy-moves.md`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SkillDef {
    /// What a client asks for — `robot.do {skill: "roulade"}` — and what a pad button binds to.
    pub name: String,

    /// The `.onnx`. Absent means this robot's own copy, by convention `<name>.onnx` in the
    /// policy directory, so a built-in needs no path and a fetched one carries the path
    /// `robotctl policy load` wrote.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<PathBuf>,

    /// Seconds the network drives before handing back to walk or stand — or, when `unwind_s`
    /// is set, before it starts coming back.
    pub duration: f64,

    /// The twist this skill's network is fed while it runs.
    ///
    /// Zeros for most of them, which is what a policy trained with `zero_command_padding`
    /// expects and what made kicks, roulade and `polite-bow` the same arm. A non-zero constant
    /// is how a policy with its own command encoding becomes a one-shot: the published flamingo
    /// reads `[flag, side, 0]`, so `[1, 1, 0]` is "lift the left leg".
    ///
    /// Head and body stay zeroed either way. Every one-shot published so far declares them
    /// unused, and a skill that wanted them live would be a different shape than this.
    #[serde(default)]
    pub command: [f64; 3],

    /// The twist fed for `unwind_s` after the window, before handing back.
    ///
    /// **This is what lets a policy with no ending of its own be a one-shot.** An episodic
    /// policy returns itself to a safe pose — `polite-bow` is standing again after its four
    /// seconds — so handing straight back to walk is fine. A perpetual one does not: it holds
    /// until told otherwise, and handing back mid-hold gives walk a robot balanced on one foot.
    /// Driving the idle command for a moment first is the daemon supplying the ending the policy
    /// does not have, which is exactly what the sit toggle already does on its way up.
    #[serde(default)]
    pub unwind: [f64; 3],

    /// Seconds spent on `unwind` before handing back. Zero — the default — means the policy ends
    /// itself and the window simply expires.
    #[serde(default)]
    pub unwind_s: f64,

    /// Whether a request arriving while it runs starts another when this one finishes — how a
    /// client maps "the button is held" onto a one-shot. Roulade does; a kick does not.
    #[serde(default)]
    pub chain: bool,

    /// What this skill changes about the robot while it runs, and only while it runs.
    #[serde(default)]
    pub params: SkillOverrides,
}

/// Parameters a skill changes for its duration, restored when it ends.
///
/// **Raw parameter names, not a vocabulary of our own.** `cmd_alpha` rather than an invented
/// `smoothing = "off"`: one set of names for a person to learn, and `robotctl configure` already
/// documents every one of them.
///
/// **No fall-gate override here either, and for a sharper reason: a running skill already has
/// the fall reflex switched off.** The limp-fall predictor is only consulted while the
/// controller is not `busy()`, and any active skill makes it busy — so a move that leans past
/// the gate cannot trip it, and a field to raise the gate would have been decoration. What that
/// *does* mean is that the reflex is off for the whole of a skill, which was uncontroversial for
/// a half-second kick and is worth a second look for anything long.
///
/// **No `cmd_alpha` here, and its absence is the point.** Smoothing is applied to the *client's*
/// command on its way in, and a skill never reads that: the loop builds a fresh command block
/// from `command` or `unwind` and feeds it straight to the network. So a skill's twist is
/// unsmoothed by construction — which is exactly what a policy reading a flag rather than a
/// velocity needs, and the reason driving one through `robot.move` into the walk slot needed
/// `cmd_alpha = 1.0` set globally and remembered afterwards.
///
/// **A named set rather than "any key".** A skill that could widen a joint limit or lengthen the
/// deadman would be reaching past the layer that makes a stranger's policy safe to try at all —
/// which is the entire argument for allowing one on the robot. Everything here is tuning or a
/// threshold that a *move* legitimately owns for its own duration.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct SkillOverrides {
    /// Scales raw policy output into a joint offset, as `[policy] action_scale` does.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action_scale: Option<f64>,

    /// Multiplier on the running gain, so a move can be softer or stiffer than the gait.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gain_ratio: Option<f64>,
}

impl SkillDef {
    /// The file this skill runs, resolved the way every other policy path resolves.
    pub fn resolved_path(&self) -> Option<PathBuf> {
        match &self.path {
            Some(p) if is_none_sentinel(p) => None,
            Some(p) => Some(p.clone()),
            None => Some(PathBuf::from(POLICY_DIR).join(format!("{}.onnx", self.name))),
        }
    }
}

/// What the official policy set says about itself, installed beside the `.onnx` files.
///
/// The set is fetched from the Hub and versioned there, so what it contains — and how long each
/// one-shot runs — is a property of the set rather than of this build. Without this, adding a
/// tenth policy to the set meant a daemon release: one edit to the seeder's download list so it
/// arrives, and another here so it is a skill. That is the same coupling this whole exercise
/// removed for a stranger's policy, still in place for our own.
///
/// Absent is normal and not an error: a board seeded before the set carried one, or one where
/// the fetch has not happened yet. The three built-ins below are the fallback.
#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(default)]
pub struct SetManifest {
    pub policies: Vec<SetPolicy>,
}

/// One policy in the official set.
///
/// **The same field names a single-policy repo uses**, plus `file` to say which `.onnx` it
/// describes. That is deliberate: the community convention is one repo per policy with a flat
/// manifest, and the official set is nine policies in one repo. Sharing the vocabulary means
/// asking a publisher to *add fields*, not to adopt a second format, and it means one reader
/// understands both.
#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(default)]
pub struct SetPolicy {
    /// The `.onnx`, as it is named in the repo and on disk. The only field a standalone manifest
    /// has no use for, since a repo with one policy has nothing to disambiguate.
    pub file: String,
    /// What a client asks for, when this is a one-shot. Absent means the file's stem, so
    /// `roulade.onnx` needs no name while `ball_kick_left.onnx` says `kick_left` — the names are
    /// roles and the files are training runs, an indirection worth keeping.
    pub name: Option<String>,
    /// `"episodic"`, `"perpetual"` or `"scripted"`, and the difference is who supplies the
    /// ending.
    ///
    /// An episodic policy runs for `duration_s` and returns itself to a safe pose. On a constant
    /// command it is a skill on its own; on a phase command (`command.encoding = "phase"`) it is
    /// the ground pick — the daemon writes the phase, and the numbers here are how fast. A
    /// perpetual one has no length of its own — how long to hold a foot up is a person's choice,
    /// so it takes a config entry rather than appearing. A scripted one is episodic but
    /// interruptible: the daemon drives it through a command it can change mid-flight, which is
    /// what the sit↔stand posture flag is.
    pub kind: Option<String>,
    /// Seconds it runs, for a policy that ends itself. For a phase policy this is
    /// `period_s × end_phase`, recorded so a reader need not multiply.
    pub duration_s: Option<f64>,
    /// Whether a request arriving while it runs starts another when this one finishes — how a
    /// client maps "the button is held" onto a one-shot.
    pub chain: bool,
    /// Scales raw output into a joint offset, when this policy wants its own.
    pub action_scale: Option<f64>,
    /// Seconds a perpetual policy needs to get back to its idle command — or, for the sit↔stand,
    /// how long the rise on posture flag 0 gets before the gait takes over.
    pub unwind_s: Option<f64>,
    /// Seconds the network takes to reach its commanded posture after the flag flips. The
    /// sit↔stand is trained on a 2 s slewed target, so the seat is a ~2 s glide; the shutdown sit
    /// waits twice that before cutting torque.
    pub ramp_s: Option<f64>,
    /// Which drive mode this policy belongs to. Absent means walking. The roller crouch is the
    /// ground pick of `"roller"` mode, and the two must not be confused with each other.
    pub mode: Option<Mode>,
    /// The command block: how the daemon is meant to drive this network.
    pub command: Option<SetCommand>,
}

/// The machine-readable half of a manifest's command block.
///
/// Three encodings exist in the set, and `encoding` names which one this is:
///
/// - absent or `"constant"`: the skill family — a fixed twist for the window, `idle` on the way
///   back. Every kick, the roulade, and every community one-shot so far.
/// - `"phase"`: `[cos 2πφ, sin 2πφ, 0]` with φ advancing from 0 over `period_s` seconds and the
///   move handing back at `end_phase`. The ground pick, and the roller crouch.
/// - `"posture_flag"`: one slot carries `sit` or `stand`. The sit↔stand.
#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(default)]
pub struct SetCommand {
    pub encoding: Option<String>,
    /// The twist that means "stop doing the thing".
    pub idle: Option<[f64; 3]>,
    /// Phase encoding: seconds per full cycle.
    pub period_s: Option<f64>,
    /// Phase encoding: the fraction of the cycle at which the move hands back.
    /// A full-cycle pickup must finish its rise before handing back to HOME.
    pub end_phase: Option<f64>,
    /// Posture flag: the value that means "sit".
    pub sit: Option<f64>,
    /// Posture flag: the value that means "stand".
    pub stand: Option<f64>,
}

/// The ground pick's cycle: what a phase-encoded set entry declares.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhaseTiming {
    /// Seconds per full cycle.
    pub period_s: f64,
    /// Fraction of the cycle at which the move hands back.
    pub end_phase: f64,
    /// Action scale while it runs, if the entry says.
    pub action_scale: Option<f64>,
}

/// The sit↔stand's timing: what the scripted set entry declares.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SitStandTiming {
    /// Seconds the rise runs on the sitstand network before the gait takes over.
    pub rise_s: f64,
    /// Seconds the seat takes to settle after the flag flips.
    pub ramp_s: f64,
}

impl SetPolicy {
    pub fn skill_name(&self) -> String {
        self.name.clone().unwrap_or_else(|| {
            std::path::Path::new(&self.file)
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_else(|| self.file.clone())
        })
    }

    pub fn is_episodic(&self) -> bool {
        self.kind.as_deref() == Some("episodic")
    }

    pub fn is_scripted(&self) -> bool {
        self.kind.as_deref() == Some("scripted")
    }

    fn encoding(&self) -> Option<&str> {
        self.command.as_ref().and_then(|c| c.encoding.as_deref())
    }

    /// The daemon writes this policy's command as a phase. A `period_s` with no `encoding` is
    /// taken the same way — the field has no other meaning.
    pub fn is_phase(&self) -> bool {
        self.encoding() == Some("phase")
            || self.command.as_ref().is_some_and(|c| c.period_s.is_some())
    }

    /// The daemon writes this policy's command as a posture flag.
    pub fn is_posture_flag(&self) -> bool {
        self.encoding() == Some("posture_flag")
    }

    /// The mode this policy belongs to; absent means walking.
    pub fn mode(&self) -> Mode {
        self.mode.unwrap_or_default()
    }

    /// **An episodic policy on a constant command**: what becomes a skill on its own.
    ///
    /// A phase-encoded one does not — it is the ground pick, whose command the daemon generates,
    /// and loading it as a generic one-shot would feed it all-zeros: a robot moving plausibly and
    /// wrongly, which `duck_control::obs`'s header calls the hardest failure to see.
    pub fn is_zero_command_skill(&self) -> bool {
        self.is_episodic() && !self.is_phase() && !self.is_posture_flag()
    }

    /// The phase timing, for an episodic policy the daemon drives through a phase.
    pub fn phase_timing(&self) -> Option<PhaseTiming> {
        if !self.is_episodic() || !self.is_phase() {
            return None;
        }
        let command = self.command.as_ref()?;
        Some(PhaseTiming {
            period_s: command.period_s?,
            end_phase: command.end_phase.unwrap_or(DEFAULT_GROUND_PICK_END_PHASE),
            action_scale: self.action_scale,
        })
    }

    /// The sit↔stand timing, for the scripted posture-flag policy.
    pub fn sitstand_timing(&self) -> Option<SitStandTiming> {
        if !self.is_scripted() || !self.is_posture_flag() {
            return None;
        }
        Some(SitStandTiming {
            rise_s: self.unwind_s.unwrap_or(DEFAULT_SITSTAND_RISE_S),
            ramp_s: self.ramp_s.unwrap_or(DEFAULT_SITSTAND_RAMP_S),
        })
    }
}

impl SetManifest {
    /// The ground pick of one mode, if the set declares it: the phase-encoded episodic entry
    /// tagged with that mode (walking when untagged). The first one wins, and a set that lists
    /// two for the same mode has made a mistake this cannot see.
    pub fn ground_pick(&self, mode: Mode) -> Option<PhaseTiming> {
        self.policies
            .iter()
            .filter(|p| p.mode() == mode)
            .find_map(|p| p.phase_timing())
    }

    /// The sit↔stand's timing, if the set declares it. The sitstand is mode-independent — both
    /// presets load the same network — so the first scripted posture-flag entry is it.
    pub fn sitstand(&self) -> Option<SitStandTiming> {
        self.policies.iter().find_map(|p| p.sitstand_timing())
    }

    /// The entries that are skills on their own: episodic, constant-command, and not named as
    /// something the daemon drives itself.
    ///
    /// **A set cannot claim a name the daemon drives itself.** The ground pick and the sit toggle
    /// live in their own arm of the cascade, and a second entry answering to the same name would
    /// shadow one with a network fed an all-zero command it was never trained on. The manifest
    /// lives on the Hub and cannot be checked from here, so the guard belongs on the board.
    ///
    /// It guards the *name* and the *encoding*, not the file. A set that marks
    /// `alpha_ground_pick.onnx` episodic with neither a name nor a phase command still produces a
    /// skill — called `alpha_ground_pick`, running a phase-scripted network on zeros. That is a
    /// publisher's mistake rather than a trap: it shadows nothing, it is plainly visible in
    /// `robotctl policy list`, and nothing invokes it unless somebody asks for it by that name.
    /// Catching it would mean a hardcoded list of our own filenames, which is the coupling this
    /// whole manifest exists to remove.
    pub fn skills(&self) -> impl Iterator<Item = &SetPolicy> {
        self.policies
            .iter()
            .filter(|p| p.is_zero_command_skill())
            .filter(|p| !DAEMON_OWNED_SKILLS.contains(&p.skill_name().as_str()))
    }
}

/// Skill names the daemon implements itself, which nothing else may take over.
///
/// Public because there are three ways to add a skill — a set's manifest, `robotctl policy add`
/// and `robot.setSkill` — and a list only one of them checks is a guard for one of them. Each
/// has its own arm of the control cascade, so a table entry answering to either name is
/// unreachable: `robot.do` matches the built-in first, and the entry sits in the list being
/// offered and never run.
pub const DAEMON_OWNED_SKILLS: [&str; 2] = ["ground_pick", "sit_toggle"];

/// Complete the pickup cycle when neither the configuration nor the set declares an ending.
pub const DEFAULT_GROUND_PICK_END_PHASE: f64 = 1.0;
/// How long the sitstand network rises (posture flag 0) before the gait takes over, when the
/// set does not say. 1 s is enough on the robot — velstand owns the tail of the rise fine.
pub const DEFAULT_SITSTAND_RISE_S: f64 = 1.0;
/// How long the seat takes after the flag flips, when the set does not say: the ~2 s glide the
/// sit↔stand is trained on (`POSTURE_RAMP_S`).
pub const DEFAULT_SITSTAND_RAMP_S: f64 = 2.0;

/// Read the installed set's manifest, if it has one.
pub fn set_manifest() -> Option<SetManifest> {
    let text = std::fs::read_to_string(PathBuf::from(POLICY_DIR).join("manifest.json")).ok()?;
    serde_json::from_str(&text).ok()
}

/// The skills a robot has when its config says nothing.
///
/// From the installed set where it says, and from the three below where it does not.
///
/// A board whose set predates the manifest keeps its kicks and its roulade with no config
/// written and no migration run, which is the whole reason absence resolves to something rather
/// than nothing. This goes when every tagged set carries one.
fn builtin_skills(manifest: Option<&SetManifest>) -> Vec<SkillDef> {
    // What the set itself declares. A policy is a skill only if it says it is episodic, drives on
    // a constant command, and how long it runs — a gait is not something to ask for by name, a
    // perpetual one needs a hold length that only a person can choose, and a phase-encoded one
    // is the ground pick.
    if let Some(manifest) = manifest {
        let from_set: Vec<SkillDef> = manifest
            .skills()
            .filter_map(|p| {
                Some(SkillDef {
                    name: p.skill_name(),
                    path: Some(PathBuf::from(POLICY_DIR).join(&p.file)),
                    duration: p.duration_s?,
                    chain: p.chain,
                    unwind: p.command.as_ref().and_then(|c| c.idle).unwrap_or_default(),
                    unwind_s: p.unwind_s.unwrap_or(0.0),
                    params: SkillOverrides {
                        action_scale: p.action_scale,
                        ..Default::default()
                    },
                    ..Default::default()
                })
            })
            .collect();
        if !from_set.is_empty() {
            return from_set;
        }
    }

    fallback_skills()
}

/// The three skills every robot has had since the prototype, for a board whose set says nothing
/// about itself — and the timing a skill slot borrows when it names one the set left out.
fn fallback_skills() -> Vec<SkillDef> {
    let kick = |name: &str, file: &str| SkillDef {
        name: name.to_owned(),
        // The kick files are `ball_kick_*.onnx`, which is not `<name>.onnx` — the names are the
        // roles and the files are the training runs. The set's manifest keeps that indirection
        // in its own `name` field; this is the same thing for a set that predates it.
        path: Some(PathBuf::from(POLICY_DIR).join(file)),
        duration: 0.5,
        chain: false,
        command: [0.0; 3],
        unwind: [0.0; 3],
        unwind_s: 0.0,
        params: SkillOverrides::default(),
    };
    vec![
        // Order is priority, replacing the hardcoded `roulade > kick` precedence.
        SkillDef {
            name: "roulade".to_owned(),
            path: None,
            duration: 1.0,
            // Holding the button chains rolls, which is how the prototype maps a held trigger
            // onto a one-shot.
            chain: true,
            command: [0.0; 3],
            unwind: [0.0; 3],
            unwind_s: 0.0,
            params: SkillOverrides::default(),
        },
        kick("kick_left", "ball_kick_left.onnx"),
        kick("kick_right", "ball_kick_right.onnx"),
    ]
}

fn fallback_skill(name: &str) -> Option<SkillDef> {
    fallback_skills().into_iter().find(|s| s.name == name)
}

/// One policy slot, named — the seven `[policy]` path keys as a value rather than a field name.
///
/// It exists because three places now need to turn the string `"ground_pick"` into *that
/// particular key*: `robot.loadPolicy` on the wire, the `toml_edit` write `robotctl policy load`
/// performs, and the per-slot report `robot.policies` answers with. Spelling the mapping out in
/// each of them is how one of them ends up writing `policy.groundpick` to a file nobody reads
/// back until the robot will not walk.
///
/// [`Slot::as_str`] is the serde key, not a display name, and
/// [`tests::every_slot_is_a_registry_key`] is what keeps that true.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Slot {
    Walk,
    Stand,
    SitStand,
    GroundPick,
    KickLeft,
    KickRight,
    Roulade,
}

impl Slot {
    /// Every slot, in the order `[policy]` lists them and the order a report should print them.
    pub const ALL: [Slot; 7] = [
        Slot::Walk,
        Slot::Stand,
        Slot::SitStand,
        Slot::GroundPick,
        Slot::KickLeft,
        Slot::KickRight,
        Slot::Roulade,
    ];

    /// The slots that are one-shot skills: each names the `[[policy.skill]]` entry of the same
    /// name, and its path is what that skill runs. See `PolicyParams::resolved_skills`.
    pub const SKILLS: [Slot; 3] = [Slot::KickLeft, Slot::KickRight, Slot::Roulade];

    /// The serde key, exactly as `[policy]` spells it.
    pub fn as_str(self) -> &'static str {
        match self {
            Slot::Walk => "walk",
            Slot::Stand => "stand",
            Slot::SitStand => "sitstand",
            Slot::GroundPick => "ground_pick",
            Slot::KickLeft => "kick_left",
            Slot::KickRight => "kick_right",
            Slot::Roulade => "roulade",
        }
    }

    /// `section.key`, which is what the registry and `toml_edit` want.
    pub fn config_key(self) -> String {
        format!("policy.{}", self.as_str())
    }

    /// Parse a slot name off the wire. `None` for anything else, so a caller can refuse with
    /// the list of names it does know rather than failing to deserialize.
    pub fn parse(name: &str) -> Option<Slot> {
        Slot::ALL.into_iter().find(|s| s.as_str() == name)
    }

    /// Every slot name, for the "expected one of" half of a refusal.
    pub fn names() -> String {
        Slot::ALL
            .iter()
            .map(|s| s.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    }
}

impl std::fmt::Display for Slot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// `[policy]` with every absent field resolved against the mode's defaults.
///
/// This is what the rest of `robotd` consumes — nothing downstream should ever have to ask
/// "walk or roller?" to know the action scale.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedPolicy {
    pub enabled: bool,
    pub mode: Mode,
    /// The walking slot may be explicitly disabled for a posture-only robot.
    pub walk: Option<PathBuf>,
    pub stand: Option<PathBuf>,
    pub sitstand: Option<PathBuf>,
    pub ground_pick: Option<PathBuf>,
    pub kick_left: Option<PathBuf>,
    pub kick_right: Option<PathBuf>,
    pub roulade: Option<PathBuf>,
    pub action_scale: f64,
    pub standing_action_scale: f64,
    pub standing_gain_ratio: f64,
    pub gain: u16,
    pub head_lowpass: Option<f64>,
    pub legs_lowpass: Option<f64>,
    pub ground_pick_period: f64,
    /// Fraction of the cycle at which the ground pick hands back.
    pub ground_pick_end_phase: f64,
    pub ground_pick_action_scale: f64,
    pub ground_pick_gain_ratio: f64,
    /// Seconds the sitstand network rises before the gait takes over.
    pub sitstand_rise_s: f64,
    /// Seconds the seat takes to settle after the flag flips.
    pub sitstand_ramp_s: f64,
    /// The one-shot skills, config merged over the built-ins, in priority order.
    pub skills: Vec<SkillDef>,
    pub voltage_adapt: bool,
    pub nominal_voltage: f64,
}

impl ResolvedPolicy {
    /// The file that will actually be loaded into one slot, after mode defaults are applied.
    /// `None` means the slot is empty — a capability this robot does not have.
    pub fn slot(&self, slot: Slot) -> Option<&std::path::Path> {
        match slot {
            Slot::Walk => self.walk.as_deref(),
            Slot::Stand => self.stand.as_deref(),
            Slot::SitStand => self.sitstand.as_deref(),
            Slot::GroundPick => self.ground_pick.as_deref(),
            Slot::KickLeft => self.kick_left.as_deref(),
            Slot::KickRight => self.kick_right.as_deref(),
            Slot::Roulade => self.roulade.as_deref(),
        }
    }
}

impl PolicyParams {
    /// The built-in skills with config merged over them, by name.
    ///
    /// Merge rather than replace, for the reason every other key in this file resolves the way it
    /// does: the file is a list of decisions, not a copy of the defaults. Adding `polite-bow` is
    /// one entry and does not mean re-declaring the three that were already there — and forgetting
    /// to re-declare one cannot silently remove it, which is the failure mode of the other rule.
    ///
    /// A named skill keeps the built-in's position in the priority order; a new one goes last.
    ///
    /// **The three skill slots are applied last and win.** `kick_left`, `kick_right` and `roulade`
    /// are `[policy]` keys like `walk`, and `robotctl policy load roulade <file>` writes that key —
    /// so the file it names has to be the one the `roulade` skill runs, or the load reports a
    /// file the robot never touches, which is what it did. A slot naming a skill the set does not
    /// declare adds it with the built-in's timing; `"none"` switches it off, as it does for a
    /// `[[policy.skill]]` entry.
    pub fn resolved_skills(&self) -> Vec<SkillDef> {
        self.resolved_skills_with(set_manifest().as_ref())
    }

    /// [`Self::resolved_skills`] against a manifest already read — or none, which is the
    /// fallback three.
    pub fn resolved_skills_with(&self, manifest: Option<&SetManifest>) -> Vec<SkillDef> {
        let mut resolved = builtin_skills(manifest);
        for configured in &self.skills {
            match resolved.iter_mut().find(|s| s.name == configured.name) {
                Some(builtin) => *builtin = configured.clone(),
                None => resolved.push(configured.clone()),
            }
        }
        for slot in Slot::SKILLS {
            let Some(path) = self.slot(slot) else {
                continue;
            };
            match resolved.iter_mut().find(|s| s.name == slot.as_str()) {
                Some(skill) => skill.path = Some(path.clone()),
                None => {
                    if let Some(mut skill) = fallback_skill(slot.as_str()) {
                        skill.path = Some(path.clone());
                        resolved.push(skill);
                    }
                }
            }
        }
        // A skill whose path is the `"none"` sentinel is switched off, which is how a built-in is
        // removed without a second mechanism for it.
        resolved.retain(|s| s.resolved_path().is_some());
        resolved
    }

    /// What config says about one slot: `None` unset (resolve the mode's default), `Some("none")`
    /// disabled outright, `Some(path)` an override.
    pub fn slot(&self, slot: Slot) -> &Option<PathBuf> {
        match slot {
            Slot::Walk => &self.walk,
            Slot::Stand => &self.stand,
            Slot::SitStand => &self.sitstand,
            Slot::GroundPick => &self.ground_pick,
            Slot::KickLeft => &self.kick_left,
            Slot::KickRight => &self.kick_right,
            Slot::Roulade => &self.roulade,
        }
    }

    /// Set or clear one slot's override. Clearing is what `robotctl policy reset` does, and it
    /// is why this takes an `Option` rather than having a second method for it.
    pub fn set_slot(&mut self, slot: Slot, path: Option<PathBuf>) {
        let field = match slot {
            Slot::Walk => &mut self.walk,
            Slot::Stand => &mut self.stand,
            Slot::SitStand => &mut self.sitstand,
            Slot::GroundPick => &mut self.ground_pick,
            Slot::KickLeft => &mut self.kick_left,
            Slot::KickRight => &mut self.kick_right,
            Slot::Roulade => &mut self.roulade,
        };
        *field = path;
    }

    pub fn resolved(&self) -> ResolvedPolicy {
        self.resolved_with(set_manifest().as_ref())
    }

    /// [`Self::resolved`] against a manifest already read — or none, which is what a board whose
    /// set predates the manifest has, and resolves to the prototype's numbers.
    ///
    /// **The set says how its own policies run.** The ground pick's cycle and the sit↔stand's rise
    /// used to be literals here, per mode, which meant a retrained pick with a longer cycle was a
    /// daemon release. They come from the set's phase-encoded and posture-flag entries now; the
    /// literals stay as the fallback, and a `[policy]` key still overrides either, because the
    /// file is the list of a person's decisions.
    pub fn resolved_with(&self, manifest: Option<&SetManifest>) -> ResolvedPolicy {
        let release = |name: &str| PathBuf::from(POLICY_DIR).join(name);
        let path = |field: &Option<PathBuf>, default: Option<&str>| -> Option<PathBuf> {
            match field {
                Some(p) if is_none_sentinel(p) => None,
                Some(p) => Some(p.clone()),
                None => default.map(release),
            }
        };

        let (walk_default, stand, sitstand, ground_pick) = match self.mode {
            // The velstand gait (set v5) walks on a twist and stands still at zero command,
            // so no standing network is loaded by default: with `stand` unset the walking
            // policy runs at every velocity. `alpha_walking.onnx` + `alpha_stand.onnx` stay
            // in the set for a board that loads them back by hand.
            Mode::Walk => (
                "velstand.onnx",
                None,
                Some("alpha_sitstand.onnx"),
                Some("alpha_ground_pick.onnx"),
            ),
            // The prototype's roller preset, since rebased on the alpha defaults: roller
            // policy, crouch on the ground-pick trigger, and everything else — sit/stand,
            // kicks, the trained low-pass — as the walking mode has it. `stand` stays
            // unloaded, deliberately: the prototype loads the standing network in roller
            // mode and then skips every standing transition while `roller_mode` is set, so
            // it never runs — not loading it is the same robot without the dead session.
            Mode::Roller => (
                "roller.onnx",
                None,
                Some("alpha_sitstand.onnx"),
                Some("roller_crouch.onnx"),
            ),
        };

        // What each skill slot reports is what the skill of that name will run — derived from the
        // list rather than resolved beside it, so `robot.policies` cannot name a file the robot is
        // not running.
        let skills = self.resolved_skills_with(manifest);
        let skill_file = |name: &str| {
            skills
                .iter()
                .find(|s| s.name == name)
                .and_then(|s| s.resolved_path())
        };

        // The set's own timing for the two networks the daemon drives itself, for this mode.
        let pick = manifest.and_then(|m| m.ground_pick(self.mode));
        let seat = manifest.and_then(|m| m.sitstand());

        ResolvedPolicy {
            enabled: self.enabled,
            mode: self.mode,
            // An explicit `walk = "none"` is a supported posture-only configuration. An
            // omitted value still resolves to the mode's walking default.
            walk: path(&self.walk, Some(walk_default)),
            stand: path(&self.stand, stand),
            sitstand: path(&self.sitstand, sitstand),
            ground_pick: path(&self.ground_pick, ground_pick),
            kick_left: skill_file("kick_left"),
            kick_right: skill_file("kick_right"),
            roulade: skill_file("roulade"),
            action_scale: self.action_scale.unwrap_or(match self.mode {
                Mode::Walk => 0.9,
                Mode::Roller => 0.8,
            }),
            standing_action_scale: self.standing_action_scale,
            standing_gain_ratio: self.standing_gain_ratio,
            gain: self.gain,
            head_lowpass: Some(self.head_lowpass.unwrap_or(0.5)).filter(|a| *a < 1.0),
            legs_lowpass: Some(self.legs_lowpass.unwrap_or(0.7)).filter(|a| *a < 1.0),
            ground_pick_period: self
                .ground_pick_period
                .or(pick.map(|t| t.period_s))
                .unwrap_or(match self.mode {
                    Mode::Walk => 4.0,
                    Mode::Roller => 3.0,
                }),
            ground_pick_end_phase: self
                .ground_pick_end_phase
                .or(pick.map(|t| t.end_phase))
                .unwrap_or(DEFAULT_GROUND_PICK_END_PHASE),
            ground_pick_action_scale: self
                .ground_pick_action_scale
                .or(pick.and_then(|t| t.action_scale))
                .unwrap_or(match self.mode {
                    Mode::Walk => 1.0,
                    Mode::Roller => 0.8,
                }),
            ground_pick_gain_ratio: self.ground_pick_gain_ratio,
            sitstand_rise_s: seat.map_or(DEFAULT_SITSTAND_RISE_S, |t| t.rise_s),
            sitstand_ramp_s: seat.map_or(DEFAULT_SITSTAND_RAMP_S, |t| t.ramp_s),
            skills,
            voltage_adapt: self.voltage_adapt,
            nominal_voltage: self.nominal_voltage,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct SafetyParams {
    /// Projected-gravity z above which the robot counts as going down. Upright is about
    /// -1.0; on its side is near 0.
    pub fall_gravity_z: f64,
    /// How long that has to hold. Debounced so a firm footfall is not a fall.
    pub fall_debounce_ms: u64,
    /// Intent age past which the velocity is zeroed. Stop, not limp.
    pub deadman_ms: u64,
    /// The gain limp-fall yields at — low enough to give way rather than fight the floor.
    pub gain_limp: u16,
    /// Sit down and power the machine off when the battery EMA reaches the empty floor
    /// (6.6 V — `duck_control::model::BATTERY_EMPTY_V`). The EMA moves over ~10 s, so a
    /// load sag cannot trip it.
    pub battery_empty_shutdown: bool,

    /// Go limp *while falling*, to land soft instead of fighting the floor all the way
    /// down. **Off by default** since the velstand gait became the default walk (set v5):
    /// the hand-back it ends with is to the standing network, which the default configuration
    /// no longer loads. Turn it on with a robot that runs a standing policy.
    ///
    /// The only thing the daemon does about a fall. Drop to `gain_limp`, let the robot
    /// collapse, pose it back to standing once it has landed, then hand it to the standing
    /// policy — which stands up far more cleanly from a still robot than from one that has
    /// been thrashing since the fall began. With it off, a fall changes nothing: the policy
    /// keeps driving and the humans stay in charge.
    pub limp_fall: bool,
    /// Projected-gravity z the robot must already be past before a fall prediction counts
    /// — about 26° of tilt, which ordinary walking does not reach.
    pub limp_fall_tilt_z: f64,
    /// Where the extrapolation must reach to count as falling. Same sense as
    /// `fall_gravity_z`, and by default the same number.
    pub limp_fall_predict_z: f64,
    /// How far ahead the tilt rate is extrapolated.
    pub limp_fall_lookahead_ms: u64,
    /// How long the fall verdict must hold before the gains drop. Three ticks at 50 Hz —
    /// longer than a footfall impulse, short enough to leave most of the fall to limp
    /// through.
    pub limp_fall_debounce_ms: u64,
    /// Angular-rate magnitude below which the robot counts as having landed, rad/s.
    pub limp_fall_still_rate: f64,
    /// How long it has to stay that still before the limp ends.
    pub limp_fall_still_ms: u64,
    /// Hard cap on the limp, however the landing reads. A robot that never goes still —
    /// held in someone's hands, or resting against something that keeps nudging it —
    /// must not stay limp forever.
    pub limp_fall_max_ms: u64,
    /// How long the ramp back to the standing pose takes, once the robot has landed.
    /// 0.6 s — settled on at the robot. The joints travel across the floor unloaded rather
    /// than lifting anything, so a full second was mostly dead time before the stand-up;
    /// 0.6 keeps some margin over the 0.3 that also worked.
    pub limp_fall_pose_ms: u64,
    /// Gain for that ramp. The joints have to actually travel across the floor, so it is
    /// not the limp gain; it is the softened standing gain rather than the walking one.
    pub limp_fall_pose_gain: u16,
}

impl Default for PolicyParams {
    fn default() -> Self {
        Self {
            enabled: true,
            mode: Mode::Walk,
            walk: None,
            stand: None,
            sitstand: None,
            ground_pick: None,
            kick_left: None,
            kick_right: None,
            roulade: None,
            action_scale: None,
            standing_action_scale: 1.0,
            // The prototype's `--standing-kp-ratio`.
            standing_gain_ratio: 0.8,
            gain: 200,
            head_lowpass: None,
            legs_lowpass: None,
            ground_pick_period: None,
            ground_pick_end_phase: None,
            ground_pick_action_scale: None,
            ground_pick_gain_ratio: 1.0,
            skills: Vec::new(),
            voltage_adapt: true,
            nominal_voltage: 7.4,
        }
    }
}

impl Default for SafetyParams {
    fn default() -> Self {
        Self {
            fall_gravity_z: -0.5,
            fall_debounce_ms: 200,
            deadman_ms: 500,
            gain_limp: 50,
            battery_empty_shutdown: true,
            limp_fall: false,
            limp_fall_tilt_z: -0.90,
            limp_fall_predict_z: -0.5,
            limp_fall_lookahead_ms: 300,
            limp_fall_debounce_ms: 60,
            limp_fall_still_rate: 1.0,
            limp_fall_still_ms: 200,
            limp_fall_max_ms: 1500,
            limp_fall_pose_ms: 600,
            limp_fall_pose_gain: 160,
        }
    }
}

/// Serial protocol used by the motor/IMU bus.
///
/// `dynamixel` is the shipped XL330 path. `hx` selects the HX broadcast adapter; it requires
/// HX firmware that implements sync read/write and an IMU exposing the shared 12-byte block.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum BusProtocol {
    Dynamixel,
    Hx,
}

impl Default for BusProtocol {
    fn default() -> Self {
        Self::Dynamixel
    }
}

impl BusProtocol {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Dynamixel => "dynamixel",
            Self::Hx => "hx",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Bus {
    /// Wire protocol for the shared serial bus.
    pub protocol: BusProtocol,
    /// Native running P on HX when limp_fall is enabled; independent of policy.gain.
    /// An explicit baseline also restores normal P after a daemon restart during a fall.
    pub position_p_gain: u8,
    /// Serial port the servos and the IMU board share. The Radxa Zero 3W wires them to
    /// `/dev/ttyS2`.
    pub port: String,
    /// Read the bus with fast sync read (protocol 2.0 instruction 0x8A) rather than a plain
    /// sync read: the sixteen devices append their blocks to one status packet instead of
    /// each sending its own, which is fifteen packet headers and fifteen bus turnarounds off
    /// every tick.
    ///
    /// On by default, because that is what this robot's hardware does and a setting nobody
    /// has to find is worth more than a saving nobody gets. It is a setting rather than a
    /// constant because the instruction is a property of *firmware* — XL330 v46 or newer, and
    /// the `imu_to_dxl` board has to implement it too — so a board built before either is the
    /// one case this code cannot talk its way out of. Turning it off is the whole remedy.
    ///
    /// The symptom of getting it wrong is unambiguous, which is why the default can be the
    /// brave one: a device that does not implement 0x8A does not answer at all, so *every*
    /// read times out rather than some of them returning something plausible. The loop reports
    /// the bus drops, `update_gate` sees an unhealthy robot, and a release that turned this on
    /// against firmware that cannot do it is rolled back on its own.
    pub fast_sync_read: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Control {
    /// Control loop rate. 50 Hz is inherited from the prototype, where it was chosen on a
    /// Pi Zero 2W — re-derive it on the Radxa rather than trusting it.
    pub hz: u32,
    /// Per-tick EMA on the velocity command: `cmd += α × (target − cmd)`. The prototype's
    /// `--cmd-alpha` — what turns a stick snap into a ramp the gait can follow. `1.0` is
    /// pass-through.
    pub cmd_alpha: f64,
    /// Same, for head targets and the body pose.
    pub head_alpha: f64,
    /// Whether the state stream carries measured joint velocity and load
    /// (`RobotState::velocities`, `RobotState::currents_ma`).
    ///
    /// On. The servos report both in the same twelve-byte read as position, so the robot pays
    /// nothing on the bus for them, and present current is its only measure of external force —
    /// a foot taking weight, a hand on the beak, a servo on its way to an overload shutdown.
    ///
    /// Here, on the control loop, because the control loop is what assembles the frame. What it
    /// costs is frame size: about 10–12 % of a full frame, which is +15 to +19 KB/s at 50 Hz to
    /// each subscriber that asked for every tick. Off is the way out if that turns out to matter
    /// on a particular robot, or if a consumer turns out to mishandle the fields — and off means
    /// *absent*, the same thing an older daemon sends, not zeros. Nothing in this daemon reads
    /// them back, so turning it off costs the robot nothing but the information.
    pub publish_velocity_and_load: bool,
}

/// Thresholds that decide `healthy` — and therefore whether an update is kept.
///
/// **Not** the thresholds for everything `robot.health` reports. That answer also describes the
/// battery, the motor temperatures and the loop counters, and none of those may reach a verdict
/// — so none of them has a setting here. Naming this section
/// `[health]` invited exactly that mistake: it reads like "how the robot is doing", when what it
/// configures is the one question auto-rollback turns on.
///
/// Everything here is a property of the *software*. A future `[thermal]` section for a motor
/// temperature that should throttle the robot would be a different thing, and belongs under a
/// different name.
///
/// The section was called `[health]`. Renamed outright rather than aliased: a board carrying
/// the old name gets a parse error naming the section, which is a better outcome than a robot
/// quietly running on default thresholds nobody chose.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct UpdateGate {
    /// Below this achieved rate the robot reports unhealthy, which is what makes the
    /// updater's auto-rollback mean something. A loop running at 60% of target is alive,
    /// answers every request, and is badly broken.
    pub min_achieved_hz: f64,
    /// How many periods may pass with no tick before the loop counts as **wedged**.
    ///
    /// This detects a dead loop, not a slow one — `min_achieved_hz` owns degradation. Keep
    /// the two apart: set this near the period and it fires on ordinary scheduler jitter,
    /// which on a loaded board would report a perfectly good release unhealthy and roll it
    /// back. A loop that has not ticked in half a second is genuinely gone; one that
    /// ticked 80 ms late is just late.
    pub stall_periods: u32,
    /// Consecutive bus read failures tolerated before reporting unhealthy. One dropped
    /// transaction is ordinary; a run of them means the bus is gone.
    pub max_consecutive_errors: u32,
}

impl Default for Bus {
    fn default() -> Self {
        Self {
            protocol: BusProtocol::Dynamixel,
            position_p_gain: 15,
            port: "/dev/ttyS2".into(),
            fast_sync_read: true,
        }
    }
}

impl Default for Control {
    fn default() -> Self {
        Self {
            hz: 50,
            cmd_alpha: 0.2,
            head_alpha: 0.2,
            publish_velocity_and_load: true,
        }
    }
}

impl Default for UpdateGate {
    fn default() -> Self {
        Self {
            // 90% of the default rate. Generous enough not to trip on a slow tick, tight
            // enough that a loop losing every tenth cycle is not called healthy.
            min_achieved_hz: 45.0,
            // 500 ms at the default rate. Deliberately far from the period: three periods
            // is 60 ms, which ordinary scheduler jitter exceeds on a busy machine, and a
            // health check that trips on jitter rolls back good releases.
            stall_periods: 25,
            max_consecutive_errors: 10,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ParamsError {
    #[error("{path}: idle.{key} {reason}")]
    Idle {
        path: String,
        key: &'static str,
        reason: &'static str,
    },
    #[error("{path}: {reason}")]
    FallPositionGain {
        path: String,
        reason: &'static str,
    },
    #[error("reading {path}: {source}")]
    Read {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("parsing {path}: {source}")]
    Parse {
        path: String,
        #[source]
        source: toml::de::Error,
    },
    #[error("{path}: control.hz must be between 1 and 1000, got {got}")]
    Rate { path: String, got: u32 },
    #[error("{path}: policy.ground_pick_end_phase must be finite and in (0, 1], got {got}")]
    GroundPickEndPhase { path: String, got: f64 },
    #[error(
        "{path}: media.bitrate must be between {min} and {max} bits per second, got {got} — \
         the unit is bits, so 2 Mb/s is 2000000"
    )]
    Bitrate {
        path: String,
        got: u32,
        min: u32,
        max: u32,
    },
    #[error(
        "{path}: pad_drive.{axis}_min must be zero or negative and pad_drive.{axis}_max zero or \
         positive, got {min} and {max} — the bounds are signed, so full stick back at 0.2 m/s is \
         vx_min = -0.2"
    )]
    PadDrive {
        path: String,
        axis: &'static str,
        min: f64,
        max: f64,
    },
}

/// The band `media.bitrate` is accepted in, bits per second.
///
/// The floor is not taste: it is where a typo lands. `bitrate = 2000` is somebody who meant
/// kilobits, and 2 kb/s is a stream that never produces a picture — far better refused at the
/// editor than debugged off a board. The ceiling is what the link and the VPU are for; above it
/// the encoder is being asked for something no robot's wifi will carry.
pub const BITRATE_MIN: u32 = 100_000;
pub const BITRATE_MAX: u32 = 20_000_000;

impl Params {
    /// Native gains only for the HX fall sequence. Ordinary policy selection (including
    /// Stand's 0.8 ratio) must not lower the native running gain.
    pub fn fall_position_gains(&self) -> Result<Option<FallPositionGains>, &'static str> {
        if self.bus.protocol != BusProtocol::Hx || !self.safety.limp_fall {
            return Ok(None);
        }
        if self.bus.position_p_gain == 0 || self.policy.gain == 0 {
            return Err("HX limp_fall requires nonzero bus.position_p_gain and policy.gain");
        }
        let scaled = |gain: u16| {
            let value = (f64::from(self.bus.position_p_gain) * f64::from(gain)
                / f64::from(self.policy.gain))
            .round();
            if !(1.0..=255.0).contains(&value) {
                return Err("HX limp_fall gain ratios must round into 1..=255");
            }
            Ok(value as u8)
        };
        Ok(Some(FallPositionGains {
            running: self.bus.position_p_gain,
            limp: scaled(self.safety.gain_limp)?,
            posing: scaled(self.safety.limp_fall_pose_gain)?,
        }))
    }

    /// Load from `path`. A missing file at the *default* location is not an error — an
    /// unprovisioned board should still come up on defaults rather than refuse to start,
    /// and a daemon that will not start is much harder to diagnose remotely than one
    /// running on known defaults. A file explicitly named on the command line must exist.
    pub fn load(path: &Path, explicit: bool) -> Result<Self, ParamsError> {
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound && !explicit => {
                tracing::warn!(path = %path.display(), "no params file; using defaults");
                return Ok(Self::default());
            }
            Err(source) => {
                return Err(ParamsError::Read {
                    path: path.display().to_string(),
                    source,
                });
            }
        };

        // Strict first. A file this build fully understands parses exactly as it always has —
        // one pass, serde's own spans, no second guess — and that is the overwhelmingly common
        // case. The lenient path below only ever runs on a file that has already failed.
        let params = match toml::from_str::<Params>(&text) {
            Ok(params) => params,
            // Not an unknown-key problem: a syntax error, or a value of the wrong type. The
            // strict error is what gets reported, because it is the one carrying a line and a
            // column.
            Err(source) => {
                let Some((reparsed, ignored)) = without_unknown_keys(&text) else {
                    return Err(ParamsError::Parse {
                        path: path.display().to_string(),
                        source,
                    });
                };
                tracing::warn!(
                    path = %path.display(),
                    ignored = %ignored.join(", "),
                    "this build has no such keys; they are ignored and their values do nothing"
                );
                // Pruned, and what is left still does not parse — a real error sharing a file
                // with an inert one. This reports the real one, which costs the position: serde
                // is now looking at a table rather than at the text. The alternative was worse.
                // Returning the strict error here would name the key this release just declared
                // harmless as the reason the daemon will not start, and send whoever reads it to
                // delete a section that was never the problem.
                reparsed.map_err(|source| ParamsError::Parse {
                    path: path.display().to_string(),
                    source,
                })?
            }
        };
        params.validate(path)?;
        Ok(params)
    }

    /// Reject values that would produce a loop that cannot work, at startup rather than as
    /// a division by zero three seconds later.
    fn validate(&self, path: &Path) -> Result<(), ParamsError> {
        if let Some((key, reason)) = self.idle.invalid() {
            return Err(ParamsError::Idle {
                path: path.display().to_string(),
                key,
                reason,
            });
        }
        self.fall_position_gains()
            .map_err(|reason| ParamsError::FallPositionGain {
                path: path.display().to_string(),
                reason,
            })?;
        if let Some(phase) = self.policy.ground_pick_end_phase
            && !(phase.is_finite() && phase > 0.0 && phase <= 1.0)
        {
            return Err(ParamsError::GroundPickEndPhase {
                path: path.display().to_string(),
                got: phase,
            });
        }
        if self.control.hz == 0 || self.control.hz > 1000 {
            return Err(ParamsError::Rate {
                path: path.display().to_string(),
                got: self.control.hz,
            });
        }
        // Checked here rather than in `mediad`, so `robotctl configure` refuses to write it:
        // the daemon that would choke on this one is not the daemon whose gate the editor runs.
        if let Some(bitrate) = self.media.bitrate
            && !(BITRATE_MIN..=BITRATE_MAX).contains(&bitrate)
        {
            return Err(ParamsError::Bitrate {
                path: path.display().to_string(),
                got: bitrate,
                min: BITRATE_MIN,
                max: BITRATE_MAX,
            });
        }
        // Signed bounds, so a positive `_min` is somebody who wrote a magnitude: full stick
        // back would then walk the robot *forward*. Refused rather than taken as its absolute
        // value, because the editor should say which of the two readings it was not going to
        // guess.
        for (axis, min, max) in self.pad_drive.axes() {
            if !(min.is_finite() && max.is_finite() && min <= 0.0 && max >= 0.0) {
                return Err(ParamsError::PadDrive {
                    path: path.display().to_string(),
                    axis,
                    min,
                    max,
                });
            }
        }
        Ok(())
    }

    pub fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(1.0 / self.control.hz as f64)
    }
}

/// Verified native values for the three phases, without changing the legacy gain scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FallPositionGains {
    pub running: u8,
    pub limp: u8,
    pub posing: u8,
}

/// Re-parse a file that `deny_unknown_fields` rejected, dropping the keys this build has no
/// place for, and say which they were. `None` when nothing was dropped — the parse failed for
/// some other reason and the caller should report that instead.
///
/// **Why unknown keys are no longer fatal.** They were, and the reasoning was sound as far as it
/// went: a silently ignored `min_acheived_hz` leaves an operator believing they moved a threshold
/// they did not. What that argument missed is the other way a key becomes unknown — the build
/// changed underneath a file nobody typed into. A robot running a branch had `[chorale]` in its
/// `robotd.toml`; updating it to a `main` without that feature produced a `robotd` that would not
/// start, four consecutive rollbacks, and a bench session spent on a robot that was fine. The
/// section was inert. Refusing to run over it is a far larger penalty than the mistake it guards
/// against, and it lands on exactly the transitions — a downgrade, a branch, a release that
/// dropped a feature — where the operator did nothing wrong at all.
///
/// So the value is kept and the enforcement moved: every dropped key is named at `warn`, and
/// `robotctl configure` writes only keys the registry knows. What is gone is a robot that will
/// not walk because of a line in a config file that does nothing.
///
/// **The registry is the authority on what a key is**, not serde. It has to be: serde's answer
/// arrives as prose inside an error, and this needs the question asked per key. That is safe
/// because it is not a second copy of the schema —
/// [`registry::tests::the_registry_covers_every_key_exactly`] pins it to [`Params`] in both
/// directions, so a key the registry does not know is a key `Params` does not have.
///
/// `deny_unknown_fields` stays on the structs. It is what makes that test possible, and it is
/// the backstop here: if the registry ever did drift, the pruned table would still be rejected
/// rather than quietly deserialised into something else.
#[allow(clippy::type_complexity)]
fn without_unknown_keys(text: &str) -> Option<(Result<Params, toml::de::Error>, Vec<String>)> {
    let mut table: toml::Table = text.parse().ok()?;
    let mut ignored: Vec<String> = Vec::new();

    table.retain(|section, value| {
        let Some(fields) = value.as_table_mut() else {
            // A bare value at the top level — `hz = 50` written outside any section, which is
            // the shape a hand-edit takes when someone forgets the header. No registry key can
            // name it, and reporting it by its bare name is what tells them why.
            ignored.push(section.to_string());
            return false;
        };
        if !registry::has_section(section) {
            // Reported as the section rather than as each of its keys: `[chorale]` is one
            // decision someone made, not four mistakes.
            ignored.push(format!("[{section}]"));
            return false;
        }
        fields.retain(|key, _| {
            if registry::entry_for(&format!("{section}.{key}")).is_some() {
                true
            } else {
                ignored.push(format!("{section}.{key}"));
                false
            }
        });
        true
    });

    if ignored.is_empty() {
        // Nothing here was an unknown key, so the strict parse failed for a reason this cannot
        // help with. `None` says exactly that, and keeps the caller's two cases apart.
        return None;
    }
    ignored.sort();
    Some((toml::Value::Table(table).try_into::<Params>(), ignored))
}
