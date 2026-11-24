//! The machine-readable index of every key in `robotd.toml` — what an editor needs to know.
//!
//! [`Params`] and its `Default` impls are the truth about *values*; what they cannot carry at
//! runtime is the part a human-facing editor needs — a one-line description, what kind of value
//! a key takes, which named choices an enum offers. That is this table.
//!
//! **A table this shape usually drifts, so its completeness is a test, not a hope**:
//! [`tests::the_registry_covers_every_key_exactly`] serializes `Params::default()` and walks the
//! key tree — every leaf must appear here and nothing else may. Add a section to [`Params`] and
//! the build stays green but the test names the keys the registry is missing; the editor can
//! never silently not know about a setting. (The same trick that pins `deploy/robotd.toml` to
//! the defaults.)
//!
//! The one-line docs are deliberately *short* — the full reasoning lives in the doc comments on
//! [`Params`] and in the shipped `deploy/robotd.toml`, and an editor's footer is not the place
//! for four paragraphs on voltage adaptation.

/// What kind of value a key takes — what an editor should offer for it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// On or off. Toggle.
    Bool,
    /// On, off, or absent-meaning-auto (`Option<bool>` — e.g. pet detection resolves per
    /// mode when unset). Cycle through the three.
    TriBool,
    /// A whole number.
    Integer,
    /// A fractional number.
    Float,
    /// A fractional number, or absent meaning "resolved per mode / measured / default".
    OptionalFloat,
    /// A whole number, or absent meaning "follows something else" — `media.bitrate` follows
    /// the quality. Editors show what it resolves to.
    OptionalInteger,
    /// One of a fixed set of names.
    Choice(&'static [&'static str]),
    /// Free text (an ALSA device, a socket path...).
    Text,
    /// A filesystem path, or absent meaning this robot's own copy; the literal `"none"`
    /// disables the slot outright.
    OptionalPath,
    /// A list of whole numbers, edited as comma-separated text ("4, 5, 9").
    IntegerList,
    /// A **repeating table** — `[[policy.skill]]` — rather than a single value.
    ///
    /// Listed here and not editable in place. Not an oversight and not laziness: every other
    /// kind is one value with one cursor position, and a repeating table is a list a person adds
    /// to, removes from and reorders. Rendering that inside a key/value editor would be a worse
    /// tool than the commands that already do it — `robotctl policy` — which the doc line points
    /// at.
    ///
    /// It is *in* the registry so the completeness test keeps meaning what it says: a section
    /// this editor cannot edit is still a section it must know exists, or the next repeating
    /// table added to `Params` goes unnoticed.
    Table,
    /// A **nested table of related values** — `[media.intrinsics]` — written by a tool rather
    /// than typed.
    ///
    /// Distinct from [`Kind::Table`], which is a repeating one, and distinct from every scalar
    /// kind for the same reason as that: it has no single cursor position. It is also not a thing
    /// anybody should type — six numbers from a calibration, where a typo produces a plausible
    /// wrong answer rather than an error — so the editor lists it and says what writes it.
    ///
    /// The string is a TOML body for the table, and it earns its place in the type rather than in
    /// a comment: the completeness test uses it to prove the key parses, so a record whose fields
    /// are renamed under it fails here instead of at a robot's next boot.
    Record(&'static str),
}

/// One key of `robotd.toml`.
#[derive(Debug, Clone, Copy)]
pub struct Entry {
    /// `section.key`, exactly as serde spells it.
    pub key: &'static str,
    pub kind: Kind,
    /// One line, for an editor's footer. The paragraph lives on the struct field.
    pub doc: &'static str,
    /// A *feature switch* — the handful of keys someone opens an editor to flip, as opposed
    /// to tuning they should read the full docs before touching. Editors list these first.
    pub feature: bool,
}

const fn entry(key: &'static str, kind: Kind, doc: &'static str) -> Entry {
    Entry {
        key,
        kind,
        doc,
        feature: false,
    }
}

const fn feature(key: &'static str, kind: Kind, doc: &'static str) -> Entry {
    Entry {
        key,
        kind,
        doc,
        feature: true,
    }
}

/// Every key, grouped by section, sections in the shipped file's order.
pub const REGISTRY: &[Entry] = &[
    // ── [bus] ────────────────────────────────────────────────────────────────
    entry(
        "bus.protocol",
        Kind::Choice(&["dynamixel", "hx"]),
        "Bus protocol: dynamixel or hx",
    ),
    entry(
        "bus.position_p_gain",
        Kind::Integer,
        "HX native running P for limp_fall (default 15); temporary register changes only",
    ),
    entry(
        "bus.port",
        Kind::Text,
        "Serial port device for the selected bus protocol",
    ),
    // Not a feature switch, though it is a `Bool`: the front page is "what does this robot
    // do", and this is "what does this robot's firmware understand". It belongs beside the
    // serial port, with the other thing you set once per board and then forget.
    entry(
        "bus.fast_sync_read",
        Kind::Bool,
        "Dynamixel fast sync read: needs XL330 firmware v46+; unused by HX",
    ),
    // ── [control] ────────────────────────────────────────────────────────────
    entry("control.hz", Kind::Integer, "Control loop rate"),
    entry(
        "control.cmd_alpha",
        Kind::Float,
        "EMA smoothing on stick twist, 1.0 = pass-through",
    ),
    entry(
        "control.head_alpha",
        Kind::Float,
        "EMA smoothing on head targets, 1.0 = pass-through",
    ),
    entry(
        "control.publish_velocity_and_load",
        Kind::Bool,
        "Put measured joint velocity and load on the state stream",
    ),
    // ── [update_gate] ────────────────────────────────────────────────────────
    entry(
        "update_gate.min_achieved_hz",
        Kind::Float,
        "Loop rate below which an update is refused as unhealthy",
    ),
    entry(
        "update_gate.stall_periods",
        Kind::Integer,
        "Periods with no tick at all before the loop counts as wedged",
    ),
    entry(
        "update_gate.max_consecutive_errors",
        Kind::Integer,
        "Consecutive bus read failures before reporting unhealthy",
    ),
    // ── [policy] ─────────────────────────────────────────────────────────────
    feature(
        "policy.enabled",
        Kind::Bool,
        "Load policies at all — off holds the pose and stays healthy (bench)",
    ),
    feature(
        "policy.mode",
        Kind::Choice(&["walk", "roller"]),
        "Legs or the roller: picks policies and tuning. Held DPad-Up switches it live; this is \
         the mode a reboot comes back in",
    ),
    entry(
        "policy.skill",
        Kind::Table,
        "One-shot skills, in priority order — add and remove with `robotctl policy`",
    ),
    entry(
        "policy.walk",
        Kind::OptionalPath,
        "Walking policy; unset = this robot's own",
    ),
    entry(
        "policy.stand",
        Kind::OptionalPath,
        "Standing policy; unset = this robot's own",
    ),
    entry(
        "policy.sitstand",
        Kind::OptionalPath,
        "Sit/stand policy; sit toggle and shutdown sit need it",
    ),
    entry(
        "policy.ground_pick",
        Kind::OptionalPath,
        "Ground-pick policy (roller: the crouch)",
    ),
    entry("policy.kick_left", Kind::OptionalPath, "Left-kick policy"),
    entry("policy.kick_right", Kind::OptionalPath, "Right-kick policy"),
    entry("policy.roulade", Kind::OptionalPath, "Forward-roll policy"),
    entry(
        "policy.action_scale",
        Kind::OptionalFloat,
        "Policy output to joint offset; unset resolves per mode",
    ),
    entry(
        "policy.standing_action_scale",
        Kind::Float,
        "Action scale while standing",
    ),
    entry(
        "policy.standing_gain_ratio",
        Kind::Float,
        "Standing runs at this fraction of gain",
    ),
    entry(
        "policy.gain",
        Kind::Integer,
        "Servo position P gain while running",
    ),
    entry(
        "policy.head_lowpass",
        Kind::OptionalFloat,
        "Low-pass on head targets; must match training (0.5)",
    ),
    entry(
        "policy.legs_lowpass",
        Kind::OptionalFloat,
        "Low-pass on leg targets; walking default 0.7",
    ),
    entry(
        "policy.ground_pick_period",
        Kind::OptionalFloat,
        "One ground-pick cycle, seconds; unset resolves per mode",
    ),
    entry(
        "policy.ground_pick_end_phase",
        Kind::OptionalFloat,
        "Pickup handback phase; unset resolves from the set or a full cycle",
    ),
    entry(
        "policy.ground_pick_action_scale",
        Kind::OptionalFloat,
        "Action scale during the ground pick; unset resolves per mode",
    ),
    entry(
        "policy.ground_pick_gain_ratio",
        Kind::Float,
        "Gain multiplier during the ground pick",
    ),
    feature(
        "policy.voltage_adapt",
        Kind::Bool,
        "Scale actions with battery voltage (nominal/measured)",
    ),
    entry(
        "policy.nominal_voltage",
        Kind::Float,
        "Reference voltage the gains were identified at",
    ),
    // ── [safety] ─────────────────────────────────────────────────────────────
    entry(
        "safety.fall_gravity_z",
        Kind::Float,
        "Projected-gravity z above which the robot counts as fallen",
    ),
    entry(
        "safety.fall_debounce_ms",
        Kind::Integer,
        "How long past the threshold before the fall verdict holds",
    ),
    entry(
        "safety.deadman_ms",
        Kind::Integer,
        "Intent age past which velocity is zeroed",
    ),
    entry(
        "safety.gain_limp",
        Kind::Integer,
        "The gain limp-fall yields at",
    ),
    feature(
        "safety.battery_empty_shutdown",
        Kind::Bool,
        "Sit and power off when the battery EMA reaches empty",
    ),
    feature(
        "safety.limp_fall",
        Kind::Bool,
        "Go limp while falling, land soft, pose back, hand to standing",
    ),
    entry(
        "safety.limp_fall_tilt_z",
        Kind::Float,
        "Tilt the robot must already be past before a prediction counts",
    ),
    entry(
        "safety.limp_fall_predict_z",
        Kind::Float,
        "Where the extrapolated tilt must reach to count as falling",
    ),
    entry(
        "safety.limp_fall_lookahead_ms",
        Kind::Integer,
        "How far ahead the tilt rate is extrapolated",
    ),
    entry(
        "safety.limp_fall_debounce_ms",
        Kind::Integer,
        "How long the fall verdict holds before the gains drop",
    ),
    entry(
        "safety.limp_fall_still_rate",
        Kind::Float,
        "Angular rate below which the robot counts as landed, rad/s",
    ),
    entry(
        "safety.limp_fall_still_ms",
        Kind::Integer,
        "How long it must stay that still before the limp ends",
    ),
    entry(
        "safety.limp_fall_max_ms",
        Kind::Integer,
        "Hard cap on the limp, however the landing reads",
    ),
    entry(
        "safety.limp_fall_pose_ms",
        Kind::Integer,
        "Ramp back to the standing pose, once landed",
    ),
    entry(
        "safety.limp_fall_pose_gain",
        Kind::Integer,
        "Gain for that ramp — softened standing, not limp",
    ),
    // ── [duck_detector] ──────────────────────────────────────────────────────
    feature(
        "duck_detector.enabled",
        Kind::Bool,
        "Look for other ducks in the camera (mediad runs it; needs a restart)",
    ),
    entry(
        "duck_detector.model",
        Kind::OptionalPath,
        "Model to run; unset = the release's, .rknn on the NPU before .onnx on the CPU",
    ),
    entry(
        "duck_detector.hz",
        Kind::Float,
        "Looks per second. 2 is a thermal limit, not a preference — flat out cooks the board",
    ),
    entry(
        "duck_detector.threshold",
        Kind::Float,
        "Confidence a detection needs, on this model's own scale (int8 scores are not 0..1)",
    ),
    // ── [chorale] ────────────────────────────────────────────────────────────
    feature(
        "chorale.accept",
        Kind::Bool,
        "Sing with nearby ducks — off means silent AND invisible on the air",
    ),
    // ── [idle] ───────────────────────────────────────────────────────────────
    feature("idle.enabled", Kind::Bool, "Yawn while the policy is idle"),
    entry(
        "idle.after_s",
        Kind::Float,
        "Seconds still before idle starts",
    ),
    entry("idle.fade_s", Kind::Float, "Seconds to fade in or out"),
    entry(
        "idle.yawn_after_s",
        Kind::Float,
        "Idle seconds before yawns are eligible",
    ),
    entry(
        "idle.yawn_every_s",
        Kind::Float,
        "Mean random wait for a yawn; 0 disables",
    ),
    entry("idle.yawn_s", Kind::Float, "Seconds per yawn"),
    entry("idle.yawn_open", Kind::Float, "Yawn mouth opening, 0 to 1"),
    entry(
        "idle.yawn_pitch",
        Kind::Float,
        "Head pitch offset in radians; negative looks up",
    ),
    // ── [theremin] ───────────────────────────────────────────────────────────
    feature(
        "theremin.enabled",
        Kind::Bool,
        "The ToF theremin may be picked up at all — off by default (robot.theremin starts it)",
    ),
    entry("theremin.socket", Kind::Text, "tofd's depth stream"),
    entry(
        "theremin.near_m",
        Kind::Float,
        "Nearest playable range, metres",
    ),
    entry(
        "theremin.far_m",
        Kind::Float,
        "Farthest playable range, metres",
    ),
    entry(
        "theremin.min_zones",
        Kind::Integer,
        "Fewest zones that make a hand",
    ),
    entry(
        "theremin.statuses",
        Kind::IntegerList,
        "ToF status bytes believed — this list decides how far the instrument reaches",
    ),
    entry(
        "theremin.hold_ms",
        Kind::Integer,
        "How long a note rides over a sensor dropout, milliseconds",
    ),
    // ── [head_imu] ───────────────────────────────────────────────────────────
    feature(
        "head_imu.enabled",
        Kind::Bool,
        "Read the head IMU (BMI088) at all — off by default; ~4% of a core when on",
    ),
    // ── [audio] ──────────────────────────────────────────────────────────────
    feature(
        "audio.enabled",
        Kind::Bool,
        "The voice and the microphone; off walks identically and stays quiet",
    ),
    entry("audio.device", Kind::Text, "ALSA playback device"),
    entry("audio.capture_device", Kind::Text, "ALSA microphone capture PCM"),
    entry(
        "audio.player",
        Kind::Text,
        "Playback executable with aplay arguments",
    ),
    entry("audio.bank", Kind::Text, "Voice bank directory"),
    feature(
        "audio.greet",
        Kind::Bool,
        "Quack once at startup — the audible \"robotd is running\"",
    ),
    feature(
        "audio.pet_detect",
        Kind::TriBool,
        "Coo when petted; unset means off — an opt-in",
    ),
    entry(
        "audio.pet_model",
        Kind::OptionalPath,
        "Petting classifier; unset = the release's copy",
    ),
    entry(
        "audio.pet_enter_threshold",
        Kind::Float,
        "Petting starts above this probability",
    ),
    entry(
        "audio.pet_exit_threshold",
        Kind::Float,
        "…and ends below this one (hysteresis)",
    ),
    // ── [media] ──────────────────────────────────────────────────────────────
    feature(
        "media.source",
        Kind::Choice(crate::MEDIA_SOURCE_LABELS),
        "Where video comes from: the head camera, or a cheap test pattern for a board without one",
    ),
    feature(
        "media.quality",
        Kind::Choice(crate::QUALITY_LABELS),
        "Video frame size and rate; 720p30 is the rung mediad was measured at",
    ),
    entry(
        "media.bitrate",
        Kind::OptionalInteger,
        "Starting video bitrate, bits/s — unset follows the quality",
    ),
    entry(
        "media.intrinsics",
        Kind::Record(
            "width = 1280\nheight = 720\nfx = 1809.5\nfy = 1809.5\ncx = 640.0\ncy = 360.0",
        ),
        "This robot's own camera calibration — a per-robot solve writes it; absent, mediad publishes the family's",
    ),
    entry(
        "media.congestion_control",
        Kind::Choice(crate::CONGESTION_LABELS),
        "Adapt the send rate to the link — disabled costs adaptivity and saves a core's worth",
    ),
    // ── [pad] ────────────────────────────────────────────────────────────────
    //
    // Which button runs which skill. Read by `padd`, not by `robotd` — but it lives in the same
    // file so `robotctl configure` stays the one editor a person has to know, and so a robot's
    // whole configuration is one thing to back up and one thing to diff.
    feature(
        "pad.a",
        Kind::Text,
        "Skill on the A button — `robotctl policy list` names what this robot has",
    ),
    feature("pad.lb", Kind::Text, "Skill on the left bumper"),
    feature("pad.rb", Kind::Text, "Skill on the right bumper"),
    // ── [pad_drive] ──────────────────────────────────────────────────────────
    //
    // What full stick deflection asks for while walking. Read by `padd`, like `[pad]`. Signed
    // bounds in the robot's frame: a `_min` is negative.
    entry("pad_drive.vx_max", Kind::Float, "Full stick forward, m/s"),
    entry(
        "pad_drive.vx_min",
        Kind::Float,
        "Full stick back, m/s — negative (-0.2 is 0.2 m/s backward)",
    ),
    entry(
        "pad_drive.vy_max",
        Kind::Float,
        "Full stick left (strafe), m/s",
    ),
    entry(
        "pad_drive.vy_min",
        Kind::Float,
        "Full stick right (strafe), m/s — negative",
    ),
    entry("pad_drive.vyaw_max", Kind::Float, "Full turn left, rad/s"),
    entry(
        "pad_drive.vyaw_min",
        Kind::Float,
        "Full turn right, rad/s — negative",
    ),
];

/// Sections that changed name: `(old, new)`.
///
/// The loader takes the old name through a `#[serde(alias)]` on the field, so a file written
/// before the rename keeps loading; the editor (`edit.rs`) carries the section to its new name so
/// its next save cannot leave both in one file, which the loader refuses as a duplicate. Listed
/// here, beside the registry, because the coverage test below has to know an alias is not a
/// section of its own — serde names aliases in its "unknown field" message like any other field.
pub const RENAMED_SECTIONS: &[(&str, &str)] = &[
    // The pad's IMU steering the head, a letter-swap away from `head_imu` — the IMU *in* the
    // head. Renamed 2026-09 for that reason alone.
    ("imu_head", "pad_imu_head_control"),
    // "detect" read as "detect what?" in the editor. Renamed 2026-09 for the thing it detects.
    ("detect", "duck_detector"),
];

/// The registry entry for a key, if it is one.
pub fn entry_for(key: &str) -> Option<&'static Entry> {
    REGISTRY.iter().find(|entry| entry.key == key)
}

/// Does this build have a `[section]` at all?
///
/// Separate from [`entry_for`] because an unknown *section* and an unknown *key inside a known
/// section* deserve different reports: one is a feature this build does not have, the other is
/// almost always a typo of a key next to it. `Params::load` says so in those terms.
pub fn has_section(section: &str) -> bool {
    REGISTRY
        .iter()
        .any(|entry| entry.key.split_once('.').is_some_and(|(s, _)| s == section))
}
