//! The signed manifest describing one release.
//!
//! Fields that cannot be retrofitted onto
//! already-shipped robots are present from the first release even while unused —
//! a robot that never learned to read `min_supported` can't be force-upgraded
//! later.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Manifest {
    /// Channel this manifest belongs to, cross-checked against config so a
    /// misconfigured URL can't silently install a model as the daemon.
    pub channel: String,

    pub version: semver::Version,

    /// Artifact location. Absolute, or relative to the manifest for local/HF
    /// sources.
    pub url: String,

    /// Lowercase hex SHA-256 of the artifact.
    pub sha256: String,

    /// Detached minisign signature of the artifact.
    pub sig_url: String,

    /// Compressed artifact size, if the publisher recorded it. Used only for the
    /// preflight space estimate — the authoritative integrity check is `sha256`.
    #[serde(default)]
    pub size: Option<u64>,

    /// Minimum hardware revision. Refused if above the robot's `hw_rev`.
    #[serde(default)]
    pub min_hw_rev: u32,

    /// On-disk/config schema this release expects.
    ///
    /// **Not a compatibility gate.** It is context handed to the post-install hook,
    /// which performs the migration. Gating on it would
    /// be self-defeating: the engine evaluating a manifest is always the *previous*
    /// release's engine, so refusing `schema_version > supported` would make every
    /// schema bump undeliverable — including the release that brings the engine which
    /// understands it.
    #[serde(default = "one")]
    pub schema_version: u32,

    /// Minimum-version floor: robots below this must upgrade, without waiting
    /// for a tap in the app.
    #[serde(default)]
    pub min_supported: Option<semver::Version>,

    /// Model channel only: the daemon model-API version this bundle requires.
    /// Loadable when `model_api <= daemon's model_api`.
    ///
    #[serde(default)]
    pub model_api: Option<u32>,

    /// Git SHA of the build, recorded for provenance.
    ///
    #[serde(default)]
    pub source_revision: Option<String>,

    /// Shown in the app. Untrusted display text — never interpreted.
    #[serde(default)]
    pub changelog: Option<String>,
}

fn one() -> u32 {
    1
}

/// Facts about the robot that a manifest is checked against.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Capabilities {
    pub hw_rev: u32,
    /// Model API the running daemon implements. `None` when `robotd` is
    /// unreachable — model compatibility then cannot be confirmed.
    pub model_api: Option<u32>,
    /// Schema version the running release understands.
    ///
    /// Recorded for hook context and diagnostics only — deliberately *not* compared
    /// against the manifest; see [`Manifest::schema_version`].
    pub schema_version: u32,
}

impl Manifest {
    /// Whether this release can be installed on this robot.
    ///
    /// Refusals carry a human-readable reason — it surfaces in the app, so it must
    /// say *why* rather than just "incompatible".
    pub fn compatibility(&self, caps: &Capabilities) -> Compatibility {
        if self.min_hw_rev > caps.hw_rev {
            return Compatibility::Refused(format!(
                "requires hardware revision {} or newer; this robot is rev {}",
                self.min_hw_rev, caps.hw_rev
            ));
        }

        if let Some(required) = self.model_api {
            match caps.model_api {
                Some(supported) if required <= supported => {}
                Some(supported) => {
                    return Compatibility::Refused(format!(
                        "needs model API {required}, but the running daemon implements \
                         {supported} — update the daemon first"
                    ));
                }
                // Deliberately Unknown, not Refused: for the daemon channel this
                // is fine to proceed through (that update is how a dead robotd
                // gets fixed), while for the model channel the caller should wait.
                // Collapsing the two would either block recovery or install an
                // unloadable model.
                None => {
                    return Compatibility::Unknown(
                        "robotd is unreachable, so its model API version is unknown".into(),
                    );
                }
            }
        }

        Compatibility::Ok
    }

    /// Does the floor make upgrading to this release non-optional?
    ///
    /// True when the robot is running something below `min_supported` — the
    /// mechanism for forcing robots off a known-bad release without waiting for
    /// someone to tap update.
    ///
    /// A robot with nothing installed is *not* treated as mandatory: there is no
    /// bad version to escape, and the normal install path applies.
    pub fn is_mandatory_for(&self, installed: Option<&semver::Version>) -> bool {
        match (&self.min_supported, installed) {
            (Some(floor), Some(current)) => current < floor,
            _ => false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Compatibility {
    Ok,
    /// Refused, with a reason safe to show the user.
    Refused(String),
    /// Cannot be determined right now — e.g. `robotd` is down so its model API
    /// is unknown.
    ///
    /// Deliberately distinct from `Refused`: for the daemon channel this is
    /// fine to proceed through (that update is how you *fix* a dead `robotd`),
    /// while for the model channel it is a reason to wait. Collapsing the two
    /// would either block recovery or allow an unloadable model.
    Unknown(String),
}

impl Compatibility {
    pub fn is_ok(&self) -> bool {
        matches!(self, Compatibility::Ok)
    }

    pub fn reason(&self) -> Option<&str> {
        match self {
            Compatibility::Ok => None,
            Compatibility::Refused(r) | Compatibility::Unknown(r) => Some(r),
        }
    }
}
