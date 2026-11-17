//! Per-robot configuration.
//!
//! The engine is generic; everything robot-specific lives here. Adapting to a
//! different robot should mean a new config file, new signing keys, and possibly
//! a new health probe — not engine changes.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::Duration;

use serde::{Deserialize, Serialize};

/// Root of `/etc/robot/updater.toml`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// The file this was read from, when it was read from one.
    ///
    /// Not configuration — it is how the engine hands the *same* config to the release's own
    /// `updaterd --self-test` before committing to it. That check exists to catch a new binary
    /// which rejects the board's `updater.toml`, and it can only do that if it is pointed at the
    /// file in use: `--config` has a default, so a self-test run without one silently validates
    /// `/etc/robot/updater.toml` however the running daemon was started. It read as the release
    /// being broken.
    ///
    /// `None` when the config came from text rather than a path — every test, and nothing on a
    /// board.
    #[serde(skip)]
    pub loaded_from: Option<PathBuf>,

    /// Directory of trusted minisign public keys. A signature is valid if it
    /// verifies against *any* key in here.
    ///
    /// A set rather than one key so a lost or compromised key is survivable —
    ///
    pub trusted_keys_dir: PathBuf,

    /// Single forward-compatibility guard. An artifact declaring a higher
    /// `min_hw_rev` than this is refused.
    ///
    /// v1 targets one hardware configuration, so this is deliberately one
    /// integer and not a capability matrix.
    pub hw_rev: u32,

    /// Engine-owned state: lock file, update log, boot counter. Must NOT be
    /// inside any component's `install_dir` — it has to survive every swap and
    /// rollback.
    pub state_dir: PathBuf,

    /// Where fetched policies are kept: one directory per `org/name/revision`, outside every
    /// release directory so a policy somebody chose survives an update and a rollback.
    ///
    ///
    /// **Configurable because it was a constant, and a constant is right exactly once.**
    /// `/var/lib/robot/policies` is correct on a board and unwritable on the laptop the twin runs
    /// on, so every `policy.fetch` against a simulated duck refused with `Permission denied (os
    /// error 13)` naming a path no simulator should ever have been given. The default is still
    /// the board's, so nothing in a release changes; `scripts/duck-sim` sets it per duck.
    #[serde(default = "default_policy_library")]
    pub policy_library: PathBuf,

    /// Where `robotd` listens. Used by every `health = { probe = "socket" }` component
    /// and by the pre-restart `safeToRestart` query.
    ///
    /// Absent or silent is a normal state, not an error:
    /// `robotd` may legitimately be stopped, crashed, or not yet installed.
    #[serde(default = "default_robot_socket")]
    pub robot_socket: PathBuf,

    /// Accept artifacts signed with a key marked dev-only. Off in production.
    ///
    #[serde(default)]
    pub allow_dev_keys: bool,

    /// Largest uncompressed size an artifact may expand to, in bytes.
    ///
    /// Configurable because a model bundle of several ONNX policies is legitimately
    /// far larger than a daemon binary; a global ceiling would reject a real release.
    #[serde(default)]
    pub max_uncompressed_bytes: Option<u64>,

    /// Largest number of entries an artifact archive may contain.
    #[serde(default)]
    pub max_archive_entries: Option<usize>,

    /// How often to check each component's source for a new release.
    ///
    /// `None` disables periodic checking entirely, which also disables the only path
    /// that makes `min_supported` effective (§8.1) — a robot nobody taps update on
    /// never learns a floor exists.
    #[serde(default, with = "humantime_serde::option")]
    pub check_interval: Option<Duration>,

    /// Which updates the periodic check may apply with **no client attached**.
    ///
    /// Inert without [`Self::check_interval`]: the scheduler is the only thing that
    /// applies unattended, so a policy with no timer to run it does nothing.
    #[serde(default)]
    pub auto_apply: AutoApply,

    /// Extra uids permitted to perform **mutating** operations over the IPC socket.
    ///
    /// The uid `updaterd` itself runs as is always permitted — it could replace the
    /// daemon anyway, so denying it would be theatre. Everyone else needs listing
    /// here (or in [`Self::allow_gids`]) to apply, roll back, select or pin.
    ///
    /// Read-only requests (`status`, `log`, `listInstalled`, `check`, `subscribe`) are
    /// **not** gated by this: reaching the socket at all already requires its group
    /// (mode 0660), and support needs to be able to look at a robot without being
    /// authorised to change it.
    #[serde(default)]
    pub allow_uids: Vec<u32>,

    /// Groups permitted to perform mutating operations, by gid.
    #[serde(default)]
    pub allow_gids: Vec<u32>,

    /// Users permitted to perform mutating operations, **by name**.
    ///
    /// Preferred over [`Self::allow_uids`] for anything shipped, because
    /// `systemd-sysusers` allocates uids dynamically: a number that is correct on the
    /// board a config was written for is wrong on the next one. `btd` is listed here so
    /// the app can trigger an update, which is what M6 exists to deliver.
    ///
    /// A name that does not resolve is a warning, not a fatal error. A robot missing an
    /// optional service must still serve status and logs.
    #[serde(default)]
    pub allow_users: Vec<String>,

    /// Groups permitted to perform mutating operations, by name.
    ///
    /// Deliberately empty in the shipped config, and a test enforces that `robot` never
    /// appears here: membership of `robot` is what gets a process as far as *talking* to
    /// updaterd, and listing it would collapse that layer into the change-authority one.
    #[serde(default)]
    pub allow_groups: Vec<String>,

    /// Defaulted so a config with no components reaches [`Config::validate`] and
    /// gets a clear message, rather than a bare serde "missing field".
    #[serde(rename = "component", default)]
    pub components: BTreeMap<String, ComponentConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentConfig {
    pub source: SourceConfig,

    /// Root under which releases are staged and linked.
    pub install_dir: PathBuf,

    pub on_apply: ApplyAction,

    #[serde(default)]
    pub health: HealthCheck,

    /// Retained previous releases, for rollback. The golden release is kept
    /// independently of this count.
    #[serde(default = "default_keep_previous")]
    pub keep_previous: usize,

    /// Never-pruned known-good release.
    #[serde(default)]
    pub golden: Option<semver::Version>,

    /// Refuse anything but this version. Set by `robotctl pin`.
    #[serde(default)]
    pub pinned: Option<semver::Version>,

    /// Files required in the extracted artifact before hooks run or the release becomes live.
    #[serde(default)]
    pub required_files: Vec<PathBuf>,

    /// Maximum compressed artifact size. When set, the signed manifest must declare a size.
    #[serde(default)]
    pub max_artifact_bytes: Option<u64>,
}

fn default_keep_previous() -> usize {
    1
}

/// Which updates the periodic check may apply with no client attached.
///
/// One ordered setting rather than a boolean per urgency. Two booleans would let a config
/// say "apply ordinary updates automatically but not mandatory ones" — auto-updating
/// everything *except* the releases published specifically to rescue a broken fleet. That
/// combination has no legitimate use and is not worth being able to write down.
///
/// Whatever the policy, an unattended apply is still an ordinary apply: it runs the same
/// preflight, so a robot that is walking or streaming refuses and retries at the next
/// interval rather than restarting under someone's hands (`preflight.rs`), and the same
/// health gate, so a release that does not come up is rolled back.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AutoApply {
    /// Never apply without a client. Availability is still logged, and a mandatory
    /// release is logged loudly — an ignored one is worth shouting about.
    Off,

    /// Only a release whose manifest declares the running version below `min_supported`.
    ///
    /// The default, and the remediation path for "we shipped a bad release": robots pull
    /// themselves forward instead of waiting for someone to open the app (§8.1). Ordinary
    /// releases still wait for a client, because when a robot restarts is the owner's
    /// decision.
    #[default]
    Mandatory,

    /// Any available release.
    ///
    /// For canary and bench robots — §16.2's Tier 2 wants lab robots that track
    /// `staging` and update on every candidate. On a client robot this takes the
    /// "when does my robot restart" decision away from its owner, which is the decision
    /// the whole app-driven update flow exists to give them.
    All,
}

impl AutoApply {
    /// May the scheduler apply a candidate of this urgency unattended?
    pub fn permits(self, mandatory: bool) -> bool {
        match self {
            AutoApply::Off => false,
            AutoApply::Mandatory => mandatory,
            AutoApply::All => true,
        }
    }
}

/// Where artifacts come from. Per-component because the daemon lives on GitHub
/// Releases and models on HF Hub.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum SourceConfig {
    GithubReleases {
        /// `ORG/REPO`.
        repo: String,
        /// Tag prefix identifying the channel, e.g. `daemon-v`.
        tag_prefix: String,
        /// Release asset holding the signed manifest.
        #[serde(default = "default_manifest_asset")]
        manifest_asset: String,
        /// Tag prefix for per-branch dev builds, so `--ref my-branch` resolves to
        /// `daemon-dev-my-branch`.
        ///
        /// Separate from `tag_prefix` because the two streams must not be confusable: a
        /// dev tag *moves* (it points at whatever that branch built last) while a release
        /// tag is immutable, and `newest_version` must never consider a dev tag when
        /// resolving `latest` for the fleet.
        #[serde(default = "default_ref_tag_prefix")]
        ref_tag_prefix: String,
        /// Tag prefix for release candidates, so `--staging` resolves to
        /// `daemon-staging-v<version>`.
        ///
        /// A third prefix rather than a flag on `tag_prefix`, for the reason the second one
        /// exists: the streams must not be confusable. A candidate is flagged as a prerelease
        /// on GitHub precisely so `newest_version` cannot reach it, and giving that scan an
        /// "unless…" would put the fleet one config typo away from tracking candidates.
        #[serde(default = "default_staging_tag_prefix")]
        staging_tag_prefix: String,
    },
    HfHub {
        /// `ORG/MODEL`.
        repo: String,
        /// Branch, tag, or commit. A moving branch means "latest".
        revision: String,
        #[serde(default = "default_manifest_asset")]
        manifest_file: String,
    },
    /// A local directory. Not a production source — this is what makes the
    /// engine testable against the real code path with no network, and backs
    /// the dev sideload flow.
    LocalDir { path: PathBuf },
}

/// Default prefix for dev-build tags.
///
/// Names `daemon` because that is this robot's only source-backed component today; a robot
/// with a differently-named channel sets it explicitly, the same as `tag_prefix`.
fn default_ref_tag_prefix() -> String {
    "daemon-dev-".to_owned()
}

/// Default prefix for release-candidate tags, matching what `release.yml` pushes.
fn default_staging_tag_prefix() -> String {
    "daemon-staging-v".to_owned()
}

fn default_manifest_asset() -> String {
    "manifest.json".to_owned()
}

/// What to do once the new release is linked.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "action", rename_all = "snake_case")]
pub enum ApplyAction {
    /// Nothing to do (the consumer picks it up on its own).
    None,
    /// Full restart. Drops motor control briefly — only for the daemon channel.
    Restart { units: Vec<String> },
    /// Signal in place, no restart. Used for models so a weights swap doesn't
    /// interrupt motor control.
    Reload { unit: String, signal: String },
}

/// How to decide whether the new release is good.
///
/// This gate is what makes auto-rollback meaningful, so a weak probe here
/// undermines the whole design —
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(tag = "probe", rename_all = "snake_case")]
pub enum HealthCheck {
    /// No gate: commit as soon as apply returns. Only acceptable for components
    /// that cannot break the robot.
    #[default]
    None,
    /// Ask `robotd` over its unix socket and wait for it to report healthy.
    ///
    /// No path here on purpose. The socket is one robot-wide fact, not a per-component
    /// one, so it lives in [`Config::robot_socket`].
    Socket {
        #[serde(with = "humantime_serde")]
        timeout: Duration,
    },
    /// Run a command; exit status 0 means healthy. Escape hatch for probes that
    /// don't fit the socket model.
    Command {
        program: PathBuf,
        #[serde(default)]
        args: Vec<String>,
        #[serde(with = "humantime_serde")]
        timeout: Duration,
    },
}

impl HealthCheck {
    pub fn timeout(&self) -> Option<Duration> {
        match self {
            HealthCheck::None => None,
            HealthCheck::Socket { timeout, .. } | HealthCheck::Command { timeout, .. } => {
                Some(*timeout)
            }
        }
    }
}

fn default_robot_socket() -> PathBuf {
    PathBuf::from("/run/robotd.sock")
}

fn default_policy_library() -> PathBuf {
    PathBuf::from(crate::policy::LIBRARY_ROOT)
}

impl Config {
    /// Parse from TOML text. Always validated — an invalid config must not be
    /// constructible.
    pub fn from_toml(text: &str) -> Result<Self, crate::Error> {
        let mut value: toml::Value = toml::from_str(text).map_err(|e| crate::Error::Config(e.to_string()))?;
        // Older device configurations may contain this obsolete boolean.
        if let Some(table) = value.as_table_mut()
            && let Some(legacy) = table.remove("allow_fault_injection")
            && !legacy.is_bool()
        {
            return Err(crate::Error::Config("invalid obsolete updater setting".into()));
        }
        let config: Self = value.try_into().map_err(|e: toml::de::Error| crate::Error::Config(e.to_string()))?;
        config.validate()?;
        Ok(config)
    }

    pub fn load(path: &std::path::Path) -> Result<Self, crate::Error> {
        let text = std::fs::read_to_string(path).map_err(|e| crate::Error::Io {
            path: path.to_path_buf(),
            source: e,
        })?;
        let mut config = Self::from_toml(&text)?;
        // Absolute, because the self-test runs as a fresh process whose working directory is not
        // this one's.
        config.loaded_from = Some(path.canonicalize().unwrap_or_else(|_| path.to_path_buf()));
        Ok(config)
    }

    /// Bounds applied when extracting an artifact.
    pub fn archive_limits(&self) -> crate::verify::ArchiveLimits {
        let defaults = crate::verify::ArchiveLimits::default();
        crate::verify::ArchiveLimits {
            max_uncompressed_bytes: self
                .max_uncompressed_bytes
                .unwrap_or(defaults.max_uncompressed_bytes),
            max_entries: self.max_archive_entries.unwrap_or(defaults.max_entries),
        }
    }

    pub fn component(&self, id: &str) -> Result<&ComponentConfig, crate::Error> {
        self.components
            .get(id)
            .ok_or_else(|| crate::Error::UnknownComponent(id.to_owned()))
    }

    /// Reject self-inconsistent configs at load time rather than mid-update.
    ///
    /// Each of these would otherwise surface as data loss or a mysterious failure
    /// during an update, which is the worst time to discover them.
    pub fn validate(&self) -> Result<(), crate::Error> {
        let bad = |msg: String| Err(crate::Error::Config(msg));

        if self.components.is_empty() {
            return bad("no components configured; nothing could ever be updated".into());
        }

        for (name, component) in &self.components {
            for required in &component.required_files {
                if required.as_os_str().is_empty()
                    || !required
                        .components()
                        .all(|part| matches!(part, std::path::Component::Normal(_)))
                {
                    return bad(format!(
                        "component {name}: required_files entry {} must be a nonempty relative file path without ..",
                        required.display()
                    ));
                }
            }
            if component.max_artifact_bytes == Some(0) {
                return bad(format!(
                    "component {name}: max_artifact_bytes must be positive"
                ));
            }

            // A relative install_dir would resolve against the daemon's cwd,
            // which systemd does not guarantee.
            if !component.install_dir.is_absolute() {
                return bad(format!(
                    "component {name}: install_dir must be absolute, got {}",
                    component.install_dir.display()
                ));
            }

            // The decisive one: engine state inside a release tree would be
            // destroyed by the very swap or rollback it exists to record.
            if self.state_dir.starts_with(&component.install_dir) {
                return bad(format!(
                    "state_dir {} is inside component {name}'s install_dir {} — a swap or \
                     rollback would destroy the update log and boot counter",
                    self.state_dir.display(),
                    component.install_dir.display()
                ));
            }

            // Two components sharing a tree would prune each other's releases.
            for (other_name, other) in &self.components {
                if other_name != name && component.install_dir == other.install_dir {
                    return bad(format!(
                        "components {name} and {other_name} share install_dir {} — they would \
                         prune each other's releases",
                        component.install_dir.display()
                    ));
                }
            }

            // Golden exists to be a rollback target that is never pruned; a
            // keep_previous of 0 leaves nothing else to fall back to.
            if component.keep_previous == 0 && component.golden.is_none() {
                return bad(format!(
                    "component {name}: keep_previous = 0 with no golden release leaves no \
                     rollback target"
                ));
            }
        }

        if !self.state_dir.is_absolute() {
            return bad(format!(
                "state_dir must be absolute, got {}",
                self.state_dir.display()
            ));
        }

        // Same rule as `state_dir`, for the same reason: this is resolved by a daemon whose
        // working directory is not anybody's, and a relative path there is a directory nobody
        // meant to write to.
        if !self.policy_library.is_absolute() {
            return bad(format!(
                "policy_library must be absolute, got {}",
                self.policy_library.display()
            ));
        }

        Ok(())
    }
}
