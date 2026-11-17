//! The config store: a file, not a service.
//!
//! Deliberately not a daemon of its own. A plain file plus `flock` for write serialisation and
//! write-to-temp-plus-`rename(2)` for atomicity gives no single point of failure, stays readable
//! when any service is down, and the updater never touches it — so it survives an update *and*
//! a rollback.
//!
//! `inotify` is not here yet. It belongs when a *second* process reads this file; today
//! `configd` is the only one, and watching a file you are the sole writer of is ceremony.
//!
//! This holds identity and preferences. It holds no credentials — NetworkManager owns those.

use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// Longest name accepted.
///
/// Bounded by BLE, not by taste: a legacy advertisement has 31 bytes of payload total, and the
/// local name shares it with flags and a 16-byte service UUID. A name longer than this is
/// silently truncated by the adapter or pushed into a scan response the phone may not request —
/// either way the robot appears under a name nobody chose. Truncating here, visibly, and
/// returning what was stored is the honest version.
pub const MAX_NAME: usize = 24;

/// The factory pairing PIN.
///
/// A known default rather than a random per-board value, deliberately, and it is the weakest
/// link here: everyone in radio range knows it, so it proves physical presence and nothing more.
/// It exists so the *process* is in place — the agent, the storage, the six-digit contract — and
/// so a robot is provisionable out of the box. A per-device PIN printed under the robot is
/// provisioning's job, and `is_default` on the result exists so every
/// client can say out loud that this one is not a secret.
pub const DEFAULT_PIN: &str = "000000";

/// How many digits a PIN has.
///
/// **Six, not five.** Bluetooth's passkey entry is defined as a six-digit value, 000000–999999
/// (Core spec, LE Secure Connections), and BlueZ hands it to an agent as exactly that. Five
/// digits would have to be padded to six somewhere, and the two sides would then disagree about
/// whether the PIN is "12345" or "012345" — which is a support call nobody can diagnose. Six is
/// the format the radio already speaks.
pub const PIN_DIGITS: usize = 6;

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct Config {
    #[serde(skip_serializing_if = "Option::is_none")]
    name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pairing_pin: Option<String>,
}

pub struct Store {
    path: PathBuf,
    /// Used when the file has no name yet, so an unprovisioned robot still has an identity
    /// rather than an empty string in a phone's Bluetooth list.
    ///
    /// Supplied by the caller rather than computed here, because it is derived from the board —
    /// [`crate::identity`] turns the SoC serial into `duck-7f3a` — and this type is a file, not a
    /// place that knows about hardware.
    fallback: String,
}

impl Store {
    pub fn new(path: impl Into<PathBuf>, fallback: impl Into<String>) -> Self {
        Self {
            path: path.into(),
            fallback: fallback.into(),
        }
    }

    /// The robot's name, or the fallback.
    ///
    /// A missing or unparseable file yields the fallback rather than an error: an unprovisioned
    /// board must come up with a working name, and a daemon that refuses to start because its
    /// optional config is malformed is far harder to diagnose remotely than one that logs and
    /// carries on — the same reasoning as `robotd.toml` being optional.
    pub fn name(&self) -> String {
        match self.read() {
            Ok(config) => config.name.unwrap_or_else(|| self.fallback.clone()),
            Err(e) => {
                tracing::warn!(path = %self.path.display(), error = %e, "unreadable config; using the default name");
                self.fallback.clone()
            }
        }
    }

    /// Store a name, returning what was actually stored.
    ///
    /// The caller must display the returned value, not what it sent: trimming and truncation
    /// mean they can differ, and a client showing its own input would disagree with the robot.
    pub fn set_name(&self, requested: &str) -> std::io::Result<String> {
        let name = sanitise(requested);
        if name.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "a name must have at least one printable character",
            ));
        }

        let mut config = self.read().unwrap_or_default();
        config.name = Some(name.clone());
        self.write(&config)?;
        Ok(name)
    }

    /// The pairing PIN, or the factory default.
    pub fn pairing_pin(&self) -> String {
        self.read()
            .ok()
            .and_then(|config| config.pairing_pin)
            .unwrap_or_else(|| DEFAULT_PIN.to_owned())
    }

    /// Store a PIN. Six digits, leading zeros kept.
    pub fn set_pairing_pin(&self, requested: &str) -> std::io::Result<String> {
        let pin = requested.trim();
        if pin.len() != PIN_DIGITS || !pin.chars().all(|c| c.is_ascii_digit()) {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("a pairing PIN is exactly {PIN_DIGITS} digits, 000000 to 999999"),
            ));
        }

        let mut config = self.read().unwrap_or_default();
        config.pairing_pin = Some(pin.to_owned());
        self.write(&config)?;
        Ok(pin.to_owned())
    }

    /// The PIN as a protocol result, so the `is_default` judgement lives with the constant it
    /// compares against rather than in every caller.
    pub fn name_and_pin_result(&self) -> duck_ipc_proto::PairingPinResult {
        let pin = self.pairing_pin();
        duck_ipc_proto::PairingPinResult {
            is_default: pin == DEFAULT_PIN,
            pin,
        }
    }

    fn read(&self) -> std::io::Result<Config> {
        match fs::read_to_string(&self.path) {
            Ok(text) => serde_json::from_str(&text)
                .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e)),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Config::default()),
            Err(e) => Err(e),
        }
    }

    /// Write-to-temp, fsync, rename, fsync the directory.
    ///
    /// The directory fsync is the step usually forgotten: without it the rename may not survive
    /// a power cut, and a robot switched off at the wall is the normal case rather than the
    /// exceptional one. The update journal uses the same discipline.
    fn write(&self, config: &Config) -> std::io::Result<()> {
        let dir = self.path.parent().unwrap_or(Path::new("."));
        fs::create_dir_all(dir)?;

        // `flock` serialises writers. Held on the lock file rather than the config itself, so
        // the lock outlives the rename that replaces the config.
        let lock_path = self.path.with_extension("lock");
        let lock = fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(false)
            .open(&lock_path)?;
        lock.lock()?;

        let text = serde_json::to_string_pretty(config)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;

        let temp_path = self.path.with_extension("tmp");
        {
            let mut temp = fs::File::create(&temp_path)?;
            temp.write_all(text.as_bytes())?;
            temp.write_all(b"\n")?;
            temp.sync_all()?;
        }
        fs::rename(&temp_path, &self.path)?;
        fs::File::open(dir)?.sync_all()?;
        Ok(())
    }
}

/// Trim, drop anything unprintable, and truncate to [`MAX_NAME`] on a character boundary.
///
/// Control characters are the ones that matter: this string reaches a BLE advertisement, a log
/// line and eventually an app's UI, and a newline in it would let a name split a journal record
/// in two.
fn sanitise(requested: &str) -> String {
    let cleaned: String = requested
        .trim()
        .chars()
        .filter(|c| !c.is_control())
        .collect();

    // `chars().take()` rather than slicing by byte: a name is UTF-8 and truncating mid-codepoint
    // would panic.
    cleaned
        .chars()
        .take(MAX_NAME)
        .collect::<String>()
        .trim_end()
        .to_owned()
}
