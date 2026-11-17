//! What this daemon streams and what it looks for, out of the config file `robotd` already reads.
//!
//! `[media]` in `/etc/robot/robotd.toml` — camera or test pattern, frame size, rate, bitrate — and
//! `[duck_detector]` beside it, which is this daemon's too because the frames are on this daemon's tee.
//! The schema, the defaults and the validation are `robotd_params`'s, which is the point: the crate
//! read here is the one `robotctl configure` writes through, so the editor cannot offer a value
//! this daemon would not understand.
//!
//! Its own module rather than four lines in `main`, for one reason: `main` is Linux-only, so
//! anything living there is not compiled — let alone tested — on the machine it is written on.

use std::path::{Path, PathBuf};

use robotd_params::Params;

/// The file, when `--config` said nothing.
pub fn default_path() -> PathBuf {
    PathBuf::from(robotd_params::DEFAULT_PATH)
}

/// Read the file, or fall back to the built-in defaults.
///
/// **A file this daemon cannot read is not a reason to have no video.** `robotd` refuses to start
/// on a broken params file — that is the loud signal, and it is the daemon whose control loop the
/// file configures. A robot in that state is already down, and its camera is how somebody looks at
/// it. So this warns, names the file, and streams the defaults rather than joining the outage.
///
/// A *missing* file is not even a warning at the default path: an unprovisioned board has none and
/// streams its camera at 720p30 like every other. A path named on the command line must exist,
/// which is `Params::load`'s own rule and the reason `explicit` is passed through.
pub fn load(path: &Path, explicit: bool) -> Params {
    match Params::load(path, explicit) {
        Ok(params) => params,
        Err(e) => {
            tracing::warn!(
                error = %e,
                path = %path.display(),
                "unusable params file; streaming the built-in defaults"
            );
            Params::default()
        }
    }
}
