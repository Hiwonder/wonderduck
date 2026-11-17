//! Pre/post-install hooks.
//!
//! Hooks ship *inside* the signed artifact, so no unsigned code ever runs. Same
//! idea as dpkg's `postinst`;
//!
//! Ordering:
//! ```text
//! extract → [pre_install] → symlink swap → [post_install] → apply → health gate
//! ```
//!
//! A non-zero exit is a failed update and triggers rollback, identically to a
//! failed health probe. Hooks are part of the gate, not fire-and-forget.

use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::Error;

/// Hook filenames looked for inside a release, relative to its root.
pub const PRE_INSTALL: &str = "hooks/preinstall";
pub const POST_INSTALL: &str = "hooks/postinstall";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HookKind {
    PreInstall,
    PostInstall,
}

impl HookKind {
    pub fn relative_path(self) -> &'static str {
        match self {
            HookKind::PreInstall => PRE_INSTALL,
            HookKind::PostInstall => POST_INSTALL,
        }
    }
}

/// Context passed to hooks as environment variables, so one hook can branch on
/// what kind of transition it's in (notably a `schema_version` bump needing a
/// config migration).
#[derive(Debug, Clone)]
pub struct HookContext {
    /// Component name, e.g. `daemon`. Exported as both `UPDATE_COMPONENT` and
    /// `UPDATE_CHANNEL`: they are the same thing here (a component's channel is its
    /// name), and §9 documents the latter.
    pub component: String,
    pub old_version: Option<semver::Version>,
    pub new_version: semver::Version,
    /// Component root, e.g. `/opt/robot/daemon`.
    pub install_dir: PathBuf,
    /// The release being installed, e.g. `/opt/robot/daemon/releases/1.4.2`. This is
    /// what a migration usually wants.
    pub release_dir: PathBuf,
    pub old_schema_version: Option<u32>,
    pub new_schema_version: u32,
}

impl HookContext {
    /// `UPDATE_*` environment pairs. Absent values are omitted rather than set
    /// empty, so a hook can distinguish "first install" from "unknown".
    pub fn env(&self) -> Vec<(String, String)> {
        let mut env = vec![
            ("UPDATE_COMPONENT".into(), self.component.clone()),
            // Documented in §9 as `UPDATE_CHANNEL`; same value.
            ("UPDATE_CHANNEL".into(), self.component.clone()),
            ("UPDATE_NEW_VERSION".into(), self.new_version.to_string()),
            // The component's *root* (e.g. /opt/robot/daemon), not the release
            // directory. A hook's own location is its cwd, and `$PWD` is the release
            // being installed — see §9.
            (
                "UPDATE_INSTALL_DIR".into(),
                self.install_dir.display().to_string(),
            ),
            (
                "UPDATE_RELEASE_DIR".into(),
                self.release_dir.display().to_string(),
            ),
            (
                "UPDATE_NEW_SCHEMA_VERSION".into(),
                self.new_schema_version.to_string(),
            ),
        ];
        if let Some(old) = &self.old_version {
            env.push(("UPDATE_OLD_VERSION".into(), old.to_string()));
        }
        if let Some(old) = self.old_schema_version {
            env.push(("UPDATE_OLD_SCHEMA_VERSION".into(), old.to_string()));
        }
        env
    }
}

#[derive(Debug, Clone)]
pub struct HookOutcome {
    pub ran: bool,
    pub exit_code: Option<i32>,
    /// Captured output, truncated. Logged by [`run`] whatever the outcome, and
    /// returned so a caller can put it in a reply too.
    pub output: String,
}

/// Captured output is truncated to this, so a chatty or runaway hook can't blow up
/// the update log.
const MAX_OUTPUT: usize = 8 * 1024;

/// Run a hook if present.
///
/// A missing hook is success (`ran: false`) — most releases won't have one. A
/// present-but-failing hook is an error, and so is a timeout: a hook that hangs
/// must not wedge the updater forever, and an unfinished migration is not a
/// successful one.
pub async fn run(
    release_dir: &Path,
    kind: HookKind,
    ctx: &HookContext,
    timeout: Duration,
) -> Result<HookOutcome, Error> {
    let hook = release_dir.join(kind.relative_path());
    if !hook.exists() {
        return Ok(HookOutcome {
            ran: false,
            exit_code: Some(0),
            output: String::new(),
        });
    }

    let hook_name = kind.relative_path().to_owned();

    let mut command = tokio::process::Command::new(&hook);
    command
        .current_dir(release_dir)
        .env_clear()
        // A minimal environment: hooks get what we pass plus PATH, so their
        // behaviour doesn't depend on however systemd happened to invoke us.
        .env(
            "PATH",
            "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        )
        .kill_on_drop(true)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped());

    for (key, value) in ctx.env() {
        command.env(key, value);
    }

    let child = crate::spawn::retrying_busy(&mut command)
        .await
        .map_err(|e| Error::Hook {
            hook: hook_name.clone(),
            detail: format!("could not execute {}: {e}", hook.display()),
        })?;

    // `kill_on_drop` plus this timeout is what guarantees a hanging hook is
    // reaped rather than left behind holding the update open.
    let finished = tokio::time::timeout(timeout, child.wait_with_output()).await;

    let output = match finished {
        Ok(Ok(output)) => output,
        Ok(Err(e)) => {
            return Err(Error::Hook {
                hook: hook_name,
                detail: format!("failed while running: {e}"),
            });
        }
        Err(_elapsed) => {
            return Err(Error::Hook {
                hook: hook_name,
                detail: format!("timed out after {}s", timeout.as_secs()),
            });
        }
    };

    let mut text = String::from_utf8_lossy(&output.stdout).into_owned();
    text.push_str(&String::from_utf8_lossy(&output.stderr));
    if text.len() > MAX_OUTPUT {
        // Cut on a char boundary: `from_utf8_lossy` can place a multi-byte character (often a
        // 3-byte U+FFFD for invalid input) straddling the cap, and a bare `truncate` panics on
        // that — inside `Engine::apply`, where it kills the unattended-update task for good.
        let mut end = MAX_OUTPUT;
        while !text.is_char_boundary(end) {
            end -= 1;
        }
        text.truncate(end);
    }

    if !output.status.success() {
        return Err(Error::Hook {
            hook: hook_name,
            detail: format!(
                "exited with {}: {}",
                output
                    .status
                    .code()
                    .map(|c| c.to_string())
                    .unwrap_or_else(|| "signal".into()),
                text.trim()
            ),
        });
    }

    // **Logged on success, not only on failure.** Both call sites take this outcome with `?` and
    // drop it, so for a hook that worked the output went nowhere — and this is the hook that
    // installs ONNX Runtime and the GStreamer stack, whose whole report is the answer to "can this
    // board encode H.264 in hardware, at this release". A grep of `journalctl -u updaterd` for it
    // came back empty on a board where the hook had plainly run, which is how this was found.
    //
    // Line by line rather than one entry: a multi-line message is one journal record, and what a
    // reader does with this is grep it.
    for line in text.lines() {
        if !line.trim().is_empty() {
            tracing::info!(hook = %hook_name, "{}", line.trim_end());
        }
    }

    Ok(HookOutcome {
        ran: true,
        exit_code: output.status.code(),
        output: text,
    })
}
