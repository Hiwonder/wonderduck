//! Own the optional human-control client outside the motor control thread.

use std::io;
use std::path::Path;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::Duration;

pub struct GamepadProcess {
    child: Child,
}

/// Incoming motion caps and optional overrides for the owned gamepad client's walking limits.
#[derive(clap::Args, Debug, Default)]
pub struct SpeedLimits {
    /// Cap forward/backward velocity from any client, m/s; also configures padd.
    #[arg(long, value_name = "MPS", value_parser = parse_speed_limit)]
    max_linear: Option<f64>,

    /// Cap lateral velocity from any client, m/s (default: --max-linear).
    #[arg(long, value_name = "MPS", value_parser = parse_speed_limit)]
    max_lateral: Option<f64>,

    /// Separate gamepad backward cap, m/s (default: --max-linear, otherwise padd's 0.4).
    #[arg(long, requires = "with_gamepad", value_name = "MPS", value_parser = parse_speed_limit)]
    max_linear_backward: Option<f64>,

    /// Cap incoming angular velocity from any client, rad/s; also configures padd.
    #[arg(long, value_name = "RAD_S", value_parser = parse_speed_limit)]
    max_angular: Option<f64>,
}

impl SpeedLimits {
    pub fn motion_caps(&self) -> (Option<f64>, Option<f64>, Option<f64>) {
        (self.max_linear, self.max_lateral.or(self.max_linear), self.max_angular)
    }

    pub fn backward_cap(&self) -> Option<f64> {
        self.max_linear_backward.or(self.max_linear)
    }
}

fn parse_speed_limit(value: &str) -> Result<f64, String> {
    let limit = value
        .parse::<f64>()
        .map_err(|_| "expected a finite speed >= 0")?;
    if !limit.is_finite() || limit < 0.0 {
        return Err("expected a finite speed >= 0; use 0 to disable motion on this axis".into());
    }
    Ok(limit)
}

fn client_command(binary: &Path, socket: &Path, params: &Path, limits: &SpeedLimits, precision: bool) -> Command {
    let mut command = Command::new(binary);
    command
        .args(["--layout", "classic"])
        .arg("--socket")
        .arg(socket)
        .arg("--config")
        .arg(params)
        .stdin(Stdio::null());
    if precision {
        command.arg("--gamepad-precision");
    }
    for (name, limit) in [
        ("--max-linear", limits.max_linear),
        ("--max-lateral", limits.max_lateral),
        (
            "--max-linear-backward",
            limits.max_linear_backward.or(limits.max_linear),
        ),
        ("--max-angular", limits.max_angular),
    ] {
        if let Some(limit) = limit {
            command.arg(name).arg(limit.to_string());
        }
    }
    command
}

impl GamepadProcess {
    pub fn id(&self) -> u32 {
        self.child.id()
    }

    pub fn start(socket: &Path, params: &Path, limits: &SpeedLimits, precision: bool) -> io::Result<Self> {
        let executable = std::env::current_exe()?;
        let parent = executable.parent().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "cannot locate executable directory",
            )
        })?;
        let binary = parent.join("padd");
        let client = Self::launch(client_command(&binary, socket, params, limits, precision))?;
        tracing::info!(pid = client.child.id(), path = %binary.display(),
            layout = "classic", socket = %socket.display(), ?limits, precision, "gamepad control started");
        Ok(client)
    }

    fn launch(mut command: Command) -> io::Result<Self> {
        #[cfg(target_os = "linux")]
        {
            use std::os::unix::process::CommandExt;
            let parent = unsafe { libc::getpid() };
            // Only robotd handles terminal Ctrl+C; Drop cleans up its client.
            command.process_group(0);
            // SAFETY: after fork this closure only calls libc and constructs errno errors.
            unsafe {
                command.pre_exec(move || {
                    if libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGTERM) != 0 {
                        return Err(io::Error::last_os_error());
                    }
                    if libc::getppid() != parent {
                        return Err(io::Error::from_raw_os_error(libc::ESRCH));
                    }
                    Ok(())
                });
            }
        }
        Ok(Self {
            child: command.spawn()?,
        })
    }
}

impl Drop for GamepadProcess {
    fn drop(&mut self) {
        match self.child.try_wait() {
            Ok(Some(_)) => return,
            Ok(None) => {}
            Err(error) => tracing::warn!(%error, "cannot inspect gamepad control during shutdown"),
        }
        if let Err(error) = self.child.kill() {
            tracing::warn!(%error, "cannot stop gamepad control");
        }
        if let Err(error) = self.child.wait() {
            tracing::warn!(%error, "cannot reap gamepad control");
        }
    }
}

pub async fn wait_for_exit(client: &mut Option<GamepadProcess>) -> io::Result<ExitStatus> {
    let Some(client) = client else {
        return std::future::pending().await;
    };
    loop {
        if let Some(status) = client.child.try_wait()? {
            return Ok(status);
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
    }
}
