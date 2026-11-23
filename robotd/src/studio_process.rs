//! Start the installed Studio alongside the existing web and media services.

use std::io;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::time::Duration;
use tokio::process::{Child, Command};

pub struct StudioProcess { child: Child }

pub(crate) fn script_path(executable: &Path) -> Option<PathBuf> {
    let environment = executable.parent()?.parent()?.parent()?;
    Some(environment.join("extensions/wonderduck_studio/console/server.py"))
}

impl StudioProcess {
    pub fn start(socket: &Path, port: &str) -> io::Result<Option<Self>> {
        let executable = std::env::current_exe()?;
        let root = executable.parent().and_then(Path::parent).and_then(Path::parent)
            .ok_or_else(|| io::Error::other("cannot locate robot runtime directory"))?;
        let script = script_path(&executable).ok_or_else(|| io::Error::other("cannot locate Studio directory"))?;
        if !script.is_file() { return Ok(None); }
        #[cfg(unix)]
        let mut command = {
            use std::os::unix::fs::MetadataExt;
            let owner = script.metadata()?.uid();
            if unsafe { libc::geteuid() } == 0 && owner != 0 {
                std::os::unix::fs::chown(socket, Some(owner), None)?;
                let mut command = Command::new("sudo");
                command.args(["-n", "-H", "-u", &format!("#{owner}"), "--", "python3"]);
                command
            } else {
                Command::new("python3")
            }
        };
        #[cfg(not(unix))]
        let mut command = Command::new("python3");
        command.arg(&script).args(["--studio", "--port", "8090", "--hz", "20", "--serial"])
            .arg(port).arg("--imu-source").arg("id200").arg("--robotd-socket").arg(socket)
            .arg("--robot-root").arg(root)
            .stdin(Stdio::null());
        #[cfg(target_os = "linux")]
        {
            let parent = unsafe { libc::getpid() };
            command.process_group(0);
            // SAFETY: this child setup only uses async-signal-safe libc calls.
            unsafe {
                command.pre_exec(move || {
                    if libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGTERM) != 0 {
                        return Err(io::Error::last_os_error());
                    }
                    if libc::getppid() != parent { return Err(io::Error::from_raw_os_error(libc::ESRCH)); }
                    Ok(())
                });
            }
        }
        let child = command.spawn()?;
        tracing::info!(pid = ?child.id(), path = %script.display(), "Studio started");
        Ok(Some(Self { child }))
    }

    pub async fn stop(mut self) {
        let Some(pid) = self.child.id() else { return; };
        #[cfg(unix)]
        if unsafe { libc::kill(pid as libc::pid_t, libc::SIGTERM) } != 0 {
            tracing::warn!(error = %io::Error::last_os_error(), "cannot signal Studio shutdown");
            return;
        }
        match tokio::time::timeout(Duration::from_secs(15), self.child.wait()).await {
            Ok(Ok(status)) => tracing::info!(%status, "Studio stopped"),
            Ok(Err(error)) => tracing::warn!(%error, "cannot wait for Studio shutdown"),
            Err(_) => tracing::error!("Studio is still closing its maintenance session"),
        }
    }
}

pub async fn wait_for_exit(client: &mut Option<StudioProcess>) -> io::Result<std::process::ExitStatus> {
    let Some(client) = client else {
        return std::future::pending().await;
    };
    client.child.wait().await
}
