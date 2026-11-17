//! Own the optional web control client outside the motor control thread.

use std::io;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::Duration;

pub struct WebProcess {
    child: Child,
}

pub enum WebStart {
    Started(WebProcess),
    NotInstalled,
}

const PERSON_VISION_RECORD_ENV: &str = "WONDERDUCK_PERSON_VISION_RECORD";
const TARGET_VISION_RECORD_ENV: &str = "WONDERDUCK_TARGET_VISION_RECORD";

fn person_vision_record_path(record: &Path) -> PathBuf {
    let mut path = record.as_os_str().to_os_string();
    path.push(".person-vision.jsonl");
    PathBuf::from(path)
}

fn target_vision_record_path(record: &Path) -> PathBuf {
    let mut path = record.as_os_str().to_os_string();
    path.push(".target-vision.jsonl");
    PathBuf::from(path)
}

fn configure_vision_record(command: &mut Command, record: Option<&Path>, person_only: bool, target_only: bool) {
    // Never inherit stale recording paths. A plain --record enables both vision records;
    // a follow-only selection enables only the requested matching sidecars.
    command.env_remove(PERSON_VISION_RECORD_ENV);
    command.env_remove(TARGET_VISION_RECORD_ENV);
    if let Some(record) = record {
        if person_only || !target_only { command.env(PERSON_VISION_RECORD_ENV, person_vision_record_path(record)); }
        if target_only || !person_only { command.env(TARGET_VISION_RECORD_ENV, target_vision_record_path(record)); }
    }
}

pub fn enabled(fake: bool, sim: bool) -> bool {
    !fake && !sim
}

fn client_path(executable: &Path) -> io::Result<PathBuf> {
    let directory = executable.parent().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            "cannot locate executable directory",
        )
    })?;
    Ok(directory.join("webpadd"))
}

impl WebProcess {
    pub fn start(socket: &Path, params: &Path, record: Option<&Path>, person_only: bool, target_only: bool) -> io::Result<WebStart> {
        Self::start_from(&std::env::current_exe()?, socket, params, record, person_only, target_only)
    }

    fn start_from(executable: &Path, socket: &Path, params: &Path, record: Option<&Path>, person_only: bool, target_only: bool) -> io::Result<WebStart> {
        let binary = client_path(executable)?;
        match std::fs::symlink_metadata(&binary) {
            Ok(_) => {}
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                return Ok(WebStart::NotInstalled);
            }
            Err(error) => return Err(error),
        }

        let mut command = Command::new(&binary);
        command
            .args(["--listen", "0.0.0.0:8081", "--socket"])
            .arg(socket)
            .arg("--config")
            .arg(params)
            .stdin(Stdio::null());
        configure_vision_record(&mut command, record, person_only, target_only);
        let client = Self::launch(command)?;
        tracing::info!(pid = client.child.id(), path = %binary.display(),
            socket = %socket.display(), "web control started");
        Ok(WebStart::Started(client))
    }

    fn launch(mut command: Command) -> io::Result<Self> {
        #[cfg(target_os = "linux")]
        {
            use std::os::unix::process::CommandExt;
            let parent = unsafe { libc::getpid() };
            command.process_group(0);
            // SAFETY: after fork the closure only calls libc and constructs errno errors.
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

impl Drop for WebProcess {
    fn drop(&mut self) {
        match self.child.try_wait() {
            Ok(Some(_)) => return,
            Ok(None) => {}
            Err(error) => tracing::warn!(%error, "cannot inspect web control during shutdown"),
        }
        if let Err(error) = self.child.kill() {
            tracing::warn!(%error, "cannot stop web control");
        }
        if let Err(error) = self.child.wait() {
            tracing::warn!(%error, "cannot reap web control");
        }
    }
}

pub async fn wait_for_exit(client: &mut Option<WebProcess>) -> io::Result<ExitStatus> {
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
