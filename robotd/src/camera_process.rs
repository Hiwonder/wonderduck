//! Own the installed camera launcher outside the motor control thread.

use duck_ipc_proto as proto;
use std::io;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use tokio::sync::oneshot;
use tokio::task::JoinHandle;

const LAUNCHER: &str = "camera-launch";
const CHECK_TIMEOUT: Duration = Duration::from_secs(3);
const STOP_TIMEOUT: Duration = Duration::from_secs(3);
const PROBE_TIMEOUT: Duration = Duration::from_secs(3);
const PROBE_INTERVAL: Duration = Duration::from_secs(5);
const STARTUP_GRACE: Duration = Duration::from_secs(10);
const RETRY_DELAY: Duration = Duration::from_secs(5);
const MISSING_RETRY_DELAY: Duration = Duration::from_secs(30);
const PROBE_FAILURE_LIMIT: u32 = 3;

pub struct CameraProcess {
    child: Option<Child>,
    frame_socket: PathBuf,
}

pub struct CameraSupervisor {
    shutdown: Option<oneshot::Sender<()>>,
    task: Option<JoinHandle<()>>,
}

pub enum CameraStart {
    Started(CameraProcess),
    NotInstalled,
    NotDetected,
    AlreadyRunning,
}

pub fn enabled(fake: bool, sim: bool, with_camera: bool, no_camera: bool) -> bool {
    !no_camera && (with_camera || (!fake && !sim))
}

impl CameraProcess {
    async fn start_in(directory: &Path) -> io::Result<CameraStart> {
        let launcher = directory.join(LAUNCHER);
        match std::fs::symlink_metadata(&launcher) {
            Ok(_) => {}
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                return Ok(CameraStart::NotInstalled);
            }
            Err(error) => return Err(error),
        }
        // Resolve the link so the launcher's own relative runtime paths remain correct.
        let launcher = std::fs::canonicalize(launcher)?;
        let mut check = tokio::process::Command::new(&launcher);
        check
            .arg("--check")
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .kill_on_drop(true);
        let output = tokio::time::timeout(CHECK_TIMEOUT, check.output())
            .await
            .map_err(|_| io::Error::new(io::ErrorKind::TimedOut, "camera detection timed out"))??;
        match output.status.code() {
            Some(0) => {}
            Some(1) => return Ok(CameraStart::NotDetected),
            Some(3) => return Ok(CameraStart::AlreadyRunning),
            _ => {
                return Err(io::Error::other(format!(
                    "camera detection failed ({}): {}",
                    output.status,
                    String::from_utf8_lossy(&output.stderr).trim()
                )));
            }
        }

        let source = launcher.parent().and_then(Path::parent).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "cannot locate camera runtime")
        })?;
        let runtime = std::env::var_os("MEDIA_RUNTIME_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| source.join("media-runtime-candidate"));
        let mut command = Command::new(&launcher);
        command.stdin(Stdio::null());
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
        let child = command.spawn()?;
        tracing::info!(pid = child.id(), path = %launcher.display(), "camera launcher started");
        Ok(CameraStart::Started(Self {
            child: Some(child),
            frame_socket: runtime.join("webpadd-media.sock"),
        }))
    }

    pub async fn stop(mut self) {
        let child = self.child.as_mut().expect("camera child exists");
        match child.try_wait() {
            Ok(Some(_)) => return,
            Ok(None) => {}
            Err(error) => {
                tracing::warn!(%error, "cannot inspect camera during shutdown");
                return;
            }
        }
        #[cfg(unix)]
        {
            if unsafe { libc::kill(child.id() as libc::pid_t, libc::SIGTERM) } != 0 {
                tracing::warn!(error = %io::Error::last_os_error(), "cannot signal camera shutdown");
                return;
            }
            let deadline = Instant::now() + STOP_TIMEOUT;
            while Instant::now() < deadline {
                match child.try_wait() {
                    Ok(Some(status)) => {
                        tracing::info!(%status, "camera stopped");
                        return;
                    }
                    Ok(None) => tokio::time::sleep(Duration::from_millis(50)).await,
                    Err(error) => {
                        tracing::warn!(%error, "cannot inspect camera shutdown");
                        return;
                    }
                }
            }
            tracing::warn!("camera did not stop in time; terminating it");
        }
    }
}

impl Drop for CameraProcess {
    fn drop(&mut self) {
        let Some(mut child) = self.child.take() else {
            return;
        };
        match child.try_wait() {
            Ok(Some(_)) => return,
            Ok(None) => {}
            Err(error) => tracing::warn!(%error, "cannot inspect camera during cleanup"),
        }
        if let Err(error) = child.kill() {
            tracing::warn!(%error, "cannot terminate camera");
        }
        // A stalled driver can keep a killed process in uninterruptible sleep.
        if let Err(error) = std::thread::Builder::new()
            .name("camera-reap".into())
            .spawn(move || {
                if let Err(error) = child.wait() {
                    tracing::warn!(%error, "cannot reap camera");
                }
            })
        {
            tracing::error!(%error, "cannot start camera reaper");
        }
    }
}

#[derive(Clone, Copy)]
struct Timing {
    startup_grace: Duration,
    probe_interval: Duration,
    probe_timeout: Duration,
    retry_delay: Duration,
    missing_retry_delay: Duration,
    failure_limit: u32,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            startup_grace: STARTUP_GRACE,
            probe_interval: PROBE_INTERVAL,
            probe_timeout: PROBE_TIMEOUT,
            retry_delay: RETRY_DELAY,
            missing_retry_delay: MISSING_RETRY_DELAY,
            failure_limit: PROBE_FAILURE_LIMIT,
        }
    }
}

enum CameraEvent {
    Exited(ExitStatus),
    Stalled(String),
}

async fn probe_frame(socket: &Path, timeout: Duration) -> io::Result<()> {
    tokio::time::timeout(timeout, async {
        let mut stream = UnixStream::connect(socket).await?;
        let request = proto::Request {
            jsonrpc: "2.0".into(),
            id: Some(proto::Id::Number(1)),
            method: proto::method::MEDIA_FRAME.into(),
            params: None,
        };
        let mut request = serde_json::to_vec(&request)?;
        request.push(b'\n');
        stream.write_all(&request).await?;
        let mut reader = BufReader::new(stream);
        let mut line = Vec::new();
        (&mut reader)
            .take(4097)
            .read_until(b'\n', &mut line)
            .await?;
        if line.len() > 4096 || !line.ends_with(b"\n") {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid camera response",
            ));
        }
        let response: proto::Response = serde_json::from_slice(&line)?;
        if response.id != Some(proto::Id::Number(1)) || response.jsonrpc != "2.0" {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "camera response ID mismatch",
            ));
        }
        if let Some(error) = response.error.as_ref() {
            return Err(io::Error::other(format!("camera: {}", error.message)));
        }
        let header: proto::MediaFrameHeader = response
            .result_as()
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
        if !header.valid_uyvy() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid camera frame",
            ));
        }
        let copied = tokio::io::copy(
            &mut reader.take(header.bytes as u64),
            &mut tokio::io::sink(),
        )
        .await?;
        if copied != header.bytes as u64 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "incomplete camera frame",
            ));
        }
        Ok(())
    })
    .await
    .map_err(|_| io::Error::new(io::ErrorKind::TimedOut, "camera frame probe timed out"))?
}

async fn wait_for_event(camera: &mut CameraProcess, timing: Timing) -> io::Result<CameraEvent> {
    let mut failures = 0;
    let mut next_probe = tokio::time::Instant::now() + timing.startup_grace;
    loop {
        if let Some(status) = camera
            .child
            .as_mut()
            .expect("camera child exists")
            .try_wait()?
        {
            return Ok(CameraEvent::Exited(status));
        }
        if tokio::time::Instant::now() >= next_probe {
            match probe_frame(&camera.frame_socket, timing.probe_timeout).await {
                Ok(()) => failures = 0,
                Err(error) => {
                    failures += 1;
                    if failures >= timing.failure_limit {
                        return Ok(CameraEvent::Stalled(error.to_string()));
                    }
                    tracing::debug!(%error, failures, "camera frame probe failed");
                }
            }
            next_probe = tokio::time::Instant::now() + timing.probe_interval;
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
    }
}

impl CameraSupervisor {
    pub fn start(explicit: bool) -> Self {
        let (shutdown, receiver) = oneshot::channel();
        let task = tokio::spawn(async move {
            let directory = match std::env::current_exe()
                .ok()
                .and_then(|path| path.parent().map(Path::to_path_buf))
            {
                Some(directory) => directory,
                None => {
                    tracing::error!("cannot locate camera launcher directory");
                    return;
                }
            };
            supervise(&directory, explicit, Timing::default(), receiver).await;
        });
        Self {
            shutdown: Some(shutdown),
            task: Some(task),
        }
    }

    pub async fn stop(mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
        if let Some(task) = self.task.take() {
            if let Err(error) = task.await {
                tracing::error!(%error, "camera supervisor failed");
            }
        }
    }
}

impl Drop for CameraSupervisor {
    fn drop(&mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
    }
}

async fn supervise(
    directory: &Path,
    explicit: bool,
    timing: Timing,
    mut shutdown: oneshot::Receiver<()>,
) {
    let mut missing_logged = false;
    let mut external_logged = false;
    loop {
        let started = tokio::select! {
            _ = &mut shutdown => return,
            result = CameraProcess::start_in(directory) => result,
        };
        let delay = match started {
            Ok(CameraStart::Started(mut camera)) => {
                missing_logged = false;
                external_logged = false;
                let event = tokio::select! {
                    _ = &mut shutdown => { camera.stop().await; return; }
                    event = wait_for_event(&mut camera, timing) => event,
                };
                let retry_delay = match event {
                    Ok(CameraEvent::Exited(status)) if status.code() == Some(3) => {
                        tracing::info!("camera stream already running; camera autostart skipped");
                        external_logged = true;
                        timing.missing_retry_delay
                    }
                    Ok(CameraEvent::Exited(status)) => {
                        tracing::error!(%status, "camera exited; retrying");
                        timing.retry_delay
                    }
                    Ok(CameraEvent::Stalled(error)) => {
                        tracing::error!(%error, "camera stopped delivering frames; restarting");
                        timing.retry_delay
                    }
                    Err(error) => {
                        tracing::error!(%error, "camera monitoring failed; restarting");
                        timing.retry_delay
                    }
                };
                camera.stop().await;
                retry_delay
            }
            Ok(CameraStart::NotInstalled) => {
                if explicit {
                    tracing::error!("camera launcher is not installed beside robotd");
                } else {
                    tracing::info!("camera launcher is not installed; camera autostart disabled");
                }
                return;
            }
            Ok(CameraStart::NotDetected) => {
                if !missing_logged {
                    tracing::info!("no camera detected; camera autostart skipped");
                }
                missing_logged = true;
                timing.missing_retry_delay
            }
            Ok(CameraStart::AlreadyRunning) => {
                if !external_logged {
                    tracing::info!("camera stream already running; camera autostart skipped");
                }
                external_logged = true;
                timing.missing_retry_delay
            }
            Err(error) => {
                tracing::error!(%error, "camera detection or startup failed; retrying");
                timing.missing_retry_delay
            }
        };
        tokio::select! {
            _ = &mut shutdown => return,
            _ = tokio::time::sleep(delay) => {}
        }
    }
}
