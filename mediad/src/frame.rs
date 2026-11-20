//! A local `media.frame` endpoint for a recorder or perception process on the robot.
//!
//! A frame stays out of the WebRTC control channel: at the default geometry the UYVY payload is
//! about 1.8 MiB, so JSON/base64 would make a control request several MiB and let a slow peer tie
//! camera data to the network. This socket sends one JSON-RPC response header, then precisely
//! `bytes` raw bytes, which keeps the metadata inspectable without copying pixels through a text
//! encoding.
//!
//! **It asks for a frame rather than taking the last one.** [`Frames`] is a rendezvous, not a
//! cache: the capture branch copies a buffer only when a reader has asked for one
//! ([`crate::pipeline::Frames`] explains why — 1.84 MiB thirty times a second for readers that
//! want two). So a caller here waits for the capture that answers it, bounded by
//! [`pipeline::FRAME_TIMEOUT`], and a camera that has stopped is reported as a timeout rather than
//! answered with the frame it stopped on.
//!
//! The socket is group-readable like the other observation sockets: whoever may watch
//! `robot.state` may ask for a picture.

use std::os::unix::fs::PermissionsExt;
use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, UNIX_EPOCH};

use anyhow::{Context, Result};
use duck_ipc_proto as proto;

use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};

use crate::pipeline::Frames;
use crate::session::{VIDEO_METHOD, Video, video_params};

const SOCKET_MODE: u32 = 0o660;

/// The group that may ask for a frame. Deliberately the same one as `robotd`'s socket, `padd`'s
/// tap and `tof`'s stream: whoever may watch the robot may watch what it sees.
const GROUP: &str = "robot";

/// The longest request this endpoint will read. `media.frame` takes no parameters worth naming, so
/// anything approaching this is a client that has lost the plot.
const MAX_REQUEST_BYTES: usize = 4096;

/// Claim the socket without replacing a live listener or a non-socket file.
pub async fn bind(socket: &Path) -> Result<(std::fs::File, UnixListener)> {
    use std::os::unix::fs::FileTypeExt;
    if let Some(parent) = socket.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    // Keep the lock inode for the listener's lifetime, including the stale-socket probe.
    let mut lock_path = socket.as_os_str().to_os_string();
    lock_path.push(".lock");
    let lock = std::fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(lock_path)?;
    lock.try_lock()
        .context("another mediad owns the frame socket")?;
    let listener = match UnixListener::bind(socket) {
        Ok(listener) => listener,
        Err(error) if error.kind() == std::io::ErrorKind::AddrInUse => {
            if !std::fs::symlink_metadata(socket)?.file_type().is_socket() {
                return Err(error.into());
            }
            match tokio::time::timeout(Duration::from_secs(1), UnixStream::connect(socket)).await {
                Ok(Err(probe)) if probe.kind() == std::io::ErrorKind::ConnectionRefused => {
                    std::fs::remove_file(socket)?;
                    UnixListener::bind(socket)?
                }
                _ => return Err(error.into()),
            }
        }
        Err(error) => return Err(error.into()),
    };
    std::fs::set_permissions(socket, std::fs::Permissions::from_mode(SOCKET_MODE))
        .with_context(|| format!("setting permissions on {}", socket.display()))?;
    if let Err(error) = give_to_group(socket, GROUP) {
        // Not fatal, and said out loud with what it means: the socket exists, and only `mediad`
        // and root can reach it. On a board that is a broken install; on a laptop it is a machine
        // with no `robot` group, which is ordinary.
        tracing::warn!(
            error = %error, group = GROUP, socket = %socket.display(),
            "media.frame stays private to mediad — nothing else can ask for a picture"
        );
    }
    tracing::info!(
        path = %socket.display(),
        mode = format!("{SOCKET_MODE:o}"),
        "serving media.frame locally"
    );

    Ok((lock, listener))
}

/// Bound both simultaneous clients and the lifetime of silent or slow clients.
///
/// `rotate` is degrees clockwise the camera is mounted from upright, reported in every frame
/// header. It is the caller's business whether the pipeline already applied it.
pub async fn serve(listener: UnixListener, frames: Frames, video: Video) -> Result<()> {
    let video = Arc::new(video);
    let slots = Arc::new(tokio::sync::Semaphore::new(16));
    loop {
        match listener.accept().await {
            Ok((stream, _)) => {
                let Ok(permit) = slots.clone().try_acquire_owned() else {
                    continue;
                };
                let frames = frames.clone();
                let video = video.clone();
                tokio::spawn(async move {
                    let _permit = permit;
                    let _ = handle(stream, frames, &video).await;
                });
            }
            Err(error) => tracing::warn!(error = %error, "media.frame accept failed"),
        }
    }
}

async fn handle(stream: UnixStream, frames: Frames, video: &Video) -> Result<()> {
    let (read, mut write) = stream.into_split();
    // Bounded *before* the line is buffered. Checking the length afterwards would mean a client
    // could make this process hold an arbitrarily long line first, which is the thing the cap is
    // for. One byte over the cap is read so that "too large" stays distinguishable from a request
    // that exactly fills it.
    let mut reader = BufReader::new(read);
    loop {
        let mut line = Vec::new();
        let read = tokio::time::timeout(Duration::from_secs(5), (&mut reader)
            .take(MAX_REQUEST_BYTES as u64 + 1)
            .read_until(b'\n', &mut line)).await??;
        if read == 0 {
            return Ok(());
        }
        if line.len() > MAX_REQUEST_BYTES {
            write_response(
                &mut write,
                proto::Response::err(
                    None,
                    proto::Error::new(proto::code::INVALID_PARAMS, "request is too large"),
                ),
            )
            .await?;
            return Ok(());
        }
        let request: proto::Request = match serde_json::from_slice(&line) {
            Ok(request) => request,
            Err(error) => {
                write_response(
                    &mut write,
                    proto::Response::err(
                        None,
                        proto::Error::new(proto::code::PARSE_ERROR, error.to_string()),
                    ),
                )
                .await?;
                return Ok(());
            }
        };
        if request.method == proto::method::HELLO {
            write_response(
                &mut write,
                proto::Response::ok(
                    request.id,
                    &proto::HelloResult {
                        api_version: proto::API_VERSION,
                        daemon_version: proto::semver::Version::parse(env!("CARGO_PKG_VERSION"))
                            .ok(),
                        revision: proto::build_info!().revision.map(str::to_owned),
                    },
                ),
            )
            .await?;
            continue;
        }
        if request.method == VIDEO_METHOD {
            write_response(&mut write, proto::Response::ok(request.id, &video_params(video))).await?;
            continue;
        }
        if request.method != proto::method::MEDIA_FRAME {
            write_response(
                &mut write,
                proto::Response::err(
                    request.id,
                    proto::Error::new(
                        proto::code::METHOD_NOT_FOUND,
                        format!("{} is not served by mediad", request.method),
                    ),
                ),
            )
            .await?;
            continue;
        }
        // `next_frame` registers the demand and parks on a condvar until the capture that answers it
        // lands, so it cannot run on the runtime's thread.
        let capture = frames.clone();
        let frame = tokio::task::spawn_blocking(move || capture.next_frame()).await?;
        let Some(frame) = frame else {
            write_response(
                &mut write,
                proto::Response::err(
                    request.id,
                    proto::Error::new(
                        proto::code::INTERNAL_ERROR,
                        "no frame arrived within the capture timeout",
                    ),
                ),
            )
            .await?;
            return Ok(());
        };
        let captured_at_unix_us = frame
            .captured_at
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros();
        let header = proto::MediaFrameHeader {
            width: frame.width,
            height: frame.height,
            format: frame.format.to_owned(),
            bytes: frame.data.len(),
            captured_at_unix_us,
            rotate: video.rotate,
        };
        tokio::time::timeout(Duration::from_secs(5), async {
            write_response(&mut write, proto::Response::ok(request.id, &header)).await?;
            write.write_all(&frame.data).await?;
            write.flush().await?;
            Ok::<_, anyhow::Error>(())
        }).await??;
        // Keep the local perception connection for its next fresh-frame request.
        // Capture remains demand-driven; an idle connection requests no frames.
    }
}

/// Hand the socket to `GROUP`. Mirrors `tof`'s stream and `padd`'s tap, including that a missing
/// group is a warning rather than a failure.
fn give_to_group(socket: &Path, group: &str) -> std::io::Result<()> {
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;

    let name = CString::new(group).map_err(std::io::Error::other)?;
    // SAFETY: `getgrnam` reads the group database and returns a pointer into storage it owns. The
    // name is a valid C string for the length of the call, and nothing else in this process calls
    // into the group database.
    let entry = unsafe { libc::getgrnam(name.as_ptr()) };
    if entry.is_null() {
        return Err(std::io::Error::other(format!(
            "no {group} group on this system"
        )));
    }
    // SAFETY: checked non-null immediately above, and `struct group` is fully initialised by
    // `getgrnam` when it returns a pointer at all.
    let gid = unsafe { (*entry).gr_gid };

    let path = CString::new(socket.as_os_str().as_bytes()).map_err(std::io::Error::other)?;
    // SAFETY: a valid C string path; `-1` for the owner is the documented "leave it alone".
    if unsafe { libc::chown(path.as_ptr(), u32::MAX, gid) } != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

async fn write_response(
    write: &mut tokio::net::unix::OwnedWriteHalf,
    response: proto::Response,
) -> Result<()> {
    let mut line = serde_json::to_vec(&response)?;
    line.push(b'\n');
    write.write_all(&line).await?;
    write.flush().await?;
    Ok(())
}
