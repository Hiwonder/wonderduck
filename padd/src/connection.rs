//! Bounded IPC operations. A failed exchange invalidates the session.

use std::io::{self, BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::Path;
use std::time::{Duration, Instant};

const IO_TIMEOUT: Duration = Duration::from_millis(250);
const MAX_RESPONSE: usize = 1024 * 1024;

#[cfg(target_os = "linux")]
fn connect_socket(path: &Path) -> io::Result<UnixStream> {
    use std::os::fd::{AsRawFd, FromRawFd};
    use std::os::unix::ffi::OsStrExt;

    let name = path.as_os_str().as_bytes();
    let mut address: libc::sockaddr_un = unsafe { std::mem::zeroed() };
    if name.is_empty() || name.contains(&0) || name.len() >= address.sun_path.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "invalid robotd socket path",
        ));
    }
    address.sun_family = libc::AF_UNIX as libc::sa_family_t;
    for (slot, byte) in address.sun_path.iter_mut().zip(name) {
        *slot = *byte as libc::c_char;
    }
    // A nonblocking connect also bounds retries when the listener's accept queue is full.
    let fd = unsafe {
        libc::socket(
            libc::AF_UNIX,
            libc::SOCK_STREAM | libc::SOCK_NONBLOCK | libc::SOCK_CLOEXEC,
            0,
        )
    };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    let stream = unsafe { UnixStream::from_raw_fd(fd) };
    let len = std::mem::offset_of!(libc::sockaddr_un, sun_path) + name.len() + 1;
    let result = unsafe {
        libc::connect(
            fd,
            (&address as *const libc::sockaddr_un).cast(),
            len as libc::socklen_t,
        )
    };
    if result < 0 {
        let error = io::Error::last_os_error();
        if error.raw_os_error() != Some(libc::EINPROGRESS) {
            return Err(error);
        }
        let deadline = Instant::now() + IO_TIMEOUT;
        let mut poll = libc::pollfd {
            fd: stream.as_raw_fd(),
            events: libc::POLLOUT,
            revents: 0,
        };
        loop {
            let millis = remaining(deadline)?.as_millis().max(1) as i32;
            let result = unsafe { libc::poll(&mut poll, 1, millis) };
            if result > 0 {
                break;
            }
            if result == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "robotd connection deadline expired",
                ));
            }
            let error = io::Error::last_os_error();
            if error.kind() != io::ErrorKind::Interrupted {
                return Err(error);
            }
        }
        let mut socket_error: libc::c_int = 0;
        let mut len = std::mem::size_of_val(&socket_error) as libc::socklen_t;
        if unsafe {
            libc::getsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_ERROR,
                (&mut socket_error as *mut libc::c_int).cast(),
                &mut len,
            )
        } < 0
        {
            return Err(io::Error::last_os_error());
        }
        if socket_error != 0 {
            return Err(io::Error::from_raw_os_error(socket_error));
        }
    }
    stream.set_nonblocking(false)?;
    Ok(stream)
}

#[cfg(not(target_os = "linux"))]
fn connect_socket(path: &Path) -> io::Result<UnixStream> {
    UnixStream::connect(path)
}

pub fn connect(path: &Path) -> io::Result<UnixStream> {
    let stream = connect_socket(path)?;
    stream.set_read_timeout(Some(IO_TIMEOUT))?;
    stream.set_write_timeout(Some(IO_TIMEOUT))?;
    Ok(stream)
}

fn remaining(deadline: Instant) -> io::Result<Duration> {
    deadline
        .checked_duration_since(Instant::now())
        .filter(|left| !left.is_zero())
        .ok_or_else(|| io::Error::new(io::ErrorKind::TimedOut, "robotd IPC deadline expired"))
}

fn write_until(stream: &mut UnixStream, mut bytes: &[u8], deadline: Instant) -> io::Result<()> {
    while !bytes.is_empty() {
        stream.set_write_timeout(Some(remaining(deadline)?))?;
        match stream.write(bytes) {
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::WriteZero,
                    "robotd IPC write failed",
                ));
            }
            Ok(written) => bytes = &bytes[written..],
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

pub fn send(stream: &mut UnixStream, bytes: &[u8]) -> io::Result<()> {
    write_until(stream, bytes, Instant::now() + IO_TIMEOUT)
}

pub fn exchange(
    stream: &mut UnixStream,
    request: &duck_ipc_proto::Request,
) -> io::Result<duck_ipc_proto::Response> {
    exchange_inner(stream, request, None, IO_TIMEOUT)
}

/// A durable recovery can legitimately take longer than a control frame.
pub fn exchange_with_timeout(
    stream: &mut UnixStream,
    request: &duck_ipc_proto::Request,
    timeout: Duration,
) -> io::Result<duck_ipc_proto::Response> {
    exchange_inner(stream, request, None, timeout)
}

pub fn exchange_with_heartbeat(
    stream: &mut UnixStream,
    request: &duck_ipc_proto::Request,
    heartbeat: &duck_ipc_proto::Request,
    period: Duration,
) -> io::Result<duck_ipc_proto::Response> {
    exchange_inner(stream, request, Some((heartbeat, period)), IO_TIMEOUT)
}

fn exchange_inner(
    stream: &mut UnixStream,
    request: &duck_ipc_proto::Request,
    heartbeat: Option<(&duck_ipc_proto::Request, Duration)>,
    timeout: Duration,
) -> io::Result<duck_ipc_proto::Response> {
    let deadline = Instant::now() + timeout;
    let heartbeat = heartbeat
        .map(|(request, period)| {
            let mut bytes = serde_json::to_vec(request)?;
            bytes.push(b'\n');
            Ok::<_, io::Error>((bytes, period))
        })
        .transpose()?;
    if let Some((bytes, _)) = &heartbeat {
        write_until(stream, bytes, deadline)?;
    }
    let mut line = serde_json::to_vec(request)?;
    line.push(b'\n');
    write_until(stream, &line, deadline)?;
    line.clear();
    let mut next_heartbeat = heartbeat
        .as_ref()
        .map(|(_, period)| Instant::now() + *period);
    let mut reader = BufReader::new(stream);
    loop {
        let time_left = remaining(deadline)?;
        if let (Some((bytes, period)), Some(next)) = (&heartbeat, next_heartbeat) {
            if Instant::now() >= next {
                write_until(reader.get_mut(), bytes, deadline)?;
                next_heartbeat = Some(Instant::now() + *period);
                continue;
            }
        }
        let timeout = next_heartbeat.map_or(time_left, |next| {
            time_left.min(next.saturating_duration_since(Instant::now()))
        });
        if timeout.is_zero() {
            continue;
        }
        reader.get_mut().set_read_timeout(Some(timeout))?;
        let buffer = match reader.fill_buf() {
            Ok(buffer) => buffer,
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error)
                if heartbeat.is_some()
                    && matches!(
                        error.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
            {
                continue;
            }
            Err(error) => return Err(error),
        };
        if buffer.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "robotd disconnected",
            ));
        }
        let count = buffer
            .iter()
            .position(|byte| *byte == b'\n')
            .map_or(buffer.len(), |i| i + 1);
        if line.len() + count > MAX_RESPONSE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "robotd response too large",
            ));
        }
        line.extend_from_slice(&buffer[..count]);
        let complete = line.last() == Some(&b'\n');
        reader.consume(count);
        if complete {
            break;
        }
    }
    let response: duck_ipc_proto::Response = serde_json::from_slice(&line)?;
    if response.id != request.id {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "robotd response ID mismatch",
        ));
    }
    Ok(response)
}
