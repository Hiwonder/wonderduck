//! A status UART read must outlive the Studio connection if its reply is unknown.

use std::io::{self, Write};
use std::path::Path;

/// A physical bus transaction survives daemon and OS restarts until it is confirmed.
/// Simulations use a socket-adjacent file and must never block a physical robot.
pub(crate) fn journal_path(socket: &Path, physical: bool) -> std::path::PathBuf {
    if physical {
        std::path::PathBuf::from("/var/lib/robotd-status-read.pending")
    } else {
        socket.with_extension("status-read.pending")
    }
}

pub(crate) fn pending(path: &Path) -> io::Result<bool> {
    match std::fs::symlink_metadata(path) {
        Ok(_) => Ok(true),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(false),
        Err(error) => Err(error),
    }
}

pub(crate) fn recover_inflight(_path: &Path, _port: &str) -> io::Result<()> {
    // With no transaction ID or documented maximum response delay, silence
    // cannot establish whether a device may still reply. The old reader may
    // also have consumed part of that reply before it disconnected. Keep the
    // bus parked even if someone removes the file while the process is running.
    Err(io::Error::other("unconfirmed status reply; motor bus remains locked"))
}

fn boot_id() -> io::Result<String> {
    let value = std::fs::read_to_string("/proc/sys/kernel/random/boot_id")?;
    let value = value.trim();
    let bytes = value.as_bytes();
    if bytes.len() != 36
        || bytes.iter().enumerate().any(|(index, byte)| {
            if matches!(index, 8 | 13 | 18 | 23) {
                *byte != b'-'
            } else {
                !byte.is_ascii_hexdigit()
            }
        })
    {
        return Err(io::Error::other("invalid Linux boot ID"));
    }
    Ok(value.to_ascii_lowercase())
}

fn create_journal(path: &Path, details: &str) -> io::Result<()> {
    use std::os::unix::fs::OpenOptionsExt;
    // If the boot identity cannot be recorded, do not begin a bus transaction.
    let boot_id = boot_id()?;
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(path)?;
    // Leave even an incomplete file in place on write failure: it is a stop marker.
    write!(file, "version=1\nboot_id={boot_id}\n{details}\n")?;
    file.sync_all()?;
    std::fs::File::open(path.parent().ok_or_else(|| io::Error::other("missing journal parent"))?)?.sync_all()
}

pub(crate) fn begin(path: &Path, id: u8, address: u8, length: u8) -> io::Result<()> {
    create_journal(
        path,
        &format!("kind=read\nid={id}\naddress={address}\nlength={length}"),
    )
}

/// Persist one marker for the complete individual-observation session. Any
/// crash during the session is treated as an unconfirmed transaction.
pub(crate) fn begin_session(path: &Path) -> io::Result<()> {
    create_journal(path, "kind=session")
}

pub(crate) fn complete(path: &Path) -> io::Result<()> {
    std::fs::remove_file(path)?;
    std::fs::File::open(path.parent().ok_or_else(|| io::Error::other("missing journal parent"))?)?.sync_all()
}
