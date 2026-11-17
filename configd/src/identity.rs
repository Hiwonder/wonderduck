//! Per-device identity, and the default name derived from it.
//!
//! The SoC serial provides a stable default name for distinguishing devices before renaming them.
//! It remains available across reflashes and radio replacements, unlike an adapter address, and
//! can be read from the device tree without waiting for Bluetooth to initialise.

use std::path::Path;

use sha2::{Digest, Sha256};

/// Where the bootloader leaves the SoC serial.
///
/// `/proc/device-tree/serial-number` rather than `/proc/cpuinfo`: both report the same value on
/// this board, but the devicetree property is a generic binding that any
/// board populating it satisfies, while `cpuinfo`'s `Serial` line is a per-architecture quirk of
/// how the kernel chooses to print it. A future board on a different SoC is more likely to keep
/// the first.
const SERIAL_PATH: &str = "/proc/device-tree/serial-number";

/// The prefix every derived name carries.
///
/// Hardcoded rather than built from the hostname, which would give `radxa-zero3-7f3a` — longer
/// than what it replaces and no more meaningful. The robot is a duck.
const NAME_PREFIX: &str = "duck";

/// The board's serial number, or `None` where there is not one to read.
///
/// `None` on a macOS laptop, and on any board whose bootloader does not fill the property in. The
/// caller falls back rather than failing: a robot with no serial must still come up with a working
/// name.
pub fn serial() -> Option<String> {
    serial_at(Path::new(SERIAL_PATH))
}

/// [`serial`], against an arbitrary path so it can be tested off a board.
pub fn serial_at(path: &Path) -> Option<String> {
    let raw = std::fs::read(path).ok()?;
    let text = String::from_utf8_lossy(&raw);

    // Devicetree properties are NUL-terminated strings, and a property may hold several — so the
    // first NUL ends the value rather than being trailing junk to trim.
    let serial = text.split('\0').next()?.trim();

    // A property that exists but is blank, or full of control characters, is no identity at all,
    // and it also reaches a client verbatim through `system.info`.
    if serial.is_empty() || !serial.chars().all(|c| c.is_ascii_graphic()) {
        return None;
    }
    Some(serial.to_owned())
}

/// The name a robot answers to before anyone has renamed it: `duck-7f3a`.
///
/// **Hashed rather than sliced.** Taking the last four characters of the serial would assume that
/// is where two chips differ, and nothing guarantees it — a vendor is free to make any part of the
/// value sequential, or constant. A digest spreads whatever entropy the serial has across the
/// whole output.
///
/// **SHA-256 rather than [`std::hash::DefaultHasher`]**, whose output is explicitly not stable
/// across Rust releases. A `rustup update` would silently rename every robot in the field, and
/// nobody would ever connect the two.
///
/// Four hex characters, so 65 536 possibilities: three robots in a room share a name about once in
/// 22 000 times. This is a *default* meant to be distinguishable, not a unique key — `system.setName`
/// from a phone and `robotctl system set-name` from provisioning both override it, and renaming is
/// the escape hatch on the rare collision.
pub fn default_name(serial: &str) -> String {
    let digest = Sha256::digest(serial.as_bytes());
    format!("{NAME_PREFIX}-{:02x}{:02x}", digest[0], digest[1])
}
