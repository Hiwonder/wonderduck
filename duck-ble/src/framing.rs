//! Turning a GATT characteristic into a line-oriented byte pipe.
//!
//! The control plane is NDJSON — one JSON object per line — over every transport.
//! BLE is the only one where a message does not fit in a single
//! write: usable payload is `ATT_MTU - 3`, which on a phone that never renegotiates is 20
//! bytes, and on a good link is a few hundred. So a line arrives in pieces and leaves in
//! pieces.
//!
//! **There is no framing header.** The newline that already separates messages is the frame
//! delimiter, in both directions. That is safe rather than lucky: `serde_json` escapes a
//! newline inside a string as `\n`, so a raw `0x0A` never appears inside a serialised JSON
//! object — which is the same property that makes NDJSON work on the unix socket.
//!
//! Adding a length prefix would mean a second framing dialect that only BLE speaks, and
//! every client would have to implement it. A phone app can instead do exactly what
//! `robotctl` does: write bytes, read until newline.

/// Longest line we will reassemble before giving up on a peer.
///
/// A BLE client that never sends a newline would otherwise grow this buffer without bound,
/// and it is reachable by anyone in radio range. Generous next to any real request — the
/// largest is an `update.apply` with a long ref — and far below `updaterd`'s own 1 MiB line
/// limit, because nothing that big has any business arriving over BLE.
///
/// **This bounds what a peer may send, not what the robot may answer.** The two are not the
/// same size and were one constant until a reply outgrew it: see [`MAX_REPLY_LINE`].
pub const MAX_LINE: usize = 8 * 1024;

/// Longest line a *client* will reassemble from the robot.
///
/// The other direction, and it needs its own number. [`MAX_LINE`] is a security bound — the
/// peer it limits is anyone in radio range, so it is deliberately tight — whereas this limits
/// the robot a client chose to connect to and already trusts. Reusing the tight one here made
/// every reply over 8 KiB unreadable, which quietly took `update.show` (kilobytes for an
/// ordinary run) and `system.logs` (a screenful of journal) out of reach of `duckctl` while
/// `btd` served both correctly.
///
/// 64 KiB, and the real limit on a reply is smaller still and lives elsewhere: `proto`'s
/// `MAX_LOG_BYTES` keeps the journal tail well inside this, because at a typical ATT MTU
/// 64 KiB is a minute on the air and no answer should take that long.
pub const MAX_REPLY_LINE: usize = 64 * 1024;

/// Notification payload a session starts with, before any write has reported the negotiated one.
///
/// 20 bytes is what every BLE link is required to support, so it is the only safe *first* guess.
pub const FLOOR_MTU: usize = 20;

/// The largest a GATT characteristic value may be, whatever the MTU says.
///
/// **512 octets, from the Core specification** (Vol 3, Part F, §3.2.9), and it is not the same
/// number as the ATT MTU. An MTU of 517 *is* this limit — 512 bytes of value plus five bytes of
/// ATT overhead — which is why 517 is the figure every stack converges on, and why subtracting
/// only the three-byte notification header from it overshoots by two.
pub const MAX_ATTRIBUTE_VALUE: usize = 512;

/// How large a notification this link will actually carry.
///
/// `mtu - 3` is the ATT arithmetic and it is not sufficient on its own: against the 517-byte MTU
/// a CoreBluetooth central negotiates it yields 514, and a central handed a 514-byte value keeps
/// the first 512 and discards the rest. **Silently** — no error on either side.
///
/// What that looked like before it was understood: two bytes disappearing from the middle of
/// every reply longer than one chunk, so `{"security":"wpa_psk"}` arrived as
/// `{"serity":"wpa_psk"}` and a client failed to parse a field the robot had sent correctly.
/// Worse when the missing pair happened to include the newline, because then the line never
/// completed and the call waited out its budget against a robot that had already answered. Single
/// chunk replies — `net.status`, `system.info`, `hello` — were fine throughout, which is what
/// made it look like a problem with the larger methods rather than with every boundary.
pub fn notification_payload(mtu: u16) -> usize {
    // `clamp` rather than `min` then `max`: the two constants are ordered by construction, and
    // clippy is right that spelling it out this way says so.
    usize::from(mtu)
        .saturating_sub(3)
        .clamp(FLOOR_MTU, MAX_ATTRIBUTE_VALUE)
}

/// Reassembles inbound chunks into whole lines.
#[derive(Debug)]
pub struct Reassembler {
    buf: Vec<u8>,
    /// What "too long" means for this direction. See [`MAX_LINE`] and [`MAX_REPLY_LINE`].
    limit: usize,
}

/// The robot's side: [`MAX_LINE`], the tight bound, because the default must be the safe one.
impl Default for Reassembler {
    fn default() -> Self {
        Self {
            buf: Vec::new(),
            limit: MAX_LINE,
        }
    }
}

/// Why a peer's bytes were rejected. Both cases mean "drop the connection", but they are
/// logged differently: one is a client that cannot frame, the other may be an attack.
#[derive(Debug, PartialEq, Eq)]
pub enum FramingError {
    /// No newline within this reassembler's limit, which the variant carries because the two
    /// directions have different ones and the message must say which was hit.
    LineTooLong { limit: usize },
    /// Not valid UTF-8, so it cannot be JSON either.
    NotUtf8,
}

impl std::fmt::Display for FramingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::LineTooLong { limit } => write!(f, "no newline within {limit} bytes"),
            Self::NotUtf8 => write!(f, "not valid UTF-8"),
        }
    }
}

// A real error, not just a Debug-able enum: this crate is a library, and a client using it — the
// `duckctl` example is the first — wants `?` to work.
impl std::error::Error for FramingError {}

impl Reassembler {
    pub fn new() -> Self {
        Self::default()
    }

    /// A reassembler for the answers coming *back*, bounded by [`MAX_REPLY_LINE`].
    ///
    /// Named for the direction rather than taking a number, so that a client reading replies
    /// cannot pick the wrong bound and `btd` cannot accidentally be given the loose one.
    pub fn for_replies() -> Self {
        Self {
            buf: Vec::new(),
            limit: MAX_REPLY_LINE,
        }
    }

    /// Feed one chunk; get back every complete line it completed.
    ///
    /// Returns a `Vec` because one write can legitimately carry several short lines — a
    /// client that batches `hello` and `update.status` into one 40-byte write is being
    /// efficient, not wrong.
    pub fn push(&mut self, chunk: &[u8]) -> Result<Vec<String>, FramingError> {
        if self.buf.len() + chunk.len() > self.limit {
            // Clear rather than keep the partial line: whatever follows is unparseable
            // anyway, and holding it would let a peer pin the memory.
            self.buf.clear();
            return Err(FramingError::LineTooLong { limit: self.limit });
        }
        self.buf.extend_from_slice(chunk);

        let mut lines = Vec::new();
        while let Some(at) = self.buf.iter().position(|&b| b == b'\n') {
            let line = self.buf.drain(..=at).collect::<Vec<u8>>();
            // Drop the newline, and tolerate CRLF from a client that helpfully added one.
            let line = &line[..line.len() - 1];
            let line = line.strip_suffix(b"\r").unwrap_or(line);

            if line.is_empty() {
                continue;
            }
            match std::str::from_utf8(line) {
                Ok(text) => lines.push(text.to_owned()),
                Err(_) => {
                    self.buf.clear();
                    return Err(FramingError::NotUtf8);
                }
            }
        }
        Ok(lines)
    }

    /// Bytes held pending a newline. For logging a peer that has gone quiet mid-line.
    pub fn pending(&self) -> usize {
        self.buf.len()
    }
}

/// Split one outbound line into notification-sized chunks, newline included.
///
/// The newline is part of the payload rather than a separate final chunk: a client that
/// reassembles until newline then needs no notion of "end of message" beyond the one it
/// already has.
pub fn chunks(line: &str, mtu: usize) -> Vec<Vec<u8>> {
    // A zero or absurd MTU would divide by zero or emit one chunk per byte.
    let mtu = mtu.max(FLOOR_MTU);

    let mut bytes = line.as_bytes().to_vec();
    if !bytes.ends_with(b"\n") {
        bytes.push(b'\n');
    }
    bytes.chunks(mtu).map(<[u8]>::to_vec).collect()
}
