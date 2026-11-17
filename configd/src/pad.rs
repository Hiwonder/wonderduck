//! Gamepad configuration, exposed through a trait and implemented by BlueZ.
//!
//! Pairing requires access to the Bluetooth adapter and remains available independently of robot
//! motion control. `padd` reads controller input and forwards intents to `robotd` without needing
//! to configure the adapter. `crate::bluez` handles the pairing order for each transport.

use std::time::Duration;

use async_trait::async_trait;
use duck_ipc_proto as proto;

/// What went wrong, in terms a caller can act on.
pub type PadResult<T> = Result<T, String>;

/// How long to look for a pad when the caller does not say.
///
/// Fifteen seconds because someone is standing there holding a sync button, and that is about how
/// long a person will wait before concluding it did not work. Long enough for BlueZ to report a
/// device that only advertises every few seconds; short enough that a phone gets an answer rather
/// than a spinner.
pub const DEFAULT_PAIR_TIMEOUT: Duration = Duration::from_secs(15);

/// Longest window a caller may ask for.
///
/// A cap rather than a courtesy: discovery is on for the whole window, and a client that asked for
/// an hour would leave the adapter scanning long after whoever typed it walked away.
pub const MAX_PAIR_TIMEOUT: Duration = Duration::from_secs(120);

#[async_trait]
pub trait Pads: Send + Sync {
    /// Every pad this robot is bonded to, connected first.
    async fn status(&self) -> PadResult<Vec<proto::Pad>>;

    /// Pair whatever pad is in pairing mode, or the one at `mac`.
    ///
    /// A *refusal* — nothing found, two candidates, BlueZ said no — is an `Ok` carrying
    /// [`proto::PadPairResult::Failed`]. `Err` is reserved for the machinery being broken, which is
    /// the same split [`crate::net::Net`] uses and the same one the dispatcher turns into either a
    /// result or an `INTERNAL_ERROR`.
    async fn pair(&self, mac: Option<&str>, timeout: Duration) -> PadResult<proto::PadPairResult>;

    /// Drop the bond, so this pad stops reconnecting.
    async fn forget(&self, mac: &str) -> PadResult<proto::PadForgetResult>;
}

/// Clamp a caller's timeout into something the adapter should be asked to do.
pub fn pair_timeout(requested: Option<u32>) -> Duration {
    match requested {
        // Zero is "look once", not "look forever": a scripted retry loop should be able to ask
        // whether a pad is there right now without holding discovery open.
        Some(seconds) => Duration::from_secs(u64::from(seconds)).min(MAX_PAIR_TIMEOUT),
        None => DEFAULT_PAIR_TIMEOUT,
    }
}

/// Is this Bluetooth device a gamepad?
///
/// Four signals, because no single one is present on every pad, and getting this wrong in either
/// direction is bad in a different way: too narrow and the pad in pairing mode is invisible, too
/// broad and `pad.pair` bonds the robot to a colleague's headphones.
///
///  - **`icon`** is BlueZ's own classification, derived from the class or the appearance. When it
///    says `input-gaming` the question is settled, and this is the signal an Xbox controller
///    actually presents on this board — from its LE appearance, since that pad has no class.
///  - **`class`** is the BR/EDR class-of-device: bits 8-12 are the major device class, and `0x05`
///    is Peripheral. Bits 6-7 of the minor field distinguish keyboard from pointing device from
///    gamepad — `0x01` in bits 2-5 with the keyboard/pointer bits clear is a joystick or gamepad.
///    Present for a classic pad, absent for a BLE-only one. A no-name "Pro Controller" (a Switch
///    Pro clone) presents `0x2508` — peripheral, gamepad — and this is the arm that names it when
///    BlueZ has not yet derived the icon from it.
///  - **`appearance`** is the BLE equivalent: category 15 (`0x03C0..=0x03C4`) is HID, and `0x03C4`
///    is specifically Gamepad. Many pads never set it, which is why it cannot stand alone. Only the
///    gamepad value counts, so an LE pad advertising generic HID falls through to its name.
///  - **the name**, last and deliberately: it is the signal that works when the other three are
///    absent, which for a pad still in pairing mode is common, and it is the one that can be wrong.
///
/// A device that satisfies none of these is not offered as a pad. `mac` is how someone pairs
/// hardware this does not recognise, which is the escape hatch that keeps the heuristic from being
/// load-bearing.
pub fn looks_like_a_gamepad(
    name: &str,
    icon: Option<&str>,
    class: Option<u32>,
    appearance: Option<u16>,
) -> bool {
    gamepad_evidence(name, icon, class, appearance).is_some()
}

/// How a device came to look like a gamepad — and how much that is worth.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Evidence {
    /// The radio said so: BlueZ's icon, the class-of-device or the gamepad appearance. Settled.
    Classified,
    /// Only the name said so. Enough to pair when nothing better turns up, and not enough to stop
    /// looking: the Pro Controller clone's LE face is `BLE Controller_000001` with nothing else
    /// set, and stopping on it is what cost every pairing thirty seconds and a dead object.
    NameOnly,
}

/// [`looks_like_a_gamepad`], with the strength of the answer. Same four signals, same order.
pub fn gamepad_evidence(
    name: &str,
    icon: Option<&str>,
    class: Option<u32>,
    appearance: Option<u16>,
) -> Option<Evidence> {
    if icon == Some("input-gaming") {
        return Some(Evidence::Classified);
    }

    if let Some(class) = class {
        let major = (class >> 8) & 0x1f;
        let minor = (class >> 2) & 0x3f;
        // Peripheral, with the joystick/gamepad/remote minor bits. 0x01 is joystick, 0x02 gamepad,
        // 0x03 remote control — the keyboard (0x10) and pointing-device (0x20) bits are the ones
        // this must not match, and they live above these values rather than overlapping them.
        if major == 0x05 && matches!(minor & 0x0f, 0x01 | 0x02) {
            return Some(Evidence::Classified);
        }
    }

    if let Some(appearance) = appearance {
        // 0x03C4 is Gamepad; the rest of category 15 is other HID. Only the gamepad value counts,
        // because a Bluetooth keyboard is category 15 too and must not be paired as a pad.
        if appearance == 0x03C4 {
            return Some(Evidence::Classified);
        }
    }

    // Last resort, and case-insensitive: BlueZ reports names as the device advertises them, and
    // pads are inconsistent about capitalisation.
    let lower = name.to_lowercase();
    [
        "controller",
        "gamepad",
        "joystick",
        "dualsense",
        "dualshock",
    ]
    .iter()
    .any(|needle| lower.contains(needle))
    .then_some(Evidence::NameOnly)
}

/// Are these two Bluetooth addresses two faces of one pad?
///
/// Some controllers expose LE and BR/EDR faces with identical lower three address octets. Match
/// these faces so discovery can prefer the classic HID device. Comparing three octets leaves the
/// vendor half free to differ while reducing collisions between units from the same vendor.
pub fn same_pad(a: &str, b: &str) -> bool {
    let tail = |mac: &str| -> Option<[u8; 3]> {
        let mut parts = mac.rsplit(':');
        let mut tail = [0u8; 3];
        for slot in tail.iter_mut().rev() {
            *slot = u8::from_str_radix(parts.next()?, 16).ok()?;
        }
        Some(tail)
    };
    match (tail(a), tail(b)) {
        (Some(a), Some(b)) => a == b,
        _ => false,
    }
}

/// A set of pads that exists only in memory.
///
/// Used by the tests and by `--fake-pads`, which is what makes the whole `pad.*` surface — and the
/// failures that are awkward to arrange with real hardware, like two pads in pairing mode at once
/// — exercisable from a laptop with no radio.
pub struct FakePads {
    inner: tokio::sync::Mutex<FakeState>,
}

struct FakeState {
    /// Every pad the radio can see, bonded or not — which is the shape BlueZ reports and the reason
    /// this is one list rather than two.
    ///
    /// Modelling "in pairing mode" and "already bonded" separately looked tidier and hid the bug
    /// that mattered: a bonded pad shows up in *every* sweep, so the selection rule has to prefer an
    /// unbonded one or a robot can never be given a second pad.
    visible: Vec<proto::Pad>,
}

impl FakePads {
    /// One pad in pairing mode, nothing bonded — a fresh robot next to a controller with its sync
    /// light flashing.
    pub fn new() -> Self {
        Self::with(vec![unpaired(
            "02:00:00:00:00:03",
            "Xbox Wireless Controller",
        )])
    }

    pub fn with(visible: Vec<proto::Pad>) -> Self {
        Self {
            inner: tokio::sync::Mutex::new(FakeState { visible }),
        }
    }
}

/// A pad in pairing mode: seen, not bonded.
pub fn unpaired(mac: &str, name: &str) -> proto::Pad {
    proto::Pad {
        mac: mac.to_owned(),
        name: name.to_owned(),
        paired: false,
        trusted: false,
        connected: false,
    }
}

/// A pad this robot already has, in range and connected.
pub fn bonded(mac: &str, name: &str) -> proto::Pad {
    proto::Pad {
        mac: mac.to_owned(),
        name: name.to_owned(),
        paired: true,
        trusted: true,
        connected: true,
    }
}

impl Default for FakePads {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Pads for FakePads {
    async fn status(&self) -> PadResult<Vec<proto::Pad>> {
        let state = self.inner.lock().await;
        Ok(state
            .visible
            .iter()
            .filter(|pad| pad.paired)
            .cloned()
            .collect())
    }

    async fn pair(&self, mac: Option<&str>, _timeout: Duration) -> PadResult<proto::PadPairResult> {
        let mut state = self.inner.lock().await;

        let candidates: Vec<proto::Pad> = state
            .visible
            .iter()
            .filter(|pad| mac.is_none_or(|wanted| wanted.eq_ignore_ascii_case(&pad.mac)))
            .cloned()
            .collect();

        // The same rule as `crate::bluez`: an unbonded pad wins, because that is the one someone
        // just put into pairing mode. A pad the robot already has is not a competing answer, and
        // treating it as one is what made a second pad impossible to add.
        let fresh: Vec<&proto::Pad> = candidates.iter().filter(|pad| !pad.paired).collect();
        let pad = match (fresh.as_slice(), candidates.as_slice()) {
            ([only], _) => (*only).clone(),
            ([], []) => {
                return Ok(proto::PadPairResult::Failed {
                    reason: proto::PadPairFailure::NotFound,
                    detail: None,
                });
            }
            // Nothing new in pairing mode, so the answer is the pad already bonded: an idempotent
            // re-run, which is also how a lost `Trusted` is repaired.
            ([], already) => already
                .iter()
                .min_by(|a, b| a.mac.cmp(&b.mac))
                .expect("non-empty")
                .clone(),
            (several, _) => {
                return Ok(proto::PadPairResult::Failed {
                    reason: proto::PadPairFailure::Ambiguous,
                    detail: Some(format!("{} pads are in pairing mode", several.len())),
                });
            }
        };

        let paired = proto::Pad {
            paired: true,
            trusted: true,
            connected: true,
            ..pad
        };
        state.visible.retain(|p| p.mac != paired.mac);
        state.visible.push(paired.clone());
        Ok(proto::PadPairResult::Paired { pad: paired })
    }

    async fn forget(&self, mac: &str) -> PadResult<proto::PadForgetResult> {
        let mut state = self.inner.lock().await;
        let before = state.visible.len();
        // `RemoveDevice` drops the object, not just the keys, so the pad is no longer visible at all
        // until something rediscovers it.
        state.visible.retain(|p| !p.mac.eq_ignore_ascii_case(mac));
        Ok(proto::PadForgetResult {
            removed: state.visible.len() != before,
        })
    }
}
