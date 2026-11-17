//! Who is allowed to talk to this robot over BLE.
//!
//! §4.2 says BLE authorisation is "physical presence + pairing", and §7 requires the
//! characteristic carrying wifi credentials to be paired and encrypted. Both hold — but the PIN
//! check lives **above** the link layer, and that is forced rather than chosen.
//!
//! ## Why BLE could not do this
//!
//! The first design had the robot answer BlueZ's passkey request with its stored PIN. That cannot
//! work on a headless robot. In LE passkey entry one side *displays* a passkey and the other
//! *inputs* it, and the roles follow from the IO capabilities each side declares. Implementing
//! `request_passkey` declares "this device can input", so macOS took the display role, generated a
//! random six-digit code, and waited for someone to type it into a robot with no keyboard.
//!
//! The reverse fails too: with `DisplayPasskey` the robot takes the display role, but **BlueZ
//! generates the passkey** — the spec has the displaying side choose it at random. A fixed PIN
//! printed on a sticker is simply not expressible in BLE passkey entry.
//!
//! ## So: just-works pairing, plus a PIN the transport checks
//!
//! Pairing is just-works (all agent handlers `None`, which BlueZ reads as `NoInputNoOutput`), so
//! the link is encrypted but **not** authenticated. The read on the RPC characteristic requires
//! encryption, which is what triggers the bond. Then `btd` serves nothing until the client proves
//! the PIN via `system.authenticate`, which is why that call is answered by the transport rather
//! than forwarded.
//!
//! The cost, stated plainly: the PIN crosses an encrypted-but-unauthenticated link, so an attacker
//! present *at the moment of pairing* could capture it. The alternatives were no authentication at
//! all, or an out-of-band QR flow that BlueZ barely supports and no phone app exists to drive. For
//! a robot in a home this is the better trade, and it is revisitable without touching the
//! transport, because the check is ours now rather than the spec's.
//!
//! **The factory PIN is `000000` and everyone can read it in this repository.** So out of the box
//! this proves physical presence and nothing more, which is the same guarantee just-works pairing
//! gives — the difference being that the mechanism, the storage and the six-digit contract are
//! all in place, so making it a real secret is a provisioning change rather than a redesign. A
//! per-robot PIN printed under the robot is what turns this into security, and that is
//! per-device state.
//!
//! ## No pairing window, and that is decided rather than deferred
//!
//! The robot is pairable whenever it advertises. A physical button-held window is the usual
//! answer, and it was considered and rejected: a **per-robot PIN already carries the property a
//! window would add.** If the PIN is unique and printed under the robot, knowing it requires
//! physical access — and anyone who can read the sticker can also pick the robot up. A window
//! would defend only against someone in range while the factory default is still in place, and
//! the answer to that is a real PIN, not a button.
//!
//! What a button would buy beyond this: a visible consent moment, a recovery path when the PIN is
//! lost, and defence in depth if a sticker is photographed. None is needed for v1, and each is
//! additive later — an enclosure with a button can gate `set_pairable` without changing anything
//! here.
//!
//! So the security of this rests entirely on the PIN being per-robot, which makes it a
//! provisioning obligation rather than a software one: something has to generate it, print it and
//! record what was printed. The robot does now have a per-device identity — `configd::identity`
//! derives one from the SoC serial — but a PIN cannot be derived from it, and that is worth stating
//! rather than rediscovering: the identity is *published*, in an advertisement anyone in range can
//! collect, so a PIN computed from it would be public the moment the derivation is known. Only the
//! name hangs off the identity. A secret still has to be generated, printed and recorded, which is
//! per-device state.
//!
//! Still open, and smaller: **no bond management.** Every paired phone stays paired and nothing
//! revokes one; `bluetoothctl untrust` is the manual escape until there is an API for it.

use std::time::Duration;

use duck_ipc_proto as proto;

/// How long to wait for `configd` to answer with the PIN.
///
/// Short: BlueZ is holding a pairing exchange open, and a phone shows a spinner while we decide.
/// If `configd` cannot answer in this long it is not going to.
const PIN_TIMEOUT: Duration = Duration::from_secs(3);

/// Ask `configd` for the pairing PIN.
///
/// Fetched **per pairing request** rather than cached at startup, so `robotctl system set-pin`
/// takes effect on the next pairing rather than the next reboot. One socket round-trip during an
/// exchange that already takes a human several seconds.
///
/// Returned whole rather than parsed: the comparison is on the string, because `000042` and `42`
/// are different PINs and a numeric parse would make them the same. `is_default` comes with it so
/// the caller can say out loud that a factory PIN authenticates anyone who read this repository.
///
/// The PIN is never logged. It is barely a secret today, but a per-robot one is meant to be, and
/// the journal is the wrong place for it.
pub async fn pin(config_socket: &std::path::Path) -> Result<proto::PairingPinResult, String> {
    crate::upstream::ask(
        "configd",
        config_socket,
        &proto::Call::SystemPairingPin,
        PIN_TIMEOUT,
    )
    .await?
    .result_as()
    .map_err(|e| e.to_string())
}
