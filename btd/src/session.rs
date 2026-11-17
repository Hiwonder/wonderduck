//! One connected central, from `hello` to disconnect.
//!
//! This is the whole of `btd`'s behaviour, and it holds no state about the robot — only about
//! the conversation: a reassembly buffer and whichever upstream sockets this session has had
//! reason to open.
//!
//! A request is forwarded **verbatim**. `btd` parses each line only far enough to answer two
//! questions — is this method allowed here, and which socket owns it — and then passes the
//! original bytes on. It never rewrites `id`, never re-serialises params, and never invents a
//! result. That is what keeps it a transport rather than a second implementation of the API,
//! and it is why adding a protocol method costs one line in [`crate::route`] and nothing here.

use duck_ipc_proto as proto;
use tokio::sync::mpsc;

use crate::link::{Link, QUEUE};
use crate::pairing;
use crate::route::{self, Route};
use crate::upstream::{Pool, Sockets};
use duck_ble::framing::{self, Reassembler};

/// How many wrong PINs a session may offer before it is closed.
///
/// A six-digit PIN is a million guesses, and the link is encrypted but not authenticated, so
/// rationing attempts is the only thing standing between a peer in radio range and brute force.
/// Three, then the session ends: reconnecting costs a full BLE connect and bond, which turns an
/// afternoon of guessing into something far longer while staying invisible to a legitimate client
/// that mistypes twice.
const PIN_ATTEMPTS: u32 = 3;

/// Serve one central until it disconnects or breaks framing.
pub async fn run(mut link: Link, sockets: Sockets) {
    let peer = link.peer.clone();
    tracing::info!(peer = %peer, mtu = link.mtu(), "session opened; mtu is the floor \
     until the first write reports the negotiated one");

    let (replies_tx, mut replies) = mpsc::channel::<String>(QUEUE);
    let config_socket = sockets.config.clone();
    let mut pool = Pool::new(sockets, replies_tx);
    let mut inbound = Reassembler::new();

    // Nothing but `hello` and `system.authenticate` is served until the client proves the PIN.
    // See `crate::pairing` for why this is here rather than in the bond.
    let mut authenticated = false;
    let mut attempts_left = PIN_ATTEMPTS;

    loop {
        tokio::select! {
            // Bytes from the radio.
            chunk = link.inbound.recv() => {
                let Some(chunk) = chunk else { break };

                let lines = match inbound.push(&chunk) {
                    Ok(lines) => lines,
                    Err(e) => {
                        // Framing failures end the session rather than being answered. There
                        // is no id to answer *to* — we never saw a complete request — and a
                        // peer that cannot frame will not be helped by a JSON error it also
                        // cannot parse.
                        tracing::warn!(peer = %peer, error = ?e, "framing failed; closing session");
                        break;
                    }
                };

                for line in lines {
                    let outcome = dispatch(
                        &mut pool,
                        &config_socket,
                        &line,
                        &mut authenticated,
                        &mut attempts_left,
                    )
                    .await;

                    if let Some(response) = outcome.response
                        && send_line(&link, &response).await.is_err()
                    {
                        return;
                    }
                    if outcome.close {
                        tracing::warn!(peer = %peer, "closing the session: too many bad PINs");
                        return;
                    }
                }
            }

            // A reply or a notification from a service.
            line = replies.recv() => {
                // The channel is held by `pool`, which lives as long as this loop, so `None`
                // is unreachable — but treat it as end-of-session rather than panicking.
                let Some(line) = line else { break };
                if send_line(&link, &line).await.is_err() {
                    return;
                }
            }
        }
    }

    if inbound.pending() > 0 {
        // Worth a line: it distinguishes "client finished and left" from "client vanished
        // mid-message", which is the difference between a normal disconnect and a bug.
        tracing::debug!(peer = %peer, pending = inbound.pending(), "session ended mid-line");
    }
    tracing::info!(peer = %peer, "session closed");
}

/// What handling one line produced.
struct Outcome {
    /// A response `btd` must send itself. `None` means an upstream will answer — the ordinary path.
    response: Option<String>,
    /// End the session after sending. Only ever set by exhausting the PIN attempts.
    close: bool,
}

impl Outcome {
    fn nothing() -> Self {
        Self {
            response: None,
            close: false,
        }
    }
    fn reply(response: String) -> Self {
        Self {
            response: Some(response),
            close: false,
        }
    }
}

/// Handle one complete line.
async fn dispatch(
    pool: &mut Pool,
    config_socket: &std::path::Path,
    line: &str,
    authenticated: &mut bool,
    attempts_left: &mut u32,
) -> Outcome {
    let request: proto::Request = match serde_json::from_str(line) {
        Ok(request) => request,
        Err(e) => {
            // No id is recoverable from an unparseable line, so the response carries `null` —
            // which the spec requires and every client already handles.
            return Outcome::reply(encode(&proto::Response::err(
                None,
                proto::Error::new(proto::code::PARSE_ERROR, e.to_string()),
            )));
        }
    };

    // A notification — no id — expects no reply, so a refused one is dropped silently. It is
    // still not forwarded: the allowlist is not advisory.
    let id = request.id.clone();

    let call = match request.as_call() {
        Ok(call) => call,
        Err(e) => {
            return match id.map(|id| encode(&proto::Response::err(Some(id), e))) {
                Some(r) => Outcome::reply(r),
                None => Outcome::nothing(),
            };
        }
    };

    // The PIN gate. `hello` is allowed through because it only reports versions — the same
    // information the GATT read already gives an unauthenticated client — and refusing it would
    // leave a mismatched client unable to learn why nothing works.
    if !*authenticated
        && !matches!(
            call,
            proto::Call::SystemAuthenticate(_) | proto::Call::Hello(_)
        )
    {
        tracing::info!(method = call.method(), "refused: not authenticated");
        let error = proto::Error::new(
            proto::code::PERMISSION_DENIED,
            format!(
                "{} needs authentication first: send system.authenticate with the robot's PIN \
                 (`robotctl system pin` on the robot)",
                call.method()
            ),
        );
        return match id.map(|id| encode(&proto::Response::err(Some(id), error))) {
            Some(r) => Outcome::reply(r),
            None => Outcome::nothing(),
        };
    }

    let (upstream, lane) = match route::route_for(&call) {
        Route::To(upstream, lane) => (upstream, lane),
        Route::Local => {
            let proto::Call::SystemAuthenticate(params) = &call else {
                // `route_for` returns `Local` for exactly one variant; anything else here is a
                // routing table that grew a local method without teaching this function about it.
                tracing::error!(method = call.method(), "routed locally with no handler");
                let error = proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    "routed locally with no handler",
                );
                return match id.map(|id| encode(&proto::Response::err(Some(id), error))) {
                    Some(r) => Outcome::reply(r),
                    None => Outcome::nothing(),
                };
            };
            return authenticate(config_socket, params, authenticated, attempts_left, id).await;
        }
        Route::Refused => {
            tracing::info!(method = call.method(), "refused over BLE");
            return match id.map(|id| encode(&proto::Response::err(Some(id), route::refusal(&call))))
            {
                Some(r) => Outcome::reply(r),
                None => Outcome::nothing(),
            };
        }
    };

    if let Err(e) = pool.send(upstream, lane, line).await {
        tracing::warn!(method = call.method(), upstream = ?upstream, error = %e, "upstream unreachable");
        // Naming the service is what makes this diagnosable from a phone screenshot: "robotd
        // is not answering" is a different problem from "the robot refused".
        let error = proto::Error::new(
            proto::code::INTERNAL_ERROR,
            format!("{upstream:?} is not answering: {e}"),
        );
        return match id.map(|id| encode(&proto::Response::err(Some(id), error))) {
            Some(r) => Outcome::reply(r),
            None => Outcome::nothing(),
        };
    }
    Outcome::nothing()
}

/// Check the PIN, and ration the attempts.
///
/// Fetched from `configd` per attempt rather than cached, so `robotctl system set-pin` takes effect
/// on the next try rather than the next reboot — and so a `configd` that cannot answer means the
/// session is refused rather than admitted.
///
/// The comparison is on the string, not a number: `000042` and `42` are different PINs, and a
/// numeric parse would make them the same.
async fn authenticate(
    config_socket: &std::path::Path,
    params: &proto::AuthenticateParams,
    authenticated: &mut bool,
    attempts_left: &mut u32,
    id: Option<proto::Id>,
) -> Outcome {
    let expected = match pairing::pin(config_socket).await {
        Ok(expected) => expected,
        Err(e) => {
            tracing::warn!(error = %e, "cannot read the PIN; refusing to authenticate");
            let error = proto::Error::new(
                proto::code::INTERNAL_ERROR,
                "cannot check the PIN: configd is not answering",
            );
            return match id.map(|id| encode(&proto::Response::err(Some(id), error))) {
                Some(r) => Outcome::reply(r),
                None => Outcome::nothing(),
            };
        }
    };

    // Constant-time-ish: compare whole strings of equal length rather than returning early on the
    // first differing digit. Over BLE the timing signal is buried in milliseconds of radio, so this
    // is hygiene rather than a defence — but it costs nothing.
    let ok = expected.pin.len() == params.pin.len()
        && expected
            .pin
            .bytes()
            .zip(params.pin.bytes())
            .fold(0u8, |acc, (a, b)| acc | (a ^ b))
            == 0;

    if ok {
        *authenticated = true;
        tracing::info!(default_pin = expected.is_default, "authenticated");
        if expected.is_default {
            // Worth saying every time: a factory PIN authenticates anyone who read this repository.
            tracing::warn!(
                "authenticated with the FACTORY PIN, which is public. Set a per-robot one: \
                 robotctl system set-pin <6 digits>"
            );
        }
        let result = proto::AuthenticateResult {
            authenticated: true,
            attempts_remaining: PIN_ATTEMPTS,
        };
        return match id.map(|id| encode(&proto::Response::ok(Some(id), &result))) {
            Some(r) => Outcome::reply(r),
            None => Outcome::nothing(),
        };
    }

    *attempts_left = attempts_left.saturating_sub(1);
    tracing::warn!(attempts_left = *attempts_left, "wrong PIN");

    let result = proto::AuthenticateResult {
        authenticated: false,
        attempts_remaining: *attempts_left,
    };
    let response = id.map(|id| encode(&proto::Response::ok(Some(id), &result)));
    Outcome {
        response,
        close: *attempts_left == 0,
    }
}

/// Chunk one line out to the central.
async fn send_line(link: &Link, line: &str) -> Result<(), ()> {
    // Read once here, so one line is chunked one way even if a write reports a new MTU
    // meanwhile. See `Link::mtu`.
    for chunk in framing::chunks(line, link.mtu()) {
        if link.outbound.send(chunk).await.is_err() {
            // The backend dropped its half: the central is gone.
            return Err(());
        }
    }
    Ok(())
}

fn encode(response: &proto::Response) -> String {
    // A Response is plain strings, ints and enums; this cannot fail. If it somehow did,
    // sending nothing would hang the client, so send something it can parse as an error.
    serde_json::to_string(response).unwrap_or_else(|_| {
        r#"{"jsonrpc":"2.0","id":null,"error":{"code":-32603,"message":"internal error"}}"#
            .to_owned()
    })
}
