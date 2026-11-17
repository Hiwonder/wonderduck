//! The robot's half of the bridge to the rendezvous service.
//!
//! A LAN client reaches `webrtcsink`'s signalling server directly and nothing here is involved.
//! This is what makes a duck reachable from *outside* its network: it connects **outward** to
//! `reachy_mini_central` holding the account token, registers as a **producer**, and the service
//! shows a client only the robots its own account owns.
//!
//! # What it does
//!
//! **Registration and liveness** (§8, slice 2): the robot appears in the service's listing and
//! stays there, for as long as the account token on disk is the one it connected with.
//!
//! **And it carries a session** (slice 4). A remote consumer cannot reach `webrtcsink`'s
//! signalling server — it is on a loopback address behind somebody's router — so when one asks
//! the rendezvous for a session, [`bridge`] opens that socket *as the consumer on its behalf* and
//! carries envelopes between the two, rewriting the `sessionId` each hop names and reading none
//! of the payload. One session at a time; a second is refused by name.
//!
//! # A control lane that needs no candidate pair, and why there is one
//!
//! The bridge above carries a *negotiation*: SDP and ICE, so that a consumer and the robot can
//! find a path between them and speak WebRTC over it. When they cannot find one, everything built
//! on it is gone. A relay candidate is what keeps that from happening (§6) — and it is a
//! dependency rather than a guarantee: it is somebody else's service, metered per account, and
//! the control channel is SCTP over whatever pair ICE settled on, so a relay that stops being
//! available takes a JSON-RPC call of a few hundred bytes down with the video.
//!
//! So the JSON-RPC a consumer wants to send does not have to go through WebRTC at all, and the
//! rendezvous turns out to already carry it: `handle_peer_message` in their `app.py` relays
//! **every key of a `peer` envelope except `type` and `sessionId`** verbatim to the session
//! partner, without looking at `sdp` or `ice`. A `peer` message carrying an `rpc` key is therefore
//! a control call, relayed opaquely, with no change to a service the mini fleet also depends on.
//!
//! ```text
//!   consumer ──POST /send {type:peer, sessionId, rpc:{…}}──► rendezvous ──SSE──► this relay
//!            ◄─────────── SSE {type:peer, sessionId, rpc:{…}} ◄──POST /send────────┘
//! ```
//!
//! `session::run` is what answers it, unchanged, because it is transport-agnostic on purpose. Same
//! routing table, same per-lane sockets, same refusal to parse a reply. No ICE, no DTLS, no TURN.
//!
//! **What it is not is a teleop lane.** The rendezvous allows 1200 requests per 60 s per peer, so
//! roughly twenty a second shared with the heartbeat — four calls to install and run a policy is
//! nothing, and a 50 Hz intent stream is over budget in a second. Worse, exceeding it earns a
//! `429` on the *whole peer*, which would take the robot's own lease down with it: a client could
//! knock a robot off the rendezvous by subscribing to telemetry. Hence [`Budget`], which bounds
//! notifications and never a reply — replies are one-per-request and so already bounded by
//! whatever the client itself can afford to send.
//!
//! # Why the transport is HTTP, which is not what `remote-webrtc.md` §7 assumed
//!
//! The envelopes are the gst signalling protocol's — the same messages a LAN client exchanges —
//! but they arrive over **SSE** and are sent with **`POST /send`**, with per-hop peer and session
//! ids. So the payload stays opaque and the envelope does not: this is a translator with an
//! opaque payload rather than a relay. §3.2 has the two sides side by side.
//!
//! # Three things read out of their source that shape the code below
//!
//! - **`POST /send` before `GET /events` is a 400.** The peer does not exist until the stream
//!   does — identity comes from the bearer token, and the token is bound to a peer by the
//!   `/events` connection. So the stream is opened *first* and registration follows the welcome,
//!   which happens to be the order §3.4 wanted anyway for a different reason.
//! - **The lease is refreshed by inbound `POST`, not by a healthy stream.** Thirty seconds, and a
//!   half-open TCP connection absorbs server-pushed keepalives silently for minutes — during
//!   which the robot believes it is reachable and is not. Hence [`heartbeat`-cadence] re-posts of
//!   `setPeerStatus`, and hence the split-brain poll.
//! - **Only producers carrying `meta.hardware_id` are swept.** A producer without it is never
//!   evicted, so a crashed daemon would leave a ghost in somebody's robot list forever. This
//!   always sends one — the SoC serial, or `/etc/machine-id` where there is no serial to read.
//!
//! [`heartbeat`-cadence]: Welcome::heartbeat
//!
//! # It is a task, not a daemon
//!
//! In `mediad` rather than a `relayd` for §3.5's reasons: a separate unit would need its own copy
//! of the producer identity, its own config and its own restart story, and would still be useless
//! without `mediad` running. Nothing here touches GStreamer — the boundary `pipeline.rs`'s
//! no-panic rule lives on — and nothing here is `cfg(target_os)`-gated, so the whole of it is
//! testable on a laptop against a fake service.

use std::path::{Path, PathBuf};
use std::time::Duration;

use duck_ipc_proto::{RemoteLink, RemoteStatus};
use eventsource_stream::Eventsource as _;
use futures_util::StreamExt;
use serde::{Deserialize, Serialize};

/// The Space the mini's fleet already registers against. §4.
pub const DEFAULT_RENDEZVOUS: &str = "https://pollen-robotics-reachy-mini-central.hf.space";

/// Where `updaterd` keeps the account credential.
///
/// **A cross-daemon file format, and `hf_robot_account` owns it.** `updaterd` performs the login
/// and that crate writes the file; a test there pins the one key `read_access_token` takes out of
/// it, because the writer is what can break the contract.
/// Read on every connect attempt rather than cached: a login that happens while this task is
/// waiting has to take effect without a restart, and re-reading a small file on a path that
/// already sleeps for thirty seconds costs nothing.
pub const DEFAULT_TOKEN_PATH: &str = "/etc/robot/hf-token";

/// How long to wait between looks at a token file that is not there yet.
///
/// This is the `waiting for token` state, and it is the ordinary state of a robot nobody has
/// signed in — so it must be quiet in the journal and cheap on the board.
const NO_TOKEN_POLL: Duration = Duration::from_secs(30);

/// How long a read from the event stream may go quiet before the connection is presumed dead.
///
/// The service emits `event: ping` after 30 s of idle, whose only job is to keep the proxy in
/// front of the Space from killing the connection. Sixty seconds is two missed pings, which is
/// what `reachy_mini`'s relay uses. §3.3.
const READ_TIMEOUT: Duration = Duration::from_secs(60);

/// How long the welcome gets to arrive before the connection is abandoned.
const WELCOME_TIMEOUT: Duration = Duration::from_secs(20);

/// The fallback heartbeat cadence, when the welcome names none.
///
/// The service publishes `recommended_heartbeat_interval_seconds: 10.0` and **no `lease_seconds`**
/// — so `reachy_mini`'s middle rung, `lease_seconds / 3`, is unreachable here and is not
/// reproduced. Five seconds is a sixth of the lease, which survives a missed post.
const HEARTBEAT_FALLBACK: Duration = Duration::from_secs(5);

/// The cadence is clamped, so a misconfigured service can neither ask for a request storm nor
/// talk us into a cadence slower than our own eviction.
const HEARTBEAT_BOUNDS: (Duration, Duration) = (Duration::from_secs(1), Duration::from_secs(60));

/// How often to ask the service whether it still lists this robot. §3.4, split-brain.
const STATUS_POLL: Duration = Duration::from_secs(30);

/// How many consecutive times the service may fail to list this robot before reconnecting.
///
/// Two rather than one: `/api/robot-status` is a separate request from the stream, and one lost
/// answer is not evidence of anything.
const MISSES_BEFORE_RECONNECT: u32 = 2;

/// Reconnect backoff: where it starts, where it stops, and how much noise goes on top.
const BACKOFF_START: Duration = Duration::from_secs(5);
const BACKOFF_MAX: Duration = Duration::from_secs(60);
const BACKOFF_JITTER: f64 = 0.10;

/// How long a request that is not the event stream gets.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(20);

/// How long the robot's own signalling server gets to hand over a session.
///
/// It is in this process — `webrtcsink` runs it — so this is generous rather than tuned. What it
/// guards against is a pipeline that never reached PLAYING, which produces a server with no
/// producer and a handshake that would otherwise wait for one forever.
const LOCAL_HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(10);

/// Where the robot's own signalling server listens.
///
/// `webrtcsink` runs it in this process with `signalling-server-host` and `-port`, so this is the
/// same 8443 `mediad`'s `--port` defaults to. Loopback, because the bridge and the server are one
/// process — a remote peer reaches this robot through the rendezvous, never through this socket.
pub const DEFAULT_LOCAL_SIGNALLING: &str = "ws://127.0.0.1:8443";

/// Every interval this task runs on, in one place.
///
/// A value rather than the constants directly, and it exists for the tests: §3.4's four failure
/// modes are all *timing* failures — a lease that stops being refreshed, a service that goes on
/// answering while it has forgotten us, a fleet reconnecting in lockstep — and none of them can be
/// reproduced on demand by hand on a board. With the intervals injectable, each one is a test that
/// runs in under a second. Production always uses [`Timings::default`], which is the constants
/// above.
#[derive(Debug, Clone, Copy)]
pub struct Timings {
    pub no_token_poll: Duration,
    pub read_timeout: Duration,
    pub welcome_timeout: Duration,
    pub heartbeat_fallback: Duration,
    pub heartbeat_bounds: (Duration, Duration),
    pub status_poll: Duration,
    pub backoff_start: Duration,
    pub backoff_max: Duration,
}

impl Default for Timings {
    fn default() -> Self {
        Self {
            no_token_poll: NO_TOKEN_POLL,
            read_timeout: READ_TIMEOUT,
            welcome_timeout: WELCOME_TIMEOUT,
            heartbeat_fallback: HEARTBEAT_FALLBACK,
            heartbeat_bounds: HEARTBEAT_BOUNDS,
            status_poll: STATUS_POLL,
            backoff_start: BACKOFF_START,
            backoff_max: BACKOFF_MAX,
        }
    }
}

// ── what a client sees in the listing ────────────────────────────────────────

/// What this robot calls itself to the service.
///
/// Free-form to the protocol and **not to the server**, which reads three of these keys — see the
/// module header on `hardware_id`. The rest are for whoever is looking at a list of robots.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Meta {
    /// The stable-identity key: same physical robot across reinstalls, renames and new tokens.
    ///
    /// The server evicts an older producer of the same user carrying the same value, which is how
    /// a re-flashed board or a restarted daemon stops showing up as a second robot. It is also
    /// what makes this robot sweepable at all.
    pub hardware_id: String,
    /// What a person sees. Absent when `configd` did not answer in time, as elsewhere.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// `wonderduck`, which is what lets one client list two families of robot without opening a
    /// session to ask what it found.
    pub kind: &'static str,
    /// The release this robot is running, for the reason the local `meta` carries it.
    pub release: String,
    pub api_version: u32,
    /// This robot is a duck in MuJoCo, and whoever is reading the listing should be told.
    ///
    /// **The listing is the one place this genuinely matters.** On a LAN you know what you
    /// started; in an account's robot list a simulated duck sits next to real ones, and a client
    /// that cannot tell them apart ends with somebody driving a simulation and wondering why the
    /// robot on the shelf is still. `meta` is free-form to the service, so this costs nothing
    /// there — §3.7 names the keys it does read, and this is not one of them.
    ///
    /// Absent rather than `false` on a real robot: the key is worth noticing where it appears.
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub simulated: bool,
}

impl Meta {
    /// This robot's `meta`, from what the local producer already learned.
    ///
    /// `hardware_id` falls back to `/etc/machine-id` when there is no SoC serial to read — a
    /// developer's laptop, or a board whose device tree has no `serial-number`. Stable per
    /// install rather than per robot, which is weaker and still correct for the purpose: it keeps
    /// one machine from being listed twice, and it keeps the producer sweepable. `sounds` makes
    /// exactly this substitution for exactly this reason.
    ///
    /// **A simulated duck never reaches that fallback**, and that is why `configd --simulated`
    /// takes a serial rather than one being invented here. The fallback is per *install*: four
    /// ducks in one MuJoCo scene share one `/etc/machine-id`, so they would evict each other from
    /// the listing one at a time — and on macOS there is no such file at all, so none of them
    /// would register. A serial per duck makes every one of them a robot by the same rules.
    pub fn of(producer: &crate::producer::Producer, machine_id: Option<String>) -> Option<Self> {
        let hardware_id = producer
            .serial
            .clone()
            .or(machine_id)
            .or_else(|| read_machine_id(Path::new("/etc/machine-id")))?;
        Some(Self {
            hardware_id,
            name: producer.name.clone(),
            kind: "wonderduck",
            release: producer.release.clone(),
            api_version: producer.api_version,
            simulated: producer.simulated,
        })
    }
}

fn read_machine_id(path: &Path) -> Option<String> {
    let id = std::fs::read_to_string(path).ok()?.trim().to_owned();
    (!id.is_empty()).then_some(id)
}

// ── the wire ─────────────────────────────────────────────────────────────────

/// What the service sends down the event stream.
///
/// Unknown types are a variant rather than an error: this is somebody else's service and it is
/// allowed to grow messages we do not handle. The ones named here are the ones acted on.
///
/// **`Peer` carries the whole envelope rather than its fields**, and that is the design rather
/// than laziness: an SDP or an ICE candidate passes through this process untouched, and the only
/// thing rewritten is the `sessionId` around it. Deserialising the payload would mean owning a
/// copy of a schema that belongs to WebRTC, and re-serialising it would mean re-encoding SDP that
/// arrived perfectly good. §3.2 — a translator with an opaque payload.
#[derive(Debug, Clone, PartialEq)]
enum Inbound {
    Welcome(Welcome),
    /// A consumer wants a session, which is what the bridge exists to serve.
    StartSession {
        session_id: String,
    },
    /// The other side gave up, or the service ended it.
    EndSession {
        session_id: Option<String>,
    },
    /// SDP or ICE for a session in flight.
    Peer(serde_json::Value),
    Other,
}

/// Read a message off the wire far enough to route it, and no further.
fn classify(raw: &str) -> Result<Inbound, String> {
    let value: serde_json::Value =
        serde_json::from_str(raw).map_err(|e| format!("unparseable message: {e}"))?;
    let session_id = |value: &serde_json::Value| value["sessionId"].as_str().map(str::to_owned);
    Ok(match value["type"].as_str().unwrap_or_default() {
        "welcome" => Inbound::Welcome(
            serde_json::from_value(value)
                .map_err(|e| format!("a welcome this page cannot read: {e}"))?,
        ),
        "startSession" => match session_id(&value) {
            Some(session_id) => Inbound::StartSession { session_id },
            // A `startSession` with no id is not something to answer: there is nothing to answer
            // *about*, and inventing one would open a session the service cannot route.
            None => return Err("a startSession with no sessionId".to_owned()),
        },
        "endSession" => Inbound::EndSession {
            session_id: session_id(&value),
        },
        "peer" => Inbound::Peer(value),
        _ => Inbound::Other,
    })
}

/// The first message on a healthy stream, and the only one that has to arrive.
///
/// **Two casings in one object**, which is the service's and not a mistake here: `peerId` is
/// camelCase like every other envelope field, and `recommended_heartbeat_interval_seconds` is
/// snake_case like every `meta` key. A blanket `rename_all` silently reads the cadence as absent
/// and falls back to five seconds — a robot that works while posting twice as often as asked, and
/// nothing anywhere says why. So that one field is named outright.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Welcome {
    /// The id this connection is known by, which `/api/robot-status` reports back. Kept so the
    /// split-brain poll can look for *this* robot rather than for any robot.
    peer_id: String,
    /// The account the token belongs to, as the service resolved it. Logged once: it is the
    /// answer to "whose robot does the service think this is".
    #[serde(default)]
    username: Option<String>,
    /// What the service asks for, in seconds. Absent on a service that does not say.
    #[serde(default, rename = "recommended_heartbeat_interval_seconds")]
    recommended_heartbeat_interval_seconds: Option<f64>,
}

impl Welcome {
    /// The cadence to post at: what was asked for, clamped, or the fallback.
    fn heartbeat(&self, timings: &Timings) -> Duration {
        let (min, max) = timings.heartbeat_bounds;
        match self.recommended_heartbeat_interval_seconds {
            Some(seconds) if seconds.is_finite() && seconds > 0.0 => {
                Duration::from_secs_f64(seconds).clamp(min, max)
            }
            _ => timings.heartbeat_fallback,
        }
    }
}

/// What this robot sends. `POST /send`, one object per request.
#[derive(Debug, Serialize)]
#[serde(tag = "type", rename_all = "camelCase")]
enum Outbound<'a> {
    /// Registration, and every heartbeat after it: the same message, which is why the lease is
    /// keyed on the request rather than on its contents.
    SetPeerStatus {
        roles: [&'static str; 1],
        meta: &'a Meta,
    },
    /// How this slice refuses a session it cannot serve yet.
    #[serde(rename_all = "camelCase")]
    EndSession {
        session_id: &'a str,
        reason: &'a str,
    },
}

/// What `/api/robot-status` answers. Only the ids are read.
#[derive(Debug, Deserialize)]
struct RobotStatus {
    #[serde(default)]
    robots: Vec<RobotStatusEntry>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct RobotStatusEntry {
    peer_id: String,
}

// ── the task ─────────────────────────────────────────────────────────────────

/// Why a connection ended, which is what decides how long to wait before the next one.
#[derive(Debug)]
enum Ended {
    /// The stream closed or went quiet, or a post failed. Ordinary; back off and reconnect.
    Reconnect(String),
    /// The service accepted the token and stopped listing this robot anyway. §3.4.
    SplitBrain,
    /// The service refused the token. Backing off does not fix this — a login does — so this
    /// waits on the token file instead of on a timer.
    Unauthorised,
    /// The credential on disk is not the one this connection is using: a `logout`, or a
    /// `login --force` onto another account.
    ///
    /// **This is what makes `account.logout` mean anything here.** The token is read once per
    /// connection, so without this the relay would go on refreshing the lease with a credential
    /// its owner had deleted — a robot signed out of an account and still listed under it, which
    /// makes the credential revocable.
    /// Dropping the stream is the deregistration: a clean disconnect evicts the peer at once, and
    /// the 30 s sweep is only there for sockets that never report closing.
    CredentialChanged,
}

/// What this task last published, so a heartbeat can move `lastHeartbeat` without moving `since`.
struct Published<'a> {
    path: &'a Path,
    service: &'a str,
    current: Option<RemoteStatus>,
    /// Said once: on a laptop there is no `/run/mediad`, and every heartbeat would say it again.
    warned: bool,
}

impl<'a> Published<'a> {
    fn new(path: &'a Path, service: &'a str) -> Self {
        Self {
            path,
            service,
            current: None,
            warned: false,
        }
    }

    fn is(&self, test: impl FnOnce(&RemoteLink) -> bool) -> bool {
        self.current
            .as_ref()
            .is_some_and(|status| test(&status.link))
    }

    /// Publish `link`, keeping `since` while the kind of state is unchanged. Rewrites nothing when
    /// nothing moved, which is every no-token poll on a robot nobody has signed in.
    fn set(&mut self, link: RemoteLink) {
        let since = match &self.current {
            Some(current)
                if std::mem::discriminant(&current.link) == std::mem::discriminant(&link) =>
            {
                current.since
            }
            _ => unix_now(),
        };
        let status = RemoteStatus {
            service: self.service.to_owned(),
            since,
            link,
        };
        if self.current.as_ref() == Some(&status) {
            return;
        }
        if let Err(e) = duck_ipc_proto::publish_remote_status(self.path, &status)
            && !self.warned
        {
            tracing::warn!(
                error = %e,
                "could not publish the rendezvous status; `robotctl health` will not show it"
            );
            self.warned = true;
        }
        self.current = Some(status);
    }
}

fn unix_now() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs() as i64)
}

/// The relay, as the task that owns the outward connection.
pub struct Relay {
    base: String,
    token_path: PathBuf,
    meta: Meta,
    client: reqwest::Client,
    timings: Timings,
    local_signalling: String,
    /// What the pipeline knows, for the two calls [`crate::session::run`] answers itself.
    ///
    /// **A watch rather than a value, because the relay starts before the answer exists.** It is
    /// spawned deliberately early — a robot that appears in its owner's list and cannot stream is
    /// still a robot somebody can reach to find out why — while the frame geometry is only
    /// truthful *after* the pipeline is up, since which sensor mode is in force is not known until
    /// something has tried to set it. So `main` hands over a receiver and fills it in later, and a
    /// lane reads whatever is current when it opens. Empty means `media.video` is refused rather
    /// than answered with zeros.
    video: Option<tokio::sync::watch::Receiver<Option<crate::session::Media>>>,
    /// Where each service listens, for the control lane's pool. Given rather than defaulted for
    /// the same reason `--rendezvous-url` is: the whole of this module is meant to be exercisable
    /// on a laptop, and a lane whose sockets were hardcoded to `/run/robot` could only be tested
    /// on a board.
    sockets: crate::upstream::Sockets,
    /// Where to publish [`RemoteStatus`] for `robotctl health`. An argument for `sockets`' reason:
    /// a test must not write the board's `/run/mediad`, and `main` must not forget to name it.
    status_path: PathBuf,
}

impl Relay {
    /// Build one. Fails only if the HTTP client will not build, which means no TLS stack.
    ///
    /// **`sockets` is an argument and not a builder, which is the whole of the fix this carries.**
    /// It used to default to `/run/*.sock` and be overridable with `with_sockets`, and `main` was
    /// the one caller that never called it — so every control-lane call from the rendezvous
    /// dialled `/run/robotd.sock` no matter what `--robot-socket` said. On a board that is
    /// invisible, because there the default is right; on the twin it is `os error 2` for every
    /// method except `media.video`, which `session::run` answers without an upstream at all.
    /// A caller cannot forget an argument, and this lane has no default worth having.
    pub fn new(
        base: impl Into<String>,
        token_path: impl Into<PathBuf>,
        meta: Meta,
        sockets: crate::upstream::Sockets,
        status_path: impl Into<PathBuf>,
    ) -> Option<Self> {
        let client = reqwest::Client::builder()
            .connect_timeout(Duration::from_secs(10))
            .user_agent(concat!("mediad/", env!("CARGO_PKG_VERSION")))
            .build()
            .map_err(|e| {
                tracing::error!(
                    error = %e,
                    "no HTTP client, so this robot cannot be reached from outside its network"
                );
            })
            .ok()?;
        Some(Self {
            base: base.into().trim_end_matches('/').to_owned(),
            token_path: token_path.into(),
            meta,
            client,
            timings: Timings::default(),
            local_signalling: DEFAULT_LOCAL_SIGNALLING.to_owned(),
            video: None,
            sockets,
            status_path: status_path.into(),
        })
    }

    /// Where to read the video's geometry when a control lane opens.
    ///
    /// A builder rather than a fourth argument to [`Relay::new`] because the video is discovered
    /// later than the relay is built, and because every test here is about the wire rather than
    /// about the camera.
    pub fn with_video(
        mut self,
        video: tokio::sync::watch::Receiver<Option<crate::session::Media>>,
    ) -> Self {
        self.video = Some(video);
        self
    }

    /// Point the bridge at a signalling server other than `webrtcsink`'s own.
    ///
    /// For tests, and for a `mediad` whose `--port` is not the default: the bridge connects to
    /// the server this same process is running, so the two numbers have to agree.
    pub fn with_local_signalling(mut self, url: impl Into<String>) -> Self {
        self.local_signalling = url.into();
        self
    }

    /// Run on intervals other than the shipped ones. See [`Timings`]; tests only.
    #[doc(hidden)]
    pub fn with_timings(mut self, timings: Timings) -> Self {
        self.timings = timings;
        self
    }

    /// Stay registered for as long as this process runs.
    ///
    /// Never returns. Every failure is a reconnect, because there is no state here worth keeping
    /// across one: the service's view of this robot is rebuilt by the next `setPeerStatus`.
    pub async fn run(self) {
        let mut backoff = self.timings.backoff_start;
        let mut published = Published::new(&self.status_path, &self.base);
        loop {
            let Some(token) = self.token() else {
                published.set(RemoteLink::SignedOut);
                // At `debug`: a robot nobody has signed in is not a robot with a problem, and
                // this is every thirty seconds forever.
                tracing::debug!(
                    path = %self.token_path.display(),
                    "no account token yet; this robot is reachable on its own network only"
                );
                tokio::time::sleep(self.timings.no_token_poll).await;
                continue;
            };

            // Not over a retry or a refusal: "retrying since" and "refused since" should date the
            // first failure, and a reconnect that flipped through `connecting` would reset both.
            if !published
                .is(|link| matches!(link, RemoteLink::Retrying { .. } | RemoteLink::Refused))
            {
                published.set(RemoteLink::Connecting);
            }

            match self.session(&token, &mut published).await {
                Ended::Unauthorised => {
                    published.set(RemoteLink::Refused);
                    tracing::warn!(
                        "the rendezvous service refused this robot's account token; a new login \
                         is what fixes it"
                    );
                    tokio::time::sleep(self.timings.no_token_poll).await;
                }
                Ended::CredentialChanged => {
                    // No backoff: either there is a new token to use immediately, or there is
                    // none and the loop above is about to wait on the file anyway.
                    tracing::info!(
                        "this robot's account credential changed; the rendezvous connection is \
                         dropped, which takes the robot out of the service's listing"
                    );
                    backoff = self.timings.backoff_start;
                }
                Ended::SplitBrain => {
                    published.set(RemoteLink::Retrying {
                        reason: "the service stopped listing this robot although the stream was \
                                 healthy"
                            .to_owned(),
                    });
                    // Reconnect immediately rather than backing off: the connection looked
                    // healthy, so there is nothing to wait for, and every second here is a
                    // second the robot is not reachable while believing it is.
                    tracing::warn!(
                        "the service no longer lists this robot although the stream was healthy; \
                         reconnecting"
                    );
                    backoff = self.timings.backoff_start;
                }
                Ended::Reconnect(why) => {
                    tracing::info!(%why, retry_in = ?backoff, "remote access is off; will retry");
                    published.set(RemoteLink::Retrying { reason: why });
                    tokio::time::sleep(jittered(backoff)).await;
                    backoff = (backoff * 2).min(self.timings.backoff_max);
                }
            }
        }
    }

    /// The access token, or `None` when this robot belongs to nobody.
    ///
    /// `hf_robot_account` owns the reading of it: it is the crate that writes the file, and the
    /// TURN credentials need the same token out of the same place.
    fn token(&self) -> Option<String> {
        hf_robot_account::read_access_token(&self.token_path)
    }

    /// One connection: open the stream, register, then hold the lease until something breaks.
    async fn session(&self, token: &str, published: &mut Published<'_>) -> Ended {
        let mut events = match self.open_stream(token).await {
            Ok(events) => events,
            Err(ended) => return ended,
        };

        let welcome = match self.await_welcome(&mut events).await {
            Ok(welcome) => welcome,
            Err(ended) => return ended,
        };

        // Registered *before* anything reports this robot as reachable, so no observer can see
        // "remote access enabled" while the service does not yet know the robot exists. §3.4.
        if let Err(ended) = self.set_peer_status(token).await {
            return ended;
        }
        let heartbeat = welcome.heartbeat(&self.timings);
        tracing::info!(
            peer_id = %welcome.peer_id,
            account = welcome.username.as_deref().unwrap_or("unknown"),
            ?heartbeat,
            "registered with the rendezvous service; this robot is reachable from outside its \
             network"
        );
        let registered = |published: &mut Published<'_>| {
            published.set(RemoteLink::Registered {
                account: welcome.username.clone(),
                peer_id: welcome.peer_id.clone(),
                last_heartbeat: unix_now(),
            });
        };
        registered(published);

        // At most one at a time. The service gates this too (`sessionRejected` with `robot_busy`),
        // so this is belt-and-braces — and it stays, because two remote peers writing into one
        // intent slot is `remote-webrtc.md` §9's interleaving bug with the pad replaced by a
        // second continent. §3.4.
        let mut bridged: Option<Bridged> = None;

        // The control lane, which is independent of the one above: a consumer may hold one, the
        // other, or both. Built on the first `rpc` envelope of a session and dropped with it.
        let mut control: Option<Control> = None;

        let mut heartbeats = tokio::time::interval(heartbeat);
        heartbeats.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        heartbeats.tick().await; // the first tick is immediate, and registration just happened
        let mut polls = tokio::time::interval(self.timings.status_poll);
        polls.tick().await;
        let mut misses = 0;

        loop {
            tokio::select! {
                _ = heartbeats.tick() => {
                    // Checked here rather than on its own timer: the heartbeat is already the
                    // fastest thing in this loop, so a `logout` takes effect within one cadence
                    // — ten seconds — and re-reading a small file that often costs nothing.
                    if self.token().as_deref() != Some(token) {
                        return Ended::CredentialChanged;
                    }
                    if let Err(ended) = self.set_peer_status(token).await {
                        return ended;
                    }
                    registered(published);
                }
                _ = polls.tick() => {
                    match self.lists_us(token, &welcome.peer_id).await {
                        Ok(true) => misses = 0,
                        Ok(false) => {
                            misses += 1;
                            tracing::warn!(
                                misses,
                                peer_id = %welcome.peer_id,
                                "the service did not list this robot"
                            );
                            if misses >= MISSES_BEFORE_RECONNECT {
                                return Ended::SplitBrain;
                            }
                        }
                        // A failed poll is not a miss: it says nothing about whether the service
                        // lists us, and treating it as one would reconnect a healthy stream
                        // every time the network hiccuped twice.
                        Err(why) => tracing::debug!(%why, "could not ask whether we are listed"),
                    }
                }
                // A session that ended on its own — the peer left, the pipeline stopped — is
                // reaped here rather than being noticed the next time one is asked for, so
                // `account.status` and the service agree about whether this robot is busy.
                _ = async {
                    match bridged.as_mut() {
                        Some(session) => (&mut session.task).await.ok(),
                        // Nothing to wait for; this branch must never be the one that fires.
                        None => std::future::pending().await,
                    }
                }, if bridged.is_some() => {
                    tracing::debug!("the bridged session finished");
                    bridged = None;
                }
                event = tokio::time::timeout(self.timings.read_timeout, events.next()) => {
                    match event {
                        Err(_) => return Ended::Reconnect(format!(
                            "nothing arrived on the event stream for {:?}, which is two missed \
                             pings",
                            self.timings.read_timeout
                        )),
                        Ok(None) => return Ended::Reconnect(
                            "the service closed the event stream".to_owned(),
                        ),
                        Ok(Some(Err(e))) => return Ended::Reconnect(
                            format!("the event stream failed: {e}"),
                        ),
                        Ok(Some(Ok(message))) => {
                            if let Some(ended) =
                                self.handle(token, message, &mut bridged, &mut control).await
                            {
                                return ended;
                            }
                        }
                    }
                }
            }
        }
    }

    /// `GET /events`, as a stream of parsed messages.
    async fn open_stream(
        &self,
        token: &str,
    ) -> Result<impl futures_util::Stream<Item = Result<Inbound, String>> + Unpin, Ended> {
        let url = format!("{}/events", self.base);
        let response = self
            .client
            .get(&url)
            .bearer_auth(token)
            .header("accept", "text/event-stream")
            .send()
            .await
            .map_err(|e| Ended::Reconnect(format!("GET {url}: {e}")))?;

        if response.status() == reqwest::StatusCode::UNAUTHORIZED {
            return Err(Ended::Unauthorised);
        }
        if !response.status().is_success() {
            return Err(Ended::Reconnect(format!(
                "GET {url}: HTTP {}",
                response.status()
            )));
        }

        // `eventsource-stream` owns the framing: `data:` split across TCP reads, multi-line
        // payloads, comments and the fields we do not use. What is left here is JSON.
        let events = response
            .bytes_stream()
            .eventsource()
            .filter_map(|event| async move {
                match event {
                    Err(e) => Some(Err(format!("{e}"))),
                    // The service's keepalive carries no data and means only "still here".
                    Ok(event) if event.data.trim().is_empty() => None,
                    Ok(event) => Some(classify(&event.data)),
                }
            });
        Ok(Box::pin(events))
    }

    /// Read until the welcome, which is the message that says the peer now exists.
    async fn await_welcome(
        &self,
        events: &mut (impl futures_util::Stream<Item = Result<Inbound, String>> + Unpin),
    ) -> Result<Welcome, Ended> {
        let deadline = tokio::time::Instant::now() + self.timings.welcome_timeout;
        loop {
            let event = tokio::time::timeout_at(deadline, events.next())
                .await
                .map_err(|_| {
                    Ended::Reconnect(format!(
                        "no welcome within {:?}",
                        self.timings.welcome_timeout
                    ))
                })?;
            match event {
                None => {
                    return Err(Ended::Reconnect(
                        "the stream closed before the welcome".to_owned(),
                    ));
                }
                Some(Err(why)) => {
                    // A message we cannot read is not a reason to drop a stream that is otherwise
                    // working; the welcome may be the next one.
                    tracing::debug!(%why, "skipping a message while waiting for the welcome");
                }
                Some(Ok(Inbound::Welcome(welcome))) => return Ok(welcome),
                Some(Ok(_)) => {}
            }
        }
    }

    /// Register, and refresh the lease. The same request does both.
    async fn set_peer_status(&self, token: &str) -> Result<(), Ended> {
        self.send(
            token,
            &Outbound::SetPeerStatus {
                roles: ["producer"],
                meta: &self.meta,
            },
        )
        .await
    }

    /// One `POST /send`.
    async fn send(&self, token: &str, message: &Outbound<'_>) -> Result<(), Ended> {
        let url = format!("{}/send", self.base);
        let response = self
            .client
            .post(&url)
            .bearer_auth(token)
            .timeout(REQUEST_TIMEOUT)
            .json(message)
            .send()
            .await
            .map_err(|e| Ended::Reconnect(format!("POST {url}: {e}")))?;

        match response.status() {
            status if status.is_success() => Ok(()),
            reqwest::StatusCode::UNAUTHORIZED => Err(Ended::Unauthorised),
            // 400 here means the peer does not exist — the stream this token was bound to is
            // gone. Reconnecting is what rebuilds it, and it is the whole reason the stream is
            // opened before anything is posted.
            status => Err(Ended::Reconnect(format!("POST {url}: HTTP {status}"))),
        }
    }

    /// Whether the service still lists this robot. §3.4, split-brain.
    async fn lists_us(&self, token: &str, peer_id: &str) -> Result<bool, String> {
        let url = format!("{}/api/robot-status", self.base);
        let response = self
            .client
            .get(&url)
            .bearer_auth(token)
            .timeout(REQUEST_TIMEOUT)
            .send()
            .await
            .map_err(|e| format!("GET {url}: {e}"))?;
        if !response.status().is_success() {
            return Err(format!("GET {url}: HTTP {}", response.status()));
        }
        let status: RobotStatus = response
            .json()
            .await
            .map_err(|e| format!("GET {url}: {e}"))?;
        Ok(status.robots.iter().any(|robot| robot.peer_id == peer_id))
    }

    /// One message from the service. `Some` ends the connection.
    async fn handle(
        &self,
        token: &str,
        message: Inbound,
        bridged: &mut Option<Bridged>,
        control: &mut Option<Control>,
    ) -> Option<Ended> {
        match message {
            // A second welcome on one stream would mean the service rebound this token, which is
            // what happens when another process registers with it. Reconnecting is how we find
            // out whose peer we are now.
            Inbound::Welcome(_) => Some(Ended::Reconnect(
                "the service sent a second welcome on the same stream".to_owned(),
            )),
            Inbound::StartSession { session_id } => {
                if bridged.as_ref().is_some_and(Bridged::live) {
                    // Refused by name rather than dropped: an unanswered `startSession` leaves a
                    // peer waiting on a robot that is never going to answer, and the owner
                    // looking at a robot that reads as broken rather than busy.
                    tracing::info!(%session_id, "refusing a second remote session");
                    return self
                        .send(
                            token,
                            &Outbound::EndSession {
                                session_id: &session_id,
                                reason: "this robot is already in a remote session",
                            },
                        )
                        .await
                        .err();
                }

                let (to_local, from_remote) = tokio::sync::mpsc::channel(64);
                let hub = Hub {
                    client: self.client.clone(),
                    base: self.base.clone(),
                    token: token.to_owned(),
                };
                let local_url = self.local_signalling.clone();
                let remote_id = session_id.clone();
                let task = tokio::spawn(async move {
                    if let Err(why) = bridge(hub, local_url, remote_id, from_remote).await {
                        tracing::warn!(%why, "a remote session could not be bridged");
                    }
                });
                *bridged = Some(Bridged {
                    remote_id: session_id,
                    to_local,
                    task,
                });
                None
            }
            Inbound::Peer(envelope) => {
                // **`rpc` first, because it needs no session to have been negotiated.** A
                // consumer that never intends to speak WebRTC still had to `startSession` to get
                // a session id — `handle_peer_message` drops an envelope naming a session the
                // service does not know — but it has no offer to send and none to answer. So a
                // control call is dispatched before the bridge is consulted, and a robot whose
                // media never connected still answers.
                if !envelope["rpc"].is_null() {
                    return self.control(token, &envelope, control).await;
                }

                // The envelope is forwarded whole and the payload is never read. What decides
                // where it goes is the session it names — and a `peer` for a session this robot
                // is not in is dropped rather than guessed at.
                let Some(session) = bridged.as_ref() else {
                    tracing::debug!("a peer message arrived with no session to carry it");
                    return None;
                };
                let names_it = envelope["sessionId"].as_str() == Some(session.remote_id.as_str());
                if !names_it {
                    tracing::debug!(
                        session = ?envelope["sessionId"].as_str(),
                        "a peer message for another session; dropping it"
                    );
                    return None;
                }
                if session.to_local.send(envelope).await.is_err() {
                    tracing::debug!("the bridged session went away before its message arrived");
                    *bridged = None;
                }
                None
            }
            Inbound::EndSession { session_id } => {
                // Dropping `Bridged` aborts the task, which closes the local socket — that is
                // what tells `webrtcsink` the consumer is gone.
                if control
                    .as_ref()
                    .is_some_and(|c| session_id.as_deref().is_none_or(|id| id == c.remote_id))
                {
                    tracing::info!(?session_id, "the service ended the control lane");
                    *control = None;
                }
                if bridged
                    .as_ref()
                    .is_some_and(|s| session_id.as_deref().is_none_or(|id| id == s.remote_id))
                {
                    tracing::info!(?session_id, "the service ended the bridged session");
                    *bridged = None;
                } else if control.is_none() {
                    tracing::debug!(?session_id, "the service ended a session we do not have");
                }
                None
            }
            Inbound::Other => None,
        }
    }
}

// ── the control lane: JSON-RPC over the rendezvous, with no candidate pair ───

/// How many notifications a control lane may post per window.
///
/// The rendezvous allows 1200 requests per 60 s **per peer**, and exceeding it earns a `429` on
/// everything that token does — including the heartbeat that holds this robot's lease. So a
/// consumer subscribing to 50 Hz telemetry would not merely get a slow stream, it would take the
/// robot off its owner's listing. Half the allowance is left for the heartbeat, the status poll
/// and whatever a person is actually doing.
const NOTIFICATIONS_PER_WINDOW: u32 = 400;
const NOTIFICATION_WINDOW: Duration = Duration::from_secs(60);

/// A sliding allowance for lines nobody asked for.
///
/// **Replies are deliberately not subject to it.** One reply answers one request, and a request
/// cost the consumer a `POST` of its own, so replies are already bounded by whatever the consumer
/// can afford — throttling them would only break callers. Notifications have no such bound: a
/// subscription is one request and then an unbounded stream.
struct Budget {
    allowance: u32,
    window: Duration,
    spent: u32,
    dropped: u64,
    since: tokio::time::Instant,
}

impl Budget {
    fn new(allowance: u32, window: Duration) -> Self {
        Self {
            allowance,
            window,
            spent: 0,
            dropped: 0,
            since: tokio::time::Instant::now(),
        }
    }

    /// Whether one notification may go out now.
    fn take(&mut self) -> bool {
        let now = tokio::time::Instant::now();
        if now.duration_since(self.since) >= self.window {
            if self.dropped > 0 {
                tracing::info!(
                    dropped = self.dropped,
                    "notifications dropped on the control lane: it is not a telemetry transport, \
                     and the alternative is a rate limit that would end this robot's lease"
                );
                self.dropped = 0;
            }
            self.spent = 0;
            self.since = now;
        }
        if self.spent < self.allowance {
            self.spent += 1;
            true
        } else {
            self.dropped += 1;
            false
        }
    }
}

/// One consumer's control lane, and the task that owns both its halves.
struct Control {
    /// The id the *service* knows this session by, which is what every envelope has to name.
    remote_id: String,
    /// Lines on their way into [`crate::session::run`].
    to_session: tokio::sync::mpsc::Sender<String>,
    task: tokio::task::JoinHandle<()>,
}

impl Control {
    fn live(&self) -> bool {
        !self.task.is_finished()
    }
}

impl Drop for Control {
    fn drop(&mut self) {
        // The task owns the session and its pool of unix sockets; aborting it closes every one,
        // which is what ends a subscription a consumer walked away from.
        self.task.abort();
    }
}

impl Relay {
    /// One `peer` envelope carrying `rpc`.
    ///
    /// Opens the lane if this session has none yet, which is what makes a control-only consumer
    /// need no cooperation from the media path: it says `startSession`, the service gives it an
    /// id, and its first call is what builds everything on this side.
    async fn control(
        &self,
        token: &str,
        envelope: &serde_json::Value,
        control: &mut Option<Control>,
    ) -> Option<Ended> {
        let Some(session_id) = envelope["sessionId"].as_str() else {
            tracing::debug!("an rpc envelope with no sessionId; dropping it");
            return None;
        };

        // A new session supersedes an old lane rather than being refused. Unlike a media session
        // — where two consumers writing into one intent slot is `remote-webrtc.md` §9's
        // interleaving bug — a lane holds no hardware: it is a socket per service, and the
        // service's own concurrency gate already means one consumer at a time.
        let reusable = control
            .as_ref()
            .is_some_and(|c| c.remote_id == session_id && c.live());
        if !reusable {
            *control = Some(self.open_control(token, session_id));
        }

        // The payload goes in as the line `session::run` expects, which is the object itself
        // rather than a string containing one: this transport is JSON all the way down, so
        // encoding a JSON-RPC object *inside* a JSON string would be an escaping bug waiting for
        // its first apostrophe.
        let line = envelope["rpc"].to_string();
        let lane = control.as_ref().expect("just built");
        if lane.to_session.send(line).await.is_err() {
            tracing::debug!("the control lane went away before its call arrived");
            *control = None;
        }
        None
    }

    /// Build a lane: a session, a pool of upstream sockets, and the task that posts its answers.
    fn open_control(&self, token: &str, session_id: &str) -> Control {
        let (to_session, from_consumer) = tokio::sync::mpsc::channel::<String>(64);
        // Deeper than the inbound half on purpose: one call can answer with a stream, and the
        // budget below would rather drop a notification than have a service block writing it.
        let (to_consumer, mut from_session) = tokio::sync::mpsc::channel::<String>(256);

        let hub = Hub {
            client: self.client.clone(),
            base: self.base.clone(),
            token: token.to_owned(),
        };
        let pool = crate::upstream::Pool::new(self.sockets.clone(), to_consumer.clone());
        // Read now rather than held as a receiver: a lane's answer to `media.video` should be
        // whatever was true when the consumer connected, and a picture that changed shape
        // mid-session is a `media.video` notification's job on the media path.
        let video = self.video.as_ref().and_then(|watch| watch.borrow().clone());
        let remote_id = session_id.to_owned();
        let session_id = session_id.to_owned();

        let task = tokio::spawn(async move {
            let session = tokio::spawn(crate::session::run(
                from_consumer,
                to_consumer,
                pool,
                // `None` on a board with no camera, and on a lane opened before the pipeline has
                // said what the video is — which is possible because the relay starts first, on
                // purpose. `session::run` refuses `media.video` rather than answering with zeros.
                video,
            ));

            let mut budget = Budget::new(NOTIFICATIONS_PER_WINDOW, NOTIFICATION_WINDOW);
            while let Some(line) = from_session.recv().await {
                let Ok(payload) = serde_json::from_str::<serde_json::Value>(&line) else {
                    // `session::run` builds every line through `duck-ipc-proto`, and a service's
                    // own output is forwarded verbatim — so this is a daemon emitting something
                    // that is not JSON, which is worth a line rather than a silent drop.
                    tracing::warn!(line = %line.chars().take(120).collect::<String>(),
                        "a control lane answer that is not JSON; dropping it");
                    continue;
                };
                // An answer has an id because the call it answers had one. Everything else is a
                // notification, and only notifications are rationed.
                let answers = !payload["id"].is_null();
                if !answers && !budget.take() {
                    continue;
                }
                let envelope = serde_json::json!({
                    "type": "peer",
                    "sessionId": session_id,
                    "rpc": payload,
                });
                if let Err(why) = hub.send(&envelope).await {
                    // The stream this token is bound to has gone, or the service refused. Either
                    // way the connection loop is about to find out on its own; this lane just
                    // stops.
                    tracing::info!(%why, "a control lane could not reach the service");
                    break;
                }
            }
            session.abort();
        });

        tracing::info!(%remote_id, "a control lane is open: JSON-RPC without a candidate pair");
        Control {
            remote_id,
            to_session,
            task,
        }
    }
}

// ── the local half: one bridged session ──────────────────────────────────────
//
// A remote consumer wants a session with this robot. On the LAN that consumer would open a
// WebSocket to `webrtcsink`'s own signalling server and ask it for one; from off the LAN it cannot
// reach that socket at all, so **this task plays the consumer on its behalf**: it opens the local
// WebSocket, asks for a session with the local producer, and then carries envelopes between the
// two sides, rewriting the one field whose value differs per hop.
//
// The roles are inverted on the two sides, and that is the whole shape of it (§3.1): to the
// rendezvous this process *is* the robot, and to `webrtcsink` it is a peer asking for a session.
//
// **One local connection per bridged session**, opened when the session starts and dropped when it
// ends. A long-lived local socket multiplexing several sessions would need the session table §3.2
// describes; one connection per session makes that table a single pair of ids, and the concurrent
// session the table would have existed for is refused anyway — the service gates it, and §3.4 says
// to keep gating it here too.

/// What the bridge needs to talk to the rendezvous while a session is in flight.
#[derive(Clone)]
struct Hub {
    client: reqwest::Client,
    base: String,
    token: String,
}

impl Hub {
    /// `POST /send`, for a task that has no `Relay` to hand.
    async fn send(&self, message: &serde_json::Value) -> Result<(), String> {
        let url = format!("{}/send", self.base);
        let response = self
            .client
            .post(&url)
            .bearer_auth(&self.token)
            .timeout(REQUEST_TIMEOUT)
            .json(message)
            .send()
            .await
            .map_err(|e| format!("POST {url}: {e}"))?;
        if !response.status().is_success() {
            return Err(format!("POST {url}: HTTP {}", response.status()));
        }
        Ok(())
    }

    /// Tell the service a session is over. Best effort: a failure here is a session that is
    /// already gone, and the peer finds out when the media stops either way.
    async fn end(&self, session_id: &str, reason: &str) {
        let message = serde_json::json!({
            "type": "endSession", "sessionId": session_id, "reason": reason,
        });
        if let Err(why) = self.send(&message).await {
            tracing::debug!(%why, "could not tell the service the session ended");
        }
    }
}

/// A bridged session, from the rendezvous side's point of view.
struct Bridged {
    /// The id the *service* knows this session by.
    remote_id: String,
    /// Envelopes from the service, on their way to the local signalling server.
    to_local: tokio::sync::mpsc::Sender<serde_json::Value>,
    task: tokio::task::JoinHandle<()>,
}

impl Bridged {
    /// Whether the task carrying this session is still running.
    fn live(&self) -> bool {
        !self.task.is_finished()
    }
}

impl Drop for Bridged {
    fn drop(&mut self) {
        // The task owns a WebSocket and a channel; aborting it closes both, which is what tells
        // `webrtcsink` the consumer went away.
        self.task.abort();
    }
}

/// Carry one session, and **tell the service when it is over however that happens**.
///
/// The wrapper exists for that second half. A consumer whose session ends without being told sits
/// looking at a robot it believes is connecting, forever — and the failure that produces it is
/// never the ordinary path, it is an early return from somewhere in the middle. So there is one
/// place that posts `endSession` and every exit goes through it.
async fn bridge(
    hub: Hub,
    local_url: String,
    remote_id: String,
    from_remote: tokio::sync::mpsc::Receiver<serde_json::Value>,
) -> Result<(), String> {
    let outcome = carry(&hub, &local_url, &remote_id, from_remote).await;
    match &outcome {
        Ok(why) => {
            hub.end(&remote_id, why).await;
            tracing::info!(remote = %remote_id, %why, "the bridged session ended");
        }
        Err(why) => {
            tracing::warn!(remote = %remote_id, %why, "the bridged session failed");
            hub.end(&remote_id, why).await;
        }
    }
    outcome.map(|_| ())
}

/// One session, from the local handshake to whichever side stops first.
///
/// `Ok` carries why it ended, which is what the consumer is told.
async fn carry(
    hub: &Hub,
    local_url: &str,
    remote_id: &str,
    mut from_remote: tokio::sync::mpsc::Receiver<serde_json::Value>,
) -> Result<String, String> {
    use futures_util::SinkExt;
    use tokio_tungstenite::tungstenite::Message;

    let (mut socket, _) = tokio_tungstenite::connect_async(local_url)
        .await
        .map_err(|e| format!("{local_url}: {e}"))?;

    // The consumer's own handshake against `webrtcsink`'s server: a welcome, then ask what is
    // producing, then ask that producer for a session. Exactly what the console page does over a
    // LAN, which is why the page was the reference for this rather than the protocol document.
    let mut producer = None;
    let mut local_id = None;
    let deadline = tokio::time::Instant::now() + LOCAL_HANDSHAKE_TIMEOUT;
    while local_id.is_none() {
        let message = tokio::time::timeout_at(deadline, socket.next())
            .await
            .map_err(|_| {
                format!("{local_url} did not start a session within {LOCAL_HANDSHAKE_TIMEOUT:?}")
            })?
            .ok_or_else(|| format!("{local_url} closed during the handshake"))?
            .map_err(|e| format!("{local_url}: {e}"))?;
        let Message::Text(text) = message else {
            continue;
        };

        // Read as the envelope it is rather than through `classify`: two of these three types
        // exist only on this side of the bridge, and giving them variants in the rendezvous's
        // vocabulary would put local-only messages in a remote-only enum.
        let value: serde_json::Value =
            serde_json::from_str(&text).map_err(|e| format!("{local_url}: {e}"))?;
        match value["type"].as_str().unwrap_or_default() {
            "welcome" => {
                socket
                    .send(Message::text(r#"{"type":"list"}"#))
                    .await
                    .map_err(|e| format!("{local_url}: {e}"))?;
            }
            "list" => {
                let first = value["producers"].get(0).and_then(|p| p["id"].as_str());
                let Some(id) = first else {
                    return Err(
                        "the robot's own signalling server lists no producer, so there is \
                         nothing to bridge — its pipeline has not reached PLAYING"
                            .to_owned(),
                    );
                };
                producer = Some(id.to_owned());
                let request = serde_json::json!({ "type": "startSession", "peerId": id });
                socket
                    .send(Message::text(request.to_string()))
                    .await
                    .map_err(|e| format!("{local_url}: {e}"))?;
            }
            "sessionStarted" => {
                local_id = value["sessionId"].as_str().map(str::to_owned);
            }
            _ => {}
        }
    }

    let local_id = local_id.expect("the loop above does not end until it is set");
    tracing::info!(
        remote = %remote_id,
        local = %local_id,
        producer = producer.as_deref().unwrap_or("unknown"),
        "a remote session is bridged to this robot's own signalling server"
    );

    // From here it is two directions and one rewritten field.
    let outcome = loop {
        tokio::select! {
            // The local producer: SDP, ICE, or the end of the session.
            message = socket.next() => {
                let Some(message) = message else {
                    break Err("the robot's signalling server closed the session".to_owned());
                };
                let message = message.map_err(|e| format!("{local_url}: {e}"))?;
                let Message::Text(text) = message else { continue };
                match classify(&text)? {
                    Inbound::Peer(mut envelope) => {
                        envelope["sessionId"] = serde_json::Value::String(remote_id.to_owned());
                        hub.send(&envelope).await?;
                    }
                    Inbound::EndSession { .. } => {
                        break Ok("the robot ended the session".to_owned());
                    }
                    _ => {}
                }
            }
            // The remote consumer, by way of the event stream this session arrived on.
            envelope = from_remote.recv() => {
                let Some(mut envelope) = envelope else {
                    break Ok("the rendezvous connection went away".to_owned());
                };
                envelope["sessionId"] = serde_json::Value::String(local_id.clone());
                socket
                    .send(Message::text(envelope.to_string()))
                    .await
                    .map_err(|e| format!("{local_url}: {e}"))?;
            }
        }
    };

    // The local side is told with a message rather than a dropped socket, so `webrtcsink` frees
    // its consumer immediately instead of on a timeout. The remote side is told by the caller,
    // which is the only place that does it.
    let farewell = serde_json::json!({ "type": "endSession", "sessionId": local_id });
    let _ = socket.send(Message::text(farewell.to_string())).await;
    let _ = socket.close(None).await;
    outcome
}

/// A duration plus up to [`BACKOFF_JITTER`] of itself, so a fleet does not reconnect in lockstep.
fn jittered(base: Duration) -> Duration {
    let spread = base.mul_f64(BACKOFF_JITTER);
    base + spread.mul_f64(rand::random::<f64>())
}
