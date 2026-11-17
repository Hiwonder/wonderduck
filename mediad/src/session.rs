//! One peer's control channel, as a pipe to the services that own the answers.
//!
//! Transport-agnostic on purpose: this takes lines in and gives lines out, and knows nothing about
//! datachannels. That is what makes it testable without a WebRTC peer — the tests below drive it
//! over channels against fake daemons on real unix sockets — and it is also what lets the
//! rendezvous control lane reuse it unchanged.
//!
//! ## What it does not do
//!
//! **It never parses a reply.** Requests are read far enough to route them and no further;
//! everything a service emits is forwarded verbatim. Two things follow, and both matter:
//!
//! - A subscription works with no special case. It is a stream of notifications on an open
//!   connection, and every one has to reach the peer — which correlating replies to requests would
//!   break, keeping the first and dropping the rest.
//! - Adding a method to the API costs nothing here. `duck-ipc-proto` stays the only place a method
//!   is defined, and this file does not grow a case for it.
//!
//! **It does not authenticate.** `remote-webrtc.md` §4: there is no gate on the robot. A LAN peer
//! may drive it, and a bridged peer authenticated to the rendezvous service — on both sides —
//! before arriving. `route::permits` refuses `system.authenticate` by name rather than answering
//! it, so a client that asks gets a clear no instead of a lie.

use duck_ipc_proto as proto;
use tokio::sync::mpsc;

use crate::route::{self, Route};
use crate::upstream::Pool;

/// Drive one peer's control channel until its inbound stream ends.
///
/// `inbound` carries one JSON-RPC object per item, as the peer sent it. `outbound` is where replies
/// and notifications go, merged from every service — the peer sorts them out by `id`, which is its
/// business rather than ours.
/// What the video is: the frame the encoder sends, and how far the camera is mounted from upright.
///
/// Held by the session because a peer has to be able to *ask*. Pushing it when the channel appears
/// races the browser: `mediad` writes the line the moment `webrtcsink` hands over the channel, and
/// if the peer's datachannel is not open yet the line is dropped — which is exactly what happened,
/// and the console showed a sideways picture with nothing in the log to say why. The push is kept
/// as a courtesy for a client that only listens; `media.video` as a *call* is what the console uses,
/// because a question it asks when it is ready cannot arrive too early.
#[derive(Debug, Clone, PartialEq)]
pub struct Video {
    pub width: u32,
    pub height: u32,
    /// Degrees clockwise the camera is mounted from upright.
    pub rotate: u32,
    /// The camera's geometry, for a consumer that has to turn pixels into directions — SLAM,
    /// visual odometry, "how far is that". `None` when there is no camera or its mode is not one
    /// `crate::camera` knows the field of view of, which is a robot that should not be believed
    /// rather than one to guess about.
    pub intrinsics: Option<crate::camera::Intrinsics>,
}

/// The methods a peer asks with. Answered here rather than routed: no service owns them.
const VIDEO_METHOD: &str = "media.video";
const STREAM_METHOD: &str = "media.stream";

/// What this daemon can answer about its own media, when it has a pipeline to answer from.
///
/// One handle rather than two Options, because the two arrive together: both are known only once
/// the pipeline is up — `sensor_mode()` is not truthful before then, and there are no frames to
/// encode either — and both are absent for the same reasons, a board with no camera or a lane
/// opened before the pipeline started. A peer asking either gets a refusal that says which.
#[derive(Clone)]
pub struct Media {
    pub video: Video,
    pub streamer: std::sync::Arc<crate::stream::Streamer>,
}

/// What the video is, told to a peer once when its channel opens.
///
/// **The rotation is the whole point.** Nothing on the robot rotates pixels any more — a `videoflip`
/// in the pipeline cost the encoder its zero-copy path and the board 22 fps — so the stream a
/// browser receives is the picture the camera took, sideways on a robot whose camera is mounted a
/// quarter turn off. The page cannot work out by how much: a 180° mount is indistinguishable from an
/// upright one, and even a quarter turn is only a guess from the aspect ratio. So it is told.
///
/// A notification, with no id, because the page already treats an id-less line as something that
/// streams (`robot.state` is the other one) — no new mechanism at either end.
pub fn video_notification(video: &Video) -> String {
    // Built with `serde_json` rather than `format!` since it gained a nested object: one escaping
    // mistake in a hand-written line is a peer that cannot parse anything this daemon says.
    serde_json::json!({
        "jsonrpc": "2.0",
        "method": VIDEO_METHOD,
        "params": video_params(video),
    })
    .to_string()
}

/// The video's description, which the notification pushes and the call answers with.
///
/// One function for both, because a peer that asks and a peer that listens must be told the same
/// thing — and the console does both, a push when the channel opens and a call when it is ready.
fn video_params(video: &Video) -> serde_json::Value {
    let mut params = serde_json::json!({
        "width": video.width,
        "height": video.height,
        "rotate": video.rotate,
        // The two clocks at one instant: RTCP sender reports state RTP time in wall-clock
        // (`real_ns`), `robot.state`/`tof.frame` stamp with `mono_ns`'s clock. A peer that has both
        // can put the picture on the robot's axis.
        "mono_ns": proto::clock::monotonic_ns(),
        "real_ns": proto::clock::realtime_ns(),
    });
    // Absent rather than null when the geometry is unknown: a consumer reading a missing key knows
    // it must calibrate, where one reading `null` has to be told what that meant.
    if let Some(intrinsics) = &video.intrinsics {
        params["intrinsics"] = serde_json::to_value(intrinsics).expect("intrinsics serialise");
    }
    params
}

/// `media` is `None` when there is no pipeline to answer for.
///
/// A datachannel always has one — it exists because `webrtcsink` handed over a consumer — but the
/// rendezvous control lane (`relay::Relay::control`) is JSON-RPC with no media beside it, and it
/// can open before the pipeline has started. Answering `media.video` with zeros in that case would
/// be a consumer told the picture is 0×0 and upright, which is worse than being told there is no
/// picture: one is actionable, the other is a wrong number that looks like a right one.
pub async fn run(
    mut inbound: mpsc::Receiver<String>,
    outbound: mpsc::Sender<String>,
    mut pool: Pool,
    media: Option<Media>,
) {
    while let Some(line) = inbound.recv().await {
        let line = line.trim().to_string();
        if line.is_empty() {
            continue;
        }
        if let Some(reply) = handle(&line, &mut pool, media.as_ref()).await {
            // A closed outbound means the peer is gone; there is nothing left to do for it.
            if outbound.send(reply).await.is_err() {
                break;
            }
        }
    }
    tracing::debug!("control channel ended");
}

/// Route one line. Returns a reply to send back only when this transport answers it itself —
/// which is to say, only when it refuses.
async fn handle(line: &str, pool: &mut Pool, media: Option<&Media>) -> Option<String> {
    let request: proto::Request = match serde_json::from_str(line) {
        Ok(request) => request,
        Err(e) => {
            // Unparseable, so there is no `id` to answer against and no method to name. A
            // JSON-RPC parse error with a null id is the honest reply.
            tracing::debug!(error = %e, "unparseable frame on the control channel");
            return Some(error_line(
                None,
                proto::Error::new(proto::code::PARSE_ERROR, "not a JSON-RPC object"),
            ));
        }
    };

    // The id, kept before the request is consumed. A notification has none, and `Response`
    // serialises that as `null` — which is right: a refusal to a notification is still worth
    // sending, because silence would look like acceptance.
    let id = request.id.clone();

    // Answered here, before anything tries to make a `Call` of it: this is `mediad`'s own question
    // about `mediad`'s own pipeline, and there is no service to route it to.
    if request.method == VIDEO_METHOD {
        return Some(match media {
            Some(media) => {
                serde_json::to_string(&proto::Response::ok(id, &video_params(&media.video)))
                    .expect("Response serialises")
            }
            None => error_line(
                id,
                proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    "this robot is not publishing video, so there is no geometry to describe",
                ),
            ),
        });
    }

    // **The one call that makes this robot send its camera somewhere it was told about.**
    //
    // Answered here rather than routed for the same reason `media.video` is — the pipeline is
    // `mediad`'s and no service owns it — and it is the reason a frame stream needs no relay
    // candidate: the robot dials out, so a NAT is not a participant. `stream.rs`'s header has the
    // argument; this is where a peer asks for it.
    if request.method == STREAM_METHOD {
        let Some(media) = media else {
            return Some(error_line(
                id,
                proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    "this robot has no camera to stream",
                ),
            ));
        };
        return Some(match stream_request(&request.params) {
            // No `url` key at all is a question rather than an instruction, which is what lets a
            // client show what is streaming without having to remember what it asked for.
            Ask::Status => {
                serde_json::to_string(&proto::Response::ok(id, &media.streamer.status()))
                    .expect("Response serialises")
            }
            Ask::Stop => serde_json::to_string(&proto::Response::ok(id, &media.streamer.stop()))
                .expect("Response serialises"),
            Ask::Start(config) => match media.streamer.start(config) {
                Ok(answer) => serde_json::to_string(&proto::Response::ok(id, &answer))
                    .expect("Response serialises"),
                Err(why) => error_line(id, proto::Error::new(proto::code::INVALID_PARAMS, why)),
            },
        });
    }

    let call = match request.as_call() {
        Ok(call) => call,
        Err(e) => {
            // An unknown method or a params shape this release does not know. `as_call` names
            // which, and that message is the whole value — a version skew fails on the one call
            // that cannot be served rather than on the handshake.
            return Some(error_line(id, e));
        }
    };

    match route::route_for(&call) {
        Route::Refused => {
            tracing::debug!(method = call.method(), "refused over WebRTC");
            Some(error_line(id, route::refusal(&call)))
        }
        Route::To(service, lane) => {
            match pool.send(service, lane, line).await {
                Ok(()) => None,
                Err(e) => {
                    // The service is not there, or stopped reading. Answering is important: a
                    // client that gets silence cannot tell "the robot is thinking" from "nothing
                    // will ever come back", and `robotd` is the service most likely to be missing
                    // because it is the one an update restarts.
                    tracing::debug!(method = call.method(), error = %e, "upstream unreachable");
                    Some(error_line(
                        id,
                        proto::Error::new(
                            proto::code::INTERNAL_ERROR,
                            format!("{:?} is not answering: {e}", service),
                        ),
                    ))
                }
            }
        }
    }
}

/// One refusal, as a line. Built through [`proto::Response`] rather than by hand so the envelope
/// has exactly one definition — the same reason `duck-ipc-proto` exists.
/// What a `media.stream` call is asking for.
///
/// `url` present is start, `url: null` is stop, no `url` key is a question. The same three-way
/// reading `robot.loadPolicy` gives `slot` and `path`, and for the same reason: one method that a
/// client can use to look, to change and to put back is one method to route and one to permit.
enum Ask {
    Status,
    Stop,
    Start(crate::stream::Config),
}

fn stream_request(params: &Option<serde_json::Value>) -> Ask {
    use crate::stream::Config;

    // Absent params is the same question as params with no `url`: a client asking what is
    // streaming should not have to send `{}` to be understood.
    let params = match params {
        None => return Ask::Status,
        Some(params) => params,
    };
    match params.get("url") {
        None => Ask::Status,
        Some(serde_json::Value::Null) => Ask::Stop,
        Some(url) => Ask::Start(Config {
            url: url.as_str().unwrap_or_default().to_owned(),
            // Every number is optional: what a caller almost always wants is "stream to here",
            // and a default that is cheap enough to ignore is better than four required fields.
            fps: params
                .get("fps")
                .and_then(serde_json::Value::as_f64)
                .unwrap_or(Config::DEFAULT_FPS),
            longest: params
                .get("longest")
                .and_then(serde_json::Value::as_u64)
                .unwrap_or(Config::DEFAULT_LONGEST as u64) as u32,
            quality: params
                .get("quality")
                .and_then(serde_json::Value::as_u64)
                .unwrap_or(Config::DEFAULT_QUALITY as u64)
                .min(100) as u8,
            // H.264 unless asked otherwise: the encode is the VPU's rather than a core's, and
            // inter-frame prediction is worth five to fifteen times the bytes over the same
            // wifi. JPEG stays reachable because it needs no keyframe to start and no decoder
            // state to keep, which is what a receiver that reconnects constantly wants.
            encoding: params
                .get("encoding")
                .and_then(serde_json::Value::as_str)
                .and_then(crate::stream::Encoding::parse)
                .unwrap_or_default(),
        }),
    }
}

fn error_line(id: Option<proto::Id>, error: proto::Error) -> String {
    // A `Response` cannot fail to serialise: every field is a `String`, an `Id` or an `Error`.
    serde_json::to_string(&proto::Response::err(id, error)).expect("Response serialises")
}
