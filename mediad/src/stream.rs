//! Frames out to a WebSocket **this robot dials**, for a Space that runs a model on them.
//!
//! # This is the fallback, and WebRTC is the default
//!
//! **A consumer should use WebRTC**, which carries encrypted media, a control channel on the same
//! session and a return path, and which reaches a data centre because the robot offers a relay
//! candidate. `docs/faq.md` is the decision, in the shape somebody
//! arrives at it. This module is for the narrow case WebRTC serves badly: a **program** consuming
//! **frames only** on a **long-running** stream, where a relay's metered bandwidth is the cost
//! that matters.
//!
//! In that case an outbound WebSocket needs nobody's relay. **The robot already proves this every
//! second it is reachable**: `relay.rs` holds an outbound HTTPS stream to a Space right now, and
//! nothing about a home router objects. So the frames go the same way the registration does —
//! outward — and NAT stops being a participant.
//!
//! ```text
//!   Space  ──media.stream {url: "wss://…/frames"}──►  rendezvous  ──►  this robot
//!   robot  ═══════════ JPEG frames, outbound wss, direct ═══════════►  Space
//! ```
//!
//! The rendezvous carries **the instruction and not the pixels**, which is the property that makes
//! this scale where relaying payload through a shared service would not: one small envelope per
//! session, on a service the mini fleet also depends on, and the bytes go point to point.
//!
//! **This was written when the relay endpoint was dead and WebRTC could not connect from a data
//! centre at all.** That is fixed (§6), so the reason this exists is now the narrow one above and
//! not "the alternative does not work". What survives of the original argument is the cost: a
//! relay is metered per Hugging Face account at 10 GB a month, and a stream that runs all day
//! spends an allowance its owner also needs for being *watched*.
//!
//! # What it is not
//!
//! **Not a replacement for WebRTC, and not the path to reach for first.** There is no return
//! media path and no control channel, so nothing here helps a browser *watch* a robot, carries
//! audio, or closes a teleop loop; driving means a separate JSON-RPC call over the rendezvous.
//! Encryption is the receiver's TLS rather than DTLS-SRTP, terminating at a server instead of at
//! the peer. A consumer that is not all three of program, frames-only and long-running wants §6.
//!
//! # This half is portable, and that is deliberate
//!
//! [`pump`] takes a channel of already-encoded frames and knows nothing about GStreamer, so the
//! reconnect, the backoff, the framing and the counters are all exercised on a laptop against a
//! fake server. What touches [`crate::pipeline::Frames`] is [`encode_frames`], which is the thin
//! linux-only half: read a frame, turn it upright, JPEG it, hand it over. The same split
//! `session.rs` has, for the same reason — every failure worth testing here is a timing failure.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::Duration;

use futures_util::{SinkExt as _, StreamExt as _};
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
use image::ImageEncoder as _;

/// What the frames are, on the wire.
///
/// The hello carries this, so a receiver branches on it rather than sniffing bytes — and so that
/// adding the second one was a field rather than a protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Encoding {
    /// One JPEG per message, every one independently decodable.
    Jpeg,
    /// H.264 access units, SPS/PPS in front of every keyframe.
    ///
    /// **Cheaper on the board and on the wire, and not free.** The encode is the VPU's rather than
    /// the CPU's and inter-frame prediction is worth five to fifteen times the bytes — but a
    /// receiver joining mid-stream can decode nothing until a keyframe arrives, and a dropped
    /// frame corrupts every frame after it until the next one. Both of those are handled here
    /// rather than wished away: see [`Streamer::start`]'s drop policy and
    /// `pipeline::force_keyframe`.
    #[default]
    H264,
}

impl Encoding {
    pub fn wire_name(self) -> &'static str {
        match self {
            Self::Jpeg => "jpeg",
            Self::H264 => "h264",
        }
    }

    pub fn parse(name: &str) -> Option<Self> {
        match name.to_ascii_lowercase().as_str() {
            "jpeg" | "jpg" | "mjpeg" => Some(Self::Jpeg),
            "h264" | "avc" => Some(Self::H264),
            _ => None,
        }
    }
}

/// One encoded thing to send: a JPEG, or an H.264 access unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Unit {
    pub bytes: Vec<u8>,
    /// Whether a receiver could start decoding here. Always true for JPEG; true for an H.264 IDR.
    ///
    /// **This is what makes the drop policy correct.** Discarding the oldest and keeping the
    /// newest is right for independent frames and wrong for a predicted stream: the newest P-frame
    /// refers to one that was thrown away, so a receiver decodes garbage until the next keyframe.
    /// Knowing which is which is the difference between dropping a frame and corrupting a second.
    pub keyframe: bool,
}

/// What to send, and where.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// `wss://…` for anything that is not on this network. `ws://` is accepted because a laptop
    /// on the bench serving a receiver over TLS is a certificate nobody wants to make, and it is
    /// logged either way — the destination of this robot's camera is worth a line in the journal.
    pub url: String,
    /// Frames a second. Not the capture rate: every frame costs a colour conversion and a JPEG on
    /// the CPU, and what receives them is a model that does not want thirty.
    pub fps: f64,
    /// The longest side, in pixels. Downscale only.
    pub longest: u32,
    /// JPEG quality, 1..=100. Ignored for H.264, whose bitrate is the pipeline's.
    pub quality: u8,
    pub encoding: Encoding,
}

impl Config {
    /// What a caller gets for asking for nothing: enough for a model, cheap enough to ignore.
    pub const DEFAULT_FPS: f64 = 5.0;
    pub const DEFAULT_LONGEST: u32 = 640;
    pub const DEFAULT_QUALITY: u8 = 70;

    /// How long to wait between pulls, which is nothing when the source paces itself.
    ///
    /// A JPEG is made on demand from whatever the camera last captured, so the rate is this
    /// thread's to keep. An H.264 unit arrives when the encoder emits one, and the rate was
    /// already imposed by a `videorate` in the pipeline — so waiting here would delay a frame
    /// that exists rather than avoid making one.
    pub fn interval(&self) -> Duration {
        if self.encoding == Encoding::H264 {
            return Duration::ZERO;
        }
        let fps = if self.fps.is_finite() {
            self.fps
        } else {
            Self::DEFAULT_FPS
        };
        Duration::from_secs_f64(1.0 / fps.clamp(0.2, 15.0))
    }
}

/// What a stream has done so far, for `media.stream`'s answer.
#[derive(Debug, Default)]
pub struct Counters {
    pub connected: AtomicBool,
    /// Frames handed to the socket.
    pub sent: AtomicU64,
    /// Frames encoded and thrown away because the socket was not keeping up.
    ///
    /// **Dropping is correct, and what to drop depends on the encoding.** For JPEG the newest
    /// frame is what a model wants, so the oldest goes. For H.264 a gap corrupts everything until
    /// the next keyframe, so the stream is abandoned to the next one instead — which is why this
    /// can jump by a whole group of pictures at a time.
    pub dropped: AtomicU64,
    /// Text frames the far end sent back — a model's answers, if it sends any.
    pub replies: AtomicU64,
    pub reconnects: AtomicU64,
}

/// How long to wait before redialling, and how much of itself to add as jitter.
///
/// A struct rather than constants for the reason `relay::Timings` is one: every failure worth
/// testing here is a timing failure, and a test that waited two real seconds per reconnect would
/// be a test nobody runs. `tokio`'s paused clock would do it too, at the cost of a feature on the
/// dependency for the whole crate.
#[derive(Debug, Clone, Copy)]
pub struct Timings {
    pub backoff_start: Duration,
    pub backoff_max: Duration,
    pub jitter: f64,
}

impl Default for Timings {
    fn default() -> Self {
        Self {
            backoff_start: Duration::from_secs(2),
            backoff_max: Duration::from_secs(30),
            jitter: 0.2,
        }
    }
}

/// Dial `url`, send `hello`, then forward every frame until told to stop.
///
/// Reconnects for as long as `running` says so: a Space restarts on every push and sleeps when
/// nobody is looking at it, so a receiver going away is the ordinary case rather than the end of
/// the stream. Frames encoded while there is no socket are dropped, not queued — see
/// [`Counters::dropped`].
pub async fn pump(
    url: String,
    token: Option<String>,
    hello: String,
    mut frames: tokio::sync::mpsc::Receiver<Vec<u8>>,
    counters: Arc<Counters>,
    running: Arc<AtomicBool>,
    timings: Timings,
) {
    let mut backoff = timings.backoff_start;
    while running.load(Ordering::Relaxed) {
        match connect(&url, token.as_deref()).await {
            Err(why) => {
                counters.connected.store(false, Ordering::Relaxed);
                tracing::warn!(%url, %why, "the frame receiver could not be reached");
            }
            Ok(mut socket) => {
                counters.connected.store(true, Ordering::Relaxed);
                backoff = timings.backoff_start;
                tracing::info!(%url, "streaming frames to a receiver this robot dialled");

                let outcome = carry(&mut socket, &hello, &mut frames, &counters, &running).await;
                counters.connected.store(false, Ordering::Relaxed);
                // Closed politely, so the far end knows this was not a crash.
                let _ = socket.close(None).await;
                match outcome {
                    Carried::Stopped => break,
                    Carried::Lost(why) => tracing::info!(%url, %why, "the frame stream dropped"),
                }
            }
        }
        if !running.load(Ordering::Relaxed) {
            break;
        }
        counters.reconnects.fetch_add(1, Ordering::Relaxed);
        tokio::time::sleep(jittered(backoff, timings.jitter)).await;
        backoff = (backoff * 2).min(timings.backoff_max);
    }
    counters.connected.store(false, Ordering::Relaxed);
    tracing::info!(%url, "the frame stream ended");
}

type Socket =
    tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

/// Why [`carry`] returned, which decides whether to redial.
enum Carried {
    /// Asked to stop. Nothing to redial.
    Stopped,
    Lost(String),
}

async fn connect(url: &str, token: Option<&str>) -> Result<Socket, String> {
    use tokio_tungstenite::tungstenite::client::IntoClientRequest as _;

    let mut request = url
        .into_client_request()
        .map_err(|e| format!("{url} is not a WebSocket url: {e}"))?;
    if let Some(token) = token {
        // **The robot's own credential, as the handshake header.** The receiver is a public
        // endpoint, so it has to be able to say whose camera this is — the same `whoami-v2`
        // resolution the rendezvous does with this token. Without it a Space would take frames
        // from anybody and show them to anybody.
        let value = format!("Bearer {token}")
            .parse()
            .map_err(|_| "the account token is not a header value".to_owned())?;
        request.headers_mut().insert("authorization", value);
    }
    let (socket, _) = tokio_tungstenite::connect_async(request)
        .await
        .map_err(|e| e.to_string())?;
    Ok(socket)
}

async fn carry(
    socket: &mut Socket,
    hello: &str,
    frames: &mut tokio::sync::mpsc::Receiver<Vec<u8>>,
    counters: &Counters,
    running: &AtomicBool,
) -> Carried {
    use tokio_tungstenite::tungstenite::Message;

    // **What is coming, before any of it arrives.** A receiver reading binary frames has no way to
    // know their size, their rate or which robot's camera they are, and guessing from the first
    // JPEG is a decoder's job rather than a protocol.
    if let Err(e) = socket.send(Message::Text(hello.into())).await {
        return Carried::Lost(format!("the hello would not send: {e}"));
    }

    loop {
        tokio::select! {
            frame = frames.recv() => match frame {
                None => return Carried::Stopped,
                Some(frame) => {
                    if !running.load(Ordering::Relaxed) {
                        return Carried::Stopped;
                    }
                    if let Err(e) = socket.send(Message::Binary(frame.into())).await {
                        return Carried::Lost(format!("a frame would not send: {e}"));
                    }
                    counters.sent.fetch_add(1, Ordering::Relaxed);
                }
            },
            // **Read, or the connection dies of politeness.** tungstenite answers pings while the
            // stream is polled and not otherwise, so a sender that only ever writes stops
            // answering keepalives and the far end hangs up. Whatever the receiver says back — a
            // model's answer — is counted and logged rather than acted on: nothing on this robot
            // has asked to be driven by it.
            inbound = socket.next() => match inbound {
                None => return Carried::Lost("the receiver closed the socket".to_owned()),
                Some(Err(e)) => return Carried::Lost(e.to_string()),
                Some(Ok(Message::Close(_))) => {
                    return Carried::Lost("the receiver said goodbye".to_owned());
                }
                Some(Ok(Message::Text(text))) => {
                    counters.replies.fetch_add(1, Ordering::Relaxed);
                    tracing::debug!(reply = %text.chars().take(200).collect::<String>(),
                        "the frame receiver answered");
                }
                Some(Ok(_)) => {}
            },
        }
    }
}

fn jittered(base: Duration, jitter: f64) -> Duration {
    base + base.mul_f64(jitter).mul_f64(rand::random::<f64>())
}

/// One encoded frame, on demand, at the size and quality asked for.
///
/// A closure rather than a `Frames` handle, and that is what keeps this module portable:
/// everything that touches GStreamer lives on the far side of it, and the linux-only half is
/// [`jpeg_encoder`] alone. On a laptop a test supplies a closure that returns bytes.
pub type Encode = Arc<dyn Fn(&Config) -> Option<Unit> + Send + Sync>;

/// The ways this robot can encode what it streams, and the gate on the expensive one.
///
/// Both are held rather than one chosen at startup, because `media.stream` names an encoding per
/// call: H.264 by default, and JPEG for a receiver that reconnects constantly enough to care more
/// about starting instantly than about bytes.
pub struct Encoders {
    pub jpeg: Encode,
    /// `None` on a board with no H.264 encoder at all, where asking for it is refused with that
    /// as the reason rather than accepted and silently served as JPEG.
    pub h264: Option<Encode>,
    /// Opens and shuts the H.264 branch's valve. `true` on start, `false` on stop.
    pub gate: Option<Arc<dyn Fn(bool) + Send + Sync>>,
}

/// The robot's frame stream: at most one, started and stopped by `media.stream`.
///
/// **One at a time**, and unlike the media path that is a resource decision rather than a
/// protocol one: every frame costs a colour conversion and a JPEG on a CPU that is also running a
/// control loop, so two receivers would be two of that. A second `media.stream` replaces the
/// first, which is also the only way to change the rate without a stop.
pub struct Streamer {
    encoders: Encoders,
    producer: crate::producer::Producer,
    /// The mount angle, for the hello — these frames are already upright, and a receiver has to be
    /// told that rather than left to apply it twice.
    rotate: u32,
    token_path: std::path::PathBuf,
    timings: Timings,
    live: std::sync::Mutex<Option<Live>>,
}

struct Live {
    config: Config,
    running: Arc<AtomicBool>,
    counters: Arc<Counters>,
}

impl Drop for Live {
    fn drop(&mut self) {
        // Both halves watch this: the encoder thread stops reading frames, and the pump stops
        // redialling. Dropping the channel is what actually wakes the pump.
        self.running.store(false, Ordering::Relaxed);
    }
}

impl Streamer {
    pub fn new(
        encoders: Encoders,
        producer: crate::producer::Producer,
        rotate: u32,
        token_path: impl Into<std::path::PathBuf>,
    ) -> Self {
        Self {
            encoders,
            producer,
            rotate,
            token_path: token_path.into(),
            timings: Timings::default(),
            live: std::sync::Mutex::new(None),
        }
    }

    pub fn with_timings(mut self, timings: Timings) -> Self {
        self.timings = timings;
        self
    }

    /// Start streaming, replacing whatever was streaming before.
    pub fn start(&self, config: Config) -> Result<serde_json::Value, String> {
        if config.url.is_empty() {
            return Err("no url to stream to".to_owned());
        }
        if !(config.url.starts_with("ws://") || config.url.starts_with("wss://")) {
            return Err(format!("{} is not a ws:// or wss:// url", config.url));
        }
        if !(1..=100).contains(&config.quality) {
            return Err("quality is 1..=100".to_owned());
        }

        // **Logged before anything is sent, at info.** This is the one call that makes a robot
        // hand its camera to somewhere it was told about rather than somewhere it knows, so where
        // that was has to be in the journal whether or not anybody was watching the page that
        // asked. `remote-webrtc.md` §4 leaves this transport ungated on purpose; a line in the
        // log is what makes that decision auditable rather than invisible.
        tracing::info!(
            url = %config.url, fps = config.fps, longest = config.longest,
            "asked to stream frames out"
        );

        let encode = match config.encoding {
            Encoding::Jpeg => Arc::clone(&self.encoders.jpeg),
            Encoding::H264 => match self.encoders.h264.as_ref() {
                Some(encode) => Arc::clone(encode),
                None => {
                    return Err(
                        "this robot has no H.264 encoder, so it can only stream jpeg — ask for \
                         `\"encoding\": \"jpeg\"`"
                            .to_owned(),
                    );
                }
            },
        };
        // Opened before the encoder thread reads, or its first pull waits a whole frame timeout
        // for a branch nothing is flowing through.
        if let Some(gate) = self.encoders.gate.as_ref() {
            gate(config.encoding == Encoding::H264);
        }

        let running = Arc::new(AtomicBool::new(true));
        let counters = Arc::new(Counters::default());
        // Two deep: the newest frame and one in flight. A model wants the freshest picture, so a
        // frame encoded while the socket is behind is dropped rather than queued — a backlog is
        // latency that never comes back, and `Counters::dropped` is how a caller sees it happening.
        let (to_socket, from_encoder) = tokio::sync::mpsc::channel::<Vec<u8>>(2);

        let interval = config.interval();
        let for_encoder = config.clone();
        let encoding = Arc::clone(&running);
        let counted = Arc::clone(&counters);
        std::thread::Builder::new()
            .name("frame-stream".to_owned())
            .spawn(move || {
                // Set after a drop on a predicted stream: everything until the next keyframe
                // would decode against a frame the receiver never got, so it is skipped rather
                // than sent. For JPEG this never becomes true — every unit is a keyframe.
                let mut awaiting_key = false;

                while encoding.load(Ordering::Relaxed) {
                    let began = std::time::Instant::now();
                    if let Some(unit) = encode(&for_encoder) {
                        if awaiting_key && !unit.keyframe {
                            counted.dropped.fetch_add(1, Ordering::Relaxed);
                            continue;
                        }
                        awaiting_key = false;
                        match to_socket.try_send(unit.bytes) {
                            Ok(()) => {}
                            Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                                counted.dropped.fetch_add(1, Ordering::Relaxed);
                                // A gap has happened. On a predicted stream the only safe thing
                                // to send next is a keyframe.
                                awaiting_key = !unit.keyframe || awaiting_key;
                                if for_encoder.encoding == Encoding::H264 {
                                    awaiting_key = true;
                                }
                            }
                            // The pump is gone, so there is nowhere left to send.
                            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => return,
                        }
                    }
                    // Zero for a source that paces itself: an H.264 appsink blocks until the
                    // encoder produces, and `videorate` upstream is what limits the rate. Sleeping
                    // on top of that would only add latency to frames that already exist.
                    if let Some(rest) = interval.checked_sub(began.elapsed()) {
                        std::thread::sleep(rest);
                    }
                }
            })
            .map_err(|e| format!("no thread for the frame stream: {e}"))?;

        tokio::spawn(pump(
            config.url.clone(),
            hf_robot_account::read_access_token(&self.token_path),
            hello(&config, &self.producer, self.rotate),
            from_encoder,
            Arc::clone(&counters),
            Arc::clone(&running),
            self.timings,
        ));

        let answer = describe(Some(&config), &counters);
        *self.live.lock().expect("not poisoned") = Some(Live {
            config,
            running,
            counters,
        });
        Ok(answer)
    }

    /// Stop streaming. Answers whether there was anything to stop.
    pub fn stop(&self) -> serde_json::Value {
        let was = self.live.lock().expect("not poisoned").take();
        // Shut the branch whatever it was streaming: a valve left open is a second encode for
        // nobody, which is the cost this whole arrangement exists to avoid.
        if let Some(gate) = self.encoders.gate.as_ref() {
            gate(false);
        }
        match &was {
            Some(live) => tracing::info!(url = %live.config.url, "the frame stream was stopped"),
            None => tracing::debug!("nothing was streaming"),
        }
        serde_json::json!({ "streaming": false, "was": was.as_ref().map(|l| l.config.url.clone()) })
    }

    /// What is streaming and how it is going.
    pub fn status(&self) -> serde_json::Value {
        let live = self.live.lock().expect("not poisoned");
        match live.as_ref() {
            None => describe(None, &Counters::default()),
            Some(live) => describe(Some(&live.config), &live.counters),
        }
    }
}

fn describe(config: Option<&Config>, counters: &Counters) -> serde_json::Value {
    let mut answer = serde_json::json!({
        "streaming": config.is_some(),
        "sent": counters.sent.load(Ordering::Relaxed),
        "dropped": counters.dropped.load(Ordering::Relaxed),
        "replies": counters.replies.load(Ordering::Relaxed),
        "reconnects": counters.reconnects.load(Ordering::Relaxed),
        "connected": counters.connected.load(Ordering::Relaxed),
    });
    if let Some(config) = config {
        answer["url"] = config.url.clone().into();
        answer["fps"] = config.fps.into();
        answer["longest"] = config.longest.into();
        answer["quality"] = config.quality.into();
    }
    answer
}

/// The line that opens a stream, so a receiver knows what it is about to be sent.
pub fn hello(config: &Config, producer: &crate::producer::Producer, rotate: u32) -> String {
    serde_json::json!({
        "type": "hello",
        "robot": {
            "name": producer.name,
            "serial": producer.serial,
            "release": producer.release,
            "api_version": producer.api_version,
            "kind": "wonderduck",
        },
        "frames": {
            "encoding": config.encoding.wire_name(),
            "longest": config.longest,
            "fps": config.fps,
            "quality": config.quality,
            // Zero, always, and said out loud: unlike the WebRTC path, these frames are turned
            // upright before they are encoded. A receiver that honoured a mount angle here would
            // rotate an already-upright picture.
            "rotate": 0,
            "mount_rotate": rotate,
            // H.264 only, and stated because it decides how a receiver frames what arrives: one
            // WebSocket message is one access unit, with SPS and PPS repeated in front of every
            // keyframe (`h264parse config-interval=-1`), so a receiver that joins mid-stream needs
            // nothing from the messages it missed.
            "annexb": config.encoding == Encoding::H264,
        },
    })
    .to_string()
}

/// The linux half for H.264: take the next access unit the branch's encoder produced.
///
/// Nothing is converted or copied here beyond the unit itself — the VPU did the work, upstream of
/// an appsink — which is the whole reason to prefer this over JPEG on a board. The rate was
/// imposed by a `videorate` in the branch, so this blocks until there is something and never
/// paces anything itself ([`Config::interval`] returns zero for it).
///
/// Opening the valve is [`Encoders::gate`]'s job rather than this closure's, so that a stream
/// which stops shuts it again: a second encoder running for nobody is what the valve exists to
/// prevent.
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
pub fn h264_encoder(branch: crate::pipeline::StreamBranch) -> Encode {
    Arc::new(move |_config: &Config| {
        let (bytes, keyframe) = branch.encoded.next_unit()?;
        Some(Unit { bytes, keyframe })
    })
}

/// The linux half: read a frame off the tee, turn it upright, and JPEG it.
///
/// **The only part of this module that knows a pipeline exists.** Everything above takes bytes
/// from a channel, which is what lets the reconnect and the framing be tested on a laptop; this is
/// four lines of pixels and a call into `duck-detect`, where the UYVY arithmetic already lives
/// because the detector needed exactly the same conversion.
///
/// `next_frame` blocks on a condvar until the *next* capture, so this runs on its own thread —
/// [`Streamer::start`] gives it one, the same way the detector and the exposure loop have theirs.
/// A frame is asked for rather than published, which is the whole design of `Frames`: at 5 fps
/// this copies five of thirty rather than all thirty.
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
pub fn jpeg_encoder(frames: crate::pipeline::Frames, turn: uyvy::Turn) -> Encode {
    // Reused across frames: at 640×480 the RGB buffer is 920 KB, and allocating that five times a
    // second forever is a page fault storm for no reason. A `Mutex` because `Encode` is `Fn` — one
    // thread ever takes it, so it is uncontended by construction.
    let scratch = std::sync::Mutex::new((Vec::<u8>::new(), Vec::<u8>::new()));

    Arc::new(move |config: &Config| {
        let frame = frames.next_frame()?;
        if frame.format != crate::pipeline::CAPTURE_FORMAT {
            // The caps changed under us. Refusing beats encoding one format as another, which
            // produces a picture that is wrong in a way a receiver cannot detect.
            tracing::warn!(
                format = frame.format,
                expected = crate::pipeline::CAPTURE_FORMAT,
                "not streaming a frame in a format this encoder does not know"
            );
            return None;
        }

        let mut held = scratch.lock().expect("not poisoned");
        let (rgb, jpeg) = &mut *held;
        let (width, height) = uyvy::rgb_from_uyvy(
            &frame.data,
            frame.width as usize,
            frame.height as usize,
            config.longest as usize,
            turn,
            rgb,
        );

        jpeg.clear();
        let encoder =
            image::codecs::jpeg::JpegEncoder::new_with_quality(&mut *jpeg, config.quality);
        match encoder.write_image(
            rgb,
            width as u32,
            height as u32,
            image::ExtendedColorType::Rgb8,
        ) {
            // Every JPEG is a keyframe by construction, which is the property that makes this
            // encoding worth keeping: a receiver that reconnects can decode the very next
            // message, where H.264 has to wait for one.
            Ok(()) => Some(Unit {
                bytes: jpeg.clone(),
                keyframe: true,
            }),
            Err(e) => {
                tracing::warn!(error = %e, "a frame would not encode");
                None
            }
        }
    })
}
