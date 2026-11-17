//! The page, served by the daemon it drives.
//!
//! An embedded console and an on-demand PNG snapshot, with no frontend build step. `http://<robot>:8080/` and there is nothing else to run —
//! which avoids a separate web server.
//!
//! **It deletes four problems rather than one.** No `python3 -m http.server`, so the instruction is
//! an address rather than two commands. No URL to type, because a page served by the robot knows
//! which robot it came from and derives its signalling target from `location.hostname`. Chrome's
//! Private Network Access check stops applying, because that check is on requests from a *public or
//! opaque* origin to a private address — and a page served from `192.168.x` has a private one, so
//! the failure that made `file://` unusable cannot happen. And page and binary ship together, so a
//! client from a checkout can no longer be pointed at a robot from a release.
//!
//! ## The port and the API version are filled in here, not carried by the page
//!
//! [`page`] substitutes both into the embedded page once, at startup. That is what makes `--port`
//! safe to change: nothing holds a second copy of it. The *host* is the browser's to fill —
//! `location.hostname`, which is the robot that served the page — so this is one number and not a
//! URL.
//!
//! The API version is the same idea applied to the skew banner. A page cannot compare versions
//! without knowing which one it speaks, and a literal in the page would be the second copy of
//! [`proto::API_VERSION`] — wrong on the day it is bumped, and wrong in the direction that reports
//! agreement. Substituted, a page served by a robot is a page that speaks that robot's release, and
//! the banner it shows is about a *checkout* page pointed at a robot, which is the case it exists
//! for.
//!
//! A page opened straight from the source tree keeps the token, reads it as `NaN`, and falls back to
//! `webrtcsink`'s own 8443. It also then knows it was *not* served by a robot, which is what lets it
//! tell "wrong host" apart from "the robot served me but its signalling port did not answer" — the
//! one failure two ports can produce, and §1.3's requirement that it reach a person as a diagnosis.
//!
//! ## Portable, unlike [`crate::pipeline`]
//!
//! Nothing here touches GStreamer, so it compiles and its tests run on a laptop. The substitution is
//! the part worth a test: a page served with the token still in it would connect to the wrong port
//! and say nothing about why.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, Result};
use axum::Router;
use axum::response::Html;
use axum::routing::get;
use duck_ipc_proto as proto;

/// The page as it sits in the source tree.
///
/// `include_str!` rather than a file read at request time, and that is a decision: installing it
/// under `current/webclient/` would cost an `--include` line in three places that already drift
/// (`_build-release.yml`, `dev.yml`, `scripts/dev-push.sh`), put a filesystem read behind a network
/// request in a unit running `ProtectSystem=strict`, and make "which page is this robot serving" a
/// question with two answers. The cost is a rebuild to change a stylesheet, which is the right trade
/// for a page that is part of the daemon's interface.
const PAGE: &str = include_str!("../webclient/index.html");

/// Where the page carries its signalling port until this module fills one in.
const PORT_TOKEN: &str = "{{SIGNALLING_PORT}}";

/// Where it carries the API version this release speaks.
const API_TOKEN: &str = "{{API_VERSION}}";

/// The page, with the signalling port and the API version filled in.
pub fn page(signalling_port: u32) -> String {
    PAGE.replace(PORT_TOKEN, &signalling_port.to_string())
        .replace(API_TOKEN, &proto::API_VERSION.to_string())
}

/// Serve `page` on `host:port` until the process ends.
///
/// Returns only on failure — a bind that was refused, or a listener that died. The caller decides
/// what that costs; in `mediad` it costs the page and not the video, because a robot that streams
/// and answers control calls with no console is a great deal better than one that does neither.
pub async fn serve(host: &str, port: u16, page: String, frame_socket: PathBuf) -> Result<()> {
    let address: SocketAddr = format!("{host}:{port}")
        .parse()
        .with_context(|| format!("{host}:{port} is not an address to listen on"))?;
    let listener = tokio::net::TcpListener::bind(address)
        .await
        .with_context(|| format!("could not listen on {address}"))?;

    tracing::info!(%address, "serving the console");
    axum::serve(listener, router(page, frame_socket))
        .await
        .context("the console's listener stopped")
}

/// The console and its bounded, uncached snapshot endpoint.
fn router(page: String, frame_socket: PathBuf) -> Router {
    let slots = Arc::new(tokio::sync::Semaphore::new(4));
    Router::new()
        .route("/", get(move || std::future::ready(Html(page))))
        .route(
            "/frame",
            get(move || snapshot(frame_socket.clone(), slots.clone())),
        )
}

async fn snapshot(socket: PathBuf, slots: Arc<tokio::sync::Semaphore>) -> axum::response::Response {
    use axum::http::{StatusCode, header};
    use axum::response::IntoResponse;
    let result: Result<Vec<u8>> = async {
        let permit = slots
            .try_acquire_owned()
            .context("snapshot capacity reached")?;
        let (metadata, pixels) = crate::snapshot::fetch(&socket).await?;
        tokio::task::spawn_blocking(move || {
            // The permit belongs to the encoder, even if the HTTP client disconnects.
            let _permit = permit;
            crate::snapshot::png(metadata, pixels)
        })
        .await?
    }
    .await;
    let mut response = match result {
        Ok(png) => ([(header::CONTENT_TYPE, "image/png")], png).into_response(),
        Err(error) => {
            tracing::debug!(%error, "snapshot unavailable");
            (
                StatusCode::SERVICE_UNAVAILABLE,
                "Camera snapshot unavailable; retry when capture is running.\n",
            )
                .into_response()
        }
    };
    response
        .headers_mut()
        .insert(header::CACHE_CONTROL, "no-store".parse().unwrap());
    response
}
