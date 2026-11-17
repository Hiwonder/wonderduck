//! The monitor's camera block: what the robot is looking at, in half-block pixels.
//!
//! Every other block on the frame is a number about the robot. This one is the picture the robot's
//! own perception runs on, which is the only way to answer the questions a number cannot: whether
//! the head is pointing where the joints say it is, whether the lens is smeared, whether the room
//! is far too dark for the detector to find anything in it.
//!
//! **It asks `mediad` for a frame rather than subscribing to one**, because `media.frame` is a
//! rendezvous: the capture branch copies a 1.84 MiB buffer only when a reader has asked for one,
//! and drops every other frame unread (`mediad/src/pipeline.rs` explains what that saves). So this
//! asks twice a second while the block is open, and — the part that matters — not at all while it
//! is closed. A monitor left running all day with the block shut costs the camera nothing.
//!
//! **The frame is kept as it arrived and decoded at render time.** The picture is a few hundred
//! pixels; converting 921 600 of them to fill it would be most of the work for none of the result,
//! so the sampler is pointed straight at the block's own size and the terminal's, and the raw
//! bytes stay put so a resize redraws from the frame rather than waiting for the next one.
//!
//! The mount turn **is** applied here, unlike on the raw `media.frame` path where the header names
//! the angle and the recorder turns it. A person looking at a terminal has nowhere to put a
//! rotation flag — the same reasoning `mediad`'s PNG route gives for applying it there.

use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::style::{Color, Modifier, Style, Stylize};
use ratatui::text::{Line, Span};

/// Past this, the picture on screen is not what the camera is looking at any more. Generous
/// against the half-second request period: a board under load answers late, and a block that
/// called itself stale on every hiccup would teach the reader to ignore the word.
const STALE: Duration = Duration::from_secs(3);

/// One frame, as it came off the socket.
pub(crate) struct Shot {
    pub(crate) header: proto::MediaFrameHeader,
    /// `UYVY`, exactly the bytes `mediad` sent. Checked against the header's geometry before it
    /// got here — see `frame::read_frame` — so the sampler cannot be handed a short buffer.
    pub(crate) data: Vec<u8>,
    /// How long `mediad` took to answer. This is the camera's own liveness: the endpoint parks
    /// until the *next* capture lands, so a healthy 30 fps camera answers in a frame period and a
    /// stopped one takes the full capture timeout and then fails.
    pub(crate) waited: Duration,
}

/// The camera block's state: the last frame, when it arrived, and why there is none.
#[derive(Default)]
pub(crate) struct CameraView {
    shot: Option<Shot>,
    /// By this view's clock, not the frame's: `captured_at_unix_us` is the robot's, and the
    /// question here is how old what is on screen is.
    arrived: Option<Instant>,
    /// Why there is no picture, when there is none.
    lost: Option<String>,
}

impl CameraView {
    pub(crate) fn absorb(&mut self, shot: Shot) {
        self.shot = Some(shot);
        self.arrived = Some(Instant::now());
        self.lost = None;
    }

    pub(crate) fn lost(&mut self, why: String) {
        self.lost = Some(why);
    }

    /// Drop the picture when the block closes.
    ///
    /// Nothing is being fetched while it is shut, so whatever is held would be however many
    /// minutes old when the block is opened again — and a stale picture of a room is
    /// indistinguishable from a live one. Reopening waits half a second for a real frame instead.
    pub(crate) fn forget(&mut self) {
        self.shot = None;
        self.arrived = None;
        self.lost = None;
    }

    /// Is there a frame to draw? The block shows a sentence instead when there is not.
    pub(crate) fn has_picture(&self) -> bool {
        self.shot.is_some()
    }

    /// `camera 1280×720 · mount 90° · answered in 38 ms`
    pub(crate) fn title(&self) -> Vec<Span<'static>> {
        let mut title = vec![Span::raw(" camera ")];
        let Some(shot) = self.shot.as_ref() else {
            title.push(Span::raw("waiting… ").dim());
            return title;
        };
        title.push(Span::styled(
            format!("{}×{}", shot.header.width, shot.header.height),
            Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
        ));
        // The mount, because the picture is drawn upright and there is otherwise nothing on the
        // frame to say the pixels were not. A camera bolted on straight says 0°.
        title.push(Span::raw(format!(" · mount {}°", shot.header.rotate)));
        title.push(Span::raw(format!(
            " · answered in {} ms ",
            shot.waited.as_millis()
        )));
        title
    }

    /// How old the picture is, which is the one thing a still image cannot say about itself.
    pub(crate) fn caption(&self) -> Line<'static> {
        let Some(age) = self.arrived.map(|at| at.elapsed()) else {
            return Line::from(String::new());
        };
        let text = format!(" {:.1} s ago ", age.as_secs_f64());
        if age > STALE {
            Line::from(text).fg(Color::Yellow)
        } else {
            Line::from(text).dim()
        }
    }

    /// What to say when there is no picture. Three cases, because they need different fixes:
    /// `mediad` is not running, `mediad` is running and the camera is not producing, or the first
    /// request is simply still in flight.
    pub(crate) fn absence(&self) -> String {
        match self.lost.as_deref() {
            Some(why) => format!("no picture: {why}"),
            None => "asking mediad for a frame…".to_owned(),
        }
    }

    /// Draw the frame into `area`, centred, as large as the block allows.
    ///
    /// Half-block cells hold two pixels each, so the pixel grid is as wide as the area and twice
    /// as tall. After the quarter turn this robot's camera needs, the picture is portrait —
    /// 720×1280 — so it is height-bound and leaves room either side, which is why it is centred
    /// rather than pinned to the left edge.
    pub(crate) fn draw(&self, area: Rect, buf: &mut Buffer) {
        let Some(shot) = self.shot.as_ref() else {
            return;
        };
        let (box_w, box_h) = (usize::from(area.width), usize::from(area.height) * 2);
        if box_w == 0 || box_h == 0 {
            return;
        }
        // A mount that is not a quarter turn cannot reach here: the header is rejected on the way
        // in by `valid_uyvy`, which is also what guarantees the geometry below matches the bytes.
        let turn = uyvy::Turn::from_degrees(shot.header.rotate).unwrap_or_default();
        let (width, height) = (shot.header.width as usize, shot.header.height as usize);
        let (upright_w, upright_h) = turn.upright(width, height);
        // `rgb_from_uyvy` fits a box by its longest edge, so the longest edge is what this has to
        // work out: the scale that fits *both* ways, applied to whichever edge is longer.
        let scale = (box_w as f32 / upright_w as f32).min(box_h as f32 / upright_h as f32);
        let longest = ((upright_w.max(upright_h) as f32 * scale).round() as usize).max(1);
        // Decoded per redraw rather than cached: the picture is a few hundred pixels of integer
        // arithmetic, and a cache would have to be invalidated on every resize of the block.
        let mut rgb = Vec::new();
        let (picture_w, picture_h) =
            uyvy::rgb_from_uyvy(&shot.data, width, height, longest, turn, &mut rgb);

        let left = box_w.saturating_sub(picture_w) / 2;
        let top = box_h.saturating_sub(picture_h) / 2;
        let rgb = &rgb;
        crate::cells::blit(area, buf, |x, y| {
            let (x, y) = (x.checked_sub(left)?, y.checked_sub(top)?);
            if x >= picture_w || y >= picture_h {
                return None;
            }
            let i = (y * picture_w + x) * 3;
            Some([*rgb.get(i)?, *rgb.get(i + 1)?, *rgb.get(i + 2)?])
        });
    }
}
