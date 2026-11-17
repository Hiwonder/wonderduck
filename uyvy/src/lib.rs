//! What a camera frame is, before anything decides what it means.
//!
//! The head camera hands out `UYVY` at 1280×720 and is mounted a quarter turn off, so every
//! consumer of a frame does the same two things first: turn it the right way up, and sample it
//! down to the size it actually wants. Three of them do — `mediad` encoding a JPEG or a PNG,
//! `duck-detect` feeding a model, and `robotctl monitor` drawing one in a terminal — and this is
//! the one copy of that arithmetic, so a picture cannot come out upright in one and sideways in
//! the next.
//!
//! **The turn happens while sampling, not before it.** Both samplers walk their *output* and pull
//! the source pixel each one needs, so the rotation is a change of index inside a loop that was
//! already running rather than a pass of its own. [`Turn`] says why that matters: the pipeline
//! tried rotating in GStreamer once and it cost the board 22 fps.

/// The grey ultralytics pads a letterbox with, and therefore what calibration and training saw.
pub const PAD: u8 = 114;

/// How a frame was fitted into the model's square, so detections can be mapped back out of it.
#[derive(Debug, Clone, Copy)]
pub struct Letterbox {
    pub scale: f32,
    pub pad_x: f32,
    pub pad_y: f32,
}

/// Fitted dimensions and coordinate transform for a square model input.
pub fn letterbox_layout(
    width: usize,
    height: usize,
    size: usize,
    turn: Turn,
) -> (Letterbox, usize, usize) {
    let (upright_w, upright_h) = turn.upright(width, height);
    let scale = (size as f32 / upright_w as f32).min(size as f32 / upright_h as f32);
    let fitted_w = ((upright_w as f32 * scale).round() as usize)
        .max(1)
        .min(size);
    let fitted_h = ((upright_h as f32 * scale).round() as usize)
        .max(1)
        .min(size);
    let fit = Letterbox {
        scale,
        pad_x: ((size - fitted_w) / 2) as f32,
        pad_y: ((size - fitted_h) / 2) as f32,
    };
    (fit, fitted_w, fitted_h)
}

/// How the camera is mounted, and therefore how far the sampler has to turn the picture.
///
/// **Turned here rather than in the pipeline, and that is a performance decision, not a taste.**
/// A `videoflip` before the tee cost 145% of a core on the robot: `mpph264enc` hands UYVY→NV12 to
/// the SoC's 2D engine for free, and the flip's buffers are ones the RGA refuses
/// (`RGA_BLIT fail: Bad address`), so MPP fell back to converting every frame in software — 97 °C,
/// the CPU throttled to 408 MHz, and 8 fps out of a 30 fps camera. This sampler is already
/// resampling to 320×320, so doing the turn in the same pass costs nothing at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Turn {
    #[default]
    None,
    /// A quarter turn clockwise: what this robot's camera mount needs.
    Right,
    Half,
    Left,
}

impl Turn {
    /// From degrees clockwise, which is how the flag is written.
    pub fn from_degrees(degrees: u32) -> Option<Self> {
        match degrees % 360 {
            0 => Some(Self::None),
            90 => Some(Self::Right),
            180 => Some(Self::Half),
            270 => Some(Self::Left),
            _ => None,
        }
    }

    /// The frame's size once turned — a quarter turn swaps the axes.
    pub fn upright(self, width: usize, height: usize) -> (usize, usize) {
        match self {
            Self::Right | Self::Left => (height, width),
            Self::None | Self::Half => (width, height),
        }
    }

    /// Where a pixel of the *upright* picture is in the frame the camera took.
    ///
    /// The inverse mapping, because the sampler walks the output and pulls from the input. Written
    /// as one function so the four cases are in one place rather than spread through a loop.
    fn source(self, ux: usize, uy: usize, width: usize, height: usize) -> (usize, usize) {
        match self {
            Self::None => (ux, uy),
            // A quarter turn clockwise sends source (x, y) to upright (h-1-y, x), so the inverse
            // takes upright (ux, uy) from source (uy, h-1-ux).
            Self::Right => (uy, height.saturating_sub(1).saturating_sub(ux)),
            Self::Half => (
                width.saturating_sub(1).saturating_sub(ux),
                height.saturating_sub(1).saturating_sub(uy),
            ),
            Self::Left => (width.saturating_sub(1).saturating_sub(uy), ux),
        }
    }
}

/// `UYVY` straight into a letterboxed RGB square — one pass, and only the pixels that survive.
///
/// **This replaced converting the frame and then shrinking it, which cost 345 ms of the 407 ms a
/// look took on the robot.** The tee carries 720×1280 4:2:2 because that is what the encoder wants;
/// the model wants 320×320 RGB. Converting all 921 600 pixels to throw 89% of them away is nine
/// times the arithmetic for the same answer, so this samples the source at the target grid instead —
/// 102 400 pixels, integer maths, no intermediate buffer.
///
/// Nearest-neighbour, and chroma from the pair without interpolation: the input is a blurred
/// photograph of a room being downscaled by four, and nothing in a bounding box survives at that
/// precision.
pub fn letterbox_from_uyvy(
    uyvy: &[u8],
    width: usize,
    height: usize,
    size: usize,
    turn: Turn,
    out: &mut Vec<u8>,
) -> Letterbox {
    letterbox_from_uyvy_with_pad(uyvy, width, height, size, turn, PAD, out)
}

/// Convert and letterbox with an explicit padding value.
pub fn letterbox_from_uyvy_with_pad(
    uyvy: &[u8],
    width: usize,
    height: usize,
    size: usize,
    turn: Turn,
    pad: u8,
    out: &mut Vec<u8>,
) -> Letterbox {
    // Everything below is in *upright* coordinates — the picture the right way up, which is what
    // the model was trained on and what a detection has to be reported in. The turn is undone only
    // at the moment a source pixel is fetched.
    let (upright_w, upright_h) = turn.upright(width, height);
    let (fit, fitted_w, fitted_h) = letterbox_layout(width, height, size, turn);
    let pad_x = fit.pad_x as usize;
    let pad_y = fit.pad_y as usize;
    let stride = width * 2;

    out.clear();
    out.resize(size * size * 3, pad);
    for y in 0..fitted_h {
        let uy = (y * upright_h) / fitted_h;
        for x in 0..fitted_w {
            let ux = (x * upright_w) / fitted_w;
            let (source_x, source_y) = turn.source(ux, uy, width, height);
            let row = source_y * stride;
            if row + stride > uyvy.len() {
                // A frame that arrives mid-teardown is short. What is missing stays padding rather
                // than taking the daemon down over a picture.
                continue;
            }
            let pair = row + (source_x / 2) * 4;
            // U Y0 V Y1: the luma is the odd byte of the half this pixel falls in.
            let luma = uyvy[pair + 1 + 2 * (source_x & 1)] as i32 - 16;
            let u = uyvy[pair] as i32 - 128;
            let v = uyvy[pair + 2] as i32 - 128;

            // BT.601 limited range in fixed point — the ISP's convention, and the one every JPEG
            // the dataset was labelled from went through. Integer because this is the inner loop.
            let r = (298 * luma + 409 * v + 128) >> 8;
            let g = (298 * luma - 100 * u - 208 * v + 128) >> 8;
            let b = (298 * luma + 516 * u + 128) >> 8;

            let target = ((y + pad_y) * size + (x + pad_x)) * 3;
            out[target] = r.clamp(0, 255) as u8;
            out[target + 1] = g.clamp(0, 255) as u8;
            out[target + 2] = b.clamp(0, 255) as u8;
        }
    }

    fit
}

/// The frame as an upright RGB picture, scaled to fit a box, with no padding.
///
/// [`letterbox_from_uyvy`] above is for the model: a square, padded, at whatever size the network
/// wants. This is for a *person or a program looking at the picture* — a JPEG on its way to a
/// Space that runs a model of its own — so it keeps the aspect ratio and pads nothing.
///
/// **The turn is applied here rather than reported.** `mediad`'s pipeline deliberately does not
/// rotate: a `videoflip` cost the encoder its zero-copy path and the board 22 fps, so a WebRTC
/// consumer is told the mount angle and turns the picture itself (`media.video`). That reasoning
/// does not carry over to this path, because the conversion is a per-pixel loop either way and the
/// turn is a change of which source pixel is fetched — free, inside a loop that is already
/// running. And what receives these frames is a model, which wants them the way up it was trained
/// on rather than a rotation flag to honour.
///
/// Returns the size written, which is not `(long, short)` in any predictable order: a quarter turn
/// swaps the axes, so the caller is told rather than left to work it out.
pub fn rgb_from_uyvy(
    uyvy: &[u8],
    width: usize,
    height: usize,
    longest: usize,
    turn: Turn,
    out: &mut Vec<u8>,
) -> (usize, usize) {
    let (upright_w, upright_h) = turn.upright(width, height);
    // Downscale only. Asking for a box bigger than the sensor would interpolate detail that was
    // never captured and cost the bandwidth of pretending.
    let scale = (longest as f32 / upright_w.max(upright_h) as f32).min(1.0);
    let out_w = ((upright_w as f32 * scale).round() as usize).max(1);
    let out_h = ((upright_h as f32 * scale).round() as usize).max(1);
    let stride = width * 2;

    out.clear();
    out.resize(out_w * out_h * 3, 0);
    for y in 0..out_h {
        let uy = (y * upright_h) / out_h;
        for x in 0..out_w {
            let ux = (x * upright_w) / out_w;
            let (source_x, source_y) = turn.source(ux, uy, width, height);
            let row = source_y * stride;
            if row + stride > uyvy.len() {
                // A frame that arrives mid-teardown is short. What is missing stays black rather
                // than taking the daemon down over a picture — the same call `letterbox_from_uyvy`
                // makes.
                continue;
            }
            let pair = row + (source_x / 2) * 4;
            // U Y0 V Y1: the luma is the odd byte of the half this pixel falls in.
            let luma = uyvy[pair + 1 + 2 * (source_x & 1)] as i32 - 16;
            let u = uyvy[pair] as i32 - 128;
            let v = uyvy[pair + 2] as i32 - 128;

            // BT.601 limited range in fixed point, as above: the ISP's convention.
            let r = (298 * luma + 409 * v + 128) >> 8;
            let g = (298 * luma - 100 * u - 208 * v + 128) >> 8;
            let b = (298 * luma + 516 * u + 128) >> 8;

            let target = (y * out_w + x) * 3;
            out[target] = r.clamp(0, 255) as u8;
            out[target + 1] = g.clamp(0, 255) as u8;
            out[target + 2] = b.clamp(0, 255) as u8;
        }
    }
    (out_w, out_h)
}
