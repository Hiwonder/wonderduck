//! Finding other WonderDucks in this robot's camera, on the NPU.
//!
//! The model is trained in [`duck_detector`](https://github.com/pollen-robotics/duck_detector) and
//! arrives here as an INT8 `.rknn`: one class, 320×320 in, 2100 candidate boxes out. This crate is
//! the three things between a camera frame and a bounding box — the letterbox, the runtime, and the
//! decode.
//!
//! The sampler that turns a `UYVY` frame the right way up and shrinks it is not here: it is in
//! [`uyvy`], because `mediad` and `robotctl monitor` need the same arithmetic and must not link a
//! model runtime to get it.
//!
//! **Everything here has to agree with how the model was trained**, and nothing enforces that
//! across the two repositories except this comment and the numbers below:
//!
//! * frames are letterboxed, not stretched, into a square, padded with 114 grey;
//! * RGB, not BGR;
//! * the picture is the one `mediad` has already turned the right way up (`--rotate`, default 90°),
//!   because that is what the dataset was captured through.
//!
//! Get one of those wrong and the detector does not fail — it just gets quietly worse, which is the
//! failure mode this crate is most exposed to.

pub mod onnx;
pub mod rknn;
pub mod rga;
pub mod objects;

// Its own signatures speak in these: `letterbox_rgb` returns a [`Letterbox`] and `decode` takes
// one back. The samplers themselves are not re-exported — a consumer that wants pixels wants
// [`uyvy`], not a second name for it here.
pub use uyvy::{Letterbox, PAD};

/// What the head emits per candidate: cx, cy, w, h, score.
const STRIDE: usize = 5;

/// One detection, in the coordinates of the frame that went in — not of the letterbox.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detection {
    pub score: f32,
    /// Pixels, `x0 y0 x1 y1`, in the original frame.
    pub box_: [f32; 4],
}

impl Detection {
    pub fn width(&self) -> f32 {
        self.box_[2] - self.box_[0]
    }

    pub fn height(&self) -> f32 {
        self.box_[3] - self.box_[1]
    }

    /// Where the duck is, across the frame: -1 hard left, 0 straight ahead, 1 hard right.
    ///
    /// The one number a behaviour actually wants — "turn towards it" needs a bearing, not a box.
    pub fn bearing(&self, frame_width: f32) -> f32 {
        let centre = (self.box_[0] + self.box_[2]) / 2.0;
        (centre / frame_width) * 2.0 - 1.0
    }
}

/// Scale to fit and pad to square, writing NHWC RGB bytes into `out`.
///
/// Nearest-neighbour on purpose: this runs per frame beside a 50 Hz control loop, the input is a
/// blurred 720×1280 photograph of a room, and a bilinear resize costs three times as much to move a
/// box by a pixel. If a measurement ever says the accuracy is worth it, the RGA can do it for free.
pub fn letterbox_rgb(
    frame: &[u8],
    width: usize,
    height: usize,
    size: usize,
    out: &mut Vec<u8>,
) -> Letterbox {
    let scale = (size as f32 / width as f32).min(size as f32 / height as f32);
    let fitted_w = ((width as f32 * scale).round() as usize).max(1).min(size);
    let fitted_h = ((height as f32 * scale).round() as usize).max(1).min(size);
    let pad_x = (size - fitted_w) / 2;
    let pad_y = (size - fitted_h) / 2;

    out.clear();
    out.resize(size * size * 3, PAD);
    for y in 0..fitted_h {
        // Nearest source row/column, computed from the fitted size so rounding cannot walk off the
        // end of the source on the last row.
        let source_y = (y * height) / fitted_h.max(1);
        for x in 0..fitted_w {
            let source_x = (x * width) / fitted_w.max(1);
            let source = (source_y * width + source_x) * 3;
            let target = ((y + pad_y) * size + (x + pad_x)) * 3;
            out[target..target + 3].copy_from_slice(&frame[source..source + 3]);
        }
    }

    Letterbox {
        scale,
        pad_x: pad_x as f32,
        pad_y: pad_y as f32,
    }
}

/// Candidates over `threshold`, suppressed, and mapped back to the original frame.
///
/// **The head does not suppress anything.** 2100 candidates means one duck comes back as twenty
/// overlapping boxes, and every consumer of this crate would otherwise have to know that. The
/// threshold is a property of *this* model, quantised: an INT8 output tensor carries its own scale,
/// so a value that means 0.9 on the float model is not 0.9 here. Tune it against the board.
pub fn decode(raw: &[f32], letterbox: Letterbox, threshold: f32, iou_limit: f32) -> Vec<Detection> {
    let candidates = raw.len() / STRIDE;
    let mut found: Vec<Detection> = Vec::new();

    // The tensor is `[1, 5, N]`: all the cx values, then all the cy values, and so on — not five
    // numbers per box. Reading it as interleaved gives boxes that are almost plausible, which is
    // the worst kind of wrong.
    for index in 0..candidates {
        let score = raw[4 * candidates + index];
        if score < threshold {
            continue;
        }
        let cx = raw[index];
        let cy = raw[candidates + index];
        let w = raw[2 * candidates + index];
        let h = raw[3 * candidates + index];
        // Out of the letterbox, back into the frame the camera took.
        let unpad = |value: f32, pad: f32| (value - pad) / letterbox.scale;
        found.push(Detection {
            score,
            box_: [
                unpad(cx - w / 2.0, letterbox.pad_x),
                unpad(cy - h / 2.0, letterbox.pad_y),
                unpad(cx + w / 2.0, letterbox.pad_x),
                unpad(cy + h / 2.0, letterbox.pad_y),
            ],
        });
    }

    found.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut kept: Vec<Detection> = Vec::new();
    for detection in found {
        if kept
            .iter()
            .all(|other| iou(&detection.box_, &other.box_) < iou_limit)
        {
            kept.push(detection);
        }
    }
    kept
}

fn iou(a: &[f32; 4], b: &[f32; 4]) -> f32 {
    let x0 = a[0].max(b[0]);
    let y0 = a[1].max(b[1]);
    let x1 = a[2].min(b[2]);
    let y1 = a[3].min(b[3]);
    let overlap = (x1 - x0).max(0.0) * (y1 - y0).max(0.0);
    let area = |r: &[f32; 4]| (r[2] - r[0]).max(0.0) * (r[3] - r[1]).max(0.0);
    let union = area(a) + area(b) - overlap;
    if union <= 0.0 { 0.0 } else { overlap / union }
}
