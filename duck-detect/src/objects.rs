//! Decode class scores and corner coordinates from an object detection model.

use anyhow::{Result, ensure};
use uyvy::Letterbox;

const CLASSES: usize = 80;
const CANDIDATES: usize = 8400;
const MAX_DETECTIONS: usize = 300;

#[derive(Debug, Clone, PartialEq)]
pub struct Object {
    pub class: usize,
    pub score: f32,
    pub bounds: [f32; 4],
}

pub fn decode(
    scores: &[f32],
    boxes: &[f32],
    fit: Letterbox,
    width: usize,
    height: usize,
    threshold: f32,
) -> Result<Vec<Object>> {
    ensure!(
        scores.len() == CLASSES * CANDIDATES,
        "invalid class score tensor"
    );
    ensure!(boxes.len() == 4 * CANDIDATES, "invalid box tensor");
    ensure!(
        fit.scale.is_finite() && fit.scale > 0.0 && width > 0 && height > 0,
        "invalid frame geometry"
    );
    // Scores are class-major: contiguous reads avoid a strided 80-class reduction
    // for every anchor. These arrays use 42 KB on the stack, with no per-class allocation.
    let mut best_scores = [f32::NEG_INFINITY; CANDIDATES];
    let mut best_classes = [0u8; CANDIDATES];
    for (class, class_scores) in scores.chunks_exact(CANDIDATES).enumerate() {
        for ((best_score, best_class), &score) in best_scores
            .iter_mut()
            .zip(best_classes.iter_mut())
            .zip(class_scores)
        {
            // Strict comparison preserves the lowest class on ties, including signed zero.
            let replace = score.is_finite() && score > *best_score;
            *best_score = if replace { score } else { *best_score };
            *best_class = if replace { class as u8 } else { *best_class };
        }
    }
    let mut found = Vec::new();
    for index in 0..CANDIDATES {
        if best_scores[index] < threshold {
            continue;
        }
        let mut bounds = [0.0; 4];
        for (axis, bound) in bounds.iter_mut().enumerate() {
            let value = boxes[axis * CANDIDATES + index];
            let (pad, limit) = if axis % 2 == 0 {
                (fit.pad_x, width)
            } else {
                (fit.pad_y, height)
            };
            *bound = ((value - pad) / fit.scale).clamp(0.0, limit as f32);
        }
        if bounds.iter().any(|v| !v.is_finite()) || bounds[2] <= bounds[0] || bounds[3] <= bounds[1]
        {
            continue;
        }
        found.push(Object {
            class: best_classes[index] as usize,
            score: best_scores[index],
            bounds,
        });
    }
    found.sort_unstable_by(|a, b| b.score.total_cmp(&a.score));
    found.truncate(MAX_DETECTIONS);
    Ok(found)
}
