//! The pad's IMU, drawn: a wireframe pad posed like the real one.
//!
//! The attitude comes from the `pad-imu` crate — the same filter `padd` poses the head with — and
//! this module only draws it. A dozen line segments rasterised into half-block pixels, redrawn at
//! the monitor's own pace rather than the IMU's: six hundred samples a second is a rate for a
//! filter, not for a terminal. The wireframe puts a marker on the front edge so a wrong guess
//! about the body frame is visible the first time the pad tilts.

use pad_imu::Imu;
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::style::Color;

/// Camera yaw, radians: a three-quarter view, so both the front edge and a grip are visible.
const CAMERA_AZIMUTH: f32 = -0.55;
/// Camera elevation above the pad's plane, radians.
const CAMERA_ELEVATION: f32 = 0.55;

/// Draw the pad's attitude into `area`, two pixels per cell.
///
/// A wireframe of a pad — a flat slab, two grips toward the player, the two sticks, a marker on
/// the front edge — posed by the orientation and seen from a fixed three-quarter camera. Cheap on
/// purpose: a dozen segments, no depth sorting. The eye resolves a wireframe pad without it.
pub fn draw(imu: &Imu, area: Rect, buf: &mut Buffer) {
    let (w, h) = (usize::from(area.width), usize::from(area.height) * 2);
    if w < 6 || h < 6 {
        return;
    }
    let mut canvas = Canvas::new(w, h);
    let rotation = pad_imu::matrix(imu.quaternion().unwrap_or([1.0, 0.0, 0.0, 0.0]));

    // World → screen: yaw the camera for a three-quarter view, then look down at it. The result
    // is in body millimetres until it is scaled below.
    let (az_sin, az_cos) = CAMERA_AZIMUTH.sin_cos();
    let (el_sin, el_cos) = CAMERA_ELEVATION.sin_cos();
    let project = |p: [f32; 3]| -> (f32, f32) {
        let world = rotate_matrix(rotation, p);
        let x = world[0] * az_cos - world[1] * az_sin;
        let y = world[0] * az_sin + world[1] * az_cos;
        let up = world[2] * el_cos + y * el_sin;
        (x, -up)
    };

    // One zoom for every attitude: the pad's bounding sphere is what has to fit, so a pad on its
    // edge and a pad lying flat are drawn at the same scale and tilting reads as tilting, not as
    // the picture breathing. The body origin sits at the canvas centre. Framed at 80% of the
    // sphere rather than all of it: only a grip tip pointing straight at the camera's edge ever
    // reaches the last 20%, and framing for that moment shrinks every other one.
    let radius = PAD_WIREFRAME
        .iter()
        .flat_map(|s| [s.from, s.to])
        .map(pad_imu::norm)
        .fold(1.0f32, f32::max)
        * 0.8;
    let scale = ((w as f32 - 2.0) / (2.0 * radius)).min((h as f32 - 2.0) / (2.0 * radius));
    let (cx, cy) = (w as f32 / 2.0, h as f32 / 2.0);
    let place = |p: (f32, f32)| (p.0 * scale + cx, p.1 * scale + cy);

    for segment in PAD_WIREFRAME {
        canvas.line(
            place(project(segment.from)),
            place(project(segment.to)),
            segment.colour(),
        );
    }
    canvas.blit(area, buf);
}

/// One segment of the drawn pad, in the body frame: millimetres, +X front, +Y left, +Z up.
struct Segment {
    from: [f32; 3],
    to: [f32; 3],
    part: Part,
}

#[derive(Clone, Copy)]
enum Part {
    Body,
    Grip,
    Stick,
    Front,
}

impl Segment {
    const fn new(from: [f32; 3], to: [f32; 3], part: Part) -> Self {
        Self { from, to, part }
    }

    fn colour(&self) -> Color {
        match self.part {
            Part::Body => Color::Cyan,
            Part::Grip => Color::Rgb(0, 140, 160),
            Part::Stick => Color::White,
            Part::Front => Color::Yellow,
        }
    }
}

/// The pad, in as few lines as still read as a pad: the top face of a slab 50 mm deep by 150
/// wide, its front edge given thickness, two grips reaching back toward the player, the two
/// sticks standing proud, and a bar along the front edge so that "which way is it facing" never
/// has to be inferred from the grips alone. Every line dropped from a fuller model — the bottom
/// face, the grips' far edges — was one that turned the picture into hatching at terminal
/// resolution without adding a degree of freedom the eye could read.
const PAD_WIREFRAME: &[Segment] = &{
    const B: Part = Part::Body;
    const G: Part = Part::Grip;
    const S: Part = Part::Stick;
    const F: Part = Part::Front;
    [
        // Top face.
        Segment::new([25.0, 75.0, 10.0], [25.0, -75.0, 10.0], B),
        Segment::new([25.0, -75.0, 10.0], [-25.0, -75.0, 10.0], B),
        Segment::new([-25.0, -75.0, 10.0], [-25.0, 75.0, 10.0], B),
        Segment::new([-25.0, 75.0, 10.0], [25.0, 75.0, 10.0], B),
        // Thickness, on the front edge only.
        Segment::new([25.0, 75.0, 10.0], [25.0, 75.0, -10.0], B),
        Segment::new([25.0, -75.0, 10.0], [25.0, -75.0, -10.0], B),
        Segment::new([25.0, 75.0, -10.0], [25.0, -75.0, -10.0], B),
        // Grips: the outer edge of each, back and down from the rear corners, closed at the end.
        Segment::new([-25.0, 70.0, 10.0], [-70.0, 55.0, -20.0], G),
        Segment::new([-25.0, 40.0, 10.0], [-70.0, 35.0, -20.0], G),
        Segment::new([-70.0, 55.0, -20.0], [-70.0, 35.0, -20.0], G),
        Segment::new([-25.0, -70.0, 10.0], [-70.0, -55.0, -20.0], G),
        Segment::new([-25.0, -40.0, 10.0], [-70.0, -35.0, -20.0], G),
        Segment::new([-70.0, -55.0, -20.0], [-70.0, -35.0, -20.0], G),
        // Sticks: the Pro Controller's left stick sits forward-left, the right one back-right.
        Segment::new([8.0, 45.0, 10.0], [8.0, 45.0, 26.0], S),
        Segment::new([-10.0, -30.0, 10.0], [-10.0, -30.0, 26.0], S),
        // Front bar, along the top of the front edge, with a nose at its middle.
        Segment::new([25.0, 60.0, 14.0], [25.0, -60.0, 14.0], F),
        Segment::new([25.0, 0.0, 14.0], [45.0, 0.0, 14.0], F),
    ]
};

/// A half-block pixel canvas: one colour per pixel, two pixels per terminal row.
struct Canvas {
    w: usize,
    h: usize,
    pixels: Vec<Option<Color>>,
}

impl Canvas {
    fn new(w: usize, h: usize) -> Self {
        Self {
            w,
            h,
            pixels: vec![None; w * h],
        }
    }

    fn set(&mut self, x: i32, y: i32, colour: Color) {
        if x < 0 || y < 0 {
            return;
        }
        let (x, y) = (x as usize, y as usize);
        if x < self.w && y < self.h {
            self.pixels[y * self.w + x] = Some(colour);
        }
    }

    /// Bresenham, on rounded endpoints. Clipped per pixel: a segment that leaves the canvas is
    /// drawn as far as it goes rather than dropped.
    fn line(&mut self, a: (f32, f32), b: (f32, f32), colour: Color) {
        let (mut x0, mut y0) = (a.0.round() as i32, a.1.round() as i32);
        let (x1, y1) = (b.0.round() as i32, b.1.round() as i32);
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        loop {
            self.set(x0, y0, colour);
            if x0 == x1 && y0 == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x0 += sx;
            }
            if e2 <= dx {
                err += dx;
                y0 += sy;
            }
        }
    }

    fn blit(&self, area: Rect, buf: &mut Buffer) {
        for row in 0..usize::from(area.height) {
            for col in 0..self.w {
                let top = self.pixels[(row * 2) * self.w + col];
                let bottom = self
                    .pixels
                    .get((row * 2 + 1) * self.w + col)
                    .copied()
                    .flatten();
                let Some(cell) = buf.cell_mut((area.x + col as u16, area.y + row as u16)) else {
                    continue;
                };
                match (top, bottom) {
                    (Some(t), Some(b)) => {
                        cell.set_symbol("▀").set_fg(t).set_bg(b);
                    }
                    (Some(t), None) => {
                        cell.set_symbol("▀").set_fg(t);
                    }
                    (None, Some(b)) => {
                        cell.set_symbol("▄").set_fg(b);
                    }
                    (None, None) => {}
                }
            }
        }
    }
}

fn rotate_matrix(m: [[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}
