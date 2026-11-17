//! What the camera's geometry is, so a consumer can do more than look at the picture.
//!
//! A frame is pixels. Turning pixels into directions — which is what SLAM, visual odometry, or
//! "how far away is that duck" all need — takes the **intrinsics**: the focal length in pixels and
//! where the optical axis crosses the image. Without them a monocular reconstruction is
//! scale-free and its angles are wrong; with them it maps a room.
//!
//! # Derived from the field of view, so it can be checked rather than trusted
//!
//! The IMX219's full array is 3280×2464 at a 1.12 µm pitch — 3.67 mm wide — behind the head
//! camera's ~3.05 mm M12 lens, a **horizontal field of view of ~62°** across the full width. On
//! this board's Rockchip driver the production **1920×1080@30** mode is a *scaled full-frame
//! readout* (the whole 62° width, downscaled and cropped to 16:9), **not** the native-pixel 39°
//! crop the datasheet's "1080p" implies. The family default in `robotd-params` carries this geometry
//! (`fx` ≈ 1062 at 1280×720). So the **field of view** — not a native focal length — is what the
//! nominal geometry rests on, because the field of view is what stays fixed as the ISP scales the
//! frame to whatever `[media] quality` asks for:
//!
//! - Focal length in pixels of a delivered frame `W` wide: `fx = (W/2) / tan(62°/2)`, so 1280 wide
//!   gives `fx` ≈ 1065; a uniform scale moves `fy`, `cx`, `cy` with it.
//! - The vertical is the 16:9 crop of that, and the principal point is *assumed* central — which is
//!   the part a real calibration corrects (the measured `cy` sits ~110 px low).
//!
//! # And when the pinned mode is not confirmed, this publishes nothing
//!
//! `pin_sensor_mode` shells out to `media-ctl` and can fail — a board without `v4l-utils`, an
//! entity name that moved. Capture still works from the 3280×2464 boot mode (also ~62°, at 21 fps),
//! but its exact 4:3→16:9 framing is not the pinned mode's, so this withholds nominal intrinsics
//! rather than publish a geometry it cannot vouch for. **Numbers that are quietly wrong are worse
//! than none**: a consumer told nothing knows it must calibrate.
//!
//! # Nominal is not calibrated, and the wire says which
//!
//! Everything above is the *design* of the camera, not a measurement of the one on this robot:
//! lens focal lengths vary by a few percent unit to unit, the principal point is never exactly the
//! centre, and nothing here models distortion at all. That is enough for a room-scale map and not
//! enough for photogrammetry, so a record built from them carries `calibrated: false`. In practice
//! every alpha robot publishes a measurement: `robotd-params` ships the family's solve as the
//! `[media.intrinsics]` default (the camera and lens are one part), and a robot with its own solve
//! written there publishes that. The nominal path is the fallback for a camera nobody has solved.

/// The head camera's horizontal field of view, degrees — the full IMX219 array (3.67 mm wide)
/// behind the ~3.05 mm M12 lens. The one physical number the nominal geometry rests on, and the one
/// the family calibration confirms (it solves to 62.2°). See the module header.
const FULL_FIELD_HFOV_DEG: f64 = 62.0;

/// The MuJoCo twin head camera's vertical field of view. The MJCF sets no `fovy`, so MuJoCo's
/// default 45° applies; see [`Intrinsics::sim`].
const SIM_VFOV_DEG: f64 = 45.0;

/// Horizontal focal length in pixels for a delivered frame `width` wide, from [`FULL_FIELD_HFOV_DEG`].
/// The delivered field of view is the sensor's full width in every mode this driver offers, so this
/// depends only on the output width, not on which sensor mode fed it.
fn nominal_focal_px(width: u32) -> f64 {
    f64::from(width) / 2.0 / (FULL_FIELD_HFOV_DEG / 2.0).to_radians().tan()
}

/// The sensor readout mode `pipeline::pin_sensor_mode` puts the IMX219 in — carried so
/// [`Intrinsics::nominal`] can tell "the mode we pinned" from "we could not confirm it", and refuse
/// a delivered frame whose aspect ratio is not the mode's.
///
/// # Both modes this driver offers are the full ~62° field
///
/// The IMX219's full array is 3280×2464 at a 1.12 µm pitch behind the ~3.05 mm M12 lens — `2·atan(
/// 3280·1.12µm / 2 / 3.05mm)` ≈ **62°** horizontally. On this board's Rockchip driver `1920×1080`
/// is a *scaled full-frame* readout, not a native crop, so it is the same 62° field (validated on
/// hardware; see the module header):
///
/// | mode | how | horizontal FOV | note |
/// |---|---|---|---|
/// | 1920×1080 | full field, scaled/cropped to 16:9 | **62°** | what this daemon pins, at 30 fps |
/// | 3280×2464 | full 4:3 array | **62°** | the boot mode, at 21 fps |
///
/// So the field of view does not change with the mode — only the resolution and frame rate do, and
/// the geometry a consumer needs is the same either way. `1920×1080` is pinned for the frame rate;
/// there is no wide-vs-fast trade to make here, because the wide field is already the fast mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorMode {
    pub width: u32,
    pub height: u32,
}

impl SensorMode {
    /// The mode `pipeline::pin_sensor_mode` asks for: the full field at 30 fps.
    pub const PINNED: Self = Self {
        width: 1920,
        height: 1080,
    };
}

/// Where the optical axis is and how long the focal length is, in pixels of a delivered frame.
///
/// **For the frame as it is sent**, which is not rotated: the camera is mounted a quarter turn off
/// and nothing on the robot turns the pixels back (`pipeline`'s header says why). A consumer that
/// rotates the image has to rotate these too — `cx` and `cy` swap, and so do `fx` and `fy` — and
/// the `rotate` field alongside these in `media.video` is what tells it by how much.
/// Where a published calibration came from — so a consumer can tell *this* robot's own solve from
/// the family's, which look identical in the numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Source {
    /// A `[media.intrinsics]` calibration measured on this robot.
    Robot,
    /// The hardware family's shared calibration (`robotd-params` ships it): a real solve of the
    /// same camera-and-lens part, not of this particular unit.
    Family,
    /// The module's design figures — arithmetic from the datasheet, not a measurement.
    Nominal,
    /// The MuJoCo twin's rendered camera — exact geometry from the simulator's field of view, not a
    /// physical measurement (there is no physical sensor). Lets twin recordings self-describe.
    Sim,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct Intrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// **Whether these came from a real solve.** `false` is `Source::Nominal` — the module's design
    /// figures, no distortion model, enough for a room-scale map rather than metrology. `true`
    /// covers both a per-robot and the family calibration; [`Intrinsics::source`] says which, so a
    /// consumer that needs *this* robot's own solve can tell it must still ask.
    pub calibrated: bool,
    /// Whose solve this is: this robot's, the family's, or none (the datasheet). See [`Source`].
    pub source: Source,
    /// Radial and tangential terms in OpenCV's order — `k1 k2 p1 p2 k3` — or empty for "no model
    /// of the distortion", which is what a nominal record has.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub distortion: Vec<f64>,
}

impl Intrinsics {
    /// The design figures for a delivered frame, or `None` when the geometry is not known.
    ///
    /// `None` has one cause and it is worth surfacing rather than papering over: the sensor is not
    /// in the mode this code pins, so how much of the sensor a frame covers is unknown.
    pub fn nominal(mode: Option<SensorMode>, width: u32, height: u32) -> Option<Self> {
        let mode = mode?;
        if width == 0 || height == 0 || mode.width == 0 || mode.height == 0 {
            return None;
        }

        // The delivered frame has to be a uniform scale of the mode — the same 16:9 aspect. The ISP
        // *can* scale the axes independently, and then part of the frame was cropped or squashed on
        // the way out, which the output size alone cannot tell apart from a resize.
        let scale_x = f64::from(width) / f64::from(mode.width);
        let scale_y = f64::from(height) / f64::from(mode.height);
        if (scale_x - scale_y).abs() > 0.01 {
            return None;
        }

        // The focal length is fixed by the field of view and the delivered width. The field of view
        // is the sensor's full ~62° in every mode, so it does not depend on which mode fed the frame.
        let focal = nominal_focal_px(width);
        Some(Self {
            fx: focal,
            fy: focal,
            // The principal point is *assumed* central, which is what makes this nominal: on a real
            // module it is a few pixels off (the measured cy sits ~110 px low), only a calibration
            // knows in which direction.
            cx: f64::from(width) / 2.0,
            cy: f64::from(height) / 2.0,
            calibrated: false,
            source: Source::Nominal,
            distortion: Vec::new(),
        })
    }

    /// A calibration, scaled from the resolution it was measured at to the one being delivered.
    ///
    /// Scaling a calibration is exact for a uniform resize — every intrinsic is in pixels and
    /// pixels all change size together — and wrong for a crop, which is why the record carries the
    /// resolution it was taken at rather than assuming one. An aspect change between the two means
    /// the second image is not the first one resized, so this refuses it.
    pub fn scaled_from(
        measured: &robotd_params::CameraIntrinsics,
        width: u32,
        height: u32,
    ) -> Option<Self> {
        Self::scaled(measured, width, height, Source::Robot)
    }

    /// A calibration scaled to the delivered frame, tagged with whose solve it is. `Robot` for a
    /// per-robot `[media.intrinsics]` table, `Family` for the shipped family default.
    fn scaled(
        measured: &robotd_params::CameraIntrinsics,
        width: u32,
        height: u32,
        source: Source,
    ) -> Option<Self> {
        if measured.width == 0 || measured.height == 0 || width == 0 || height == 0 {
            return None;
        }
        let scale_x = f64::from(width) / f64::from(measured.width);
        let scale_y = f64::from(height) / f64::from(measured.height);
        if (scale_x - scale_y).abs() > 0.01 {
            tracing::warn!(
                measured = format!("{}x{}", measured.width, measured.height),
                delivered = format!("{width}x{height}"),
                "the calibration was measured at a different aspect ratio than the stream, so it \
                 cannot be scaled to it; publishing nominal intrinsics instead"
            );
            return None;
        }
        Some(Self {
            fx: measured.fx * scale_x,
            fy: measured.fy * scale_y,
            cx: measured.cx * scale_x,
            cy: measured.cy * scale_y,
            calibrated: true,
            source,
            // Distortion coefficients are dimensionless in normalised image coordinates, so a
            // uniform resize leaves them alone.
            distortion: measured.distortion.clone(),
        })
    }

    /// The hardware family's calibration, scaled to the delivered frame. The camera and lens are
    /// one part across a revision, so this is a real solve of the same optics — just not of this
    /// particular unit, which is why it is published as `Source::Family` rather than `Robot`.
    ///
    /// Gated on a known sensor mode for the same reason [`Intrinsics::nominal`] is: the solve was
    /// taken in the pinned mode, and an unconfirmed one (the boot mode — the same ~62° field, but a
    /// different 4:3→16:9 framing and principal point) would place it slightly wrong — so an
    /// unconfirmed mode publishes nothing rather than the family's numbers off by that framing.
    pub fn family(mode: Option<SensorMode>, width: u32, height: u32) -> Option<Self> {
        mode?;
        Self::scaled(
            &robotd_params::CameraIntrinsics::alpha(),
            width,
            height,
            Source::Family,
        )
    }

    /// The MuJoCo twin's head camera. The MJCF sets no `fovy` on the camera, so MuJoCo's default
    /// **45° VERTICAL** field applies over the rendered height: `fx = fy = (height/2)/tan(45°/2)`,
    /// principal point central, no distortion (a rendered pinhole has none). Exact for the simulator,
    /// so twin recordings self-describe and need no `--calib`. Tagged [`Source::Sim`], `calibrated`
    /// false (it is not a measurement of a physical sensor). If a scene ever sets a custom camera
    /// `fovy`, update `SIM_VFOV_DEG`.
    pub fn sim(width: u32, height: u32) -> Option<Self> {
        if width == 0 || height == 0 {
            return None;
        }
        let focal = f64::from(height) / 2.0 / (SIM_VFOV_DEG / 2.0).to_radians().tan();
        Some(Self {
            fx: focal,
            fy: focal,
            cx: f64::from(width) / 2.0,
            cy: f64::from(height) / 2.0,
            calibrated: false,
            source: Source::Sim,
            distortion: Vec::new(),
        })
    }

    /// What to publish, in order of preference: this robot's own `[media.intrinsics]` calibration;
    /// else the hardware family's, which every unit shares; else the module's design figures. Each
    /// carries its [`Source`], so "calibrated" never has to stand in for "measured on *this* robot".
    pub fn published(
        configured: Option<&robotd_params::CameraIntrinsics>,
        mode: Option<SensorMode>,
        width: u32,
        height: u32,
    ) -> Option<Self> {
        configured
            .and_then(|measured| Self::scaled_from(measured, width, height))
            .or_else(|| Self::family(mode, width, height))
            .or_else(|| Self::nominal(mode, width, height))
    }
}
