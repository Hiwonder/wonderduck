//! Camera rays and head-joint kinematics in the trunk frame.
//!
//! The caller supplies both the current robot's MJCF and the rotation from the
//! upright optical frame (+x right, +y down, +z forward) into `head_roll`.
//! Rays describe directions only: camera translation and target distance do not
//! participate in projection or its Jacobian.

use crate::{Model, ParseError, Pose, Quat};
use thiserror::Error;

const HEAD_JOINTS: [&str; 4] = ["neck_pitch", "head_pitch", "head_yaw", "head_roll"];
const DIFFERENCE_STEP: f64 = 1e-5;
const UNDISTORT_TOLERANCE: f64 = 1e-12;
const UNDISTORT_ITERATIONS: usize = 30;

#[derive(Debug, Error)]
pub enum VisualTrackingError {
    #[error("invalid MJCF: {0}")]
    Parse(#[from] ParseError),
    #[error("missing joint {0}")]
    MissingJoint(&'static str),
    #[error("duplicate joint {0}")]
    DuplicateJoint(&'static str),
    #[error("missing body {0}")]
    MissingBody(&'static str),
    #[error("duplicate body {0}")]
    DuplicateBody(&'static str),
    #[error("unsupported camera joint chain: {0}")]
    InvalidJointChain(&'static str),
    #[error("invalid MJCF transform or joint: {0}")]
    InvalidModel(&'static str),
    #[error("the optical mounting quaternion must be finite and nonzero")]
    InvalidMount,
    #[error("invalid camera dimensions, rotation, intrinsics or distortion")]
    InvalidCamera,
    #[error("upright image coordinates must be finite and within [-1, 1]")]
    InvalidImagePoint,
    #[error("head joint positions must be finite")]
    InvalidJointPositions,
    #[error("the direction must be finite and nonzero")]
    InvalidDirection,
    #[error("the direction is not in front of the camera")]
    BehindCamera,
    #[error("camera distortion could not be inverted")]
    UndistortionFailed,
    #[error("camera projection produced a non-finite value")]
    InvalidProjection,
    #[error("joint perturbation cannot be represented for the image Jacobian")]
    JacobianUnavailable,
}

/// Intrinsics in pixels of the raw, unrotated image.
///
/// `rotate` is the raw-to-upright image rotation in degrees. Distortion is
/// OpenCV's `[k1, k2, p1, p2, k3]`; callers must supply the actual model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraModel {
    pub width: u32,
    pub height: u32,
    pub rotate: u16,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub distortion: [f64; 5],
}

impl CameraModel {
    /// Check the supplied geometry without supplying a calibration or mount.
    pub fn validate(&self) -> Result<(), VisualTrackingError> {
        if self.width == 0
            || self.height == 0
            || !matches!(self.rotate, 0 | 90 | 180 | 270)
            || !self.fx.is_finite()
            || self.fx <= 0.0
            || !self.fy.is_finite()
            || self.fy <= 0.0
            || !self.cx.is_finite()
            || !self.cy.is_finite()
            || !self.distortion.iter().all(|value| value.is_finite())
        {
            return Err(VisualTrackingError::InvalidCamera);
        }
        Ok(())
    }

    /// Convert an upright image point into a unit upright optical-frame ray.
    ///
    /// Points use continuous image-edge coordinates: left/top are -1 and
    /// right/bottom are +1, matching the existing vision target protocol.
    pub fn upright_ray(&self, point: [f64; 2]) -> Result<[f64; 3], VisualTrackingError> {
        self.validate()?;
        if !point
            .iter()
            .all(|value| value.is_finite() && value.abs() <= 1.0)
        {
            return Err(VisualTrackingError::InvalidImagePoint);
        }
        let u = (point[0] + 1.0) / 2.0;
        let v = (point[1] + 1.0) / 2.0;
        let (raw_u, raw_v) = match self.rotate {
            0 => (u, v),
            90 => (v, 1.0 - u),
            180 => (1.0 - u, 1.0 - v),
            270 => (1.0 - v, u),
            _ => unreachable!("camera rotation was validated"),
        };
        let distorted = [
            (raw_u * f64::from(self.width) - self.cx) / self.fx,
            (raw_v * f64::from(self.height) - self.cy) / self.fy,
        ];
        let [x, y] = self.undistort(distorted)?;
        let upright = match self.rotate {
            0 => [x, y, 1.0],
            90 => [-y, x, 1.0],
            180 => [-x, -y, 1.0],
            270 => [y, -x, 1.0],
            _ => unreachable!("camera rotation was validated"),
        };
        unit_direction(upright)
    }

    /// Project an upright optical-frame direction into upright image coordinates.
    ///
    /// A point outside `[-1, 1]` is returned as-is, so projections and
    /// derivatives near an image edge are not clipped.
    pub fn project_upright(&self, direction: [f64; 3]) -> Result<[f64; 2], VisualTrackingError> {
        self.validate()?;
        let [x, y, z] = unit_direction(direction)?;
        if z <= 0.0 {
            return Err(VisualTrackingError::BehindCamera);
        }
        let (raw_x, raw_y) = match self.rotate {
            0 => (x, y),
            90 => (y, -x),
            180 => (-x, -y),
            270 => (-y, x),
            _ => unreachable!("camera rotation was validated"),
        };
        let (distorted, _) = self.distort([raw_x / z, raw_y / z]);
        let raw_u = (distorted[0] * self.fx + self.cx) / f64::from(self.width);
        let raw_v = (distorted[1] * self.fy + self.cy) / f64::from(self.height);
        let (u, v) = match self.rotate {
            0 => (raw_u, raw_v),
            90 => (1.0 - raw_v, raw_u),
            180 => (1.0 - raw_u, 1.0 - raw_v),
            270 => (raw_v, 1.0 - raw_u),
            _ => unreachable!("camera rotation was validated"),
        };
        let point = [2.0 * u - 1.0, 2.0 * v - 1.0];
        if !point.iter().all(|value| value.is_finite()) {
            return Err(VisualTrackingError::InvalidProjection);
        }
        Ok(point)
    }

    // Brown-Conrady projection and its 2x2 derivative in raw optical axes.
    fn distort(&self, [x, y]: [f64; 2]) -> ([f64; 2], [[f64; 2]; 2]) {
        let [k1, k2, p1, p2, k3] = self.distortion;
        let r2 = x * x + y * y;
        let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
        let radial_slope = k1 + r2 * (2.0 * k2 + r2 * 3.0 * k3);
        let dx = 2.0 * x * radial_slope;
        let dy = 2.0 * y * radial_slope;
        (
            [
                x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
                y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
            ],
            [
                [
                    radial + x * dx + 2.0 * p1 * y + 6.0 * p2 * x,
                    x * dy + 2.0 * p1 * x + 2.0 * p2 * y,
                ],
                [
                    y * dx + 2.0 * p1 * x + 2.0 * p2 * y,
                    radial + y * dy + 6.0 * p1 * y + 2.0 * p2 * x,
                ],
            ],
        )
    }

    fn undistort(&self, distorted: [f64; 2]) -> Result<[f64; 2], VisualTrackingError> {
        let mut point = distorted;
        for _ in 0..UNDISTORT_ITERATIONS {
            let (projected, j) = self.distort(point);
            let error = [projected[0] - distorted[0], projected[1] - distorted[1]];
            if error
                .iter()
                .all(|value| value.is_finite() && value.abs() <= UNDISTORT_TOLERANCE)
            {
                return Ok(point);
            }
            let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
            if !det.is_finite() || det == 0.0 {
                return Err(VisualTrackingError::UndistortionFailed);
            }
            point[0] -= (j[1][1] * error[0] - j[0][1] * error[1]) / det;
            point[1] -= (j[0][0] * error[1] - j[1][0] * error[0]) / det;
            if !point.iter().all(|value| value.is_finite()) {
                return Err(VisualTrackingError::UndistortionFailed);
            }
        }
        Err(VisualTrackingError::UndistortionFailed)
    }
}

#[derive(Clone, Copy)]
struct HeadLink {
    rest: Pose,
    joint: Option<(usize, [f64; 3])>,
}

/// The compiled public-model head chain, independent of any camera calibration.
pub struct VisualKinematics {
    head_chain: Box<[HeadLink]>,
    camera_offset: [f64; 3],
    optical_to_head_roll: Quat,
}

impl VisualKinematics {
    /// Compile the caller's MJCF and its explicit upright-optical-to-head rotation.
    ///
    /// The `camera` body origin is used as the camera position. Its orientation
    /// is not treated as an optical calibration. The camera must be fixed to
    /// `head_roll`, and its chain must contain exactly the four named joints.
    pub fn new(xml: &str, optical_to_head_roll: Quat) -> Result<Self, VisualTrackingError> {
        let optical_to_head_roll = unit_quaternion(optical_to_head_roll)?;
        let model = Model::parse(xml)?;
        validate_model_path(xml)?;

        let mut joint_indices = [0; 4];
        for (slot, name) in HEAD_JOINTS.into_iter().enumerate() {
            let mut matches = model
                .joint_names
                .iter()
                .enumerate()
                .filter(|(_, item)| *item == name);
            joint_indices[slot] = matches
                .next()
                .ok_or(VisualTrackingError::MissingJoint(name))?
                .0;
            if matches.next().is_some() {
                return Err(VisualTrackingError::DuplicateJoint(name));
            }
        }
        let body = |name: &'static str| {
            let mut matches = model
                .body_names
                .iter()
                .enumerate()
                .filter(|(_, item)| *item == name);
            let index = matches
                .next()
                .ok_or(VisualTrackingError::MissingBody(name))?
                .0;
            if matches.next().is_some() {
                return Err(VisualTrackingError::DuplicateBody(name));
            }
            Ok(index)
        };
        let camera = body("camera")?;
        let head_roll = body("head_roll")?;
        if model.body_joint[head_roll].map(|joint| joint.0) != Some(joint_indices[3]) {
            return Err(VisualTrackingError::InvalidJointChain(
                "head_roll body and joint differ",
            ));
        }

        let mut path = Vec::new();
        let mut current = Some(camera);
        while let Some(index) = current {
            if model.body_parents[index].is_some() {
                path.push(index);
            }
            current = model.body_parents[index];
        }
        path.reverse();
        let head_end = path.iter().position(|&index| index == head_roll).ok_or(
            VisualTrackingError::InvalidJointChain("camera is not under head_roll"),
        )?;
        let mut head_chain = Vec::with_capacity(head_end + 1);
        let mut next_joint = 0;
        for &index in &path[..=head_end] {
            let joint = match model.body_joint[index] {
                Some((model_joint, axis)) => {
                    if next_joint >= joint_indices.len() || model_joint != joint_indices[next_joint]
                    {
                        return Err(VisualTrackingError::InvalidJointChain(
                            "unexpected head joint or order",
                        ));
                    }
                    let slot = next_joint;
                    next_joint += 1;
                    Some((slot, axis))
                }
                None => None,
            };
            head_chain.push(HeadLink {
                rest: model.body_rest[index],
                joint,
            });
        }
        if next_joint != joint_indices.len() {
            return Err(VisualTrackingError::InvalidJointChain(
                "head chain is incomplete",
            ));
        }

        let mut camera_in_head_roll = Pose::IDENTITY;
        for &index in &path[head_end + 1..] {
            if model.body_joint[index].is_some() {
                return Err(VisualTrackingError::InvalidJointChain(
                    "camera is not fixed to head_roll",
                ));
            }
            camera_in_head_roll = camera_in_head_roll * model.body_rest[index];
        }
        Ok(Self {
            head_chain: head_chain.into_boxed_slice(),
            camera_offset: camera_in_head_roll.pos,
            optical_to_head_roll,
        })
    }

    /// Camera body origin and upright optical orientation in the trunk frame.
    ///
    /// Angles are measured radians in ID 30, 31, 32, 33 order. The position is
    /// the model's `camera` body origin, not an independently measured optical centre.
    pub fn camera_pose(&self, joints: [f64; 4]) -> Result<Pose, VisualTrackingError> {
        if !joints.iter().all(|angle| angle.is_finite()) {
            return Err(VisualTrackingError::InvalidJointPositions);
        }
        let mut head = Pose::IDENTITY;
        for link in &self.head_chain {
            head = head * link.rest;
            if let Some((index, axis)) = link.joint {
                head.quat = head.quat * Quat::from_axis_angle(axis, joints[index]);
            }
        }
        let pose = Pose::new(
            head.transform_point(self.camera_offset),
            head.quat * self.optical_to_head_roll,
        );
        if !pose
            .pos
            .iter()
            .chain(pose.quat.wxyz().iter())
            .all(|value| value.is_finite())
        {
            return Err(VisualTrackingError::InvalidModel("non-finite camera pose"));
        }
        Ok(pose)
    }

    /// Convert an upright normalized image point to a unit trunk-frame direction.
    pub fn ray_in_trunk(
        &self,
        camera: &CameraModel,
        point: [f64; 2],
        joints: [f64; 4],
    ) -> Result<[f64; 3], VisualTrackingError> {
        let ray = camera.upright_ray(point)?;
        unit_direction(self.camera_pose(joints)?.quat.rotate(ray))
    }

    /// Project a trunk-frame direction into the upright normalized image.
    pub fn project_to_image(
        &self,
        camera: &CameraModel,
        ray_in_trunk: [f64; 3],
        joints: [f64; 4],
    ) -> Result<[f64; 2], VisualTrackingError> {
        let ray = unit_direction(ray_in_trunk)?;
        let optical = self.camera_pose(joints)?.quat.conjugate().rotate(ray);
        camera.project_upright(optical)
    }

    /// Derivative of upright image coordinates for a fixed trunk-frame ray.
    ///
    /// Rows are image x/y; columns are ID 31 pitch / ID 32 yaw, in normalized
    /// image units per radian. This direction-only Jacobian ignores translation.
    pub fn image_jacobian(
        &self,
        camera: &CameraModel,
        ray_in_trunk: [f64; 3],
        joints: [f64; 4],
    ) -> Result<[[f64; 2]; 2], VisualTrackingError> {
        if !joints.iter().all(|angle| angle.is_finite()) {
            return Err(VisualTrackingError::InvalidJointPositions);
        }
        let mut jacobian = [[0.0; 2]; 2];
        for (column, joint) in [1, 2].into_iter().enumerate() {
            let mut plus = joints;
            let mut minus = joints;
            plus[joint] += DIFFERENCE_STEP;
            minus[joint] -= DIFFERENCE_STEP;
            if plus[joint] == joints[joint] || minus[joint] == joints[joint] {
                return Err(VisualTrackingError::JacobianUnavailable);
            }
            let upper = self.project_to_image(camera, ray_in_trunk, plus)?;
            let lower = self.project_to_image(camera, ray_in_trunk, minus)?;
            for row in 0..2 {
                jacobian[row][column] = (upper[row] - lower[row]) / (2.0 * DIFFERENCE_STEP);
                if !jacobian[row][column].is_finite() {
                    return Err(VisualTrackingError::InvalidProjection);
                }
            }
        }
        Ok(jacobian)
    }
}

fn unit_direction(direction: [f64; 3]) -> Result<[f64; 3], VisualTrackingError> {
    if !direction.iter().all(|value| value.is_finite()) {
        return Err(VisualTrackingError::InvalidDirection);
    }
    let scale = direction
        .iter()
        .map(|value| value.abs())
        .fold(0.0, f64::max);
    if scale == 0.0 {
        return Err(VisualTrackingError::InvalidDirection);
    }
    let scaled = direction.map(|value| value / scale);
    let length = scaled[0].hypot(scaled[1]).hypot(scaled[2]);
    Ok(scaled.map(|value| value / length))
}

fn unit_quaternion(quaternion: Quat) -> Result<Quat, VisualTrackingError> {
    let components = quaternion.wxyz();
    if !components.iter().all(|value| value.is_finite()) {
        return Err(VisualTrackingError::InvalidMount);
    }
    let scale = components
        .iter()
        .map(|value| value.abs())
        .fold(0.0, f64::max);
    if scale == 0.0 {
        return Err(VisualTrackingError::InvalidMount);
    }
    let [w, x, y, z] = components.map(|value| value / scale);
    let length = w.hypot(x).hypot(y).hypot(z);
    Ok(Quat::new(w / length, x / length, y / length, z / length))
}

// The generic MJCF reader intentionally ignores some attributes. Refuse those
// on this chain rather than producing a plausible but incorrect camera pose.
fn validate_model_path(xml: &str) -> Result<(), VisualTrackingError> {
    let document = roxmltree::Document::parse(xml).map_err(ParseError::from)?;
    let trunk = document
        .descendants()
        .find(|node| node.has_tag_name("body") && node.attribute("name") == Some("trunk_base"))
        .ok_or(VisualTrackingError::MissingBody("trunk_base"))?;
    let camera = trunk
        .descendants()
        .find(|node| node.has_tag_name("body") && node.attribute("name") == Some("camera"))
        .ok_or(VisualTrackingError::MissingBody("camera"))?;
    for body in camera.ancestors().take_while(|node| *node != trunk) {
        if !body.has_tag_name("body") {
            continue;
        }
        if ["euler", "axisangle", "xyaxes", "zaxis"]
            .iter()
            .any(|name| body.attribute(*name).is_some())
        {
            return Err(VisualTrackingError::InvalidModel(
                "unsupported body rotation attribute",
            ));
        }
        if let Some(position) = body.attribute("pos") {
            parse_vector::<3>(position)?;
        }
        if let Some(quaternion) = body.attribute("quat") {
            let q = parse_vector::<4>(quaternion)?;
            let norm = q.iter().map(|value| value * value).sum::<f64>().sqrt();
            if !norm.is_finite() || norm < 1e-12 {
                return Err(VisualTrackingError::InvalidModel("invalid body quaternion"));
            }
        }
        let mut joints = body.children().filter(|node| node.has_tag_name("joint"));
        if let Some(joint) = joints.next() {
            if joints.next().is_some()
                || joint.attribute("type").is_some_and(|kind| kind != "hinge")
            {
                return Err(VisualTrackingError::InvalidModel(
                    "body must have at most one hinge",
                ));
            }
            if let Some(axis) = joint.attribute("axis") {
                let axis = parse_vector::<3>(axis)?;
                let norm = axis.iter().map(|value| value * value).sum::<f64>().sqrt();
                if !norm.is_finite() || norm < 1e-12 {
                    return Err(VisualTrackingError::InvalidModel("invalid joint axis"));
                }
            }
            if let Some(position) = joint.attribute("pos")
                && parse_vector::<3>(position)? != [0.0; 3]
            {
                return Err(VisualTrackingError::InvalidModel(
                    "offset hinge is unsupported",
                ));
            }
            if let Some(reference) = joint.attribute("ref")
                && reference
                    .parse::<f64>()
                    .ok()
                    .filter(|value| *value == 0.0)
                    .is_none()
            {
                return Err(VisualTrackingError::InvalidModel(
                    "nonzero joint reference is unsupported",
                ));
            }
        }
    }
    Ok(())
}

fn parse_vector<const N: usize>(value: &str) -> Result<[f64; N], VisualTrackingError> {
    let mut result = [0.0; N];
    let mut values = value.split_whitespace();
    for item in &mut result {
        *item = values
            .next()
            .and_then(|part| part.parse::<f64>().ok())
            .filter(|part| part.is_finite())
            .ok_or(VisualTrackingError::InvalidModel("invalid vector"))?;
    }
    if values.next().is_some() {
        return Err(VisualTrackingError::InvalidModel("invalid vector"));
    }
    Ok(result)
}
