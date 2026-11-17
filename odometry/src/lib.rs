//! Where the robot is, estimated from its own legs and IMU.
//!
//! A port of the prototype runtime's contact-based odometry, itself derived
//! from Rhoban's humanoid `model_service`. The idea: at any moment one point of
//! one sole is the robot's contact with the ground. Anchor that point to the
//! world (it is on flat ground, so its world Z is 0 and its world X/Y are
//! wherever it was when it became the anchor), orient the trunk by the IMU, and
//! the trunk's world position follows by forward kinematics. When some other
//! sole corner drops below the anchor — a step — the anchor moves there, at
//! that corner's current world X/Y, so the estimate never jumps.
//!
//! Two changes from the prototype:
//!   - The foot chains come from the `kinematics` crate's MJCF model instead of
//!     hand-transcribed segment tables — the geometry has one source of truth.
//!   - Each foot's chain is evaluated once per tick and its four corners are
//!     transformed through the result; the prototype re-walked a full leg chain
//!     per corner (9 chain evaluations per tick, now 2).
//!
//! Heading is whatever the IMU's integrated yaw says — there is no
//! magnetometer, so the world frame is "wherever the robot was looking at
//! boot", which is all a relative-motion consumer needs.
//!
//! Alpha only, like the daemon: v1/v1.5 geometry stayed in the prototype.
//!
//! The candidate contact points — the "sole corners" — are an [`AnchorSet`],
//! sampled on the sole mesh by `microduck_rl/scripts/odom_anchor_points.py`
//! (see [`anchors`]). [`Odometry::alpha`] uses [`ALPHA16`], a 4x4 grid over
//! the whole sole on the mesh: in the MuJoCo twin over a 3.2 m walk it drifted
//! 9 mm where the legacy v1.5 bbox ([`V15`]) drifted 17 mm and the flat
//! patch's four corners ([`ALPHA4`]) 16 mm, and it stands at the true height
//! (V15 sat 3.7 mm high). The scan costs about half a microsecond more.

mod anchors;

pub use anchors::{ALPHA4, ALPHA16, AnchorSet, V15};
use duck_ipc_proto::JOINT_NAMES;
use kinematics::{Model, Pose, Quat, SiteId};

/// A candidate corner must sit below world Z = `-SWITCH_MARGIN` to bid for the
/// anchor. The anchor itself sits at Z = 0, so the margin is slack for FK and
/// IMU noise, not a physical depth.
const SWITCH_MARGIN: f64 = -0.010;

/// Ticks a candidate must stay the lowest point before the anchor moves. At
/// the control loop's 50 Hz this is 40 ms — well inside a stance phase, well
/// past a one-tick glitch.
const SWITCH_CONFIRM_TICKS: u32 = 2;

const LEFT: usize = 0;
const RIGHT: usize = 1;

pub struct Odometry {
    model: &'static Model,
    feet: [SiteId; 2],
    /// The candidate contact points on each sole.
    anchors: &'static AnchorSet,
    /// `JOINT_NAMES` position → model joint index. `None` for the mouth, which
    /// moves no leg.
    joint_map: [Option<usize>; JOINT_NAMES.len()],
    /// Scratch angle slice in model order, reused every tick.
    angles: Vec<f64>,
    /// Which foot the anchor is on.
    anchor_foot: usize,
    /// The contact point, in that foot's site frame.
    anchor_local: [f64; 3],
    /// The contact point's world X/Y. Its Z is 0 by definition: flat ground.
    anchor_xy: [f64; 2],
    position: [f64; 3],
    yaw: f64,
    /// A corner bidding to become the anchor: (foot, local point, world X/Y).
    pending: Option<(usize, [f64; 3], [f64; 2])>,
    pending_ticks: u32,
    /// True until the first update seeds `anchor_xy` from the actual startup
    /// pose, so the trunk starts at (0, 0) instead of offset by the foot.
    needs_init: bool,
}

impl Odometry {
    pub fn new(model: &'static Model, anchors: &'static AnchorSet) -> Self {
        let feet = [
            model.site("left_foot").expect("model has a left_foot site"),
            model
                .site("right_foot")
                .expect("model has a right_foot site"),
        ];
        Self {
            feet,
            anchors,
            joint_map: JOINT_NAMES.map(|name| model.joint_index(name)),
            angles: vec![0.0; model.num_joints()],
            model,
            anchor_foot: LEFT,
            anchor_local: [0.0; 3],
            anchor_xy: [0.0; 2],
            position: [0.0; 3],
            yaw: 0.0,
            pending: None,
            pending_ticks: 0,
            needs_init: true,
        }
    }

    /// The alpha robot with the [`ALPHA16`] grid — the production estimator.
    pub fn alpha() -> Self {
        Self::new(Model::alpha(), &ALPHA16)
    }

    /// The alpha robot with a chosen anchor set.
    pub fn alpha_with(anchors: &'static AnchorSet) -> Self {
        Self::new(Model::alpha(), anchors)
    }

    /// The anchor set this estimator picks its contact point from.
    pub fn anchors(&self) -> &'static AnchorSet {
        self.anchors
    }

    /// Advance the estimate by one sensor sample.
    ///
    /// `joints` are measured angles in [`JOINT_NAMES`] order (the mouth is
    /// ignored); `quat_wxyz` is the IMU's trunk-in-world orientation. Call it
    /// on fresh samples only — a coasted tick has nothing new to say.
    pub fn update(&mut self, joints: &[f64; JOINT_NAMES.len()], quat_wxyz: [f64; 4]) {
        for (angle, idx) in joints.iter().zip(self.joint_map) {
            if let Some(idx) = idx {
                self.angles[idx] = *angle;
            }
        }
        let [w, x, y, z] = quat_wxyz;
        let rot = Quat::new(w, x, y, z).normalized();

        // Both foot poses, once — everything below reads these.
        let feet = [
            self.model.site_pose(self.feet[LEFT], &self.angles),
            self.model.site_pose(self.feet[RIGHT], &self.angles),
        ];

        if self.needs_init {
            let anchor_world = rot.rotate(feet[self.anchor_foot].pos);
            self.anchor_xy = [anchor_world[0], anchor_world[1]];
            self.needs_init = false;
        }

        self.reproject(rot, &feet);

        // A corner below the anchor is a step landing — but only after it holds
        // the claim for SWITCH_CONFIRM_TICKS, so FK jitter cannot walk the
        // anchor around mid-stance.
        match self.lowest_corner(rot, &feet) {
            None => {
                self.pending = None;
                self.pending_ticks = 0;
            }
            Some((foot, local, world_xy)) => {
                if self.pending.is_some_and(|(pf, _, _)| pf == foot) {
                    self.pending_ticks += 1;
                } else {
                    self.pending = Some((foot, local, world_xy));
                    self.pending_ticks = 1;
                }
                if self.pending_ticks >= SWITCH_CONFIRM_TICKS {
                    let (foot, local, world_xy) = self.pending.take().expect("just matched");
                    self.anchor_foot = foot;
                    self.anchor_local = local;
                    // The corner keeps the world X/Y it already has, so the
                    // estimate is continuous across the switch.
                    self.anchor_xy = world_xy;
                    self.reproject(rot, &feet);
                    self.pending_ticks = 0;
                }
            }
        }

        self.yaw = rot.yaw();
    }

    /// Trunk position in the world frame, metres. Z is height above the ground
    /// plane the anchor defines.
    pub fn position(&self) -> [f64; 3] {
        self.position
    }

    /// Trunk heading, radians, in the IMU's boot-relative world frame.
    pub fn yaw(&self) -> f64 {
        self.yaw
    }

    /// The contact anchor's world X/Y — where the estimator believes the stance
    /// foot touches the ground. For telemetry; `position` is the answer.
    pub fn anchor_xy(&self) -> [f64; 2] {
        self.anchor_xy
    }

    /// Which foot carries the anchor: 0 = left, 1 = right.
    pub fn anchor_foot(&self) -> usize {
        self.anchor_foot
    }

    /// Trunk position from the anchor: the contact point sits at
    /// (`anchor_xy`, 0), the trunk is minus the world-rotated trunk→contact
    /// vector away from it.
    fn reproject(&mut self, rot: Quat, feet: &[Pose; 2]) {
        let contact_in_trunk = feet[self.anchor_foot].transform_point(self.anchor_local);
        let contact = rot.rotate(contact_in_trunk);
        self.position = [
            self.anchor_xy[0] - contact[0],
            self.anchor_xy[1] - contact[1],
            -contact[2],
        ];
    }

    /// The lowest sole corner below the switch threshold, if any, with its
    /// current world X/Y.
    fn lowest_corner(&self, rot: Quat, feet: &[Pose; 2]) -> Option<(usize, [f64; 3], [f64; 2])> {
        let mut lowest = -SWITCH_MARGIN;
        let mut best = None;
        for foot in [LEFT, RIGHT] {
            for &corner in self.anchors.foot(foot) {
                let in_world = rot.rotate(feet[foot].transform_point(corner));
                let world = [
                    self.position[0] + in_world[0],
                    self.position[1] + in_world[1],
                    self.position[2] + in_world[2],
                ];
                if world[2] < lowest {
                    lowest = world[2];
                    best = Some((foot, corner, [world[0], world[1]]));
                }
            }
        }
        best
    }
}
