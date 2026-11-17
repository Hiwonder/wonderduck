//! Validated displacement policy.

use std::path::Path;

use crate::model::{DEFAULT_POSITION, JOINT_NAMES, MOUTH_INDEX, NUM_JOINTS};
use crate::obs::{ACTION_LEN, Observation};

use super::{
    Network, PolicyError, catching_ort_panics, ensure_runtime, open_with_precision, shape_error,
};

/// The preloaded displacement network. Raw action history belongs to the caller.
pub struct PrecisionPolicy {
    student: Network,
    home: [f64; NUM_JOINTS],
}

impl PrecisionPolicy {
    /// Load and warm the displacement network after validating its model contract.
    pub fn load(student: &Path) -> Result<Self, PolicyError> {
        ensure_runtime()?;
        catching_ort_panics(|| {
            let mut student = open_with_precision(student, true)?;
            let student_contract = Contract::read(&student)?;

            let zero = Observation::zeroed();
            student.run(&zero)?;
            student.reset();
            Ok(Self {
                student,
                home: student_contract.home(),
            })
        })
    }

    /// The student HOME in physical joint order, including the fixed mouth slot.
    pub fn home(&self) -> [f64; NUM_JOINTS] {
        self.home
    }

    /// Infer directly from the supplied observation, including its previous raw action.
    pub fn infer(&mut self, observation: &Observation) -> Result<[f32; ACTION_LEN], PolicyError> {
        let result = self.student.run(observation);
        if result.is_err() {
            self.student.reset();
        }
        result
    }

    /// Clear the network state. The caller retains physical action history.
    pub fn reset(&mut self) {
        self.student.reset();
    }
}

struct Contract {
    home: Vec<f64>,
}

impl Contract {
    fn read(network: &Network) -> Result<Self, PolicyError> {
        let metadata = network
            .session
            .metadata()
            .map_err(|source| PolicyError::Load {
                path: network.path.clone(),
                source,
            })?;
        Self::parse(&network.path, |key| metadata.custom(key))
    }

    fn parse(
        path: &Path,
        get: impl Fn(&str) -> Option<String>,
    ) -> Result<Self, PolicyError> {
        let encoding = get("command_encoding");
        if encoding.as_deref() != Some("precision_displacement") {
            return Err(shape_error(
                path,
                "command_encoding",
                "precision_displacement",
                encoding.unwrap_or_default(),
            ));
        }
        require_value(path, "collision_profile", get("collision_profile"), "full")?;
        require_value(
            path,
            "command_names",
            get("command_names"),
            "twist,head_pose,body_pose",
        )?;
        let scale = required(path, "action_scale", get("action_scale"))?;
        if scale.trim().parse::<f64>().ok() != Some(1.0) {
            return Err(shape_error(path, "action_scale", "1", scale));
        }
        let names = required(path, "joint_names", get("joint_names"))?;
        let actual: Vec<_> = names.split(',').map(str::trim).collect();
        let expected: Vec<_> = JOINT_NAMES
            .iter()
            .enumerate()
            .filter_map(|(index, name)| (index != MOUTH_INDEX).then_some(*name))
            .collect();
        if actual != expected {
            return Err(shape_error(path, "joint_names", &expected.join(","), names));
        }
        let encoded_home = required(path, "default_joint_pos", get("default_joint_pos"))?;
        let encoded_reference =
            get("policy_joint_reference").unwrap_or_else(|| encoded_home.clone());
        joint_values(path, "policy_joint_reference", &encoded_reference)?;
        Ok(Self {
            home: joint_values(path, "default_joint_pos", &encoded_home)?,
        })
    }

    fn home(&self) -> [f64; NUM_JOINTS] {
        let mut home = DEFAULT_POSITION;
        for ((_, joint), value) in home
            .iter_mut()
            .enumerate()
            .filter(|(index, _)| *index != MOUTH_INDEX)
            .zip(self.home.iter())
        {
            *joint = *value;
        }
        home
    }
}

fn required(
    path: &Path,
    field: &'static str,
    value: Option<String>,
) -> Result<String, PolicyError> {
    value.ok_or_else(|| shape_error(path, field, "present", "missing".into()))
}

fn require_value(
    path: &Path,
    field: &'static str,
    value: Option<String>,
    expected: &str,
) -> Result<(), PolicyError> {
    if value.as_deref() != Some(expected) {
        return Err(shape_error(
            path,
            field,
            expected,
            value.unwrap_or_else(|| "missing".into()),
        ));
    }
    Ok(())
}

fn joint_values(path: &Path, field: &'static str, encoded: &str) -> Result<Vec<f64>, PolicyError> {
    let values: Result<Vec<f64>, _> = encoded
        .split(',')
        .map(|value| value.trim().parse())
        .collect();
    match values {
        Ok(values)
            if values.len() == ACTION_LEN && values.iter().all(|value| value.is_finite()) =>
        {
            Ok(values)
        }
        _ => Err(shape_error(
            path,
            field,
            "14 finite joint values",
            encoded.into(),
        )),
    }
}
