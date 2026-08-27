use crate::config::JointMotorConfig;

pub const DEFAULT_MAX_JOINT_ACCEL_RAD_S2: f64 = 8.0;
pub const DEFAULT_MAX_JOINT_JERK_RAD_S3: f64 = 50.0;

/// Stream teleop: keep a visible S-curve at low `vlim` instead of a rectangular velocity.
const STREAM_ACCEL_TIME_S: f64 = 0.15;
const STREAM_JERK_TIME_S: f64 = 0.05;
const MIN_STREAM_ACCEL_RAD_S2: f64 = 0.15;
const MIN_STREAM_JERK_RAD_S3: f64 = 1.0;

#[derive(Debug, Clone)]
pub struct TrajectoryLimits {
    pub max_velocity: Vec<f64>,
    pub max_acceleration: Vec<f64>,
    pub max_jerk: Vec<f64>,
}

fn per_joint_vlim(num_joints: usize, vlim: f64, joint_cfgs: &[JointMotorConfig]) -> Vec<f64> {
    if joint_cfgs.len() >= num_joints {
        joint_cfgs
            .iter()
            .take(num_joints)
            .map(|joint| {
                if joint.pv_vlim > 0.0 {
                    vlim.min(joint.pv_vlim)
                } else {
                    vlim
                }
            })
            .collect()
    } else {
        vec![vlim; num_joints]
    }
}

fn finite_positive(value: Option<f64>) -> Option<f64> {
    value.filter(|v| v.is_finite() && *v > 0.0)
}

pub fn build_trajectory_limits_from_vlim(
    num_joints: usize,
    vlim_rad_s: f64,
    joint_cfgs: &[JointMotorConfig],
    max_accel_rad_s2: Option<f64>,
    max_jerk_rad_s3: Option<f64>,
) -> TrajectoryLimits {
    let vlim = vlim_rad_s.max(0.0);
    let amax = finite_positive(max_accel_rad_s2).unwrap_or(DEFAULT_MAX_JOINT_ACCEL_RAD_S2);
    let jmax = finite_positive(max_jerk_rad_s3).unwrap_or(DEFAULT_MAX_JOINT_JERK_RAD_S3);
    TrajectoryLimits {
        max_velocity: per_joint_vlim(num_joints, vlim, joint_cfgs),
        max_acceleration: vec![amax; num_joints],
        max_jerk: vec![jmax; num_joints],
    }
}

/// Like [`build_trajectory_limits_from_vlim`], but scales `amax`/`jmax` with `vlim`.
pub fn build_stream_limits_from_vlim(
    num_joints: usize,
    vlim_rad_s: f64,
    joint_cfgs: &[JointMotorConfig],
    max_accel_rad_s2: Option<f64>,
    max_jerk_rad_s3: Option<f64>,
) -> TrajectoryLimits {
    let vlim = vlim_rad_s.max(0.0);
    let amax = finite_positive(max_accel_rad_s2).unwrap_or_else(|| {
        (vlim / STREAM_ACCEL_TIME_S).clamp(MIN_STREAM_ACCEL_RAD_S2, DEFAULT_MAX_JOINT_ACCEL_RAD_S2)
    });
    let jmax = finite_positive(max_jerk_rad_s3).unwrap_or_else(|| {
        (amax / STREAM_JERK_TIME_S).clamp(MIN_STREAM_JERK_RAD_S3, DEFAULT_MAX_JOINT_JERK_RAD_S3)
    });
    TrajectoryLimits {
        max_velocity: per_joint_vlim(num_joints, vlim, joint_cfgs),
        max_acceleration: vec![amax; num_joints],
        max_jerk: vec![jmax; num_joints],
    }
}
