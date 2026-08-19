use crate::config::JointMotorConfig;

pub const DEFAULT_MAX_JOINT_ACCEL_RAD_S2: f64 = 8.0;
pub const DEFAULT_MAX_JOINT_JERK_RAD_S3: f64 = 50.0;

#[derive(Debug, Clone)]
pub struct TrajectoryLimits {
    pub max_velocity: Vec<f64>,
    pub max_acceleration: Vec<f64>,
    pub max_jerk: Vec<f64>,
}

pub fn build_trajectory_limits_from_vlim(
    num_joints: usize,
    vlim_rad_s: f64,
    joint_cfgs: &[JointMotorConfig],
    max_accel_rad_s2: Option<f64>,
    max_jerk_rad_s3: Option<f64>,
) -> TrajectoryLimits {
    let vlim = vlim_rad_s.max(0.0);
    let amax = max_accel_rad_s2
        .filter(|v| v.is_finite() && *v > 0.0)
        .unwrap_or(DEFAULT_MAX_JOINT_ACCEL_RAD_S2);
    let jmax = max_jerk_rad_s3
        .filter(|v| v.is_finite() && *v > 0.0)
        .unwrap_or(DEFAULT_MAX_JOINT_JERK_RAD_S3);

    let per_joint_vlim: Vec<f64> = if joint_cfgs.len() >= num_joints {
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
    };

    TrajectoryLimits {
        max_velocity: per_joint_vlim,
        max_acceleration: vec![amax; num_joints],
        max_jerk: vec![jmax; num_joints],
    }
}
