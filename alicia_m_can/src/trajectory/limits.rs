use crate::config::JointMotorConfig;

pub const DEFAULT_MAX_JOINT_ACCEL_RAD_S2: f64 = 8.0;
pub const DEFAULT_MAX_JOINT_JERK_RAD_S3: f64 = 50.0;

const ACCEL_TIME_S: f64 = 0.20;
const JERK_TIME_S: f64 = 0.05;
const MIN_ACCEL_RAD_S2: f64 = 0.20;
const MIN_JERK_RAD_S3: f64 = 4.0;

/// One continuous speed-to-limits mapping shared by P2P and stream control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionLimits {
    pub velocity: f64,
    pub acceleration: f64,
    pub jerk: f64,
}

pub fn motion_limits_from_vlim(vlim_rad_s: f64) -> MotionLimits {
    let velocity = vlim_rad_s.max(0.0);
    let acceleration =
        (velocity / ACCEL_TIME_S).clamp(MIN_ACCEL_RAD_S2, DEFAULT_MAX_JOINT_ACCEL_RAD_S2);
    let jerk = (acceleration / JERK_TIME_S).clamp(MIN_JERK_RAD_S3, DEFAULT_MAX_JOINT_JERK_RAD_S3);
    MotionLimits {
        velocity,
        acceleration,
        jerk,
    }
}

pub fn stream_amax_from_vlim(vlim_rad_s: f64) -> f64 {
    motion_limits_from_vlim(vlim_rad_s).acceleration
}

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
    let limits = motion_limits_from_vlim(vlim_rad_s);
    let vlim = limits.velocity;
    let amax = finite_positive(max_accel_rad_s2).unwrap_or(limits.acceleration);
    let jmax = finite_positive(max_jerk_rad_s3).unwrap_or(limits.jerk);
    TrajectoryLimits {
        max_velocity: per_joint_vlim(num_joints, vlim, joint_cfgs),
        max_acceleration: vec![amax; num_joints],
        max_jerk: vec![jmax; num_joints],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn p2p_amax_scales_with_vlim() {
        let lim = build_trajectory_limits_from_vlim(6, 0.6, &[], None, None);
        assert!((lim.max_acceleration[0] - 0.6 / ACCEL_TIME_S).abs() < 1e-9);
        assert!(lim.max_jerk[0] >= MIN_JERK_RAD_S3);
    }

    #[test]
    fn explicit_amax_overrides_scale() {
        let lim = build_trajectory_limits_from_vlim(1, 0.6, &[], Some(8.0), Some(50.0));
        assert!((lim.max_acceleration[0] - 8.0).abs() < 1e-9);
        assert!((lim.max_jerk[0] - 50.0).abs() < 1e-9);
    }
}
