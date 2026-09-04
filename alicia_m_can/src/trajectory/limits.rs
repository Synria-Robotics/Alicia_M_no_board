use crate::config::JointMotorConfig;

pub const DEFAULT_MAX_JOINT_ACCEL_RAD_S2: f64 = 8.0;
pub const DEFAULT_MAX_JOINT_JERK_RAD_S3: f64 = 50.0;

/// Stream teleop: keep a visible S-curve at low `vlim` instead of a rectangular velocity.
const STREAM_ACCEL_TIME_S: f64 = 0.15;
const MIN_STREAM_ACCEL_RAD_S2: f64 = 0.15;

/// P2P: ramp to vmax in ~0.25 s so low `speed` is not a square-wave start/stop.
const P2P_ACCEL_TIME_S: f64 = 0.25;
const MIN_P2P_ACCEL_RAD_S2: f64 = 1.0;
const P2P_JERK_TIME_S: f64 = 0.05;
const MIN_P2P_JERK_RAD_S3: f64 = 8.0;

/// Walker / stream Ruckig share this `amax(vlim)` so MIT `dq` ramps in ~0.15 s.
pub fn stream_amax_from_vlim(vlim_rad_s: f64) -> f64 {
    let vlim = vlim_rad_s.max(0.0);
    (vlim / STREAM_ACCEL_TIME_S).clamp(MIN_STREAM_ACCEL_RAD_S2, DEFAULT_MAX_JOINT_ACCEL_RAD_S2)
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
    let vlim = vlim_rad_s.max(0.0);
    let amax = finite_positive(max_accel_rad_s2).unwrap_or_else(|| {
        (vlim / P2P_ACCEL_TIME_S).clamp(MIN_P2P_ACCEL_RAD_S2, DEFAULT_MAX_JOINT_ACCEL_RAD_S2)
    });
    let jmax = finite_positive(max_jerk_rad_s3).unwrap_or_else(|| {
        (amax / P2P_JERK_TIME_S).clamp(MIN_P2P_JERK_RAD_S3, DEFAULT_MAX_JOINT_JERK_RAD_S3)
    });
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
        assert!((lim.max_acceleration[0] - 0.6 / P2P_ACCEL_TIME_S).abs() < 1e-9);
        assert!(lim.max_jerk[0] >= MIN_P2P_JERK_RAD_S3);
    }

    #[test]
    fn explicit_amax_overrides_scale() {
        let lim = build_trajectory_limits_from_vlim(1, 0.6, &[], Some(8.0), Some(50.0));
        assert!((lim.max_acceleration[0] - 8.0).abs() < 1e-9);
        assert!((lim.max_jerk[0] - 50.0).abs() < 1e-9);
    }
}
