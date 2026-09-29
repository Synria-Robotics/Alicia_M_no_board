//! Alicia-D → Alicia-M joint map used by teleoperation.
//!
//! Same relationship as Alicia-M-SDK ``Teleoperation`` when its ``joint_mapper``
//! is ``convert_joints_rad_from_alicia_d_to_alicia_m``:
//! D zero aligns to the M interval midpoint, joint 3 (index 2) is negated and
//! scaled by travel, joints 4 and 6 (indexes 3 and 5) are reversed, and every
//! output is clipped to the Alicia-M limits.

use std::f64::consts::PI;

const D_LIMITS: [(f64, f64); 6] = [
    (-157.5, 157.5),
    (-100.2, 100.2),
    (-34.3, 126.0),
    (-159.8, 159.8),
    (-89.9, 89.9),
    (-179.9, 179.9),
];

const M_LIMITS: [(f64, f64); 6] = [
    (-157.5, 157.5),
    (-179.9, 0.0),
    (-179.9, 0.0),
    (-89.9, 89.9),
    (-89.9, 89.9),
    (-157.5, 157.5),
];

fn clamp(value: f64, lower: f64, upper: f64) -> f64 {
    value.clamp(lower.min(upper), lower.max(upper))
}

fn map_linear(value: f64, src_min: f64, src_max: f64, dst_min: f64, dst_max: f64) -> f64 {
    ((value - src_min) / (src_max - src_min)) * (dst_max - dst_min) + dst_min
}

fn map_with_m_limit(value: f64, dst_min: f64, dst_max: f64, reverse: bool) -> f64 {
    let dst_origin = (dst_min + dst_max) / 2.0;
    let direction = if reverse { -1.0 } else { 1.0 };
    clamp(direction * value + dst_origin, dst_min.min(dst_max), dst_min.max(dst_max))
}

pub fn alicia_d_deg_to_alicia_m_deg(joints_deg: &[f64; 6]) -> [f64; 6] {
    let mut out = [0.0; 6];
    for i in 0..6 {
        let joint = joints_deg[i];
        let (dst_min, dst_max) = M_LIMITS[i];
        out[i] = if i == 2 {
            let (src_min, src_max) = (-D_LIMITS[i].1, -D_LIMITS[i].0);
            let joint = clamp(-joint, src_min, src_max);
            clamp(
                map_linear(joint, src_min, src_max, dst_min, dst_max),
                dst_min.min(dst_max),
                dst_min.max(dst_max),
            )
        } else {
            map_with_m_limit(joint, dst_min, dst_max, i == 3 || i == 5)
        };
    }
    out
}

pub fn alicia_d_rad_to_alicia_m_rad(joints_rad: &[f64; 6]) -> [f64; 6] {
    let deg: [f64; 6] = joints_rad.map(|v| v * 180.0 / PI);
    alicia_d_deg_to_alicia_m_deg(&deg).map(|v| v * PI / 180.0)
}

pub fn map_gripper(value: f64, scale: f64) -> f64 {
    (value * scale).clamp(0.0, 1000.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_mapping_matches_sdk() {
        // Alicia-M-SDK JointMappingTest.test_zero_mapping_and_reversed_joints
        let mapped = alicia_d_deg_to_alicia_m_deg(&[0.0, 0.0, 0.0, 10.0, 0.0, 10.0]);
        assert!((mapped[0] - 0.0).abs() < 1e-9);
        assert!((mapped[1] - -89.95).abs() < 1e-9);
        assert!((mapped[3] - -10.0).abs() < 1e-9);
        assert!((mapped[5] - -10.0).abs() < 1e-9);
    }

    #[test]
    fn proportional_joint_and_clamping_match_sdk() {
        // Alicia-M-SDK JointMappingTest.test_proportional_joint_and_clamping
        let mapped = alicia_d_deg_to_alicia_m_deg(&[999.0, 0.0, 999.0, 0.0, 0.0, 0.0]);
        assert!((mapped[0] - 157.5).abs() < 1e-9);
        assert!((mapped[2] - -179.9).abs() < 1e-9);
    }

    #[test]
    fn rad_zero_maps_joint2_to_midpoint() {
        // Alicia-M-SDK JointMappingTest.test_rad_mapping
        let mapped = alicia_d_rad_to_alicia_m_rad(&[0.0; 6]);
        assert_eq!(mapped.len(), 6);
        assert!((mapped[1] - (-89.95_f64).to_radians()).abs() < 1e-9);
    }
}
