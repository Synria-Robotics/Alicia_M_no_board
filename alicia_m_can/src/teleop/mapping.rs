//! Alicia-D leader joints are copied onto Alicia-M and clipped to the M limits.
//!
//! Joints 3, 4, and 6 (indexes 2, 3, and 5) are sign-flipped. The other joints
//! are unchanged. Every output is then clipped to the Alicia-M limits.

/// Alicia-M joint limits in degrees, same bounds the previous mapper clipped to.
const M_LIMITS_DEG: [(f64, f64); 6] = [
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

/// Joints 3, 4, and 6. Indexes are zero-based.
const REVERSED_JOINTS: [usize; 3] = [2, 3, 5];

pub fn alicia_d_rad_to_alicia_m_rad(joints_rad: &[f64; 6]) -> [f64; 6] {
    let mut out = [0.0; 6];
    for i in 0..6 {
        let (lo, hi) = M_LIMITS_DEG[i];
        let q = if REVERSED_JOINTS.contains(&i) {
            -joints_rad[i]
        } else {
            joints_rad[i]
        };
        out[i] = clamp(q, lo.to_radians(), hi.to_radians());
    }
    out
}

pub fn map_gripper(value: f64, scale: f64) -> f64 {
    (value * scale).clamp(0.0, 1000.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joints_3_4_6_are_reversed_and_the_rest_pass_through() {
        let leader = [0.13, -1.09, 0.56, -0.08, -0.82, 0.2];
        let mapped = alicia_d_rad_to_alicia_m_rad(&leader);
        assert!((mapped[0] - 0.13).abs() < 1e-12);
        assert!((mapped[1] - -1.09).abs() < 1e-12);
        assert!((mapped[2] - -0.56).abs() < 1e-12);
        assert!((mapped[3] - 0.08).abs() < 1e-12);
        assert!((mapped[4] - -0.82).abs() < 1e-12);
        assert!((mapped[5] - -0.2).abs() < 1e-12);
    }

    #[test]
    fn reversed_joints_clamp_after_the_sign_flip() {
        let hi = 157.5_f64.to_radians();
        let mapped = alicia_d_rad_to_alicia_m_rad(&[999.0, 0.5, -1.0, 0.0, 0.0, 999.0]);
        assert!((mapped[0] - hi).abs() < 1e-12);
        assert!((mapped[1] - 0.0).abs() < 1e-12);
        assert!((mapped[2] - 0.0).abs() < 1e-12);
        assert!((mapped[5] - -hi).abs() < 1e-12);
    }

    #[test]
    fn zero_stays_zero() {
        let mapped = alicia_d_rad_to_alicia_m_rad(&[0.0; 6]);
        assert!(mapped.iter().all(|q| q.abs() < 1e-12));
    }
}
