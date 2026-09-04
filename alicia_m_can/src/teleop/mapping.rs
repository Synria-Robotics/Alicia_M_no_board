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
    fn maps_six_joints() {
        let mapped = alicia_d_rad_to_alicia_m_rad(&[0.0; 6]);
        assert_eq!(mapped.len(), 6);
    }
}
