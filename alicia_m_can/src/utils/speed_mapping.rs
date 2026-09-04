pub const ALICIA_SPEED_MAX: f64 = 400.0;
const MIN_VLIM_RAD_S: f64 = 0.01;

pub fn speed_to_vlim_rad_s(speed: f64, max_vlim_rad_s: f64) -> f64 {
    let cap = max_vlim_rad_s.max(0.0);
    let s = speed.clamp(0.0, ALICIA_SPEED_MAX);
    if s <= 0.0 || cap <= 0.0 {
        return 0.0;
    }
    (cap * (s / ALICIA_SPEED_MAX)).max(MIN_VLIM_RAD_S)
}

pub fn speed_to_gripper_vlim_rad_s(speed: f64, max_vlim_rad_s: f64) -> f64 {
    speed_to_vlim_rad_s(speed, max_vlim_rad_s)
}
