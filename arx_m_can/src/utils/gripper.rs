pub const GRIPPER_MIN: f64 = 0.0;
pub const GRIPPER_MAX: f64 = 1000.0;

pub fn validate_gripper_value(value: f64) -> f64 {
    value.clamp(GRIPPER_MIN, GRIPPER_MAX)
}

pub fn user_to_motor(value: f64, closed_motor: f64, open_motor: f64) -> f64 {
    let user = validate_gripper_value(value);
    let ratio = user / GRIPPER_MAX;
    closed_motor + (open_motor - closed_motor) * ratio
}

pub fn motor_to_user(motor: f64, closed_motor: f64, open_motor: f64) -> f64 {
    if (open_motor - closed_motor).abs() < f64::EPSILON {
        return GRIPPER_MIN;
    }
    let ratio = (motor - closed_motor) / (open_motor - closed_motor);
    validate_gripper_value(ratio * GRIPPER_MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gripper_roundtrip() {
        let closed = 0.0;
        let open = 3.0;
        let motor = user_to_motor(500.0, closed, open);
        assert!((motor_to_user(motor, closed, open) - 500.0).abs() < 1e-6);
    }
}
