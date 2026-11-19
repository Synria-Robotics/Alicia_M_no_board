use crate::error::{ArxError, Result};
use crate::types::MotorDiagnostic;

/// Result of a guarded close operation.
#[derive(Debug, Clone)]
pub struct GraspResult {
    pub success: bool,
    pub contact_position: f64,
    pub contact_torque_nm: f64,
    pub hold_torque_nm: f64,
    pub peak_temperature_c: f64,
    pub reason: Option<String>,
}

/// Configuration and state for the one supported business-level close path.
/// The actual bus commands remain in `hardware::arm`; this type owns policy.
#[derive(Debug, Clone)]
pub struct GripperController {
    pub contact_torque_nm: f64,
    pub hold_torque_nm: f64,
    pub kp_close: f64,
    pub kd_close: f64,
    pub kp_hold: f64,
    pub kd_hold: f64,
    pub thermal_limit_c: f64,
    pub contact_position: Option<f64>,
    pub hold_margin: f64,
}

impl Default for GripperController {
    fn default() -> Self {
        Self {
            contact_torque_nm: 0.25,
            hold_torque_nm: 0.8,
            kp_close: 4.0,
            kd_close: 0.5,
            kp_hold: 4.0,
            kd_hold: 0.5,
            thermal_limit_c: 50.0,
            contact_position: None,
            hold_margin: 60.0,
        }
    }
}

impl GripperController {
    pub fn validate(&self) -> Result<()> {
        if self.contact_torque_nm <= 0.0 || self.hold_torque_nm <= 0.0 {
            return Err(ArxError::InvalidArgument(
                "gripper torque limits must be positive".into(),
            ));
        }
        if self.thermal_limit_c <= 0.0 {
            return Err(ArxError::InvalidArgument(
                "gripper thermal limit must be positive".into(),
            ));
        }
        Ok(())
    }

    pub fn temperature(diags: &[MotorDiagnostic]) -> f64 {
        Self::gripper_diagnostic(diags)
            .map(|d| d.mos_temperature_c.max(d.rotor_temperature_c))
            .unwrap_or(f64::NAN)
    }

    pub fn gripper_diagnostic(diags: &[MotorDiagnostic]) -> Option<&MotorDiagnostic> {
        diags.iter().find(|d| d.motor_id == 7)
    }

    pub fn begin_hold(&mut self, contact_position: f64) {
        self.contact_position = Some(contact_position);
        self.hold_margin = 60.0;
    }

    pub fn hold_target(&mut self, torque_nm: f64) -> f64 {
        self.hold_margin =
            (self.hold_margin + 10.0 * (self.hold_torque_nm - torque_nm)).clamp(0.0, 300.0);
        (self.contact_position.unwrap_or(0.0) - self.hold_margin).max(0.0)
    }

    pub fn clear_hold(&mut self) {
        self.contact_position = None;
        self.hold_margin = 60.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn diagnostic(motor_id: u16, temperature: f64) -> MotorDiagnostic {
        MotorDiagnostic {
            name: String::new(),
            motor_id,
            feedback_id: 0,
            status_code: 0,
            status_name: String::new(),
            position: 0.0,
            velocity: 0.0,
            torque: 0.0,
            mos_temperature_c: temperature,
            rotor_temperature_c: temperature,
        }
    }

    #[test]
    fn temperature_only_uses_gripper_motor() {
        let diags = vec![diagnostic(1, 80.0), diagnostic(7, 34.0)];
        assert_eq!(GripperController::temperature(&diags), 34.0);
    }

    #[test]
    fn hold_margin_adapts_without_resetting() {
        let mut controller = GripperController::default();
        controller.begin_hold(400.0);
        assert_eq!(controller.hold_target(0.2), 334.0);
        assert_eq!(controller.hold_target(0.2), 328.0);
    }
}
