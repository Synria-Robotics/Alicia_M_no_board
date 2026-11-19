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
}

impl Default for GripperController {
    fn default() -> Self {
        Self {
            contact_torque_nm: 0.25,
            hold_torque_nm: 0.8,
            kp_close: 2.0,
            kd_close: 0.1,
            kp_hold: 2.0,
            kd_hold: 0.1,
            thermal_limit_c: 50.0,
            contact_position: None,
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
        diags
            .iter()
            .map(|d| d.mos_temperature_c.max(d.rotor_temperature_c))
            .fold(f64::NAN, f64::max)
    }

    pub fn gripper_diagnostic(diags: &[MotorDiagnostic]) -> Option<&MotorDiagnostic> {
        diags.iter().find(|d| d.motor_id == 7)
    }
}
