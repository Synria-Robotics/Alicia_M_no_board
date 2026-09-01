//! MIT reference governor: limits commanded position error so Kp tracking stays within torque budget.

use motor_vendor_damiao::model_limits;

use crate::config::JointMotorConfig;

/// Fraction of rated torque reserved for Kp position tracking (rest for feedforward / margin).
pub const DEFAULT_TORQUE_MARGIN: f64 = 0.8;

#[derive(Debug, Clone)]
pub struct ReferenceGovernor {
    kp: Vec<f64>,
    tmax: Vec<f64>,
    torque_margin: f64,
}

impl ReferenceGovernor {
    pub fn from_joint_configs(joint_cfgs: &[JointMotorConfig], torque_margin: f64) -> Self {
        let margin = torque_margin.clamp(0.0, 1.0);
        let kp: Vec<f64> = joint_cfgs.iter().map(|joint| joint.mit_kp).collect();
        let tmax: Vec<f64> = joint_cfgs
            .iter()
            .map(|joint| {
                model_limits(&joint.model)
                    .map(|(_, _, t)| f64::from(t))
                    .unwrap_or(10.0)
            })
            .collect();
        Self {
            kp,
            tmax,
            torque_margin: margin,
        }
    }

    #[allow(clippy::unused_self)]
    pub fn reset(&mut self) {}

    /// Clamp `q` near measured joints so `Kp * (q_send - q_meas) + tau_ff` stays within `tmax`.
    ///
    /// Returns the caller's `dq_ideal` unchanged (walker / Ruckig owns velocity).
    pub fn apply(
        &self,
        q_ideal: &[f64],
        dq_ideal: &[f64],
        q_meas: &[f64],
        tau_ff: Option<&[f64]>,
        _dt: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let n = q_ideal.len();
        let mut q_send = vec![0.0; n];
        for i in 0..n {
            let qm = q_meas.get(i).copied().unwrap_or(q_ideal[i]);
            let ff = tau_ff
                .and_then(|tau| tau.get(i))
                .copied()
                .unwrap_or(0.0)
                .abs();
            let kp = self.kp.get(i).copied().unwrap_or(0.0);
            let tmax = self.tmax.get(i).copied().unwrap_or(10.0);
            let tau_for_kp = (tmax - ff).max(0.0) * self.torque_margin;
            let eps_max = if kp > 1e-6 {
                tau_for_kp / kp
            } else {
                f64::INFINITY
            };
            let err = q_ideal[i] - qm;
            let err_clamped = err.clamp(-eps_max, eps_max);
            q_send[i] = qm + err_clamped;
        }
        (q_send, dq_ideal.to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::JointMotorConfig;

    fn test_joint(model: &str, kp: f64) -> JointMotorConfig {
        JointMotorConfig {
            name: "j".into(),
            motor_id: 1,
            feedback_id: 1,
            model: model.into(),
            mit_kp: kp,
            mit_kd: 1.0,
            pv_vel_kp: 0.0,
            pv_vel_ki: 0.0,
            pv_pos_kp: 0.0,
            pv_pos_ki: 0.0,
            pv_vlim: 2.0,
        }
    }

    #[test]
    fn eps_max_from_tmax_and_kp() {
        let gov = ReferenceGovernor::from_joint_configs(&[test_joint("4340P", 120.0)], 0.8);
        let q_meas = vec![0.0];
        let q_ideal = vec![1.0];
        let (q_send, _) = gov.apply(&q_ideal, &[0.0], &q_meas, None, 0.002);
        // 4340P tmax=28, margin=0.8 -> eps_max = 0.8*28/120 ≈ 0.187
        assert!((q_send[0] - 0.186_666).abs() < 1e-3);
    }

    #[test]
    fn no_clamp_when_error_small() {
        let gov = ReferenceGovernor::from_joint_configs(&[test_joint("4310", 36.0)], 0.8);
        let q_meas = vec![0.0];
        let q_ideal = vec![0.05];
        let (q_send, _) = gov.apply(&q_ideal, &[0.1], &q_meas, None, 0.002);
        assert!((q_send[0] - 0.05).abs() < 1e-9);
    }

    #[test]
    fn dq_ideal_passed_through_when_q_clamped() {
        let gov = ReferenceGovernor::from_joint_configs(&[test_joint("4340P", 120.0)], 0.8);
        let q_meas = vec![0.0];
        let q_ideal = vec![1.0];
        let (_, dq) = gov.apply(&q_ideal, &[0.3], &q_meas, None, 0.002);
        assert!((dq[0] - 0.3).abs() < 1e-12);
    }

    #[test]
    fn feedforward_reduces_tracking_budget() {
        let gov = ReferenceGovernor::from_joint_configs(&[test_joint("4340P", 120.0)], 1.0);
        let q_meas = vec![0.0];
        let q_ideal = vec![1.0];
        let (q_send, _) = gov.apply(&q_ideal, &[0.0], &q_meas, Some(&[7.0]), 0.002);
        // eps_max = (28 - 7) / 120 = 0.175
        assert!((q_send[0] - 0.175).abs() < 1e-3);
    }
}
