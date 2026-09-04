//! Rigid-body gravity compensation via RoboCore native dynamics FFI.

use std::path::Path;

use robocore_dynamics_ffi::{ChainDynamics, ChainDynamicsModel, DynamicsError};

use crate::error::{ArxError, Result};

/// Per-joint gravity torque scale (matches arx_d_can_ref joint2/3 tuning).
#[derive(Debug, Clone)]
pub struct GravityCompConfig {
    pub enabled: bool,
    pub model_path: Option<String>,
    pub joint_scales: [f64; 6],
    pub gripper_q_default: f64,
}

impl Default for GravityCompConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            model_path: None,
            joint_scales: [1.0, 1.55, 1.55, 1.0, 1.0, 1.0],
            gripper_q_default: 0.0,
        }
    }
}

pub struct GravityCompensator {
    dynamics: ChainDynamics,
    model_dof: usize,
    arm_dof: usize,
    joint_scales: [f64; 6],
    gripper_q_default: f64,
    q_buf: Vec<f64>,
    tau_buf: Vec<f64>,
}

impl GravityCompensator {
    pub fn from_config(config: &GravityCompConfig, arm_dof: usize) -> Result<Self> {
        if !config.enabled {
            return Err(ArxError::InvalidArgument(
                "gravity compensation disabled in config".into(),
            ));
        }
        let model = load_model(config.model_path.as_deref())?;
        let dynamics = ChainDynamics::from_model(&model).map_err(map_dynamics_err)?;
        let model_dof = dynamics.n_dof();
        Ok(Self {
            dynamics,
            model_dof,
            arm_dof,
            joint_scales: config.joint_scales,
            gripper_q_default: config.gripper_q_default,
            q_buf: vec![0.0; model_dof],
            tau_buf: vec![0.0; model_dof],
        })
    }

    pub fn model_dof(&self) -> usize {
        self.model_dof
    }

    /// Compute arm gravity feedforward torques from measured joint positions.
    pub fn compute_arm_gravity_ff(
        &mut self,
        arm_q: &[f64],
        gripper_q: Option<f64>,
    ) -> Result<Vec<f64>> {
        if arm_q.len() < self.arm_dof {
            return Err(ArxError::InvalidArgument(format!(
                "expected at least {} arm joints, got {}",
                self.arm_dof,
                arm_q.len()
            )));
        }
        for (i, value) in arm_q.iter().take(self.arm_dof).enumerate() {
            self.q_buf[i] = *value;
        }
        if self.model_dof > self.arm_dof {
            let gq = gripper_q.unwrap_or(self.gripper_q_default);
            for value in self.q_buf.iter_mut().skip(self.arm_dof) {
                *value = gq;
            }
        }
        self.dynamics
            .gravity(&self.q_buf, &mut self.tau_buf)
            .map_err(map_dynamics_err)?;
        let mut out = vec![0.0; self.arm_dof];
        for (i, tau) in out.iter_mut().enumerate().take(self.arm_dof) {
            let scale = self.joint_scales.get(i).copied().unwrap_or(1.0);
            *tau = self.tau_buf[i] * scale;
        }
        Ok(out)
    }
}

fn load_model(path: Option<&str>) -> Result<ChainDynamicsModel> {
    if let Some(path) = path {
        return ChainDynamicsModel::from_json_file(path).map_err(map_dynamics_err);
    }
    const EMBEDDED: &str = include_str!("../../models/alicia_m_px6d_dynamics.json");
    ChainDynamicsModel::from_json_str(EMBEDDED).map_err(map_dynamics_err)
}

fn map_dynamics_err(err: DynamicsError) -> ArxError {
    ArxError::Runtime(err.to_string())
}

pub fn default_model_path() -> &'static Path {
    Path::new("models/alicia_m_px6d_dynamics.json")
}
