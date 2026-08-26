use std::collections::HashMap;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use motor_vendor_damiao::{ControlMode, DamiaoController, DamiaoMotor};

use crate::config::JointCfg;
use crate::error::{ArxError, Result};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GroupMode {
    Mit,
    PosVel,
}

const STATUS_ENABLED: u8 = 0x1;
const CTRL_MODE_MIT: u32 = ControlMode::Mit as u32;
const MODE_READ_TIMEOUT: Duration = Duration::from_millis(80);
const ENABLE_SETTLE: Duration = Duration::from_millis(50);

pub struct NoOpGroup;

impl NoOpGroup {
    pub fn new() -> Self {
        Self
    }

    pub fn num_joints(&self) -> usize {
        0
    }

    pub fn enable(&self) {}
    pub fn disable(&self) {}
    pub fn mode_mit(&mut self) -> bool {
        true
    }
    pub fn mode_pos_vel(&mut self) -> bool {
        true
    }
    pub fn send_mit(&self, _pos: &[f64], _vel: Option<&[f64]>, _tau_ff: Option<&[f64]>) {}
    pub fn send_pos_vel(&self, _pos: &[f64], _vlim: Option<&[f64]>) {}
}

pub struct JointGroup {
    pub name: String,
    joint_names: Vec<String>,
    joint_cfgs: Vec<JointCfg>,
    motors: HashMap<String, Arc<DamiaoMotor>>,
    controller: Arc<DamiaoController>,
    mode: GroupMode,
    mit_kp: Vec<f64>,
    mit_kd: Vec<f64>,
    pv_vlim: Vec<f64>,
}

impl JointGroup {
    pub fn new(
        name: impl Into<String>,
        joint_names: Vec<String>,
        all_joints: &[JointCfg],
        motor_map: &HashMap<String, Arc<DamiaoMotor>>,
        controller: Arc<DamiaoController>,
    ) -> Self {
        let joint_cfgs: Vec<JointCfg> = joint_names
            .iter()
            .map(|name| {
                all_joints
                    .iter()
                    .find(|joint| &joint.name == name)
                    .cloned()
                    .expect("joint must exist in config")
            })
            .collect();
        let mit_kp = joint_cfgs.iter().map(|j| j.kp).collect();
        let mit_kd = joint_cfgs.iter().map(|j| j.kd).collect();
        let pv_vlim = joint_cfgs.iter().map(|j| j.vlim).collect();
        let motors = joint_names
            .iter()
            .map(|name| (name.clone(), motor_map[name].clone()))
            .collect();

        Self {
            name: name.into(),
            joint_names,
            joint_cfgs,
            motors,
            controller,
            mode: GroupMode::Mit,
            mit_kp,
            mit_kd,
            pv_vlim,
        }
    }

    pub fn num_joints(&self) -> usize {
        self.joint_names.len()
    }

    fn motor_enabled(motor: &DamiaoMotor) -> bool {
        motor
            .latest_state()
            .map(|state| state.status_code == STATUS_ENABLED)
            .unwrap_or(false)
    }

    fn motor_mit_mode(motor: &DamiaoMotor) -> bool {
        motor
            .get_register_u32(10, MODE_READ_TIMEOUT)
            .ok()
            .is_some_and(|mode| mode == CTRL_MODE_MIT)
    }

    pub fn all_motors_enabled(&self) -> bool {
        self.joint_cfgs
            .iter()
            .all(|joint| Self::motor_enabled(&self.motors[&joint.name]))
    }

    pub fn all_motors_mit_mode(&self) -> bool {
        self.joint_cfgs
            .iter()
            .all(|joint| Self::motor_mit_mode(&self.motors[&joint.name]))
    }

    pub fn adopt_mode_mit(&mut self) {
        self.mode = GroupMode::Mit;
    }

    /// Enable only motors in this group that are not already enabled.
    pub fn enable_selective(&self) -> Result<()> {
        self.request_feedback();

        let needs_enable: Vec<&JointCfg> = self
            .joint_cfgs
            .iter()
            .filter(|joint| !Self::motor_enabled(&self.motors[&joint.name]))
            .collect();

        if needs_enable.is_empty() {
            return Ok(());
        }

        for joint in &needs_enable {
            let _ = self.motors[&joint.name].clear_error();
        }
        thread::sleep(Duration::from_millis(20));

        for joint in &needs_enable {
            self.motors[&joint.name].enable()?;
        }
        thread::sleep(ENABLE_SETTLE);
        self.request_feedback();

        let failed: Vec<String> = needs_enable
            .iter()
            .filter(|joint| !Self::motor_enabled(&self.motors[&joint.name]))
            .map(|joint| joint.name.clone())
            .collect();
        if failed.is_empty() {
            return Ok(());
        }

        Err(ArxError::Runtime(format!(
            "{} enable failed for: {}",
            self.name,
            failed.join(", ")
        )))
    }

    pub fn enable(&self) -> Result<()> {
        self.enable_selective()
    }

    pub fn disable(&self) {
        for joint in &self.joint_cfgs {
            let _ = self.motors[&joint.name].disable();
        }
        thread::sleep(Duration::from_millis(50));
    }

    fn write_pv_params(&self, joint: &JointCfg) {
        let motor = &self.motors[&joint.name];
        let _ = motor.write_register_f32(25, joint.vel_kp as f32);
        let _ = motor.write_register_f32(26, joint.vel_ki as f32);
        let _ = motor.write_register_f32(27, joint.pos_kp as f32);
        let _ = motor.write_register_f32(28, joint.pos_ki as f32);
        thread::sleep(Duration::from_millis(20));
    }

    /// Enter MIT mode only for joints that are not already in MIT.
    pub fn ensure_mit_mode(&mut self) -> Result<()> {
        for joint in &self.joint_cfgs {
            let motor = &self.motors[&joint.name];
            if Self::motor_mit_mode(motor) {
                continue;
            }
            let mut last_error = None;
            for _ in 0..3 {
                match motor.ensure_control_mode(ControlMode::Mit, Duration::from_secs(1)) {
                    Ok(()) => {
                        last_error = None;
                        break;
                    }
                    Err(error) => {
                        last_error = Some(error);
                        thread::sleep(Duration::from_millis(50));
                    }
                }
            }
            if let Some(error) = last_error {
                return Err(ArxError::Runtime(format!(
                    "{} failed to enter MIT mode after 3 attempts: {error}",
                    joint.name
                )));
            }
        }
        self.mode = GroupMode::Mit;
        Ok(())
    }

    pub fn mode_mit(&mut self) -> Result<()> {
        self.ensure_mit_mode()
    }

    pub fn mode_pos_vel(&mut self) -> Result<()> {
        for joint in &self.joint_cfgs {
            self.write_pv_params(joint);
            let motor = &self.motors[&joint.name];
            let mut last_error = None;
            for _ in 0..3 {
                match motor.ensure_control_mode(ControlMode::PosVel, Duration::from_secs(1)) {
                    Ok(()) => {
                        last_error = None;
                        break;
                    }
                    Err(error) => {
                        last_error = Some(error);
                        thread::sleep(Duration::from_millis(100));
                    }
                }
            }
            if let Some(error) = last_error {
                return Err(ArxError::Runtime(format!(
                    "{} failed to enter POS_VEL mode after 3 attempts: {error}",
                    joint.name
                )));
            }
            thread::sleep(Duration::from_millis(50));
        }
        self.mode = GroupMode::PosVel;
        thread::sleep(Duration::from_millis(100));
        Ok(())
    }

    pub fn send_mit(
        &self,
        pos: &[f64],
        vel: Option<&[f64]>,
        tau_ff: Option<&[f64]>,
    ) -> Result<()> {
        for (i, joint) in self.joint_cfgs.iter().enumerate() {
            let motor = &self.motors[&joint.name];
            let p = pos.get(i).copied().unwrap_or(0.0) as f32;
            let v = vel.and_then(|v| v.get(i)).copied().unwrap_or(0.0) as f32;
            let kp = self.mit_kp.get(i).copied().unwrap_or(joint.kp) as f32;
            let kd = self.mit_kd.get(i).copied().unwrap_or(joint.kd) as f32;
            let tau = tau_ff.and_then(|t| t.get(i)).copied().unwrap_or(0.0) as f32;
            motor.send_cmd_mit(p, v, kp, kd, tau)?;
        }
        Ok(())
    }

    pub fn mit_gains(&self) -> (Vec<f64>, Vec<f64>) {
        (self.mit_kp.clone(), self.mit_kd.clone())
    }

    pub fn set_mit_gains(&mut self, kp: Option<&[f64]>, kd: Option<&[f64]>) -> Result<()> {
        if let Some(values) = kp {
            if values.len() != self.joint_cfgs.len() {
                return Err(ArxError::InvalidArgument(format!(
                    "expected {} kp values, got {}",
                    self.joint_cfgs.len(),
                    values.len()
                )));
            }
            if values.iter().any(|v| !v.is_finite() || *v < 0.0) {
                return Err(ArxError::InvalidArgument(
                    "kp values must be finite and non-negative".into(),
                ));
            }
            self.mit_kp = values.to_vec();
        }
        if let Some(values) = kd {
            if values.len() != self.joint_cfgs.len() {
                return Err(ArxError::InvalidArgument(format!(
                    "expected {} kd values, got {}",
                    self.joint_cfgs.len(),
                    values.len()
                )));
            }
            if values.iter().any(|v| !v.is_finite() || *v < 0.0) {
                return Err(ArxError::InvalidArgument(
                    "kd values must be finite and non-negative".into(),
                ));
            }
            self.mit_kd = values.to_vec();
        }
        Ok(())
    }

    pub fn send_pos_vel(&self, pos: &[f64], vlim: Option<&[f64]>) {
        let vlim_values: Vec<f64> = match vlim {
            Some(values) => values.to_vec(),
            None => self.pv_vlim.clone(),
        };
        for (i, joint) in self.joint_cfgs.iter().enumerate() {
            let motor = &self.motors[&joint.name];
            let p = pos.get(i).copied().unwrap_or(0.0) as f32;
            let v = vlim_values.get(i).copied().unwrap_or(joint.vlim) as f32;
            let _ = motor.send_cmd_pos_vel(p, v);
        }
    }

    pub fn request_feedback(&self) {
        for joint in &self.joint_cfgs {
            let _ = self.motors[&joint.name].request_motor_feedback();
        }
        let _ = self.controller.poll_feedback_once();
    }
}

pub enum GroupKind {
    Active(JointGroup),
    NoOp(NoOpGroup),
}

impl GroupKind {
    pub fn enable(&self) {
        match self {
            GroupKind::Active(group) => {
                let _ = group.enable_selective();
            }
            GroupKind::NoOp(group) => group.enable(),
        }
    }

    pub fn disable(&self) {
        match self {
            GroupKind::Active(group) => group.disable(),
            GroupKind::NoOp(group) => group.disable(),
        }
    }
}
