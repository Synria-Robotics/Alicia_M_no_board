use crate::config::{default_config, ArxMCanConfig};
use crate::error::{ArxError, Result};
use crate::hardware::bus::ArxMCan;
use crate::scan::{scan_ids as run_scan, ScanOptions};
use crate::types::{ArxMCanState, JointState, MotorDiagnostic, MotorState, VlimInput};
use crate::utils::gripper::{motor_to_user, user_to_motor};

pub struct ArxMCanArm {
    config: ArxMCanConfig,
    robot: Option<ArxMCan>,
    enable_gripper: bool,
    connected: bool,
    enabled: bool,
    mode: String,
}

fn normalize_mode(mode: &str) -> String {
    mode.trim().to_ascii_lowercase().replace('_', "")
}

impl ArxMCanArm {
    pub fn new(config: ArxMCanConfig, enable_gripper: bool) -> Result<Self> {
        Ok(Self {
            mode: normalize_mode(&config.arm_control_mode),
            config,
            robot: None,
            enable_gripper,
            connected: false,
            enabled: false,
        })
    }

    pub fn with_defaults(
        port: Option<&str>,
        baud: Option<u32>,
        control_mode: &str,
        enable_gripper: bool,
    ) -> Result<Self> {
        Self::new(
            default_config(port, baud, 100.0, control_mode, None)?,
            enable_gripper,
        )
    }

    pub fn config(&self) -> &ArxMCanConfig {
        &self.config
    }

    pub fn control_mode(&self) -> &str {
        &self.mode
    }

    pub fn is_mit_mode(&self) -> bool {
        normalize_mode(&self.mode) == "mit"
    }

    pub fn joint_names(&self) -> Vec<String> {
        self.config.joint_names()
    }

    pub fn connected(&self) -> bool {
        self.connected
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn connect(&mut self) -> Result<()> {
        let mut robot = ArxMCan::new(
            Some(&self.config.port),
            Some(self.config.baud),
            self.config.hw_yaml.as_deref(),
        )?;
        robot.connect()?;
        self.robot = Some(robot);
        self.connected = true;
        Ok(())
    }

    pub fn configure(&mut self, mode: Option<&str>) -> Result<()> {
        self.prepare_for_control(mode, false)
    }

    /// Connect, optionally preserving an already-enabled MIT hardware state.
    pub fn prepare_for_control(
        &mut self,
        mode: Option<&str>,
        preserve_hardware: bool,
    ) -> Result<()> {
        self.require_connected()?;
        let mode_str = mode
            .map(normalize_mode)
            .unwrap_or_else(|| self.mode.clone());
        let robot = self.robot.as_mut().ok_or(ArxError::NotConnected)?;
        robot.request_all_feedback()?;

        if preserve_hardware && normalize_mode(&mode_str) == "mit" {
            self.mode = "mit".to_string();
            robot.arm_mut()?.adopt_mode_mit();
            if self.enable_gripper && self.config.gripper.is_some() {
                if let Ok(gripper) = robot.gripper_mut() {
                    gripper.adopt_mode_mit();
                }
            }
            return Ok(());
        }

        self.configure_mode(&mode_str)?;
        if self.enable_gripper && self.config.gripper.is_some() {
            let robot = self.robot.as_mut().ok_or(ArxError::NotConnected)?;
            robot.gripper_mut()?.ensure_mit_mode()?;
        }
        Ok(())
    }

    pub fn detect_hardware_preserve(&self, mode: &str) -> Result<bool> {
        if normalize_mode(mode) != "mit" {
            return Ok(false);
        }
        let robot = self.robot.as_ref().ok_or(ArxError::NotConnected)?;
        robot.arm_hardware_ready(true)
    }

    pub fn enable_if_needed(&mut self) -> Result<()> {
        self.require_connected()?;
        let robot = self.robot.as_ref().ok_or(ArxError::NotConnected)?;
        robot.enable_selective()?;
        self.enabled = robot.arm()?.all_motors_enabled();
        Ok(())
    }

    pub fn hardware_all_enabled(&self) -> Result<bool> {
        Ok(self
            .robot
            .as_ref()
            .ok_or(ArxError::NotConnected)?
            .arm()?
            .all_motors_enabled())
    }

    pub fn close(&mut self, disable: bool) -> Result<()> {
        if disable && self.enabled {
            self.disable()?;
        }
        if let Some(robot) = self.robot.as_mut() {
            robot.disconnect()?;
        }
        self.robot = None;
        self.connected = false;
        self.enabled = false;
        Ok(())
    }

    pub fn enable(&mut self) -> Result<()> {
        self.require_connected()?;
        let robot = self.robot.as_mut().ok_or(ArxError::NotConnected)?;
        robot.enable_selective()?;
        self.enabled = robot.arm()?.all_motors_enabled();
        Ok(())
    }

    pub fn disable(&mut self) -> Result<()> {
        self.require_connected()?;
        let robot = self.robot.as_mut().ok_or(ArxError::NotConnected)?;
        robot.arm_mut()?.disable();
        if self.enable_gripper && self.config.gripper.is_some() {
            robot.gripper_mut()?.disable();
        }
        self.enabled = false;
        Ok(())
    }

    pub fn adopt_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    pub fn toggle_enable(&mut self) -> Result<bool> {
        if self.enabled {
            self.disable()?;
        } else {
            self.enable()?;
        }
        Ok(self.enabled)
    }

    pub fn configure_mode(&mut self, mode: &str) -> Result<()> {
        self.require_connected()?;
        let normalized = normalize_mode(mode);
        let robot = self.robot.as_mut().ok_or(ArxError::NotConnected)?;
        match normalized.as_str() {
            "posvel" | "pv" => {
                robot.arm_mut()?.mode_pos_vel()?;
                self.mode = "posvel".to_string();
            }
            "mit" => {
                robot.arm_mut()?.ensure_mit_mode()?;
                self.mode = "mit".to_string();
            }
            _ => {
                return Err(ArxError::InvalidArgument(
                    "mode must be 'posvel' or 'mit'".into(),
                ));
            }
        }
        Ok(())
    }

    pub fn read_state(&self, request_feedback: bool) -> Result<ArxMCanState> {
        self.require_connected()?;
        let robot = self.robot.as_ref().ok_or(ArxError::NotConnected)?;
        let (pos, vel, tau) = robot.get_state(request_feedback)?;
        let names = robot.joint_names();
        let take = |name: &str, values: &[f64]| -> f64 {
            names
                .iter()
                .position(|n| n == name)
                .and_then(|i| values.get(i).copied())
                .unwrap_or(f64::NAN)
        };
        let arm_names = self.config.joint_names();
        let gripper_state = self.config.gripper.as_ref().and_then(|gripper| {
            let i = names.iter().position(|n| n == &gripper.name)?;
            let position = pos.get(i).copied().filter(|p| p.is_finite())?;
            Some(MotorState {
                name: gripper.name.clone(),
                motor_id: gripper.motor_id,
                feedback_id: gripper.feedback_id,
                position,
                velocity: vel.get(i).copied().unwrap_or(0.0),
                torque: tau.get(i).copied().unwrap_or(0.0),
            })
        });
        Ok(ArxMCanState {
            arm: JointState {
                names: arm_names.clone(),
                positions: arm_names.iter().map(|n| take(n, &pos)).collect(),
                velocities: arm_names.iter().map(|n| take(n, &vel)).collect(),
                torques: arm_names.iter().map(|n| take(n, &tau)).collect(),
            },
            gripper: gripper_state,
        })
    }

    pub fn motor_diagnostics(&self, request_feedback: bool) -> Result<Vec<MotorDiagnostic>> {
        self.require_connected()?;
        self.robot
            .as_ref()
            .ok_or(ArxError::NotConnected)?
            .get_motor_diagnostics(request_feedback)
    }

    pub fn send_joint_positions(
        &mut self,
        positions: &[f64],
        mode: Option<&str>,
        vlim_rad_s: Option<VlimInput>,
        require_enabled: bool,
    ) -> Result<()> {
        self.require_connected()?;
        if require_enabled && !self.enabled {
            return Err(ArxError::NotEnabled);
        }
        if positions.len() != self.config.arm_joints.len() {
            return Err(ArxError::InvalidArgument(format!(
                "expected {} joint positions, got {}",
                self.config.arm_joints.len(),
                positions.len()
            )));
        }
        if positions.iter().any(|value| !value.is_finite()) {
            return Err(ArxError::InvalidArgument(
                "joint positions must be finite".into(),
            ));
        }

        let active_mode = normalize_mode(mode.unwrap_or(&self.mode));
        let is_posvel = matches!(active_mode.as_str(), "posvel" | "pv");
        let is_mit = active_mode == "mit";

        if is_posvel && self.mode != "posvel" {
            self.configure_mode("posvel")?;
        } else if is_mit && self.mode != "mit" {
            self.configure_mode("mit")?;
        }

        let vlim = if is_posvel {
            self.resolve_vlim_array(vlim_rad_s)?
        } else {
            None
        };

        let robot = self.robot.as_mut().ok_or(ArxError::NotConnected)?;
        if is_posvel {
            robot.arm_mut()?.send_pos_vel(positions, vlim.as_deref());
        } else if is_mit {
            robot.arm_mut()?.send_mit(positions, None, None)?;
        } else {
            return Err(ArxError::InvalidArgument(
                "mode must be 'posvel' or 'mit'".into(),
            ));
        }
        Ok(())
    }

    /// Send arm command using the configured control mode.
    pub fn send_arm_command(
        &mut self,
        positions: &[f64],
        velocities: Option<&[f64]>,
        vlim_rad_s: Option<VlimInput>,
        require_enabled: bool,
        tau_ff: Option<&[f64]>,
    ) -> Result<()> {
        self.require_connected()?;
        if require_enabled && !self.enabled {
            return Err(ArxError::NotEnabled);
        }
        if positions.len() != self.config.arm_joints.len() {
            return Err(ArxError::InvalidArgument(format!(
                "expected {} joint positions, got {}",
                self.config.arm_joints.len(),
                positions.len()
            )));
        }
        if positions.iter().any(|value| !value.is_finite()) {
            return Err(ArxError::InvalidArgument(
                "joint positions must be finite".into(),
            ));
        }
        if let Some(vel) = velocities {
            if vel.len() != positions.len() || vel.iter().any(|v| !v.is_finite()) {
                return Err(ArxError::InvalidArgument(
                    "joint velocities must match positions and be finite".into(),
                ));
            }
        }
        if let Some(tau) = tau_ff {
            if tau.len() != positions.len() || tau.iter().any(|t| !t.is_finite()) {
                return Err(ArxError::InvalidArgument(
                    "joint feedforward torques must match positions and be finite".into(),
                ));
            }
        }

        if self.is_mit_mode() {
            self.send_joint_mit(positions, velocities, tau_ff)
        } else {
            self.send_joint_positions(positions, None, vlim_rad_s, false)
        }
    }

    pub fn send_joint_mit(
        &mut self,
        positions: &[f64],
        velocities: Option<&[f64]>,
        tau_ff: Option<&[f64]>,
    ) -> Result<()> {
        self.require_connected()?;
        if self.mode != "mit" {
            self.configure_mode("mit")?;
        }
        let robot = self.robot.as_mut().ok_or(ArxError::NotConnected)?;
        robot.arm_mut()?.send_mit(positions, velocities, tau_ff)?;
        Ok(())
    }

    pub fn get_arm_mit_gains(&self) -> Result<(Vec<f64>, Vec<f64>)> {
        self.require_connected()?;
        let robot = self.robot.as_ref().ok_or(ArxError::NotConnected)?;
        Ok(robot.arm()?.mit_gains())
    }

    pub fn set_arm_mit_gains(&mut self, kp: Option<&[f64]>, kd: Option<&[f64]>) -> Result<()> {
        self.require_connected()?;
        let robot = self.robot.as_mut().ok_or(ArxError::NotConnected)?;
        robot.arm_mut()?.set_mit_gains(kp, kd)
    }

    pub fn scan_ids(
        &self,
        start_id: u16,
        end_id: u16,
        model: &str,
        timeout_ms: u32,
        feedback_base: &str,
    ) -> Result<Vec<u16>> {
        run_scan(
            &self.config.port,
            self.config.baud,
            &ScanOptions {
                start_id,
                end_id,
                model: model.to_string(),
                timeout_ms,
                feedback_base: feedback_base.to_string(),
            },
        )
    }

    pub fn set_zero(&mut self) -> Result<()> {
        self.require_connected()?;
        self.robot
            .as_ref()
            .ok_or(ArxError::NotConnected)?
            .set_zero()
    }

    pub fn set_gripper_mit(
        &mut self,
        value: f64,
        velocity: f64,
        kp: f64,
        kd: f64,
        tau_ff: f64,
        require_enabled: bool,
    ) -> Result<()> {
        self.require_connected()?;
        if require_enabled && !self.enabled {
            return Err(ArxError::NotEnabled);
        }
        let target = user_to_motor(
            value,
            self.config.gripper_closed_value,
            self.config.gripper_open_value,
        );
        let robot = self.robot.as_mut().ok_or(ArxError::NotConnected)?;
        robot
            .gripper_mut()?
            .send_mit_with_gains(target, velocity, kp, kd, tau_ff)
    }

    pub fn get_gripper_value(&self, request_feedback: bool) -> Result<Option<f64>> {
        self.require_connected()?;
        let gripper = match &self.config.gripper {
            Some(gripper) => gripper,
            None => return Ok(None),
        };
        let robot = self.robot.as_ref().ok_or(ArxError::NotConnected)?;
        let Some(position) = robot.read_joint_position(&gripper.name, request_feedback)? else {
            return Ok(None);
        };
        Ok(Some(motor_to_user(
            position,
            self.config.gripper_closed_value,
            self.config.gripper_open_value,
        )))
    }

    pub fn set_gripper_motor_value(
        &mut self,
        value: f64,
        vel: f64,
        require_enabled: bool,
    ) -> Result<()> {
        self.require_connected()?;
        if self.config.gripper.is_none() {
            return Ok(());
        }
        if !self.enable_gripper {
            return Err(ArxError::Runtime(
                "gripper disabled; pass enable_gripper=true when creating the arm".into(),
            ));
        }
        if require_enabled && !self.enabled {
            return Err(ArxError::NotEnabled);
        }
        self.robot
            .as_mut()
            .ok_or(ArxError::NotConnected)?
            .gripper_mut()?
            .send_mit(&[value], Some(&[vel]), None)?;
        Ok(())
    }

    pub fn get_gripper_button_event(&self) -> Result<Option<u8>> {
        self.require_connected()?;
        self.robot
            .as_ref()
            .ok_or(ArxError::NotConnected)?
            .poll_gripper_button_event()
    }

    fn resolve_vlim_array(&self, vlim_rad_s: Option<VlimInput>) -> Result<Option<Vec<f64>>> {
        let Some(vlim_rad_s) = vlim_rad_s else {
            return Ok(None);
        };
        let limits: Vec<f64> = self
            .config
            .arm_joints
            .iter()
            .map(|joint| joint.pv_vlim)
            .collect();
        let arr = match vlim_rad_s {
            VlimInput::Scalar(value) => vec![value; self.config.arm_joints.len()],
            VlimInput::PerJoint(values) => {
                if values.len() != self.config.arm_joints.len() {
                    return Err(ArxError::InvalidArgument(format!(
                        "expected {} vlim values, got {}",
                        self.config.arm_joints.len(),
                        values.len()
                    )));
                }
                values
            }
        };
        Ok(Some(
            arr.into_iter()
                .zip(limits.iter())
                .map(|(value, limit)| value.clamp(0.0, *limit))
                .collect(),
        ))
    }

    fn require_connected(&self) -> Result<()> {
        if !self.connected {
            return Err(ArxError::NotConnected);
        }
        Ok(())
    }
}

impl Drop for ArxMCanArm {
    fn drop(&mut self) {
        let _ = self.close(false);
    }
}
