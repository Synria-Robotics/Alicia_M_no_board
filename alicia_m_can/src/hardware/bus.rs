use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use motor_vendor_damiao::{DamiaoController, DamiaoMotor};

use super::joint_group::{GroupKind, JointGroup, NoOpGroup};
use crate::config::{load_cfg, GripperButtonConfig, JointCfg, LoadedCfg};
use crate::error::{ArxError, Result};
use crate::types::MotorDiagnostic;
use crate::utils::gripper_button::decode_button_event_from_state;

fn is_dm_serial_channel(channel: &str) -> bool {
    let path = channel.trim();
    path.starts_with("/dev/tty") || path.starts_with("/dev/serial/")
}

const FEEDBACK_POLL_INTERVAL_MS: u64 = 3;
const FEEDBACK_WAIT_ATTEMPTS: u32 = 80;

fn joint_has_feedback(motor_map: &HashMap<String, Arc<DamiaoMotor>>, joint: &JointCfg) -> bool {
    motor_map
        .get(&joint.name)
        .and_then(|motor| motor.latest_state())
        .is_some()
}

fn request_feedback_for_missing(
    motor_map: &HashMap<String, Arc<DamiaoMotor>>,
    joints: &[JointCfg],
) {
    for joint in joints {
        if !joint_has_feedback(motor_map, joint) {
            if let Some(motor) = motor_map.get(&joint.name) {
                let _ = motor.request_motor_feedback();
            }
        }
    }
}

fn all_joints_have_feedback(
    motor_map: &HashMap<String, Arc<DamiaoMotor>>,
    joints: &[JointCfg],
) -> bool {
    joints
        .iter()
        .all(|joint| joint_has_feedback(motor_map, joint))
}

fn wait_for_joint_feedback(
    controller: &DamiaoController,
    motor_map: &HashMap<String, Arc<DamiaoMotor>>,
    joints: &[JointCfg],
) {
    for _ in 0..FEEDBACK_WAIT_ATTEMPTS {
        request_feedback_for_missing(motor_map, joints);
        let _ = controller.poll_feedback_once();
        if all_joints_have_feedback(motor_map, joints) {
            return;
        }
        thread::sleep(Duration::from_millis(FEEDBACK_POLL_INTERVAL_MS));
    }
}

fn read_joint_state_vectors(
    motor_map: &HashMap<String, Arc<DamiaoMotor>>,
    joints: &[JointCfg],
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut pos = Vec::with_capacity(joints.len());
    let mut vel = Vec::with_capacity(joints.len());
    let mut torq = Vec::with_capacity(joints.len());
    for joint in joints {
        if let Some(state) = motor_map
            .get(&joint.name)
            .and_then(|motor| motor.latest_state())
        {
            pos.push(f64::from(state.pos));
            vel.push(f64::from(state.vel));
            torq.push(f64::from(state.torq));
        } else {
            pos.push(f64::NAN);
            vel.push(f64::NAN);
            torq.push(f64::NAN);
        }
    }
    (pos, vel, torq)
}

pub struct ArxMCan {
    cfg: LoadedCfg,
    channel: String,
    baud: u32,
    controller: Option<Arc<DamiaoController>>,
    motor_map: HashMap<String, Arc<DamiaoMotor>>,
    groups: HashMap<String, GroupKind>,
    gripper_button_cfg: Option<GripperButtonConfig>,
    gripper_button_motor: Option<Arc<DamiaoMotor>>,
    connected: bool,
}

impl ArxMCan {
    pub fn new(channel: Option<&str>, baud: Option<u32>, hw_yaml: Option<&Path>) -> Result<Self> {
        let mut cfg = load_cfg(hw_yaml)?;
        if let Some(channel) = channel {
            cfg.channel = channel.to_string();
        }
        let baud = baud.unwrap_or(cfg.baud);
        Ok(Self {
            channel: cfg.channel.clone(),
            baud,
            cfg,
            controller: None,
            motor_map: HashMap::new(),
            groups: HashMap::new(),
            gripper_button_cfg: None,
            gripper_button_motor: None,
            connected: false,
        })
    }

    pub fn connect(&mut self) -> Result<()> {
        if self.connected {
            return Ok(());
        }
        self.setup_motors()?;
        self.connected = true;
        Ok(())
    }

    fn make_controller(&self) -> Result<Arc<DamiaoController>> {
        let controller = if is_dm_serial_channel(&self.channel) {
            DamiaoController::new_dm_serial(&self.channel, self.baud)?
        } else {
            DamiaoController::new_socketcan(&self.channel)?
        };
        Ok(Arc::new(controller))
    }

    fn setup_motors(&mut self) -> Result<()> {
        let controller = self.make_controller()?;
        for joint in &self.cfg.joints {
            let motor = controller.add_motor(joint.motor_id, joint.feedback_id, &joint.model)?;
            self.motor_map.insert(joint.name.clone(), motor);
        }
        if let Some(button_cfg) = &self.cfg.gripper_button {
            self.gripper_button_cfg = Some(button_cfg.clone());
            let motor = controller.add_motor(
                button_cfg.motor_id,
                button_cfg.feedback_id,
                &button_cfg.model,
            )?;
            self.gripper_button_motor = Some(motor);
        }
        self.build_groups(controller)?;
        Ok(())
    }

    fn build_groups(&mut self, controller: Arc<DamiaoController>) -> Result<()> {
        self.groups.clear();
        for (group_name, joint_names) in &self.cfg.groups {
            let group = JointGroup::new(
                group_name.clone(),
                joint_names.clone(),
                &self.cfg.joints,
                &self.motor_map,
                controller.clone(),
            );
            self.groups
                .insert(group_name.clone(), GroupKind::Active(group));
        }
        if !self.groups.contains_key("gripper") {
            self.groups
                .insert("gripper".to_string(), GroupKind::NoOp(NoOpGroup::new()));
        }
        self.controller = Some(controller);
        Ok(())
    }

    pub fn num_joints(&self) -> usize {
        self.cfg.joints.len()
    }

    pub fn joint_names(&self) -> Vec<String> {
        self.cfg.joints.iter().map(|j| j.name.clone()).collect()
    }

    pub fn has_gripper_button(&self) -> bool {
        self.gripper_button_cfg.is_some()
    }

    pub fn arm_mut(&mut self) -> Result<&mut JointGroup> {
        match self.groups.get_mut("arm") {
            Some(GroupKind::Active(group)) => Ok(group),
            _ => Err(ArxError::Runtime("arm group not configured".into())),
        }
    }

    pub fn arm(&self) -> Result<&JointGroup> {
        match self.groups.get("arm") {
            Some(GroupKind::Active(group)) => Ok(group),
            _ => Err(ArxError::Runtime("arm group not configured".into())),
        }
    }

    pub fn gripper_mut(&mut self) -> Result<&mut JointGroup> {
        match self.groups.get_mut("gripper") {
            Some(GroupKind::Active(group)) => Ok(group),
            _ => Err(ArxError::Runtime("gripper group not configured".into())),
        }
    }

    pub fn gripper(&self) -> Result<&JointGroup> {
        match self.groups.get("gripper") {
            Some(GroupKind::Active(group)) => Ok(group),
            _ => Err(ArxError::Runtime("gripper group not configured".into())),
        }
    }

    pub fn request_all_feedback(&self) -> Result<()> {
        let controller = self.controller.as_ref().ok_or(ArxError::NotConnected)?;
        for motor in self.motor_map.values() {
            let _ = motor.request_motor_feedback();
        }
        wait_for_joint_feedback(controller, &self.motor_map, &self.cfg.joints);
        Ok(())
    }

    pub fn arm_hardware_ready(&self, require_mit: bool) -> Result<bool> {
        let arm = self.arm()?;
        if !arm.all_motors_enabled() {
            return Ok(false);
        }
        if require_mit && !arm.all_motors_mit_mode() {
            return Ok(false);
        }
        Ok(true)
    }

    pub fn ensure_mit_mode(&mut self) -> Result<()> {
        self.arm_mut()?.ensure_mit_mode()?;
        if let Ok(gripper) = self.gripper_mut() {
            gripper.ensure_mit_mode()?;
        }
        Ok(())
    }

    pub fn enable_selective(&self) -> Result<()> {
        self.arm()?.enable_selective()?;
        if let Ok(gripper) = self.gripper() {
            gripper.enable_selective()?;
        }
        Ok(())
    }

    pub fn set_zero(&self) -> Result<()> {
        for group in self.groups.values() {
            group.disable();
        }
        thread::sleep(Duration::from_millis(300));

        let controller = self.controller.as_ref().ok_or(ArxError::NotConnected)?;

        for joint in &self.cfg.joints {
            for _ in 0..200 {
                for motor in self.motor_map.values() {
                    let _ = motor.request_motor_feedback();
                }
                let _ = controller.poll_feedback_once();
                if let Some(state) = self.motor_map[&joint.name].latest_state() {
                    if state.status_code == 0 {
                        break;
                    }
                }
                thread::sleep(Duration::from_millis(50));
            }
            let _ = self.motor_map[&joint.name].set_zero_position();
            thread::sleep(Duration::from_millis(100));
        }
        Ok(())
    }

    pub fn get_state(&self, request_feedback: bool) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>)> {
        let controller = self.controller.as_ref().ok_or(ArxError::NotConnected)?;

        if request_feedback {
            for motor in self.motor_map.values() {
                let _ = motor.request_motor_feedback();
            }
            wait_for_joint_feedback(controller, &self.motor_map, &self.cfg.joints);
        } else {
            let _ = controller.poll_feedback_once();
        }

        Ok(read_joint_state_vectors(&self.motor_map, &self.cfg.joints))
    }

    /// Encoder position of one named joint in radians (gripper name is `gripper`).
    pub fn read_joint_position(&self, name: &str, request_feedback: bool) -> Result<Option<f64>> {
        let motor = self
            .motor_map
            .get(name)
            .ok_or_else(|| ArxError::Runtime(format!("joint {name} not on bus")))?;
        let controller = self.controller.as_ref().ok_or(ArxError::NotConnected)?;
        if request_feedback {
            let _ = motor.request_motor_feedback();
            // A few RX polls so the 0x7FF reply can land without waiting on arm joints.
            for _ in 0..5 {
                let _ = controller.poll_feedback_once();
                if motor
                    .latest_state()
                    .is_some_and(|s| f64::from(s.pos).is_finite())
                {
                    break;
                }
                thread::sleep(Duration::from_millis(FEEDBACK_POLL_INTERVAL_MS));
            }
        } else {
            let _ = controller.poll_feedback_once();
        }
        Ok(motor
            .latest_state()
            .map(|s| f64::from(s.pos))
            .filter(|p| p.is_finite()))
    }

    pub fn get_motor_diagnostics(&self, request_feedback: bool) -> Result<Vec<MotorDiagnostic>> {
        let controller = self.controller.as_ref().ok_or(ArxError::NotConnected)?;
        if request_feedback {
            for motor in self.motor_map.values() {
                motor.request_motor_feedback()?;
            }
            wait_for_joint_feedback(controller, &self.motor_map, &self.cfg.joints);
        } else {
            controller.poll_feedback_once()?;
        }

        Ok(self
            .cfg
            .joints
            .iter()
            .map(|joint| {
                let state = self.motor_map[&joint.name].latest_state();
                MotorDiagnostic {
                    name: joint.name.clone(),
                    motor_id: joint.motor_id,
                    feedback_id: joint.feedback_id,
                    status_code: state.as_ref().map_or(0xff, |s| s.status_code),
                    status_name: state
                        .as_ref()
                        .map_or_else(|| "NO_FEEDBACK".to_string(), |s| s.status_name.to_string()),
                    position: state.as_ref().map_or(f64::NAN, |s| f64::from(s.pos)),
                    velocity: state.as_ref().map_or(f64::NAN, |s| f64::from(s.vel)),
                    torque: state.as_ref().map_or(f64::NAN, |s| f64::from(s.torq)),
                    mos_temperature_c: state.as_ref().map_or(f64::NAN, |s| f64::from(s.t_mos)),
                    rotor_temperature_c: state.as_ref().map_or(f64::NAN, |s| f64::from(s.t_rotor)),
                }
            })
            .collect())
    }

    pub fn poll_gripper_button_event(&self) -> Result<Option<u8>> {
        let button_cfg = match &self.gripper_button_cfg {
            Some(cfg) => cfg,
            None => return Ok(None),
        };
        let motor = match &self.gripper_button_motor {
            Some(motor) => motor,
            None => return Ok(None),
        };
        let controller = self.controller.as_ref().ok_or(ArxError::NotConnected)?;

        let _ = motor.send_cmd_mit(0.0, 0.0, 0.0, 0.0, 0.0);
        thread::sleep(Duration::from_millis(2));
        let _ = controller.poll_feedback_once();
        Ok(decode_button_event_from_state(
            motor.latest_state().as_ref(),
            button_cfg.motor_id,
            button_cfg.feedback_id,
            button_cfg.pmax,
        ))
    }

    pub fn disconnect(&mut self) -> Result<()> {
        if !self.connected {
            return Ok(());
        }
        if let Some(controller) = self.controller.take() {
            // Match arx_d_can: close serial/CAN only; do not disable motors.
            let _ = controller.close_bus();
        }
        self.motor_map.clear();
        self.groups.clear();
        self.gripper_button_motor = None;
        self.connected = false;
        Ok(())
    }
}

impl Drop for ArxMCan {
    fn drop(&mut self) {
        let _ = self.disconnect();
    }
}
