use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use super::defaults::resolve_hw_cfg_path;
use crate::error::{ArxError, Result};

#[derive(Debug, Clone, Deserialize)]
struct RawJoint {
    name: String,
    motor_id: u16,
    feedback_id: u16,
    model: Option<String>,
    #[serde(default, rename = "MIT")]
    mit: RawMit,
    #[serde(default, rename = "POS_VEL")]
    pos_vel: RawPosVel,
}

#[derive(Debug, Clone, Default, Deserialize)]
struct RawMit {
    kp: Option<f64>,
    kd: Option<f64>,
}

#[derive(Debug, Clone, Default, Deserialize)]
struct RawPosVel {
    vel_kp: Option<f64>,
    vel_ki: Option<f64>,
    pos_kp: Option<f64>,
    pos_ki: Option<f64>,
    vlim: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
struct RawGripperButton {
    motor_id: u16,
    feedback_id: u16,
    model: Option<String>,
    pmax: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
struct RawGripperMapping {
    closed_value: Option<f64>,
    open_value: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
struct RawCfg {
    name: Option<String>,
    arm_control_mode: Option<String>,
    channel: Option<String>,
    baud: Option<u32>,
    rate: Option<f64>,
    groups: Option<HashMap<String, RawGroup>>,
    joints: Vec<RawJoint>,
    gripper_mapping: Option<RawGripperMapping>,
    gripper_button: Option<RawGripperButton>,
}

#[derive(Debug, Clone, Deserialize)]
struct RawGroup {
    joints: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct JointCfg {
    pub name: String,
    pub motor_id: u16,
    pub feedback_id: u16,
    pub model: String,
    pub kp: f64,
    pub kd: f64,
    pub vel_kp: f64,
    pub vel_ki: f64,
    pub pos_kp: f64,
    pub pos_ki: f64,
    pub vlim: f64,
}

#[derive(Debug, Clone)]
pub struct GripperMapping {
    pub closed_value: f64,
    pub open_value: f64,
}

#[derive(Debug, Clone)]
pub struct GripperButtonConfig {
    pub motor_id: u16,
    pub feedback_id: u16,
    pub model: String,
    pub pmax: f64,
}

#[derive(Debug, Clone)]
pub struct LoadedCfg {
    pub name: String,
    pub arm_control_mode: Option<String>,
    pub channel: String,
    pub baud: u32,
    pub rate: f64,
    pub groups: HashMap<String, Vec<String>>,
    pub joints: Vec<JointCfg>,
    pub gripper_mapping: GripperMapping,
    pub gripper_button: Option<GripperButtonConfig>,
}

#[derive(Debug, Clone)]
pub struct JointMotorConfig {
    pub name: String,
    pub motor_id: u16,
    pub feedback_id: u16,
    pub model: String,
    pub mit_kp: f64,
    pub mit_kd: f64,
    pub pv_vel_kp: f64,
    pub pv_vel_ki: f64,
    pub pv_pos_kp: f64,
    pub pv_pos_ki: f64,
    pub pv_vlim: f64,
}

#[derive(Debug, Clone)]
pub struct ArxMCanConfig {
    pub port: String,
    pub baud: u32,
    pub control_hz: f64,
    pub arm_control_mode: String,
    pub arm_joints: Vec<JointMotorConfig>,
    pub gripper: Option<JointMotorConfig>,
    pub gripper_closed_value: f64,
    pub gripper_open_value: f64,
    pub gripper_button: Option<GripperButtonConfig>,
    /// Resolved hardware YAML used to build this config (for reconnect / bus init).
    pub hw_yaml: Option<PathBuf>,
}

impl ArxMCanConfig {
    pub fn joint_names(&self) -> Vec<String> {
        self.arm_joints.iter().map(|j| j.name.clone()).collect()
    }
}

fn joint_from_yaml(joint: &JointCfg) -> JointMotorConfig {
    JointMotorConfig {
        name: joint.name.clone(),
        motor_id: joint.motor_id,
        feedback_id: joint.feedback_id,
        model: joint.model.clone(),
        mit_kp: joint.kp,
        mit_kd: joint.kd,
        pv_vel_kp: joint.vel_kp,
        pv_vel_ki: joint.vel_ki,
        pv_pos_kp: joint.pos_kp,
        pv_pos_ki: joint.pos_ki,
        pv_vlim: joint.vlim,
    }
}

pub fn load_cfg(hw_yaml: Option<&Path>) -> Result<LoadedCfg> {
    let path = resolve_hw_cfg_path(hw_yaml)?;
    let text = fs::read_to_string(&path)?;
    let data: RawCfg = serde_yaml::from_str(&text)?;

    let joints: Vec<JointCfg> = data
        .joints
        .iter()
        .map(|joint| JointCfg {
            name: joint.name.clone(),
            motor_id: joint.motor_id,
            feedback_id: joint.feedback_id,
            model: joint.model.clone().unwrap_or_else(|| "4340P".to_string()),
            kp: joint.mit.kp.unwrap_or(0.0),
            kd: joint.mit.kd.unwrap_or(0.0),
            vel_kp: joint.pos_vel.vel_kp.unwrap_or(0.0),
            vel_ki: joint.pos_vel.vel_ki.unwrap_or(0.0),
            pos_kp: joint.pos_vel.pos_kp.unwrap_or(0.0),
            pos_ki: joint.pos_vel.pos_ki.unwrap_or(0.0),
            vlim: joint.pos_vel.vlim.unwrap_or(2.0),
        })
        .collect();

    let groups = data.groups.unwrap_or_default();
    let groups: HashMap<String, Vec<String>> = groups
        .into_iter()
        .map(|(name, group)| (name, group.joints))
        .collect();

    let gripper_mapping = data.gripper_mapping.unwrap_or(RawGripperMapping {
        closed_value: Some(0.0),
        open_value: Some(5.0),
    });

    let gripper_button = data.gripper_button.map(|raw| GripperButtonConfig {
        motor_id: raw.motor_id,
        feedback_id: raw.feedback_id,
        model: raw.model.unwrap_or_else(|| "4310".to_string()),
        pmax: raw.pmax.unwrap_or(12.5),
    });

    Ok(LoadedCfg {
        name: data.name.unwrap_or_else(|| "ALICIA-M-CAN".to_string()),
        arm_control_mode: data.arm_control_mode,
        channel: data.channel.unwrap_or_else(|| "/dev/ttyACM0".to_string()),
        baud: data.baud.unwrap_or(1_000_000),
        rate: data.rate.unwrap_or(500.0),
        groups,
        joints,
        gripper_mapping: GripperMapping {
            closed_value: gripper_mapping.closed_value.unwrap_or(0.0),
            open_value: gripper_mapping.open_value.unwrap_or(5.0),
        },
        gripper_button,
    })
}

pub fn default_config(
    port: Option<&str>,
    baud: Option<u32>,
    control_hz: f64,
    arm_control_mode: &str,
    hw_yaml: Option<&Path>,
) -> Result<ArxMCanConfig> {
    let hw_path = resolve_hw_cfg_path(hw_yaml)?;
    let data = load_cfg(Some(&hw_path))?;
    let joints_by_name: HashMap<String, JointCfg> = data
        .joints
        .iter()
        .cloned()
        .map(|joint| (joint.name.clone(), joint))
        .collect();

    let arm_names = data.groups.get("arm").cloned().unwrap_or_default();
    let gripper_names = data.groups.get("gripper").cloned().unwrap_or_default();

    let arm_joints: Result<Vec<JointMotorConfig>> = arm_names
        .iter()
        .map(|name| {
            joints_by_name
                .get(name)
                .map(joint_from_yaml)
                .ok_or_else(|| ArxError::Config(format!("joint {name} not found in config")))
        })
        .collect();

    let gripper = gripper_names
        .first()
        .and_then(|name| joints_by_name.get(name))
        .map(joint_from_yaml);

    let mode = data.arm_control_mode.as_deref().unwrap_or(arm_control_mode);

    Ok(ArxMCanConfig {
        port: port
            .map(str::to_string)
            .unwrap_or_else(|| data.channel.clone()),
        baud: baud.unwrap_or(data.baud),
        control_hz,
        arm_control_mode: mode.to_string(),
        arm_joints: arm_joints?,
        gripper,
        gripper_closed_value: data.gripper_mapping.closed_value,
        gripper_open_value: data.gripper_mapping.open_value,
        gripper_button: data.gripper_button,
        hw_yaml: Some(hw_path),
    })
}
