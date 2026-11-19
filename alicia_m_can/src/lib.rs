//! Alicia M CAN — Rust robot SDK for Damiao motors over USB2CAN (motorbridge).

pub mod config;
pub mod control_loop;
pub mod dynamics;
pub mod error;
pub mod gripper;
pub mod hardware;
pub mod robot;
pub mod scan;
pub mod sparkvis;
pub mod teleop;
pub mod trajectory;
pub mod types;
pub mod utils;

pub use config::{default_config, ArxMCanConfig, JointMotorConfig};
pub use dynamics::{GravityCompConfig, GravityCompensator};
pub use error::{ArxError, Result};
pub use gripper::{GraspResult, GripperController};
pub use hardware::{ArxMCanArm, JointGroup};
pub use robot::{
    ArxMCanRobot, DEFAULT_ARM_CONTROL_MODE, DEFAULT_BAUD, DEFAULT_CONTROL_HZ,
    DEFAULT_GRIPPER_SPEED, DEFAULT_JOINT_SPEED, DEFAULT_MAX_JOINT_VEL_RAD_S,
    DEFAULT_MIT_CONTROL_HZ, DEFAULT_PORT, DEFAULT_WAIT_TIMEOUT_SEC, DEFAULT_WAIT_TOLERANCE_RAD,
    HOME_JOINTS, NUM_ARM_JOINTS,
};
pub use scan::{scan_ids, ScanOptions};
pub use sparkvis::{SparkVisBridge, SparkVisConfig};
pub use teleop::mapping::alicia_d_rad_to_alicia_m_rad;
pub use teleop::{LeaderSample, TeleopConfig, TeleopSession};
pub use trajectory::{
    TrajectoryLimits, DEFAULT_MAX_JOINT_ACCEL_RAD_S2, DEFAULT_MAX_JOINT_JERK_RAD_S3,
};
pub use types::{ArxMCanState, JointState, MotorDiagnostic, MotorState, VlimInput};

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
