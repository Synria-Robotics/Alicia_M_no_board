#[derive(Debug, Clone)]
pub struct MotorState {
    pub name: String,
    pub motor_id: u16,
    pub feedback_id: u16,
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
}

#[derive(Debug, Clone)]
pub struct MotorDiagnostic {
    pub name: String,
    pub motor_id: u16,
    pub feedback_id: u16,
    pub status_code: u8,
    pub status_name: String,
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub mos_temperature_c: f64,
    pub rotor_temperature_c: f64,
}

#[derive(Debug, Clone)]
pub struct JointState {
    pub names: Vec<String>,
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub torques: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct ArxMCanState {
    pub arm: JointState,
    pub gripper: Option<MotorState>,
}

#[derive(Debug, Clone)]
pub enum VlimInput {
    Scalar(f64),
    PerJoint(Vec<f64>),
}
