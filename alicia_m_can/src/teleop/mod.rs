pub mod mapping;

use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::error::{ArxError, Result};
use crate::robot::ArxMCanRobot;
use crate::utils::gripper::validate_gripper_value;
use mapping::{alicia_d_rad_to_alicia_m_rad, map_gripper};

#[derive(Debug, Clone)]
pub struct TeleopConfig {
    pub frequency_hz: f64,
    pub follower_speed: f64,
    pub gripper_speed: f64,
    pub gripper_scale: f64,
    pub use_joint_mapping: bool,
    pub max_joint_accel_rad_s2: Option<f64>,
    pub max_joint_jerk_rad_s3: Option<f64>,
}

impl Default for TeleopConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 60.0,
            follower_speed: 400.0,
            gripper_speed: 400.0,
            gripper_scale: 1.0,
            use_joint_mapping: true,
            max_joint_accel_rad_s2: None,
            max_joint_jerk_rad_s3: None,
        }
    }
}

pub struct LeaderSample {
    pub joints: [f64; 6],
    pub gripper: f64,
}

pub struct TeleopSession {
    follower: Arc<Mutex<ArxMCanRobot>>,
    config: TeleopConfig,
    stop: Arc<Mutex<bool>>,
    handle: Option<JoinHandle<()>>,
}

impl TeleopSession {
    pub fn new(follower: ArxMCanRobot, config: TeleopConfig) -> Self {
        Self {
            follower: Arc::new(Mutex::new(follower)),
            config,
            stop: Arc::new(Mutex::new(false)),
            handle: None,
        }
    }

    pub fn start<F>(&mut self, read_leader: F) -> Result<()>
    where
        F: FnMut() -> Result<Option<LeaderSample>> + Send + 'static,
    {
        if self.handle.is_some() {
            return Err(ArxError::Runtime("teleop already running".into()));
        }
        let follower = self.follower.clone();
        let stop = self.stop.clone();
        let config = self.config.clone();
        *self.stop.lock().unwrap() = false;
        self.handle = Some(thread::Builder::new().name("arx-teleop".into()).spawn(
            move || teleop_loop(follower, stop, config, read_leader),
        )?);
        Ok(())
    }

    pub fn stop(&mut self) {
        *self.stop.lock().unwrap() = true;
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }

    pub fn disconnect_follower(&mut self) -> Result<()> {
        self.stop();
        self.follower
            .lock()
            .map_err(|_| ArxError::Runtime("follower mutex poisoned".into()))?
            .disconnect()
    }
}

impl Drop for TeleopSession {
    fn drop(&mut self) {
        self.stop();
    }
}

fn teleop_loop<F>(
    follower: Arc<Mutex<ArxMCanRobot>>,
    stop: Arc<Mutex<bool>>,
    config: TeleopConfig,
    mut read_leader: F,
) where
    F: FnMut() -> Result<Option<LeaderSample>>,
{
    let interval = Duration::from_secs_f64(1.0 / config.frequency_hz.max(1.0));
    while !*stop.lock().unwrap() {
        let t0 = Instant::now();
        if let Ok(Some(sample)) = read_leader() {
            let mut joints = sample.joints;
            if config.use_joint_mapping {
                joints = alicia_d_rad_to_alicia_m_rad(&joints);
            }
            let gripper = map_gripper(sample.gripper, config.gripper_scale);
            if let Ok(mut robot) = follower.lock() {
                let _ = robot.set_robot_state(
                    Some(&joints),
                    Some(validate_gripper_value(gripper)),
                    config.follower_speed,
                    config.gripper_speed,
                    false,
                    None,
                    None,
                    config.max_joint_accel_rad_s2,
                    config.max_joint_jerk_rad_s3,
                    true,
                    None,
                    None,
                );
            }
        }
        let elapsed = t0.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }
}
