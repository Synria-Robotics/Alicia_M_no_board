use std::path::Path;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crate::config::default_config;
use crate::control_loop::{compute_gravity_ff, CommandRefreshLoop};
use crate::dynamics::{GravityCompConfig, GravityCompensator};
use crate::error::{ArxError, Result};
use crate::hardware::ArxMCanArm;
use crate::trajectory::convergence::{max_abs_joint_delta, StallWatch};
use crate::trajectory::governor::{ReferenceGovernor, DEFAULT_TORQUE_MARGIN};
use crate::trajectory::{build_trajectory_limits_from_vlim, needs_trajectory};
use crate::types::{ArxMCanState, MotorDiagnostic, VlimInput};
use crate::utils::gripper::validate_gripper_value;
use crate::utils::speed_mapping::speed_to_vlim_rad_s;

pub const NUM_ARM_JOINTS: usize = 6;
pub const DEFAULT_PORT: &str = "/dev/ttyACM0";
pub const DEFAULT_BAUD: u32 = 1_000_000;
pub const DEFAULT_CONTROL_HZ: f64 = 100.0;
pub const DEFAULT_MIT_CONTROL_HZ: f64 = 500.0;
pub const DEFAULT_ARM_CONTROL_MODE: &str = "mit";
pub const DEFAULT_MAX_JOINT_VEL_RAD_S: f64 = 3.0;
pub const DEFAULT_JOINT_SPEED: f64 = 400.0;
pub const DEFAULT_GRIPPER_SPEED: f64 = 100.0;
pub const DEFAULT_GRIPPER_WAIT_TOLERANCE: f64 = 20.0;
pub const DEFAULT_GRIPPER_WAIT_TIMEOUT_SEC: f64 = 5.0;
pub const DEFAULT_WAIT_TOLERANCE_RAD: f64 = 0.02;
pub const DEFAULT_WAIT_TIMEOUT_SEC: f64 = 15.0;
pub const HOME_JOINTS: [f64; NUM_ARM_JOINTS] = [0.0; NUM_ARM_JOINTS];

/// MIT frames sent immediately after enable, before the refresh thread's first tick.
const STARTUP_BURST_FRAMES: u32 = 8;
const STARTUP_BURST_PERIOD: Duration = Duration::from_millis(2);

fn arm_max_vlim_rad_s(arm: &ArxMCanArm) -> f64 {
    arm.config()
        .arm_joints
        .iter()
        .filter_map(|joint| (joint.pv_vlim > 0.0).then_some(joint.pv_vlim))
        .reduce(f64::min)
        .unwrap_or(DEFAULT_MAX_JOINT_VEL_RAD_S)
}

fn normalize_mode(mode: &str) -> String {
    mode.trim().to_ascii_lowercase().replace('_', "")
}

fn lock_arm(arm: &Arc<Mutex<ArxMCanArm>>) -> Result<std::sync::MutexGuard<'_, ArxMCanArm>> {
    arm.lock()
        .map_err(|_| ArxError::Runtime("arm mutex poisoned".into()))
}

/// Wait without retaining the arm mutex between polls.
///
/// The MIT refresh thread needs this same mutex on every control tick.
/// A joint error that stays at least 0.05 rad while the arm is slower than
/// 0.08 rad/s for 0.3 s returns false immediately. The timeout still covers
/// a move that is traveling.
fn wait_until_at_target(
    arm: &Arc<Mutex<ArxMCanArm>>,
    target: &[f64],
    tolerance: f64,
    timeout: f64,
    poll_interval: f64,
) -> Result<bool> {
    let deadline = Instant::now() + Duration::from_secs_f64(timeout.max(0.0));
    let pause = Duration::from_secs_f64(poll_interval.max(0.001));
    let mut stall = StallWatch::arm_blocked();
    let mut prev: Option<(Vec<f64>, Instant)> = None;
    while Instant::now() < deadline {
        let now = Instant::now();
        let measured = {
            let guard = lock_arm(arm)?;
            let state = guard.read_state(false)?;
            if state.arm.positions.len() < target.len() {
                None
            } else {
                Some(state.arm.positions[..target.len()].to_vec())
            }
        };
        if let Some(q) = measured {
            let err = max_abs_joint_delta(&q, target);
            if err <= tolerance {
                return Ok(true);
            }
            if let Some((prev_q, prev_t)) = prev.as_ref() {
                let dt = now.saturating_duration_since(*prev_t).as_secs_f64();
                let vel = max_abs_joint_delta(&q, prev_q) / dt.max(1e-3);
                if stall.observe(err, vel, now) {
                    return Ok(false);
                }
            }
            prev = Some((q, now));
        }
        thread::sleep(pause);
    }
    Ok(false)
}

pub struct ArxMCanRobot {
    control_hz: f64,
    wait_tolerance_rad: f64,
    wait_timeout_sec: f64,
    arm_control_mode: String,
    arm: Arc<Mutex<ArxMCanArm>>,
    refresh: Option<CommandRefreshLoop>,
    gravity: Option<Arc<Mutex<GravityCompensator>>>,
    enabled: bool,
    max_joint_vel_rad_s: f64,
    mit_mode: bool,
    commanded_joints: Vec<f64>,
    command_vlim: f64,
}

impl ArxMCanRobot {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        port: &str,
        baud: u32,
        control_hz: f64,
        max_joint_vel_rad_s: Option<f64>,
        wait_tolerance_rad: f64,
        wait_timeout_sec: f64,
        auto_enable: bool,
        preserve_enable_state: bool,
        arm_control_mode: Option<&str>,
        hw_yaml: Option<&Path>,
        gravity_config: Option<GravityCompConfig>,
    ) -> Result<Self> {
        let mode = arm_control_mode.unwrap_or(DEFAULT_ARM_CONTROL_MODE);
        let mut gravity_cfg = gravity_config.unwrap_or_default();
        let mit_mode = normalize_mode(mode) == "mit";
        if gravity_cfg.enabled && !mit_mode {
            return Err(ArxError::InvalidArgument(
                "gravity feedforward requires arm_control_mode='mit'".into(),
            ));
        }
        let effective_hz = if gravity_cfg.enabled {
            control_hz.max(DEFAULT_MIT_CONTROL_HZ)
        } else {
            control_hz
        };

        let mut arm = ArxMCanArm::new(
            default_config(Some(port), Some(baud), effective_hz, mode, hw_yaml)?,
            true,
        )?;
        arm.connect()?;

        let max_joint_vel_rad_s = max_joint_vel_rad_s
            .filter(|value| *value > 0.0)
            .unwrap_or_else(|| arm_max_vlim_rad_s(&arm));
        let arm = Arc::new(Mutex::new(arm));
        let q0 = read_joint_feedback(&arm)?;
        let control_hz = effective_hz.max(1.0);

        let gravity = if gravity_cfg.enabled {
            gravity_cfg.enabled = true;
            Some(Arc::new(Mutex::new(GravityCompensator::from_config(
                &gravity_cfg,
                NUM_ARM_JOINTS,
            )?)))
        } else {
            None
        };

        let mut robot = Self {
            control_hz,
            wait_tolerance_rad,
            wait_timeout_sec,
            arm_control_mode: mode.to_string(),
            arm: arm.clone(),
            refresh: None,
            gravity,
            enabled: false,
            max_joint_vel_rad_s,
            mit_mode,
            commanded_joints: q0.clone(),
            command_vlim: 0.0,
        };

        if preserve_enable_state || auto_enable {
            let preserve = preserve_enable_state || {
                lock_arm(&robot.arm)?
                    .detect_hardware_preserve(mode)
                    .unwrap_or(false)
            };

            // Adopt software enabled state before MIT sends (prewarm uses require_enabled).
            if lock_arm(&robot.arm)?.hardware_all_enabled()? {
                lock_arm(&robot.arm)?.adopt_enabled(true);
            }

            robot.prewarm_if_enabled()?;

            {
                let mut guard = lock_arm(&robot.arm)?;
                guard.prepare_for_control(Some(mode), preserve)?;
                if !guard.enabled() {
                    guard.enable_if_needed()?;
                }
                robot.enabled = guard.enabled();
            }

            if robot.enabled {
                robot.startup_mit_hold()?;
            }
        } else {
            lock_arm(&robot.arm)?.configure(None)?;
        }
        // auto_enable=false: leave hardware enable/mode unchanged (read-only / external control).

        Ok(robot)
    }

    pub fn arm_control_mode(&self) -> &str {
        &self.arm_control_mode
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn enable(&mut self) -> Result<()> {
        {
            let mut guard = lock_arm(&self.arm)?;
            guard.enable_if_needed()?;
            self.enabled = guard.enabled();
        }
        if self.enabled {
            self.startup_mit_hold()?;
        }
        Ok(())
    }

    pub fn disable(&mut self) -> Result<()> {
        self.pause_refresh_loop();
        lock_arm(&self.arm)?.disable()?;
        self.enabled = false;
        Ok(())
    }

    /// Read feedback and send once (arx_d_can_ref ``hold_current_position``).
    pub fn hold_current_position(&mut self) -> Result<Vec<f64>> {
        let joints = read_joint_feedback(&self.arm)?;
        self.send_joints_direct(&joints, 0.0)?;
        Ok(joints)
    }

    pub fn get_gripper_button_event(&self) -> Result<Option<u8>> {
        lock_arm(&self.arm)?.get_gripper_button_event()
    }

    pub fn get_robot_state_joint(&self) -> Result<Vec<f64>> {
        read_joint_feedback(&self.arm)
    }

    pub fn get_arm_state(&self, request_feedback: bool) -> Result<ArxMCanState> {
        lock_arm(&self.arm)?.read_state(request_feedback)
    }

    /// Gripper opening in user units (0=closed, 1000=open).
    pub fn get_gripper_value(&self, request_feedback: bool) -> Result<Option<f64>> {
        lock_arm(&self.arm)?.get_gripper_value(request_feedback)
    }

    pub fn get_motor_diagnostics(&self, request_feedback: bool) -> Result<Vec<MotorDiagnostic>> {
        lock_arm(&self.arm)?.motor_diagnostics(request_feedback)
    }

    pub fn get_control_loop_stats(&self) -> (u64, u64) {
        self.refresh
            .as_ref()
            .map_or((0, 0), CommandRefreshLoop::stats)
    }

    pub fn get_arm_mit_gains(&self) -> Result<(Vec<f64>, Vec<f64>)> {
        lock_arm(&self.arm)?.get_arm_mit_gains()
    }

    pub fn set_arm_mit_gains(&mut self, kp: Option<Vec<f64>>, kd: Option<Vec<f64>>) -> Result<()> {
        let mut guard = lock_arm(&self.arm)?;
        guard.set_arm_mit_gains(kp.as_deref(), kd.as_deref())
    }

    pub fn get_ik_seed_joints(&self) -> Vec<f64> {
        self.commanded_joints.clone()
    }

    /// MIT freeze at `positions` (`dq = 0`, no walker). Sit-still / enable.
    pub fn hold_arm_joints(&mut self, positions: &[f64], speed: f64) -> Result<()> {
        let target: Vec<f64> = positions.iter().take(NUM_ARM_JOINTS).copied().collect();
        let vlim = self.vlim_from_speed(speed);
        self.send_joints_direct(&target, vlim)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn set_robot_state(
        &mut self,
        target_joints: Option<&[f64]>,
        gripper_value: Option<f64>,
        speed: f64,
        gripper_speed: f64,
        wait_for_completion: bool,
        joint_tolerance: Option<f64>,
        converge_timeout: Option<f64>,
        max_joint_accel_rad_s2: Option<f64>,
        max_joint_jerk_rad_s3: Option<f64>,
        stream: bool,
        gripper_tolerance: Option<f64>,
        gripper_timeout_sec: Option<f64>,
    ) -> Result<bool> {
        let mut joints_converged = true;
        if let Some(joints) = target_joints {
            let target: Vec<f64> = joints.iter().take(NUM_ARM_JOINTS).copied().collect();
            let vlim = self.vlim_from_speed(speed);
            if vlim > 0.0 {
                joints_converged = self.move_arm_joints(
                    &target,
                    vlim,
                    joint_tolerance.unwrap_or(self.wait_tolerance_rad),
                    converge_timeout.unwrap_or(self.wait_timeout_sec),
                    wait_for_completion,
                    max_joint_accel_rad_s2,
                    max_joint_jerk_rad_s3,
                    stream,
                )?;
            }
        }
        let mut gripper_ok = true;
        if let Some(value) = gripper_value {
            gripper_ok = lock_arm(&self.arm)?.set_gripper(
                validate_gripper_value(value),
                gripper_speed,
                Some(self.control_hz),
                true,
                wait_for_completion,
                gripper_tolerance.unwrap_or(DEFAULT_GRIPPER_WAIT_TOLERANCE),
                gripper_timeout_sec.unwrap_or(DEFAULT_GRIPPER_WAIT_TIMEOUT_SEC),
            )?;
        }
        Ok(joints_converged && (gripper_ok || !wait_for_completion))
    }

    pub fn go_home(&mut self, speed: f64) -> Result<()> {
        self.set_robot_state(
            Some(&HOME_JOINTS),
            None,
            speed,
            DEFAULT_GRIPPER_SPEED,
            true,
            None,
            None,
            None,
            None,
            false,
            None,
            None,
        )?;
        Ok(())
    }

    pub fn disconnect(&mut self) -> Result<()> {
        if let Some(ref mut refresh) = self.refresh {
            refresh.stop();
        }
        self.refresh = None;
        lock_arm(&self.arm)?.close(false)
    }

    #[allow(clippy::too_many_arguments)]
    fn move_arm_joints(
        &mut self,
        target: &[f64],
        vlim: f64,
        tolerance: f64,
        timeout: f64,
        wait_for_completion: bool,
        max_joint_accel_rad_s2: Option<f64>,
        max_joint_jerk_rad_s3: Option<f64>,
        stream: bool,
    ) -> Result<bool> {
        if stream {
            self.stream_arm_joints(target, vlim)?;
            if wait_for_completion {
                return wait_until_at_target(&self.arm, target, tolerance, timeout, 0.01);
            }
            return Ok(true);
        }

        let state = lock_arm(&self.arm)?.read_state(true)?;
        let q_meas: Vec<f64> = state.arm.positions.iter().take(NUM_ARM_JOINTS).copied().collect();
        let dq_meas: Vec<f64> = state
            .arm
            .velocities
            .iter()
            .take(NUM_ARM_JOINTS)
            .copied()
            .collect();

        if !needs_trajectory(&q_meas, target, tolerance) {
            self.send_joints_direct(target, vlim)?;
            return Ok(true);
        }

        if self.mit_mode && self.refresh.is_some() {
            let joint_cfgs = lock_arm(&self.arm)?.config().arm_joints.clone();
            let limits = build_trajectory_limits_from_vlim(
                NUM_ARM_JOINTS,
                vlim,
                &joint_cfgs,
                max_joint_accel_rad_s2,
                max_joint_jerk_rad_s3,
            );
            let refresh = self.refresh.as_ref().expect("refresh loop");
            refresh.start_trajectory(&q_meas, target, &dq_meas, &limits, vlim)?;
            self.commanded_joints = target.to_vec();
            self.command_vlim = vlim;
            if wait_for_completion {
                return wait_until_at_target(&self.arm, target, tolerance, timeout, 0.01);
            }
            Ok(true)
        } else {
            self.send_joints_direct(target, vlim)?;
            if wait_for_completion {
                wait_until_at_target(&self.arm, target, tolerance, timeout, 0.01)
            } else {
                Ok(true)
            }
        }
    }

    fn start_refresh_loop(&mut self) -> Result<()> {
        if self.refresh.is_some() {
            self.resume_refresh_loop();
            return Ok(());
        }
        let joint_cfgs = lock_arm(&self.arm)?.config().arm_joints.clone();
        let governor = if self.mit_mode {
            Some(ReferenceGovernor::from_joint_configs(
                &joint_cfgs,
                DEFAULT_TORQUE_MARGIN,
            ))
        } else {
            None
        };
        let refresh = CommandRefreshLoop::new(
            self.arm.clone(),
            self.control_hz,
            self.mit_mode,
            self.commanded_joints.clone(),
            self.command_vlim,
            self.gravity.clone(),
            governor,
        );
        refresh.resume();
        self.refresh = Some(refresh);
        Ok(())
    }

    fn pause_refresh_loop(&mut self) {
        if let Some(ref refresh) = self.refresh {
            refresh.pause();
        }
    }

    fn resume_refresh_loop(&mut self) {
        if let Some(ref refresh) = self.refresh {
            refresh.resume();
        }
    }

    fn vlim_from_speed(&self, speed: f64) -> f64 {
        speed_to_vlim_rad_s(speed, self.max_joint_vel_rad_s)
    }

    /// Send a few MIT+gravity frames immediately when hardware is already enabled.
    fn prewarm_if_enabled(&mut self) -> Result<()> {
        if !self.mit_mode || self.gravity.is_none() {
            return Ok(());
        }
        if !lock_arm(&self.arm)?.hardware_all_enabled()? {
            return Ok(());
        }
        let joints = read_joint_feedback(&self.arm)?;
        for _ in 0..3 {
            self.send_joints_direct(&joints, 0.0)?;
            thread::sleep(STARTUP_BURST_PERIOD);
        }
        Ok(())
    }

    /// Hold current pose with gravity FF, then start the refresh loop.
    ///
    /// Sends several MIT frames before the background thread starts so the arm
    /// does not sag after enable (mode switch leaves kp=0/kd=0/tau=0 as the last
    /// command until gravity FF arrives).
    fn startup_mit_hold(&mut self) -> Result<()> {
        let joints = read_joint_feedback(&self.arm)?;
        for _ in 0..STARTUP_BURST_FRAMES {
            self.send_joints_direct(&joints, 0.0)?;
            thread::sleep(STARTUP_BURST_PERIOD);
        }
        self.start_refresh_loop()
    }

    fn gravity_ff_for(&self, joints: &[f64]) -> Option<Vec<f64>> {
        let guard = lock_arm(&self.arm).ok()?;
        compute_gravity_ff(&guard, self.gravity.as_ref(), joints)
    }

    /// Walk MIT `q_des` to `positions`. `speed` limits the first catch-up only.
    pub fn shape_arm_joints(&mut self, positions: &[f64], speed: f64) -> Result<()> {
        let target: Vec<f64> = positions.iter().take(NUM_ARM_JOINTS).copied().collect();
        if let Some(ref refresh) = self.refresh {
            refresh.set_sync_shape(target.clone(), self.vlim_from_speed(speed));
            self.commanded_joints = target;
            return Ok(());
        }
        self.send_joints_direct(&target, 0.0)
    }

    /// Latch latest `q*` for the 500 Hz FOH walker (coast at sample velocity).
    fn stream_arm_joints(&mut self, target: &[f64], vlim: f64) -> Result<()> {
        if let Some(ref refresh) = self.refresh {
            refresh.set_stream_goal(target.to_vec(), vlim);
            self.commanded_joints = target.to_vec();
            self.command_vlim = vlim;
            return Ok(());
        }
        self.send_joints_direct(target, vlim)
    }

    /// Freeze MIT at `positions` (no walker). Used for enable/startup and already-at-target.
    fn send_joints_direct(&mut self, positions: &[f64], vlim: f64) -> Result<()> {
        if self.mit_mode {
            if let Some(ref refresh) = self.refresh {
                refresh.set_hold(positions.to_vec(), vlim);
            } else {
                let zeros = vec![0.0; positions.len()];
                let tau_ff = self.gravity_ff_for(positions);
                lock_arm(&self.arm)?.send_arm_command(
                    positions,
                    Some(&zeros),
                    Some(VlimInput::Scalar(vlim)),
                    true,
                    tau_ff.as_deref(),
                )?;
            }
        } else {
            lock_arm(&self.arm)?.send_arm_command(
                positions,
                None,
                Some(VlimInput::Scalar(vlim)),
                true,
                None,
            )?;
        }
        self.commanded_joints = positions.to_vec();
        self.command_vlim = vlim;
        Ok(())
    }
}

impl Drop for ArxMCanRobot {
    fn drop(&mut self) {
        let _ = self.disconnect();
    }
}

fn read_joint_feedback(arm: &Arc<Mutex<ArxMCanArm>>) -> Result<Vec<f64>> {
    let guard = lock_arm(arm)?;
    let state = guard.read_state(true)?;
    Ok(state
        .arm
        .positions
        .into_iter()
        .take(NUM_ARM_JOINTS)
        .collect())
}
