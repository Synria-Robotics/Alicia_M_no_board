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
    pub shape_min_s: f64,
    pub shape_max_s: f64,
    pub stale_hold_s: f64,
}

impl Default for TeleopConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 100.0,
            follower_speed: 400.0,
            gripper_speed: 400.0,
            gripper_scale: 1.0,
            use_joint_mapping: true,
            max_joint_accel_rad_s2: None,
            max_joint_jerk_rad_s3: None,
            shape_min_s: 0.006,
            shape_max_s: 0.015,
            stale_hold_s: 0.050,
        }
    }
}

pub struct LeaderSample {
    pub joints: [f64; 6],
    pub gripper: f64,
    pub sequence: u64,
    pub captured_at_ns: u64,
}

#[derive(Debug, Clone, Default)]
pub struct TeleopStats {
    pub leader_samples: u64,
    pub duplicate_samples: u64,
    pub read_errors: u64,
    pub stale_events: u64,
    pub interval_mean_ms: f64,
    pub interval_max_ms: f64,
    pub last_sample_age_ms: f64,
    pub control_sent: u64,
    pub control_send_errors: u64,
    pub mailbox_overwrites: u64,
    pub control_overruns: u64,
    pub control_missed_deadlines: u64,
}

#[derive(Default)]
struct TeleopStatsState {
    snapshot: TeleopStats,
    interval_sum_s: f64,
    interval_count: u64,
    last_received: Option<Instant>,
}

pub struct TeleopSession {
    follower: Arc<Mutex<ArxMCanRobot>>,
    config: TeleopConfig,
    stop: Arc<Mutex<bool>>,
    stats: Arc<Mutex<TeleopStatsState>>,
    handle: Option<JoinHandle<()>>,
}

impl TeleopSession {
    pub fn new(follower: ArxMCanRobot, config: TeleopConfig) -> Self {
        Self::from_shared(Arc::new(Mutex::new(follower)), config)
    }

    /// Share the follower with the caller so joint reads stay available during teleop.
    pub fn from_shared(follower: Arc<Mutex<ArxMCanRobot>>, config: TeleopConfig) -> Self {
        Self {
            follower,
            config,
            stop: Arc::new(Mutex::new(false)),
            stats: Arc::new(Mutex::new(TeleopStatsState::default())),
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
        let stats = self.stats.clone();
        *self.stop.lock().unwrap() = false;
        *self.stats.lock().unwrap() = TeleopStatsState::default();
        self.handle = Some(
            thread::Builder::new()
                .name("arx-teleop".into())
                .spawn(move || teleop_loop(follower, stop, config, stats, read_leader))?,
        );
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

    pub fn stats(&self) -> TeleopStats {
        let mut guard = self.stats.lock().unwrap();
        if let Some(last) = guard.last_received {
            guard.snapshot.last_sample_age_ms = last.elapsed().as_secs_f64() * 1000.0;
        }
        if let Ok(robot) = self.follower.lock() {
            let (sent, errors) = robot.get_control_loop_stats();
            let (overwrites, overruns, missed) = robot.get_control_loop_timing_stats();
            guard.snapshot.control_sent = sent;
            guard.snapshot.control_send_errors = errors;
            guard.snapshot.mailbox_overwrites = overwrites;
            guard.snapshot.control_overruns = overruns;
            guard.snapshot.control_missed_deadlines = missed;
        }
        guard.snapshot.clone()
    }

    pub fn config(&self) -> &TeleopConfig {
        &self.config
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
    stats: Arc<Mutex<TeleopStatsState>>,
    mut read_leader: F,
) where
    F: FnMut() -> Result<Option<LeaderSample>>,
{
    let interval = Duration::from_secs_f64(1.0 / config.frequency_hz.max(1.0));
    // Joint targets are shaped on the 500 Hz loop. Accel limits stay on Ruckig only.
    let _ = (config.max_joint_accel_rad_s2, config.max_joint_jerk_rad_s3);
    // Gripper CAN writes share the bus with the 500 Hz joint stream.
    let gripper_period = Duration::from_millis(20);
    let mut last_gripper_send = Instant::now() - gripper_period;
    let mut last_gripper_cmd = f64::NAN;
    let mut last_sequence = 0_u64;
    let mut last_captured_at_ns = 0_u64;
    let mut last_received: Option<Instant> = None;
    let mut stale_active = true;
    let mut next_tick = Instant::now() + interval;
    while !*stop.lock().unwrap() {
        match read_leader() {
            Ok(Some(sample)) => {
                if sample.sequence != 0 && sample.sequence <= last_sequence {
                    stats.lock().unwrap().snapshot.duplicate_samples += 1;
                    continue;
                }
                let received = Instant::now();
                let receive_dt = last_received
                    .map(|last| received.duration_since(last).as_secs_f64())
                    .unwrap_or(interval.as_secs_f64());
                let frame_dt =
                    if sample.captured_at_ns > last_captured_at_ns && last_captured_at_ns != 0 {
                        (sample.captured_at_ns - last_captured_at_ns) as f64 * 1.0e-9
                    } else {
                        receive_dt
                    };
                let recovered = stale_active
                    || last_received
                        .map(|last| last.elapsed().as_secs_f64() >= config.stale_hold_s)
                        .unwrap_or(true);
                let mut joints = sample.joints;
                if config.use_joint_mapping {
                    joints = alicia_d_rad_to_alicia_m_rad(&joints);
                }
                let gripper = map_gripper(sample.gripper, config.gripper_scale);
                if let Ok(mut robot) = follower.lock() {
                    // First/recovered catch-up uses --speed. Fresh samples bridge
                    // over their measured interval on the 500 Hz refresh loop.
                    let _ = robot.shape_arm_joints_timed(
                        &joints,
                        config.follower_speed,
                        frame_dt,
                        config.shape_min_s,
                        config.shape_max_s,
                        recovered,
                    );
                    let gripper_due = !last_gripper_cmd.is_finite()
                        || ((gripper - last_gripper_cmd).abs() > 0.5
                            && last_gripper_send.elapsed() >= gripper_period);
                    if gripper_due {
                        let _ = robot.set_robot_state(
                            None,
                            Some(validate_gripper_value(gripper)),
                            config.follower_speed,
                            config.gripper_speed,
                            false,
                            None,
                            None,
                            None,
                            None,
                            false,
                            None,
                            None,
                        );
                        last_gripper_cmd = gripper;
                        last_gripper_send = Instant::now();
                    }
                }
                last_sequence = sample.sequence;
                last_captured_at_ns = sample.captured_at_ns;
                last_received = Some(received);
                stale_active = false;
                let mut guard = stats.lock().unwrap();
                guard.snapshot.leader_samples += 1;
                guard.interval_sum_s += frame_dt;
                guard.interval_count += 1;
                guard.snapshot.interval_mean_ms =
                    guard.interval_sum_s * 1000.0 / guard.interval_count as f64;
                guard.snapshot.interval_max_ms =
                    guard.snapshot.interval_max_ms.max(frame_dt * 1000.0);
                guard.snapshot.last_sample_age_ms = 0.0;
                guard.last_received = Some(received);
            }
            Ok(None) => {
                if !stale_active
                    && last_received
                        .map(|last| last.elapsed().as_secs_f64() >= config.stale_hold_s)
                        .unwrap_or(false)
                {
                    stale_active = true;
                    stats.lock().unwrap().snapshot.stale_events += 1;
                }
            }
            Err(_) => {
                stats.lock().unwrap().snapshot.read_errors += 1;
            }
        }
        let now = Instant::now();
        if now < next_tick {
            thread::sleep(next_tick - now);
            next_tick += interval;
        } else {
            while next_tick <= now {
                next_tick += interval;
            }
        }
    }
}
