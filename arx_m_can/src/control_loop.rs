//! Periodic MIT/PV command refresh with gravity feedforward.
//!
//! Stream teleop latches `q_goal` and coasts `q_des` at the 60 Hz sample
//! velocity (first-order hold + accel-limited `dq`; see `trajectory::walker`).
//! Discrete goto still uses Ruckig `StartTrajectory`.

use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use crate::dynamics::GravityCompensator;
use crate::hardware::ArxMCanArm;
use crate::trajectory::governor::ReferenceGovernor;
use crate::trajectory::walker::{note_stream_goal, step_joint_des};
use crate::trajectory::{JointTrajectoryPlanner, TrajectoryLimits};
use crate::types::VlimInput;

enum MotionCommand {
    Freeze {
        q: Vec<f64>,
        vlim: f64,
    },
    StreamGoal {
        q: Vec<f64>,
        vlim: f64,
    },
    StartTrajectory {
        q_start: Vec<f64>,
        q_goal: Vec<f64>,
        dq_start: Vec<f64>,
        limits: TrajectoryLimits,
        vlim: f64,
    },
}

struct LoopState {
    command: Option<MotionCommand>,
    active: bool,
    sent_commands: u64,
    send_errors: u64,
}

pub struct CommandRefreshLoop {
    state: Arc<Mutex<LoopState>>,
    stop: Arc<Mutex<bool>>,
    gravity: Option<Arc<Mutex<GravityCompensator>>>,
    handle: Option<JoinHandle<()>>,
}

impl CommandRefreshLoop {
    pub fn new(
        arm: Arc<Mutex<ArxMCanArm>>,
        hz: f64,
        mit_mode: bool,
        initial_joints: Vec<f64>,
        initial_vlim: f64,
        gravity: Option<Arc<Mutex<GravityCompensator>>>,
        governor: Option<ReferenceGovernor>,
    ) -> Self {
        let state = Arc::new(Mutex::new(LoopState {
            command: Some(MotionCommand::Freeze {
                q: initial_joints.clone(),
                vlim: initial_vlim,
            }),
            active: false,
            sent_commands: 0,
            send_errors: 0,
        }));
        let stop = Arc::new(Mutex::new(false));
        Self {
            state,
            stop,
            gravity,
            handle: None,
        }
        .spawn(arm, hz, mit_mode, initial_joints, initial_vlim, governor)
    }

    fn spawn(
        mut self,
        arm: Arc<Mutex<ArxMCanArm>>,
        hz: f64,
        mit_mode: bool,
        initial_joints: Vec<f64>,
        initial_vlim: f64,
        mut governor: Option<ReferenceGovernor>,
    ) -> Self {
        let loop_state = self.state.clone();
        let loop_stop = self.stop.clone();
        let gravity = self.gravity.clone();
        let period = Duration::from_secs_f64(1.0 / hz.max(1.0));
        let dt = period.as_secs_f64();
        let handle = thread::Builder::new()
            .name("arx-m-can-refresh".into())
            .spawn(move || {
                let mut planner: Option<JointTrajectoryPlanner> = None;
                let mut q_des = initial_joints.clone();
                let mut q_goal = initial_joints;
                let mut hold_vlim = initial_vlim;
                let mut traj_vlim = initial_vlim;
                let mut walk_vlim = initial_vlim;
                let mut last_dq = vec![0.0; q_des.len()];
                let mut v_stream = vec![0.0; q_des.len()];
                let mut last_goal_at: Option<std::time::Instant> = None;
                let mut goal_age = 0.0;
                let mut walking = false;

                while !*loop_stop.lock().unwrap() {
                    let t0 = std::time::Instant::now();

                    if let Some(cmd) = loop_state.lock().unwrap().command.take() {
                        match cmd {
                            MotionCommand::Freeze { q, vlim } => {
                                planner = None;
                                walking = false;
                                q_des = q.clone();
                                q_goal = q;
                                last_dq = vec![0.0; q_des.len()];
                                v_stream = vec![0.0; q_des.len()];
                                last_goal_at = None;
                                goal_age = 0.0;
                                hold_vlim = vlim;
                                walk_vlim = vlim;
                            }
                            MotionCommand::StreamGoal { q, vlim } => {
                                planner = None;
                                walking = true;
                                walk_vlim = vlim;
                                let now = std::time::Instant::now();
                                let dt_goal = last_goal_at
                                    .map(|t| now.duration_since(t).as_secs_f64())
                                    .unwrap_or(1.0 / 60.0);
                                let q = pad_to_len(q, q_des.len());
                                note_stream_goal(&mut q_goal, &mut v_stream, &q, dt_goal, vlim);
                                last_goal_at = Some(now);
                                goal_age = 0.0;
                            }
                            MotionCommand::StartTrajectory {
                                q_start,
                                q_goal: goal,
                                dq_start,
                                limits,
                                vlim,
                            } => {
                                traj_vlim = vlim;
                                walking = false;
                                v_stream.fill(0.0);
                                last_goal_at = None;
                                goal_age = 0.0;
                                match JointTrajectoryPlanner::new(
                                    &q_start, &goal, &dq_start, &limits, hz,
                                ) {
                                    Ok(p) => {
                                        if let Some(gov) = governor.as_mut() {
                                            gov.reset();
                                        }
                                        q_goal = pad_to_len(goal, q_des.len());
                                        planner = Some(p);
                                    }
                                    Err(_) => {
                                        let mut guard = loop_state.lock().unwrap();
                                        guard.send_errors += 1;
                                    }
                                }
                            }
                        }
                    }

                    let active = loop_state.lock().unwrap().active;
                    if active {
                        let send_result = (|| {
                            let (q_ideal, dq_ideal, vlim) = if let Some(p) = planner.as_mut() {
                                match p.step() {
                                    Ok((q, dq, _done)) => {
                                        q_des = q.clone();
                                        last_dq = dq.clone();
                                        (q, dq, traj_vlim)
                                    }
                                    Err(_) => (q_des.clone(), last_dq.clone(), traj_vlim),
                                }
                            } else if walking {
                                let dq = step_joint_des(
                                    &mut q_des,
                                    &q_goal,
                                    &mut last_dq,
                                    &v_stream,
                                    walk_vlim,
                                    dt,
                                    goal_age,
                                );
                                goal_age += dt;
                                (q_des.clone(), dq, walk_vlim)
                            } else {
                                (q_des.clone(), vec![0.0; q_des.len()], hold_vlim)
                            };

                            let mut arm = arm.lock().map_err(|_| ())?;
                            let (joints, velocities, tau_ff) = if mit_mode {
                                if let Some(gov) = governor.as_mut() {
                                    let state = arm.read_state(false).map_err(|_| ())?;
                                    let q_meas = &state.arm.positions;
                                    let tau_ff =
                                        compute_gravity_ff(&arm, gravity.as_ref(), q_meas);
                                    let (q_send, dq_send) = gov.apply(
                                        &q_ideal,
                                        &dq_ideal,
                                        q_meas,
                                        tau_ff.as_deref(),
                                        dt,
                                    );
                                    (q_send, dq_send, tau_ff)
                                } else {
                                    let tau_ff =
                                        compute_gravity_ff(&arm, gravity.as_ref(), &q_ideal);
                                    (q_ideal, dq_ideal, tau_ff)
                                }
                            } else {
                                (q_ideal, dq_ideal, None)
                            };
                            let vel = mit_mode.then(|| velocities.as_slice());
                            arm.send_arm_command(
                                &joints,
                                vel,
                                Some(VlimInput::Scalar(vlim)),
                                true,
                                tau_ff.as_deref(),
                            )
                            .map_err(|_| ())
                        })();

                        let mut guard = loop_state.lock().unwrap();
                        match send_result {
                            Ok(()) => guard.sent_commands += 1,
                            Err(()) => guard.send_errors += 1,
                        }
                    }

                    let elapsed = t0.elapsed();
                    if elapsed < period {
                        thread::sleep(period - elapsed);
                    }
                }
            })
            .expect("spawn control refresh thread");
        self.handle = Some(handle);
        self
    }

    fn enqueue(&self, command: MotionCommand) {
        let mut guard = self.state.lock().unwrap();
        guard.command = Some(command);
    }

    /// Freeze MIT at `joints` (`dq = 0`). Sit-still / enable / already-at-target.
    pub fn set_hold(&self, joints: Vec<f64>, vlim: f64) {
        self.enqueue(MotionCommand::Freeze { q: joints, vlim })
    }

    /// Latch teleop `q*` and coast `q_des` (FOH) at `vlim` on the refresh thread.
    pub fn set_stream_goal(&self, joints: Vec<f64>, vlim: f64) {
        self.enqueue(MotionCommand::StreamGoal { q: joints, vlim });
    }

    pub fn start_trajectory(
        &self,
        q_start: &[f64],
        q_goal: &[f64],
        dq_start: &[f64],
        limits: &TrajectoryLimits,
        vlim: f64,
    ) -> crate::error::Result<()> {
        self.enqueue(MotionCommand::StartTrajectory {
            q_start: q_start.to_vec(),
            q_goal: q_goal.to_vec(),
            dq_start: dq_start.to_vec(),
            limits: limits.clone(),
            vlim,
        });
        Ok(())
    }

    pub fn stats(&self) -> (u64, u64) {
        let guard = self.state.lock().unwrap();
        (guard.sent_commands, guard.send_errors)
    }

    pub fn resume(&self) {
        self.state.lock().unwrap().active = true;
    }

    pub fn pause(&self) {
        self.state.lock().unwrap().active = false;
    }

    pub fn stop(&mut self) {
        *self.stop.lock().unwrap() = true;
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for CommandRefreshLoop {
    fn drop(&mut self) {
        self.stop();
    }
}

fn pad_to_len(mut q: Vec<f64>, n: usize) -> Vec<f64> {
    if q.len() < n {
        q.resize(n, 0.0);
    } else if q.len() > n {
        q.truncate(n);
    }
    q
}

pub(crate) fn compute_gravity_ff(
    arm: &ArxMCanArm,
    gravity: Option<&Arc<Mutex<GravityCompensator>>>,
    target_joints: &[f64],
) -> Option<Vec<f64>> {
    let compensator = gravity?;
    let state = arm.read_state(false).ok()?;
    let measured = &state.arm.positions;
    let gripper_q = state.gripper.as_ref().map(|g| g.position);
    let mut guard = compensator.lock().ok()?;
    guard
        .compute_arm_gravity_ff(measured, gripper_q)
        .ok()
        .or_else(|| guard.compute_arm_gravity_ff(target_joints, gripper_q).ok())
}
