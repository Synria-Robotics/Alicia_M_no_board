//! Periodic MIT/PV command refresh with optional gravity feedforward and Ruckig trajectories.

use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use crate::dynamics::GravityCompensator;
use crate::hardware::ArxMCanArm;
use crate::trajectory::governor::ReferenceGovernor;
use crate::trajectory::{JointTrajectoryPlanner, TrajectoryLimits};
use crate::types::VlimInput;

enum MotionCommand {
    Hold { q: Vec<f64>, vlim: f64 },
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
            command: Some(MotionCommand::Hold {
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
                let mut hold_q = initial_joints;
                let mut hold_vlim = initial_vlim;
                let mut traj_vlim = initial_vlim;

                while !*loop_stop.lock().unwrap() {
                    let t0 = std::time::Instant::now();

                    if let Some(cmd) = loop_state.lock().unwrap().command.take() {
                        match cmd {
                            MotionCommand::Hold { q, vlim } => {
                                planner = None;
                                hold_q = q;
                                hold_vlim = vlim;
                            }
                            MotionCommand::StartTrajectory {
                                q_start,
                                q_goal,
                                dq_start,
                                limits,
                                vlim,
                            } => {
                                traj_vlim = vlim;
                                match JointTrajectoryPlanner::new(
                                    &q_start,
                                    &q_goal,
                                    &dq_start,
                                    &limits,
                                    hz,
                                ) {
                                    Ok(p) => {
                                        if let Some(gov) = governor.as_mut() {
                                            gov.reset();
                                        }
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
                                    Ok((q, dq, done)) => {
                                        if done {
                                            hold_q = q.clone();
                                            hold_vlim = traj_vlim;
                                            planner = None;
                                        }
                                        (q, dq, traj_vlim)
                                    }
                                    Err(_) => return Err(()),
                                }
                            } else {
                                (
                                    hold_q.clone(),
                                    vec![0.0; hold_q.len()],
                                    hold_vlim,
                                )
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

    pub fn set_hold(&self, joints: Vec<f64>, vlim: f64) {
        self.enqueue(MotionCommand::Hold { q: joints, vlim });
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
