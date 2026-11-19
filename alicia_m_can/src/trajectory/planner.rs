use ruckig::{InputParameter, OutputParameter, Result as RuckigResult, Ruckig, Synchronization};

use crate::error::{ArxError, Result};
use crate::trajectory::limits::TrajectoryLimits;

pub struct JointTrajectoryPlanner {
    otg: Ruckig,
    input: InputParameter,
    output: OutputParameter,
    dof: usize,
    finished: bool,
    goal: Vec<f64>,
}

impl JointTrajectoryPlanner {
    pub fn new(
        q_start: &[f64],
        q_goal: &[f64],
        dq_start: &[f64],
        limits: &TrajectoryLimits,
        control_hz: f64,
    ) -> Result<Self> {
        let dof = q_start.len().min(q_goal.len());
        if dof == 0 {
            return Err(ArxError::InvalidArgument("trajectory requires joints".into()));
        }
        let dt = 1.0 / control_hz.max(1.0);
        let mut input = InputParameter::new(dof);
        // One shared time law: q(t) = q0 + s(t)(q1-q0). Falls back to Time if
        // Phase is infeasible. Independent per-joint profiles look stepwise on
        // large asymmetric moves (home ↔ a folded pose).
        input.synchronization = Synchronization::Phase;
        for i in 0..dof {
            input.current_position[i] = q_start[i];
            input.current_velocity[i] = dq_start.get(i).copied().unwrap_or(0.0);
            input.current_acceleration[i] = 0.0;
            input.target_position[i] = q_goal[i];
            input.target_velocity[i] = 0.0;
            input.target_acceleration[i] = 0.0;
            input.max_velocity[i] = limits.max_velocity.get(i).copied().unwrap_or(0.0);
            input.max_acceleration[i] = limits.max_acceleration.get(i).copied().unwrap_or(0.0);
            input.max_jerk[i] = limits.max_jerk.get(i).copied().unwrap_or(0.0);
        }

        Ok(Self {
            otg: Ruckig::new_direct(dof, dt),
            input,
            output: OutputParameter::new_direct(dof),
            dof,
            finished: false,
            goal: q_goal.iter().take(dof).copied().collect(),
        })
    }

    /// Keep current `q/dq/ddq` and replan toward a new goal (online teleop).
    pub fn retarget(
        &mut self,
        q_goal: &[f64],
        dq_goal: Option<&[f64]>,
        limits: &TrajectoryLimits,
    ) {
        self.apply_limits(limits);
        let n = self.dof.min(q_goal.len());
        for i in 0..n {
            let vmax = self.input.max_velocity[i];
            self.input.target_position[i] = q_goal[i];
            let dq = dq_goal.and_then(|d| d.get(i).copied()).unwrap_or(0.0);
            self.input.target_velocity[i] = dq.clamp(-vmax, vmax);
            self.input.target_acceleration[i] = 0.0;
        }
        self.goal = q_goal.iter().take(self.dof).copied().collect();
        self.finished = false;
    }

    fn apply_limits(&mut self, limits: &TrajectoryLimits) {
        for i in 0..self.dof {
            self.input.max_velocity[i] = limits.max_velocity.get(i).copied().unwrap_or(0.0).max(1e-6);
            self.input.max_acceleration[i] =
                limits.max_acceleration.get(i).copied().unwrap_or(0.0).max(1e-6);
            self.input.max_jerk[i] = limits.max_jerk.get(i).copied().unwrap_or(0.0).max(1e-6);
        }
    }

    pub fn step(&mut self) -> Result<(Vec<f64>, Vec<f64>, bool)> {
        if self.finished {
            let q = self.goal.clone();
            let dq = vec![0.0; self.dof];
            return Ok((q, dq, true));
        }

        match self.otg.update(&mut self.input, &mut self.output) {
            RuckigResult::Working => {
                let q = self.output.new_position.clone();
                let dq = self.output.new_velocity.clone();
                self.output.pass_to_input(&mut self.input);
                Ok((q, dq, false))
            }
            RuckigResult::Finished => {
                let q = self.output.new_position.clone();
                let dq = self.output.new_velocity.clone();
                self.output.pass_to_input(&mut self.input);
                self.finished = true;
                Ok((q, dq, true))
            }
            RuckigResult::Error
            | RuckigResult::ErrorInvalidInput
            | RuckigResult::ErrorTrajectoryDuration
            | RuckigResult::ErrorPositionalLimits
            | RuckigResult::ErrorZeroLimits
            | RuckigResult::ErrorExecutionTimeCalculation
            | RuckigResult::ErrorSynchronizationCalculation => Err(ArxError::Runtime(
                "ruckig trajectory generation failed".into(),
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trajectory::limits::TrajectoryLimits;

    #[test]
    fn reaches_goal_with_zero_terminal_velocity() {
        let limits = TrajectoryLimits {
            max_velocity: vec![1.0; 6],
            max_acceleration: vec![8.0; 6],
            max_jerk: vec![50.0; 6],
        };
        let start = vec![0.0; 6];
        let goal = vec![0.5; 6];
        let mut planner =
            JointTrajectoryPlanner::new(&start, &goal, &[], &limits, 500.0).unwrap();
        let mut last_q = start;
        for _ in 0..5000 {
            let (q, dq, done) = planner.step().unwrap();
            last_q = q;
            assert!(dq.iter().all(|v| v.abs() <= limits.max_velocity[0] + 1e-6));
            if done {
                break;
            }
        }
        for (q, g) in last_q.iter().zip(goal.iter()) {
            assert!((q - g).abs() < 1e-3);
        }
    }

    #[test]
    fn retarget_reaches_new_goal_from_current_state() {
        let limits = TrajectoryLimits {
            max_velocity: vec![1.0; 6],
            max_acceleration: vec![8.0; 6],
            max_jerk: vec![50.0; 6],
        };
        let start = vec![0.0; 6];
        let mut planner =
            JointTrajectoryPlanner::new(&start, &[0.8; 6], &[], &limits, 500.0).unwrap();
        for _ in 0..40 {
            let _ = planner.step().unwrap();
        }
        let new_goal = vec![0.15; 6];
        planner.retarget(&new_goal, None, &limits);
        let mut last_q = start;
        let mut done = false;
        for _ in 0..5000 {
            let (q, dq, finished) = planner.step().unwrap();
            last_q = q;
            assert!(dq.iter().all(|v| v.abs() <= limits.max_velocity[0] + 1e-6));
            if finished {
                done = true;
                break;
            }
        }
        assert!(done);
        for (q, g) in last_q.iter().zip(new_goal.iter()) {
            assert!((q - g).abs() < 1e-3);
        }
    }

    #[test]
    fn phase_keeps_asymmetric_progress_aligned() {
        let limits = TrajectoryLimits {
            max_velocity: vec![1.0; 6],
            max_acceleration: vec![8.0; 6],
            max_jerk: vec![50.0; 6],
        };
        let start = vec![0.0; 6];
        let goal = vec![0.80, 0.16, 0.0, 0.0, 0.40, 0.0];
        let mut planner =
            JointTrajectoryPlanner::new(&start, &goal, &[], &limits, 500.0).unwrap();
        let mut samples = 0;
        for _ in 0..5000 {
            let (q, _, done) = planner.step().unwrap();
            let s0 = q[0] / goal[0];
            if (0.15..=0.85).contains(&s0) {
                samples += 1;
                assert!((q[1] / goal[1] - s0).abs() < 0.04);
                assert!((q[4] / goal[4] - s0).abs() < 0.04);
                assert!(q[2].abs() < 1e-4);
            }
            if done {
                break;
            }
        }
        assert!(samples > 10);
    }
}
