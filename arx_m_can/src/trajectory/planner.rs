use ruckig::{InputParameter, OutputParameter, Result as RuckigResult, Ruckig};

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
}
