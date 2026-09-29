//! 500 Hz linear bridge between teleop samples.
//!
//! Same role as the STM32 sync shaper (`configure_joint_sync_shaper` + `Divspos`):
//! a new leader pose is the endpoint, and `q_des` walks there one control tick
//! at a time. The segment is not scaled by `--speed`, and it stops on the
//! endpoint instead of coasting past it. MIT `dq` stays 0; smoothness is the
//! position ramp.

/// Shortest bridge. A shorter gap would be an instant step.
/// Matches STM32 `k_sync_shape_min_ms`.
pub const SYNC_SHAPE_MIN_S: f64 = 0.004;
/// Longest bridge. STM32 `k_sync_shape_max_ms`: a longer ramp feels soft.
pub const SYNC_SHAPE_MAX_S: f64 = 0.008;
/// First sample, before an inter-arrival time exists. STM32 nominal 5 ms.
pub const SYNC_SHAPE_DEFAULT_S: f64 = 0.005;

/// Firmware `motorTypeKp` / `motorTypeKd` for the Alicia-D sync session.
/// Teleop sends these with `tau = 0`. Profile YAML gains stay on other motions.
const FIRMWARE_SYNC_KP: [f64; 6] = [500.0, 500.0, 500.0, 20.0, 60.0, 150.0];
const FIRMWARE_SYNC_KD: [f64; 6] = [5.0, 5.0, 5.0, 1.0, 2.0, 2.0];

pub fn firmware_sync_gains(n: usize) -> (Vec<f64>, Vec<f64>) {
    let kp = (0..n)
        .map(|i| FIRMWARE_SYNC_KP.get(i).copied().unwrap_or(20.0))
        .collect();
    let kd = (0..n)
        .map(|i| FIRMWARE_SYNC_KD.get(i).copied().unwrap_or(1.0))
        .collect();
    (kp, kd)
}

#[derive(Debug, Clone)]
pub struct SyncSegment {
    pub goal: Vec<f64>,
    pub step: Vec<f64>,
}

pub fn begin_sync_segment(
    q_des: &[f64],
    goal: &[f64],
    frame_dt: f64,
    tick_dt: f64,
) -> SyncSegment {
    let n = q_des.len().max(goal.len());
    let frame_dt = if frame_dt.is_finite() && frame_dt > 0.0 {
        frame_dt.clamp(SYNC_SHAPE_MIN_S, SYNC_SHAPE_MAX_S)
    } else {
        SYNC_SHAPE_DEFAULT_S
    };
    let tick_dt = if tick_dt.is_finite() && tick_dt > 0.0 {
        tick_dt
    } else {
        0.002
    };
    let ticks = (frame_dt / tick_dt).max(1.0);
    let mut out_goal = vec![0.0; n];
    let mut step = vec![0.0; n];
    for i in 0..n {
        let qd = q_des.get(i).copied().unwrap_or(0.0);
        let qg = goal.get(i).copied().unwrap_or(qd);
        out_goal[i] = qg;
        step[i] = (qg - qd) / ticks;
    }
    SyncSegment {
        goal: out_goal,
        step,
    }
}

/// Walk toward `goal` at `vlim` rad/s. The first teleop catch-up uses this so a
/// large pose error is not crossed inside the 4–8 ms tracking window.
pub fn begin_approach_segment(
    q_des: &[f64],
    goal: &[f64],
    vlim: f64,
    tick_dt: f64,
) -> SyncSegment {
    let n = q_des.len().max(goal.len());
    let tick_dt = if tick_dt.is_finite() && tick_dt > 0.0 {
        tick_dt
    } else {
        0.002
    };
    let max_step = if vlim.is_finite() && vlim > 0.0 {
        vlim * tick_dt
    } else {
        0.0
    };
    let mut out_goal = vec![0.0; n];
    let mut step = vec![0.0; n];
    for i in 0..n {
        let qd = q_des.get(i).copied().unwrap_or(0.0);
        let qg = goal.get(i).copied().unwrap_or(qd);
        let delta = qg - qd;
        out_goal[i] = qg;
        step[i] = if max_step <= 0.0 {
            0.0
        } else {
            delta.clamp(-max_step, max_step)
        };
    }
    SyncSegment {
        goal: out_goal,
        step,
    }
}

/// Advance one tick. Returns true when every joint is on the goal.
pub fn step_sync_segment(q_des: &mut [f64], seg: &SyncSegment) -> bool {
    let mut arrived = true;
    for i in 0..q_des.len() {
        let goal = seg.goal.get(i).copied().unwrap_or(q_des[i]);
        let step = seg.step.get(i).copied().unwrap_or(0.0);
        let remain = goal - q_des[i];
        if remain == 0.0
            || (step != 0.0 && (remain.abs() <= step.abs() || remain * step <= 0.0))
        {
            q_des[i] = goal;
        } else {
            q_des[i] += step;
            arrived = false;
        }
    }
    arrived
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn walks_to_goal_without_overshoot() {
        let tick = 0.002;
        let frame = 0.010;
        let mut q = vec![0.0, 0.0];
        let seg = begin_sync_segment(&q, &[0.05, -0.02], frame, tick);
        let mut arrived = false;
        for _ in 0..20 {
            let before = q.clone();
            arrived = step_sync_segment(&mut q, &seg);
            assert!(q[0] + 1e-12 >= before[0]);
            assert!(q[0] <= 0.05 + 1e-12);
            assert!(q[1] <= before[1] + 1e-12);
            assert!(q[1] + 1e-12 >= -0.02);
            if arrived {
                break;
            }
        }
        assert!(arrived);
        assert!((q[0] - 0.05).abs() < 1e-12);
        assert!((q[1] + 0.02).abs() < 1e-12);
    }

    #[test]
    fn does_not_snap_on_the_first_tick() {
        let mut q = vec![0.0];
        let seg = begin_sync_segment(&q, &[0.10], 0.010, 0.002);
        let arrived = step_sync_segment(&mut q, &seg);
        assert!(!arrived);
        assert!(q[0] > 0.0);
        assert!(q[0] < 0.03);
    }

    #[test]
    fn restarts_from_the_current_command() {
        let mut q = vec![0.04];
        let seg = begin_sync_segment(&q, &[0.10], 0.008, 0.002);
        step_sync_segment(&mut q, &seg);
        assert!(q[0] > 0.04);
        assert!(q[0] < 0.10);
    }

    #[test]
    fn long_gap_clamps_to_firmware_window() {
        let slow = begin_sync_segment(&[0.0], &[0.08], 0.040, 0.002);
        let fast = begin_sync_segment(&[0.0], &[0.08], 0.008, 0.002);
        assert!((slow.step[0] - fast.step[0]).abs() < 1e-12);
        assert!((slow.step[0] - 0.02).abs() < 1e-12);
    }

    #[test]
    fn approach_step_scales_with_speed() {
        let slow = begin_approach_segment(&[0.0], &[1.0], 0.6, 0.002);
        let fast = begin_approach_segment(&[0.0], &[1.0], 1.2, 0.002);
        assert!((slow.step[0] - 0.0012).abs() < 1e-12);
        assert!((fast.step[0] - 0.0024).abs() < 1e-12);
        let mut q = vec![0.0];
        assert!(!step_sync_segment(&mut q, &slow));
        assert!((q[0] - 0.0012).abs() < 1e-12);
    }

    #[test]
    fn firmware_gains_match_motor_type_table() {
        let (kp, kd) = firmware_sync_gains(6);
        assert_eq!(kp, vec![500.0, 500.0, 500.0, 20.0, 60.0, 150.0]);
        assert_eq!(kd, vec![5.0, 5.0, 5.0, 1.0, 2.0, 2.0]);
    }
}
