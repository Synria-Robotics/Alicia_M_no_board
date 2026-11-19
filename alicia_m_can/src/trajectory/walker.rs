//! Stream teleop walker: first-order hold of the 60 Hz `q*` latch at 500 Hz.
//!
//! While the stream is live, `q_des` coasts at the finite-difference velocity of
//! successive latches plus a soft catch-up to the latest sample. It does **not**
//! clamp onto `q_goal` (that was the 60 Hz go-stop-go staircase).
//!
//! If no new latch arrives for [`STREAM_STALE_S`], treat the last sample as a
//! real stop: brake with `amax` and settle. The window is longer than one
//! leader serial round-trip: Alicia-D joint state is request/response, so a
//! 25 ms gap is a late reply, not a stop. Braking on that gap is the teleop
//! stutter. Accel-limit `dq` so Damiao MIT is not a square wave.

use crate::trajectory::limits::stream_amax_from_vlim;

pub const SETTLED_EPS_RAD: f64 = 5e-4;
pub const STREAM_STALE_S: f64 = 0.10;
const CATCHUP_TAU_S: f64 = 0.04;
const SETTLED_DQ_EPS: f64 = 1e-3;
const DT_GOAL_DEFAULT: f64 = 1.0 / 60.0;
const DT_GOAL_MIN: f64 = 0.008;
const DT_GOAL_MAX: f64 = 0.05;

/// Finite-difference velocity of the command stream. Isotropic-clamped to `vlim`.
pub fn note_stream_goal(
    q_goal: &mut Vec<f64>,
    v_stream: &mut Vec<f64>,
    q_new: &[f64],
    dt_goal: f64,
    vlim: f64,
) {
    let n = q_goal.len().max(q_new.len());
    if q_goal.len() != n {
        q_goal.resize(n, 0.0);
    }
    if v_stream.len() != n {
        v_stream.resize(n, 0.0);
    }
    let dt_goal = if dt_goal.is_finite() && dt_goal > 0.0 {
        dt_goal.clamp(DT_GOAL_MIN, DT_GOAL_MAX)
    } else {
        DT_GOAL_DEFAULT
    };
    let vlim = vlim.max(0.0);
    for i in 0..n {
        let qn = q_new.get(i).copied().unwrap_or(q_goal[i]);
        v_stream[i] = (qn - q_goal[i]) / dt_goal;
        q_goal[i] = qn;
    }
    scale_isotropic(v_stream, vlim);
}

/// Advance `q_des` one MIT tick. Updates `last_dq` and returns it for the motor.
pub fn step_joint_des(
    q_des: &mut [f64],
    q_goal: &[f64],
    last_dq: &mut Vec<f64>,
    v_stream: &[f64],
    vlim: f64,
    dt: f64,
    goal_age: f64,
) -> Vec<f64> {
    let amax = stream_amax_from_vlim(vlim);
    step_joint_des_with_amax(q_des, q_goal, last_dq, v_stream, vlim, dt, goal_age, amax)
}

fn scale_isotropic(v: &mut [f64], vlim: f64) {
    if vlim <= 0.0 {
        v.fill(0.0);
        return;
    }
    let mut max_abs = 0.0_f64;
    for x in v.iter() {
        max_abs = max_abs.max(x.abs());
    }
    if max_abs > vlim {
        let s = max_abs / vlim;
        for x in v.iter_mut() {
            *x /= s;
        }
    }
}

fn step_joint_des_with_amax(
    q_des: &mut [f64],
    q_goal: &[f64],
    last_dq: &mut Vec<f64>,
    v_stream: &[f64],
    vlim: f64,
    dt: f64,
    goal_age: f64,
    amax: f64,
) -> Vec<f64> {
    let n = q_des.len();
    if last_dq.len() != n {
        last_dq.resize(n, 0.0);
    }
    let vlim = vlim.max(0.0);
    let amax = amax.max(0.0);
    let dt = dt.max(0.0);
    if n == 0 || dt == 0.0 {
        last_dq.fill(0.0);
        return vec![0.0; n];
    }

    let mut delta = vec![0.0; n];
    let mut max_abs = 0.0_f64;
    for i in 0..n {
        let goal = q_goal.get(i).copied().unwrap_or(q_des[i]);
        delta[i] = goal - q_des[i];
        max_abs = max_abs.max(delta[i].abs());
    }

    let stale = goal_age >= STREAM_STALE_S;
    let dv_max = amax * dt;
    let tau = CATCHUP_TAU_S.max(dt);

    if stale && max_abs <= SETTLED_EPS_RAD {
        for i in 0..n {
            q_des[i] = q_goal.get(i).copied().unwrap_or(q_des[i]);
        }
        for v in last_dq.iter_mut() {
            *v += (0.0 - *v).clamp(-dv_max, dv_max);
            if v.abs() <= SETTLED_DQ_EPS {
                *v = 0.0;
            }
        }
        return vec![0.0; n];
    }

    let mut dq_des = vec![0.0; n];
    for i in 0..n {
        let catch = delta[i] / tau;
        dq_des[i] = if stale {
            catch
        } else {
            v_stream.get(i).copied().unwrap_or(0.0) + catch
        };
    }
    scale_isotropic(&mut dq_des, vlim);

    let mut dq = vec![0.0; n];
    for i in 0..n {
        let mut v = last_dq[i] + (dq_des[i] - last_dq[i]).clamp(-dv_max, dv_max);
        v = v.clamp(-vlim, vlim);
        let step = v * dt;
        if stale && delta[i] * v > 0.0 && step.abs() >= delta[i].abs() {
            q_des[i] += delta[i];
            v = 0.0;
        } else {
            q_des[i] += step;
        }
        last_dq[i] = v;
        dq[i] = v;
    }
    dq
}

#[cfg(test)]
mod tests {
    use super::{note_stream_goal, step_joint_des, step_joint_des_with_amax, STREAM_STALE_S};
    use crate::trajectory::limits::stream_amax_from_vlim;

    fn live(
        q_des: &mut [f64],
        q_goal: &[f64],
        last_dq: &mut Vec<f64>,
        v_stream: &[f64],
        vlim: f64,
        dt: f64,
    ) -> Vec<f64> {
        step_joint_des(q_des, q_goal, last_dq, v_stream, vlim, dt, 0.0)
    }

    fn stale(
        q_des: &mut [f64],
        q_goal: &[f64],
        last_dq: &mut Vec<f64>,
        vlim: f64,
        dt: f64,
    ) -> Vec<f64> {
        step_joint_des(
            q_des,
            q_goal,
            last_dq,
            &[0.0],
            vlim,
            dt,
            STREAM_STALE_S + 1e-6,
        )
    }

    #[test]
    fn note_stream_goal_is_finite_difference() {
        let mut q_goal = vec![0.0, 0.0];
        let mut v_stream = vec![0.0, 0.0];
        note_stream_goal(&mut q_goal, &mut v_stream, &[0.016, 0.004], 0.016, 10.0);
        assert!((q_goal[0] - 0.016).abs() < 1e-12);
        assert!((v_stream[0] - 1.0).abs() < 1e-12);
        assert!((v_stream[1] - 0.25).abs() < 1e-12);
    }

    #[test]
    fn note_stream_goal_isotropic_vlim() {
        let mut q_goal = vec![0.0, 0.0];
        let mut v_stream = vec![0.0, 0.0];
        note_stream_goal(&mut q_goal, &mut v_stream, &[1.0, 0.25], 0.016, 0.5);
        assert!((v_stream[0] - 0.5).abs() < 1e-12);
        assert!((v_stream[1] - 0.125).abs() < 1e-12);
    }

    #[test]
    fn isotropic_scales_by_max_axis() {
        let mut q_des = vec![0.0, 0.0];
        let q_goal = vec![1.0, 0.25];
        let mut last_dq = vec![0.0, 0.0];
        let vlim = 0.5;
        let dt = 0.02;
        let dq = step_joint_des_with_amax(
            &mut q_des,
            &q_goal,
            &mut last_dq,
            &[0.0, 0.0],
            vlim,
            dt,
            0.0,
            1e9,
        );
        let max_delta = vlim * dt;
        assert!((q_des[0] - max_delta).abs() < 1e-12);
        assert!((q_des[1] - 0.25 * max_delta).abs() < 1e-12);
        assert!((dq[0] - vlim).abs() < 1e-12);
        assert!((dq[1] - 0.25 * vlim).abs() < 1e-12);
    }

    #[test]
    fn first_tick_from_rest_is_amax_dt_not_vlim() {
        let mut q_des = vec![0.0];
        let q_goal = vec![1.0];
        let mut last_dq = vec![0.0];
        let vlim = 0.6;
        let dt = 0.002;
        let amax = stream_amax_from_vlim(vlim);
        let dq = live(&mut q_des, &q_goal, &mut last_dq, &[0.0], vlim, dt);
        assert!((dq[0] - amax * dt).abs() < 1e-12);
        assert!(dq[0] < vlim);
        assert!(q_des[0] > 0.0);
        assert!((q_des[0] - 1.0).abs() > 0.5);
    }

    #[test]
    fn ramps_dq_to_vlim() {
        let mut q_des = vec![0.0];
        let q_goal = vec![100.0];
        let mut last_dq = vec![0.0];
        let vlim = 0.6;
        let dt = 0.002;
        let amax = stream_amax_from_vlim(vlim);
        let ticks = ((vlim / (amax * dt)).ceil() as usize) + 2;
        let mut dq = vec![0.0];
        for _ in 0..ticks {
            dq = live(&mut q_des, &q_goal, &mut last_dq, &[0.0], vlim, dt);
        }
        assert!((dq[0] - vlim).abs() < 1e-9);
    }

    #[test]
    fn stale_does_not_overshoot_goal() {
        let mut q_des = vec![0.0];
        let q_goal = vec![0.003];
        let mut last_dq = vec![0.0];
        let dt = 0.002;
        let vlim = 0.6;
        for _ in 0..200 {
            stale(&mut q_des, &q_goal, &mut last_dq, vlim, dt);
        }
        assert!((q_des[0] - 0.003).abs() < 1e-12);
        assert!(q_des[0] <= 0.003 + 1e-12);
    }

    #[test]
    fn stale_settled_publishes_zero_dq() {
        let mut q_des = vec![0.1];
        let q_goal = vec![0.1 + super::SETTLED_EPS_RAD * 0.5];
        let mut last_dq = vec![0.3];
        let dq = stale(&mut q_des, &q_goal, &mut last_dq, 0.6, 0.002);
        assert_eq!(dq, vec![0.0]);
        assert!(last_dq[0] < 0.3);
    }

    #[test]
    fn first_tick_from_seed_is_one_step_not_the_goal() {
        let mut q_des = vec![0.2, -0.1];
        let q_goal = vec![1.2, 0.9];
        let mut last_dq = vec![0.0, 0.0];
        let dt = 0.002;
        let vlim = 0.6;
        live(&mut q_des, &q_goal, &mut last_dq, &[0.0, 0.0], vlim, dt);
        assert!((q_des[0] - 1.2).abs() > 0.5);
        assert!((q_des[1] - 0.9).abs() > 0.5);
        assert!(q_des[0] > 0.2);
        assert!(q_des[1] > -0.1);
        assert!((q_des[0] - 0.2).abs() <= vlim * dt + 1e-12);
    }

    #[test]
    fn live_coasts_through_latch() {
        let mut q_des = vec![0.0];
        let mut last_dq = vec![0.6];
        let dt = 0.002;
        let vlim = 0.6;
        live(&mut q_des, &[0.001], &mut last_dq, &[0.6], vlim, dt);
        assert!(q_des[0] > 0.001);
        assert!(last_dq[0] > 0.5);
    }

    #[test]
    fn serial_rtt_gap_does_not_brake() {
        // A 40 ms gap used to exceed the 25 ms stale window and brake mid-motion.
        let mut q_des = vec![0.0];
        let mut last_dq = vec![0.5];
        let dq = step_joint_des(&mut q_des, &[0.02], &mut last_dq, &[0.5], 1.5, 0.002, 0.04);
        assert!(
            dq[0] > 0.2,
            "leader reply gap must keep coasting, got dq={dq:?}"
        );
    }

    #[test]
    fn live_holds_stream_velocity() {
        let mut q_des = vec![0.0];
        let mut last_dq = vec![0.6];
        let dt = 0.002;
        let vlim = 0.6;
        live(&mut q_des, &[0.001], &mut last_dq, &[0.6], vlim, dt);
        let q0 = q_des[0];
        live(&mut q_des, &[0.001], &mut last_dq, &[0.6], vlim, dt);
        assert!(q_des[0] > q0);
        assert!((last_dq[0] - 0.6).abs() < 0.05);
    }
}
