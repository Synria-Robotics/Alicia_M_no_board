use std::time::Instant;

pub fn needs_trajectory(current: &[f64], target: &[f64], tolerance: f64) -> bool {
    let tol = tolerance.max(0.0);
    !current
        .iter()
        .zip(target.iter())
        .all(|(c, t)| (t - c).abs() <= tol)
}

pub fn max_abs_joint_delta(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(left, right)| (left - right).abs())
        .fold(0.0_f64, f64::max)
}

/// Abort a joint wait when the arm is stuck short of the target.
///
/// Matches `move_until_contact_or_target`: error at least 0.05 rad and
/// speed at most 0.08 rad/s, held for 0.3 s. A move that is still traveling
/// keeps the full converge timeout.
#[derive(Clone, Debug)]
pub struct StallWatch {
    err_min_rad: f64,
    vel_max_rad_s: f64,
    hold_sec: f64,
    started: Option<Instant>,
}

impl StallWatch {
    pub fn new(err_min_rad: f64, vel_max_rad_s: f64, hold_sec: f64) -> Self {
        Self {
            err_min_rad,
            vel_max_rad_s,
            hold_sec,
            started: None,
        }
    }

    pub fn arm_blocked() -> Self {
        Self::new(0.05, 0.08, 0.3)
    }

    /// `true` once the stuck condition has lasted `hold_sec`.
    pub fn observe(&mut self, err_rad: f64, vel_rad_s: f64, now: Instant) -> bool {
        let stuck = err_rad >= self.err_min_rad && vel_rad_s <= self.vel_max_rad_s;
        if !stuck {
            self.started = None;
            return false;
        }
        match self.started {
            None => {
                self.started = Some(now);
                self.hold_sec <= 0.0
            }
            Some(t0) => now.saturating_duration_since(t0).as_secs_f64() >= self.hold_sec,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    #[test]
    fn at_target_within_tolerance() {
        assert!(!needs_trajectory(&[1.0, 2.0], &[1.005, 1.995], 0.01));
        assert!(needs_trajectory(&[1.0, 2.0], &[1.02, 1.995], 0.01));
    }

    #[test]
    fn stall_aborts_only_after_hold() {
        let t0 = Instant::now();
        let mut watch = StallWatch::arm_blocked();
        assert!(!watch.observe(0.2, 0.0, t0));
        assert!(!watch.observe(0.2, 0.0, t0 + Duration::from_millis(200)));
        assert!(watch.observe(0.2, 0.0, t0 + Duration::from_millis(300)));
    }

    #[test]
    fn motion_resets_stall() {
        let t0 = Instant::now();
        let mut watch = StallWatch::arm_blocked();
        assert!(!watch.observe(0.2, 0.0, t0));
        assert!(!watch.observe(0.2, 0.2, t0 + Duration::from_millis(100)));
        assert!(!watch.observe(0.2, 0.0, t0 + Duration::from_millis(400)));
        assert!(watch.observe(0.2, 0.0, t0 + Duration::from_millis(700)));
    }

    #[test]
    fn near_target_is_not_a_stall() {
        let t0 = Instant::now();
        let mut watch = StallWatch::arm_blocked();
        assert!(!watch.observe(0.04, 0.0, t0));
        assert!(!watch.observe(0.04, 0.0, t0 + Duration::from_secs(1)));
    }
}
