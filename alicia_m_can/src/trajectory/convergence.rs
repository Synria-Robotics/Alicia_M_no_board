pub fn needs_trajectory(current: &[f64], target: &[f64], tolerance: f64) -> bool {
    let tol = tolerance.max(0.0);
    !current
        .iter()
        .zip(target.iter())
        .all(|(c, t)| (t - c).abs() <= tol)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn at_target_within_tolerance() {
        assert!(!needs_trajectory(&[1.0, 2.0], &[1.005, 1.995], 0.01));
        assert!(needs_trajectory(&[1.0, 2.0], &[1.02, 1.995], 0.01));
    }
}
