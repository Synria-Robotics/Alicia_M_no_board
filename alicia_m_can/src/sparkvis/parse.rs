use std::collections::HashMap;

use crate::utils::gripper::validate_gripper_value;

pub const NUM_ARM_JOINTS: usize = 6;
const FINGER_TRAVEL_M: f64 = 0.025;

pub fn parse_joint_values(
    joint_values: &HashMap<String, f64>,
) -> (HashMap<usize, f64>, Option<f64>) {
    let mut joints = HashMap::new();
    let mut gripper = None;
    let mut left_finger = None;
    let mut right_finger = None;

    for (raw_key, val) in joint_values {
        if !val.is_finite() {
            continue;
        }
        let key = raw_key.trim();
        if let Some(idx) = parse_joint_index(key) {
            if idx < NUM_ARM_JOINTS {
                joints.insert(idx, *val);
            }
            continue;
        }
        match key.to_ascii_lowercase().as_str() {
            "gripper" => {
                gripper = Some(if *val <= 1.0 + 1e-6 {
                    *val * 1000.0
                } else {
                    *val
                });
            }
            "left_finger" => left_finger = Some(*val),
            "right_finger" => right_finger = Some(*val),
            _ => {}
        }
    }

    if gripper.is_none() {
        if let Some(v) = left_finger {
            let ratio = (-v / FINGER_TRAVEL_M).clamp(0.0, 1.0);
            gripper = Some((1.0 - ratio) * 1000.0);
        } else if let Some(v) = right_finger {
            let ratio = (v / FINGER_TRAVEL_M).clamp(0.0, 1.0);
            gripper = Some((1.0 - ratio) * 1000.0);
        }
    }
    if let Some(g) = gripper {
        gripper = Some(validate_gripper_value(g));
    }
    (joints, gripper)
}

fn parse_joint_index(key: &str) -> Option<usize> {
    let lower = key.to_ascii_lowercase();
    if lower.starts_with("joint") {
        let digits = lower.trim_start_matches("joint").trim_start_matches('_');
        if let Ok(n) = digits.parse::<usize>() {
            return Some(n.saturating_sub(1));
        }
    }
    None
}

pub fn merge_joint_target(current: &mut [f64; NUM_ARM_JOINTS], partial: &HashMap<usize, f64>) {
    for (idx, val) in partial {
        if *idx < NUM_ARM_JOINTS {
            current[*idx] = *val;
        }
    }
}
