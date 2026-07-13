"""Gripper user scale — Alicia-M compatible (0 = closed, 1000 = open)."""

from __future__ import annotations

GRIPPER_MIN = 0.0
GRIPPER_MAX = 1000.0
GRIPPER_CLOSED = GRIPPER_MIN
GRIPPER_OPEN = GRIPPER_MAX


def validate_gripper_value(value: float) -> float:
    """Clamp gripper command to [0, 1000]."""
    return max(GRIPPER_MIN, min(GRIPPER_MAX, float(value)))


def user_to_motor(
    value: float,
    *,
    closed_motor: float,
    open_motor: float,
) -> float:
    """Map user gripper value [0, 1000] to motor MIT position (rad)."""
    user = validate_gripper_value(value)
    ratio = user / GRIPPER_MAX
    return float(closed_motor + (open_motor - closed_motor) * ratio)


def motor_to_user(
    motor: float,
    *,
    closed_motor: float,
    open_motor: float,
) -> float:
    """Map motor feedback position (rad) to user gripper value [0, 1000]."""
    if open_motor == closed_motor:
        return GRIPPER_MIN
    ratio = (float(motor) - closed_motor) / (open_motor - closed_motor)
    return validate_gripper_value(ratio * GRIPPER_MAX)
