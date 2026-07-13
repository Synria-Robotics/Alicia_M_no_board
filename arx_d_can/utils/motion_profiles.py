"""Motion profile helpers (linear slew, duration from speed)."""

from __future__ import annotations

from collections.abc import Sequence


def step_joints_toward(
    current: Sequence[float],
    target: Sequence[float],
    *,
    max_step: float,
) -> tuple[float, ...]:
    """Slew each joint toward *target* by at most *max_step* (rad)."""
    if max_step <= 0.0:
        raise ValueError("max_step must be > 0")
    out: list[float] = []
    for cur, tgt in zip(current, target):
        err = float(tgt) - float(cur)
        if abs(err) <= max_step:
            out.append(float(tgt))
        else:
            out.append(float(cur) + max_step * (1.0 if err > 0.0 else -1.0))
    return tuple(out)


def joint_move_duration(
    start: Sequence[float],
    end: Sequence[float],
    *,
    max_joint_vel_rad_s: float,
    min_duration: float = 0.0,
) -> float:
    """Duration for joint-space motion at constant max joint velocity (rad/s)."""
    if max_joint_vel_rad_s <= 0.0:
        raise ValueError("max_joint_vel_rad_s must be > 0")
    delta = max(abs(float(e) - float(s)) for s, e in zip(start, end))
    if delta <= 0.0:
        return max(0.0, min_duration)
    return max(min_duration, delta / max_joint_vel_rad_s)


def linear_interpolate(
    start: Sequence[float],
    end: Sequence[float],
    ratio: float,
) -> tuple[float, ...]:
    """Linear blend between joint vectors at *ratio* in [0, 1]."""
    t = max(0.0, min(1.0, float(ratio)))
    return tuple(
        float(s) + (float(e) - float(s)) * t
        for s, e in zip(start, end)
    )
