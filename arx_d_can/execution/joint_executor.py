"""Blocking joint-space interpolation for POS_VEL streaming."""

from __future__ import annotations

import time
from collections.abc import Sequence
from typing import TYPE_CHECKING

from ..utils.motion_profiles import joint_move_duration, linear_interpolate

if TYPE_CHECKING:
    from ..hardware.arm import ArxDCanArm


def interpolate_joint_positions(
    arm: ArxDCanArm,
    start: Sequence[float],
    end: Sequence[float],
    *,
    seconds: float | None = None,
    hz: float | None = None,
    max_joint_vel_rad_s: float | None = None,
) -> None:
    """Move from *start* to *end* with fixed duration or joint-space constant speed."""
    control_hz = float(hz if hz is not None else arm.config.control_hz)
    stream_tick = 1.0 / max(1.0, control_hz)

    if seconds is not None:
        if seconds <= 0.0:
            arm.send_joint_positions(end)
            return
        duration = float(seconds)
    elif max_joint_vel_rad_s is not None and max_joint_vel_rad_s > 0.0:
        duration = joint_move_duration(
            start,
            end,
            max_joint_vel_rad_s=max_joint_vel_rad_s,
            min_duration=stream_tick,
        )
    else:
        duration = stream_tick

    if duration <= 0.0:
        arm.send_joint_positions(end)
        return

    period = stream_tick
    steps = max(1, int(duration * control_hz))
    start_values = tuple(float(value) for value in start)
    end_values = tuple(float(value) for value in end)
    if len(start_values) != len(end_values):
        raise ValueError("start and end must have the same length")

    for step in range(1, steps + 1):
        ratio = step / steps
        target = linear_interpolate(start_values, end_values, ratio)
        arm.send_joint_positions(target)
        time.sleep(period)
    arm.send_joint_positions(end_values)
