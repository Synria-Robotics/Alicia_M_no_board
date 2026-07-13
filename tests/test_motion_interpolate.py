"""Tests for motion interpolation helpers."""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock, call

from arx_d_can.execution.joint_executor import (
    interpolate_joint_positions,
    joint_move_duration,
)
from arx_d_can.utils.motion_profiles import step_joints_toward


class InterpolateJointPositionsTests(unittest.TestCase):
    def test_step_joints_toward_clamps_per_joint(self) -> None:
        out = step_joints_toward([0.0, 0.0], [1.0, -0.2], max_step=0.1)
        self.assertEqual(out, (0.1, -0.1))

    def test_zero_seconds_sends_end_only(self) -> None:
        arm = MagicMock()
        arm.config.control_hz = 100.0
        interpolate_joint_positions(arm, [0.0] * 6, [1.0] * 6, seconds=0.0)
        arm.send_joint_positions.assert_called_once_with([1.0] * 6)

    def test_one_second_at_100hz_sends_101_positions(self) -> None:
        arm = MagicMock()
        arm.config.control_hz = 100.0
        start = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        end = (0.0, -0.4, 0.0, 0.0, 0.0, 0.0)
        interpolate_joint_positions(arm, start, end, seconds=1.0)
        self.assertEqual(arm.send_joint_positions.call_count, 101)
        first_target = arm.send_joint_positions.call_args_list[0].args[0]
        last_target = arm.send_joint_positions.call_args_list[-1].args[0]
        self.assertAlmostEqual(first_target[1], -0.4 / 100.0, places=6)
        self.assertAlmostEqual(last_target[1], -0.4, places=6)

    def test_constant_speed_duration_scales_with_delta(self) -> None:
        duration = joint_move_duration(
            [0.0] * 6,
            [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
            max_joint_vel_rad_s=0.5,
            min_duration=0.01,
        )
        self.assertAlmostEqual(duration, 1.0, places=6)

    def test_constant_speed_interpolate_uses_duration(self) -> None:
        arm = MagicMock()
        arm.config.control_hz = 100.0
        interpolate_joint_positions(
            arm,
            [0.0] * 6,
            [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
            max_joint_vel_rad_s=0.5,
        )
        self.assertEqual(arm.send_joint_positions.call_count, 101)


if __name__ == "__main__":
    unittest.main()
