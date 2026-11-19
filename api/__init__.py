"""User-facing robot API."""

from __future__ import annotations

from . import kinematics
from .robot import (
    Robot, RawArm, create_robot, DEFAULT_BAUD, DEFAULT_GRIPPER_SPEED,
    DEFAULT_JOINT_SPEED, DEFAULT_PORT, HOME_JOINTS, NUM_ARM_JOINTS,
    SparkVisBridge, Teleoperation, scan_ids,
)
from .profiles import (
    DEFAULT_PROFILE,
    PROFILE_NAMES,
    RobotProfile,
    get_profile,
    get_robot,
    load_gravity_scales,
)

DEFAULT_URDF = kinematics.DEFAULT_URDF
DEFAULT_BASE_LINK = kinematics.DEFAULT_BASE_LINK
DEFAULT_END_LINK = kinematics.DEFAULT_END_LINK

__all__ = [
    "Robot",
    "Teleoperation",
    "DEFAULT_BASE_LINK",
    "DEFAULT_BAUD",
    "DEFAULT_END_LINK",
    "DEFAULT_GRIPPER_SPEED",
    "DEFAULT_JOINT_SPEED",
    "DEFAULT_PORT",
    "DEFAULT_PROFILE",
    "DEFAULT_URDF",
    "HOME_JOINTS",
    "NUM_ARM_JOINTS",
    "PROFILE_NAMES",
    "RobotProfile",
    "SparkVisBridge",
    "Gripper",
    "Diagnostics",
    "get_profile",
    "get_robot",
    "kinematics",
    "load_gravity_scales",
    "scan_ids",
]

from .diagnostics import Diagnostics
from .gripper import Gripper
