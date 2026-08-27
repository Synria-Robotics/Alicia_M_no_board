"""User-facing robot API."""

from __future__ import annotations

import importlib
from typing import Any

from . import kinematics
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
    "Arm",
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
    "SynriaRobotAPI",
    "Teleoperation",
    "create_robot",
    "get_profile",
    "get_robot",
    "kinematics",
    "load_gravity_scales",
    "scan_ids",
]


def __getattr__(name: str) -> Any:
    module = importlib.import_module(".synria_robot_api", __name__)
    try:
        value = getattr(module, name)
    except AttributeError:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from None
    globals()[name] = value
    return value
