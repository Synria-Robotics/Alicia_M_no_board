"""User-facing robot API."""

from __future__ import annotations

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


def create_robot(*args, **kwargs):
    """Create a robot after the native extension is actually needed."""
    from .robot import create_robot as factory

    return factory(*args, **kwargs)

__all__ = [
    "create_robot",
    "DEFAULT_BASE_LINK",
    "DEFAULT_PROFILE",
    "DEFAULT_URDF",
    "PROFILE_NAMES",
    "RobotProfile",
    "get_profile",
    "get_robot",
    "kinematics",
    "load_gravity_scales",
]
