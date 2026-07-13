"""User-facing Synria ARX API."""

from .synria_arx_api import (
    ArxDCanRobot,
    DEFAULT_ROBOT_VARIANT,
    HOME_JOINTS,
    NUM_ARM_JOINTS,
    resolve_slave_motion,
    resolve_slave_port,
)

__all__ = [
    "ArxDCanRobot",
    "DEFAULT_ROBOT_VARIANT",
    "HOME_JOINTS",
    "NUM_ARM_JOINTS",
    "resolve_slave_motion",
    "resolve_slave_port",
]
