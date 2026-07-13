"""Hardware layer — bus, groups, and high-level arm SDK."""

from __future__ import annotations

import importlib
from typing import Any

from .motor_config import JointCfg, load_cfg
from .serial_channel import is_dm_serial_channel

_LAZY = {
    "ArxDCan": ".bus",
    "ArxDCanArm": ".arm",
    "ArxDCanConfig": ".arm",
    "ArxDCanState": ".arm",
    "JointGroup": ".damiao_group",
    "JointMotorConfig": ".arm",
    "JointState": ".arm",
    "MotorState": ".arm",
    "NoOpGroup": ".damiao_group",
    "default_config": ".arm",
}


def __getattr__(name: str) -> Any:
    if name not in _LAZY:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = importlib.import_module(_LAZY[name], __name__)
    return getattr(module, name)


__all__ = [
    "ArxDCan",
    "ArxDCanArm",
    "ArxDCanConfig",
    "ArxDCanState",
    "JointCfg",
    "JointGroup",
    "JointMotorConfig",
    "JointState",
    "MotorState",
    "NoOpGroup",
    "default_config",
    "is_dm_serial_channel",
    "load_cfg",
]
