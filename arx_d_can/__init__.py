"""ARX-D-CAN Python SDK (Damiao motors over USB2CAN)."""

from __future__ import annotations

import importlib
from typing import Any

__version__ = "0.1.0"

_LAZY_EXPORTS = {
    "ArxDCan": ".hardware",
    "ArxDCanArm": ".hardware",
    "ArxDCanConfig": ".hardware",
    "ArxDCanState": ".hardware",
    "JointCfg": ".hardware",
    "JointGroup": ".hardware",
    "JointMotorConfig": ".hardware",
    "JointState": ".hardware",
    "MotorState": ".hardware",
    "default_config": ".hardware",
    "load_cfg": ".hardware",
    "ArxDCanRobot": ".api",
    "resolve_slave_motion": ".api",
    "resolve_slave_port": ".api",
}


def __getattr__(name: str) -> Any:
    if name in _LAZY_EXPORTS:
        module = importlib.import_module(_LAZY_EXPORTS[name], __name__)
        return getattr(module, name)
    if name in {"api", "execution", "hardware", "kinematics", "utils"}:
        return importlib.import_module(f"{__name__}.{name}")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "ArxDCan",
    "ArxDCanArm",
    "ArxDCanConfig",
    "ArxDCanRobot",
    "ArxDCanState",
    "JointCfg",
    "JointGroup",
    "JointMotorConfig",
    "JointState",
    "MotorState",
    "api",
    "default_config",
    "execution",
    "hardware",
    "kinematics",
    "load_cfg",
    "resolve_slave_motion",
    "resolve_slave_port",
    "utils",
    "__version__",
]
