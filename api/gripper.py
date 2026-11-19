"""Business-level gripper API. All closing uses guarded continuous grasping."""

from __future__ import annotations

from typing import Any


class Gripper:
    def __init__(self, native: Any) -> None:
        self._native = native

    def close(self) -> dict[str, Any]:
        return dict(self._native.gripper_close())

    def open(self, target: float = 1000.0) -> None:
        self._native.gripper_open(float(target))

    def hold(self) -> None:
        self._native.gripper_hold()

    def release(self) -> None:
        self._native.gripper_release()

    @property
    def value(self) -> float | None:
        return self._native.get_gripper_value(True)
