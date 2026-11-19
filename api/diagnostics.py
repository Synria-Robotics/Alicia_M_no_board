"""Read-only diagnostics and thermal policy observations."""

from __future__ import annotations

import math
from typing import Any


class Diagnostics:
    def __init__(self, native: Any) -> None:
        self._native = native

    def read(self) -> list[dict[str, Any]]:
        return [dict(item) for item in self._native.get_motor_diagnostics(True)]

    def thermal(
        self, *, max_temperature_c: float = 70.0, motor_id: int | None = None
    ) -> dict[str, Any]:
        motors = self.read()
        monitored = [m for m in motors if motor_id is None or m.get("motor_id") == motor_id]
        hottest = max(
            monitored,
            key=lambda m: max(float(m.get("mos_temperature_c", "nan")),
                              float(m.get("rotor_temperature_c", "nan"))),
            default=None,
        )
        if hottest is None:
            return {"ok": False, "reason": "no motor diagnostics", "motors": motors}
        temperature = max(float(hottest.get("mos_temperature_c", "nan")),
                          float(hottest.get("rotor_temperature_c", "nan")))
        return {
            "ok": math.isfinite(temperature) and temperature < max_temperature_c,
            "limit_c": max_temperature_c,
            "temperature_c": temperature,
            "hottest": hottest,
            "motors": motors,
        }
