"""Read-only diagnostics and thermal policy observations."""

from __future__ import annotations

import math
from typing import Any


class Diagnostics:
    def __init__(self, native: Any) -> None:
        self._native = native

    def read(self) -> list[dict[str, Any]]:
        return [dict(item) for item in self._native.get_motor_diagnostics(True)]

    def thermal(self, *, max_temperature_c: float = 70.0) -> dict[str, Any]:
        motors = self.read()
        hottest = max(
            motors,
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
