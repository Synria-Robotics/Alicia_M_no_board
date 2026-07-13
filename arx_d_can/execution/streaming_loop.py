"""Background joint streaming servo for teleop."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Sequence

import numpy as np

from ..utils.motion_profiles import step_joints_toward


class JointStreamServo:
    """Rate-limited background thread that slews commanded joints toward a target."""

    def __init__(
        self,
        *,
        hz: float,
        max_joint_vel_rad_s: float,
        send_fn: Callable[[Sequence[float]], None],
        initial_cmd: Sequence[float],
    ) -> None:
        self._hz = float(hz)
        self._max_joint_vel_rad_s = float(max_joint_vel_rad_s)
        self._send_fn = send_fn
        self._lock = threading.Lock()
        self._q_cmd = np.asarray(initial_cmd, dtype=np.float64).copy()
        self._q_target = self._q_cmd.copy()
        self._paused = False
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._loop,
            name="arx-d-can-stream",
            daemon=True,
        )
        self._thread.start()

    @property
    def commanded_joints(self) -> np.ndarray:
        with self._lock:
            return self._q_cmd.copy()

    def set_target(self, target: Sequence[float]) -> None:
        with self._lock:
            self._q_target = np.asarray(target, dtype=np.float64).copy()

    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        self._paused = False

    def sync_command(self, joints: Sequence[float]) -> None:
        with self._lock:
            arr = np.asarray(joints, dtype=np.float64).copy()
            self._q_cmd = arr
            self._q_target = arr.copy()

    def blocking_move(
        self,
        target: Sequence[float],
        *,
        move_fn: Callable[[Sequence[float], Sequence[float], float], None],
        move_seconds: float,
    ) -> None:
        """Pause streaming, run a blocking move, then resume with synced state."""
        self._paused = True
        try:
            with self._lock:
                start = self._q_cmd.copy()
            move_fn(start, target, move_seconds)
        finally:
            with self._lock:
                arr = np.asarray(target, dtype=np.float64).copy()
                self._q_cmd = arr
                self._q_target = arr.copy()
            self._paused = False

    def stop(self, *, join_timeout: float = 1.0) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=join_timeout)

    def _loop(self) -> None:
        period = 1.0 / max(1.0, self._hz)
        max_step = self._max_joint_vel_rad_s * period
        while not self._stop.is_set():
            t0 = time.monotonic()
            if not self._paused:
                with self._lock:
                    stepped = step_joints_toward(
                        self._q_cmd,
                        self._q_target,
                        max_step=max_step,
                    )
                    self._q_cmd = np.asarray(stepped, dtype=np.float64)
                    q_send = self._q_cmd.copy()
                self._send_fn(q_send.tolist())
            elapsed = time.monotonic() - t0
            sleep_s = period - elapsed
            if sleep_s > 0.0:
                time.sleep(sleep_s)
