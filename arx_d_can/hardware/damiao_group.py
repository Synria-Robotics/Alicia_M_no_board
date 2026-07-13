"""JointGroup and NoOpGroup — per-group Damiao motor control."""

from __future__ import annotations

import time
from typing import Dict, List, Optional

import numpy as np
from motorbridge import Controller, Mode, CallError

from .motor_config import JointCfg


class NoOpGroup:
    """Stub gripper group when no gripper is configured."""

    name: str = "gripper"
    _mode: str = "mit"

    @property
    def num_joints(self) -> int:
        return 0

    @property
    def joint_names(self) -> List[str]:
        return []

    @property
    def mode(self) -> str:
        return "mit"

    def enable(self) -> None:
        pass

    def disable(self) -> None:
        pass

    def mode_mit(self, kp=None, kd=None) -> bool:
        self._mode = "mit"
        return True

    def mode_pos_vel(self, vlim=None) -> bool:
        self._mode = "pos_vel"
        return True

    def mode_vel(self) -> bool:
        self._mode = "vel"
        return True

    def send_mit(self, pos, vel=None, kp=None, kd=None, tau=None) -> None:
        pass

    def send_pos_vel(self, pos, vlim=None) -> None:
        pass

    def send_vel(self, vel) -> None:
        pass

    def get_positions(self) -> np.ndarray:
        return np.array([], dtype=np.float64)

    def get_velocities(self) -> np.ndarray:
        return np.array([], dtype=np.float64)

    def __repr__(self) -> str:
        return "NoOpGroup(gripper, no actuator)"


class JointGroup:
    """One named group of joints with independent control mode."""

    def __init__(
        self,
        name: str,
        joint_names: List[str],
        all_joints: List[JointCfg],
        motor_map: Dict[str, any],
        ctrl_map: Dict[str, Controller],
    ) -> None:
        self.name = name
        self._jn: List[str] = joint_names
        self._jcfgs: List[JointCfg] = [
            next(j for j in all_joints if j.name == n) for n in joint_names
        ]
        self._mm: Dict[str, any] = motor_map
        self._cm: Dict[str, Controller] = ctrl_map
        self._mode: str = "mit"
        self._mit_kp: np.ndarray = np.array([j.kp for j in self._jcfgs], dtype=np.float64)
        self._mit_kd: np.ndarray = np.array([j.kd for j in self._jcfgs], dtype=np.float64)
        self._pv_vlim: np.ndarray = np.array([j.vlim for j in self._jcfgs], dtype=np.float64)

    @property
    def num_joints(self) -> int:
        return len(self._jn)

    @property
    def joint_names(self) -> List[str]:
        return list(self._jn)

    @property
    def mode(self) -> str:
        return self._mode

    def enable(self) -> None:
        try:
            self._cm["main"].enable_all()
        except CallError as e:
            print(f"[{self.name}/enable] {e}")
        time.sleep(0.05)

    def disable(self) -> None:
        try:
            self._cm["main"].disable_all()
        except CallError as e:
            print(f"[{self.name}/disable] {e}")
        time.sleep(0.05)

    def _write_pv_params(self, jc: JointCfg) -> None:
        motor = self._mm[jc.name]
        try:
            motor.write_register_f32(25, jc.vel_kp)
            motor.write_register_f32(26, jc.vel_ki)
            motor.write_register_f32(27, jc.pos_kp)
            motor.write_register_f32(28, jc.pos_ki)
            time.sleep(0.02)
        except Exception as e:
            print(f"[{self.name}/pv_params/{jc.name}] {e}")

    def mode_mit(
        self,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
    ) -> bool:
        self._mode = "mit"
        if kp is not None:
            self._mit_kp = np.asarray(kp, dtype=np.float64).reshape(-1)
        if kd is not None:
            self._mit_kd = np.asarray(kd, dtype=np.float64).reshape(-1)
        ok = True
        for jc in self._jcfgs:
            try:
                self._mm[jc.name].ensure_mode(Mode.MIT, 1000)
            except CallError as e:
                print(f"[{self.name}/mode_mit/{jc.name}] {e}")
                ok = False
            time.sleep(0.05)
        time.sleep(0.2)
        return ok

    def mode_pos_vel(
        self,
        vlim: Optional[np.ndarray] = None,
    ) -> bool:
        self._mode = "pos_vel"
        if vlim is not None:
            self._pv_vlim = np.asarray(vlim, dtype=np.float64).reshape(-1)
        ok = True
        for jc in self._jcfgs:
            self._write_pv_params(jc)
            try:
                self._mm[jc.name].ensure_mode(Mode.POS_VEL, 1000)
            except CallError as e:
                print(f"[{self.name}/mode_pos_vel/{jc.name}] {e}")
                ok = False
            time.sleep(0.05)
        time.sleep(0.2)
        return ok

    def mode_vel(self) -> bool:
        self._mode = "vel"
        ok = True
        for jc in self._jcfgs:
            try:
                self._mm[jc.name].ensure_mode(Mode.VEL, 1000)
            except CallError as e:
                print(f"[{self.name}/mode_vel/{jc.name}] {e}")
                ok = False
            time.sleep(0.05)
        time.sleep(0.2)
        return ok

    def send_mit(
        self,
        pos: np.ndarray,
        vel: Optional[np.ndarray] = None,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
        tau: Optional[np.ndarray] = None,
    ) -> None:
        n = self.num_joints
        pos = np.asarray(pos, dtype=np.float64).reshape(-1)
        if vel is None:
            vel = np.zeros(n)
        if tau is None:
            tau = np.zeros(n)
        if kp is None:
            kp = self._mit_kp
        if kd is None:
            kd = self._mit_kd

        for i, jc in enumerate(self._jcfgs):
            try:
                self._mm[jc.name].send_mit(
                    float(pos[i]),
                    float(vel[i]),
                    float(kp[i]),
                    float(kd[i]),
                    float(tau[i]),
                )
            except CallError:
                pass

    def send_pos_vel(
        self,
        pos: np.ndarray,
        vlim: Optional[np.ndarray] = None,
    ) -> None:
        pos = np.asarray(pos, dtype=np.float64).reshape(-1)
        if vlim is None:
            vlim = self._pv_vlim
        vlim = np.asarray(vlim, dtype=np.float64).reshape(-1)
        for i in range(min(len(pos), len(vlim))):
            try:
                self._mm[self._jcfgs[i].name].send_pos_vel(
                    float(pos[i]),
                    float(vlim[i]),
                )
            except CallError:
                pass

    def send_vel(self, vel: np.ndarray) -> None:
        vel = np.asarray(vel, dtype=np.float64).reshape(-1)
        for i in range(min(len(vel), self.num_joints)):
            try:
                self._mm[self._jcfgs[i].name].send_vel(float(vel[i]))
            except CallError:
                pass

    def _request_feedback(self) -> None:
        for jc in self._jcfgs:
            try:
                self._mm[jc.name].request_feedback()
            except Exception:
                pass
        try:
            self._cm["main"].poll_feedback_once()
        except Exception:
            pass

    def get_positions(self, request_feedback: bool = True) -> np.ndarray:
        if request_feedback:
            self._request_feedback()
        return np.array(
            [
                self._mm[jc.name].get_state().pos
                if self._mm[jc.name].get_state() is not None
                else 0.0
                for jc in self._jcfgs
            ],
            dtype=np.float64,
        )

    def get_velocities(self, request_feedback: bool = True) -> np.ndarray:
        if request_feedback:
            self._request_feedback()
        return np.array(
            [
                self._mm[jc.name].get_state().vel
                if self._mm[jc.name].get_state() is not None
                else 0.0
                for jc in self._jcfgs
            ],
            dtype=np.float64,
        )

    def __repr__(self) -> str:
        return f"JointGroup({self.name!r}, joints={self.num_joints}, mode={self._mode})"
