"""ArxDCan — grouped Damiao motor bus controller."""

from __future__ import annotations

import threading
import time
from typing import Callable, Dict, List, Optional

import numpy as np
from motorbridge import Controller, CallError

from ..config.defaults import resolve_hw_cfg_path
from .damiao_group import JointGroup, NoOpGroup
from .motor_config import JointCfg, load_cfg
from .serial_channel import is_dm_serial_channel


class ArxDCan:
    """ARX-D-CAN grouped control system."""

    def __init__(
        self,
        hw_yaml: str | None = None,
        channel: str | None = None,
        baud: int | None = None,
    ) -> None:
        self._hw_yaml = resolve_hw_cfg_path(hw_yaml).name
        cfg = load_cfg(hw_yaml)
        if channel:
            cfg["channel"] = channel

        self._name: str = cfg["name"]
        self._channel: str = cfg["channel"]
        self._baud: int = int(baud or cfg.get("baud", 1_000_000))
        self._rate: float = cfg["rate"]
        self._all_joints: List[JointCfg] = cfg["joints"]
        self._groups_def: dict = cfg["groups"]

        self._ctrl_map: Dict[str, Controller] = {}
        self._motor_map: Dict[str, any] = {}
        self._groups: Dict[str, JointGroup] = {}

        self._running = False
        self._ctrl_thread: Optional[threading.Thread] = None
        self._ctrl_fn: Optional[Callable] = None
        self._ctrl_rate: float = self._rate
        self._connected: bool = False

        self._build_groups()

    def connect(self) -> None:
        if self._connected:
            return
        self._setup_motors()
        self._connected = True

    def _make_controller(self) -> Controller:
        if is_dm_serial_channel(self._channel):
            return Controller.from_dm_serial(self._channel, self._baud)
        return Controller(self._channel)

    def _setup_motors(self) -> None:
        if "main" not in self._ctrl_map:
            self._ctrl_map["main"] = self._make_controller()
        ctrl = self._ctrl_map["main"]
        for jc in self._all_joints:
            mot = ctrl.add_damiao_motor(jc.motor_id, jc.feedback_id, jc.model)
            self._motor_map[jc.name] = mot

    def _build_groups(self) -> None:
        for gname, gdef in self._groups_def.items():
            joints_def = gdef.get("joints", [])
            group = JointGroup(
                name=gname,
                joint_names=joints_def,
                all_joints=self._all_joints,
                motor_map=self._motor_map,
                ctrl_map=self._ctrl_map,
            )
            self._groups[gname] = group
        if "gripper" not in self._groups:
            self._groups["gripper"] = NoOpGroup()

    @property
    def num_joints(self) -> int:
        return len(self._all_joints)

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self._all_joints]

    @property
    def groups(self) -> Dict[str, JointGroup]:
        return self._groups

    @property
    def control_loop_active(self) -> bool:
        thread = getattr(self, "_ctrl_thread", None)
        return thread is not None and thread.is_alive()

    @property
    def rate(self) -> float:
        return self._ctrl_rate

    @property
    def has_gripper(self) -> bool:
        return not isinstance(self._groups.get("gripper", None), NoOpGroup)

    @property
    def hardware_yaml(self) -> str:
        return self._hw_yaml

    def __getattr__(self, name: str) -> any:
        if name.startswith("_"):
            raise AttributeError(name)
        if name in self._groups:
            return self._groups[name]
        raise AttributeError(name)

    def add_group(self, name: str, joint_names: List[str]) -> JointGroup:
        if name in self._groups:
            raise ValueError(f"组 {name!r} 已存在")
        group = JointGroup(
            name=name,
            joint_names=joint_names,
            all_joints=self._all_joints,
            motor_map=self._motor_map,
            ctrl_map=self._ctrl_map,
        )
        self._groups[name] = group
        return group

    def enable_all(self) -> None:
        for group in self._groups.values():
            group.enable()

    def disable_all(self) -> None:
        for group in self._groups.values():
            group.disable()

    def set_zero(self, poll_max: int = 200, poll_interval: float = 0.05) -> None:
        self.disable_all()
        time.sleep(0.3)
        for jc in self._all_joints:
            for _ in range(poll_max):
                for motor in self._motor_map.values():
                    try:
                        motor.request_feedback()
                    except Exception:
                        pass
                for ctrl in self._ctrl_map.values():
                    try:
                        ctrl.poll_feedback_once()
                    except Exception:
                        pass
                state = self._motor_map[jc.name].get_state()
                if state is not None and state.status_code == 0:
                    break
                time.sleep(poll_interval)
            try:
                self._motor_map[jc.name].set_zero_position()
            except CallError as e:
                print(f"[set_zero] {jc.name}: {e}")
            time.sleep(0.1)

    def get_state(
        self,
        request_feedback: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if request_feedback:
            for motor in self._motor_map.values():
                try:
                    motor.request_feedback()
                except Exception:
                    pass
        for ctrl in self._ctrl_map.values():
            try:
                ctrl.poll_feedback_once()
            except Exception:
                pass
        pos, vel, torq = [], [], []
        for jc in self._all_joints:
            state = self._motor_map[jc.name].get_state()
            if state is not None:
                pos.append(state.pos)
                vel.append(state.vel)
                torq.append(state.torq)
            else:
                pos.append(0.0)
                vel.append(0.0)
                torq.append(0.0)
        return (
            np.array(pos, dtype=np.float64),
            np.array(vel, dtype=np.float64),
            np.array(torq, dtype=np.float64),
        )

    def get_positions(self) -> np.ndarray:
        return self.get_state()[0]

    def get_velocities(self) -> np.ndarray:
        return self.get_state()[1]

    def get_torques(self) -> np.ndarray:
        return self.get_state()[2]

    def disconnect(self) -> None:
        if not self._connected:
            return
        self.stop_control_loop()
        for ctrl in self._ctrl_map.values():
            try:
                ctrl.close_bus()
            except Exception:
                pass
            ctrl.close()
        self._ctrl_map.clear()
        self._motor_map.clear()
        self._connected = False

    def estop(self) -> None:
        self.disable_all()

    def reconnect(
        self,
        init_delay: float = 1.0,
        post_setup_delay: float = 0.5,
    ) -> None:
        self.disconnect()
        time.sleep(init_delay)
        self._ctrl_map["main"] = self._make_controller()
        self._motor_map.clear()
        ctrl = self._ctrl_map["main"]
        for jc in self._all_joints:
            mot = ctrl.add_damiao_motor(jc.motor_id, jc.feedback_id, jc.model)
            self._motor_map[jc.name] = mot
            time.sleep(0.05)
        self._build_groups()
        time.sleep(post_setup_delay)
        print("[reconnect] 控制器和电机已重新初始化")

    def start_control_loop(
        self,
        control_fn: Callable[["ArxDCan", float], None],
        rate: Optional[float] = None,
    ) -> None:
        if self.control_loop_active:
            raise RuntimeError("控制循环已在运行，请先调用 stop_control_loop()")
        self._running = True
        self._ctrl_rate = rate if rate is not None else self._rate
        self._ctrl_fn = control_fn
        self._ctrl_thread = threading.Thread(
            target=self._control_loop_impl,
            name="arx_d_can-control-loop",
            daemon=True,
        )
        self._ctrl_thread.start()

    def _control_loop_impl(self) -> None:
        dt = 1.0 / self._ctrl_rate
        while self._running:
            t0 = time.perf_counter()
            try:
                self._ctrl_fn(self, dt)
            except Exception:
                if self._running:
                    raise
            elapsed = time.perf_counter() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop_control_loop(self) -> None:
        self._running = False
        thread = getattr(self, "_ctrl_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)

    def __enter__(self) -> "ArxDCan":
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        groups = ", ".join(f"{k}({g.num_joints}j)" for k, g in self._groups.items())
        return f"ArxDCan({self._name!r}, [{groups}], rate={self._ctrl_rate}Hz)"
