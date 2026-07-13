"""High-level ARX-D-CAN arm SDK (Damiao motors over USB2CAN)."""

from __future__ import annotations

import math
import re
import subprocess
import sys
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .bus import ArxDCan
from .motor_config import JointCfg, load_cfg
from ..utils.gripper import motor_to_user, user_to_motor, validate_gripper_value


@dataclass(slots=True, frozen=True)
class MotorState:
    name: str
    motor_id: int
    feedback_id: int
    position: float
    velocity: float
    torque: float = 0.0


@dataclass(slots=True, frozen=True)
class JointState:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    torques: tuple[float, ...]


@dataclass(slots=True, frozen=True)
class ArxDCanState:
    arm: JointState
    gripper: MotorState | None = None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.arm.names

    @property
    def positions(self) -> tuple[float, ...]:
        return self.arm.positions


@dataclass(slots=True, frozen=True)
class JointMotorConfig:
    name: str
    motor_id: int
    feedback_id: int
    model: str
    mit_kp: float
    mit_kd: float
    pv_vel_kp: float
    pv_vel_ki: float
    pv_pos_kp: float
    pv_pos_ki: float
    pv_vlim: float


@dataclass(slots=True, frozen=True)
class ArxDCanConfig:
    port: str = "/dev/ttyACM0"
    baud: int = 1_000_000
    control_hz: float = 100.0
    arm_control_mode: str = "posvel"
    arm_joints: tuple[JointMotorConfig, ...] = ()
    gripper: JointMotorConfig | None = None
    gripper_closed_value: float = 0.0   # motor pos (rad) at user gripper_value=0
    gripper_open_value: float = 5.0     # motor pos (rad) at user gripper_value=1000

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.arm_joints)


def _joint_from_yaml(joint: JointCfg) -> JointMotorConfig:
    return JointMotorConfig(
        name=joint.name,
        motor_id=joint.motor_id,
        feedback_id=joint.feedback_id,
        model=joint.model,
        mit_kp=joint.kp,
        mit_kd=joint.kd,
        pv_vel_kp=joint.vel_kp,
        pv_vel_ki=joint.vel_ki,
        pv_pos_kp=joint.pos_kp,
        pv_pos_ki=joint.pos_ki,
        pv_vlim=joint.vlim,
    )


def default_config(
    *,
    port: str | None = None,
    baud: int | None = None,
    control_hz: float = 100.0,
    arm_control_mode: str = "posvel",
) -> ArxDCanConfig:
    data = load_cfg()
    joints_by_name = {
        joint.name: _joint_from_yaml(joint)
        for joint in data.get("joints", [])
    }
    groups = data.get("groups", {})
    arm_names = list(groups.get("arm", {}).get("joints", []))
    gripper_names = list(groups.get("gripper", {}).get("joints", []))
    gripper_mapping = data.get("gripper_mapping", {}) or {}

    return ArxDCanConfig(
        port=str(port or data.get("channel", "/dev/ttyACM0")),
        baud=int(baud or data.get("baud", 1_000_000)),
        control_hz=control_hz,
        arm_control_mode=arm_control_mode,
        arm_joints=tuple(joints_by_name[name] for name in arm_names),
        gripper=joints_by_name.get(gripper_names[0]) if gripper_names else None,
        gripper_closed_value=float(gripper_mapping.get("closed_value", 0.0)),
        gripper_open_value=float(gripper_mapping.get("open_value", 5.0)),
    )


class ArxDCanArm:
    """High-level Python SDK for an ARX arm using Damiao motors over USB2CAN."""

    def __init__(
        self,
        *,
        port: str = "/dev/ttyACM0",
        baud: int = 1_000_000,
        config: ArxDCanConfig | None = None,
        control_mode: str = "posvel",
        enable_gripper: bool = False,
    ) -> None:
        self.config = config or default_config(
            port=port,
            baud=baud,
            arm_control_mode=control_mode,
        )
        self.robot = ArxDCan(channel=self.config.port, baud=self.config.baud)
        self.enable_gripper = enable_gripper
        self._connected = False
        self._enabled = False
        self._mode = self.config.arm_control_mode.strip().lower().replace("_", "")

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.config.joint_names

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def enabled(self) -> bool:
        return self._enabled

    def connect(self) -> None:
        self.robot.connect()
        self._connected = True

    def configure(self, mode: str | None = None) -> None:
        self.configure_mode(mode or self._mode)
        if self.enable_gripper and self.config.gripper is not None:
            if not self.robot.gripper.mode_mit():
                raise RuntimeError("ARX-D-CAN gripper did not enter MIT mode")

    def close(self, *, disable: bool = False) -> None:
        if disable and self._enabled:
            self.disable()
        elif self._connected:
            self.robot.stop_control_loop()
        if self._connected:
            self.robot.disconnect()
        self._connected = False
        self._enabled = False

    def enable(self) -> None:
        self._require_connected()
        self.robot.arm.enable()
        if self.enable_gripper and self.config.gripper is not None:
            self.robot.gripper.enable()
        self._enabled = True

    def disable(self) -> None:
        self._require_connected()
        self.robot.arm.disable()
        if self.enable_gripper and self.config.gripper is not None:
            self.robot.gripper.disable()
        self._enabled = False

    def configure_mode(self, mode: str = "posvel") -> None:
        self._require_connected()
        normalized = mode.strip().lower().replace("_", "")
        if normalized in ("posvel", "pv"):
            if not self.robot.arm.mode_pos_vel():
                raise RuntimeError("ARX-D-CAN arm did not enter POS_VEL mode")
            self._mode = "posvel"
            return
        if normalized == "mit":
            if not self.robot.arm.mode_mit():
                raise RuntimeError("ARX-D-CAN arm did not enter MIT mode")
            self._mode = "mit"
            return
        raise ValueError("mode must be 'posvel' or 'mit'")

    def read_state(self, *, request_feedback: bool = True) -> ArxDCanState:
        self._require_connected()
        pos, vel, tau = self.robot.get_state(request_feedback=request_feedback)
        arm_count = len(self.config.arm_joints)
        arm_pos = pos[:arm_count]
        arm_vel = vel[:arm_count]
        arm_tau = tau[:arm_count]
        gripper_state = None
        if self.config.gripper is not None and len(pos) > arm_count:
            gripper_state = MotorState(
                name=self.config.gripper.name,
                motor_id=self.config.gripper.motor_id,
                feedback_id=self.config.gripper.feedback_id,
                position=float(pos[arm_count]),
                velocity=float(vel[arm_count]),
                torque=float(tau[arm_count]),
            )
        return ArxDCanState(
            arm=JointState(
                names=self.config.joint_names,
                positions=tuple(float(value) for value in arm_pos),
                velocities=tuple(float(value) for value in arm_vel),
                torques=tuple(float(value) for value in arm_tau),
            ),
            gripper=gripper_state,
        )

    def scan_ids(
        self,
        *,
        start_id: int = 1,
        end_id: int = 16,
        model: str = "4340P",
        timeout_ms: int = 30,
        feedback_base: str = "0x10",
    ) -> list[int]:
        command = [
            sys.executable,
            "-m",
            "motorbridge.cli",
            "scan",
            "--vendor",
            "damiao",
            "--transport",
            "dm-serial",
            "--serial-port",
            self.config.port,
            "--serial-baud",
            str(self.config.baud),
            "--model",
            model,
            "--start-id",
            str(start_id),
            "--end-id",
            str(end_id),
            "--feedback-base",
            feedback_base,
            "--timeout-ms",
            str(timeout_ms),
        ]
        result = subprocess.run(command, text=True, capture_output=True, check=False)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip() or "motorbridge scan failed")
        hits: list[int] = []
        for match in re.finditer(r"\[hit\].*?probe=0x([0-9A-Fa-f]+)", result.stdout):
            hits.append(int(match.group(1), 16))
        return hits

    def send_joint_positions(
        self,
        positions: Sequence[float],
        *,
        mode: str | None = None,
        require_enabled: bool = True,
    ) -> None:
        self._require_connected()
        if require_enabled and not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        if len(positions) != len(self.config.arm_joints):
            raise ValueError(f"expected {len(self.config.arm_joints)} joint positions, got {len(positions)}")
        if any(not math.isfinite(float(value)) for value in positions):
            raise ValueError("joint positions must be finite")
        target = {
            joint.name: float(value)
            for joint, value in zip(self.config.arm_joints, positions)
        }
        active_mode = (mode or self._mode).strip().lower().replace("_", "")
        if active_mode in ("posvel", "pv"):
            if self._mode != "posvel":
                self.configure_mode("posvel")
            self.robot.arm.send_pos_vel(np.array([target[joint.name] for joint in self.config.arm_joints]))
            return
        if active_mode == "mit":
            if self._mode != "mit":
                self.configure_mode("mit")
            self.robot.arm.send_mit(np.array([target[joint.name] for joint in self.config.arm_joints]))
            return
        raise ValueError("mode must be 'posvel' or 'mit'")

    def move_joint_positions(
        self,
        positions: Sequence[float],
        *,
        start: Sequence[float] | None = None,
        move_seconds: float = 0.0,
        hz: float | None = None,
        max_joint_vel_rad_s: float | None = None,
    ) -> None:
        from ..execution.joint_executor import interpolate_joint_positions

        end = tuple(float(value) for value in positions)
        if start is None:
            start = self.read_state(request_feedback=True).arm.positions
        if move_seconds > 0.0:
            interpolate_joint_positions(
                self,
                start,
                end,
                seconds=move_seconds,
                hz=hz,
            )
            return
        interpolate_joint_positions(
            self,
            start,
            end,
            seconds=None,
            hz=hz,
            max_joint_vel_rad_s=max_joint_vel_rad_s,
        )

    def hold_current_position(self) -> ArxDCanState:
        state = self.read_state(request_feedback=True)
        self.send_joint_positions(state.arm.positions)
        return state

    def set_zero(self) -> None:
        self._require_connected()
        self.robot.set_zero()

    def set_gripper(
        self,
        value: float,
        *,
        require_enabled: bool = True,
    ) -> None:
        """Set gripper opening. User scale: 0 = closed, 1000 = open (Alicia-M compatible)."""
        self._require_connected()
        if self.config.gripper is None:
            return
        if require_enabled and not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        motor_value = user_to_motor(
            value,
            closed_motor=self.config.gripper_closed_value,
            open_motor=self.config.gripper_open_value,
        )
        self.set_gripper_motor_value(motor_value, require_enabled=require_enabled)

    def get_gripper_value(self, *, request_feedback: bool = True) -> float | None:
        """Read gripper opening in user scale [0, 1000] (0 = closed, 1000 = open)."""
        self._require_connected()
        if self.config.gripper is None:
            return None
        state = self.read_state(request_feedback=request_feedback)
        if state.gripper is None:
            return None
        return motor_to_user(
            state.gripper.position,
            closed_motor=self.config.gripper_closed_value,
            open_motor=self.config.gripper_open_value,
        )

    def set_gripper_motor_value(
        self,
        value: float,
        *,
        require_enabled: bool = True,
    ) -> None:
        self._require_connected()
        if self.config.gripper is None:
            return
        if not self.enable_gripper:
            raise RuntimeError("ARX-D-CAN gripper is disabled; create ArxDCanArm(enable_gripper=True)")
        if require_enabled and not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        self.robot.gripper.send_mit(np.array([float(value)]))

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("ARX-D-CAN arm is not connected")
