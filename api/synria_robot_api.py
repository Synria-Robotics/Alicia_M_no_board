"""SynriaRobotAPI — user-level interface for ARX-M-CAN arms."""

from __future__ import annotations

import math
import re
import threading
from collections.abc import Callable
from pathlib import Path
from typing import Any, Sequence

try:
    import _native
except ImportError as exc:
    raise ImportError(
        "alicia_m_can native extension not found. "
        "Build and install with: maturin develop (from the alicia_m_rs repo root)"
    ) from exc

from . import kinematics as _kinematics
from .profiles import DEFAULT_PROFILE, get_profile, load_gravity_scales

NUM_ARM_JOINTS = 6
DEFAULT_PORT = _native.DEFAULT_PORT
DEFAULT_BAUD = _native.DEFAULT_BAUD
DEFAULT_JOINT_SPEED = 400.0
DEFAULT_GRIPPER_SPEED = 100.0
DEFAULT_MAX_JOINT_ACCEL_RAD_S2 = getattr(_native, "DEFAULT_MAX_JOINT_ACCEL_RAD_S2", 8.0)
DEFAULT_MAX_JOINT_JERK_RAD_S3 = getattr(_native, "DEFAULT_MAX_JOINT_JERK_RAD_S3", 50.0)
DEFAULT_ARM_CONTROL_MODE = getattr(_native, "DEFAULT_ARM_CONTROL_MODE", "mit")
DEFAULT_WAIT_TOLERANCE_RAD = getattr(_native, "DEFAULT_WAIT_TOLERANCE_RAD", 0.02)
DEFAULT_WAIT_TIMEOUT_SEC = 15.0
HOME_JOINTS = (0.0,) * NUM_ARM_JOINTS
DEFAULT_URDF = _kinematics.DEFAULT_URDF
DEFAULT_BASE_LINK = _kinematics.DEFAULT_BASE_LINK
DEFAULT_END_LINK = _kinematics.DEFAULT_END_LINK


def _to_rad(values: Sequence[float], joint_format: str) -> list[float]:
    if joint_format == "deg":
        return [float(math.radians(v)) for v in values[:NUM_ARM_JOINTS]]
    return [float(v) for v in values[:NUM_ARM_JOINTS]]


_UNEXPECTED_KWARG = re.compile(r"unexpected keyword argument ['\"](\w+)['\"]")
_GRIPPER_CLOSED_MOTOR_RAD = 0.0
_GRIPPER_OPEN_MOTOR_RAD = 3.0


def _as_user_gripper(value: Any) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return max(0.0, min(1000.0, parsed))


def _motor_rad_to_user_gripper(position: Any) -> float | None:
    try:
        motor = float(position)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(motor):
        return None
    span = _GRIPPER_OPEN_MOTOR_RAD - _GRIPPER_CLOSED_MOTOR_RAD
    if abs(span) < 1e-12:
        return 0.0
    return _as_user_gripper((motor - _GRIPPER_CLOSED_MOTOR_RAD) / span * 1000.0)


def _construct_native(cls: Any, kwargs: dict[str, Any]) -> Any:
    """Build a pyo3 class, dropping kwargs the installed ``_native`` binary does not accept."""
    pending = dict(kwargs)
    while True:
        try:
            return cls(**pending)
        except TypeError as exc:
            match = _UNEXPECTED_KWARG.search(str(exc))
            if match is None or match.group(1) not in pending:
                raise
            pending.pop(match.group(1))


def _invoke_native(fn: Any, *args: Any) -> Any:
    """Call a native method, dropping trailing positional args the binary does not accept."""
    pending = list(args)
    while True:
        try:
            return fn(*pending)
        except TypeError as exc:
            msg = str(exc)
            if "positional argument" not in msg or len(pending) == 0:
                raise
            pending.pop()


class SynriaRobotAPI:
    """Unified user API for ARX-M-CAN (Damiao motors over USB2CAN)."""

    def __init__(
        self,
        *,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
        control_hz: float = 100.0,
        auto_enable: bool = True,
        arm_control_mode: str = DEFAULT_ARM_CONTROL_MODE,
        wait_tolerance_rad: float = DEFAULT_WAIT_TOLERANCE_RAD,
        wait_timeout_sec: float = DEFAULT_WAIT_TIMEOUT_SEC,
        use_gravity_ff: bool = True,
        gravity_joint_scales: Sequence[float] | None = None,
        gravity_model_path: str | Path | None = None,
        config_path: str | Path | None = None,
        urdf_path: str | Path = DEFAULT_URDF,
        base_link: str = DEFAULT_BASE_LINK,
        end_link: str = DEFAULT_END_LINK,
    ) -> None:
        self._urdf_path = Path(urdf_path)
        self._base_link = str(base_link)
        self._end_link = str(end_link)
        self._robot_model = None
        scales = list(gravity_joint_scales) if gravity_joint_scales is not None else None
        if use_gravity_ff and scales is None:
            scales = load_gravity_scales()
        kwargs: dict[str, Any] = {
            "port": port,
            "baud": int(baud),
            "control_hz": float(control_hz),
            "auto_enable": auto_enable,
            "arm_control_mode": str(arm_control_mode),
            "wait_tolerance_rad": float(wait_tolerance_rad),
            "wait_timeout_sec": float(wait_timeout_sec),
            "use_gravity_ff": use_gravity_ff,
            "gravity_joint_scales": scales,
        }
        if gravity_model_path is not None:
            kwargs["gravity_model_path"] = str(gravity_model_path)
        if config_path is not None:
            kwargs["config_path"] = str(config_path)
        self._wait_tolerance_rad = float(wait_tolerance_rad)
        self._wait_timeout_sec = float(wait_timeout_sec)
        self._robot = _construct_native(_native.Robot, kwargs)

    @property
    def arm_control_mode(self) -> str:
        return str(self._robot.arm_control_mode())

    @property
    def enabled(self) -> bool:
        return bool(self._robot.enabled())

    def connect(self) -> bool:
        """Connect is performed in __init__; kept for Alicia-D API parity."""
        return True

    def is_connected(self) -> bool:
        return True

    def get_robot_state(self, info_type: str = "joint") -> Any:
        joints = self._robot.get_joint_positions()
        if info_type == "joint":
            return joints
        if info_type == "joint_gripper":
            return {"angles": joints, "gripper": self.get_gripper_value()}
        raise NotImplementedError(f"info_type={info_type!r} not supported")

    def get_gripper_value(self, request_feedback: bool = True) -> float | None:
        """Measured gripper in user units (0=closed, 1000=open)."""
        native = getattr(self._robot, "get_gripper_value", None)
        if callable(native):
            try:
                value = native(request_feedback)
            except TypeError:
                value = native()
            parsed = _as_user_gripper(value)
            if parsed is not None:
                return parsed
        state = self._robot.get_arm_state(request_feedback)
        gripper = state.get("gripper") if isinstance(state, dict) else None
        if not isinstance(gripper, dict) or gripper.get("position") is None:
            return None
        return _motor_rad_to_user_gripper(gripper["position"])

    def get_ik_seed_joints(self) -> list[float]:
        """MIT commanded joints (reference, not lagging encoder feedback)."""
        return list(self._robot.get_commanded_joints()[:NUM_ARM_JOINTS])

    def set_robot_state(
        self,
        target_joints: Sequence[float] | None = None,
        *,
        gripper_value: float | None = None,
        joint_format: str = "rad",
        speed: float = DEFAULT_JOINT_SPEED,
        gripper_speed: float = DEFAULT_GRIPPER_SPEED,
        wait_for_completion: bool = False,
        max_joint_accel_rad_s2: float | None = None,
        max_joint_jerk_rad_s3: float | None = None,
        joint_tolerance: float | None = None,
        converge_timeout: float | None = None,
        stream: bool = False,
        gripper_tolerance: float | None = None,
        gripper_timeout_sec: float | None = None,
        **_: Any,
    ) -> bool:
        """Command joints. ``stream=False`` + wait is Ruckig Phase P2P; ``stream=True`` is the 500 Hz walker."""
        joints = None if target_joints is None else _to_rad(target_joints, joint_format)
        return bool(
            _invoke_native(
                self._robot.set_robot_state,
                joints,
                gripper_value,
                float(speed),
                float(gripper_speed),
                bool(wait_for_completion),
                max_joint_accel_rad_s2,
                max_joint_jerk_rad_s3,
                joint_tolerance,
                converge_timeout,
                bool(stream),
                gripper_tolerance,
                gripper_timeout_sec,
            )
        )

    def hold_arm_joints(
        self,
        target_joints: Sequence[float],
        *,
        speed: float = DEFAULT_JOINT_SPEED,
        joint_format: str = "rad",
    ) -> bool:
        """MIT freeze at ``target_joints`` (no walker). Sit-still / enable."""
        joints = _to_rad(target_joints, joint_format)
        native = getattr(self._robot, "hold_arm_joints", None)
        if callable(native):
            native(joints, float(speed))
            return True
        return bool(
            self.set_robot_state(
                joints,
                speed=speed,
                wait_for_completion=False,
                stream=True,
            )
        )

    def set_home(self, *, speed: float = 40.0) -> None:
        self._robot.go_home(float(speed))

    @property
    def robot_model(self) -> Any:
        if self._robot_model is None:
            self._robot_model = _kinematics.load_robot_model(
                self._urdf_path,
                base_link=self._base_link,
                end_link=self._end_link,
            )
        return self._robot_model

    def forward_kinematics(self, q: Sequence[float] | None = None) -> Any:
        joints = list(q) if q is not None else self.get_robot_state("joint")[:NUM_ARM_JOINTS]
        return _kinematics.forward_kinematics(self.robot_model, joints)

    def get_pose(self, q: Sequence[float] | None = None) -> dict[str, Any]:
        return _kinematics.pose_dict_from_transform(self.forward_kinematics(q))

    def set_pose(
        self,
        target_pose: Sequence[float],
        *,
        method: str = "dls",
        pos_tol: float = 1e-3,
        ori_tol: float = 1e-3,
        max_iters: int = 500,
        num_initial_guesses: int = 10,
        initial_guess_strategy: str = "current",
        initial_guess_scale: float = 1.0,
        random_seed: int | None = None,
        speed: float = DEFAULT_JOINT_SPEED,
        execute: bool = True,
        force_execute: bool = False,
        wait_for_completion: bool = True,
        joint_tolerance: float | None = None,
        converge_timeout: float | None = None,
        **ik_kwargs: Any,
    ) -> dict[str, Any]:
        """Solve IK (RoboCore) and optionally move the arm (MIT trajectory)."""
        strategy = initial_guess_strategy
        q0 = None
        if strategy == "current":
            q0 = self.get_ik_seed_joints()
            strategy = "random"

        if "q_prev" not in ik_kwargs:
            prev = getattr(self, "_ik_q_prev", None)
            ik_kwargs["q_prev"] = prev if prev is not None else q0
        ik_kwargs.setdefault("ema_state", self)

        result = _kinematics.inverse_kinematics(
            self.robot_model,
            target_pose,
            q0=q0,
            method=method,
            max_iters=max_iters,
            pos_tol=pos_tol,
            ori_tol=ori_tol,
            num_initial_guesses=num_initial_guesses,
            initial_guess_strategy=strategy,
            initial_guess_scale=initial_guess_scale,
            random_seed=random_seed,
            **ik_kwargs,
        )
        q = result.get("q")
        if q is not None:
            # Latch even on near-miss so the next seed is not frozen (fail cascade).
            self._ik_q_prev = list(q)[:NUM_ARM_JOINTS]
        executed = q is not None and execute and (result.get("success") or force_execute)
        converged = False
        if executed:
            converged = self.set_robot_state(
                q,
                speed=speed,
                wait_for_completion=wait_for_completion,
                stream=not wait_for_completion,
                joint_tolerance=joint_tolerance,
                converge_timeout=converge_timeout,
            )
        result["motion_executed"] = bool(executed)
        result["motion_converged"] = bool(converged)
        return result

    def __getattr__(self, name: str) -> Any:
        return getattr(self._robot, name)


class Arm:
    """Low-level ARX-M-CAN arm access (scan, zero, raw state)."""

    def __init__(
        self,
        *,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
        enable_gripper: bool = False,
        config_path: str | Path | None = None,
    ) -> None:
        kwargs: dict[str, Any] = {
            "port": port,
            "baud": int(baud),
            "enable_gripper": enable_gripper,
        }
        if config_path is not None:
            kwargs["config_path"] = str(config_path)
        self._arm = _native.Arm(**kwargs)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._arm, name)


def scan_ids(
    *,
    port: str = DEFAULT_PORT,
    baud: int = DEFAULT_BAUD,
    start_id: int = 1,
    end_id: int = 16,
    model: str = "4340P",
    timeout_ms: int = 30,
    feedback_base: str = "0x10",
) -> list[int]:
    """Scan Damiao motor IDs on a USB2CAN port without enabling the arm."""
    return list(
        _native.scan_ids(
            port=port,
            baud=int(baud),
            start_id=start_id,
            end_id=end_id,
            model=model,
            timeout_ms=timeout_ms,
            feedback_base=feedback_base,
        )
    )


def map_alicia_d_joints(joints: Sequence[float]) -> list[float]:
    """Map six Alicia-D joint angles (rad) onto the Alicia-M follower."""
    return [float(v) for v in _native.map_alicia_d_joints(list(joints)[:6])]


class Teleoperation:
    """Alicia-D leader → follower. The 500 Hz segment runs in Rust.

    The first move onto the leader pose follows ``follower_speed``. After that,
    each leader frame is a 4–8 ms segment. ``on_state`` receives
    ``(joints, gripper, loop_count, run_status_text, run_status_raw)``.
    """

    def __init__(
        self,
        follower: Any,
        *,
        frequency_hz: float = 200.0,
        follower_speed: float = 80.0,
        gripper_speed: float = 200.0,
        gripper_scale: float = 1.0,
        use_joint_mapping: bool = True,
        on_state: Callable[..., None] | None = None,
        **kwargs: Any,
    ) -> None:
        native = getattr(follower, "_robot", follower)
        self._on_state = on_state
        self._leader: Any = None
        self._loop_count = 0
        self._running = threading.Event()
        self._teleop = _native.Teleoperation(
            native,
            frequency_hz=float(frequency_hz),
            follower_speed=float(follower_speed),
            gripper_speed=float(gripper_speed),
            gripper_scale=float(gripper_scale),
            use_joint_mapping=use_joint_mapping,
            **kwargs,
        )

    def start(self, leader: Any | None = None) -> None:
        arm = leader if leader is not None else self._leader
        if arm is None:
            raise RuntimeError("leader arm is required to start teleoperation")
        self._leader = arm

        def _on_sample(joints: Any, gripper: float) -> None:
            parser = getattr(getattr(arm, "servo_driver", None), "data_parser", None)
            text = getattr(parser, "_run_status_text", None) if parser is not None else None
            raw = getattr(parser, "_run_status", None) if parser is not None else None
            self._loop_count += 1
            if self._on_state is not None:
                self._on_state(
                    joints,
                    float(gripper),
                    self._loop_count,
                    text if isinstance(text, str) else "idle",
                    raw,
                )

        self._teleop.start(arm, on_state=_on_sample)
        self._running.set()

    def stop(self) -> None:
        self._running.clear()
        self._teleop.stop()

    def disconnect_follower(self) -> None:
        self.stop()
        self._teleop.disconnect_follower()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._teleop, name)


class SparkVisBridge:
    """Bridge SparkVis joint_update WebSocket stream to an ARX-M-CAN robot."""

    def __init__(self, **kwargs: Any) -> None:
        self._bridge = _native.SparkVisBridge(**kwargs)

    def connect_robot(self, robot: SynriaRobotAPI) -> None:
        self._bridge.connect_robot(robot._robot)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._bridge, name)


def create_robot(
    port: str = DEFAULT_PORT,
    *,
    robot: str | None = None,
    baud: int = DEFAULT_BAUD,
    control_hz: float = 100.0,
    auto_enable: bool = True,
    arm_control_mode: str = DEFAULT_ARM_CONTROL_MODE,
    wait_tolerance_rad: float = DEFAULT_WAIT_TOLERANCE_RAD,
    wait_timeout_sec: float = DEFAULT_WAIT_TIMEOUT_SEC,
    use_gravity_ff: bool = True,
    gravity_joint_scales: Sequence[float] | None = None,
    gravity_model_path: str | Path | None = None,
    config_path: str | Path | None = None,
    urdf_path: str | Path | None = None,
    base_link: str | None = None,
    end_link: str | None = None,
) -> SynriaRobotAPI:
    """Factory — ``robot`` selects ``px6d`` / ``arx`` / ``follower`` (default ``px6d``)."""
    profile = get_profile(robot or DEFAULT_PROFILE)
    return SynriaRobotAPI(
        port=port or DEFAULT_PORT,
        baud=baud,
        control_hz=control_hz,
        auto_enable=auto_enable,
        arm_control_mode=arm_control_mode,
        wait_tolerance_rad=wait_tolerance_rad,
        wait_timeout_sec=wait_timeout_sec,
        use_gravity_ff=use_gravity_ff,
        gravity_joint_scales=gravity_joint_scales
        if gravity_joint_scales is not None
        else (load_gravity_scales(profile.scales_yaml) if use_gravity_ff else None),
        gravity_model_path=gravity_model_path or profile.dynamics_json,
        config_path=config_path or profile.dm_yaml,
        urdf_path=urdf_path or profile.urdf,
        base_link=base_link or profile.base_link,
        end_link=end_link or profile.end_link,
    )
