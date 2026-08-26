"""SynriaRobotAPI — user-level interface for ARX-M-CAN arms."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Sequence

try:
    import _native
except ImportError as exc:
    raise ImportError(
        "arx_m_can native extension not found. "
        "Build and install with: maturin develop (from the arx_m_can crate root)"
    ) from exc

NUM_ARM_JOINTS = 6
DEFAULT_PORT = _native.DEFAULT_PORT
DEFAULT_BAUD = _native.DEFAULT_BAUD
DEFAULT_JOINT_SPEED = 400.0
DEFAULT_GRIPPER_SPEED = 100.0
DEFAULT_MAX_JOINT_ACCEL_RAD_S2 = getattr(
    _native, "DEFAULT_MAX_JOINT_ACCEL_RAD_S2", 8.0
)
DEFAULT_MAX_JOINT_JERK_RAD_S3 = getattr(
    _native, "DEFAULT_MAX_JOINT_JERK_RAD_S3", 50.0
)
DEFAULT_ARM_CONTROL_MODE = getattr(_native, "DEFAULT_ARM_CONTROL_MODE", "mit")
DEFAULT_WAIT_TOLERANCE_RAD = getattr(_native, "DEFAULT_WAIT_TOLERANCE_RAD", 0.02)
DEFAULT_WAIT_TIMEOUT_SEC = 15.0
DEFAULT_GRAVITY_SCALES = (
    Path(__file__).resolve().parent.parent / "arx_m_can" / "config" / "gravity_joint_scales.yaml"
)
ALICIA_SPEED_MAX = 400.0
HOME_JOINTS = (0.0,) * NUM_ARM_JOINTS

from . import kinematics as _kinematics  # noqa: E402

DEFAULT_URDF = _kinematics.DEFAULT_URDF
DEFAULT_BASE_LINK = _kinematics.DEFAULT_BASE_LINK
DEFAULT_END_LINK = _kinematics.DEFAULT_END_LINK


def _load_gravity_scales(path: Path = DEFAULT_GRAVITY_SCALES) -> list[float]:
    scales: list[float] = []
    in_block = False
    for line in path.read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if text.startswith("joint_scales:"):
            in_block = True
            continue
        if in_block and text.startswith("- "):
            scales.append(float(text[2:].split("#", 1)[0]))
        elif in_block and text and not text.startswith("#"):
            break
    if len(scales) != NUM_ARM_JOINTS:
        raise ValueError(f"Expected {NUM_ARM_JOINTS} gravity scales in {path}, got {len(scales)}")
    return scales


def _to_rad(values: Sequence[float], joint_format: str) -> list[float]:
    if joint_format == "deg":
        return [float(math.radians(v)) for v in values[:NUM_ARM_JOINTS]]
    return [float(v) for v in values[:NUM_ARM_JOINTS]]


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
        self._port = port
        self._baud = int(baud)
        self._control_hz = float(control_hz)
        self._arm_control_mode = str(arm_control_mode)
        self._wait_tolerance_rad = float(wait_tolerance_rad)
        self._wait_timeout_sec = float(wait_timeout_sec)
        self._urdf_path = Path(urdf_path)
        self._base_link = str(base_link)
        self._end_link = str(end_link)
        self._robot_model = None
        scales = list(gravity_joint_scales) if gravity_joint_scales is not None else None
        if use_gravity_ff and scales is None:
            scales = _load_gravity_scales()
        robot_kwargs: dict[str, Any] = {
            "port": port,
            "baud": self._baud,
            "control_hz": self._control_hz,
            "auto_enable": auto_enable,
            "arm_control_mode": self._arm_control_mode,
            "wait_tolerance_rad": self._wait_tolerance_rad,
            "wait_timeout_sec": self._wait_timeout_sec,
            "use_gravity_ff": use_gravity_ff,
            "gravity_joint_scales": scales,
        }
        if gravity_model_path is not None:
            robot_kwargs["gravity_model_path"] = str(gravity_model_path)
        if config_path is not None:
            robot_kwargs["config_path"] = str(config_path)
        self._robot = _native.Robot(**robot_kwargs)

    @property
    def arm_control_mode(self) -> str:
        return str(self._robot.arm_control_mode())

    @property
    def enabled(self) -> bool:
        return bool(self._robot.enabled())

    def connect(self) -> bool:
        """Connect is performed in __init__; kept for Alicia-D API parity."""
        return True

    def disconnect(self) -> None:
        self._robot.disconnect()

    def enable(self) -> None:
        self._robot.enable()

    def disable(self) -> None:
        self._robot.disable()

    def is_connected(self) -> bool:
        return True

    def get_robot_state(self, info_type: str = "joint") -> Any:
        joints = self._robot.get_joint_positions()
        if info_type == "joint":
            return joints
        if info_type == "joint_gripper":
            return {"angles": joints, "gripper": None}
        raise NotImplementedError(f"info_type={info_type!r} not supported")

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
        **_: Any,
    ) -> bool:
        """Command joints with Ruckig jerk-limited trajectories in MIT mode."""
        joints = None
        if target_joints is not None:
            joints = _to_rad(target_joints, joint_format)
        return bool(
            self._robot.set_robot_state(
                joints,
                gripper_value,
                float(speed),
                float(gripper_speed),
                bool(wait_for_completion),
                max_joint_accel_rad_s2,
                max_joint_jerk_rad_s3,
                joint_tolerance,
                converge_timeout,
            )
        )

    def set_gripper_target(
        self,
        command: str | None = None,
        value: float | None = None,
        *,
        speed: float = DEFAULT_GRIPPER_SPEED,
        wait_for_completion: bool = True,
    ) -> bool:
        return bool(
            self._robot.set_gripper_target(
                command,
                value,
                float(speed),
                bool(wait_for_completion),
            )
        )

    def go_home(self, *, speed: float = 40.0) -> None:
        self._robot.go_home(float(speed))

    def set_home(self, *, speed: float = 40.0) -> None:
        """Alias matching Alicia-D SDK naming."""
        self.go_home(speed=speed)

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
        """Compute FK for ``q`` (rad) or current measured joints."""
        joints = list(q) if q is not None else self.get_robot_state("joint")[:NUM_ARM_JOINTS]
        return _kinematics.forward_kinematics(self.robot_model, joints)

    def get_pose(self, q: Sequence[float] | None = None) -> dict[str, Any]:
        """Return end-effector pose dict (position, rotation, euler, quaternion, transform)."""
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
        **ik_kwargs: Any,
    ) -> dict[str, Any]:
        """Solve IK (RoboCore) and optionally move the arm (MIT trajectory)."""
        strategy = initial_guess_strategy
        q0 = None
        if strategy == "current":
            q0 = self.get_robot_state("joint")[:NUM_ARM_JOINTS]
            strategy = "random"

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
        motion = False
        if q is not None and (execute and (result.get("success") or force_execute)):
            motion = self.set_robot_state(
                q,
                speed=speed,
                wait_for_completion=wait_for_completion,
            )
        result["motion_executed"] = bool(motion)
        return result


class Arm:
    """Low-level ARX-M-CAN arm access (scan, zero, raw state)."""

    def __init__(
        self,
        *,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
        enable_gripper: bool = False,
    ) -> None:
        self._arm = _native.Arm(port=port, baud=int(baud), enable_gripper=enable_gripper)

    def connect(self) -> None:
        self._arm.connect()

    def close(self, *, disable: bool = True) -> None:
        self._arm.close(disable=disable)

    def connected(self) -> bool:
        return bool(self._arm.connected())

    def read_state(self, *, request_feedback: bool = True) -> dict[str, Any]:
        return dict(self._arm.read_state(request_feedback=request_feedback))

    def scan_ids(
        self,
        *,
        start_id: int = 1,
        end_id: int = 16,
        model: str = "4340P",
        timeout_ms: int = 30,
        feedback_base: str = "0x10",
    ) -> list[int]:
        return list(
            self._arm.scan_ids(
                start_id=start_id,
                end_id=end_id,
                model=model,
                timeout_ms=timeout_ms,
                feedback_base=feedback_base,
            )
        )

    def set_zero(self) -> None:
        self._arm.set_zero()


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


class Teleoperation:
    """Alicia-D leader → ARX-M-CAN follower teleoperation (Rust loop)."""

    def __init__(
        self,
        follower: SynriaRobotAPI,
        *,
        frequency_hz: float = 60.0,
        follower_speed: float = 400.0,
        gripper_speed: float = 400.0,
        gripper_scale: float = 1.0,
        use_joint_mapping: bool = True,
        max_joint_accel_rad_s2: float | None = None,
        max_joint_jerk_rad_s3: float | None = None,
    ) -> None:
        self._follower = follower
        self._teleop = _native.Teleoperation(
            follower._robot,
            frequency_hz=float(frequency_hz),
            follower_speed=float(follower_speed),
            gripper_speed=float(gripper_speed),
            gripper_scale=float(gripper_scale),
            use_joint_mapping=bool(use_joint_mapping),
            max_joint_accel_rad_s2=max_joint_accel_rad_s2,
            max_joint_jerk_rad_s3=max_joint_jerk_rad_s3,
        )

    def start(self, leader: Any) -> None:
        self._teleop.start(leader)

    def stop(self) -> None:
        self._teleop.stop()

    def disconnect_follower(self) -> None:
        self._teleop.disconnect_follower()


class SparkVisBridge:
    """Bridge SparkVis joint_update WebSocket stream to an ARX-M-CAN robot."""

    def __init__(
        self,
        *,
        ws_url: str = "ws://127.0.0.1:8000/ws/teleop",
        http_base: str = "http://127.0.0.1:8000",
        speed: float = 100.0,
        gripper_speed: float = 100.0,
        command_hz: float = 50.0,
        watchdog_sec: float = 0.35,
        control_gripper: bool = False,
        dry_run: bool = False,
        sync_initial: bool = True,
        max_joint_accel_rad_s2: float | None = None,
        max_joint_jerk_rad_s3: float | None = None,
    ) -> None:
        self._bridge = _native.SparkVisBridge(
            ws_url=ws_url,
            http_base=http_base,
            speed=float(speed),
            gripper_speed=float(gripper_speed),
            command_hz=float(command_hz),
            watchdog_sec=float(watchdog_sec),
            control_gripper=bool(control_gripper),
            dry_run=bool(dry_run),
            sync_initial=bool(sync_initial),
            max_joint_accel_rad_s2=max_joint_accel_rad_s2,
            max_joint_jerk_rad_s3=max_joint_jerk_rad_s3,
        )

    def connect_robot(self, robot: SynriaRobotAPI) -> None:
        self._bridge.connect_robot(robot._robot)

    def arm(self) -> None:
        self._bridge.arm()

    def disarm(self) -> None:
        self._bridge.disarm()

    def start(self) -> None:
        self._bridge.start()

    def shutdown(self) -> None:
        self._bridge.shutdown()


def create_robot(
    port: str = DEFAULT_PORT,
    *,
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
) -> SynriaRobotAPI:
    """Factory helper — mirrors Alicia-D-SDK `create_robot`."""
    return SynriaRobotAPI(
        port=port or DEFAULT_PORT,
        baud=baud,
        control_hz=control_hz,
        auto_enable=auto_enable,
        arm_control_mode=arm_control_mode,
        wait_tolerance_rad=wait_tolerance_rad,
        wait_timeout_sec=wait_timeout_sec,
        use_gravity_ff=use_gravity_ff,
        gravity_joint_scales=gravity_joint_scales,
        gravity_model_path=gravity_model_path,
        config_path=config_path,
        urdf_path=urdf_path,
        base_link=base_link,
        end_link=end_link,
    )
