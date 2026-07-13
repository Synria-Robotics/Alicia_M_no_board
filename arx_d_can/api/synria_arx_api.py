"""User-facing API — connect, set_robot_state, get_pose, go_home."""

from __future__ import annotations

from typing import Any

import numpy as np

from ..execution.streaming_loop import JointStreamServo
from ..hardware import ArxDCanArm
from ..kinematics import IkSolver
from ..utils.gripper import validate_gripper_value

NUM_ARM_JOINTS = 6
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1_000_000
DEFAULT_MOVE_HZ = 100.0
DEFAULT_MOVE_SECONDS = 1.0
DEFAULT_MAX_JOINT_VEL_RAD_S = 1.0
DEFAULT_ROBOT_NAME = "Alicia_M"
DEFAULT_ROBOT_VERSION = "v1_2"
DEFAULT_ROBOT_VARIANT = "follower_ARX"
DEFAULT_END_FRAME = "tool0"
HOME_JOINTS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def resolve_slave_port(arm_cfg: dict[str, Any], *, port_override: str | None = None) -> str:
    if port_override and str(port_override).strip():
        return str(port_override).strip()
    port = arm_cfg.get("slave_port")
    if port is None or not str(port).strip():
        raise ValueError("arms.slave_port is required for ARX-D-CAN slave")
    return str(port).strip()


def resolve_slave_motion(arm_cfg: dict[str, Any]) -> dict[str, float]:
    """Read ARX slave motion settings from bilateral arms config."""
    return {
        "move_seconds": float(arm_cfg.get("slave_move_seconds", DEFAULT_MOVE_SECONDS)),
        "move_hz": float(arm_cfg.get("slave_move_hz", DEFAULT_MOVE_HZ)),
        "max_joint_vel_rad_s": float(
            arm_cfg.get("slave_joint_vel_rad_s", DEFAULT_MAX_JOINT_VEL_RAD_S)
        ),
    }


class ArxDCanRobot:
    """Thin wrapper so teleop/calibration can treat ARX-D-CAN like alicia_m_sdk."""

    def __init__(
        self,
        *,
        port: str,
        baud: int = DEFAULT_BAUD,
        robot_name: str = DEFAULT_ROBOT_NAME,
        robot_version: str = DEFAULT_ROBOT_VERSION,
        robot_variant: str = DEFAULT_ROBOT_VARIANT,
        end_frame: str = DEFAULT_END_FRAME,
        move_seconds: float = DEFAULT_MOVE_SECONDS,
        move_hz: float = DEFAULT_MOVE_HZ,
        max_joint_vel_rad_s: float = DEFAULT_MAX_JOINT_VEL_RAD_S,
    ) -> None:
        self._move_seconds = float(move_seconds)
        self._move_hz = float(move_hz)
        self._max_joint_vel_rad_s = float(max_joint_vel_rad_s)
        self._arm = ArxDCanArm(port=port, baud=baud, control_mode="posvel", enable_gripper=True)
        self._end_frame = end_frame
        self._ik = IkSolver(
            robot_key=robot_name,
            version=robot_version,
            variant=robot_variant,
            end_frame=end_frame,
        )
        self._arm.connect()
        self._arm.configure()
        self._arm.enable()

        q0 = np.asarray(self._read_joint_feedback(), dtype=np.float64).reshape(NUM_ARM_JOINTS)
        self._stream = JointStreamServo(
            hz=self._move_hz,
            max_joint_vel_rad_s=self._max_joint_vel_rad_s,
            send_fn=lambda q: self._arm.send_joint_positions(q),
            initial_cmd=q0,
        )

    @property
    def robot_model(self):
        return self._ik.model

    def _read_joint_feedback(self) -> list[float]:
        state = self._arm.read_state(request_feedback=True)
        return list(state.arm.positions[:NUM_ARM_JOINTS])

    def get_ik_seed_joints(self) -> list[float]:
        """Commanded joints for IK seed (tracks stream target, not lagging feedback)."""
        return self._stream.commanded_joints.astype(float).reshape(-1).tolist()

    def get_robot_state(self, info_type: str = "joint") -> Any:
        if info_type == "joint":
            return self._read_joint_feedback()
        if info_type == "joint_gripper":
            gripper = self._arm.get_gripper_value(request_feedback=True)
            if gripper is None:
                return None
            return {
                "angles": self._read_joint_feedback(),
                "gripper": gripper,
            }
        raise NotImplementedError(f"ARX-D-CAN wrapper does not support info_type={info_type!r}")

    def get_pose(self) -> dict[str, np.ndarray] | None:
        from scipy.spatial.transform import Rotation as ScipyRotation

        joints = self.get_robot_state("joint")
        if joints is None:
            return None
        transform = self._ik.tool0_pose(joints)
        pos = transform[:3, 3].copy()
        rot = transform[:3, :3].copy()
        quat_xyzw = ScipyRotation.from_matrix(rot).as_quat()
        return {
            "transform": transform,
            "position": pos,
            "rotation": rot,
            "euler_xyz": ScipyRotation.from_matrix(rot).as_euler("xyz"),
            "quaternion_xyzw": quat_xyzw,
        }

    def set_robot_state(
        self,
        target_joints: list[float] | None = None,
        *,
        gripper_value: int | None = None,
        joint_format: str = "rad",
        speed: float = 20.0,
        move_seconds: float | None = None,
        **_: Any,
    ) -> bool:
        del speed  # Alicia-M scale; ARX streaming uses slave_joint_vel_rad_s from config.
        if target_joints is not None:
            if joint_format == "deg":
                target = [float(np.deg2rad(v)) for v in target_joints[:NUM_ARM_JOINTS]]
            else:
                target = [float(v) for v in target_joints[:NUM_ARM_JOINTS]]
            target_arr = np.asarray(target, dtype=np.float64).reshape(NUM_ARM_JOINTS)

            if move_seconds is not None and float(move_seconds) > 0.0:
                self._blocking_move(target_arr, move_seconds=float(move_seconds))
            else:
                self._stream.set_target(target_arr)

        if gripper_value is not None:
            self._arm.set_gripper(validate_gripper_value(float(gripper_value)))

        return True

    def _blocking_move(self, target: np.ndarray, *, move_seconds: float) -> None:
        def move_fn(start, end, seconds):
            self._arm.move_joint_positions(
                list(end),
                start=start,
                move_seconds=seconds,
                hz=self._move_hz,
            )

        self._stream.blocking_move(target, move_fn=move_fn, move_seconds=move_seconds)

    def go_home(self, *, move_seconds: float | None = None, speed: float = 40.0) -> None:
        del speed  # Alicia-M; ARX uses move_seconds from config.
        self.set_robot_state(
            list(HOME_JOINTS),
            joint_format="rad",
            move_seconds=self._move_seconds if move_seconds is None else move_seconds,
        )

    def disconnect(self) -> None:
        self._stream.stop()
        self._arm.close(disable=False)
