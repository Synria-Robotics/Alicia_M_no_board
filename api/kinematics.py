"""RoboCore kinematics wrappers for ARX-M-CAN."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np

DEFAULT_URDF = Path(
    "/home/ubuntu/Alicia/Synria-Robot-Descriptions/synriard/urdf/Alicia_M_v1_2/"
    "Alicia_M_v1_2_follower_ARX_PX6D.urdf"
)
DEFAULT_BASE_LINK = "base_link"
DEFAULT_END_LINK = "tool0"


def load_robot_model(
    urdf_path: Path | str = DEFAULT_URDF,
    *,
    base_link: str = DEFAULT_BASE_LINK,
    end_link: str = DEFAULT_END_LINK,
) -> Any:
    from robocore.modeling import RobotModel

    return RobotModel(str(urdf_path), base_link=base_link, end_link=end_link)


def forward_kinematics(robot_model: Any, q: Sequence[float]) -> np.ndarray:
    from robocore.kinematics import forward_kinematics as rc_fk

    return np.asarray(rc_fk(robot_model, q, return_end=True), dtype=np.float64)


def pose_dict_from_transform(T: np.ndarray) -> dict[str, Any]:
    from robocore.transform import matrix_to_euler, matrix_to_quaternion

    R = np.asarray(T[:3, :3], dtype=np.float64)
    p = np.asarray(T[:3, 3], dtype=np.float64)
    return {
        "transform": np.asarray(T, dtype=np.float64),
        "position": p,
        "rotation": R,
        "euler_xyz": np.asarray(matrix_to_euler(R, seq="xyz"), dtype=np.float64),
        "quaternion_xyzw": np.asarray(matrix_to_quaternion(R), dtype=np.float64),
    }


def inverse_kinematics(
    robot_model: Any,
    target_pose: Sequence[float] | np.ndarray,
    q0: Sequence[float] | None = None,
    **kwargs: Any,
) -> dict[str, Any]:
    from robocore.kinematics import inverse_kinematics as rc_ik
    from robocore.transform import make_transform, quaternion_to_matrix

    target = np.asarray(target_pose, dtype=np.float64)
    if target.shape == (7,):
        T = make_transform(quaternion_to_matrix(target[3:]), target[:3])
    elif target.shape == (4, 4):
        T = target
    else:
        raise ValueError("target_pose must be length-7 [x,y,z,qx,qy,qz,qw] or 4x4 matrix")

    result = rc_ik(robot_model, T, q0=q0, **kwargs)
    return result if isinstance(result, dict) else {"success": False, "q": None}
