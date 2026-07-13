"""RoboCore kinematics for Alicia-M MJCF variants (ARX-D-CAN slave)."""

from __future__ import annotations

from typing import Any

import numpy as np
import robocore as rc
from robocore.kinematics import forward_kinematics, inverse_kinematics
from robocore.modeling.robot_model import RobotModel
from robocore.utils.backend import to_numpy
from scipy.spatial.transform import Rotation as ScipyRotation
from synriard import get_model_path

NUM_ARM_JOINTS = 6
DEFAULT_BACKEND = "cpp"
DEFAULT_IK_KWARGS = {
    "method": "dls",
    "max_iters": 200,
    "pos_tol": 1e-3,
    "ori_tol": 8e-3,
    "use_analytic_jacobian": True,
    "num_initial_guesses": 3,
    "initial_guess_strategy": "random",
}


def _make_target_matrix(position: Any, rotation: Any) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    T[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    return T


def _chain_q_from_arm_joints(model: RobotModel, arm_joints: Any) -> np.ndarray:
    q_arm = np.asarray(arm_joints, dtype=float).reshape(-1)[:NUM_ARM_JOINTS]
    n = int(model.num_chain_dof)
    if q_arm.size < n:
        padded = np.zeros(n, dtype=float)
        padded[: q_arm.size] = q_arm
        return padded
    return q_arm[:n]


class IkSolver:
    """RoboCore IK/FK for Alicia-M MJCF variants (e.g. follower_ARX)."""

    def __init__(
        self,
        robot_key: str = "Alicia_M",
        version: str = "v1_2",
        variant: str = "follower_ARX",
        end_frame: str = "tool0",
        base_link: str = "base_link",
        *,
        backend: str = DEFAULT_BACKEND,
        ik_kwargs: dict[str, Any] | None = None,
    ) -> None:
        mjcf_path = get_model_path(
            robot_key, version=version, variant=variant, model_format="mjcf"
        )
        rc.set_backend(backend)
        self._model = RobotModel(
            str(mjcf_path), base_link=base_link, end_link=end_frame
        )
        self._base_link = base_link
        self._end_link = end_frame
        self._backend = backend
        self._ik_kwargs = {**DEFAULT_IK_KWARGS, **(ik_kwargs or {})}

    @property
    def model(self) -> RobotModel:
        return self._model

    @property
    def base_link(self) -> str:
        return self._base_link

    @property
    def end_link(self) -> str:
        return self._end_link

    def tool0_pose(self, q) -> np.ndarray:
        q_chain = _chain_q_from_arm_joints(self._model, q)
        return np.asarray(
            forward_kinematics(
                self._model,
                q_chain,
                base_link=self._base_link,
                end_link=self._end_link,
                return_end=True,
            ),
            dtype=np.float64,
        )

    def solve(self, position: Any, rotation: Any, q_seed: Any) -> list[float] | None:
        return solve_ik(
            position,
            rotation,
            q_seed=q_seed,
            model=self._model,
            base_link=self._base_link,
            end_link=self._end_link,
            backend=self._backend,
            ik_kwargs=self._ik_kwargs,
        )


def solve_ik(
    position: Any,
    rotation: Any,
    *,
    q_seed: Any,
    model: RobotModel,
    base_link: str = "base_link",
    end_link: str = "tool0",
    backend: str = DEFAULT_BACKEND,
    ik_kwargs: dict[str, Any] | None = None,
) -> list[float] | None:
    """Solve IK from position (3,) and rotation (3, 3). Returns joint rad or None."""
    q0 = _chain_q_from_arm_joints(model, q_seed)
    target = _make_target_matrix(position, rotation)
    kwargs = {**(ik_kwargs or DEFAULT_IK_KWARGS)}
    method = str(kwargs.pop("method", "dls"))
    result = inverse_kinematics(
        model,
        target,
        q0,
        method=method,
        backend=backend,
        **kwargs,
    )
    if not isinstance(result, dict) or not result.get("success", False):
        pos = np.asarray(position, dtype=np.float64).reshape(3)
        quat = ScipyRotation.from_matrix(
            np.asarray(rotation, dtype=np.float64).reshape(3, 3)
        ).as_quat()
        print("position", pos)
        print("quaternion", quat)
        return None
    q = to_numpy(result["q"]).reshape(-1)
    return q[:NUM_ARM_JOINTS].tolist()


def solve_slave_ik(robot, position, rotation) -> list[float] | None:
    """Solve IK for an ARX-D-CAN slave robot wrapper."""
    model = getattr(robot, "robot_model", None)
    if model is None:
        return None

    q0 = robot.get_ik_seed_joints() if hasattr(robot, "get_ik_seed_joints") else robot.get_robot_state("joint")
    if q0 is None:
        return None

    ik = getattr(robot, "_ik", None)
    return solve_ik(
        position,
        rotation,
        q_seed=q0,
        model=model,
        base_link=getattr(ik, "base_link", "base_link"),
        end_link=getattr(ik, "end_link", "tool0"),
        backend=getattr(ik, "_backend", DEFAULT_BACKEND),
        ik_kwargs=getattr(ik, "_ik_kwargs", DEFAULT_IK_KWARGS),
    )
