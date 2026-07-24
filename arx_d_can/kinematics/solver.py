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
    "max_iters": 5000,
    "pos_tol": 1e-2,
    "ori_tol": 1e-2,
    "use_analytic_jacobian": True,
    "num_initial_guesses": 10,
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


def _ik_success(result: Any) -> bool:
    if not isinstance(result, dict):
        return False
    ok = result.get("success", False)
    if isinstance(ok, list):
        ok = ok[0] if ok else False
    return bool(ok)


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
    kwargs = {**DEFAULT_IK_KWARGS, **(ik_kwargs or {})}
    method = str(kwargs.pop("method", "dls"))
    kwargs.pop("initial_guess_strategy", None)
    use_analytic_jacobian = bool(kwargs.pop("use_analytic_jacobian", True))
    max_iters = int(kwargs.pop("max_iters", 5000))
    num_initial_guesses = int(kwargs.pop("num_initial_guesses", 10))
    pos_tol_current = float(kwargs.pop("pos_tol", 1e-1))
    ori_tol_current = float(kwargs.pop("ori_tol", 1e-1))
    pos_tol_random = float(kwargs.pop("pos_tol_random", 1e-2))
    ori_tol_random = float(kwargs.pop("ori_tol_random", 1e-2))

    solver_common = {
        "method": method,
        "backend": backend,
        "use_analytic_jacobian": use_analytic_jacobian,
        "max_iters": max_iters,
        "num_initial_guesses": num_initial_guesses,
        **kwargs,
    }

    # Stage 1: seed from current joints ("current" strategy via q0).
    ik = inverse_kinematics(
        model,
        target,
        q0,
        initial_guess_strategy="random",
        pos_tol=pos_tol_current,
        ori_tol=ori_tol_current,
        **solver_common,
    )
    if not _ik_success(ik):
        # Stage 2: random initial guesses without a seed.
        ik = inverse_kinematics(
            model,
            target,
            None,
            initial_guess_strategy="random",
            pos_tol=pos_tol_random,
            ori_tol=ori_tol_random,
            **solver_common,
        )

    if not _ik_success(ik):
        pos = np.asarray(position, dtype=np.float64).reshape(3)
        quat = ScipyRotation.from_matrix(
            np.asarray(rotation, dtype=np.float64).reshape(3, 3)
        ).as_quat()
        print("position", pos)
        print("quaternion", quat)
        return None
    q = to_numpy(ik["q"]).reshape(-1)
    return q[:NUM_ARM_JOINTS].tolist()
