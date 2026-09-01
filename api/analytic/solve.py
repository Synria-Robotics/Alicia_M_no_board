"""ssik closed-form IK + lock the branch nearest ``q_prev`` + optional SE(3) EMA.

Alicia ARX is UR-class (three inner parallel axes). Near the wrist singularity
the unique legal branch still amplifies tiny orientation noise into j3/j5
chatter, so Cartesian EMA is applied *before* IK. DLS remains the fallback.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

NUM_ARM_JOINTS = 6

try:
    from . import alicia_arx_ik as _ik

    ANALYTIC_AVAILABLE = True
except Exception:  # noqa: BLE001
    _ik = None
    ANALYTIC_AVAILABLE = False


def update_se3_ema(
    state: Any,
    T: np.ndarray,
    *,
    alpha_pos: float,
    alpha_rot: float,
    q_seed: np.ndarray | None = None,
) -> np.ndarray:
    """Exponentially smooth ``T`` in-place on ``state._ik_T_filt``."""
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    a_p = float(np.clip(alpha_pos, 0.0, 1.0))
    a_r = float(np.clip(alpha_rot, 0.0, 1.0))
    prev = getattr(state, "_ik_T_filt", None)
    if prev is None:
        if q_seed is not None and _ik is not None:
            prev = np.asarray(_ik.fk(np.asarray(q_seed, dtype=np.float64).reshape(-1)[:NUM_ARM_JOINTS]), dtype=np.float64)
        else:
            state._ik_T_filt = T.copy()
            return T
    if a_p >= 1.0 and a_r >= 1.0:
        state._ik_T_filt = T.copy()
        return T
    prev = np.asarray(prev, dtype=np.float64).reshape(4, 4)
    p = (1.0 - a_p) * prev[:3, 3] + a_p * T[:3, 3]
    key = Rotation.concatenate(
        [Rotation.from_matrix(prev[:3, :3]), Rotation.from_matrix(T[:3, :3])]
    )
    R = Slerp([0.0, 1.0], key)(a_r).as_matrix()
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = R
    out[:3, 3] = p
    state._ik_T_filt = out
    return out


def _ori_err(R_cur: np.ndarray, R_tgt: np.ndarray) -> float:
    R = np.asarray(R_tgt, dtype=np.float64) @ np.asarray(R_cur, dtype=np.float64).T
    c = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.arccos(c))


def try_solve(
    T_target: np.ndarray,
    q_seed: np.ndarray | None,
    *,
    state: Any | None = None,
    cart_ema_pos: float = 0.0,
    cart_ema_rot: float = 0.0,
) -> dict[str, Any] | None:
    """Return a RoboCore-like result dict, or ``None`` to fall back to DLS."""
    if not ANALYTIC_AVAILABLE or _ik is None:
        return None
    T = np.asarray(T_target, dtype=np.float64).reshape(4, 4)
    q0 = None if q_seed is None else np.asarray(q_seed, dtype=np.float64).reshape(-1)[:NUM_ARM_JOINTS]
    if state is not None and (cart_ema_pos > 0.0 or cart_ema_rot > 0.0):
        T = update_se3_ema(
            state,
            T,
            alpha_pos=cart_ema_pos if cart_ema_pos > 0.0 else 1.0,
            alpha_rot=cart_ema_rot if cart_ema_rot > 0.0 else 1.0,
            q_seed=q0,
        )
    try:
        sols = _ik.solve(
            T,
            q_seed=q0,
            max_solutions=1,
            respect_limits=True,
            seed_metric="wrap_linf",
        )
    except Exception:  # noqa: BLE001
        return None
    if not sols:
        return None
    q = np.asarray(sols[0].q, dtype=np.float64).reshape(-1)[:NUM_ARM_JOINTS]
    T_got = np.asarray(_ik.fk(q), dtype=np.float64)
    pos_err = float(np.linalg.norm(T_got[:3, 3] - T[:3, 3]))
    ori_err = _ori_err(T_got[:3, :3], T[:3, :3])
    return {
        "q": q.tolist(),
        "success": True,
        "iters": 1,
        "err_norm": float(getattr(sols[0], "fk_residual", pos_err + ori_err)),
        "method": "analytic",
        "pos_err": pos_err,
        "ori_err": ori_err,
        "backend": "ssik",
    }
