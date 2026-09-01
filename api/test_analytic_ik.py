#!/usr/bin/env python3
"""Analytical Alicia ARX IK: branch lock + SE(3) EMA."""

from __future__ import annotations

import numpy as np
import pytest

from api.analytic import ANALYTIC_AVAILABLE, try_solve
from api.analytic.solve import update_se3_ema
from api.profiles import get_profile


pytestmark = pytest.mark.skipif(not ANALYTIC_AVAILABLE, reason="ssik analytic IK not installed")


@pytest.fixture(scope="module")
def model():
    from api.kinematics import load_robot_model

    spec = get_profile("arx")
    return load_robot_model(spec.urdf, base_link=spec.base_link, end_link=spec.end_link)


def test_analytic_roundtrip(model):
    from api.analytic import alicia_arx_ik as ik
    from robocore.kinematics import forward_kinematics

    q = np.array([0.2165, -1.6069, -1.0448, -1.0474, -0.0009, 0.1932])
    T = np.asarray(forward_kinematics(model, q, return_end=True), dtype=np.float64)
    hit = try_solve(T, q, cart_ema_pos=0.0, cart_ema_rot=0.0)
    assert hit is not None
    assert hit["success"] is True
    q_hat = np.asarray(hit["q"], dtype=np.float64)
    assert np.allclose(q_hat, q, atol=1e-8)
    assert hit["pos_err"] < 1e-9
    T_ssik = np.asarray(ik.fk(q), dtype=np.float64)
    assert np.allclose(T_ssik, T, atol=1e-12)


def test_analytic_locks_nearest_seed():
    from api.analytic import alicia_arx_ik as ik

    q = np.array([0.2165, -1.6069, -1.0448, -1.0474, -0.0009, 0.1932])
    T = ik.fk(q)
    all_sols = ik.solve(T, respect_limits=False)
    assert len(all_sols) >= 1
    hit = try_solve(T, q)
    assert hit is not None
    q_hat = np.asarray(hit["q"], dtype=np.float64)
    d_lock = np.linalg.norm(np.arctan2(np.sin(q_hat - q), np.cos(q_hat - q)))
    for sol in all_sols:
        qi = np.asarray(sol.q, dtype=np.float64)[:6]
        d = np.linalg.norm(np.arctan2(np.sin(qi - q), np.cos(qi - q)))
        assert d_lock <= d + 1e-9


def test_se3_ema_reduces_joint_step():
    from api.analytic import alicia_arx_ik as ik

    q0 = np.array([0.2165, -1.6069, -1.0448, -1.0474, -0.0009, 0.1932])
    q1 = q0.copy()
    q1[3] += 0.03
    q1[5] -= 0.03
    T0 = ik.fk(q0)
    T1 = ik.fk(q1)

    class State:
        pass

    st = State()
    hit0 = try_solve(T0, q0, state=st, cart_ema_pos=0.2, cart_ema_rot=0.2)
    hit1 = try_solve(T1, np.asarray(hit0["q"]), state=st, cart_ema_pos=0.2, cart_ema_rot=0.2)
    raw = try_solve(T1, q0, cart_ema_pos=0.0, cart_ema_rot=0.0)
    assert hit0 and hit1 and raw
    dq_ema = np.max(np.abs(np.asarray(hit1["q"]) - np.asarray(hit0["q"])))
    dq_raw = np.max(np.abs(np.asarray(raw["q"]) - q0))
    assert dq_ema <= dq_raw + 1e-9


def test_ema_state_persists():
    class State:
        pass

    st = State()
    T = np.eye(4)
    T[:3, 3] = [0.2, 0.0, 0.1]
    out = update_se3_ema(st, T, alpha_pos=1.0, alpha_rot=1.0)
    assert np.allclose(out, T)
    assert np.allclose(st._ik_T_filt, T)
