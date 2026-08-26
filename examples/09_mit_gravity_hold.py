#!/usr/bin/env python3
"""MIT gravity-compensated hold with tracking error log.

Uses Kp/Kd from hardware YAML (``--float`` → kp=0 float config).

Run from the ``arx`` conda env after building the Python extension::

    conda activate arx
    cd /home/ubuntu/Synria/arx/Rust/arx_m_can
    maturin develop --release

    # Default (root config + embedded px6d dynamics)
    python examples/09_mit_gravity_hold.py --duration 10

    # Follower arm — gravity-only float test (kp=0, follower model + scales)
    python examples/09_mit_gravity_hold.py --robot follower --float --duration 30
    python examples/09_mit_gravity_hold.py --robot follower --float --focus-joint 4
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

from common import (
    add_robot_arguments,
    ensure_native_path,
    resolve_robot_setup,
    robot_gravity_kwargs,
)

ensure_native_path()

try:
    import _native
except ImportError as exc:
    raise ImportError(
        "Native extension not found. From the arx_m_can repo root run:\n"
        "  conda activate arx\n"
        "  pip install maturin\n"
        "  maturin develop --release"
    ) from exc

Robot = _native.Robot
DEFAULT_MIT_CONTROL_HZ = _native.DEFAULT_MIT_CONTROL_HZ


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--control-hz", type=float, default=DEFAULT_MIT_CONTROL_HZ)
    parser.add_argument("--focus-joint", type=int, default=None, choices=range(1, 7))
    parser.add_argument("--log-hz", type=float, default=5.0, help="Print rate (Hz)")
    add_robot_arguments(parser)
    args = parser.parse_args()

    setup = resolve_robot_setup(args)
    scales = setup.scales

    stop = False

    def _handle_sigint(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle_sigint)

    robot = Robot(
        port=args.port,
        control_hz=args.control_hz,
        **robot_gravity_kwargs(setup),
    )
    q_des = robot.get_commanded_joints()[:6]
    kp, kd = robot.get_arm_mit_gains()
    robot_note = f"robot={setup.robot}" if setup.robot else "robot=default"
    print(f"MIT hold at {args.control_hz:.0f} Hz, {robot_note}, scales={scales}")
    if setup.config_path:
        print(f"config={setup.config_path.name}")
    if setup.dynamics_model_path:
        print(f"dynamics={setup.dynamics_model_path.name}")
    print(f"Kp={[f'{v:.1f}' for v in kp[:6]]}  Kd={[f'{v:.1f}' for v in kd[:6]]}")
    print(f"q_des = {[f'{v:.4f}' for v in q_des]}")
    print("err = q_meas - q_des. Ctrl+C to stop.\n")

    err_history: list[list[float]] = []
    t0 = time.time()
    try:
        while not stop and (time.time() - t0) < args.duration:
            state = robot.get_arm_state(True)
            q = list(state["arm"]["positions"][:6])
            tau = list(state["arm"]["torques"][:6])
            err = [q[i] - q_des[i] for i in range(6)]
            err_history.append(err)

            if args.focus_joint:
                j = args.focus_joint - 1
                line = (
                    f"j{args.focus_joint}: err={err[j]:+.4f} rad ({math.degrees(err[j]):+.3f} deg)  "
                    f"q={q[j]:.4f}  tau={tau[j]:+.2f}"
                )
            else:
                line = (
                    f"err={[f'{e:+.4f}' for e in err]}  "
                    f"tau={[f'{t:+.2f}' for t in tau]}"
                )
            print(line, end="\r", flush=True)
            time.sleep(1.0 / max(args.log_hz, 0.1))
    finally:
        robot.disable()
        robot.disconnect()
        print()

        if err_history:
            n = len(err_history)
            print(f"Summary over {n} samples ({time.time() - t0:.1f}s):")
            for i in range(6):
                col = [e[i] for e in err_history]
                rmse = math.sqrt(sum(e * e for e in col) / len(col))
                mx = max(abs(e) for e in col)
                print(
                    f"  j{i + 1}: rmse={rmse:.5f} rad ({math.degrees(rmse):.3f} deg)  "
                    f"max|err|={mx:.5f} rad ({math.degrees(mx):.3f} deg)  "
                    f"last={col[-1]:+.5f} rad"
                )
        print("Stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
