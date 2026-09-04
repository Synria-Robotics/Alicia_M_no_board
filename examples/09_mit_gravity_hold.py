#!/usr/bin/env python3
"""MIT gravity-compensated hold with tracking error log.

Uses Kp/Kd from hardware YAML (``--float`` → kp=0 float config).

Run from the ``arx`` conda env after building the Python extension::

    conda activate arx
    cd /home/ubuntu/Synria/arx/Rust/alicia_m_rs
    maturin develop --release

    # Default (--robot px6d)
    python examples/09_mit_gravity_hold.py --duration 10
    python examples/09_mit_gravity_hold.py --port /dev/ttyACM1 --duration 10

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

from common import add_robot_arguments, make_robot


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_robot_arguments(parser)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--control-hz", type=float, default=500.0)
    parser.add_argument("--focus-joint", type=int, default=None, choices=range(1, 7))
    parser.add_argument("--log-hz", type=float, default=5.0, help="Print rate (Hz)")
    args = parser.parse_args()

    stop = False

    def _handle_sigint(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle_sigint)

    robot = make_robot(args, control_hz=args.control_hz)
    q_des = robot.get_commanded_joints()[:6]
    kp, kd = robot.get_arm_mit_gains()
    print(f"MIT hold at {args.control_hz:.0f} Hz, robot={args.robot}")
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
        # robot.disable()
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
