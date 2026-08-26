#!/usr/bin/env python3
"""Safely diagnose one MIT joint with status, torque, and a small optional step.

Status only (holds the current pose):
    python examples/11_diagnose_mit_joint.py

Small signed step on one joint, then restore the starting pose:
    python examples/11_diagnose_mit_joint.py --robot follower
    python examples/11_diagnose_mit_joint.py --robot follower --joint 1 --step 0.05
    python examples/11_diagnose_mit_joint.py --joint 2 --step -0.03
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from common import (
    add_robot_arguments,
    ensure_native_path,
    resolve_robot_setup,
    robot_gravity_kwargs,
)

ensure_native_path()
import _native  # noqa: E402


def print_diagnostics(robot) -> list[dict]:
    diagnostics = list(robot.get_motor_diagnostics(True))[:6]
    print("joint status          q(rad)    vel(rad/s)  torque(Nm)  MOS(C) rotor(C)")
    print("-" * 76)
    for i, item in enumerate(diagnostics, start=1):
        print(
            f"j{i:<4} {item['status_name']:<12} "
            f"{item['position']:>9.4f} {item['velocity']:>12.4f} "
            f"{item['torque']:>11.3f} {item['mos_temperature_c']:>7.1f} "
            f"{item['rotor_temperature_c']:>8.1f}"
        )
    return diagnostics


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None)
    add_robot_arguments(parser)
    parser.add_argument("--joint", type=int, choices=range(1, 7))
    parser.add_argument(
        "--step",
        type=float,
        default=None,
        help="Signed step in rad; requires --joint and is limited to |step| <= 0.10",
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--sample-hz", type=float, default=100.0)
    args = parser.parse_args()

    if (args.joint is None) != (args.step is None):
        raise SystemExit("--joint and --step must be provided together")
    if args.step is not None and (not math.isfinite(args.step) or abs(args.step) > 0.10):
        raise SystemExit("--step must be finite and satisfy |step| <= 0.10 rad")

    setup = resolve_robot_setup(args)
    robot = _native.Robot(
        port=args.port,
        **robot_gravity_kwargs(setup),
    )
    try:
        q_start = list(robot.get_joint_positions()[:6])
        print("MIT diagnostics (nominal refresh target: 500 Hz)")
        print(f"q_start={[round(value, 4) for value in q_start]}")
        loop_before = robot.get_control_loop_stats()
        rate_start = time.perf_counter()
        time.sleep(1.0)
        rate_elapsed = time.perf_counter() - rate_start
        loop_after = robot.get_control_loop_stats()
        refresh_hz = (
            loop_after["sent_commands"] - loop_before["sent_commands"]
        ) / rate_elapsed
        refresh_errors = loop_after["send_errors"] - loop_before["send_errors"]
        print(
            f"measured_refresh={refresh_hz:.1f} Hz  "
            f"send_errors={refresh_errors}"
        )
        initial = print_diagnostics(robot)
        bad = [item for item in initial if item["status_name"] != "ENABLED"]
        if bad:
            print("\nFAIL: not all arm motors report ENABLED; no motion command sent.")
            return 2
        if args.step is None:
            print("\nStatus-only check complete; no motion command was sent.")
            return 0

        joint = args.joint
        index = joint - 1
        q_target = q_start.copy()
        q_target[index] += args.step
        print(
            f"\nSmall-step test: j{joint} {q_start[index]:.4f} -> "
            f"{q_target[index]:.4f} rad"
        )
        robot.set_robot_state(
            target_joints=q_target,
            speed=400.0,
            wait_for_completion=False,
        )

        samples: list[tuple[float, float, float, str]] = []
        period = 1.0 / max(args.sample_hz, 1.0)
        start = time.perf_counter()
        deadline = start + max(args.duration, 0.1)
        while time.perf_counter() < deadline:
            diagnostic = robot.get_motor_diagnostics(True)[index]
            samples.append(
                (
                    time.perf_counter() - start,
                    float(diagnostic["position"]),
                    float(diagnostic["torque"]),
                    str(diagnostic["status_name"]),
                )
            )
            time.sleep(period)

        robot.set_robot_state(
            target_joints=q_start,
            speed=400.0,
            wait_for_completion=False,
        )
        time.sleep(1.0)

        elapsed = samples[-1][0] if samples else 0.0
        final_q = samples[-1][1] if samples else q_start[index]
        delta_q = final_q - q_start[index]
        peak_tau = max((abs(sample[2]) for sample in samples), default=0.0)
        statuses = sorted({sample[3] for sample in samples})
        sample_rate = len(samples) / elapsed if elapsed > 0.0 else 0.0
        response_ratio = abs(delta_q / args.step) if args.step else 0.0
        final_error = q_target[index] - final_q

        print(
            f"result: delta_q={delta_q:+.5f} rad  requested={args.step:+.5f} rad  "
            f"response={response_ratio:.1%}"
        )
        print(f"final_error={final_error:+.5f} rad")
        print(
            f"peak|torque|={peak_tau:.3f} Nm  statuses={statuses}  "
            f"diagnostic_sample_rate={sample_rate:.1f} Hz"
        )
        print_diagnostics(robot)
        if response_ratio < 0.20:
            print(
                "\nFAIL: joint produced <20% of the requested motion. "
                "Use torque sign/magnitude and status above to distinguish "
                "hard-stop/sign, disabled/fault, or missing command."
            )
            return 1
        if abs(final_error) > 0.01:
            print(
                "\nPARTIAL: the joint responds in the commanded direction, but it did "
                "not reach the 0.01 rad accuracy target within the test duration."
            )
            return 1
        print("\nPASS: joint responded and reached the 0.01 rad accuracy target.")
        return 0
    finally:
        robot.disable()
        robot.disconnect()


if __name__ == "__main__":
    sys.exit(main())
