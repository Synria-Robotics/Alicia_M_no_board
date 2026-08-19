#!/usr/bin/env python3
"""Verify MIT hold accuracy across calibration poses (gravity FF on).

Uses Ruckig jerk-limited trajectories in the 500 Hz refresh loop with gravity FF.

    conda activate arx
    cd /home/ubuntu/Synria/arx/Rust/arx_m_can
    maturin develop --release
    python examples/10_verify_poses.py
    python examples/10_verify_poses.py --speed 60 --motion-tolerance 0.02
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from common import (
    DEFAULT_POSES,
    DEFAULT_SCALES,
    at_target,
    ensure_native_path,
    fmt_joints,
    hold_stats,
    joint_errors,
    load_gravity_scales,
    load_poses,
)

ensure_native_path()
import _native  # noqa: E402


def worst_joint(q_meas: list[float], q_des: list[float]) -> tuple[int, float]:
    errs = [abs(q_meas[j] - q_des[j]) for j in range(6)]
    j = max(range(6), key=lambda i: errs[i])
    return j, errs[j]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--poses", type=Path, default=DEFAULT_POSES)
    p.add_argument("--scales-file", type=Path, default=DEFAULT_SCALES)
    p.add_argument(
        "--speed",
        type=float,
        default=80.0,
        help="Trajectory speed [0–400] mapped to joint vmax",
    )
    p.add_argument("--settle-sec", type=float, default=1.5, help="Extra hold time after motion")
    p.add_argument("--samples", type=int, default=25)
    p.add_argument(
        "--joint-tolerance",
        type=float,
        default=0.01,
        help="Hold RMSE gate and strict at-target check (rad)",
    )
    p.add_argument(
        "--motion-tolerance",
        type=float,
        default=0.02,
        help="Wait-for-completion tolerance during Ruckig move (rad); looser than hold",
    )
    p.add_argument("--converge-timeout", type=float, default=30.0)
    args = p.parse_args()

    poses = load_poses(args.poses)
    scales = load_gravity_scales(args.scales_file)
    robot = _native.Robot(
        arm_control_mode="mit",
        use_gravity_ff=True,
        gravity_joint_scales=scales,
        auto_enable=True,
        wait_tolerance_rad=args.motion_tolerance,
        wait_timeout_sec=args.converge_timeout,
    )

    print(f"{len(poses)} poses from {args.poses.name}")
    print("Mode: MIT + Ruckig trajectory + gravity FF")
    kp, kd = robot.get_arm_mit_gains()
    print(f"Kp={[round(x, 1) for x in kp[:6]]}  Kd={[round(x, 1) for x in kd[:6]]}")
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
        f"measured MIT refresh={refresh_hz:.1f} Hz  "
        f"send_errors_during_measurement={refresh_errors}"
    )
    print(
        f"motion_tol={args.motion_tolerance:.3f} rad  "
        f"hold_tol={args.joint_tolerance:.3f} rad  "
        f"converge_timeout={args.converge_timeout:.1f} s\n"
    )
    print(f"{'pose':>4}  {'ok':>3}  {'max|e|':>36}  {'rmse':>36}")
    print("-" * 86)

    worst_rmse = [0.0] * 6
    failed_moves = 0
    try:
        for i, q_des in enumerate(poses):
            converged = robot.set_robot_state(
                target_joints=q_des,
                speed=args.speed,
                wait_for_completion=True,
                joint_tolerance=args.motion_tolerance,
                converge_timeout=args.converge_timeout,
            )
            time.sleep(args.settle_sec)

            q_meas = robot.get_joint_positions()[:6]
            if not converged and not at_target(q_meas, q_des, args.motion_tolerance):
                failed_moves += 1
                j_worst, err_worst = worst_joint(q_meas, q_des)
                errs = joint_errors(q_meas, q_des)
                diagnostics = robot.get_motor_diagnostics(True)[:6]
                velocity = [float(d["velocity"]) for d in diagnostics]
                torque = [float(d["torque"]) for d in diagnostics]
                status = [
                    f"j{j + 1}:{d['status_name']}({d['status_code']:#x})"
                    for j, d in enumerate(diagnostics)
                ]
                print(
                    f"{i:4d}  {'NO':>3}  motion timeout — "
                    f"worst j{j_worst + 1} err={err_worst:.4f} rad"
                )
                print(f"      q_des={fmt_joints(q_des)}  q_meas={fmt_joints(q_meas)}")
                print(f"      per-joint err(rad)={fmt_joints(errs)}")
                print(f"      velocity={fmt_joints(velocity)}  torque={fmt_joints(torque)}")
                print("      status=" + " ".join(status))
                continue

            errs = []
            for _ in range(args.samples):
                q = robot.get_joint_positions()[:6]
                errs.append([q[j] - q_des[j] for j in range(6)])
                time.sleep(0.04)
            rmse, mx = hold_stats(errs)
            worst_rmse = [max(worst_rmse[j], rmse[j]) for j in range(6)]
            hold_ok = all(rmse[j] <= args.joint_tolerance for j in range(6))
            if converged and hold_ok:
                ok = "YES"
            elif at_target(q_meas, q_des, args.joint_tolerance):
                ok = " ~ "
            else:
                ok = "H~"
            print(
                f"{i:4d}  {ok:>3}  "
                f"{str([f'{x:.4f}' for x in mx]):>36}  "
                f"{str([f'{x:.4f}' for x in rmse]):>36}"
            )
    finally:
        robot.disconnect()

    print("-" * 86)
    print(
        "worst rmse per joint (rad): "
        + " ".join(f"j{j+1}={worst_rmse[j]:.4f}" for j in range(6))
    )
    if failed_moves:
        print(f"WARNING: {failed_moves} pose(s) did not finish motion within motion_tol")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
