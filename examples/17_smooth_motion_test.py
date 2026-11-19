#!/usr/bin/env python3
"""Compare low and high speed motion using the same trajectory controller.

This is a hardware test and requires the explicit ``--execute`` flag.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import threading
import time
from pathlib import Path

from common import add_robot_arguments, make_robot


def monitor(robot, rows: list[tuple[float, str, list[float]]], phase: list[str], stop: threading.Event, period: float) -> None:
    while not stop.is_set():
        try:
            rows.append((time.perf_counter(), phase[0], list(robot.get_joint_positions()[:6])))
        except Exception:
            pass
        stop.wait(period)


def metrics(rows: list[tuple[float, str, list[float]]]) -> dict[str, float]:
    velocities: list[float] = []
    accelerations: list[float] = []
    for (t0, _, q0), (t1, _, q1) in zip(rows, rows[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        v = max(abs(b - a) / dt for a, b in zip(q0, q1))
        velocities.append(v)
    for v0, v1, pair in zip(velocities, velocities[1:], zip(rows, rows[1:])):
        dt = pair[1][0] - pair[0][0]
        if dt > 0:
            accelerations.append(abs(v1 - v0) / dt)
    return {
        "samples": float(len(rows)),
        "duration_sec": rows[-1][0] - rows[0][0] if len(rows) > 1 else 0.0,
        "max_velocity_rad_s": max(velocities, default=0.0),
        "max_acceleration_rad_s2": max(accelerations, default=0.0),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_robot_arguments(parser)
    parser.add_argument("--execute", action="store_true", help="enable motion")
    parser.add_argument("--offset", type=float, default=0.15, help="multi-joint excursion scale [rad]")
    parser.add_argument("--lift-height", type=float, default=0.10, help="Cartesian lift [m]")
    parser.add_argument("--low-speed", type=float, default=20.0, help="low speed [0-400]")
    parser.add_argument("--high-speed", type=float, default=250.0, help="high speed [0-400]")
    parser.add_argument("--speed-groups", default=None,
                        help="semicolon-separated low,high pairs, e.g. 20,100;40,200;80,300")
    parser.add_argument("--repetitions", type=int, default=3, help="number of complete motion repetitions")
    parser.add_argument("--sample-hz", type=float, default=25.0)
    parser.add_argument("--output", type=Path, default=Path("smooth_motion.csv"))
    parser.add_argument("--report", type=Path, default=Path("smooth_motion.json"))
    parser.add_argument("--max-accel", type=float, default=None, help="joint acceleration limit [rad/s^2]")
    parser.add_argument("--max-jerk", type=float, default=None, help="joint jerk limit [rad/s^3]")
    args = parser.parse_args()

    if not args.execute:
        print("dry run: low/high speed test; re-run with --execute to move the robot")
        return 0
    if args.offset <= 0 or args.lift_height <= 0 or not 0 < args.sample_hz <= 100 or args.repetitions < 1:
        parser.error("offset/lift-height must be positive, repetitions >= 1, and sample-hz must be in (0,100]")
    if args.speed_groups:
        try:
            speed_groups = [tuple(float(v.strip()) for v in pair.split(","))
                            for pair in args.speed_groups.split(";")]
        except ValueError as exc:
            parser.error(f"invalid --speed-groups: {exc}")
        if any(len(pair) != 2 or not all(0 < v <= 400 for v in pair) for pair in speed_groups):
            parser.error("each speed group must be low,high with values in (0,400]")
    else:
        speed_groups = [(args.low_speed, args.high_speed)]

    robot = make_robot(args)
    rows: list[tuple[float, str, list[float]]] = []
    phase = ["startup"]
    stop = threading.Event()
    worker = threading.Thread(target=monitor, args=(robot, rows, phase, stop, 1.0 / args.sample_hz), daemon=True)
    summaries: list[dict[str, object]] = []
    initial_joints: list[list[float]] = []
    lift_joints: list[list[float]] = []
    pattern = [1.0, -0.7, 0.5, -0.4, 0.3, -0.2]
    worker.start()
    try:
        for repetition in range(1, args.repetitions + 1):
            q0 = robot.get_joint_positions()[:6]
            initial_joints.append(q0)
            base_pose = robot.get_pose(q0)
            position = list(base_pose["position"])
            quaternion = list(base_pose["quaternion_xyzw"])
            lift_pose = [position[0], position[1], position[2] + args.lift_height, *quaternion]
            phase[0] = f"rep_{repetition}/lift_10cm"
            lift = robot.set_pose(
                lift_pose, speed=speed_groups[0][0], wait_for_completion=True,
                pos_tol=0.005, ori_tol=0.08, max_iters=400,
                max_joint_accel_rad_s2=args.max_accel,
                max_joint_jerk_rad_s3=args.max_jerk,
            )
            if not lift.get("motion_executed") or not lift.get("motion_converged"):
                print({"phase": phase[0], "result": lift})
                return 1
            q_lift = robot.get_joint_positions()[:6]
            lift_joints.append(q_lift)
            for group_index, (low_speed, high_speed) in enumerate(speed_groups, start=1):
                target_a = [q + args.offset * scale for q, scale in zip(q_lift, pattern)]
                target_b = [q - args.offset * scale for q, scale in zip(q_lift, pattern)]
                moves = ((f"group_{group_index}/low_multi_joint", low_speed, target_a),
                         (f"group_{group_index}/high_multi_joint", high_speed, target_b),
                         (f"group_{group_index}/return_lifted", high_speed, q_lift),
                         (f"group_{group_index}/home", low_speed, [0.0] * 6))
                for name, speed, target in moves:
                    phase[0] = f"rep_{repetition}/{name}"
                    before = robot._robot.get_control_loop_stats()
                    started = time.perf_counter()
                    converged = robot.move_joints(target, speed=speed, wait_for_completion=True,
                                                  joint_tolerance=0.02, converge_timeout=30.0,
                                                  max_joint_accel_rad_s2=args.max_accel,
                                                  max_joint_jerk_rad_s3=args.max_jerk)
                    elapsed = time.perf_counter() - started
                    after = robot._robot.get_control_loop_stats()
                    summaries.append({"repetition": repetition, "phase": name, "speed": speed,
                              "converged": bool(converged),
                              "elapsed_sec": elapsed,
                              "sent_commands": after["sent_commands"] - before["sent_commands"],
                              "send_errors": after["send_errors"] - before["send_errors"],
                              "overruns": after["overruns"] - before["overruns"],
                              "missed_deadlines": after["missed_deadlines"] - before["missed_deadlines"]})
                    time.sleep(0.5)
    finally:
        stop.set()
        worker.join(timeout=2.0)
        robot.disconnect()

    with args.output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["time_sec", "phase", *[f"joint_{i + 1}_rad" for i in range(6)]])
        if rows:
            t0 = rows[0][0]
            writer.writerows([[t - t0, phase_name, *q] for t, phase_name, q in rows])

    report = {
        "motion": summaries,
        "lift_height_m": args.lift_height,
        "multi_joint_pattern": pattern,
        "sample_metrics": metrics(rows),
        "speed_groups": speed_groups,
        "repetitions": args.repetitions,
        "initial_joints_rad": initial_joints,
        "lift_joints_rad": lift_joints,
        "home_joints_rad": [0.0] * 6,
        "max_joint_accel_rad_s2": args.max_accel,
        "max_joint_jerk_rad_s3": args.max_jerk,
        "csv": str(args.output),
        "report": str(args.report),
    }
    args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(report)
    failed = any(not item["converged"] or item["send_errors"] or item["overruns"] for item in summaries)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
