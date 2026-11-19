#!/usr/bin/env python3
"""Compare low and high speed motion using the same trajectory controller.

This is a hardware test and requires the explicit ``--execute`` flag.
"""

from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

from common import add_robot_arguments, make_robot


def monitor(robot, rows: list[tuple[float, list[float]]], stop: threading.Event, period: float) -> None:
    while not stop.is_set():
        try:
            rows.append((time.perf_counter(), list(robot.get_joint_positions()[:6])))
        except Exception:
            pass
        stop.wait(period)


def metrics(rows: list[tuple[float, list[float]]]) -> dict[str, float]:
    velocities: list[float] = []
    accelerations: list[float] = []
    for (t0, q0), (t1, q1) in zip(rows, rows[1:]):
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
    parser.add_argument("--offset", type=float, default=0.15, help="joint 1 excursion [rad]")
    parser.add_argument("--low-speed", type=float, default=20.0, help="low speed [0-400]")
    parser.add_argument("--high-speed", type=float, default=250.0, help="high speed [0-400]")
    parser.add_argument("--sample-hz", type=float, default=25.0)
    parser.add_argument("--output", type=Path, default=Path("smooth_motion.csv"))
    args = parser.parse_args()

    if not args.execute:
        print("dry run: low/high speed test; re-run with --execute to move the robot")
        return 0
    if args.offset <= 0 or not 0 < args.sample_hz <= 100:
        parser.error("offset must be positive and sample-hz must be in (0,100]")

    robot = make_robot(args)
    rows: list[tuple[float, list[float]]] = []
    stop = threading.Event()
    worker = threading.Thread(target=monitor, args=(robot, rows, stop, 1.0 / args.sample_hz), daemon=True)
    q0 = robot.get_joint_positions()[:6]
    targets = [list(q0), list(q0), list(q0)]
    targets[0][0] += args.offset
    targets[1][0] -= args.offset
    summaries: list[dict[str, object]] = []
    worker.start()
    try:
        for name, speed, target in (("low", args.low_speed, targets[0]),
                                    ("high", args.high_speed, targets[1]),
                                    ("return", args.high_speed, targets[2])):
            before = robot._robot.get_control_loop_stats()
            started = time.perf_counter()
            converged = robot.move_joints(target, speed=speed, wait_for_completion=True,
                                          joint_tolerance=0.02, converge_timeout=30.0)
            elapsed = time.perf_counter() - started
            after = robot._robot.get_control_loop_stats()
            summaries.append({"phase": name, "speed": speed, "converged": bool(converged),
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
        writer.writerow(["time_sec", *[f"joint_{i + 1}_rad" for i in range(6)]])
        if rows:
            t0 = rows[0][0]
            writer.writerows([[t - t0, *q] for t, q in rows])

    report = {"motion": summaries, "sample_metrics": metrics(rows), "csv": str(args.output)}
    print(report)
    failed = any(not item["converged"] or item["send_errors"] or item["overruns"] for item in summaries)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
