#!/usr/bin/env python3
"""Move arm to target joint positions.

    python examples/03_move_joint.py --port /dev/ttyACM2 --robot arx --speed 60
"""

from __future__ import annotations

import argparse
import time

from common import add_robot_arguments, make_robot


def main(args: argparse.Namespace) -> None:
    target = [float(v) for v in args.joints.split(",")]
    robot = make_robot(args)
    try:
        robot.set_robot_state(target_joints=target, speed=args.speed, wait_for_completion=True)
        time.sleep(1)
        print([float(q) for q in robot.get_robot_state("joint")[:6]])
    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Move ARX-M-CAN joints")
    parser.add_argument("--speed", type=float, default=60.0, help="Joint speed [0,400]")
    parser.add_argument(
        "joints",
        nargs="?",
        default="0,-0.2,0.0,0,0,0",
        help="Comma-separated joint targets (rad)",
    )
    add_robot_arguments(parser)
    main(parser.parse_args())
