#!/usr/bin/env python3
"""Open and close gripper (0=closed, 1000=open).

    python examples/04_move_gripper.py --port /dev/ttyACM2 --robot arx
"""

from __future__ import annotations

import argparse
import time

from common import add_robot_arguments, make_robot


def main(args: argparse.Namespace) -> None:
    robot = make_robot(args)
    try:
        robot.set_robot_state(gripper_value=1000.0, gripper_speed=args.speed, wait_for_completion=True)
        time.sleep(0.5)
        robot.set_robot_state(gripper_value=0.0, gripper_speed=args.speed, wait_for_completion=True)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ARX-M-CAN gripper demo")
    parser.add_argument("--speed", type=float, default=100.0)
    add_robot_arguments(parser)
    main(parser.parse_args())
