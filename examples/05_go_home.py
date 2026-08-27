#!/usr/bin/env python3
"""Return arm to zero configuration.

    python examples/05_go_home.py --port /dev/ttyACM2 --robot arx --speed 40
"""

from __future__ import annotations

import argparse

from common import add_robot_arguments, make_robot


def main(args: argparse.Namespace) -> None:
    robot = make_robot(args)
    try:
        robot.go_home(speed=args.speed)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ARX-M-CAN go home")
    parser.add_argument("--speed", type=float, default=40.0)
    add_robot_arguments(parser)
    main(parser.parse_args())
