#!/usr/bin/env python3
"""Open and close gripper (0=closed, 1000=open)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from api.synria_robot_api import create_robot

from common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    robot = create_robot(port=args.port, baud=args.baud)
    try:
        robot.set_gripper_target(command="open", speed=args.speed)
        time.sleep(0.5)
        robot.set_gripper_target(command="close", speed=args.speed)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ARX-M-CAN gripper demo")
    parser.add_argument("--speed", type=float, default=100.0)
    add_connection_arguments(parser)
    main(parser.parse_args())
