#!/usr/bin/env python3
"""Move arm to target joint positions."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
import time
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from api.synria_robot_api import create_robot

from common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    target = [float(v) for v in args.joints.split(",")]
    robot = create_robot(port=args.port, baud=args.baud)
    try:
        robot.set_robot_state(
            target_joints=target,
            joint_format="rad",
            speed=args.speed,
            wait_for_completion=True,
        )
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
    add_connection_arguments(parser)
    main(parser.parse_args())
