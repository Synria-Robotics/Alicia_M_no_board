#!/usr/bin/env python3
"""Return arm to zero configuration."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from api.synria_robot_api import create_robot

from common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    robot = create_robot(port=args.port, baud=args.baud)
    try:
        robot.go_home(speed=args.speed)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ARX-M-CAN go home")
    parser.add_argument("--speed", type=float, default=40.0)
    add_connection_arguments(parser)
    main(parser.parse_args())
