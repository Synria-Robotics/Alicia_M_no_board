#!/usr/bin/env python3
"""Read joint positions (does not enable motors).

    python examples/01_read_state.py --port /dev/ttyACM2 --robot arx
"""

from __future__ import annotations

import argparse
import math
import sys

from common import add_robot_arguments, make_robot


def main(args: argparse.Namespace) -> None:
    print(f"robot={args.robot}", file=sys.stderr)
    robot = make_robot(args, auto_enable=False)
    try:
        values = [float(q) for q in robot.get_robot_state("joint")[:6]]
        if any(math.isnan(v) for v in values):
            print("WARNING: incomplete motor feedback (NaN joints)", file=sys.stderr)
        print(values)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read ARX-M-CAN joint state")
    add_robot_arguments(parser)
    main(parser.parse_args())
