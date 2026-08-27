#!/usr/bin/env python3
"""Read joint positions (does not enable motors).

    python examples/01_read_state.py --port /dev/ttyACM2 --robot arx
"""

from __future__ import annotations

import argparse
import math
import sys

from common import add_robot_arguments, make_arm


def main(args: argparse.Namespace) -> None:
    print(f"robot={args.robot}", file=sys.stderr)
    arm = make_arm(args)
    arm.connect()
    try:
        values = [float(q) for q in arm.read_state(request_feedback=True)["arm"]["positions"][:6]]
        if any(math.isnan(v) for v in values):
            print("WARNING: incomplete motor feedback (NaN joints)", file=sys.stderr)
        print(values)
    finally:
        arm.close(disable=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read ARX-M-CAN joint state")
    add_robot_arguments(parser)
    main(parser.parse_args())
