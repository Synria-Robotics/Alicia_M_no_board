#!/usr/bin/env python3
"""Read joint positions from ARX-M-CAN arm (does not enable/disable or switch modes)."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from api.synria_robot_api import Arm

from common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    arm = Arm(port=args.port, baud=args.baud)
    arm.connect()
    try:
        state = arm.read_state(request_feedback=True)
        values = [float(q) for q in state["arm"]["positions"][:6]]
        if any(math.isnan(v) for v in values):
            print("WARNING: incomplete motor feedback (NaN joints)", file=sys.stderr)
        print(values)
    finally:
        arm.close(disable=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read ARX-M-CAN joint state")
    add_connection_arguments(parser)
    main(parser.parse_args())
