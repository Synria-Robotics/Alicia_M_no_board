#!/usr/bin/env python3
"""Run the guarded grasp, 10 cm lift, and 10 minute hold test.

This is a hardware test and requires the explicit ``--execute`` flag.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from common import add_robot_arguments, make_robot


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_robot_arguments(parser)
    parser.add_argument("--execute", action="store_true", help="enable motion")
    parser.add_argument("--height", type=float, default=0.10, help="lift height [m]")
    parser.add_argument("--hold-min", type=float, default=10.0, help="hold time [min]")
    parser.add_argument("--output", type=Path, default=None, help="optional JSON result path")
    args = parser.parse_args()

    hold_time = args.hold_min * 60.0
    if not args.execute:
        print(f"dry run: grasp, lift {args.height:.3f} m, hold {args.hold_min:.1f} min, release")
        print("re-run with --execute to move the robot")
        return 0

    robot = make_robot(args)
    started = time.time()
    try:
        result = robot.grasp_lift_test(height=args.height, hold_time=hold_time)
        result.update({"height_m": args.height, "hold_time_sec": hold_time,
                       "elapsed_sec": time.time() - started})
        print(json.dumps(result, indent=2, ensure_ascii=False))
        if args.output:
            args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        return 0 if result.get("ok") else 1
    except KeyboardInterrupt:
        print("\ninterrupted: disconnecting and releasing", file=sys.stderr)
        robot.gripper.release()
        return 130
    finally:
        if 'robot' in locals():
            robot.disconnect()


if __name__ == "__main__":
    sys.exit(main())
