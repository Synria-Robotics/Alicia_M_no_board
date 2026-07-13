#!/usr/bin/env python3
"""Example 03: write current motor positions as zero after explicit confirmation."""
from __future__ import annotations

import argparse

try:
    from .common import add_connection_arguments, make_arm
except ImportError:
    from common import add_connection_arguments, make_arm


def main(args: argparse.Namespace) -> None:
    arm = make_arm(args)
    try:
        arm.connect()
        state = arm.read_state(request_feedback=True)
        print("current arm position:")
        for name, position in zip(state.arm.names, state.arm.positions):
            print(f"  {name}: {position:+.6f} rad")
        if state.gripper is not None:
            print(f"  {state.gripper.name}: {state.gripper.position:+.6f} rad")

        if not args.yes:
            print("")
            print("zero was NOT written.")
            print("Place the arm at the intended zero pose, then rerun with --yes.")
            return

        arm.set_zero()
        print("zero position written")
    finally:
        arm.close(disable=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Write the current ARX-D-CAN motor positions as zero."
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Actually write zero positions. Without this flag, only print current state.",
    )
    add_connection_arguments(parser)
    main(parser.parse_args())
