#!/usr/bin/env python3
"""Write current motor positions as zero after explicit confirmation.

    python examples/08_zero_current_position.py
    python examples/08_zero_current_position.py --port /dev/ttyACM2 --robot arx --yes
"""

from __future__ import annotations

import argparse

from common import add_robot_arguments, make_arm


def main(args: argparse.Namespace) -> None:
    arm = make_arm(args, enable_gripper=True)
    try:
        arm.connect()
        state = arm.read_state(request_feedback=True)
        arm_state = state["arm"]
        print("current arm position:")
        for name, position in zip(arm_state["names"], arm_state["positions"]):
            print(f"  {name}: {position:+.6f} rad")
        gripper = state.get("gripper")
        if gripper is not None:
            print(f"  {gripper['name']}: {gripper['position']:+.6f} rad")

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
        description="Write the current ARX-M-CAN motor positions as zero."
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Actually write zero positions. Without this flag, only print current state.",
    )
    add_robot_arguments(parser)
    main(parser.parse_args())
