#!/usr/bin/env python3
"""Write current motor positions as zero after explicit confirmation.

    python examples/08_zero_current_position.py
    python examples/08_zero_current_position.py --port /dev/ttyACM2 --robot arx --yes
"""

from __future__ import annotations

import argparse

from common import add_robot_arguments, make_robot


def main(args: argparse.Namespace) -> None:
    robot = make_robot(args, auto_enable=False)
    try:
        state = robot.get_arm_state(True)
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

        robot.set_zero()
        print("zero position written")
    finally:
        robot.disconnect()


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
