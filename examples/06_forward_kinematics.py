#!/usr/bin/env python3
"""Forward kinematics demo (RoboCore FK + live joint read).

    python examples/06_forward_kinematics.py --robot arx --port /dev/ttyACM2
"""

from __future__ import annotations

import argparse

import numpy as np

from common import add_robot_arguments, make_robot


def main(args: argparse.Namespace) -> None:
    robot = make_robot(args, auto_enable=False)
    try:
        print(f"port={args.port} robot={args.robot}")
        pose = robot.get_pose()
        print("position (m):", np.round(pose["position"], 6))
        print("euler_xyz (rad):", np.round(pose["euler_xyz"], 6))
        print("quaternion_xyzw:", np.round(pose["quaternion_xyzw"], 6))
        print("transform:\n", np.round(pose["transform"], 6))
    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    add_robot_arguments(parser)
    main(parser.parse_args())
