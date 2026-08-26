#!/usr/bin/env python3
"""Forward kinematics demo (RoboCore FK + live joint read).

    python examples/06_forward_kinematics.py
    python examples/06_forward_kinematics.py --robot follower
    python examples/06_forward_kinematics.py --robot px6d
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from api.synria_robot_api import create_robot

from common import add_connection_arguments, add_robot_arguments, resolve_robot_setup, synria_robot_kwargs


def main(args: argparse.Namespace) -> None:
    setup = resolve_robot_setup(args)
    robot = create_robot(
        port=args.port,
        baud=args.baud,
        auto_enable=False,
        **synria_robot_kwargs(setup),
    )
    try:
        if setup.robot:
            print(f"robot={setup.robot}")
        if setup.urdf_path:
            print(f"urdf={setup.urdf_path.name}")
        pose = robot.get_pose()
        print("position (m):", np.round(pose["position"], 6))
        print("euler_xyz (rad):", np.round(pose["euler_xyz"], 6))
        print("quaternion_xyzw:", np.round(pose["quaternion_xyzw"], 6))
        print("transform:\n", np.round(pose["transform"], 6))
    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    add_robot_arguments(parser)
    main(parser.parse_args())
