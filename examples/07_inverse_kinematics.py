#!/usr/bin/env python3
"""Inverse kinematics demo (RoboCore IK + MIT motion).

    python examples/07_inverse_kinematics.py --execute
    python examples/07_inverse_kinematics.py --robot follower --execute
    python examples/07_inverse_kinematics.py --robot px6d --execute --speed 60
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
        auto_enable=args.execute,
        **synria_robot_kwargs(setup),
    )
    try:
        if setup.robot:
            print(f"robot={setup.robot}")
        if setup.urdf_path:
            print(f"urdf={setup.urdf_path.name}")
        if setup.config_path:
            print(f"config={setup.config_path.name}")
        result = robot.set_pose(
            args.end_pose,
            method=args.method,
            pos_tol=args.pos_tol,
            ori_tol=args.ori_tol,
            max_iters=args.max_iters,
            num_initial_guesses=args.num_inits,
            initial_guess_strategy=args.init_strategy,
            initial_guess_scale=args.init_scale,
            random_seed=args.seed,
            speed=args.speed,
            execute=args.execute,
            force_execute=args.force_execute,
        )
        print(f"success={result.get('success')}  motion={result.get('motion_executed')}")
        if result.get("q") is not None:
            print("q (rad):", np.round(result["q"], 4))
            print("q (deg):", np.round(np.rad2deg(result["q"]), 2))
        if not result.get("success"):
            print("message:", result.get("message"))
    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--end-pose",
        type=float,
        nargs=7,
        default=[0.219497, -0.000911, 0.15941, 0.774915, -0.001942, 0.632059, 0.002071],
        help="Target [x y z qx qy qz qw]",
    )
    parser.add_argument("--method", default="dls", choices=["dls", "pinv", "transpose"])
    parser.add_argument("--max-iters", type=int, default=500)
    parser.add_argument("--pos-tol", type=float, default=1e-3)
    parser.add_argument("--ori-tol", type=float, default=1e-3)
    parser.add_argument("--num-inits", type=int, default=10)
    parser.add_argument(
        "--init-strategy",
        default="current",
        choices=["zero", "random", "sobol", "latin", "center", "uniform", "current"],
    )
    parser.add_argument("--init-scale", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--speed", type=float, default=60.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--force-execute", action="store_true")
    add_connection_arguments(parser)
    add_robot_arguments(parser)
    main(parser.parse_args())
