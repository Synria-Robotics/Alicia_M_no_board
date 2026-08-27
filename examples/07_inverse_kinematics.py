#!/usr/bin/env python3
"""Inverse kinematics demo (RoboCore IK + MIT motion).

    python examples/07_inverse_kinematics.py --robot arx --port /dev/ttyACM2 --execute
"""

from __future__ import annotations

import argparse

import numpy as np

from common import add_robot_arguments, make_robot


def main(args: argparse.Namespace) -> None:
    robot = make_robot(args, auto_enable=args.execute)
    try:
        print(f"port={args.port} robot={args.robot}")
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
            joint_tolerance=args.joint_tolerance,
            converge_timeout=args.converge_timeout,
        )
        print(
            f"success={result.get('success')}  motion={result.get('motion_executed')}  "
            f"converged={result.get('motion_converged')}"
        )
        if result.get("q") is not None:
            q = np.asarray(result["q"])
            print("q (rad):", np.round(q, 4))
            print("q (deg):", np.round(np.rad2deg(q), 2))
            if args.execute:
                import time
                time.sleep(0.5)
                measured = np.asarray(robot.get_joint_positions()[:6])
                error = measured - q
                print("measured (rad):", np.round(measured, 4))
                print("error (deg):", np.round(np.rad2deg(error), 3))
                print(f"max error: {np.max(np.abs(np.rad2deg(error))):.3f} deg")
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
        # default=[-0.096482,0.129968, 0.200093, -0.613277, 0.356364, 0.657447, 0.254283],
        default=[0.255244, -0.021179, 0.085898, -0.025246, 0.988857, -0.055947, 0.135626],
        # default=[0.175549, -0.001191, 0.205638, 0.001149, 0.705012, -0.005067, 0.709176],
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
    parser.add_argument("--joint-tolerance", type=float, default=0.005)
    parser.add_argument("--converge-timeout", type=float, default=6.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--force-execute", action="store_true")
    add_robot_arguments(parser)
    main(parser.parse_args())
