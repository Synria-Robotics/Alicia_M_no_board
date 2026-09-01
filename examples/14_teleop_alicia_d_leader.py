#!/usr/bin/env python3
"""Alicia-D leader → ARX-M-CAN follower teleoperation.

Uses the Rust ``Teleoperation`` loop with Alicia-D → Alicia-M joint mapping
and MIT stream latching (500 Hz ``q_des`` walker, no per-tick Ruckig).

Examples:
  python examples/14_teleop_alicia_d_leader.py
  python examples/14_teleop_alicia_d_leader.py \\
      --leader-port /dev/ttyACM0 --follower-port /dev/ttyACM1 --speed 200
  python examples/14_teleop_alicia_d_leader.py --home
"""

from __future__ import annotations

import argparse

from common import DEFAULT_BAUD, DEFAULT_PORT
import alicia_d_sdk

from api.profiles import DEFAULT_PROFILE, PROFILE_NAMES
from api.synria_robot_api import Teleoperation, create_robot


def main(args: argparse.Namespace) -> None:
    leader = alicia_d_sdk.create_robot(port=args.leader_port)
    if not leader.is_connected():
        raise SystemExit(f"leader connect failed: {args.leader_port!r}")

    follower = create_robot(port=args.follower_port, baud=args.baud, robot=args.robot)
    try:
        if args.home:
            print("Moving follower to home...")
            follower.go_home(speed=args.home_speed)

        teleop = Teleoperation(
            follower,
            frequency_hz=args.frequency,
            follower_speed=args.speed,
            gripper_speed=args.gripper_speed,
            gripper_scale=args.gripper_scale,
            use_joint_mapping=not args.no_joint_mapping,
            max_joint_accel_rad_s2=args.max_joint_accel,
            max_joint_jerk_rad_s3=args.max_joint_jerk,
        )

        print(
            f"Ready: leader={args.leader_port!r} follower={args.follower_port!r} "
            f"freq={args.frequency:.0f}Hz speed={args.speed:.0f} "
            f"gripper_speed={args.gripper_speed:.0f}"
        )
        input("Press Enter to start teleoperation...")
        teleop.start(leader)
        try:
            input("Press Enter to stop teleoperation...\n")
        except KeyboardInterrupt:
            print()
        teleop.stop()
        teleop.disconnect_follower()
    finally:
        leader.disconnect()
        print("Disconnected")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Alicia-D leader → ARX-M-CAN follower teleoperation",
    )
    parser.add_argument("--leader-port", default=DEFAULT_PORT, help="Alicia-D leader serial port")
    parser.add_argument(
        "--follower-port",
        default="/dev/ttyACM2",
        help="ARX-M-CAN follower USB2CAN port",
    )
    parser.add_argument("--frequency", type=float, default=60.0, help="Teleop loop frequency (Hz)")
    parser.add_argument("--speed", type=float, default=80.0, help="Follower joint speed [0,400]")
    parser.add_argument(
        "--gripper-speed",
        type=float,
        default=200.0,
        help="Follower gripper speed [0,400]",
    )
    parser.add_argument("--gripper-scale", type=float, default=1.0, help="Leader → follower gripper scale")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Follower USB2CAN baudrate")
    parser.add_argument(
        "--robot",
        choices=PROFILE_NAMES,
        default=DEFAULT_PROFILE,
        help="ARX-M-CAN follower variant",
    )
    parser.add_argument("--home", action="store_true", help="Move follower to zero before teleop")
    parser.add_argument("--home-speed", type=float, default=40.0, help="Home motion speed [0,400]")
    parser.add_argument(
        "--no-joint-mapping",
        action="store_true",
        help="Disable Alicia-D → Alicia-M joint mapping",
    )
    parser.add_argument(
        "--max-joint-accel",
        type=float,
        default=None,
        help="Max joint acceleration for Ruckig trajectories (rad/s²)",
    )
    parser.add_argument(
        "--max-joint-jerk",
        type=float,
        default=None,
        help="Max joint jerk for Ruckig trajectories (rad/s³)",
    )
    args = parser.parse_args()
    main(args)
