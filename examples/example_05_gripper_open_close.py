#!/usr/bin/env python3
"""Example 05: open and close the gripper (user scale: 0=closed, 1000=open)."""
from __future__ import annotations

import argparse
import time

try:
    from .common import add_connection_arguments, make_arm
except ImportError:
    from common import add_connection_arguments, make_arm


def send_gripper_for_seconds(arm, value: float, *, seconds: float, hz: float = 100.0) -> None:
    period = 1.0 / max(1.0, hz)
    deadline = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < deadline:
        arm.set_gripper(value)
        time.sleep(period)


def main(args: argparse.Namespace) -> None:
    arm = make_arm(args, enable_gripper=True)
    try:
        arm.connect()
        arm.configure()
        arm.enable()
        print("opening gripper")
        send_gripper_for_seconds(arm, args.open_value, seconds=args.open_seconds, hz=args.hz)
        print("closing gripper")
        send_gripper_for_seconds(arm, args.closed_value, seconds=args.close_seconds, hz=args.hz)
        print("gripper test finished; arm remains enabled until this process exits")
    finally:
        arm.close(disable=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Open and close the ARX-D-CAN gripper.")
    parser.add_argument("--open-value", type=float, default=1000.0)
    parser.add_argument("--closed-value", type=float, default=0.0)
    parser.add_argument("--open-seconds", type=float, default=2.0)
    parser.add_argument("--close-seconds", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=100.0)
    add_connection_arguments(parser)
    main(parser.parse_args())
