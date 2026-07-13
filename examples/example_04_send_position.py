#!/usr/bin/env python3
"""Example 04: send one requested joint position target."""
from __future__ import annotations

import argparse
import sys

try:
    from .common import (
        ZERO_ARM_POSITION,
        add_connection_arguments,
        interpolate_joint_positions,
        make_arm,
        parse_joint_positions,
    )
except ImportError:
    from common import (
        ZERO_ARM_POSITION,
        add_connection_arguments,
        interpolate_joint_positions,
        make_arm,
        parse_joint_positions,
    )


def send_target(arm, target: tuple[float, ...], *, move_seconds: float = 1.0) -> None:
    arm.move_joint_positions(target, move_seconds=move_seconds)


def stream_stdin_targets(arm, *, move_seconds: float) -> None:
    print("reading targets from stdin; one line = six comma-separated radians")
    for line in sys.stdin:
        text = line.strip()
        if not text or text.startswith("#"):
            continue
        target = parse_joint_positions(text)
        arm.move_joint_positions(target, move_seconds=move_seconds)
        print("sent:", " ".join(f"{value:+.6f}" for value in target), flush=True)


def main(args: argparse.Namespace) -> None:
    if not args.positions and not args.stdin:
        raise SystemExit("provide positions or use --stdin")

    target = parse_joint_positions(args.positions) if args.positions else None
    arm = make_arm(args)
    try:
        arm.connect()
        arm.configure()
        arm.enable()
        if target is not None:
            send_target(arm, target, move_seconds=args.move_seconds)
            print("sent:", " ".join(f"{value:+.6f}" for value in target))

        if args.stdin:
            stream_stdin_targets(arm, move_seconds=args.move_seconds)

        if args.return_zero:
            state = arm.read_state(request_feedback=True)
            interpolate_joint_positions(
                arm,
                state.arm.positions,
                ZERO_ARM_POSITION,
                seconds=args.return_seconds,
            )
        print("send-position example finished; motors remain enabled after exit")
    finally:
        arm.close(disable=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send a requested ARX-D-CAN joint position.")
    parser.add_argument(
        "positions",
        nargs="?",
        help="Six comma-separated joint positions in radians.",
    )
    parser.add_argument("--move-seconds", type=float, default=1.0, help="Interpolate from current state over this duration")
    parser.add_argument("--stdin", action="store_true", help="Read target frames from stdin, one line per target")
    parser.add_argument("--return-zero", action="store_true", help="Slowly return to zero before exit")
    parser.add_argument("--return-seconds", type=float, default=6.0)
    add_connection_arguments(parser)
    main(parser.parse_args())
