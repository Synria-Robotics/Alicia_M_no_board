#!/usr/bin/env python3
"""Example 01: scan Damiao motor IDs without enabling the arm."""
from __future__ import annotations

import argparse

try:
    from .common import add_connection_arguments, make_arm
except ImportError:
    from common import add_connection_arguments, make_arm


def main(args: argparse.Namespace) -> None:
    arm = make_arm(args)
    ids = arm.scan_ids(
        start_id=args.start_id,
        end_id=args.end_id,
        model=args.model,
        timeout_ms=args.timeout_ms,
    )
    print("found:", " ".join(f"0x{motor_id:02X}" for motor_id in ids) or "none")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Scan Damiao motor IDs without enabling the arm.")
    parser.add_argument("--start-id", type=int, default=1)
    parser.add_argument("--end-id", type=int, default=16)
    parser.add_argument("--model", default="4340P")
    parser.add_argument("--timeout-ms", type=int, default=30)
    add_connection_arguments(parser)
    main(parser.parse_args())
