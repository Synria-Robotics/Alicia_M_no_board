#!/usr/bin/env python3
"""Scan Damiao motor IDs without enabling the arm.

    python examples/02_scan_ids.py
    python examples/02_scan_ids.py --port /dev/ttyACM2 --robot arx
"""

from __future__ import annotations

import argparse

from common import add_robot_arguments
from api.robot import scan_ids


def main(args: argparse.Namespace) -> None:
    ids = scan_ids(
        start_id=args.start_id,
        end_id=args.end_id,
        model=args.model,
        timeout_ms=args.timeout_ms,
        feedback_base=args.feedback_base,
    )
    print("found:", " ".join(f"0x{motor_id:02X}" for motor_id in ids) or "none")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Scan Damiao motor IDs without enabling the arm.")
    parser.add_argument("--start-id", type=int, default=1)
    parser.add_argument("--end-id", type=int, default=16)
    parser.add_argument("--model", default="4340P")
    parser.add_argument("--timeout-ms", type=int, default=30)
    parser.add_argument("--feedback-base", default="0x10")
    add_robot_arguments(parser)
    main(parser.parse_args())
