#!/usr/bin/env python3
"""SparkVis drag-to-real-robot bridge for ARX-M-CAN.

Subscribes to SparkVis teleop hub ``joint_update`` messages and streams joint
targets to an ARX-M-CAN follower via the Rust ``SparkVisBridge``.

Prerequisites:
  1. SparkVis backend running (``http://127.0.0.1:8000``, teleop at ``/ws/teleop``)
  2. USB2CAN present (unless ``--dry-run``)

Examples:
  python examples/15_sparkvis_control.py --dry-run
  python examples/15_sparkvis_control.py --port /dev/ttyACM0
  python examples/15_sparkvis_control.py --enable --speed 100
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from api.synria_robot_api import SparkVisBridge, create_robot

from common import add_connection_arguments


def main(args: argparse.Namespace) -> int:
    bridge = SparkVisBridge(
        ws_url=args.ws_url,
        http_base=args.http_base,
        speed=args.speed,
        gripper_speed=args.gripper_speed,
        command_hz=args.hz,
        watchdog_sec=args.watchdog,
        control_gripper=args.gripper,
        dry_run=args.dry_run,
        sync_initial=not args.no_sync,
    )
    try:
        if not args.dry_run:
            robot = create_robot(port=args.port, baud=args.baud)
            bridge.connect_robot(robot)
        bridge.start()
        if args.enable or args.dry_run:
            bridge.arm()
        else:
            print("[sparkvis_control] waiting to arm — robot will NOT move until enabled")
            input("Press Enter to arm...")
            bridge.arm()
        try:
            input("Press Enter to stop...\n")
        except KeyboardInterrupt:
            print()
    finally:
        bridge.shutdown()
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Bridge SparkVis joint_update to ARX-M-CAN real robot."
    )
    parser.add_argument(
        "--ws-url",
        default="ws://127.0.0.1:8000/ws/teleop",
        help="SparkVis teleop WebSocket URL",
    )
    parser.add_argument(
        "--http-base",
        default="http://127.0.0.1:8000",
        help="SparkVis HTTP base for joint_state sync",
    )
    parser.add_argument("--speed", type=float, default=100.0, help="Joint speed [0,400]")
    parser.add_argument("--gripper-speed", type=float, default=100.0, help="Gripper speed [0,400]")
    parser.add_argument("--hz", type=float, default=50.0, help="Max outbound command rate (Hz)")
    parser.add_argument(
        "--watchdog",
        type=float,
        default=0.35,
        help="Seconds without joint_update before pausing new commands",
    )
    parser.add_argument("--enable", action="store_true", help="Arm immediately")
    parser.add_argument("--dry-run", action="store_true", help="Print targets only; no CAN")
    parser.add_argument("--no-sync", action="store_true", help="Skip initial joint_state POST")
    parser.add_argument("--gripper", action="store_true", help="Stream gripper from SparkVis")
    add_connection_arguments(parser)
    raise SystemExit(main(parser.parse_args()))
