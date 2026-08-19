"""Shared helpers for ARX-M-CAN examples.

Demo order (basic → advanced):
  01 read_state  02 scan_ids  03 move_joint  04 move_gripper  05 go_home
  06 forward_kinematics  07 inverse_kinematics  08 zero_current_position
  09 mit_gravity_hold  10 verify_poses  11 diagnose_mit_joint
  12 gravity_calibrate  13 mit_gain_autotune  14 teleop  15 sparkvis
"""

from __future__ import annotations

import math
import sys
from argparse import ArgumentParser, Namespace
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1_000_000
DEFAULT_SCALES = ROOT / "arx_m_can" / "config" / "gravity_joint_scales.yaml"
DEFAULT_POSES = Path(
    "/home/ubuntu/Synria/geom_arx/src/calibration/config/poses_slave.yaml"
)
NUM_ARM_JOINTS = 6


def add_connection_arguments(parser: ArgumentParser) -> None:
    parser.add_argument("--port", default=DEFAULT_PORT, help="USB2CAN serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="USB2CAN serial baudrate")


def make_arm(args: Namespace | None = None, *, enable_gripper: bool = False):
    ensure_repo_path()
    from api.synria_robot_api import Arm

    return Arm(
        port=DEFAULT_PORT if args is None else args.port,
        baud=DEFAULT_BAUD if args is None else args.baud,
        enable_gripper=enable_gripper,
    )


def ensure_repo_path() -> None:
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))


def ensure_native_path() -> None:
    ensure_repo_path()


def load_gravity_scales(path: Path = DEFAULT_SCALES) -> list[float]:
    scales: list[float] = []
    in_block = False
    for line in path.read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if text.startswith("joint_scales:"):
            in_block = True
            continue
        if in_block and text.startswith("- "):
            scales.append(float(text[2:].split("#", 1)[0]))
        elif in_block and text and not text.startswith("#"):
            break
    if len(scales) != NUM_ARM_JOINTS:
        raise SystemExit(f"Expected {NUM_ARM_JOINTS} gravity scales in {path}, got {len(scales)}")
    return scales


def load_poses(path: Path = DEFAULT_POSES) -> list[list[float]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    ref = data["ref"]
    poses = [list(ref)]
    for offset in data.get("offsets") or []:
        if not offset:
            continue
        poses.append([r + o for r, o in zip(ref, offset)])
    return poses


def fmt_joints(values: list[float]) -> str:
    return "[" + ", ".join(f"{v:.4f}" for v in values) + "]"


def joint_errors(q_meas: list[float], q_des: list[float]) -> list[float]:
    return [q_meas[j] - q_des[j] for j in range(NUM_ARM_JOINTS)]


def at_target(q_meas: list[float], q_des: list[float], tol: float) -> bool:
    return all(abs(q_meas[j] - q_des[j]) <= tol for j in range(NUM_ARM_JOINTS))


def hold_stats(errs: list[list[float]]) -> tuple[list[float], list[float]]:
    n = len(errs)
    rmse, mx = [], []
    for j in range(NUM_ARM_JOINTS):
        col = [e[j] for e in errs]
        rmse.append(math.sqrt(sum(x * x for x in col) / n))
        mx.append(max(abs(x) for x in col))
    return rmse, mx
