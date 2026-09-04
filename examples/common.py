"""Shared helpers for ARX-M-CAN examples."""

from __future__ import annotations

import math
import sys
from argparse import ArgumentParser, Namespace
from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from api.profiles import DEFAULT_PROFILE, PROFILE_NAMES, get_profile, load_gravity_scales

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1_000_000
# DEFAULT_POSES = Path("/home/ubuntu/Synria/geom_arx/src/calibration/config/poses_master.yaml")
DEFAULT_POSES = Path("/home/ubuntu/Synria/geom_arx/src/calibration/config/poses_slave.yaml")
NUM_ARM_JOINTS = 6


@dataclass(frozen=True)
class RobotSetup:
    robot: str
    scales: list[float]
    scales_path: Path
    dynamics_model_path: Path
    config_path: Path
    urdf_path: Path
    base_link: str = "base_link"
    end_link: str = "tool0"


def add_robot_arguments(parser: ArgumentParser, *, prefer_float: bool = False) -> None:
    parser.add_argument("--port", default=DEFAULT_PORT, help="USB2CAN serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="USB2CAN serial baudrate")
    parser.add_argument(
        "--robot",
        choices=PROFILE_NAMES,
        default=DEFAULT_PROFILE,
        help="Robot variant (URDF, dynamics, scales, hardware YAML)",
    )
    parser.add_argument(
        "--float",
        dest="use_float_config",
        action="store_true",
        help="Use alicia_m_can_dm_float.yaml (kp=0)",
    )
    parser.add_argument(
        "--no-float",
        dest="use_float_config",
        action="store_false",
        help="Use production alicia_m_can_dm.yaml",
    )
    parser.set_defaults(use_float_config=prefer_float)
    parser.add_argument("--config-path", type=Path, default=None)
    parser.add_argument("--dynamics-model", type=Path, default=None)
    parser.add_argument("--scales-file", type=Path, default=None)


def resolve_robot_setup(args: Namespace, *, prefer_float: bool = False) -> RobotSetup:
    profile = get_profile(getattr(args, "robot", None) or DEFAULT_PROFILE)
    use_float = getattr(args, "use_float_config", False) or prefer_float
    config_path = getattr(args, "config_path", None) or (
        profile.dm_float_yaml if use_float else profile.dm_yaml
    )
    scales_path = getattr(args, "scales_file", None) or profile.scales_yaml
    return RobotSetup(
        robot=profile.name,
        scales=load_gravity_scales(scales_path),
        scales_path=scales_path,
        dynamics_model_path=getattr(args, "dynamics_model", None) or profile.dynamics_json,
        config_path=config_path,
        urdf_path=profile.urdf,
        base_link=profile.base_link,
        end_link=profile.end_link,
    )


def make_robot(args: Namespace, *, prefer_float: bool = False, **extra: object):
    from api.synria_robot_api import create_robot

    setup = resolve_robot_setup(args, prefer_float=prefer_float)
    return create_robot(
        port=args.port,
        baud=args.baud,
        robot=setup.robot,
        config_path=setup.config_path,
        gravity_model_path=setup.dynamics_model_path,
        gravity_joint_scales=setup.scales,
        urdf_path=setup.urdf_path,
        base_link=setup.base_link,
        end_link=setup.end_link,
        **extra,
    )


def make_arm(args: Namespace | None = None, *, enable_gripper: bool = False):
    from api.synria_robot_api import Arm

    setup = resolve_robot_setup(args) if args is not None else None
    return Arm(
        port=DEFAULT_PORT if args is None else args.port,
        baud=DEFAULT_BAUD if args is None else args.baud,
        enable_gripper=enable_gripper,
        config_path=None if setup is None else setup.config_path,
    )


def load_poses(path: Path = DEFAULT_POSES) -> list[list[float]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    ref = data["ref"]
    poses = [list(ref)]
    for offset in data.get("offsets") or []:
        if offset:
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
