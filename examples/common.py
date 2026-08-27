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
from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1_000_000
DEFAULT_SCALES = ROOT / "arx_m_can" / "config" / "gravity_joint_scales.yaml"
DEFAULT_POSES = Path(
    # "/home/ubuntu/Synria/geom_arx/src/calibration/config/poses_master.yaml"
    "/home/ubuntu/Synria/geom_arx/src/calibration/config/poses_slave.yaml"
)
NUM_ARM_JOINTS = 6
ROBOT_CHOICES = ("px6d", "arx", "follower")


@dataclass(frozen=True)
class RobotSetup:
    robot: str | None
    scales: list[float]
    scales_path: Path
    dynamics_model_path: Path | None
    config_path: Path | None
    urdf_path: Path | None
    base_link: str = "base_link"
    end_link: str = "tool0"


def add_robot_arguments(parser: ArgumentParser, *, prefer_float: bool = False) -> None:
    """Add --robot / --float / overrides for kinematics, gravity, and hardware YAML."""
    parser.add_argument(
        "--robot",
        choices=ROBOT_CHOICES,
        default=None,
        help="Robot variant (URDF, dynamics model, scales, hardware YAML)",
    )
    parser.add_argument(
        "--float",
        dest="use_float_config",
        action="store_true",
        help="Use arx_m_can_dm_float.yaml (kp=0) from the selected robot",
    )
    parser.add_argument(
        "--no-float",
        dest="use_float_config",
        action="store_false",
        help="Use production arx_m_can_dm.yaml from the robot",
    )
    parser.set_defaults(use_float_config=prefer_float)
    parser.add_argument(
        "--config-path",
        type=Path,
        default=None,
        help="Hardware YAML override (absolute or relative to arx_m_can/config/)",
    )
    parser.add_argument(
        "--dynamics-model",
        type=Path,
        default=None,
        help="Gravity dynamics JSON override",
    )
    parser.add_argument(
        "--scales-file",
        type=Path,
        default=None,
        help="Gravity joint scales YAML override",
    )


def resolve_robot_setup(args: Namespace, *, prefer_float: bool = False) -> RobotSetup:
    """Resolve URDF, scales, dynamics JSON, and hardware YAML from CLI args + --robot."""
    profiles = _import_profiles()
    get_robot = profiles.get_robot

    robot_name = getattr(args, "robot", None)
    use_float = getattr(args, "use_float_config", False) or prefer_float
    scales_override = getattr(args, "scales_file", None)
    dynamics_override = getattr(args, "dynamics_model", None)
    config_override = getattr(args, "config_path", None)

    if robot_name:
        variant = get_robot(robot_name)
        scales_path = scales_override or variant.scales_yaml
        dynamics_path = dynamics_override or variant.dynamics_json
        urdf_path = variant.urdf
        base_link = variant.base_link
        end_link = variant.end_link
        if config_override is not None:
            config_path = config_override
        elif use_float:
            config_path = variant.dm_float_yaml
        else:
            config_path = variant.dm_yaml
    else:
        scales_path = scales_override or DEFAULT_SCALES
        dynamics_path = dynamics_override
        urdf_path = None
        base_link = "base_link"
        end_link = "tool0"
        if config_override is not None:
            config_path = config_override
        elif use_float:
            raise SystemExit("--float requires --robot or --config-path")
        else:
            config_path = None

    scales = load_gravity_scales(scales_path)
    return RobotSetup(
        robot=robot_name,
        scales=scales,
        scales_path=scales_path,
        dynamics_model_path=dynamics_path,
        config_path=config_path,
        urdf_path=urdf_path,
        base_link=base_link,
        end_link=end_link,
    )


def robot_gravity_kwargs(setup: RobotSetup, **extra: object) -> dict[str, object]:
    """Build ``_native.Robot`` keyword args for gravity-compensated MIT control."""
    kwargs: dict[str, object] = {
        "arm_control_mode": "mit",
        "use_gravity_ff": True,
        "gravity_joint_scales": setup.scales,
        "auto_enable": True,
    }
    if setup.dynamics_model_path is not None:
        kwargs["gravity_model_path"] = str(setup.dynamics_model_path)
    if setup.config_path is not None:
        kwargs["config_path"] = str(setup.config_path)
    kwargs.update(extra)
    return kwargs


def synria_robot_kwargs(setup: RobotSetup, **extra: object) -> dict[str, object]:
    """Build ``create_robot`` keyword args from a resolved robot setup."""
    kwargs: dict[str, object] = {
        "base_link": setup.base_link,
        "end_link": setup.end_link,
    }
    if setup.urdf_path is not None:
        kwargs["urdf_path"] = setup.urdf_path
    if setup.config_path is not None:
        kwargs["config_path"] = str(setup.config_path)
    if setup.dynamics_model_path is not None:
        kwargs["gravity_model_path"] = str(setup.dynamics_model_path)
    if setup.scales:
        kwargs["gravity_joint_scales"] = setup.scales
    kwargs.update(extra)
    return kwargs


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


def _import_profiles():
    """Load profiles module without importing api.synria_robot_api (needs _native)."""
    import importlib.util

    path = ROOT / "api" / "profiles.py"
    spec = importlib.util.spec_from_file_location("arx_m_can_profiles", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load profiles from {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


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
