"""Robot variant profiles (PX6D vs plain ARX URDF)."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import yaml

ProfileName = Literal["px6d", "arx", "follower"]

_REPO_ROOT = Path(__file__).resolve().parent.parent
_CONFIG_ROOT = _REPO_ROOT / "arx_m_can" / "config"
_MODELS_ROOT = _REPO_ROOT / "arx_m_can" / "models"
_URDF_ROOT = Path(
    "/home/ubuntu/Alicia/Synria-Robot-Descriptions/synriard/urdf/Alicia_M_v1_2"
)

DEFAULT_PROFILE: ProfileName = "px6d"


@dataclass(frozen=True)
class RobotProfile:
    name: ProfileName
    urdf: Path
    dynamics_json: Path
    scales_yaml: Path
    dm_yaml: Path
    dm_float_yaml: Path
    base_link: str = "base_link"
    end_link: str = "tool0"


def _profile(name: ProfileName, urdf_file: str, dynamics_file: str) -> RobotProfile:
    profile_dir = _CONFIG_ROOT / "profiles" / name
    return RobotProfile(
        name=name,
        urdf=_URDF_ROOT / urdf_file,
        dynamics_json=_MODELS_ROOT / dynamics_file,
        scales_yaml=profile_dir / "gravity_joint_scales.yaml",
        dm_yaml=profile_dir / "arx_m_can_dm.yaml",
        dm_float_yaml=profile_dir / "arx_m_can_dm_float.yaml",
    )


PROFILES: dict[ProfileName, RobotProfile] = {
    "px6d": _profile(
        "px6d",
        "Alicia_M_v1_2_follower_ARX_PX6D.urdf",
        "alicia_m_px6d_dynamics.json",
    ),
    "arx": _profile(
        "arx",
        "Alicia_M_v1_2_follower_ARX.urdf",
        "alicia_m_arx_dynamics.json",
    ),
    "follower": _profile(
        "follower",
        "Alicia_M_v1_2_follower.urdf",
        "alicia_m_follower_dynamics.json",
    ),
}


def get_profile(name: str | ProfileName = DEFAULT_PROFILE) -> RobotProfile:
    key = str(name).strip().lower()
    if key not in PROFILES:
        choices = ", ".join(sorted(PROFILES))
        raise ValueError(f"Unknown robot {name!r}; choose one of: {choices}")
    return PROFILES[key]  # type: ignore[index]


get_robot = get_profile
PROFILE_NAMES = tuple(PROFILES)


def load_gravity_scales(path: Path | None = None) -> list[float]:
    """Load 6 joint_scales from a profile YAML (default: px6d)."""
    yaml_path = path or get_profile().scales_yaml
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    scales = [float(v) for v in data.get("joint_scales") or []]
    if len(scales) != 6:
        raise ValueError(f"Expected 6 gravity scales in {yaml_path}, got {len(scales)}")
    return scales


def export_dynamics_model(profile: RobotProfile, *, out: Path | None = None) -> Path:
    """Export RoboCore dynamics JSON from the profile URDF."""
    from robocore.dynamics.cpp import arrays_from_dynamics_model
    from robocore.dynamics.model import build_dynamics_model
    from robocore.modeling import RobotModel

    if not profile.urdf.is_file():
        raise FileNotFoundError(f"URDF not found: {profile.urdf}")

    robot = RobotModel(str(profile.urdf), base_link=profile.base_link, end_link=profile.end_link)
    dyn_model = build_dynamics_model(robot)
    (
        parents,
        joint_types,
        q_indices,
        t_origin,
        axes,
        masses,
        coms,
        inertias,
        gravity,
        n_dof,
    ) = arrays_from_dynamics_model(dyn_model)

    payload = {
        "urdf": str(profile.urdf),
        "base_link": profile.base_link,
        "end_link": profile.end_link,
        "n_dof": int(n_dof),
        "n_bodies": int(parents.shape[0]),
        "parents": parents.astype(int).tolist(),
        "joint_types": joint_types.astype(int).tolist(),
        "q_indices": q_indices.astype(int).tolist(),
        "t_origin_colmajor": t_origin.reshape(-1).astype(float).tolist(),
        "axes": axes.reshape(-1).astype(float).tolist(),
        "masses": masses.astype(float).tolist(),
        "coms": coms.reshape(-1).astype(float).tolist(),
        "inertias_colmajor": inertias.reshape(-1).astype(float).tolist(),
        "gravity": gravity.astype(float).tolist(),
    }

    out_path = out or profile.dynamics_json
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return out_path
