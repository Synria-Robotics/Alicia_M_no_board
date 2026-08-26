#!/usr/bin/env python3
"""Export RoboCore chain dynamics arrays to JSON for arx_m_can.

Use the ``arx`` conda env::

    conda activate arx
    cd /home/ubuntu/Synria/arx/Rust/arx_m_can

    # By robot variant (recommended)
    python scripts/export_dynamics_model.py --robot follower
    python scripts/export_dynamics_model.py --robot px6d

    # Manual URDF path
    python scripts/export_dynamics_model.py --urdf /path/to/robot.urdf
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robocore.dynamics.cpp import arrays_from_dynamics_model
from robocore.dynamics.model import build_dynamics_model
from robocore.modeling import RobotModel

DEFAULT_URDF = Path(
    "/home/ubuntu/Alicia/Synria-Robot-Descriptions/synriard/urdf/Alicia_M_v1_2/"
    "Alicia_M_v1_2_follower_ARX_PX6D.urdf"
)
DEFAULT_OUT = ROOT / "arx_m_can" / "models" / "alicia_m_px6d_dynamics.json"


def export_model(urdf_path: Path, base_link: str, end_link: str) -> dict:
    robot = RobotModel(str(urdf_path), base_link=base_link, end_link=end_link)
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
    n_bodies = int(parents.shape[0])
    return {
        "urdf": str(urdf_path),
        "base_link": base_link,
        "end_link": end_link,
        "n_dof": int(n_dof),
        "n_bodies": n_bodies,
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        choices=("px6d", "arx", "follower"),
        default=None,
        help="Export dynamics JSON for a robot variant",
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--base-link", default="base_link")
    parser.add_argument("--end-link", default="tool0")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    if args.robot:
        import importlib.util

        path = ROOT / "api" / "profiles.py"
        spec = importlib.util.spec_from_file_location("arx_m_can_profiles", path)
        if spec is None or spec.loader is None:
            raise SystemExit(f"Cannot load profiles from {path}")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = mod
        spec.loader.exec_module(mod)
        profile = mod.get_robot(args.robot)
        out = mod.export_dynamics_model(profile, out=args.out if args.out != DEFAULT_OUT else None)
        payload = out.read_text(encoding="utf-8")
        import json

        data = json.loads(payload)
        print(f"Wrote {out} (n_dof={data['n_dof']}, n_bodies={data['n_bodies']})")
        return

    if not args.urdf.is_file():
        raise SystemExit(f"URDF not found: {args.urdf}")

    payload = export_model(args.urdf, args.base_link, args.end_link)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    import json

    args.out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {args.out} (n_dof={payload['n_dof']}, n_bodies={payload['n_bodies']})")


if __name__ == "__main__":
    main()
