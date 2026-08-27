#!/usr/bin/env python3
"""Export RoboCore chain dynamics arrays to JSON for arx_m_can.

    python scripts/export_dynamics_model.py --robot arx
    python scripts/export_dynamics_model.py --robot px6d
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from api.profiles import DEFAULT_PROFILE, PROFILE_NAMES, export_dynamics_model, get_profile


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=PROFILE_NAMES, default=DEFAULT_PROFILE)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    out = export_dynamics_model(get_profile(args.robot), out=args.out)
    data = json.loads(out.read_text(encoding="utf-8"))
    print(f"Wrote {out} (n_dof={data['n_dof']}, n_bodies={data['n_bodies']})")


if __name__ == "__main__":
    main()
