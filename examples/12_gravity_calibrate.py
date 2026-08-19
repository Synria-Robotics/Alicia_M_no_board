#!/usr/bin/env python3
"""Distal-first gravity calibration for Alicia M + PX6D (link6 → link1).

Workflow (run in ``arx`` conda env)::

    conda activate arx
    cd /home/ubuntu/Synria/arx/Rust/arx_m_can
    maturin develop --release

    # 1) After URDF edits — refresh embedded model
    python examples/12_gravity_calibrate.py export-model

    # 2) Audit masses / merged link6
    python examples/12_gravity_calibrate.py audit

    # 3) Model-only gravity at calibration poses
    python examples/12_gravity_calibrate.py model-g --focus-joint 6

    # 4) Hardware float + torque sampling (kp=0, kd=1 — manually hold pose first)
    python examples/12_gravity_calibrate.py measure --focus-joint 4 --settle-sec 3 --sample-sec 10
    python examples/12_gravity_calibrate.py float-test --duration 15

    # 5) Write tuned scales
    python examples/12_gravity_calibrate.py write-scales 1.0 1.55 1.55 1.0 1.05 1.08

Requires MIT mode config (kp=0, kd=1) for measurement. Manually move/hold the arm at the test pose.
"""

from __future__ import annotations

import argparse
import json
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = Path(
    "/home/ubuntu/Alicia/Synria-Robot-Descriptions/synriard/urdf/Alicia_M_v1_2/"
    "Alicia_M_v1_2_follower_ARX_PX6D.urdf"
)
DEFAULT_JSON = ROOT / "arx_m_can" / "models" / "alicia_m_px6d_dynamics.json"
DEFAULT_SCALES = ROOT / "arx_m_can" / "config" / "gravity_joint_scales.yaml"
EXPORT_SCRIPT = ROOT / "scripts" / "export_dynamics_model.py"

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
LINK_NAMES = [f"link{i}" for i in range(1, 7)]

# Poses for distal-first tuning (rad). Keys are focus joint 1..6.
CALIBRATION_POSES: dict[int, dict[str, list[float]]] = {
    6: {
        "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "j6_neg90": [0.0, 0.0, 0.0, 0.0, 0.0, -1.5708],
        "j6_pos90": [0.0, 0.0, 0.0, 0.0, 0.0, 1.5708],
    },
    5: {
        "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "j5_neg90": [0.0, 0.0, 0.0, 0.0, -1.5708, 0.0],
        "j5_pos90": [0.0, 0.0, 0.0, 0.0, 1.5708, 0.0],
    },
    4: {
        "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "j4_neg90": [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0],
        "j4_pos90": [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0],
    },
    3: {
        "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "j3_neg45": [0.0, 0.0, -0.7854, 0.0, 0.0, 0.0],
        "j3_pos45": [0.0, 0.0, 0.7854, 0.0, 0.0, 0.0],
    },
    2: {
        "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "j2_neg45": [0.0, -0.7854, 0.0, 0.0, 0.0, 0.0],
        "j2_pos45": [0.0, 0.7854, 0.0, 0.0, 0.0, 0.0],
    },
    1: {
        "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "j1_neg90": [-1.5708, 0.0, 0.0, 0.0, 0.0, 0.0],
        "j1_pos90": [1.5708, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
}


def load_scales_yaml(path: Path) -> tuple[list[float], dict[str, Any]]:
    text = path.read_text(encoding="utf-8")
    scales: list[float] = []
    meta: dict[str, Any] = {}
    in_scales = False
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("urdf:"):
            meta["urdf"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("joint_scales:"):
            in_scales = True
            continue
        if in_scales:
            if stripped.startswith("- "):
                value = stripped[2:].split("#", 1)[0].strip()
                scales.append(float(value))
            elif stripped and not stripped.startswith("#"):
                in_scales = False
    if len(scales) != 6:
        raise SystemExit(f"Expected 6 joint_scales in {path}, got {len(scales)}")
    return scales, meta


def write_scales_yaml(path: Path, scales: list[float], urdf: Path) -> None:
    if len(scales) != 6:
        raise SystemExit("write-scales requires exactly 6 values")
    lines = [
        "# Per-joint gravity feedforward scales (MIT tau_ff = scale[i] * g(q)[i]).",
        "# Tune distal-first: joint6 → joint1.",
        "",
        f"urdf: {urdf}",
        "base_link: base_link",
        "end_link: tool0",
        "",
        "joint_scales:",
    ]
    comments = [
        "joint1 / link1",
        "joint2 / link2",
        "joint3 / link3",
        "joint4 / link4",
        "joint5 / link5",
        "joint6 / link6 (+ merged link7/8/tool0)",
    ]
    for value, comment in zip(scales, comments):
        lines.append(f"  - {value:.4f}  # {comment}")
    lines.extend(
        [
            "",
            "notes: >",
            "  Re-export JSON after URDF edits; maturin develop --release.",
            "  Float test: arx_m_can_dm.yaml (kp=0, kd=1).",
            "",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {path}")
    print("  scales =", [f"{v:.4f}" for v in scales])


def cmd_guide(_: argparse.Namespace) -> int:
    print(
        """
Distal-first gravity calibration (link6 → link1)
==============================================

Phase 0 — Model refresh
  • Fix URDF masses (especially link6 + merged link7/8/tool0).
  • python examples/12_gravity_calibrate.py export-model
  • maturin develop --release
  • python examples/12_gravity_calibrate.py audit

Phase 1 — Joint 6 (link6 distal chain)
  • Confirm merged link6 mass in audit output.
  • Float: python examples/12_gravity_calibrate.py float-test --focus-joint 4
  • Manually hold pose, then: measure --focus-joint 4 --settle-sec 3 --sample-sec 10
  • Adjust scale[5] (6th value) until arm feels neutral (no sag / rise).

Phase 2 — Joint 5 … Phase 6 — Joint 1
  • Repeat measure/float for focus joints 5, 4, 3, 2, 1.
  • Do not retune distal joints unless URDF mass changed upstream.

Phase 7 — Full-chain validation
  • Sweep several poses; check coupling (j2/j3 often need ~1.55).
  • python examples/12_gravity_calibrate.py compare-ff --duration 10

Phase 8 — Production
  • write-scales with final values.
  • Restore arx_m_can_dm_origin.yaml MIT kp/kd.
  • Re-test with production gains + gravity FF on.

Tips
  • kp=0, kd=1: tau_cmd ≈ scale * g(q). Wrong scale → visible sag.
  • |g_model| < 0.05 N·m at a pose → skip scale estimate there.
  • Support the arm when commanding large moves.
"""
    )
    return 0


def _robot_model(urdf: Path, end_link: str = "tool0"):
    from robocore.modeling import RobotModel

    return RobotModel(str(urdf), base_link="base_link", end_link=end_link)


def cmd_audit(args: argparse.Namespace) -> int:
    from robocore.dynamics.model import build_dynamics_model

    urdf = Path(args.urdf)
    robot = _robot_model(urdf, args.end_link)
    dyn = build_dynamics_model(robot)

    inertials = robot.parsed_model.get_link_inertials()
    print(f"URDF: {urdf}")
    print(f"Chain: base_link → {args.end_link}, n_dof={dyn.nq}\n")
    print(f"{'body':>12} {'link':>8} {'type':>10} {'q_idx':>5} {'mass_kg':>10} {'com_x':>8} {'com_y':>8} {'com_z':>8}")
    print("-" * 80)
    urdf_mass_sum = 0.0
    model_mass_sum = 0.0
    for i, body in enumerate(dyn.bodies):
        urdf_m = float(inertials.get(body.link_name, (0.0,))[0])
        flag = " *" if urdf_m > 1e-9 and abs(urdf_m - body.mass) > 1e-4 and body.mass > 1e-9 else ""
        if body.mass > 1e-9:
            model_mass_sum += body.mass
        if urdf_m > 1e-9:
            urdf_mass_sum += urdf_m
        com = body.com_xyz
        print(
            f"{i:12d} {body.link_name:>8} {body.joint_type:>10} {body.q_index:5d} "
            f"{body.mass:10.4f} {com[0]:8.4f} {com[1]:8.4f} {com[2]:8.4f}{flag}"
        )

    print("-" * 80)
    print(f"Sum of non-zero model body masses: {model_mass_sum:.4f} kg")
    print(f"Sum of URDF link inertial masses (all links in file): {urdf_mass_sum:.4f} kg")
    link6 = next((b for b in dyn.bodies if b.link_name == "link6"), None)
    if link6:
        print(
            f"\nMerged link6 (includes fixed descendants): mass={link6.mass:.4f} kg, "
            f"com={link6.com_xyz.round(4).tolist()}"
        )
    tool0 = inertials.get("tool0", (0.0,))[0]
    if float(tool0) < 1e-6:
        print("Note: tool0 has zero URDF inertial — PX6D sensor mass should live in link6 merge or tool0.")

    if DEFAULT_JSON.is_file():
        payload = json.loads(DEFAULT_JSON.read_text(encoding="utf-8"))
        json_masses = payload.get("masses", [])
        if json_masses:
            print(f"\nEmbedded JSON ({DEFAULT_JSON.name}): sum(masses)={sum(json_masses):.4f} kg")
            if abs(sum(json_masses) - model_mass_sum) > 1e-3:
                print("  WARNING: JSON masses differ from current URDF — run export-model.")
    return 0


def cmd_export_model(args: argparse.Namespace) -> int:
    cmd = [
        sys.executable,
        str(EXPORT_SCRIPT),
        "--urdf",
        str(args.urdf),
        "--end-link",
        args.end_link,
        "--out",
        str(args.out),
    ]
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)
    print("Next: maturin develop --release  (re-embed JSON in Rust binary)")
    return 0


def cmd_model_g(args: argparse.Namespace) -> int:
    from robocore.dynamics.api import gravity

    urdf = Path(args.urdf)
    robot = _robot_model(urdf, args.end_link)
    focus = args.focus_joint
    poses = CALIBRATION_POSES.get(focus, CALIBRATION_POSES[6])
    if args.pose != "all" and args.pose not in poses:
        raise SystemExit(f"Unknown pose {args.pose!r} for joint {focus}. Choose: {list(poses)} or 'all'")

    q_list = list(poses.values()) if args.pose == "all" else [poses[args.pose]]
    names = list(poses.keys()) if args.pose == "all" else [args.pose]

    print(f"g(q) from RoboCore (no joint_scales), focus joint {focus}\n")
    print(f"{'pose':>12} " + " ".join(f"{n:>10}" for n in JOINT_NAMES))
    print("-" * 80)
    for name, q in zip(names, q_list):
        g = np.asarray(gravity(robot, q, backend="cpp"), dtype=float).ravel()
        mark = " <--" if focus >= 1 else ""
        cells = []
        for i, val in enumerate(g):
            suffix = mark if (i + 1) == focus else ""
            cells.append(f"{val:10.3f}{suffix}")
        print(f"{name:>12} " + " ".join(cells))
    return 0


def _import_native():
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    try:
        import _native
    except ImportError as exc:
        raise ImportError(
            "Run: conda activate arx && maturin develop --release"
        ) from exc
    return _native


def cmd_float_test(args: argparse.Namespace) -> int:
    native = _import_native()
    scales, _ = load_scales_yaml(Path(args.scales_file))
    stop = False

    def _sig(_a, _b):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sig)
    robot = native.Robot(
        port=args.port,
        control_hz=args.control_hz,
        arm_control_mode="mit",
        use_gravity_ff=True,
        gravity_joint_scales=scales,
        auto_enable=True,
    )
    focus = args.focus_joint
    print(f"Float test (FF on, scales={scales}). Focus: joint{focus}. Ctrl+C to stop.")
    t0 = time.time()
    try:
        while not stop and (time.time() - t0) < args.duration:
            state = robot.get_arm_state(True)
            q = state["arm"]["positions"][:6]
            tau = state["arm"]["torques"][:6]
            line = f"q={[f'{v:.3f}' for v in q]}  tau={[f'{v:.2f}' for v in tau]}"
            if focus:
                j = focus - 1
                line += f"  | focus j{focus}: q={q[j]:.3f} tau={tau[j]:.2f}"
            print(line, end="\r", flush=True)
            time.sleep(0.25)
    finally:
        robot.disable()
        robot.disconnect()
        print("\nStopped.")
    return 0


def cmd_measure(args: argparse.Namespace) -> int:
    from robocore.dynamics.api import gravity

    native = _import_native()
    scales, _ = load_scales_yaml(Path(args.scales_file))
    urdf = Path(args.urdf)
    robot_model = _robot_model(urdf, args.end_link)
    focus = args.focus_joint

    print("Measure at current pose (no auto-move; kp=0 cannot hold position).")
    print(f"Manually place/hold the arm, then sampling starts after {args.settle_sec:.1f}s settle.\n")

    robot = native.Robot(
        port=args.port,
        control_hz=native.DEFAULT_MIT_CONTROL_HZ,
        arm_control_mode="mit",
        use_gravity_ff=True,
        gravity_joint_scales=scales,
        auto_enable=True,
    )
    try:
        q0 = robot.get_joint_positions()
        print(f"Current q = {[f'{v:.4f}' for v in q0[:6]]}")
        print(f"Settling {args.settle_sec:.1f}s … hold pose steady (light touch OK).")
        time.sleep(args.settle_sec)

        samples_q: list[list[float]] = []
        samples_tau: list[list[float]] = []
        t_end = time.time() + args.sample_sec
        while time.time() < t_end:
            state = robot.get_arm_state(True)
            samples_q.append(list(state["arm"]["positions"][:6]))
            samples_tau.append(list(state["arm"]["torques"][:6]))
            time.sleep(1.0 / max(args.sample_hz, 1.0))

        q_med = [statistics.median(col) for col in zip(*samples_q)]
        tau_med = [statistics.median(col) for col in zip(*samples_tau)]
        g_model = np.asarray(gravity(robot_model, q_med, backend="cpp"), dtype=float).ravel()
        g_ff = [scales[i] * g_model[i] for i in range(6)]

        print(f"\nMedian over {len(samples_q)} samples at held pose:")
        print(f"  q     = {[f'{v:.4f}' for v in q_med]}")
        print(f"  tau_m = {[f'{v:.3f}' for v in tau_med]}")
        print(f"  g_mod = {[f'{v:.3f}' for v in g_model]}")
        print(f"  tau_ff= {[f'{v:.3f}' for v in g_ff]} (with current scales)")

        j = focus - 1
        if abs(g_model[j]) >= args.min_g:
            ratio = tau_med[j] / g_model[j]
            new_scale = scales[j] * (tau_med[j] / g_ff[j]) if abs(g_ff[j]) > 1e-6 else ratio
            print(
                f"\nFocus joint{focus}: scale={scales[j]:.4f}  "
                f"suggest scale≈{ratio:.4f}  (refine→{new_scale:.4f} from tau/g_ff)"
            )
            updated = scales.copy()
            updated[j] = ratio
            print(f"  Updated scales preview: {[f'{v:.4f}' for v in updated]}")
            print("  Apply: python examples/12_gravity_calibrate.py write-scales", " ".join(f"{v:.4f}" for v in updated))
        else:
            print(f"\nFocus joint{focus}: |g_model|={abs(g_model[j]):.4f} < {args.min_g} — try another pose.")

        err = [tau_med[i] - g_ff[i] for i in range(6)]
        print(f"  tau_err (meas - ff): {[f'{v:.3f}' for v in err]}")
    finally:
        robot.disable()
        robot.disconnect()
    return 0


def cmd_compare_ff(args: argparse.Namespace) -> int:
    native = _import_native()
    scales, _ = load_scales_yaml(Path(args.scales_file))

    def run_block(ff: bool, label: str) -> None:
        print(f"\n=== {label} (gravity_ff={ff}) ===")
        robot = native.Robot(
            port=args.port,
            control_hz=args.control_hz,
            arm_control_mode="mit",
            use_gravity_ff=ff,
            gravity_joint_scales=scales if ff else [1.0] * 6,
            auto_enable=True,
        )
        t0 = time.time()
        try:
            while (time.time() - t0) < args.duration:
                q = robot.get_joint_positions()
                print(f"  q={[f'{v:.3f}' for v in q[:6]]}", end="\r", flush=True)
                time.sleep(0.3)
        finally:
            robot.disable()
            robot.disconnect()
        print()

    print("A/B compare: same pose, FF off then on. Observe sag vs float.")
    input("Press Enter when arm is at test pose and clear…")
    run_block(False, "A: FF OFF")
    input("Press Enter for B (FF ON)…")
    run_block(True, "B: FF ON")
    return 0


def cmd_write_scales(args: argparse.Namespace) -> int:
    write_scales_yaml(Path(args.scales_file), list(args.values), Path(args.urdf))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--end-link", default="tool0")
    parser.add_argument("--scales-file", type=Path, default=DEFAULT_SCALES)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("guide", help="Print distal-first checklist").set_defaults(func=cmd_guide)

    p = sub.add_parser("audit", help="URDF vs dynamics model mass audit")
    p.set_defaults(func=cmd_audit)

    p = sub.add_parser("export-model", help="Re-export alicia_m_px6d_dynamics.json")
    p.add_argument("--out", type=Path, default=DEFAULT_JSON)
    p.set_defaults(func=cmd_export_model)

    p = sub.add_parser("model-g", help="Print g(q) at calibration poses (no hardware)")
    p.add_argument("--focus-joint", type=int, default=6, choices=range(1, 7))
    p.add_argument("--pose", default="all", help="Pose name or 'all'")
    p.set_defaults(func=cmd_model_g)

    p = sub.add_parser("float-test", help="Live float test with current scales")
    p.add_argument("--port", default=None)
    p.add_argument("--duration", type=float, default=15.0)
    p.add_argument("--control-hz", type=float, default=500.0)
    p.add_argument("--focus-joint", type=int, default=6)
    p.set_defaults(func=cmd_float_test)

    p = sub.add_parser("measure", help="Sample tau vs g(q) at current pose (no auto-move)")
    p.add_argument("--port", default=None)
    p.add_argument("--focus-joint", type=int, default=4, choices=range(1, 7))
    p.add_argument("--settle-sec", type=float, default=2.0, help="Wait after connect before sampling")
    p.add_argument("--sample-sec", type=float, default=3.0, help="Sampling duration at held pose")
    p.add_argument("--sample-hz", type=float, default=50.0)
    p.add_argument("--min-g", type=float, default=0.05, help="Min |g| to suggest scale")
    p.set_defaults(func=cmd_measure)

    p = sub.add_parser("compare-ff", help="A/B FF off vs on at same pose")
    p.add_argument("--port", default=None)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--control-hz", type=float, default=500.0)
    p.set_defaults(func=cmd_compare_ff)

    p = sub.add_parser("write-scales", help="Save 6 scale values to YAML")
    p.add_argument("values", type=float, nargs=6, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    p.set_defaults(func=cmd_write_scales)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
