#!/usr/bin/env python3
"""Auto-tune MIT Kp/Kd using position/velocity error metrics (gravity FF on).

Run in ``arx`` conda env with the arm manually placed at a representative hold pose::

    conda activate arx
    cd /home/ubuntu/Synria/arx/Rust/alicia_m_rs
    maturin develop --release

    # Tune all joints (proximal → distal, j1 → j6), writes alicia_m_can_dm.yaml
    python examples/13_mit_gain_autotune.py tune --write-yaml
    python examples/13_mit_gain_autotune.py --port /dev/ttyACM1 tune --write-yaml

    # Step test (target joint moves between two angles; others fixed) — best for j1–j4
    python examples/13_mit_gain_autotune.py tune --focus-joint 1 --mode step --joint-delta 0.4 --write-yaml

    # Static hold test (only useful when |g(q)| loads the joint)
    python examples/13_mit_gain_autotune.py tune --focus-joint 3 --mode hold --write-yaml

    # J6 at calibration ref:
    python examples/13_mit_gain_autotune.py tune --focus-joint 6 --goto-ref-pose --write-yaml

    # ARX arm (production config + gravity FF)
    python examples/13_mit_gain_autotune.py --robot arx --no-float tune --focus-joint 1 --mode step --write-yaml
    python examples/13_mit_gain_autotune.py --robot arx --no-float tune --write-yaml
    python examples/13_mit_gain_autotune.py --robot arx --no-float tune --focus-joint 6 --goto-ref-pose --write-yaml

    # Follower arm
    python examples/13_mit_gain_autotune.py --robot follower tune --focus-joint 1 --mode step --write-yaml
    python examples/13_mit_gain_autotune.py --robot follower tune --write-yaml

Heuristic from metrics:
  - high RMSE pos, low RMSE vel → Kp too low
  - high RMSE vel / oscillation   → Kd too low or Kp too high
  - cost = rmse_pos + w_vel * rmse_vel
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

from common import DEFAULT_POSES, add_robot_arguments, load_gravity_scales, make_robot, resolve_robot_setup

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
# 4340P for j1-j3, 4310 for j4-j6
DEFAULT_KP = [120.0, 120.0, 120.0, 18.0, 18.0, 18.0]
DEFAULT_KD = [8.0, 8.0, 8.0, 2.0, 2.0, 2.0]

KD_MULTS = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0]
KP_MULTS = [0.5, 0.75, 1.0, 1.25, 1.5]


@dataclass
class TrialMetrics:
    rmse_pos: float
    rmse_vel: float
    max_abs_err: float
    final_err: float
    overshoot: float
    cost: float


def make_q_with_focus(q_base: list[float], focus_joint: int, focus_value: float) -> list[float]:
    q = list(q_base)
    q[focus_joint - 1] = focus_value
    return q


def step_positions(q_center: list[float], focus_joint: int, delta: float) -> tuple[list[float], list[float], list[float]]:
    j = focus_joint - 1
    q_low = make_q_with_focus(q_center, focus_joint, q_center[j] - delta)
    q_high = make_q_with_focus(q_center, focus_joint, q_center[j] + delta)
    return q_low, q_center, q_high


def load_ref_pose(path: Path) -> list[float]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    ref = data.get("ref")
    if not ref or len(ref) < 6:
        raise SystemExit(f"Expected ref: [6 floats] in {path}")
    return [float(v) for v in ref[:6]]


def load_mit_gains_yaml(path: Path) -> tuple[list[float], list[float]]:
    """Load MIT kp/kd for joint1..joint6 from a dm-style yaml."""
    kp: list[float] = []
    kd: list[float] = []
    current: str | None = None
    for line in path.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if s.startswith("- name: joint"):
            current = s.split(":", 1)[1].strip()
        elif current and current in JOINT_NAMES and s.startswith("kp:"):
            kp.append(float(s.split(":", 1)[1].strip()))
        elif current and current in JOINT_NAMES and s.startswith("kd:"):
            kd.append(float(s.split(":", 1)[1].strip()))
            current = None
    if len(kp) != 6 or len(kd) != 6:
        return DEFAULT_KP.copy(), DEFAULT_KD.copy()
    return kp, kd


def resolve_baseline_gains(args: argparse.Namespace) -> tuple[list[float], list[float], Path]:
    """Baseline for tuning: dm yaml by default (keeps prior per-joint results)."""
    if args.kp_baseline and args.kd_baseline:
        return list(args.kp_baseline), list(args.kd_baseline), Path("(cli)")
    if args.from_origin and not args.origin_yaml:
        raise SystemExit("--from-origin requires --origin-yaml")
    baseline_path = Path(args.origin_yaml if args.from_origin else args.baseline_yaml)
    kp, kd = load_mit_gains_yaml(baseline_path)
    if args.kp_baseline:
        kp = list(args.kp_baseline)
    if args.kd_baseline:
        kd = list(args.kd_baseline)
    return kp, kd, baseline_path


def joint_tune_order(order: str, focus_joint: int | None) -> list[int]:
    if focus_joint is not None:
        return [focus_joint]
    if order == "distal":
        return list(range(6, 0, -1))  # j6 → j1
    return list(range(1, 7))  # j1 → j6 (default, safer for Kp/Kd)


def resolve_tune_paths(args: argparse.Namespace, setup) -> tuple[Path, Path, Path]:
    if args.from_origin and not args.origin_yaml:
        raise SystemExit("--from-origin requires --origin-yaml")
    baseline = Path(args.origin_yaml if args.from_origin else args.baseline_yaml or setup.config_path)
    dm = Path(args.dm_yaml or setup.config_path)
    return baseline, dm, setup.scales_path


def evaluate_hold(
    robot,
    q_des: list[float],
    *,
    focus_joint: int,
    duration: float,
    sample_hz: float,
    skip_sec: float,
    vel_weight: float,
) -> TrialMetrics:
    samples_q: list[list[float]] = []
    samples_v: list[list[float]] = []
    t0 = time.time()
    while time.time() - t0 < duration:
        state = robot.get_arm_state(True)
        samples_q.append(list(state["arm"]["positions"][:6]))
        samples_v.append(list(state["arm"]["velocities"][:6]))
        time.sleep(1.0 / max(sample_hz, 1.0))

    j = focus_joint - 1
    dt = 1.0 / max(sample_hz, 1.0)
    errs: list[float] = []
    vels: list[float] = []
    for idx, (q, v) in enumerate(zip(samples_q, samples_v)):
        if idx * dt < skip_sec:
            continue
        errs.append(q[j] - q_des[j])
        vels.append(v[j])

    if not errs:
        errs = [samples_q[-1][j] - q_des[j]]
        vels = [samples_v[-1][j]]

    rmse_pos = math.sqrt(sum(e * e for e in errs) / len(errs))
    rmse_vel = math.sqrt(sum(v * v for v in vels) / len(vels))
    max_abs_err = max(abs(e) for e in errs)
    final_err = abs(errs[-1])
    cost = rmse_pos + vel_weight * rmse_vel
    return TrialMetrics(rmse_pos, rmse_vel, max_abs_err, final_err, 0.0, cost)


def _sample_during(robot, sample_hz: float, stop: threading.Event) -> tuple[list[list[float]], list[list[float]]]:
    qs: list[list[float]] = []
    vs: list[list[float]] = []
    period = 1.0 / max(sample_hz, 1.0)
    while not stop.is_set():
        state = robot.get_arm_state(True)
        qs.append(list(state["arm"]["positions"][:6]))
        vs.append(list(state["arm"]["velocities"][:6]))
        time.sleep(period)
    return qs, vs


def _metrics_from_step_samples(
    q_start: list[float],
    q_target: list[float],
    focus_joint: int,
    samples_q: list[list[float]],
    samples_v: list[list[float]],
    vel_weight: float,
    overshoot_weight: float,
) -> TrialMetrics:
    j = focus_joint - 1
    if not samples_q:
        return TrialMetrics(999.0, 999.0, 999.0, 999.0, 999.0, 9999.0)

    target = q_target[j]
    direction = 1.0 if target >= q_start[j] else -1.0
    errs = [q[j] - target for q in samples_q]
    vels = [v[j] for v in samples_v]

    # Overshoot: motion past target in the direction of travel
    if direction > 0:
        overshoot = max(0.0, max(q[j] for q in samples_q) - target)
    else:
        overshoot = max(0.0, target - min(q[j] for q in samples_q))

    # Settling error: last 30% of samples
    tail_n = max(1, len(errs) * 3 // 10)
    tail_errs = errs[-tail_n:]
    tail_vels = vels[-tail_n:]
    rmse_pos = math.sqrt(sum(e * e for e in tail_errs) / len(tail_errs))
    rmse_vel = math.sqrt(sum(v * v for v in tail_vels) / len(tail_vels))
    max_abs_err = max(abs(e) for e in errs)
    final_err = abs(errs[-1])
    cost = rmse_pos + vel_weight * rmse_vel + overshoot_weight * overshoot
    return TrialMetrics(rmse_pos, rmse_vel, max_abs_err, final_err, overshoot, cost)


def evaluate_step(
    robot,
    q_center: list[float],
    q_low: list[float],
    q_high: list[float],
    *,
    focus_joint: int,
    move_speed: float,
    sample_hz: float,
    settle_sec: float,
    vel_weight: float,
    overshoot_weight: float,
) -> TrialMetrics:
    """Move focus joint low → high → low; score tracking on both steps."""
    metrics: list[TrialMetrics] = []

    for q_start, q_goal in ((q_low, q_high), (q_high, q_low)):
        robot.set_robot_state(
            target_joints=q_start,
            speed=move_speed,
            wait_for_completion=True,
        )
        time.sleep(settle_sec)

        stop = threading.Event()
        box: dict[str, list] = {"q": [], "v": []}

        def _run_sampler() -> None:
            qs, vs = _sample_during(robot, sample_hz, stop)
            box["q"] = qs
            box["v"] = vs

        sampler = threading.Thread(target=_run_sampler, daemon=True)
        sampler.start()
        robot.set_robot_state(
            target_joints=q_goal,
            speed=move_speed,
            wait_for_completion=True,
        )
        time.sleep(settle_sec)
        stop.set()
        sampler.join(timeout=2.0)

        m = _metrics_from_step_samples(
            q_start, q_goal, focus_joint, box["q"], box["v"], vel_weight, overshoot_weight
        )
        metrics.append(m)

    rmse_pos = sum(m.rmse_pos for m in metrics) / len(metrics)
    rmse_vel = sum(m.rmse_vel for m in metrics) / len(metrics)
    max_abs_err = max(m.max_abs_err for m in metrics)
    final_err = sum(m.final_err for m in metrics) / len(metrics)
    overshoot = max(m.overshoot for m in metrics)
    cost = sum(m.cost for m in metrics) / len(metrics)
    return TrialMetrics(rmse_pos, rmse_vel, max_abs_err, final_err, overshoot, cost)


def resolve_step_range(
    q_center: list[float], focus_joint: int, args: argparse.Namespace
) -> tuple[list[float], list[float]]:
    if (args.joint_low is None) ^ (args.joint_high is None):
        raise SystemExit("--joint-low and --joint-high must be set together")
    if args.joint_low is not None and args.joint_high is not None:
        return (
            make_q_with_focus(q_center, focus_joint, args.joint_low),
            make_q_with_focus(q_center, focus_joint, args.joint_high),
        )
    q_low, _, q_high = step_positions(q_center, focus_joint, args.joint_delta)
    return q_low, q_high


def run_trial(
    robot,
    q_center: list[float],
    focus_joint: int,
    q_low: list[float],
    q_high: list[float],
    args: argparse.Namespace,
) -> TrialMetrics:
    if args.mode == "step":
        return evaluate_step(
            robot,
            q_center,
            q_low,
            q_high,
            focus_joint=focus_joint,
            move_speed=args.move_speed,
            sample_hz=args.sample_hz,
            settle_sec=args.step_settle_sec,
            vel_weight=args.vel_weight,
            overshoot_weight=args.overshoot_weight,
        )
    return evaluate_hold(
        robot,
        q_center,
        focus_joint=focus_joint,
        duration=args.trial_sec,
        sample_hz=args.sample_hz,
        skip_sec=args.skip_sec,
        vel_weight=args.vel_weight,
    )


def diagnose(metrics: TrialMetrics, kp: float, kd: float, *, step_mode: bool) -> str:
    hints: list[str] = []
    if metrics.rmse_pos > 0.02 and metrics.rmse_vel < 0.15:
        hints.append("Kp may be too low (large settling error)")
    if metrics.rmse_vel > 0.25:
        hints.append("Kd may be too low or Kp too high (high velocity RMS)")
    if step_mode and metrics.overshoot > 0.03:
        hints.append(f"overshoot {metrics.overshoot:.3f} rad — try higher Kd or lower Kp")
    if metrics.max_abs_err > 0.05 and metrics.final_err < 0.01:
        hints.append("transient overshoot — try higher Kd")
    if not hints:
        hints.append("metrics look balanced")
    return f"kp={kp:.1f} kd={kd:.1f}: " + "; ".join(hints)


def _trial_header(step_mode: bool) -> str:
    if step_mode:
        return f"{'val':>8} {'rmse_p':>8} {'rmse_v':>8} {'ovrsh':>8} {'cost':>8}  note"
    return f"{'val':>8} {'rmse_p':>8} {'rmse_v':>8} {'max_e':>8} {'cost':>8}  note"


def _trial_row(val: float, m: TrialMetrics, step_mode: bool, mark: str) -> str:
    if step_mode:
        return f"{val:8.2f} {m.rmse_pos:8.4f} {m.rmse_vel:8.4f} {m.overshoot:8.4f} {m.cost:8.4f}{mark}"
    return (
        f"{val:8.2f} {m.rmse_pos:8.4f} {m.rmse_vel:8.4f} {m.max_abs_err:8.4f} {m.cost:8.4f}{mark}"
    )


def tune_joint(
    robot,
    q_center: list[float],
    focus: int,
    kp_base: list[float],
    kd_base: list[float],
    q_low: list[float],
    q_high: list[float],
    args: argparse.Namespace,
) -> tuple[float, float, TrialMetrics]:
    j = focus - 1
    kp_work = kp_base.copy()
    kd_work = kd_base.copy()
    best_kd = kd_base[j]
    best_kd_metrics: TrialMetrics | None = None
    step_mode = args.mode == "step"
    costs: list[float] = []

    print(f"\n=== Joint {focus}: Kd sweep (Kp fixed at {kp_base[j]:.1f}) ===")
    print(_trial_header(step_mode))
    print("-" * 70)
    for mult in KD_MULTS:
        kd_try = kd_base[j] * mult
        kd_work[j] = kd_try
        robot.set_arm_mit_gains(kp_work, kd_work)
        time.sleep(args.gain_settle_sec)
        m = run_trial(robot, q_center, focus, q_low, q_high, args)
        costs.append(m.cost)
        mark = ""
        if best_kd_metrics is None or m.cost < best_kd_metrics.cost:
            best_kd = kd_try
            best_kd_metrics = m
            mark = " *"
        print(_trial_row(kd_try, m, step_mode, mark))

    assert best_kd_metrics is not None
    kd_work[j] = best_kd
    print(f"  → best Kd = {best_kd:.2f}  ({diagnose(best_kd_metrics, kp_base[j], best_kd, step_mode=step_mode)})")
    if len(set(round(c, 6) for c in costs)) <= 1:
        print("  ⚠ all Kd trials identical — increase --joint-delta or check motion range")

    best_kp = kp_base[j]
    best_metrics = best_kd_metrics
    print(f"\n=== Joint {focus}: Kp sweep (Kd fixed at {best_kd:.2f}) ===")
    print(_trial_header(step_mode))
    print("-" * 70)
    costs.clear()
    for mult in KP_MULTS:
        kp_try = kp_base[j] * mult
        kp_work[j] = kp_try
        robot.set_arm_mit_gains(kp_work, kd_work)
        time.sleep(args.gain_settle_sec)
        m = run_trial(robot, q_center, focus, q_low, q_high, args)
        costs.append(m.cost)
        mark = ""
        if m.cost < best_metrics.cost:
            best_kp = kp_try
            best_metrics = m
            mark = " *"
        print(_trial_row(kp_try, m, step_mode, mark))

    print(f"  → best Kp = {best_kp:.2f}  ({diagnose(best_metrics, best_kp, best_kd, step_mode=step_mode)})")
    if len(set(round(c, 6) for c in costs)) <= 1:
        print("  ⚠ all Kp trials identical — increase --joint-delta or check motion range")
    return best_kp, best_kd, best_metrics


def write_dm_yaml(path: Path, kp: list[float], kd: list[float]) -> None:
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()
    out: list[str] = []
    current: str | None = None
    for line in lines:
        s = line.strip()
        if s.startswith("- name: joint"):
            current = s.split(":", 1)[1].strip()
        if current in JOINT_NAMES and re.match(r"^\s*kp:\s*", line):
            idx = JOINT_NAMES.index(current)
            indent = line[: len(line) - len(line.lstrip())]
            out.append(f"{indent}kp: {kp[idx]:.1f}")
            continue
        if current in JOINT_NAMES and re.match(r"^\s*kd:\s*", line):
            idx = JOINT_NAMES.index(current)
            indent = line[: len(line) - len(line.lstrip())]
            out.append(f"{indent}kd: {kd[idx]:.1f}")
            current = None
            continue
        out.append(line)
    path.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"\nWrote MIT kp/kd to {path}")


def cmd_tune(args: argparse.Namespace) -> int:
    setup = resolve_robot_setup(args)
    baseline_path, dm_path, scales_path = resolve_tune_paths(args, setup)
    args.baseline_yaml = baseline_path
    args.dm_yaml = dm_path
    scales = load_gravity_scales(scales_path)
    kp_origin, kd_origin, baseline_path = resolve_baseline_gains(args)

    print(f"Baseline yaml: {baseline_path}")
    print(f"robot={setup.robot}  config={setup.config_path.name}")
    print("Baseline Kp:   ", [f"{v:.1f}" for v in kp_origin])
    print("Baseline Kd:   ", [f"{v:.1f}" for v in kd_origin])
    print("Gravity scales:", [f"{v:.3f}" for v in scales])
    joints = joint_tune_order(args.order, args.focus_joint)
    order_label = f"j{args.focus_joint} only" if args.focus_joint else "→".join(f"j{j}" for j in joints)
    if args.mode is None:
        args.mode = "step" if args.focus_joint else "hold"
    print(f"Tune order:    {order_label}")
    print(f"Test mode:     {args.mode}")
    print("\nPlace arm at center pose (other joints fixed during step test).\n")

    robot = make_robot(args, control_hz=500.0)
    try:
        robot.set_arm_mit_gains(kp_origin, kd_origin)
        time.sleep(args.pose_settle_sec)
        if args.goto_ref_pose:
            q_ref = load_ref_pose(Path(args.poses_yaml))
            print(f"Moving to ref pose from {args.poses_yaml.name} ...")
            print(f"  q_ref = {[f'{v:.4f}' for v in q_ref]}")
            robot.set_robot_state(
                target_joints=q_ref,
                speed=args.move_speed,
                wait_for_completion=True,
                joint_tolerance=args.goto_tolerance,
                converge_timeout=args.goto_timeout,
            )
            time.sleep(args.pose_settle_sec)
            q_center = q_ref
        else:
            state = robot.get_arm_state(True)
            q_center = list(state["arm"]["positions"][:6])
        print(f"Center q = {[f'{v:.4f}' for v in q_center]}\n")

        joints = joint_tune_order(args.order, args.focus_joint)

        kp_final = kp_origin.copy()
        kd_final = kd_origin.copy()
        for focus in joints:
            q_low, q_high = resolve_step_range(q_center, focus, args)
            if args.mode == "step":
                j = focus - 1
                print(
                    f"Joint {focus} step range: "
                    f"low={q_low[j]:.4f}  high={q_high[j]:.4f}  "
                    f"(others fixed)"
                )
            best_kp, best_kd, metrics = tune_joint(
                robot, q_center, focus, kp_final, kd_final, q_low, q_high, args
            )
            kp_final[focus - 1] = best_kp
            kd_final[focus - 1] = best_kd
            robot.set_arm_mit_gains(kp_final, kd_final)
            print(
                f"\nRunning totals after j{focus}: "
                f"Kp={[f'{v:.1f}' for v in kp_final]}  "
                f"Kd={[f'{v:.1f}' for v in kd_final]}"
            )
            print(f"  final cost={metrics.cost:.4f} rmse_p={metrics.rmse_pos:.4f} rmse_v={metrics.rmse_vel:.4f}")

        print("\n=== Recommended MIT gains ===")
        for i, name in enumerate(JOINT_NAMES):
            print(f"  {name}: kp={kp_final[i]:.1f}  kd={kd_final[i]:.1f}")

        print("\nApply to yaml:")
        print(
            "  python examples/13_mit_gain_autotune.py apply "
            + " ".join(f"{v:.1f}" for v in kp_final)
            + " "
            + " ".join(f"{v:.1f}" for v in kd_final)
        )

        if args.write_yaml:
            write_dm_yaml(Path(args.dm_yaml), kp_final, kd_final)
    finally:
        robot.disable()
        robot.disconnect()
    return 0


def cmd_apply(args: argparse.Namespace) -> int:
    if len(args.values) != 12:
        raise SystemExit("apply requires 12 values: 6 kp then 6 kd")
    setup = resolve_robot_setup(args)
    write_dm_yaml(Path(args.dm_yaml or setup.config_path), list(args.values[:6]), list(args.values[6:]))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_robot_arguments(parser)
    parser.add_argument("--origin-yaml", type=Path, default=None, help="Factory defaults (use with --from-origin)")
    parser.add_argument("--baseline-yaml", type=Path, default=None, help="Starting kp/kd (default: current dm yaml)")
    parser.add_argument("--dm-yaml", type=Path, default=None, help="Output yaml for --write-yaml / apply")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("tune", help="Auto-tune Kd then Kp per joint")
    p.add_argument("--focus-joint", type=int, default=None, choices=range(1, 7))
    p.add_argument(
        "--mode",
        choices=("hold", "step"),
        default=None,
        help="hold=static pose; step=two-point move on focus joint (default: step if --focus-joint)",
    )
    p.add_argument(
        "--joint-delta",
        type=float,
        default=0.35,
        help="Step mode: focus joint moves center±delta (rad); others fixed",
    )
    p.add_argument("--joint-low", type=float, default=None, help="Step mode: absolute low angle for focus joint")
    p.add_argument("--joint-high", type=float, default=None, help="Step mode: absolute high angle for focus joint")
    p.add_argument("--move-speed", type=float, default=100.0, help="Step mode: joint move speed [0–400]")
    p.add_argument("--step-settle-sec", type=float, default=0.5, help="Step mode: settle time at each endpoint")
    p.add_argument("--overshoot-weight", type=float, default=0.8, help="Step cost weight on overshoot (rad)")
    p.add_argument(
        "--order",
        choices=("proximal", "distal"),
        default="proximal",
        help="Joint order when tuning all (default: proximal j1→j6)",
    )
    p.add_argument(
        "--from-origin",
        action="store_true",
        help="Use --origin-yaml as baseline instead of current dm yaml",
    )
    p.add_argument("--trial-sec", type=float, default=2.5, help="Seconds sampled per gain trial")
    p.add_argument("--skip-sec", type=float, default=0.4, help="Ignore first N seconds after gain change")
    p.add_argument("--gain-settle-sec", type=float, default=0.3, help="Wait after changing gains")
    p.add_argument("--pose-settle-sec", type=float, default=1.0, help="Wait after connect before tuning")
    p.add_argument("--sample-hz", type=float, default=100.0)
    p.add_argument("--vel-weight", type=float, default=0.15, help="Cost = rmse_pos + w * rmse_vel")
    p.add_argument("--write-yaml", action="store_true", help="Write results to alicia_m_can_dm.yaml")
    p.add_argument(
        "--goto-ref-pose",
        action="store_true",
        help="Move to ref pose from --poses-yaml before tuning (recommended for j6)",
    )
    p.add_argument("--poses-yaml", type=Path, default=DEFAULT_POSES)
    p.add_argument(
        "--goto-tolerance",
        type=float,
        default=0.02,
        help="Convergence tolerance when moving to ref pose (rad)",
    )
    p.add_argument(
        "--goto-timeout",
        type=float,
        default=60.0,
        help="Timeout when moving to ref pose (s)",
    )
    p.add_argument("--kp-baseline", type=float, nargs=6, default=None, metavar=("K1", "K2", "K3", "K4", "K5", "K6"))
    p.add_argument("--kd-baseline", type=float, nargs=6, default=None, metavar=("D1", "D2", "D3", "D4", "D5", "D6"))
    p.set_defaults(func=cmd_tune)

    p = sub.add_parser("apply", help="Write 6 kp + 6 kd values to dm yaml")
    p.add_argument(
        "values",
        type=float,
        nargs=12,
        metavar=("K1", "K2", "K3", "K4", "K5", "K6", "D1", "D2", "D3", "D4", "D5", "D6"),
    )
    p.set_defaults(func=cmd_apply)

    return parser


def main() -> int:
    args = build_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
