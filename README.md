# Alicia M CAN

Rust robot SDK for Alicia M arms (Damiao motors over USB2CAN), with a Python API
styled like [Alicia-D-SDK](https://github.com/Synria-Robotics/Alicia-D-SDK).

## Layout

```text
alicia_m_rs/
├── alicia_m_can/          # Rust library + config/profiles/{px6d,arx,arx_cam,follower}
├── alicia_m_can_py/       # PyO3 extension (_native)
├── api/                   # profiles, kinematics, SynriaRobotAPI
├── examples/
└── pyproject.toml
```

## Build Rust

```bash
source "$HOME/.cargo/env"
cd /home/ubuntu/Synria/arx/Rust/alicia_m_rs
cargo build -p alicia_m_can
cargo test -p alicia_m_can
```

## Install Python API

Requires `python3-dev` (Ubuntu: `sudo apt install python3-dev`).

Activate your Python environment first (e.g. `conda activate geo`), then:

```bash
pip install maturin
cd /home/ubuntu/Synria/arx/Rust/alicia_m_rs
maturin develop
```

This installs the Rust extension as top-level `_native`. Examples add the repo root to
`sys.path` and import from `api/` — run them from the repo root:

```bash
python examples/01_read_state.py --port /dev/ttyACM0
```

## Python usage

```python
from api.synria_robot_api import create_robot

robot = create_robot(port="/dev/ttyACM0")
try:
    robot.set_robot_state([0, -0.2, -0.2, 0, 0, 0], speed=200, wait_for_completion=True)
    print(robot.get_robot_state("joint"))
finally:
    robot.disconnect()
```

Higher-level helpers are also exported from `api.synria_robot_api`:

- `Arm` — low-level connect / read_state / scan_ids / set_zero
- `scan_ids()` — standalone motor ID scan without enabling the arm
- `Teleoperation` — Alicia-D leader → Alicia M CAN follower (Rust loop)
- `SparkVisBridge` — SparkVis WebSocket drag-to-robot bridge

## Examples

All examples accept `--port`, `--baud`, and `--robot {px6d,arx,arx_cam,follower}`
(default `/dev/ttyACM0`, `1000000`, `px6d`).
Ordered basic → advanced:

| # | Script | Topic |
|---|--------|--------|
| 01 | `read_state` | Read joint positions (no enable) |
| 02 | `scan_ids` | Scan motor IDs |
| 03 | `move_joint` | Joint motion |
| 04 | `move_gripper` | Gripper |
| 05 | `go_home` | Return to zero |
| 06 | `forward_kinematics` | FK (RoboCore) |
| 07 | `inverse_kinematics` | IK + optional motion |
| 08 | `zero_current_position` | Motor zero (destructive) |
| 09 | `mit_gravity_hold` | MIT + gravity FF hold |
| 10 | `verify_poses` | Calibration pose verify |
| 11 | `diagnose_mit_joint` | Single-joint MIT debug |
| 12 | `gravity_calibrate` | Gravity FF calibration |
| 13 | `mit_gain_autotune` | Kp/Kd autotune |
| 14 | `teleop_alicia_d_leader` | Alicia-D leader teleop |
| 15 | `sparkvis_control` | SparkVis WebSocket bridge |

```bash
sudo chmod 666 /dev/ttyACM0

python examples/01_read_state.py
python examples/02_scan_ids.py
python examples/03_move_joint.py "0,-0.2,0,0,0,0" --speed 60
python examples/06_forward_kinematics.py
python examples/07_inverse_kinematics.py --execute --speed 60
python examples/10_verify_poses.py --speed 60 --motion-tolerance 0.02
python examples/14_teleop_alicia_d_leader.py --leader-port /dev/ttyACM0 --follower-port /dev/ttyACM1
python examples/15_sparkvis_control.py --dry-run
```

Hardware config lives under `alicia_m_can/config/profiles/<robot>/alicia_m_can_dm.yaml`
(default robot: `px6d`).
