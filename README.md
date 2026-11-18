# Alicia M CAN

Rust robot SDK for Alicia M arms (Damiao motors over USB2CAN), with a Python API
styled like [Alicia-D-SDK](https://github.com/Synria-Robotics/Alicia-D-SDK).

This tree is the ARX / no-board driver used by PTI_vial. **Alicia-M-SDK is not required.**

## Layout

```text
Alicia_M_no_board/
├── alicia_m_can/          # Rust library + config/profiles/{px6d,arx,arx_cam,follower}
├── alicia_m_can_py/       # PyO3 extension (_native)
├── api/                   # profiles, kinematics, SynriaRobotAPI
├── examples/
└── pyproject.toml
```

## Repo path dependencies

`alicia_m_can` links local crates from the PTI_vial third_party tree:

| Crate | Path |
|-------|------|
| `motor_core` / `motor_vendor_damiao` | `/home/ubuntu/Github/PTI_vial/third_party/motorbridge/...` |
| `robocore_dynamics_ffi` | `/home/ubuntu/Github/PTI_vial/third_party/robocore_dynamics_ffi` (wraps RoboCore `@ v2.5.1-dev` native dynamics) |

## Build Rust

```bash
source "$HOME/.cargo/env"
cd /home/ubuntu/Synria/Alicia_M_no_board
cargo build -p alicia_m_can
cargo test -p alicia_m_can
```

## Install Python API

Requires `python3-dev` (Ubuntu: `sudo apt install python3-dev`).

Use the PTI_vial venv (not conda):

```bash
source /home/ubuntu/Github/PTI_vial/.venv/bin/activate
pip install maturin
export PYO3_PYTHON=/home/ubuntu/Github/PTI_vial/.venv/bin/python
cd /home/ubuntu/Synria/Alicia_M_no_board
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
