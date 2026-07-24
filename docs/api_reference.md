# ARX-D-CAN API Reference

## Teleop / calibration

```python
from arx_d_can.api import ArxDCanRobot, resolve_slave_motion, resolve_slave_port

robot = ArxDCanRobot(port="/dev/ttyACM0")
robot.set_robot_state([0.0, -0.5, 0.5, 0.0, 0.0, 0.0], gripper_value=500)
robot.go_home()
robot.disconnect()
```

Gripper user scale: **0 = closed**, **1000 = open** (Alicia-M compatible).

## Low-level hardware

```python
from arx_d_can import ArxDCanArm

arm = ArxDCanArm(port="/dev/ttyACM0", enable_gripper=True)
arm.connect()
arm.configure()
arm.enable()
arm.send_joint_positions([0.0] * 6)
arm.set_gripper(1000)
arm.close()
```

## Package layout

| Module | Purpose |
|--------|---------|
| `arx_d_can.api` | `ArxDCanRobot`, teleop helpers |
| `arx_d_can.hardware` | Bus, `JointGroup`, `ArxDCanArm` |
| `arx_d_can.execution` | Joint streaming + interpolation |
| `arx_d_can.kinematics` | RoboCore IK/FK (`IkSolver`, `solve_ik`) |
| `arx_d_can.utils` | Gripper + motion profiles |
