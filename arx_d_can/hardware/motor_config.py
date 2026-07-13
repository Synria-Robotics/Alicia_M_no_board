"""Motor and joint configuration loaded from YAML."""

from __future__ import annotations

from dataclasses import dataclass

import yaml

from ..config.defaults import resolve_hw_cfg_path


@dataclass
class JointCfg:
    name: str
    motor_id: int
    feedback_id: int
    model: str
    kp: float = 0.0
    kd: float = 0.0
    vel_kp: float = 0.0
    vel_ki: float = 0.0
    pos_kp: float = 0.0
    pos_ki: float = 0.0
    vlim: float = 0.0


def load_cfg(hw_yaml: str | None = None) -> dict:
    hw_path = resolve_hw_cfg_path(hw_yaml)

    with open(hw_path, "r") as f:
        data = yaml.safe_load(f)

    joints = []
    for joint in data.get("joints", []):
        mit = joint.get("MIT", {})
        pos_vel = joint.get("POS_VEL", {})
        joints.append(
            JointCfg(
                name=joint["name"],
                motor_id=int(joint["motor_id"]),
                feedback_id=int(joint["feedback_id"]),
                model=str(joint.get("model", "4340P")),
                kp=float(mit.get("kp", 0.0)),
                kd=float(mit.get("kd", 0.0)),
                vel_kp=float(pos_vel.get("vel_kp", 0.0)),
                vel_ki=float(pos_vel.get("vel_ki", 0.0)),
                pos_kp=float(pos_vel.get("pos_kp", 0.0)),
                pos_ki=float(pos_vel.get("pos_ki", 0.0)),
                vlim=float(pos_vel.get("vlim", 2.0)),
            )
        )

    return {
        "name": data.get("name", "ARX-D-CAN"),
        "channel": data.get("channel", "/dev/ttyACM0"),
        "baud": int(data.get("baud", 1_000_000)),
        "rate": float(data.get("rate", 500.0)),
        "groups": data.get("groups", {}),
        "joints": joints,
        "gripper_mapping": data.get("gripper_mapping", {}),
    }
