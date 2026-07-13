from arx_d_can.hardware.bus import ArxDCan
from arx_d_can.hardware.motor_config import JointCfg


class FakeController:
    def __init__(self):
        self.added_motors = []

    def add_damiao_motor(self, motor_id, feedback_id, model):
        self.added_motors.append((motor_id, feedback_id, model))
        return object()


def make_uninitialized_arm(joint: JointCfg) -> ArxDCan:
    arm = ArxDCan.__new__(ArxDCan)
    arm._all_joints = [joint]
    arm._ctrl_map = {}
    arm._motor_map = {}
    arm._fake_controller = FakeController()
    arm._make_controller = lambda: arm._fake_controller
    return arm


def test_setup_motors_registers_damiao_motor_without_motor_brand_config():
    arm = make_uninitialized_arm(
        JointCfg(
            name="joint1",
            motor_id=1,
            feedback_id=0x11,
            model="4340P",
        )
    )

    arm._setup_motors()

    assert list(arm._motor_map) == ["joint1"]
    assert arm._fake_controller.added_motors == [(1, 0x11, "4340P")]
