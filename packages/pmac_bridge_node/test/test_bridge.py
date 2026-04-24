import pytest
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "packages/pmac_bridge_node/src"))
sys.path.insert(0, str(REPO_ROOT / "packages/robot_msgs/src"))
sys.path.insert(0, str(REPO_ROOT / "packages/pmac_sdk/src"))

from pmac_bridge_node.bridge import PMACBridge
from pmac_bridge_node.utils import rad_to_pulse
from robot_msgs import MotorTarget

class FakeConfig:
    def __init__(self, pulses_per_degree: float = 1000.0):
        self.pulses_per_degree = pulses_per_degree


class FakeController:
    def __init__(self):
        self.config = FakeConfig()
        self.base_positions = [10, 20, 30, 40, 50]
        self.last_move = None

    def move_joints(self, target_pulses, move_time, accel, scurve):
        self.last_move = {
            "target_pulses": target_pulses,
            "move_time": move_time,
            "accel": accel,
            "scurve": scurve,
        }


def test_motor_target_to_pmac_command_converts_and_pads_to_five_axes():
    controller = FakeController()
    bridge = PMACBridge(controller=controller)

    motor_target = MotorTarget(motor_angles_rad=[0.1, 0.2, 0.3, 0.4], move_time_ms=1000)
    cmd = bridge.motor_target_to_pmac_command(motor_target)

    expected_first_four = [
        rad_to_pulse(0.1, controller.config.pulses_per_degree),
        rad_to_pulse(0.2, controller.config.pulses_per_degree),
        rad_to_pulse(0.3, controller.config.pulses_per_degree),
        rad_to_pulse(0.4, controller.config.pulses_per_degree),
    ]
    assert cmd.target_pulses[:4] == expected_first_four
    assert cmd.target_pulses[4] == controller.base_positions[4]
    assert len(cmd.target_pulses) == 5
    assert cmd.move_time_ms == 1000
    assert cmd.accel == 100
    assert cmd.scurve == 0


def test_motor_target_to_pmac_command_raises_when_controller_is_none():
    bridge = PMACBridge(controller=None)
    motor_target = MotorTarget(motor_angles_rad=[0.1, 0.2, 0.3, 0.4], move_time_ms=500)

    with pytest.raises(ValueError, match="not initialized"):
        bridge.motor_target_to_pmac_command(motor_target)


def test_execute_motor_target_calls_controller_move_joints_with_command_values():
    controller = FakeController()
    bridge = PMACBridge(controller=controller)
    motor_target = MotorTarget(motor_angles_rad=[0.1, 0.2, 0.3, 0.4], move_time_ms=700)

    bridge.execute_motor_target(motor_target)

    assert controller.last_move is not None
    assert controller.last_move["move_time"] == 700
    assert controller.last_move["accel"] == 100
    assert controller.last_move["scurve"] == 0
    assert len(controller.last_move["target_pulses"]) == 5


def test_execute_motor_target_with_none_controller_does_not_raise():
    bridge = PMACBridge(controller=None)
    motor_target = MotorTarget(motor_angles_rad=[0.1, 0.2, 0.3, 0.4], move_time_ms=500)

    bridge.execute_motor_target(motor_target)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
