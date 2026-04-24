from motion_coordinator.coordinator import MotionCoordinator
from robot_msgs import ControlMode, MotorTarget
import coordinator as coordinator_module


def test_motion_coordinator_handles_target_and_applies_limits(monkeypatch):
    sent_motor_msgs = []
    published_states = []

    # Avoid real waiting in interpolation loop to keep tests fast.
    monkeypatch.setattr(coordinator_module.time, "sleep", lambda *_: None)

    mc = MotionCoordinator(
        motor_min_limits=[-0.3, -0.3, -0.3, -0.3],
        motor_max_limits=[0.3, 0.3, 0.3, 0.3],
        publish_callback=published_states.append,
        send_motor_callback=sent_motor_msgs.append,
    )

    mc.set_mode(ControlMode.POSITION)
    motor_msg = MotorTarget(motor_angles_rad=[0.2, -0.2, 0.5, -0.5], move_time_ms=50)
    mc.handle_motor_target(motor_msg)

    assert sent_motor_msgs, "Expected at least one interpolated motor command"
    assert all(-0.3 <= angle <= 0.3 for angle in mc.current_motor_angles)
    assert mc.current_motor_angles == [0.2, -0.2, 0.3, -0.3]
    assert published_states, "Expected coordinator to publish RobotState"
