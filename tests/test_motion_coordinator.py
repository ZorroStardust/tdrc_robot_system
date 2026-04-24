# tests/test_motion_coordinator.py
import time
from motion_coordinator.coordinator import MotionCoordinator

def test_motion_coordinator():
    # Create an instance of MotionCoordinator
    mc = MotionCoordinator()
    mc.start()

    # Set mode to POSITION and send a motor target
    mc.set_mode("POSITION")
    motor_msg = MotorTarget(
        motor_angles_rad=[0.2, -0.2, 0.3, -0.3],
        motion_type="POSITION",
        move_time_ms=500
    )
    mc.handle_motor_target(motor_msg)
    
    # Check if the motor target has been processed and limits are applied
    assert mc.motor_target is not None
    assert all(angle <= 0.3 and angle >= -0.3 for angle in mc.motor_target.motor_angles_rad)
    print("[test_motion_coordinator] Motor target processed successfully.")
