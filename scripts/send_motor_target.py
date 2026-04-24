# tests/test_motor_target.py
import time
import zmq
from robot_msgs import MotorTarget, MotionType
from robot_msgs.serialization import dumps_topic_msg

ZMQ_PUB_ADDR = "tcp://127.0.0.1:5556"

def test_send_motor_target():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect(ZMQ_PUB_ADDR)
    time.sleep(0.1)  # Ensure subscriber is ready

    motor_msg = MotorTarget(
        motor_angles_rad=[0.1, -0.1, 0.2, -0.2],
        motion_type=MotionType.POSITION,
        move_time_ms=500
    )

    topic, payload = dumps_topic_msg("motor_target", motor_msg)
    socket.send_multipart([topic, payload])

    print(f"[test_send_motor_target] Sent MotorTarget: {motor_msg.motor_angles_rad}")
    # Add any checks or assertions here to validate the motor target was received correctly
