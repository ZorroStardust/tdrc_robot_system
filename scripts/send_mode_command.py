# tests/test_mode_command.py
import time
import zmq
from robot_msgs import SystemCommand, CommandType, ControlMode
from robot_msgs.serialization import dumps_topic_msg

ZMQ_PUB_ADDR = "tcp://127.0.0.1:5555"

def test_set_mode():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect(ZMQ_PUB_ADDR)
    time.sleep(0.1)  # Ensure subscriber is ready

    # Send a mode change command to coordinator
    cmd = SystemCommand(command=CommandType.SET_MODE, target_mode=ControlMode.POSITION)
    topic, payload = dumps_topic_msg("system_command", cmd)
    socket.send_multipart([topic, payload])

    print(f"[test_set_mode] Sent mode command: {cmd.command} -> {cmd.target_mode}")
    # Add any checks or assertions here to validate that mode has been set correctly
