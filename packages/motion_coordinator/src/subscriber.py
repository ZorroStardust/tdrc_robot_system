# src/motion_coordinator/subscriber.py
import zmq
import threading
import time
from robot_msgs import MotorTarget, SystemCommand, topics
from coordinator import MotionCoordinator
from robot_msgs.serialization import loads_topic_msg

def start_subscriber(coordinator: MotionCoordinator, stop_event: threading.Event, zmq_address="tcp://127.0.0.1:5555"):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(zmq_address)
    socket.setsockopt_string(zmq.SUBSCRIBE, topics.TOPIC_MOTOR_TARGET)
    socket.setsockopt_string(zmq.SUBSCRIBE, topics.TOPIC_SYSTEM_COMMAND)

    print(f"[Coordinator] Subscriber connected to {zmq_address}")
    while not stop_event.is_set():
        try:
            topic_bytes, msg_bytes = socket.recv_multipart(flags=zmq.NOBLOCK)
            topic = topic_bytes.decode("utf-8")

            if topic == topics.TOPIC_MOTOR_TARGET:
                msg = loads_topic_msg(topic_bytes, msg_bytes, MotorTarget)[1]
                coordinator.handle_motor_target(msg)
            elif topic == topics.TOPIC_SYSTEM_COMMAND:
                cmd = loads_topic_msg(topic_bytes, msg_bytes, SystemCommand)[1]
                coordinator.set_mode(cmd.target_mode)
        except zmq.Again:
            time.sleep(0.01)
