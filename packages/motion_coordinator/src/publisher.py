# src/motion_coordinator/publisher.py
import zmq
import time
from robot_msgs import RobotState, topics
from robot_msgs.serialization import dumps_topic_msg

def start_publisher(state_provider, stop_event, zmq_address="tcp://127.0.0.1:5556"):
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(zmq_address)
    print(f"[Coordinator] Publisher bound to {zmq_address}")

    while not stop_event.is_set():
        state = state_provider()
        topic, msg = dumps_topic_msg(topics.TOPIC_ROBOT_STATE, state)
        socket.send_multipart([topic, msg])
        time.sleep(0.05)  # 20Hz
