from pathlib import Path
import sys
import time
import zmq

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "packages/robot_msgs/src"))

from robot_msgs import MotorTarget, topics
from robot_msgs.serialization import loads_topic_msg


def main():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://127.0.0.1:5557")
    socket.setsockopt_string(zmq.SUBSCRIBE, topics.TOPIC_MOTOR_TARGET)

    print("[PMACBridge DryRun] listening on tcp://127.0.0.1:5557")

    while True:
        try:
            topic_bytes, payload = socket.recv_multipart()
            _, msg = loads_topic_msg(topic_bytes, payload, MotorTarget)
            print(
                "[PMACBridge DryRun] received MotorTarget:",
                msg.motor_angles_rad,
                "move_time_ms=",
                msg.move_time_ms,
            )
        except KeyboardInterrupt:
            print("\n[PMACBridge DryRun] stopped")
            break


if __name__ == "__main__":
    main()
