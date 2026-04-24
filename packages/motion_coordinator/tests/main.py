from pathlib import Path
import sys
import threading
import time
import math

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "packages/motion_coordinator/src"))
sys.path.insert(0, str(REPO_ROOT / "packages/robot_msgs/src"))

from coordinator import MotionCoordinator
from subscriber import start_subscriber
from publisher import start_publisher
from robot_msgs import MotorTarget, RobotState

from robot_msgs import topics
from robot_msgs.serialization import dumps_topic_msg

context = zmq.Context()
motor_pub = context.socket(zmq.PUB)
motor_pub.bind("tcp://127.0.0.1:5557")
time.sleep(0.2)

def main():
    stop_event = threading.Event()

    # 简单回调测试
    def send_motor_callback(mt: MotorTarget):
        topic, payload = dumps_topic_msg(topics.TOPIC_MOTOR_TARGET, mt)
        motor_pub.send_multipart([topic, payload])
        print(f"[Coordinator --> PMACBridge] {mt.motor_angles_rad}")
        #print(f"[SendMotor] {mt.motor_angles_rad}")

    def publish_state_callback(state):
        print(f"[State] Mode={state.mode}, Angles={state.motor_angles_rad}")

    mc = MotionCoordinator(motor_min_limits=[-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2], motor_max_limits=[math.pi/2, math.pi/2, math.pi/2, math.pi/2],
                           publish_callback=publish_state_callback,
                           send_motor_callback=send_motor_callback)

    subscriber_thread = threading.Thread(target=start_subscriber, args=(mc, stop_event), daemon=True)
    publisher_thread = threading.Thread(
        target=start_publisher,
        args=(lambda: RobotState(mode=mc.mode, motor_angles_rad=mc.current_motor_angles), stop_event),
        daemon=True,
    )

    subscriber_thread.start()
    publisher_thread.start()

    # 模拟测试
    try:
        for i in range(5):
            mt = MotorTarget(motor_angles_rad=[10*i, 5*i, -5*i, 2*i], move_time_ms=500)
            mc.handle_motor_target(mt)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        subscriber_thread.join(timeout=2)
        publisher_thread.join(timeout=2)
        print("[Coordinator] 停止")

if __name__ == "__main__":
    main()
