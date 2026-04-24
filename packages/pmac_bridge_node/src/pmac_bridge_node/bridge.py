from robot_msgs import MotorTarget, PmacCommand
from pmac_sdk.controller.robot_api import PMACRobotController
from .utils import rad_to_pulse
import zmq
import time

class PMACBridge:
    def __init__(self, controller: PMACRobotController):
        self.controller = controller
        self.context = zmq.Context()

    def motor_target_to_pmac_command(self, motor_target: MotorTarget) -> PmacCommand:
        if self.controller is None:
            raise ValueError("PMAC controller is not initialized")

        pulses = [rad_to_pulse(a, self.controller.config.pulses_per_degree)
                  for a in motor_target.motor_angles_rad]
        # 补齐到 5 轴
        while len(pulses) < 5:
            pulses.append(self.controller.base_positions[len(pulses)])
        return PmacCommand(
            target_pulses=pulses,
            move_time_ms=motor_target.move_time_ms,
            accel=100,
            scurve=0
        )

    def execute_motor_target(self, motor_target: MotorTarget):
        if self.controller is None:
            print("[PMACBridge] Skip motor target: controller is None")
            return

        cmd = self.motor_target_to_pmac_command(motor_target)
        self.controller.move_joints(
            target_pulses=cmd.target_pulses,
            move_time=cmd.move_time_ms,
            accel=cmd.accel,
            scurve=cmd.scurve
        )

    def start_subscriber(self, stop_event=None):
        socket = self.context.socket(zmq.SUB)
        socket.connect("tcp://localhost:5555")  # ZMQ 服务器地址
        socket.setsockopt_string(zmq.SUBSCRIBE, "motor_target")
        socket.setsockopt(zmq.RCVTIMEO, 500)

        print("[PMACBridge] Subscriber started at tcp://localhost:5555")

        try:
            while not (stop_event and stop_event.is_set()):
                try:
                    message = socket.recv_pyobj()  # 接收 MotorTarget 消息
                except zmq.Again:
                    continue

                if isinstance(message, MotorTarget):
                    self.execute_motor_target(message)
        finally:
            socket.close(0)
            print("[PMACBridge] Subscriber stopped")

    def start_publisher(self, stop_event=None, interval_sec: float = 0.5):
        socket = self.context.socket(zmq.PUB)
        socket.bind("tcp://*:5556")  # 发布状态消息的端口
        print("[PMACBridge] Publisher started at tcp://*:5556")

        try:
            # 示例：定期发布状态
            while not (stop_event and stop_event.is_set()):
                state = {
                    "connected": True,
                    "enabled": True,
                    "fault": False,
                    "motor_angles": [0, 0, 0, 0, 0]
                }
                socket.send_pyobj(state)
                time.sleep(interval_sec)
        finally:
            socket.close(0)
            print("[PMACBridge] Publisher stopped")
