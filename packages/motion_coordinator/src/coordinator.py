# src/motion_coordinator/coordinator.py
import threading
import time
from enum import Enum
from typing import List

from robot_msgs import MotorTarget, RobotState, SystemCommand, ControlMode
from utils import limit_motor_angles, linear_interpolation

class MotionCoordinator:
    def __init__(self, motor_min_limits: List[float], motor_max_limits: List[float], publish_callback=None, send_motor_callback=None):
        self.mode = ControlMode.IDLE
        self.motor_min_limits = motor_min_limits
        self.motor_max_limits = motor_max_limits
        self.current_motor_angles = [0.0] * len(motor_min_limits)
        self.publish_callback = publish_callback
        self.send_motor_callback = send_motor_callback
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

    def set_mode(self, mode: ControlMode):
        with self.lock:
            print(f"[Coordinator] 切换模式: {mode}")
            self.mode = mode

    def handle_motor_target(self, motor_target: MotorTarget):
        with self.lock:
            if self.mode in (ControlMode.IDLE, ControlMode.FAULT, ControlMode.ESTOP):
                print(f"[Coordinator] 当前模式 {self.mode}，拒绝运动")
                return

            # 限幅处理
            limited_angles = limit_motor_angles(motor_target.motor_angles_rad, self.motor_min_limits, self.motor_max_limits)

            # 插值 (简单分步)
            steps = max(1, int(motor_target.move_time_ms / 50))  # 50ms 步长
            interpolated = linear_interpolation(self.current_motor_angles, limited_angles, steps)

            # 按插值顺序下发
            for angles in interpolated:
                self.current_motor_angles = angles
                if self.send_motor_callback:
                    mt = MotorTarget(motor_angles_rad=angles, move_time_ms=int(motor_target.move_time_ms/steps))
                    self.send_motor_callback(mt)
                time.sleep(0.05)  # 50ms 控制周期

            # 发布系统状态
            if self.publish_callback:
                state = RobotState(mode=self.mode, motor_angles_rad=self.current_motor_angles)
                self.publish_callback(state)
