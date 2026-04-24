# src/motion_coordinator/utils.py
from typing import List

def limit_motor_angles(target_angles: List[float], min_limits: List[float], max_limits: List[float]) -> List[float]:
    """裁剪电机角度在允许范围"""
    return [max(min_limits[i], min(max_limits[i], val)) for i, val in enumerate(target_angles)]

def linear_interpolation(start: List[float], end: List[float], steps: int) -> List[List[float]]:
    """线性插值生成连续点"""
    return [
        [s + (e - s) * t / steps for s, e in zip(start, end)]
        for t in range(1, steps + 1)
    ]
