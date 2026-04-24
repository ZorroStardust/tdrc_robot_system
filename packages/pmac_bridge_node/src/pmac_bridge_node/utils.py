def rad_to_pulse(angle_rad: float, pulses_per_degree: float) -> int:
    """将弧度转换为对应 PMAC 脉冲"""
    deg = angle_rad * 180.0 / 3.141592653589793
    return int(deg * pulses_per_degree)
