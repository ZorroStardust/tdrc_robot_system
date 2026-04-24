Unit convention:

1. Joint space angles use radians.
2. MotorTarget.motor_angles_rad uses radians.
3. PMAC target positions use encoder pulses.
4. move_time_ms uses milliseconds.
5. timestamp uses Unix time in seconds.
6. PMAC bridge is responsible for rad -> degree -> pulse conversion.
7. tdrc_model_node must not output PMAC pulses directly.
8. motion_coordinator must not call pmac_sdk directly.
