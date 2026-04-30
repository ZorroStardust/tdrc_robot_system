# from dataclasses import dataclass, field
# from typing import List

# @dataclass
# class PMACConfig:
#     ip: str = '192.168.0.200'
#     modbus_port: int = 502
#     slave_id: int = 1
#     ssh_user: str = 'root'
#     ssh_pass: str = 'deltatau'
    
#     # 物理参数
#     encoder_resolution: int = 131072
#     gear_ratio: float = 97.34
#     zero_offsets: List[int] = field(default_factory=lambda: [0, 0, 0, 0, 0])
    
#     @property
#     def pulses_per_degree(self) -> float:
#         return (self.encoder_resolution * self.gear_ratio) / 360.0

from dataclasses import dataclass, field
from typing import List

@dataclass
class PMACConfig:
    ip: str = "192.168.0.200"
    modbus_port: int = 502
    slave_id: int = 1
    ssh_user: str = "root"
    ssh_pass: str = "deltatau"

    # Encoder / transmission
    single_turn_bits: int = 17
    multi_turn_bits: int = 16
    gear_ratio: float = 97.34

    zero_offsets: List[int] = field(default_factory=lambda: [0, 0, 0, 0, 0])

    @property
    def encoder_resolution(self) -> int:
        return 1 << self.single_turn_bits

    @property
    def encoder_total_range(self) -> int:
        return 1 << (self.single_turn_bits + self.multi_turn_bits)

    @property
    def pulses_per_output_rev(self) -> float:
        return self.encoder_resolution * self.gear_ratio

    @property
    def pulses_per_degree(self) -> float:
        return self.pulses_per_output_rev / 360.0

    def degrees_to_pulses(self, deg: float) -> int:
        return round(deg * self.pulses_per_degree)

    def pulses_to_degrees(self, pulses: int) -> float:
        return pulses / self.pulses_per_degree
