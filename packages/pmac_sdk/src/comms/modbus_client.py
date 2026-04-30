import time
import struct
import inspect
from typing import List, Optional

try:
    from pymodbus.client import ModbusTcpClient
except ImportError:
    from pymodbus.client.sync import ModbusTcpClient


def _id_kw(func, device_id):
    params = inspect.signature(func).parameters

    if "slave" in params:
        return {"slave": device_id}
    if "unit" in params:
        return {"unit": device_id}
    if "device_id" in params:
        return {"device_id": device_id}

    return {}


def write_regs(client, address, values, device_id=1):
    return client.write_registers(
        address,
        values=values,
        **_id_kw(client.write_registers, device_id),
    )


def read_regs(client, address, count, device_id=1):
    return client.read_holding_registers(
        address,
        count=count,
        **_id_kw(client.read_holding_registers, device_id),
    )



class PMACModbusClient:
    def __init__(self, 
                 host: str = "192.168.0.200", 
                 port: int = 502, 
                 unit_id: int = 1, 
                 timeout: float = 1.0,
                 word_order: str = "low_high"):
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self.word_order = word_order
        self.client: Optional[ModbusTcpClient] = None

    def connect(self) -> bool:
        self.client = ModbusTcpClient(host=self.host, port=self.port, timeout=self.timeout)
        return bool(self.client.connect())

    def close(self) -> None:
        if self.client:
            self.client.close()

    def _require_connected(self) -> None:
        if self.client is None:
            raise RuntimeError("Modbus client is not connected.")

    # @staticmethod
    # def int32_to_regs(value: int) -> List[int]:
    #     value = int(value)
    #     packed = struct.pack(">i", value)
    #     hi, lo = struct.unpack(">HH", packed)
    #     return [hi, lo]

    def int32_to_regs(self, value: int) -> List[int]:
        value = int(value) & 0xFFFFFFFF
        hi = (value >> 16) & 0xFFFF
        lo = value & 0xFFFF

        if self.word_order == "low_high":
            return [lo, hi]
        else:
            return [hi, lo]


    # @staticmethod
    # def regs_to_int32(regs: List[int]) -> int:
    #     packed = struct.pack(">HH", regs[0] & 0xFFFF, regs[1] & 0xFFFF)
    #     return struct.unpack(">i", packed)[0]

    def regs_to_int32(self, regs: List[int]) -> int:
        if len(regs) < 2:
            raise ValueError("regs must contain at least 2 registers.")

        if self.word_order == "low_high":
            lo = regs[0] & 0xFFFF
            hi = regs[1] & 0xFFFF
        else:
            hi = regs[0] & 0xFFFF
            lo = regs[1] & 0xFFFF

        value = (hi << 16) | lo

        if value & 0x80000000:
            value -= 0x100000000

        return value


    def write_int32(self, address: int, value: int) -> None:
        self._require_connected()
        regs = self.int32_to_regs(value)
        result = write_regs(self.client, address=address, values=regs, device_id=self.unit_id)
        if result.isError():
            raise RuntimeError(f"write_int32 failed: address={address}, value={value}, result={result}")

    def write_int32_array(self, address: int, values: List[int]) -> None:
        self._require_connected()
        regs: List[int] = []
        for value in values:
            regs.extend(self.int32_to_regs(value))
        result = write_regs(self.client, address=address, values=regs, device_id=self.unit_id)
        if result.isError():
            raise RuntimeError(f"write_int32_array failed: address={address}, values={values}, result={result}")

    def read_int32(self, address: int) -> int:
        self._require_connected()
        result = read_regs(self.client, address=address, count=2, device_id=self.unit_id)
        if result.isError():
            raise RuntimeError(f"read_int32 failed: address={address}, result={result}")
        return self.regs_to_int32(result.registers)

    def read_int32_array(self, address: int, count: int) -> List[int]:
        self._require_connected()
        result = read_regs(self.client, address=address, count=count * 2, device_id=self.unit_id)
        if result.isError():
            raise RuntimeError(f"read_int32_array failed: address={address}, count={count}, result={result}")
        regs = result.registers
        return [self.regs_to_int32(regs[i * 2:i * 2 + 2]) for i in range(count)]

    # def pulse_int32(self, address: int, value: int = 1, delay: float = 0.02) -> None:
    #     self.write_int32(address, 0)
    #     time.sleep(delay)
    #     self.write_int32(address, value)

    def pulse_int32(self, address: int, value: int = 1, delay: float = 0.02) -> None:
        self.write_int32(address, 0)
        time.sleep(delay)
        self.write_int32(address, value)
        time.sleep(delay)
        self.write_int32(address, 0)
