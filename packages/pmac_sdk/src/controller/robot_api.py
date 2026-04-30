import time
from dataclasses import dataclass
from typing import List

from comms.modbus_client import PMACModbusClient


# =========================
# Modbus register map
# int32 address = PMAC Modbus offset / 2
# =========================
REG_TARGET_POS = 0       # 0,2,4,6,8
REG_MOVE_TIME = 20       # PMAC Decode(40)
REG_ACCEL_TIME = 22      # PMAC Decode(44)
REG_SCURVE_TIME = 24     # PMAC Decode(48)

REG_ACTUAL_POS = 10      # PMAC Encode(...,20/24/28/32/36)

REG_READY = 30           # P900, Encode(60)
REG_FAULT = 32           # P901, Encode(64)
REG_INITIALIZED = 34     # P902, Encode(68)
REG_ERROR_CODE = 36      # P904, Encode(72)
REG_ENABLED = 38         # P903, Encode(76)
REG_LAST_COMMAND = 40    # P905, Encode(80)
REG_COMMAND_ACK = 42     # P122, Encode(84)
REG_MOVE_ACK = 44        # P125, Encode(88)

REG_STATUSWORD = 50      # Encode 100,104,108,112,116

REG_COMMAND = 120        # P120, Decode(240)
REG_COMMAND_SEQ = 122    # P121, Decode(244)


CMD_NONE = 0
CMD_RESET_FAULT = 1
CMD_SET_CSP = 2
CMD_ENABLE_OPERATION = 3
CMD_HOLD_POSITION = 4
CMD_MOVE = 5
CMD_STOP = 6
CMD_DISABLE_OPERATION = 7


@dataclass
class PMACStatus:
    ready: int
    fault: int
    initialized: int
    enabled: int
    error_code: int
    last_command: int
    command_ack: int
    move_ack: int

    @property
    def ok_for_motion(self) -> bool:
        return (
            self.ready == 1
            and self.fault == 0
            and self.initialized == 1
            and self.enabled == 1
        )


class PMACRobotController:
    def __init__(self, host: str = "192.168.0.200", port: int = 502, unit_id: int = 1):
        self.modbus = PMACModbusClient(host=host, port=port, unit_id=unit_id,word_order="low_high")
        self.command_seq = 0

    def connect(self) -> None:
        if not self.modbus.connect():
            raise ConnectionError("Failed to connect PMAC Modbus TCP.")

        # 从 PMAC 当前 ack 继续递增，避免重启 Python 后 seq 重复
        try:
            self.command_seq = self.modbus.read_int32(REG_COMMAND_ACK)
        except Exception:
            self.command_seq = 0

    def close(self) -> None:
        self.modbus.close()

    def read_status(self) -> PMACStatus:
        return PMACStatus(
            ready=self.modbus.read_int32(REG_READY),
            fault=self.modbus.read_int32(REG_FAULT),
            initialized=self.modbus.read_int32(REG_INITIALIZED),
            error_code=self.modbus.read_int32(REG_ERROR_CODE),
            enabled=self.modbus.read_int32(REG_ENABLED),
            last_command=self.modbus.read_int32(REG_LAST_COMMAND),
            command_ack=self.modbus.read_int32(REG_COMMAND_ACK),
            move_ack=self.modbus.read_int32(REG_MOVE_ACK),
        )

    def read_actual_positions(self) -> List[int]:
        return self.modbus.read_int32_array(REG_ACTUAL_POS, 5)

    def read_statuswords(self) -> List[int]:
        return self.modbus.read_int32_array(REG_STATUSWORD, 5)

    def send_command(self, command: int, timeout: float = 3.0) -> PMACStatus:
        self.command_seq += 1

        self.modbus.write_int32(REG_COMMAND, int(command))
        self.modbus.write_int32(REG_COMMAND_SEQ, int(self.command_seq))

        self._wait_command_ack(self.command_seq, timeout=timeout)

        status = self.read_status()
        if status.error_code != 0 and command not in (CMD_STOP, CMD_DISABLE_OPERATION):
            raise RuntimeError(f"PMAC command failed: cmd={command}, status={status}")

        return status

    def _wait_command_ack(self, seq: int, timeout: float = 3.0) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            ack = self.modbus.read_int32(REG_COMMAND_ACK)
            if ack == seq:
                return
            time.sleep(0.02)
        raise TimeoutError(f"PMAC command ack timeout: seq={seq}")

    def reset_fault(self) -> PMACStatus:
        return self.send_command(CMD_RESET_FAULT, timeout=3.0)

    def set_csp_mode(self) -> PMACStatus:
        return self.send_command(CMD_SET_CSP, timeout=2.0)

    def enable_operation(self) -> PMACStatus:
        return self.send_command(CMD_ENABLE_OPERATION, timeout=3.0)

    def hold_current_position(self) -> PMACStatus:
        return self.send_command(CMD_HOLD_POSITION, timeout=2.0)

    def stop(self) -> PMACStatus:
        return self.send_command(CMD_STOP, timeout=2.0)

    def disable_operation(self) -> PMACStatus:
        return self.send_command(CMD_DISABLE_OPERATION, timeout=2.0)

    def startup_sequence(self) -> PMACStatus:
        self.reset_fault()
        time.sleep(0.3)

        self.set_csp_mode()
        time.sleep(0.2)

        self.enable_operation()
        time.sleep(0.3)

        self.hold_current_position()
        time.sleep(0.2)

        status = self.read_status()
        if not status.ok_for_motion:
            raise RuntimeError(f"PMAC not ready after startup: {status}")

        return status

    def move_joints(
        self,
        target_positions: List[int],
        move_time: int = 500,
        accel_time: int = 100,
        scurve_time: int = 50,
    ) -> PMACStatus:
        if len(target_positions) != 5:
            raise ValueError("target_positions must contain 5 joint positions.")

        status = self.read_status()
        if not status.ok_for_motion:
            raise RuntimeError(f"PMAC is not ready for motion: {status}")

        self.modbus.write_int32_array(REG_TARGET_POS, [int(x) for x in target_positions])
        self.modbus.write_int32(REG_MOVE_TIME, int(move_time))
        self.modbus.write_int32(REG_ACCEL_TIME, int(accel_time))
        self.modbus.write_int32(REG_SCURVE_TIME, int(scurve_time))

        # 新架构：只发送 CMD_MOVE，不再写 P123/P124
        return self.send_command(CMD_MOVE, timeout=3.0)

    def move_relative(
        self,
        delta_positions: List[int],
        move_time: int = 500,
        accel_time: int = 100,
        scurve_time: int = 50,
    ) -> PMACStatus:
        if len(delta_positions) != 5:
            raise ValueError("delta_positions must contain 5 joint positions.")

        current = self.read_actual_positions()
        target = [current[i] + int(delta_positions[i]) for i in range(5)]
        return self.move_joints(target, move_time, accel_time, scurve_time)

    def print_status(self) -> None:
        status = self.read_status()
        positions = self.read_actual_positions()
        statuswords = self.read_statuswords()

        print("PMAC status:")
        print(f"  ready       : {status.ready}")
        print(f"  fault       : {status.fault}")
        print(f"  initialized : {status.initialized}")
        print(f"  enabled     : {status.enabled}")
        print(f"  error_code  : {status.error_code}")
        print(f"  last_command: {status.last_command}")
        print(f"  command_ack : {status.command_ack}")
        print(f"  move_ack    : {status.move_ack}")
        print(f"  positions   : {positions}")
        print(f"  statuswords : {statuswords}")
