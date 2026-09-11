from .base_command import BaseCommandEncoder

class ParamCommandEncoder(BaseCommandEncoder):
    """Sheet 1: 參數讀取與寫入指令 (0x30 ~ 0xB4)"""

    @classmethod
    def read_pid(cls, index: int) -> bytes:
        """0x30: 讀取 PID 參數 (index: 0x01 KP_i, 0x02 KI_i, 0x04 KP_v, 0x05 KI_v, 0x07 KP_p, 0x08 KI_p, 0x09 KD_p)"""
        return cls._pack_bytes(0x30, (1, 'B', index))

    @classmethod
    def write_pid_ram(cls, index: int, value: float) -> bytes:
        """0x31: 寫入 PID 參數到 RAM (float 寫入 DATA[4..7])"""
        return cls._pack_bytes(0x31, (1, 'B', index), (4, 'f', value))

    @classmethod
    def write_pid_rom(cls, index: int, value: float) -> bytes:
        """0x32: 寫入 PID 參數到 ROM"""
        return cls._pack_bytes(0x32, (1, 'B', index), (4, 'f', value))

    @classmethod
    def read_accel(cls, index: int) -> bytes:
        """0x42: 讀取加速度參數 (index: 0x00 PosAccel, 0x01 PosDecel, 0x02 SpdAccel...)"""
        return cls._pack_bytes(0x42, (1, 'B', index))

    @classmethod
    def write_accel(cls, index: int, accel_val: int) -> bytes:
        """0x43: 寫入加速度參數 (int32_t 寫入 DATA[4..7])"""
        return cls._pack_bytes(0x43, (1, 'B', index), (4, 'i', accel_val))

    @classmethod
    def read_multi_turn_offset(cls) -> bytes:
        """0x62: 讀取多圈編碼器零偏"""
        return cls._pack_bytes(0x62)

    @classmethod
    def write_multi_turn_offset(cls, offset: int) -> bytes:
        """0x63: 手動寫入多圈編碼器零偏值 (int32_t) 到 ROM"""
        return cls._pack_bytes(0x63, (4, 'i', offset))

    @classmethod
    def set_current_pos_as_zero(cls) -> bytes:
        """0x64: 將電機當前多圈編碼器值寫入 ROM 作為零點"""
        return cls._pack_bytes(0x64)

    @classmethod
    def set_comm_timeout(cls, timeout_ms: int) -> bytes:
        """0xB3: 設置通訊中斷保護時間 (ms, uint32_t, 0 為禁用)"""
        return cls._pack_bytes(0xB3, (4, 'I', timeout_ms))

    @classmethod
    def set_baudrate(cls, baudrate_code: int) -> bytes:
        """0xB4: 設置波特率 (0x00: RS485 115200 / CAN 1M; 0x01: RS485 5M / CAN 500K)"""
        return cls._pack_bytes(0xB4, (7, 'B', baudrate_code))