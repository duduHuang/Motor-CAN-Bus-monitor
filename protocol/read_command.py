# protocol/read_command.py
"""
ReadCommandEncoder Module
單軸讀取指令模組：提供對應的封包構建函式，將指令碼與參數轉換為 8-Byte Payload。
"""
from .base_command import BaseCommandEncoder

class ReadCommandEncoder(BaseCommandEncoder):
    """Sheet 2: 電機狀態與資訊讀取指令 (0x60 ~ 0xB5)"""

    @classmethod
    def read_encoder_multi_pos(cls) -> bytes:
        """0x60: 讀取多圈編碼器位置 (扣除零偏值)"""
        return cls._pack_bytes(0x60)

    @classmethod
    def read_raw_encoder_multi_pos(cls) -> bytes:
        """0x61: 讀取編碼器原始多圈位置 (不含零偏)"""
        return cls._pack_bytes(0x61)

    @classmethod
    def read_multi_turn_angle(cls) -> bytes:
        """0x92: 讀取多圈絕對角度"""
        return cls._pack_bytes(0x92)

    @classmethod
    def read_single_turn_angle(cls) -> bytes:
        """0x94: 讀取單圈角度 (-180° ~ 180°)"""
        return cls._pack_bytes(0x94)

    @classmethod
    def read_status_1(cls) -> bytes:
        """0x9A: 讀取電機狀態 1 與錯誤標誌 (溫度、電壓、Fault)"""
        return cls._pack_bytes(0x9A)

    @classmethod
    def read_status_2(cls) -> bytes:
        """0x9C: 讀取電機狀態 2 (溫度、iq 電流、轉速、角度)"""
        return cls._pack_bytes(0x9C)

    @classmethod
    def read_status_3(cls) -> bytes:
        """0x9D: 讀取電機狀態 3 (溫度、A/B/C 相電流)"""
        return cls._pack_bytes(0x9D)

    @classmethod
    def read_system_runtime(cls) -> bytes:
        """0xB1: 讀取系統累計運行時間 (ms)"""
        return cls._pack_bytes(0xB1)

    @classmethod
    def read_software_date(cls) -> bytes:
        """0xB2: 讀取軟體版本日期 (如 20211126)"""
        return cls._pack_bytes(0xB2)

    @classmethod
    def read_motor_model(cls, char_index: int = 0x01) -> bytes:
        """0xB5: 分段讀取電機型號字元 (index=0x01 代表第 1~5 個字元)"""
        return cls._pack_bytes(0xB5, (1, 'B', 0x01), (2, 'B', char_index))