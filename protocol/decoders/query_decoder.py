from dataclasses import dataclass
from typing import Union
from .base_decoder import BaseDecoder

@dataclass
class PIDQueryTelemetry:
    """0x30 PID 參數查詢反饋"""
    cmd_echo: int
    param_index: int
    value: float

@dataclass
class AccelQueryTelemetry:
    """0x42 加速度查詢反饋"""
    cmd_echo: int
    func_index: int
    accel_dps2: int             # 1 dps/s²

@dataclass
class EncoderPosTelemetry:
    """0x60 (扣零偏) / 0x61 (原始) 編碼器位置反饋"""
    cmd_echo: int
    encoder_pos: int            # int32 脈衝位置數據

@dataclass
class ZeroOffsetTelemetry:
    """0x62 / 0x64 編碼器零偏反饋"""
    cmd_echo: int
    encoder_offset: int         # int32 零偏脈衝值

@dataclass
class AngleQueryTelemetry:
    """0x92 (多圈) / 0x94 (單圈) 角度查詢反饋"""
    cmd_echo: int
    angle_deg: float            # 0.01° / LSB

@dataclass
class SystemModeTelemetry:
    """0x70 系統運行模式反饋"""
    cmd_echo: int
    mode_code: int              # 1: 電流環, 2: 速度環, 3: 位置環
    mode_name: str

@dataclass
class SystemInfoTelemetry:
    """0xB1 (運行時間 ms) / 0xB2 (軟體日期) 反饋"""
    cmd_echo: int
    value: int                  # uint32

@dataclass
class MotorModelTelemetry:
    """0xB5 電機型號字元分段反饋"""
    cmd_echo: int
    start_index: int
    model_chars: str            # 5 個 ASCII 型號字元

class QueryDecoder(BaseDecoder):
    HANDLED_COMMANDS = {0x30, 0x42, 0x60, 0x61, 0x62, 0x64, 0x92, 0x94, 0x70, 0xB1, 0xB2, 0xB5}

    MODE_MAP = {1: "CURRENT_LOOP", 2: "SPEED_LOOP", 3: "POSITION_LOOP"}

    @classmethod
    def decode(cls, payload: bytes) -> Union[PIDQueryTelemetry, AccelQueryTelemetry, EncoderPosTelemetry, ZeroOffsetTelemetry, AngleQueryTelemetry, SystemModeTelemetry, SystemInfoTelemetry, MotorModelTelemetry]:
        cmd_echo = payload[0]

        if cmd_echo == 0x30:
            _cmd, idx, pid_val = cls._unpack("BBxxf", payload)
            return PIDQueryTelemetry(cmd_echo = _cmd, param_index = idx, value = round(pid_val, 4))

        elif cmd_echo == 0x42:
            _cmd, idx, accel = cls._unpack("BBxxi", payload)
            return AccelQueryTelemetry(cmd_echo = _cmd, func_index = idx, accel_dps2 = accel)

        elif cmd_echo in {0x60, 0x61}:
            _cmd, pos = cls._unpack("Bxxxi", payload)
            return EncoderPosTelemetry(cmd_echo = _cmd, encoder_pos = pos)

        elif cmd_echo in {0x62, 0x64}:
            _cmd, offset = cls._unpack("Bxxxi", payload)
            return ZeroOffsetTelemetry(cmd_echo = _cmd, encoder_offset = offset)

        elif cmd_echo in {0x92, 0x94}:
            _cmd, raw_angle = cls._unpack("Bxxxi", payload)
            return AngleQueryTelemetry(cmd_echo = _cmd, angle_deg = round(raw_angle * 0.01, 2))

        elif cmd_echo == 0x70:
            _cmd, mode_code = cls._unpack("BxxxxxxB", payload)
            return SystemModeTelemetry(
                cmd_echo = _cmd,
                mode_code = mode_code,
                mode_name = cls.MODE_MAP.get(mode_code, "UNKNOWN")
            )

        elif cmd_echo in {0xB1, 0xB2}:
            _cmd, val = cls._unpack("BxxxI", payload)
            return SystemInfoTelemetry(cmd_echo = _cmd, value = val)

        elif cmd_echo == 0xB5:
            _cmd, _flag, idx, raw_str = cls._unpack("BBB5s", payload)
            chars = raw_str.decode('ascii', errors='ignore').rstrip('\x00')
            return MotorModelTelemetry(cmd_echo = _cmd, start_index = idx, model_chars = chars)