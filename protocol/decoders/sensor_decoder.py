# protocol/decoders/sensor_decoder.py
"""
SensorDecoder Module
解碼器模組，負責解析單軸感測器狀態與錯誤回應封包，並將其轉換為對應的資料結構 (Telemetry)。
"""
from dataclasses import dataclass
from typing import Union
from .base_decoder import BaseDecoder

@dataclass
class ErrorStatusFlags:
    """0x9A 錯誤狀態字 (Bitmask)"""
    raw_value: int
    stall: bool                 # Bit 1 (0x0002): 堵轉
    low_voltage: bool           # Bit 2 (0x0004): 低壓
    over_voltage: bool          # Bit 3 (0x0008): 過壓
    over_current: bool          # Bit 4 (0x0010): 相電流過流
    mos_over_temp: bool         # Bit 7 (0x0080): 元器件/MOS過溫
    motor_over_temp: bool       # Bit 12 (0x1000): 電機過溫
    encoder_calib_error: bool   # Bit 13 (0x2000): 編碼器校準錯誤

    @classmethod
    def from_uint16(cls, val: int) -> "ErrorStatusFlags":
        return cls(
            raw_value = val,
            stall = bool(val & 0x0002),
            low_voltage = bool(val & 0x0004),
            over_voltage = bool(val & 0x0008),
            over_current = bool(val & 0x0010),
            mos_over_temp = bool(val & 0x0080),
            motor_over_temp = bool(val & 0x1000),
            encoder_calib_error = bool(val & 0x2000)
        )

@dataclass
class SensorStatus1Telemetry:
    """0x9A 電機狀態 1 與錯誤標誌反饋"""
    cmd_echo: int
    temperature_c: int          # 電機內部溫度 (°C)
    mos_temperature_c: int      # MOS 溫度 (°C)
    brake_released: bool        # 抱閘狀態 (True: 已釋放, False: 鎖死)
    voltage_v: float            # 直流母線電壓 (V)
    error_flags: ErrorStatusFlags

@dataclass
class SensorStatus3Telemetry:
    """0x9D 狀態 3 相電流反饋"""
    cmd_echo: int
    temperature_c: int          # 電機內部溫度 (°C)
    phase_a_amp: float          # A 相電流 (A)
    phase_b_amp: float          # B 相電流 (A)
    phase_c_amp: float          # C 相電流 (A)

class SensorDataDecoder(BaseDecoder):
    HANDLED_COMMANDS = {0x9A, 0x9D}

    @classmethod
    def decode(cls, payload: bytes) -> Union[SensorStatus1Telemetry, SensorStatus3Telemetry]:
        cmd_echo = payload[0]

        if cmd_echo == 0x9A:
            _cmd, temp, mos_temp, brake, raw_volt, raw_err = cls._unpack("BbBBHH", payload)
            return SensorStatus1Telemetry(
                cmd_echo = _cmd,
                temperature_c = temp,
                mos_temperature_c = mos_temp,
                brake_released = bool(brake == 0x01),
                voltage_v = round(raw_volt * 0.1, 1),
                error_flags = ErrorStatusFlags.from_uint16(raw_err)
            )
        else: # 0x9D
            _cmd, temp, raw_ia, raw_ib, raw_ic = cls._unpack("Bbhhh", payload)
            return SensorStatus3Telemetry(
                cmd_echo = _cmd,
                temperature_c = temp,
                phase_a_amp = round(raw_ia * 0.01, 2),
                phase_b_amp = round(raw_ib * 0.01, 2),
                phase_c_amp = round(raw_ic * 0.01, 2)
            )