from dataclasses import dataclass
from typing import Union
from .base_decoder import BaseDecoder

@dataclass
class StandardMotionTelemetry:
    """0x9C, 0xA1, 0xA2, 0xA4, 0xA8, 0xA9, 0x72, 0x73 反饋"""
    cmd_echo: int
    temperature_c: int       # 電機內部溫度 (°C)
    iq_current_amp: float    # 轉矩電流 Iq (A)
    speed_dps: float         # 輸出軸轉速 (deg/s)
    angle_deg: int           # 電機輸出軸角度 (deg, ±32767°)

@dataclass
class SingleTurnMotionTelemetry:
    """0xA6 單圈位置控制反饋"""
    cmd_echo: int
    temperature_c: int       # 電機內部溫度 (°C)
    iq_current_amp: float    # 轉矩電流 Iq (A)
    speed_dps: float         # 輸出軸轉速 (deg/s)
    encoder_raw: int         # 電機編碼器位置 (脈衝數)

class RealtimeMotionDecoder(BaseDecoder):
    HANDLED_COMMANDS = {0x9C, 0xA1, 0xA2, 0xA4, 0xA6, 0xA8, 0xA9, 0x72, 0x73}

    @classmethod
    def decode(cls, payload: bytes) -> Union[StandardMotionTelemetry, SingleTurnMotionTelemetry]:
        cmd_echo = payload[0]

        if cmd_echo == 0xA6:
            _cmd, temp, raw_iq, raw_spd, raw_enc = cls._unpack("BbhhH", payload)
            return SingleTurnMotionTelemetry(
                cmd_echo = _cmd,
                temperature_c = temp,
                iq_current_amp = round(raw_iq * 0.01, 2),
                speed_dps = round(raw_spd * 1.0, 1),
                encoder_raw = raw_enc
            )
        else:
            _cmd, temp, raw_iq, raw_spd, raw_ang = cls._unpack("Bbhhh", payload)
            return StandardMotionTelemetry(
                cmd_echo = _cmd,
                temperature_c = temp,
                iq_current_amp = round(raw_iq * 0.01, 2),
                speed_dps = round(raw_spd * 1.0, 1),
                angle_deg = raw_ang
            )