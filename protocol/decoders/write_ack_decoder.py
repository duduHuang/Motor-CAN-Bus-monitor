# protocol/decoders/write_ack_decoder.py
"""
WriteAckDecoder Module
解碼器模組，負責解析單軸寫入指令的確認回應封包，並將其轉換為對應的資料結構 (Telemetry)。
"""
from dataclasses import dataclass
from .base_decoder import BaseDecoder

@dataclass
class WriteAckTelemetry:
    """0x31, 0x32, 0x43, 0x63, 0xB3, 0x20, 0x77, 0x78, 0x80, 0x81 寫入確認"""
    cmd_echo: int
    is_success: bool
    raw_payload: bytes

class WriteAckDecoder(BaseDecoder):
    HANDLED_COMMANDS = {0x31, 0x32, 0x43, 0x63, 0xB3, 0x20, 0x77, 0x78, 0x80, 0x81}

    @classmethod
    def decode(cls, payload: bytes) -> WriteAckTelemetry:
        cls._validate(payload)
        return WriteAckTelemetry(
            cmd_echo = payload[0],
            is_success = True,
            raw_payload = payload
        )