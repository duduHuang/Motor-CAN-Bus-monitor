import struct
from abc import ABC

class BaseCommandEncoder(ABC):
    """單機控制指令基類：處理 8-Byte Payload 填充與格式化"""
    ENDIAN = '<'
    PAYLOAD_SIZE = 8

    @classmethod
    def _pack_bytes(cls, cmd_code: int, *placements: tuple[int, str, any]) -> bytes:
        """
        通用 Payload 構建器
        :param cmd_code: Byte 0 指令碼 (uint8)
        :param placements: 元組清單 (offset, struct_fmt, value)
        """
        payload = bytearray(cls.PAYLOAD_SIZE)
        payload[0] = cmd_code & 0xFF
        
        for offset, fmt, val in placements:
            packed = struct.pack(f"{cls.ENDIAN}{fmt}", val)
            payload[offset:offset + len(packed)] = packed

        return bytes(payload)