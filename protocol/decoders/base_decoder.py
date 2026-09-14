# protocol/decoders/base_decoder.py
"""
BaseDecoder Module
所有通訊解碼器的抽象基類 (基於 Command Echo 自動派發)
"""
from abc import ABC, abstractmethod
from typing import ClassVar, Dict, Type, Any
import struct

class BaseDecoder(ABC):
    """所有通訊解碼器的抽象基類 (基於 Command Echo 自動派發)"""
    
    ENDIAN: ClassVar[str] = '<'
    PAYLOAD_SIZE: ClassVar[int] = 8
    
    HANDLED_COMMANDS: ClassVar[set[int]] = set()
    _registry: ClassVar[Dict[int, Type['BaseDecoder']]] = {}

    def __init_subclass__(cls, **kwargs):
        """自動註冊子類別至全域解碼註冊表"""
        super().__init_subclass__(**kwargs)
        for cmd in cls.HANDLED_COMMANDS:
            if cmd in cls._registry:
                raise KeyError(f"指令 Echo {hex(cmd)} 重複註冊於 {cls.__name__} 與 {cls._registry[cmd].__name__}")
            cls._registry[cmd] = cls

    @classmethod
    def _validate(cls, payload: bytes) -> None:
        if len(payload) != cls.PAYLOAD_SIZE:
            raise ValueError(f"Payload 長度錯誤: 期望 {cls.PAYLOAD_SIZE} Bytes, 收到 {len(payload)} Bytes")

    @classmethod
    def _unpack(cls, fmt: str, payload: bytes) -> tuple:
        cls._validate(payload)
        return struct.unpack(f"{cls.ENDIAN}{fmt}", payload)

    @classmethod
    @abstractmethod
    def decode(cls, payload: bytes) -> Any:
        pass

    @classmethod
    def decode_any(cls, payload: bytes) -> Any:
        """全域統一解析入口：依據 Byte 0 自動派發給對應的 Decoder 子類別"""
        cls._validate(payload)
        cmd_echo = payload[0]
        
        decoder_cls = cls._registry.get(cmd_echo)
        if not decoder_cls:
            raise ValueError(f"未註冊對應指令 Echo ({hex(cmd_echo)}) 的解碼器")
            
        return decoder_cls.decode(payload)