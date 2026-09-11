from .base_command import BaseCommandEncoder

class SystemCommandEncoder(BaseCommandEncoder):
    """Sheet 4: 系統與其他配置指令 (0x70 ~ 0x20)"""

    @classmethod
    def get_system_mode(cls) -> bytes:
        """0x70: 讀取當前系統運行模式 (如當前處於位置環/速度環等)"""
        return cls._pack_bytes(0x70)

    @classmethod
    def system_reset(cls) -> bytes:
        """0x76: 系統復位 (微控制器重啟)"""
        return cls._pack_bytes(0x76)

    @classmethod
    def release_brake(cls) -> bytes:
        """0x77: 系統抱閘釋放 (鬆開煞車器)"""
        return cls._pack_bytes(0x77)

    @classmethod
    def lock_brake(cls) -> bytes:
        """0x78: 系統抱閘鎖死 (鎖定煞車器)"""
        return cls._pack_bytes(0x78)

    @classmethod
    def set_active_response(cls, target_cmd: int, enable: bool, interval_10ms: int) -> bytes:
        """0xB6: 設置主動回覆功能 (interval_10ms: 回覆間隔，單位 10ms)"""
        return cls._pack_bytes(0xB6, (1, 'B', target_cmd), (2, 'B', int(enable)), (3, 'H', interval_10ms))

    @classmethod
    def composite_function_control(cls, index: int, value: int) -> bytes:
        """0x20: 複合功能控制 (index 0x01: 清除多圈角度零點, 0x02: 開關 CAN 濾波器)"""
        return cls._pack_bytes(0x20, (1, 'B', index), (4, 'i', value))