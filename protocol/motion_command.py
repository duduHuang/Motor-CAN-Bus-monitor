# protocol/motion_command.py
"""
MotionCommandEncoder Module
定義單軸運動與控制指令的封包編碼器 (MotionCommandEncoder)，提供外部 (如 AxisPanelView) 主動生成對應的 CAN 封包的介面。
"""
from .base_command import BaseCommandEncoder

class MotionCommandEncoder(BaseCommandEncoder):
    """Sheet 3: 電機運動與控制指令 (0x80 ~ 0xA9, 0x72, 0x73)"""

    TORQUE_SCALE = 0.01  # 0.01 A / LSB
    SPEED_SCALE  = 0.01  # 0.01 dps / LSB
    POS_SCALE    = 0.01  # 0.01 deg / LSB

    @classmethod
    def motor_shutdown(cls) -> bytes:
        """0x80: 電機關閉 (Shutdown, 進入無閉環自由狀態)"""
        return cls._pack_bytes(0x80)

    @classmethod
    def motor_stop(cls) -> bytes:
        """0x81: 電機停止 (Stop, 速度減至 0 並鎖死)"""
        return cls._pack_bytes(0x81)

    @classmethod
    def torque_control(cls, target_iq_amp: float) -> bytes:
        """0xA1: 轉矩閉環控制 (0.01A/LSB, int16_t 寫入 DATA[4..5])"""
        raw_iq = int(target_iq_amp / cls.TORQUE_SCALE)
        return cls._pack_bytes(0xA1, (4, 'h', raw_iq))

    @classmethod
    def speed_control(cls, target_spd_dps: float, max_torque_pct: int = 100) -> bytes:
        """0xA2: 速度閉環控制 (0.01dps/LSB, int32_t 寫入 DATA[4..7])"""
        raw_spd = int(target_spd_dps / cls.SPEED_SCALE)
        return cls._pack_bytes(0xA2, (1, 'B', max_torque_pct & 0xFF), (4, 'i', raw_spd))

    @classmethod
    def position_control_abs(cls, target_pos_deg: float, max_spd_dps: int = 360) -> bytes:
        """0xA4: 絕對位置閉環控制 (1dps/LSB, uint16_t 限速; 0.01deg/LSB, int32_t 位置)"""
        raw_pos = int(target_pos_deg / cls.POS_SCALE)
        return cls._pack_bytes(0xA4, (2, 'H', max_spd_dps), (4, 'i', raw_pos))

    @classmethod
    def position_control_single_turn(cls, target_deg: float, max_spd_dps: int = 360, spin_dir: int = 0) -> bytes:
        """0xA6: 直驅單圈位置控制 (spin_dir: 0x00 CW, 0x01 CCW; target_deg: 0~359.99)"""
        raw_pos = int(target_deg / cls.POS_SCALE) & 0xFFFF
        return cls._pack_bytes(0xA6, (1, 'B', spin_dir), (2, 'H', max_spd_dps), (4, 'H', raw_pos))

    @classmethod
    def position_control_inc(cls, inc_pos_deg: float, max_spd_dps: int = 360) -> bytes:
        """0xA8: 增量位置閉環控制 (以目前位置為起點)"""
        raw_inc = int(inc_pos_deg / cls.POS_SCALE)
        return cls._pack_bytes(0xA8, (2, 'H', max_spd_dps), (4, 'i', raw_inc))

    @classmethod
    def position_control_torque_limit(cls, target_pos_deg: float, max_torque_pct: int, max_spd_dps: int) -> bytes:
        """0xA9: 力控位置閉環控制 (限制最大扭矩與最大速度)"""
        raw_pos = int(target_pos_deg / cls.POS_SCALE)
        return cls._pack_bytes(0xA9, (1, 'B', max_torque_pct), (2, 'H', max_spd_dps), (4, 'i', raw_pos))

    @classmethod
    def position_control_sf(cls, target_pos_deg: float, ff_speed_pct: int, max_spd_dps: int) -> bytes:
        """0x72: SF 前饋速度位置控制 (ff_speed_pct: -128~127 額定速度百分比)"""
        raw_pos = int(target_pos_deg / cls.POS_SCALE)
        return cls._pack_bytes(0x72, (1, 'b', ff_speed_pct), (2, 'H', max_spd_dps), (4, 'i', raw_pos))

    @classmethod
    def position_control_tf(cls, target_pos_deg: float, ff_torque_pct: int, max_spd_dps: int) -> bytes:
        """0x73: TF 前饋扭矩位置控制 (ff_torque_pct: -128~127 額定扭矩百分比)"""
        raw_pos = int(target_pos_deg / cls.POS_SCALE)
        return cls._pack_bytes(0x73, (1, 'b', ff_torque_pct), (2, 'H', max_spd_dps), (4, 'i', raw_pos))