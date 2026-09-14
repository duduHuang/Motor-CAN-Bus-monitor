# protocol/mit_command.py
"""
MIT Command Module
定義 MIT 模式下的控制指令 (MITCommand) 與回傳數據 (MITTelemetry) 的資料結構，以及對應的編解碼器 (MITCommandEncoder, MITTelemetryDecoder)。
"""
from dataclasses import dataclass

@dataclass
class MITConfig:
    """MIT 模式物理量界限設定檔 (預設完全符合 Table 1 規範)"""
    p_min: float = -12.566    # rad
    p_max: float = 12.566     # rad
    v_min: float = -45.0      # rad/s
    v_max: float = 45.0       # rad/s
    kp_min: float = 0.0
    kp_max: float = 500.0     # 若驅動器韌體為 0~1000 可彈性修改
    kd_min: float = 0.0
    kd_max: float = 5.0       # 若驅動器韌體為 0~50 可彈性修改
    t_min: float = -18.0      # Nm (對應 -最大扭矩)
    t_max: float = 18.0       # Nm (對應 +最大扭矩)

@dataclass
class MITCommand:
    """MIT 模式下發控制參數"""
    p_des: float  # 期望位置 (rad)
    v_des: float  # 期望速度 (rad/s)
    kp: float     # 位置剛度
    kd: float     # 速度阻尼
    t_ff: float   # 前饋力矩 (Nm)

@dataclass
class MITTelemetry:
    """MIT 模式馬達回傳實體物理量 (RX)"""
    device_can_id: int   # DATA[0]: 設備 CAN 地址號 (0 ~ 255)
    position_rad: float  # 當前位置 p (rad)
    velocity_rads: float # 當前速度 v (rad/s)
    torque_nm: float     # 當前力矩 t (Nm)

class MITCommandEncoder:
    """MIT 模式 8-Byte Bit-Packing 編解碼器 (TX)"""

    @staticmethod
    def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
        """將浮點數線性映射並 Clamp 至指定 Bit 數量的無符號整數"""
        clamped_x = max(min(x, x_max), x_min)
        span = x_max - x_min
        offset = x_min
        max_int = (1 << bits) - 1
        return int((clamped_x - offset) * max_int / span)

    @classmethod
    def encode(cls, cmd: MITCommand, cfg: MITConfig = MITConfig()) -> bytes:
        """
        將 MIT 控制參數按 Table 2 規範壓碼為 8-byte CAN Payload
        """
        p_int  = cls.float_to_uint(cmd.p_des, cfg.p_min, cfg.p_max, 16)
        v_int  = cls.float_to_uint(cmd.v_des, cfg.v_min, cfg.v_max, 12)
        kp_int = cls.float_to_uint(cmd.kp,    cfg.kp_min, cfg.kp_max, 12)
        kd_int = cls.float_to_uint(cmd.kd,    cfg.kd_min, cfg.kd_max, 12)
        t_int  = cls.float_to_uint(cmd.t_ff,  cfg.t_min,  cfg.t_max,  12)

        payload = bytearray(8)
        
        # Byte 0: p_des [15..8]
        payload[0] = (p_int >> 8) & 0xFF
        # Byte 1: p_des [7..0]
        payload[1] = p_int & 0xFF
        # Byte 2: v_des [11..4]
        payload[2] = (v_int >> 4) & 0xFF
        # Byte 3: v_des [3..0] (bit 7~4) | kp [11..8] (bit 3~0)
        payload[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
        # Byte 4: kp [7..0]
        payload[4] = kp_int & 0xFF
        # Byte 5: kd [11..4]
        payload[5] = (kd_int >> 4) & 0xFF
        # Byte 6: kd [3..0] (bit 7~4) | t_ff [11..8] (bit 3~0)
        payload[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
        # Byte 7: t_ff [7..0]
        payload[7] = t_int & 0xFF

        return bytes(payload)

    @classmethod
    def decode(cls, payload: bytes, cfg: MITConfig = MITConfig()) -> MITCommand:
        """將上位機下發的 8-byte MIT CAN Payload 解碼還原為 MITCommand 物件"""
        if len(payload) != 8:
            raise ValueError(f"Payload 長度錯誤: 期望 8 Bytes, 收到 {len(payload)} Bytes")

        p_int  = (payload[0] << 8) | payload[1]
        v_int  = (payload[2] << 4) | (payload[3] >> 4)
        kp_int = ((payload[3] & 0x0F) << 8) | payload[4]
        kd_int = (payload[5] << 4) | (payload[6] >> 4)
        t_int  = ((payload[6] & 0x0F) << 8) | payload[7]

        p_des = MITTelemetryDecoder.uint_to_float(p_int, cfg.p_min, cfg.p_max, 16)
        v_des = MITTelemetryDecoder.uint_to_float(v_int, cfg.v_min, cfg.v_max, 12)
        kp    = MITTelemetryDecoder.uint_to_float(kp_int, cfg.kp_min, cfg.kp_max, 12)
        kd    = MITTelemetryDecoder.uint_to_float(kd_int, cfg.kd_min, cfg.kd_max, 12)
        t_ff  = MITTelemetryDecoder.uint_to_float(t_int, cfg.t_min, cfg.t_max, 12)

        return MITCommand(
            p_des = round(p_des, 4),
            v_des = round(v_des, 4),
            kp = round(kp, 2),
            kd = round(kd, 2),
            t_ff = round(t_ff, 3)
        )

class MITTelemetryDecoder:
    """MIT 模式回覆數據解碼器 (RX)"""

    @staticmethod
    def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
        """將壓碼後的整數還原為實體物理量浮點數"""
        span = x_max - x_min
        offset = x_min
        max_int = (1 << bits) - 1
        return float(x_int) * span / float(max_int) + offset


    @classmethod
    def decode(cls, payload: bytes, cfg: MITConfig = MITConfig()) -> MITCommand:
        """
        解析 MIT 模式回傳的 8-byte Payload:
        DATA[0]: CAN ID
        DATA[1..2]: Position (16-bit)
        DATA[3..4]: Velocity (12-bit)
        DATA[4..5]: Torque (12-bit)
        DATA[6..7]: Fixed 0x00
        """
        if len(payload) != 8:
            raise ValueError(f"Payload 長度錯誤: 期望 8 Bytes, 收到 {len(payload)} Bytes")

        can_id = payload[0]
        
        # 16-bit Position: DATA[1] (high 8) + DATA[2] (low 8)
        p_int = (payload[1] << 8) | payload[2]
        
        # 12-bit Velocity: DATA[3] (high 8) + DATA[4] >> 4 (low 4)
        v_int = (payload[3] << 4) | (payload[4] >> 4)
        
        # 12-bit Torque: DATA[4] & 0x0F (high 4) + DATA[5] (low 8)
        t_int = ((payload[4] & 0x0F) << 8) | payload[5]

        # 數值還原為物理量
        pos = cls.uint_to_float(p_int, cfg.p_min, cfg.p_max, 16)
        vel = cls.uint_to_float(v_int, cfg.v_min, cfg.v_max, 12)
        torque = cls.uint_to_float(t_int, cfg.t_min, cfg.t_max, 12)

        return MITTelemetry(
            device_can_id = can_id,
            position_rad = round(pos, 4),
            velocity_rads = round(vel, 4),
            torque_nm = round(torque, 3)
        )