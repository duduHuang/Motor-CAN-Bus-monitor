# trajectory/stand_hold_provider.py
import math
from typing import List, Dict, Any
from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint

class StandHoldTrajectoryProvider(BaseTrajectoryProvider):
    """
    基於 12 顆馬達實測絕對角度 (Degrees) 精確調校之安全站立 Provider。
    具備雙重硬體極限安全 Clamping 保護，防衝撞與機構毀損。
    """

    # 12 顆馬達獨立實體角度配置表 (單位：角度 Degree)
    # min: 實體極限小, max: 實體極限大, stand: 站立目標角度
    MOTOR_CALIBRATION_DEG: Dict[str, Dict[int, Dict[str, Any]]] = {
        "can1": { # 右後肢 RH
            1: {"name": "RH_HipX", "min": -62.0,  "max": -10.0,  "stand": -36.0}, # +往外, -往內 (取中間值)
            2: {"name": "RH_HipY", "min": -380.0, "max": 160.0,  "stand": 67.0},# +往前, -往後
            3: {"name": "RH_Knee", "min": -100.0, "max": 21.0,   "stand": -97.0}, # -往下伸展
        },
        "can2": { # 左後肢 LH
            1: {"name": "LH_HipX", "min": -130.0, "max": -77.0,  "stand": -103.5},# +往內, -往外 (取中間值)
            2: {"name": "LH_HipY", "min": 29.0,   "max": 545.0,  "stand": 117.0}, # +往後, -往前
            3: {"name": "LH_Knee", "min": -43.0,  "max": 80.0,   "stand": 81.0},  # +往下伸展
        },
        "can3": { # 右前肢 RF
            1: {"name": "RF_HipX", "min": 72.0,   "max": 123.0,  "stand": 97.5},  # +往內, -往外 (取中間值)
            2: {"name": "RF_HipY", "min": -210.0, "max": 325.0,  "stand": 182.0},  # +往前, -往後
            3: {"name": "RF_Knee", "min": -96.0, "max": 21.0,   "stand": -90.0}, # -往下伸展
        },
        "can4": { # 左前肢 LF
            1: {"name": "LF_HipX", "min": -1.0,   "max": 48.0,   "stand": 23.5},  # +往外, -往內 (取中間值)
            2: {"name": "LF_HipY", "min": -140.0, "max": 370.0,  "stand": -9.0}, # +往後, -往前
            3: {"name": "LF_Knee", "min": 19.0,   "max": 140.0,  "stand": 139.0}, # +往下伸展
        }
    }

    MARGIN_DEG = 3.0           # 硬體邊界安全緩衝區 (3 度)
    MAX_DEG_PER_SEC = 40.0      # 【防暴衝門檻】每秒目標指令最多只許改變 40 度

    def __init__(self, channel: str = "can1", motor_id: int = 1, up_ramp_sec: float = 10.0, kp: float = 20.0, kd: float = 1.5):
        super().__init__()
        self.channel = channel.lower()
        self.motor_id = motor_id
        self.up_ramp_sec = up_ramp_sec
        self.kp = kp
        self.kd = kd

        self.start_pos_rad = None
        self.last_p_des_rad = None
        
        ch_cfg = self.MOTOR_CALIBRATION_DEG.get(self.channel, self.MOTOR_CALIBRATION_DEG["can1"])
        m_cfg = ch_cfg.get(self.motor_id, ch_cfg[1])
        self.motor_name = m_cfg["name"]

        self.min_safe_rad = math.radians(m_cfg["min"] + self.MARGIN_DEG)
        self.max_safe_rad = math.radians(m_cfg["max"] - self.MARGIN_DEG)
        self.stand_target_rad = math.radians(m_cfg["stand"])
        self.stand_target_rad = max(self.min_safe_rad, min(self.max_safe_rad, self.stand_target_rad))

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        return [
            ParamSchema("channel", str, "can1", "CAN Channel", "Target CAN bus channel"),
            ParamSchema("motor_id", int, 1, "Motor ID", "Joint motor ID (1:HipX, 2:HipY, 3:Knee)", min_value=1, max_value=3),
            ParamSchema("up_ramp_sec", float, 10.0, "Ramp Time (s)", "Smooth transition duration", min_value=0.1, max_value=300.0, step=1.0),
            ParamSchema("kp", float, 20.0, "Stiffness (Kp)", "MIT position gain (Safe default)", min_value=0.0, max_value=200.0, step=1.0),
            ParamSchema("kd", float, 1.5, "Damping (Kd)", "MIT velocity gain", min_value=0.0, max_value=20.0, step=0.1),
        ]

    def initialize(self, init_pos_rad: float) -> None:
        """開機對齊實體角度，並對實體初始角度做安全 Clamp 保護"""
        clamped_init = max(self.min_safe_rad, min(self.max_safe_rad, init_pos_rad))
        self.start_pos_rad = clamped_init
        self.last_p_des_rad = clamped_init
        print(f"  [INIT OK] {self.channel.upper()} M{self.motor_id} ({self.motor_name}): 讀取角度 = {math.degrees(init_pos_rad):.1f}°, 鎖定起點 = {math.degrees(clamped_init):.1f}°")

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        if self.start_pos_rad is None:
            # 尚未初始化完成前的絕對安全備援
            return TrajectoryPoint(position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque_ff=0.0)

        # 1. 計算 Lerp 平滑目標角度
        alpha = min(1.0, max(0.0, elapsed_time / self.up_ramp_sec))
        raw_p_des_rad = self.start_pos_rad + alpha * (self.stand_target_rad - self.start_pos_rad)

        # 2. 【核心防暴衝】限制單步最高變化速率 (以 100Hz 週期算，單步最大變化量)
        max_step_rad = math.radians(self.MAX_DEG_PER_SEC) / 100.0  
        p_diff = raw_p_des_rad - self.last_p_des_rad
        clamped_diff = max(-max_step_rad, min(max_step_rad, p_diff))
        
        current_p_des_rad = self.last_p_des_rad + clamped_diff
        self.last_p_des_rad = current_p_des_rad

        # 3. 最終硬體極限安全防護牆
        safe_p_des_rad = max(self.min_safe_rad, min(self.max_safe_rad, current_p_des_rad))

        return TrajectoryPoint(
            position=safe_p_des_rad,
            velocity=0.0,
            kp=self.kp,
            kd=self.kd,
            torque_ff=0.0
        )