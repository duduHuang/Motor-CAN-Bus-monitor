import math
from typing import Any, Dict, List

from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint


class QuadrupedGaitProvider(BaseTrajectoryProvider):
    """階段四：對角步態 (Trot Gait) 原地踏步 Provider (膝關節獨立加強抬腿版)"""

    MOTOR_CALIBRATION_DEG: Dict[str, Dict[int, Dict[str, Any]]] = {
        "can1": {
            1: {"min": -62.0,  "max": -10.0,  "prone": -36.0,  "crouch": -36.0,  "stand": -36.0},
            2: {"min": -380.0, "max": 160.0,  "prone": 10.0,   "crouch": 40.0,   "stand": 67.0},
            3: {"min": -107.0, "max": 25.0,   "prone": 21.0,  "crouch": -39.0,  "stand": -103.0},
        },
        "can2": {
            1: {"min": -130.0, "max": -77.0,  "prone": -103.5, "crouch": -103.5, "stand": -103.5},
            2: {"min": 29.0,   "max": 545.0,  "prone": 170.0,  "crouch": 145.0,  "stand": 117.0},
            3: {"min": -46.0,  "max": 85.5,   "prone": -42.0,  "crouch": 18.0,   "stand": 81.5},
        },
        "can3": {
            1: {"min": 72.0,   "max": 123.0,  "prone": 97.5,   "crouch": 97.5,   "stand": 97.5},
            2: {"min": -210.0, "max": 325.0,  "prone": 120.0,  "crouch": 152.0,  "stand": 182.0},
            3: {"min": -104.0,  "max": 24.0,   "prone": 20.0,    "crouch": -40.0,  "stand": -100.0},
        },
        "can4": {
            1: {"min": -1.0,   "max": 48.0,   "prone": 23.5,   "crouch": 23.5,   "stand": 23.5},
            2: {"min": -140.0, "max": 370.0,  "prone": 50.0,   "crouch": 21.0,   "stand": -9.0},
            3: {"min": 16.0,   "max": 148.0,  "prone": 20.0,   "crouch": 80.0,   "stand": 144.0},
        }
    }

    MARGIN_DEG = 3.0
    MAX_DEG_PER_SEC = 360.0  # 放寬動態角速度上限，防止抬腳正弦波被削平

    def __init__(self, channel: str = "can1", motor_id: int = 1,
                 t_stand_up: float = 1.8, freq_hz: float = 1.2, 
                 knee_step_ratio: float = 0.45, hip_step_ratio: float = 0.15,
                 kp_soft: float = 20.0, kp_stand: float = 55.0, 
                 kd: float = 2.5, tau_max: float = 3.0, **kwargs):
        super().__init__()
        self.channel = channel.lower()
        self.motor_id = motor_id
        self.t_stand_up = t_stand_up      # 起身預熱時間 (1.8 秒)
        self.freq_hz = freq_hz            # 踏步頻率 (Hz)
        
        # 相容舊版 step_ratio / step_height_deg 傳參
        if "step_ratio" in kwargs:
            knee_step_ratio = float(kwargs["step_ratio"])
        elif "step_height_deg" in kwargs:
            knee_step_ratio = float(kwargs["step_height_deg"]) / 35.0

        self.knee_step_ratio = knee_step_ratio  # 膝關節抬腳比例 (預設 0.45)
        self.hip_step_ratio = hip_step_ratio    # 大腿微調比例 (預設 0.15)
        self.kp_soft = kp_soft
        self.kp_stand = kp_stand
        self.kd = kd
        self.tau_max = tau_max

        self.start_pos_rad = None
        self.last_p_des_rad = None
        self.last_time = 0.0

        # Trot 相位分組: Group A (RF, LH) -> 0.0 | Group B (LF, RH) -> 0.5
        if "can3" in self.channel or "can2" in self.channel:
            self.phase_offset = 0.0
        else:
            self.phase_offset = 0.5

        ch_cfg = self.MOTOR_CALIBRATION_DEG.get(self.channel, self.MOTOR_CALIBRATION_DEG["can1"])
        m_cfg = ch_cfg.get(self.motor_id, ch_cfg[1])

        self.min_safe_rad = math.radians(m_cfg["min"] + self.MARGIN_DEG)
        self.max_safe_rad = math.radians(m_cfg["max"] - self.MARGIN_DEG)
        self.crouch_target_rad = math.radians(m_cfg["crouch"])
        self.stand_target_rad = math.radians(m_cfg["stand"])

        # 核心：計算該關節從 Stand 到 Crouch 的收腿彎曲向量
        self.bend_vector_rad = self.crouch_target_rad - self.stand_target_rad

    @property
    def duration(self) -> float:
        return 0.0  # 無窮持續運行的動態步態

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        return [
            ParamSchema("channel", str, "can1", "CAN Channel", "Target CAN bus channel"),
            ParamSchema("motor_id", int, 1, "Motor ID", "Joint motor ID", min_value=1, max_value=3),
            ParamSchema("t_stand_up", float, 1.8, "Stand Up Time (s)", "Time to stand up before gait"),
            ParamSchema("freq_hz", float, 1.2, "Gait Frequency (Hz)", "Step frequency in Hz", min_value=0.5, max_value=3.0),
            ParamSchema("knee_step_ratio", float, 0.45, "Knee Lift Ratio", "Knee lift height ratio", min_value=0.1, max_value=0.7),
            ParamSchema("hip_step_ratio", float, 0.15, "HipY Lift Ratio", "HipY lift height ratio", min_value=0.0, max_value=0.4),
            ParamSchema("kp_stand", float, 55.0, "Stiffness Kp", "Standing stiffness", min_value=10.0, max_value=100.0),
            ParamSchema("kd", float, 2.5, "Damping Kd", "Velocity gain", min_value=0.0, max_value=10.0),
            ParamSchema("tau_max", float, 3.0, "Max Torque FF", "Feedforward torque", min_value=0.0, max_value=10.0),
        ]

    def initialize(self, init_pos: float) -> None:
        clamped_init = max(self.min_safe_rad, min(self.max_safe_rad, init_pos))
        self.start_pos_rad = clamped_init
        self.last_p_des_rad = clamped_init
        self.last_time = 0.0

    def _s_curve(self, alpha: float) -> float:
        a = max(0.0, min(1.0, alpha))
        return (1.0 - math.cos(math.pi * a)) / 2.0

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        if self.start_pos_rad is None:
            return TrajectoryPoint(position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque_ff=0.0)

        dt = elapsed_time - self.last_time
        if dt <= 0: dt = 0.01
        self.last_time = elapsed_time

        # 階段一：開機預熱站立階段 (0 ~ t_stand_up 秒，S-Curve 平滑拉升)
        if elapsed_time < self.t_stand_up:
            s = self._s_curve(elapsed_time / self.t_stand_up)
            target_rad = self.start_pos_rad + s * (self.stand_target_rad - self.start_pos_rad)
            current_kp = self.kp_soft + s * (self.kp_stand - self.kp_soft)
            
            t_ff = 0.0
            if self.motor_id == 3:
                direction = -1.0 if ("can1" in self.channel or "can3" in self.channel) else 1.0
                angle_diff = target_rad - self.stand_target_rad
                t_ff = direction * self.tau_max * math.sin(abs(angle_diff)) * s

        # 階段二：站穩後進入對角步態踏步 (elapsed_time >= t_stand_up)
        else:
            t_gait = elapsed_time - self.t_stand_up
            phi = (t_gait * self.freq_hz + self.phase_offset) % 1.0

            target_rad = self.stand_target_rad
            current_kp = self.kp_stand
            t_ff = 0.0

            if self.motor_id == 1:
                # HipX 橫向鎖定
                target_rad = self.stand_target_rad
                current_kp = self.kp_stand

            elif self.motor_id in [2, 3]:
                # HipY 使用 hip_step_ratio，Knee 使用 knee_step_ratio
                ratio = self.hip_step_ratio if self.motor_id == 2 else self.knee_step_ratio
                
                if phi < 0.5:
                    # Swing Phase (懸空抬腳): 正弦弧形抬起
                    swing_factor = math.sin(phi * 2.0 * math.pi)
                    target_rad = self.stand_target_rad + (self.bend_vector_rad * ratio * swing_factor)
                    
                    # 提高擺動相 Kp 比例至 85%，保持強勁閉環追蹤
                    current_kp = self.kp_stand * 0.85
                    
                    if self.motor_id == 3:
                        direction = -1.0 if ("can1" in self.channel or "can3" in self.channel) else 1.0
                        t_ff = direction * self.tau_max * (1.0 - swing_factor * 0.5)
                else:
                    # Stance Phase (著地踩平): 全額剛度與重力補償
                    target_rad = self.stand_target_rad
                    current_kp = self.kp_stand
                    if self.motor_id == 3:
                        direction = -1.0 if ("can1" in self.channel or "can3" in self.channel) else 1.0
                        t_ff = direction * self.tau_max

        # 動態 dt 防暴衝限制與硬體 Clamping
        max_step_rad = math.radians(self.MAX_DEG_PER_SEC) * dt
        clamped_diff = max(-max_step_rad, min(max_step_rad, target_rad - self.last_p_des_rad))
        current_p_des_rad = self.last_p_des_rad + clamped_diff
        self.last_p_des_rad = current_p_des_rad

        safe_p_des_rad = max(self.min_safe_rad, min(self.max_safe_rad, current_p_des_rad))

        return TrajectoryPoint(
            position=safe_p_des_rad,
            velocity=0.0,
            kp=current_kp,
            kd=self.kd,
            torque_ff=t_ff
        )