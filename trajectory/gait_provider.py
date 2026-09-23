# trajectory/gait_provider.py
import math
from typing import Any, Dict, List

from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint
from config_loader import ConfigManager


class QuadrupedGaitProvider(BaseTrajectoryProvider):
    """對角步態 (Trot Gait) Provider (解耦校正配置版)"""

    MARGIN_DEG = 3.0
    MAX_DEG_PER_SEC = 360.0

    def __init__(self, channel: str = "can1", motor_id: int = 1,
                 t_stand_up: float = 1.8, freq_hz: float = 1.2, 
                 knee_step_ratio: float = 0.45, hip_step_ratio: float = 0.15,
                 kp_soft: float = 20.0, kp_stand: float = 55.0, 
                 kd: float = 2.5, tau_max: float = 3.0, **kwargs):
        super().__init__()
        self.channel = channel.lower()
        self.motor_id = motor_id
        self.t_stand_up = t_stand_up
        self.freq_hz = freq_hz
        
        if "step_ratio" in kwargs:
            knee_step_ratio = float(kwargs["step_ratio"])
        elif "step_height_deg" in kwargs:
            knee_step_ratio = float(kwargs["step_height_deg"]) / 35.0

        self.knee_step_ratio = knee_step_ratio
        self.hip_step_ratio = hip_step_ratio
        self.kp_soft = kp_soft
        self.kp_stand = kp_stand
        self.kd = kd
        self.tau_max = tau_max

        self.start_pos_rad = None
        self.last_p_des_rad = None
        self.last_time = 0.0

        if "can3" in self.channel or "can2" in self.channel:
            self.phase_offset = 0.0
        else:
            self.phase_offset = 0.5

        # 動態載入硬體校正配置
        calib_db = ConfigManager().get_calibration()
        ch_cfg = calib_db.get(self.channel, calib_db.get("can1", {}))
        m_cfg = ch_cfg.get(self.motor_id, ch_cfg.get(1, {"min": -60.0, "max": 60.0, "crouch": 0.0, "stand": 0.0}))

        self.min_safe_rad = math.radians(m_cfg["min"] + self.MARGIN_DEG)
        self.max_safe_rad = math.radians(m_cfg["max"] - self.MARGIN_DEG)
        self.crouch_target_rad = math.radians(m_cfg["crouch"])
        self.stand_target_rad = math.radians(m_cfg["stand"])

        self.bend_vector_rad = self.crouch_target_rad - self.stand_target_rad

    @property
    def duration(self) -> float:
        return 0.0

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

        if elapsed_time < self.t_stand_up:
            s = self._s_curve(elapsed_time / self.t_stand_up)
            target_rad = self.start_pos_rad + s * (self.stand_target_rad - self.start_pos_rad)
            current_kp = self.kp_soft + s * (self.kp_stand - self.kp_soft)
            
            t_ff = 0.0
            if self.motor_id == 3:
                direction = -1.0 if ("can1" in self.channel or "can3" in self.channel) else 1.0
                angle_diff = target_rad - self.stand_target_rad
                t_ff = direction * self.tau_max * math.sin(abs(angle_diff)) * s

        else:
            t_gait = elapsed_time - self.t_stand_up
            phi = (t_gait * self.freq_hz + self.phase_offset) % 1.0

            target_rad = self.stand_target_rad
            current_kp = self.kp_stand
            t_ff = 0.0

            if self.motor_id == 1:
                target_rad = self.stand_target_rad
                current_kp = self.kp_stand

            elif self.motor_id in [2, 3]:
                ratio = self.hip_step_ratio if self.motor_id == 2 else self.knee_step_ratio
                
                if phi < 0.5:
                    swing_factor = math.sin(phi * 2.0 * math.pi)
                    target_rad = self.stand_target_rad + (self.bend_vector_rad * ratio * swing_factor)
                    current_kp = self.kp_stand * 0.85
                    
                    if self.motor_id == 3:
                        direction = -1.0 if ("can1" in self.channel or "can3" in self.channel) else 1.0
                        t_ff = direction * self.tau_max * (1.0 - swing_factor * 0.5)
                else:
                    target_rad = self.stand_target_rad
                    current_kp = self.kp_stand
                    if self.motor_id == 3:
                        direction = -1.0 if ("can1" in self.channel or "can3" in self.channel) else 1.0
                        t_ff = direction * self.tau_max

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