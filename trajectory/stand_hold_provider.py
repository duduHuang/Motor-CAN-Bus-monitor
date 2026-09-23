# trajectory/stand_hold_provider.py
import math
from typing import List, Dict, Any
from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint
from config_loader import ConfigManager

class StandHoldTrajectoryProvider(BaseTrajectoryProvider):
    """Prone -> Crouch -> Stand (5s) -> Crouch -> Prone 雙向平滑循環 (解耦校正配置版)"""

    MARGIN_DEG = 3.0
    MAX_DEG_PER_SEC = 90.0

    def __init__(self, channel: str = "can1", motor_id: int = 1, 
                 t_crouch: float = 0.8, t_stand: float = 1.5, t_hold: float = 5.0,
                 kp_soft: float = 20.0, kp_hard: float = 55.0, kd: float = 2.5,
                 tau_max: float = 3.0):
        super().__init__()
        self.channel = channel.lower()
        self.motor_id = motor_id
        self.t_crouch = t_crouch
        self.t_stand = t_stand
        self.t_hold = t_hold
        self.kp_soft = kp_soft
        self.kp_hard = kp_hard
        self.kd = kd
        self.tau_max = tau_max

        self.start_pos_rad = None
        self.last_p_des_rad = None
        self.last_time = 0.0

        # 動態載入硬體校正配置
        calib_db = ConfigManager().get_calibration()
        ch_cfg = calib_db.get(self.channel, calib_db.get("can1", {}))
        m_cfg = ch_cfg.get(self.motor_id, ch_cfg.get(1, {"min": -60.0, "max": 60.0, "crouch": 0.0, "stand": 0.0}))

        self.min_safe_rad = math.radians(m_cfg["min"] + self.MARGIN_DEG)
        self.max_safe_rad = math.radians(m_cfg["max"] - self.MARGIN_DEG)
        self.crouch_target_rad = math.radians(m_cfg["crouch"])
        self.stand_target_rad = math.radians(m_cfg["stand"])

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        return [
            ParamSchema("channel", str, "can1", "CAN Channel", "Target CAN bus channel"),
            ParamSchema("motor_id", int, 1, "Motor ID", "Joint motor ID", min_value=1, max_value=3),
            ParamSchema("t_crouch", float, 0.8, "Crouch Time (s)", "Time for Prone <-> Crouch"),
            ParamSchema("t_stand", float, 1.5, "Stand Time (s)", "Time for Crouch <-> Stand"),
            ParamSchema("t_hold", float, 5.0, "Stand Hold Time (s)", "Duration to hold standing posture"),
            ParamSchema("kp_hard", float, 55.0, "Standing Kp", "High stiffness for standing"),
            ParamSchema("kd", float, 2.5, "Damping (Kd)", "Velocity gain"),
            ParamSchema("tau_max", float, 3.0, "Max Torque FF (Nm)", "Max gravity feedforward torque"),
        ]

    def initialize(self, init_pos_rad: float) -> None:
        clamped_init = max(self.min_safe_rad, min(self.max_safe_rad, init_pos_rad))
        self.start_pos_rad = clamped_init
        self.last_p_des_rad = clamped_init
        self.last_time = 0.0

    def _s_curve(self, alpha: float) -> float:
        alpha_clamped = max(0.0, min(1.0, alpha))
        return (1.0 - math.cos(math.pi * alpha_clamped)) / 2.0

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        if self.start_pos_rad is None:
            return TrajectoryPoint(position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque_ff=0.0)

        dt = elapsed_time - self.last_time
        if dt <= 0: dt = 0.01
        self.last_time = elapsed_time

        t1 = self.t_crouch
        t2 = t1 + self.t_stand
        t3 = t2 + self.t_hold
        t4 = t3 + self.t_stand
        t5 = t4 + self.t_crouch

        if elapsed_time < t1:
            s = self._s_curve(elapsed_time / t1)
            target_rad = self.start_pos_rad + s * (self.crouch_target_rad - self.start_pos_rad)
            current_kp = self.kp_soft

        elif elapsed_time < t2:
            s = self._s_curve((elapsed_time - t1) / self.t_stand)
            target_rad = self.crouch_target_rad + s * (self.stand_target_rad - self.crouch_target_rad)
            current_kp = self.kp_soft + s * (self.kp_hard - self.kp_soft)

        elif elapsed_time < t3:
            target_rad = self.stand_target_rad
            current_kp = self.kp_hard

        elif elapsed_time < t4:
            s = self._s_curve((elapsed_time - t3) / self.t_stand)
            target_rad = self.stand_target_rad + s * (self.crouch_target_rad - self.stand_target_rad)
            current_kp = self.kp_hard - s * (self.kp_hard - self.kp_soft)

        elif elapsed_time < t5:
            s = self._s_curve((elapsed_time - t4) / self.t_crouch)
            target_rad = self.crouch_target_rad + s * (self.start_pos_rad - self.crouch_target_rad)
            current_kp = self.kp_soft

        else:
            target_rad = self.start_pos_rad
            current_kp = self.kp_soft

        max_step_rad = math.radians(self.MAX_DEG_PER_SEC) * dt  
        clamped_diff = max(-max_step_rad, min(max_step_rad, target_rad - self.last_p_des_rad))
        current_p_des_rad = self.last_p_des_rad + clamped_diff
        self.last_p_des_rad = current_p_des_rad

        safe_p_des_rad = max(self.min_safe_rad, min(self.max_safe_rad, current_p_des_rad))

        t_ff = 0.0
        if self.motor_id == 3:
            angle_diff = safe_p_des_rad - self.stand_target_rad
            gravity_comp = self.tau_max * math.sin(abs(angle_diff))
            direction = -1.0 if ("can1" in self.channel or "can3" in self.channel) else 1.0

            if t1 <= elapsed_time < t4:
                t_ff = direction * gravity_comp
            elif t4 <= elapsed_time < t5:
                fade_out = 1.0 - ((elapsed_time - t4) / self.t_crouch)
                t_ff = direction * gravity_comp * max(0.0, fade_out)

        return TrajectoryPoint(
            position=safe_p_des_rad,
            velocity=0.0,
            kp=current_kp,
            kd=self.kd,
            torque_ff=t_ff
        )