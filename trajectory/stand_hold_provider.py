import math
from typing import List, Dict, Any
from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint

class StandHoldTrajectoryProvider(BaseTrajectoryProvider):
    """階段二（安全完整版）：Prone -> Crouch -> Stand (5s) -> Crouch -> Prone 雙向平滑循環"""

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
    MAX_DEG_PER_SEC = 90.0  # 速度上限 (deg/s)

    def __init__(self, channel: str = "can1", motor_id: int = 1, 
                 t_crouch: float = 0.8, t_stand: float = 1.5, t_hold: float = 5.0,
                 kp_soft: float = 20.0, kp_hard: float = 55.0, kd: float = 2.5,
                 tau_max: float = 3.0):
        super().__init__()
        self.channel = channel.lower()
        self.motor_id = motor_id
        self.t_crouch = t_crouch  # Prone <-> Crouch 時間 (0.8s)
        self.t_stand = t_stand    # Crouch <-> Stand 時間 (1.5s)
        self.t_hold = t_hold      # 站立保持時間 (5.0s)
        self.kp_soft = kp_soft    # 低蹲/趴姿柔順剛度
        self.kp_hard = kp_hard    # 站立高剛度
        self.kd = kd
        self.tau_max = tau_max    # 膝關節前饋力矩上限 (Nm)

        self.start_pos_rad = None
        self.last_p_des_rad = None
        self.last_time = 0.0

        ch_cfg = self.MOTOR_CALIBRATION_DEG.get(self.channel, self.MOTOR_CALIBRATION_DEG["can1"])
        m_cfg = ch_cfg.get(self.motor_id, ch_cfg[1])

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
        """餘弦 S 曲線插值器：保證端點速度與加速度均為 0 (Jerk=0)"""
        alpha_clamped = max(0.0, min(1.0, alpha))
        return (1.0 - math.cos(math.pi * alpha_clamped)) / 2.0

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        if self.start_pos_rad is None:
            return TrajectoryPoint(position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque_ff=0.0)

        # 動態 dt 計算
        dt = elapsed_time - self.last_time
        if dt <= 0: dt = 0.01
        self.last_time = elapsed_time

        # 5 階段時序節點定義
        t1 = self.t_crouch                               # 0.8s: 到達 Crouch
        t2 = t1 + self.t_stand                           # 2.3s: 到達 Stand
        t3 = t2 + self.t_hold                            # 7.3s: 站立保持結束
        t4 = t3 + self.t_stand                           # 8.8s: 退回 Crouch
        t5 = t4 + self.t_crouch                          # 9.6s: 退回 Prone (安全貼地)

        # 階段一：Prone -> Crouch (餘弦平滑拉升)
        if elapsed_time < t1:
            s = self._s_curve(elapsed_time / t1)
            target_rad = self.start_pos_rad + s * (self.crouch_target_rad - self.start_pos_rad)
            current_kp = self.kp_soft

        # 階段二：Crouch -> Stand (剛度與位置同步餘弦拉升)
        elif elapsed_time < t2:
            s = self._s_curve((elapsed_time - t1) / self.t_stand)
            target_rad = self.crouch_target_rad + s * (self.stand_target_rad - self.crouch_target_rad)
            current_kp = self.kp_soft + s * (self.kp_hard - self.kp_soft)

        # 階段三：Stand Hold (高剛度鎖定 5 秒)
        elif elapsed_time < t3:
            target_rad = self.stand_target_rad
            current_kp = self.kp_hard

        # 階段四：Stand -> Crouch (剛度與位置同步餘弦下降)
        elif elapsed_time < t4:
            s = self._s_curve((elapsed_time - t3) / self.t_stand)
            target_rad = self.stand_target_rad + s * (self.crouch_target_rad - self.stand_target_rad)
            current_kp = self.kp_hard - s * (self.kp_hard - self.kp_soft)

        # 階段五：Crouch -> Prone (餘弦平滑貼地)
        elif elapsed_time < t5:
            s = self._s_curve((elapsed_time - t4) / self.t_crouch)
            target_rad = self.crouch_target_rad + s * (self.start_pos_rad - self.crouch_target_rad)
            current_kp = self.kp_soft

        # 階段六：完全趴平鎖定
        else:
            target_rad = self.start_pos_rad
            current_kp = self.kp_soft

        # 動態 dt 防暴衝限制與硬體 Clamping
        max_step_rad = math.radians(self.MAX_DEG_PER_SEC) * dt  
        clamped_diff = max(-max_step_rad, min(max_step_rad, target_rad - self.last_p_des_rad))
        current_p_des_rad = self.last_p_des_rad + clamped_diff
        self.last_p_des_rad = current_p_des_rad

        safe_p_des_rad = max(self.min_safe_rad, min(self.max_safe_rad, current_p_des_rad))

        # 膝關節 (ID 3) 重力補償力矩 (包含 t4~t5 的漸退 Fade-Out 歸零保護)
        t_ff = 0.0
        if self.motor_id == 3:
            angle_diff = safe_p_des_rad - self.stand_target_rad
            gravity_comp = self.tau_max * math.sin(abs(angle_diff))
            direction = -1.0 if ("can1" in self.channel or "can3" in self.channel) else 1.0

            if t1 <= elapsed_time < t4:
                # 支撐與升降階段：全額動態補償
                t_ff = direction * gravity_comp
            elif t4 <= elapsed_time < t5:
                # 降落貼地階段：前饋力矩線性漸退 (Fade-Out)，到達地面時精準歸零
                fade_out = 1.0 - ((elapsed_time - t4) / self.t_crouch)
                t_ff = direction * gravity_comp * max(0.0, fade_out)

        return TrajectoryPoint(
            position=safe_p_des_rad,
            velocity=0.0,
            kp=current_kp,
            kd=self.kd,
            torque_ff=t_ff
        )