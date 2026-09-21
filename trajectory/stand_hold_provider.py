import math
from typing import List, Dict, Any
from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint

class StandHoldTrajectoryProvider(BaseTrajectoryProvider):
    """階段二：多階段姿態過渡、動態剛度、動態 dt 速度限制與非線性重力前饋 Provider"""

    MOTOR_CALIBRATION_DEG: Dict[str, Dict[int, Dict[str, Any]]] = {
        "can1": {
            1: {"min": -62.0,  "max": -10.0,  "crouch": -36.0,  "stand": -36.0},
            2: {"min": -380.0, "max": 160.0,  "crouch": 40.0,  "stand": 67.0},
            3: {"min": -100.0, "max": 21.0,   "crouch": -40.0,  "stand": -97.0},
        },
        "can2": {
            1: {"min": -130.0, "max": -77.0,  "crouch": -103.5, "stand": -103.5},
            2: {"min": 29.0,   "max": 545.0,  "crouch": 145.0,  "stand": 117.0},
            3: {"min": -43.0,  "max": 80.0,   "crouch": 10.0,   "stand": 81.0},
        },
        "can3": {
            1: {"min": 72.0,   "max": 123.0,  "crouch": 97.5,   "stand": 97.5},
            2: {"min": -210.0, "max": 325.0,  "crouch": 152.0,    "stand": 182.0},
            3: {"min": -96.0,  "max": 21.0,   "crouch": -30.0,  "stand": -90.0},
        },
        "can4": {
            1: {"min": -1.0,   "max": 48.0,   "crouch": 23.5,   "stand": 23.5},
            2: {"min": -140.0, "max": 370.0,  "crouch": 21.0,  "stand": -9.0},
            3: {"min": 19.0,   "max": 140.0,  "crouch": 50.0,   "stand": 139.0},
        }
    }

    MARGIN_DEG = 3.0
    MAX_DEG_PER_SEC = 90.0  # 放寬速度上限，允許俐落起立

    def __init__(self, channel: str = "can1", motor_id: int = 1, 
                 t_crouch: float = 0.8, t_stand: float = 1.5, 
                 kp_soft: float = 20.0, kp_hard: float = 55.0, kd: float = 2.5,
                 tau_max: float = 3.0):
        super().__init__()
        self.channel = channel.lower()
        self.motor_id = motor_id
        self.t_crouch = t_crouch  # 0 -> Crouch 時間 (0.8s)
        self.t_stand = t_stand    # Crouch -> Stand 時間 (1.5s)
        self.kp_soft = kp_soft    # 低蹲柔順剛度
        self.kp_hard = kp_hard    # 站立高剛度 (抗腿軟)
        self.kd = kd
        self.tau_max = tau_max    # [可微調] 膝關節前饋力矩上限 (預設 3.0 Nm 安全值)

        self.start_pos_rad = None
        self.last_p_des_rad = None
        self.last_time = 0.0      # 紀錄上一次的時間以計算動態 dt

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
            ParamSchema("t_crouch", float, 0.8, "Crouch Time (s)", "Time to reach crouch pose"),
            ParamSchema("t_stand", float, 1.5, "Stand Time (s)", "Time from crouch to stand"),
            ParamSchema("kp_hard", float, 55.0, "Standing Kp", "High stiffness for standing"),
            ParamSchema("kd", float, 2.5, "Damping (Kd)", "Velocity gain"),
            ParamSchema("tau_max", float, 3.0, "Max Torque FF (Nm)", "Max gravity feedforward torque"),
        ]

    def initialize(self, init_pos_rad: float) -> None:
        clamped_init = max(self.min_safe_rad, min(self.max_safe_rad, init_pos_rad))
        self.start_pos_rad = clamped_init
        self.last_p_des_rad = clamped_init
        self.last_time = 0.0  # 初始化時間歸零

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        if self.start_pos_rad is None:
            return TrajectoryPoint(position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque_ff=0.0)

        # 動態 dt 計算 (防禦性設計，避免首幀 dt 為 0 或負數)
        dt = elapsed_time - self.last_time
        if dt <= 0:
            dt = 0.01  # 保底 100Hz 的 dt
        self.last_time = elapsed_time

        # 階段一：Prone -> Crouch (0 ~ t_crouch 秒，低剛度)
        if elapsed_time < self.t_crouch:
            alpha = elapsed_time / self.t_crouch
            target_rad = self.start_pos_rad + alpha * (self.crouch_target_rad - self.start_pos_rad)
            current_kp = self.kp_soft

        # 階段二：Crouch -> Stand (t_crouch ~ t_stand 秒，剛度線性拉升)
        elif elapsed_time < (self.t_crouch + self.t_stand):
            alpha = (elapsed_time - self.t_crouch) / self.t_stand
            target_rad = self.crouch_target_rad + alpha * (self.stand_target_rad - self.crouch_target_rad)
            current_kp = self.kp_soft + alpha * (self.kp_hard - self.kp_soft)

        # 階段三：Stand Hold (高剛度鎖定)
        else:
            target_rad = self.stand_target_rad
            current_kp = self.kp_hard

        # 依據真實 dt 的防暴衝限制與硬體 Clamping
        max_step_rad = math.radians(self.MAX_DEG_PER_SEC) * dt  
        clamped_diff = max(-max_step_rad, min(max_step_rad, target_rad - self.last_p_des_rad))
        current_p_des_rad = self.last_p_des_rad + clamped_diff
        self.last_p_des_rad = current_p_des_rad

        safe_p_des_rad = max(self.min_safe_rad, min(self.max_safe_rad, current_p_des_rad))

        # 膝關節 (ID 3) 動態非線性重力補償力矩 (傳入 self.tau_max)
        t_ff = 0.0
        if self.motor_id == 3 and elapsed_time >= self.t_crouch:
            angle_diff = current_p_des_rad - self.stand_target_rad
            gravity_comp = self.tau_max * math.sin(abs(angle_diff))
            direction = -1.0 if ("can1" in self.channel or "can3" in self.channel) else 1.0
            t_ff = direction * gravity_comp

        return TrajectoryPoint(
            position=safe_p_des_rad,
            velocity=0.0,
            kp=current_kp,
            kd=self.kd,
            torque_ff=t_ff
        )