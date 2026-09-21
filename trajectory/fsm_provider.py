import math
from enum import Enum
from typing import Any, Dict, List

from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint


class PostureState(Enum):
    PRONE = 0    # 趴姿 (底盤貼地，無前饋，低剛度)
    CROUCH = 1   # 低蹲過渡姿態
    STAND = 2    # 高剛度站立姿態


class QuadrupedFSMProvider(BaseTrajectoryProvider):
    """雙向姿態狀態機 (FSM) 軌跡提供者 (顯式 Prone 校正角度版)"""

    # 加入顯式 prone 角度，確保按 P 鍵時不受開機姿態干擾
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
    MAX_DEG_PER_SEC = 90.0  # 角速度上限 (deg/s)

    def __init__(self, channel: str = "can1", motor_id: int = 1, 
                 kp_soft: float = 35.0, kp_hard: float = 55.0, kd: float = 2.5, tau_max: float = 3.0):
        super().__init__()
        self.channel = channel.lower()
        self.motor_id = motor_id
        self.kp_soft = kp_soft    # 低蹲/趴姿柔順剛度
        self.kp_hard = kp_hard    # 站立高剛度
        self.kd = kd
        self.tau_max = tau_max    # 膝關節前饋力矩上限 (Nm)

        self.last_p_des_rad = None
        self.last_kp = kp_soft
        self.last_time = 0.0

        # FSM 狀態與軌跡管理
        self.current_state = PostureState.PRONE
        self.target_state = PostureState.PRONE
        
        self.start_rad = 0.0
        self.end_rad = 0.0
        self.start_kp = kp_soft
        self.end_kp = kp_soft
        
        self.is_two_stage = False
        self.t_trans = 1.0           
        self.trans_progress_t = 1.0  

        ch_cfg = self.MOTOR_CALIBRATION_DEG.get(self.channel, self.MOTOR_CALIBRATION_DEG["can1"])
        m_cfg = ch_cfg.get(self.motor_id, ch_cfg[1])

        self.min_safe_rad = math.radians(m_cfg["min"] + self.MARGIN_DEG)
        self.max_safe_rad = math.radians(m_cfg["max"] - self.MARGIN_DEG)
        self.prone_target_rad = math.radians(m_cfg["prone"])
        self.crouch_target_rad = math.radians(m_cfg["crouch"])
        self.stand_target_rad = math.radians(m_cfg["stand"])

    @property
    def duration(self) -> float:
        return 0.0

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        return [
            ParamSchema("channel", str, "can1", "CAN Channel", "Target CAN bus channel"),
            ParamSchema("motor_id", int, 1, "Motor ID", "Joint motor ID", min_value=1, max_value=3),
            ParamSchema("kp_soft", float, 20.0, "Soft Kp", "Stiffness for prone/crouch", min_value=0.0, max_value=200.0),
            ParamSchema("kp_hard", float, 55.0, "Hard Kp", "Stiffness for standing", min_value=0.0, max_value=200.0),
            ParamSchema("kd", float, 2.5, "Damping (Kd)", "Velocity gain", min_value=0.0, max_value=20.0),
            ParamSchema("tau_max", float, 3.0, "Max Torque FF (Nm)", "Max gravity feedforward torque", min_value=0.0, max_value=10.0),
        ]

    def initialize(self, init_pos: float) -> None:
        """開機對齊：優先對齊真實角度為起點，但保持 prone_target_rad 為顯式定義的趴姿"""
        clamped_init = max(self.min_safe_rad, min(self.max_safe_rad, init_pos))
        self.last_p_des_rad = clamped_init
        self.start_rad = clamped_init
        self.end_rad = clamped_init
        self.last_kp = self.kp_soft
        self.last_time = 0.0
        self.trans_progress_t = 1.0

    def handle_input(self, action: str, **kwargs: Any) -> None:
        act = action.lower()
        if act in ["prone", "p"]:
            self.set_target_state(PostureState.PRONE)
        elif act in ["crouch", "c"]:
            self.set_target_state(PostureState.CROUCH)
        elif act in ["stand", "s"]:
            self.set_target_state(PostureState.STAND)

    def set_target_state(self, new_state: PostureState) -> None:
        if new_state == self.target_state and self.trans_progress_t >= self.t_trans:
            return

        self.start_rad = self.last_p_des_rad if self.last_p_des_rad is not None else self.prone_target_rad
        self.start_kp = self.last_kp

        pair = (self.current_state, new_state)
        self.is_two_stage = (PostureState.PRONE in pair and PostureState.STAND in pair)

        if self.is_two_stage:
            self.t_trans = 2.3  # STAND <-> CROUCH (1.5s) + CROUCH <-> PRONE (0.8s)
        elif PostureState.CROUCH in pair and PostureState.STAND in pair:
            self.t_trans = 1.5
        else:
            self.t_trans = 0.8

        if new_state == PostureState.PRONE:
            self.end_rad = self.prone_target_rad
            self.end_kp = self.kp_soft
        elif new_state == PostureState.CROUCH:
            self.end_rad = self.crouch_target_rad
            self.end_kp = self.kp_soft
        elif new_state == PostureState.STAND:
            self.end_rad = self.stand_target_rad
            self.end_kp = self.kp_hard

        self.target_state = new_state
        self.trans_progress_t = 0.0

    def _s_curve(self, alpha: float) -> float:
        a = max(0.0, min(1.0, alpha))
        return (1.0 - math.cos(math.pi * a)) / 2.0

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        if self.last_p_des_rad is None:
            return TrajectoryPoint(position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque_ff=0.0)

        dt = elapsed_time - self.last_time
        if dt <= 0: dt = 0.01
        self.last_time = elapsed_time

        if self.trans_progress_t < self.t_trans:
            self.trans_progress_t += dt
            if self.trans_progress_t >= self.t_trans:
                self.trans_progress_t = self.t_trans
                self.current_state = self.target_state

        if self.is_two_stage:
            if self.target_state == PostureState.PRONE:
                # STAND -> CROUCH (1.5s) -> PRONE (0.8s)
                if self.trans_progress_t < 1.5:
                    s = self._s_curve(self.trans_progress_t / 1.5)
                    target_rad = self.start_rad + s * (self.crouch_target_rad - self.start_rad)
                    current_kp = self.start_kp - s * (self.start_kp - self.kp_soft)
                else:
                    s = self._s_curve((self.trans_progress_t - 1.5) / 0.8)
                    target_rad = self.crouch_target_rad + s * (self.prone_target_rad - self.crouch_target_rad)
                    current_kp = self.kp_soft
            else:
                # PRONE -> CROUCH (0.8s) -> STAND (1.5s)
                if self.trans_progress_t < 0.8:
                    s = self._s_curve(self.trans_progress_t / 0.8)
                    target_rad = self.start_rad + s * (self.crouch_target_rad - self.start_rad)
                    current_kp = self.kp_soft
                else:
                    s = self._s_curve((self.trans_progress_t - 0.8) / 1.5)
                    target_rad = self.crouch_target_rad + s * (self.stand_target_rad - self.crouch_target_rad)
                    current_kp = self.kp_soft + s * (self.kp_hard - self.kp_soft)
        else:
            alpha = self.trans_progress_t / self.t_trans
            s = self._s_curve(alpha)
            target_rad = self.start_rad + s * (self.end_rad - self.start_rad)
            current_kp = self.start_kp + s * (self.end_kp - self.start_kp)

        self.last_kp = current_kp

        max_step_rad = math.radians(self.MAX_DEG_PER_SEC) * dt
        clamped_diff = max(-max_step_rad, min(max_step_rad, target_rad - self.last_p_des_rad))
        current_p_des_rad = self.last_p_des_rad + clamped_diff
        self.last_p_des_rad = current_p_des_rad

        safe_p_des_rad = max(self.min_safe_rad, min(self.max_safe_rad, current_p_des_rad))

        t_ff = 0.0
        if self.motor_id in [2, 3]:
            # 計算相對於站立姿態的偏角
            angle_diff = safe_p_des_rad - self.stand_target_rad
            
            # HipY (ID 2) 分配 0.5 倍前饋，Knee (ID 3) 分配 1.0 倍前饋
            ff_scale = 0.5 if self.motor_id == 2 else 1.0
            gravity_comp = self.tau_max * ff_scale * math.sin(abs(angle_diff))
            
            direction = -1.0 if ("can1" in self.channel or "can3" in self.channel) else 1.0

            if self.target_state == PostureState.PRONE:
                if self.is_two_stage and self.trans_progress_t >= 1.5:
                    fade_out = 1.0 - ((self.trans_progress_t - 1.5) / 0.8)
                    t_ff = direction * gravity_comp * max(0.0, fade_out)
                elif not self.is_two_stage:
                    fade_out = 1.0 - (self.trans_progress_t / self.t_trans)
                    t_ff = direction * gravity_comp * max(0.0, fade_out)
                else:
                    t_ff = direction * gravity_comp
            elif self.current_state == PostureState.PRONE and self.target_state == PostureState.PRONE:
                t_ff = 0.0
            else:
                t_ff = direction * gravity_comp

        return TrajectoryPoint(
            position=safe_p_des_rad,
            velocity=0.0,
            kp=current_kp,
            kd=self.kd,
            torque_ff=t_ff
        )