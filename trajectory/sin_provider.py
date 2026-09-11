import math
from typing import List
from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint


class SineTrajectoryProvider(BaseTrajectoryProvider):
    """正弦波軌跡產生器 (支援無限持續循環、實體安全初始化與前饋平滑淡入)。"""

    def __init__(
        self,
        amplitude: float = 0.5,
        frequency: float = 0.5,
        stand_offset: float = 0.0,
        duration: float = 0.0,  # 0.0 代表持續無限循環 (Continuous Infinite Loop)
        kp: float = 40.0,
        kd: float = 2.2,
        enable_ff: bool = True,
        enable_inertia_ff: bool = False,
        m_leg: float = 1.8,
        r_com: float = 0.20,
        j_eq: float = 0.012,
        axis_sign: float = 1.0,
        max_gravity_ff: float = 2.0,
    ) -> None:
        self.amplitude = amplitude
        self.frequency = frequency
        self.stand_offset = stand_offset
        self.duration = duration
        self.kp = kp
        self.kd = kd

        # 前饋物理模型參數
        self.enable_ff = enable_ff
        self.enable_inertia_ff = enable_inertia_ff
        self.m_leg = m_leg
        self.g = 9.81
        self.r_com = r_com
        self.j_eq = j_eq
        self.axis_sign = axis_sign
        self.max_gravity_ff = max_gravity_ff

        # 安全保護偏置量
        self._start_offset = 0.0

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        return [
            ParamSchema("amplitude", float, 0.5, "Amplitude (rad)", "Sine wave amplitude", min_value = 0.0, max_value = 5.0, step = 0.05),
            ParamSchema("frequency", float, 0.5, "Frequency (Hz)", "Sine wave frequency", min_value = 0.1, max_value = 10.0, step = 0.1),
            ParamSchema("stand_offset", float, 0.0, "Offset (rad)", "Sine wave center offset", min_value = -5.0, max_value = 5.0, step = 0.05),
            ParamSchema("duration", float, 0.0, "Duration (s)", "Run duration (0 = Continuous Loop)", min_value = 0.0, max_value = 300.0, step = 1.0),
            ParamSchema("kp", float, 40.0, "Stiffness (Kp)", "MIT position gain", min_value = 0.0, max_value = 500.0, step = 1.0),
            ParamSchema("kd", float, 2.2, "Damping (Kd)", "MIT velocity gain", min_value = 0.0, max_value = 50.0, step = 0.1),
            ParamSchema("enable_ff", bool, True, "Enable Feedforward", "Enable gravity/inertia feedforward compensation"),
            ParamSchema("enable_inertia_ff", bool, False, "Enable Inertia FF", "Enable angular acceleration feedforward"),
            ParamSchema("max_gravity_ff", float, 2.0, "Max Gravity FF Limit", "Gravity compensation limit range (Nm)", min_value = 0.0, max_value = 10.0, step = 0.1),
        ]

    def initialize(self, init_pos: float) -> None:
        """防暴衝核心機制：接收馬達啟動時的實體角度，計算正弦波起點 (t=0) 的平移偏置。"""
        t0_pos = self.amplitude * math.sin(0.0) + self.stand_offset
        self._start_offset = init_pos - t0_pos

    def handle_input(self, action: str, **kwargs) -> None:
        """動態更新 UI 控制參數"""
        if action == "update_targets":
            for key, val in kwargs.items():
                if hasattr(self, key):
                    setattr(self, key, val)

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        omega = 2.0 * math.pi * self.frequency

        # 理論 Kinematics 計算 (透過 sin/cos 連續函數無限重複循環)
        pos = self.amplitude * math.sin(omega * elapsed_time) + self.stand_offset + self._start_offset
        vel = self.amplitude * omega * math.cos(omega * elapsed_time)
        accel = -self.amplitude * (omega ** 2) * math.sin(omega * elapsed_time)

        # 啟動平滑淡入 (0.5 秒內將速度與前饋從 0 漸進拉升)
        ramp = min(1.0, elapsed_time / 0.5) if elapsed_time < 0.5 else 1.0
        vel *= ramp

        t_ff = 0.0
        if self.enable_ff:
            theta_real = pos - self.stand_offset
            tau_gravity = self.axis_sign * (self.m_leg * self.g * self.r_com * math.cos(theta_real))
            tau_gravity = max(-self.max_gravity_ff, min(self.max_gravity_ff, tau_gravity))

            tau_inertia = 0.0
            if self.enable_inertia_ff:
                tau_inertia = self.axis_sign * (self.j_eq * accel)
                tau_inertia = max(-1.5, min(1.5, tau_inertia))

            t_ff = (tau_gravity + tau_inertia) * ramp

        return TrajectoryPoint(
            position = pos,
            velocity = vel,
            kp = self.kp,
            kd = self.kd,
            torque_ff = t_ff
        )