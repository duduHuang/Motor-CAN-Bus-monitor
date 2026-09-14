# trajectory/button_provider.py
"""
ButtonTrajectoryProvider 模組
提供 UI 方向鈕即時調參軌跡與增益的 Provider
"""
from typing import Any, List, Union
from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint

class ButtonTrajectoryProvider(BaseTrajectoryProvider):
    """ UI 方向鈕即時調參軌跡與增益提供者。 """

    def __init__(
        self,
        p_des: float = 0.0,
        v_des: float = 0.0,
        kp: float = 40.0,
        kd: float = 2.0,
        step_pos: float = 0.02,
        step_vel: float = 0.1,
        step_kp: float = 1.0,
        step_kd: float = 0.1,
    ) -> None:
        super().__init__()
        self.p_des = float(p_des)
        self.v_des = float(v_des)
        self.kp = float(kp)
        self.kd = float(kd)

        self.step_pos = float(step_pos)
        self.step_vel = float(step_vel)
        self.step_kp = float(step_kp)
        self.step_kd = float(step_kd)

        self.e_stop_triggered: bool = False

    @property
    def duration(self) -> float:
        """0.0 代表持續無限期運轉"""
        return 0.0

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        return [
            ParamSchema("step_pos", float, 0.02, "Pos Step Span (rad)", "Position adjustment step size", min_value=0.001, max_value=0.5, step=0.005),
            ParamSchema("step_vel", float, 0.1, "Vel Step Span (rad/s)", "Velocity adjustment step size", min_value=0.01, max_value=2.0, step=0.05),
            ParamSchema("kp", float, 40.0, "Stiffness (Kp)", "MIT position gain Kp", min_value=0.0, max_value=500.0, step=1.0),
            ParamSchema("kd", float, 2.0, "Damping (Kd)", "MIT velocity gain Kd", min_value=0.0, max_value=50.0, step=0.1),
            ParamSchema("step_kp", float, 1.0, "Kp Step Span", "Kp adjustment step size", min_value=0.1, max_value=10.0, step=0.5),
            ParamSchema("step_kd", float, 0.1, "Kd Step Span", "Kd adjustment step size", min_value=0.01, max_value=2.0, step=0.05),
        ]

    def initialize(self, init_pos: Union[float, List[float], Any]) -> None:
        """對齊馬達啟動時的實體角度"""
        if isinstance(init_pos, (int, float)):
            self.p_des = float(init_pos)

    def handle_input(self, action: str, **kwargs: Any) -> None:
        """接收 UI 按鈕點擊或全域鍵盤事件"""
        if action == "key":
            key = str(kwargs.get("key", "")).lower()
            self._process_key(key)
        elif action == "pos_up":
            self.p_des += self.step_pos
        elif action == "pos_down":
            self.p_des -= self.step_pos
        elif action == "vel_inc":
            self.v_des += self.step_vel
        elif action == "vel_dec":
            self.v_des -= self.step_vel
        elif action == "kp_inc":
            self.kp += self.step_kp
        elif action == "kp_dec":
            self.kp = max(0.0, self.kp - self.step_kp)
        elif action == "kd_inc":
            self.kd += self.step_kd
        elif action == "kd_dec":
            self.kd = max(0.0, self.kd - self.step_kd)
        elif action == "update_targets":
            for k in ["step_pos", "step_vel", "step_kp", "step_kd", "kp", "kd", "p_des", "v_des"]:
                if k in kwargs:
                    setattr(self, k, float(kwargs[k]))

    def _process_key(self, key: str) -> None:
        """
        熱鍵對應邏輯：
        w/s: 位置 升 / 降
        a/d: 速度 減 / 加
        1/2: Kp  加 / 減
        3/4: Kd  加 / 減
        """
        if key in ['w', 'up']:
            self.p_des += self.step_pos
        elif key in ['s', 'down']:
            self.p_des -= self.step_pos
        elif key in ['d', 'right']:
            self.v_des += self.step_vel
        elif key in ['a', 'left']:
            self.v_des -= self.step_vel
        elif key == '1':
            self.kp += self.step_kp
        elif key == '2':
            self.kp = max(0.0, self.kp - self.step_kp)
        elif key == '3':
            self.kd += self.step_kd
        elif key == '4':
            self.kd = max(0.0, self.kd - self.step_kd)

    def get_target(self, elapsed_time: float = 0.0) -> TrajectoryPoint:
        return TrajectoryPoint(
            position=self.p_des,
            velocity=self.v_des,
            kp=self.kp,
            kd=self.kd,
            torque_ff=0.0
        )