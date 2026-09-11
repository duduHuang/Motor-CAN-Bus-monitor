"""
Manual Trajectory Provider Module
提供即時手動調參機制，允許 UI 在控制過程中動態修改目標指令，以進行手動響應測試。
"""

import threading
from typing import Any, List

from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint
from trajectory.factory import ProviderFactory


class ManualProvider(BaseTrajectoryProvider):
    """
    手動即時調參 Provider。
    支援透過開放 API 進行 Thread-Safe 的運行中指令變更。
    """

    def __init__(
        self, 
        p_des: float = 0.0, 
        v_des: float = 0.0, 
        kp: float = 0.0, 
        kd: float = 0.0, 
        t_ff: float = 0.0
    ):
        self._lock = threading.Lock()
        self._target = TrajectoryPoint(
            position = p_des,
            velocity = v_des,
            kp = kp,
            kd = kd,
            torque_ff = t_ff
        )

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        return [
            ParamSchema("p_des", float, 0.0, "Target Pos (rad)"),
            ParamSchema("v_des", float, 0.0, "Target Vel (rad/s)"),
            ParamSchema("kp", float, 0.0, "Stiffness (Kp)"),
            ParamSchema("kd", float, 0.0, "Damping (Kd)"),
            ParamSchema("t_ff", float, 0.0, "Feedforward (Nm)"),
        ]

    def initialize(self, init_pos: float) -> None:
        """對齊初始位置，避免啟動時瞬間暴衝"""
        with self._lock:
            self._target.position = init_pos

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        """背景控制執行緒 (Control Thread) 輪詢調用"""
        with self._lock:
            # 拷貝一份回傳以避免外部修改影響內部狀態
            return TrajectoryPoint(
                position=self._target.position,
                velocity=self._target.velocity,
                kp=self._target.kp,
                kd=self._target.kd,
                torque_ff=self._target.torque_ff
            )

    def update_targets(self, p_des: float, v_des: float, kp: float, kd: float, t_ff: float) -> None:
        """供 UI/ViewModel 即時呼叫的強型別 API"""
        with self._lock:
            self._target.position = p_des
            self._target.velocity = v_des
            self._target.kp = kp
            self._target.kd = kd
            self._target.torque_ff = t_ff

    def handle_input(self, action: str, **kwargs: Any) -> None:
        """相容 BaseTrajectoryProvider 的泛用事件介面"""
        if action == "update_targets":
            with self._lock:
                self._target.position = kwargs.get("p_des", self._target.position)
                self._target.velocity = kwargs.get("v_des", self._target.velocity)
                self._target.kp = kwargs.get("kp", self._target.kp)
                self._target.kd = kwargs.get("kd", self._target.kd)
                self._target.torque_ff = kwargs.get("t_ff", self._target.torque_ff)


# 自動註冊至 ProviderFactory，以便 UI 啟動時可以直接列表選取
ProviderFactory.register("Manual", ManualProvider)