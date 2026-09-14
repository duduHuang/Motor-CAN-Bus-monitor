# trajectory/base.py
"""
軌跡生成器基類模組
提供軌跡生成器的抽象基類與相關資料結構。
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, List, Optional, Type


@dataclass
class ParamSchema:
    """軌跡提供者參數元資料描述，用於 UI 動態綁定與型別驗證。"""
    name: str
    param_type: Type
    default_value: Any
    display_name: str
    description: str = ""
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    step: Optional[float] = None
    is_file_path: bool = False
    file_filter: str = ".*"


@dataclass
class TrajectoryPoint:
    """MIT 模式馬達控制目標點數據結構。"""
    position: float = 0.0        # 目標位置 (rad)
    velocity: float = 0.0        # 目標速度 (rad/s)
    kp: float = 0.0              # 位置增益 (Stiffness)
    kd: float = 0.0              # 速度增益 (Damping)
    torque_ff: float = 0.0       # 前饋扭矩 (Nm)


class BaseTrajectoryProvider(ABC):
    """軌跡提供者抽象基類，支援自我描述 (Self-describing) 與互動回應機制。"""

    @classmethod
    @abstractmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        """傳回該 Provider 所需之參數定義清單，供 UI/ViewModel 動態讀取。"""
        pass

    @abstractmethod
    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        """根據累計時間 (秒) 計算並回傳 MIT 控制點。"""
        pass

    def handle_input(self, action: str, **kwargs: Any) -> None:
        """接收外部 UI/ViewModel 傳遞的即時互動事件 (預設無視，由互動型 Provider 覆寫)。"""
        pass

    def initialize(self, init_pos: float) -> None:
        """
        系統在啟動控制時 (Probing 成功後) 會呼叫此方法。
        允許 Provider 將初始目標點對齊馬達的當前真實位置，避免啟動瞬間暴衝。
        預設不作任何事，子類可視需求覆寫。
        """
        pass