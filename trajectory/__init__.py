# trajectory/__init__.py
"""
軌跡生成器模組
提供各種軌跡生成器的接口與實現。
"""
from .base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint
from .csv_provider import CsvTrajectoryProvider
from .factory import ProviderFactory
from .button_provider import ButtonTrajectoryProvider
from .keyboard_provider import KeyboardTrajectoryProvider
from .sin_provider import SineTrajectoryProvider
from .db_provider import Db3TrajectoryProvider
from .manual_provider import ManualProvider
from .stand_hold_provider import StandHoldTrajectoryProvider
from .fsm_provider import QuadrupedFSMProvider, PostureState
from .gait_provider import QuadrupedGaitProvider

def _auto_register_providers() -> None:
    """自動註冊專案內所有標準 Provider 至 ProviderFactory。"""
    ProviderFactory.register("Sine", SineTrajectoryProvider)
    ProviderFactory.register("CSV", CsvTrajectoryProvider)
    ProviderFactory.register("Button", ButtonTrajectoryProvider)
    ProviderFactory.register("Keyboard", KeyboardTrajectoryProvider)
    ProviderFactory.register("DB3", Db3TrajectoryProvider)
    ProviderFactory.register("Manual", ManualProvider)
    ProviderFactory.register("StandHold", StandHoldTrajectoryProvider)
    ProviderFactory.register("FSM", QuadrupedFSMProvider)
    ProviderFactory.register("Gait", QuadrupedGaitProvider)
# 套件載入時自動執行註冊
_auto_register_providers()

__all__ = [
    "BaseTrajectoryProvider",
    "ParamSchema",
    "TrajectoryPoint",
    "ProviderFactory",
    "SineTrajectoryProvider",
    "CsvTrajectoryProvider",
    "ButtonTrajectoryProvider",
    "KeyboardTrajectoryProvider",
    "Db3TrajectoryProvider",
    "ManualProvider",
    "StandHoldTrajectoryProvider",
    "QuadrupedFSMProvider",
    "QuadrupedGaitProvider",
]