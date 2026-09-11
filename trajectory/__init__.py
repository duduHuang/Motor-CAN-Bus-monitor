from .base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint
from .csv_provider import CsvTrajectoryProvider
from .factory import ProviderFactory
from .button_provider import ButtonTrajectoryProvider
from .keyboard_provider import KeyboardTrajectoryProvider
from .sin_provider import SineTrajectoryProvider
from .db_provider import Db3TrajectoryProvider
from .manual_provider import ManualProvider

def _auto_register_providers() -> None:
    """自動註冊專案內所有標準 Provider 至 ProviderFactory。"""
    ProviderFactory.register("Sine", SineTrajectoryProvider)
    ProviderFactory.register("CSV", CsvTrajectoryProvider)
    ProviderFactory.register("Button", ButtonTrajectoryProvider)
    ProviderFactory.register("Keyboard", KeyboardTrajectoryProvider)
    ProviderFactory.register("DB3", Db3TrajectoryProvider)
    ProviderFactory.register("Manual", ManualProvider)
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
]