from .base_ui import BaseProviderUI
from .generic_ui import GenericProviderUI
from .button_ui import ButtonProviderUI
from .manual_ui import ManualProviderUI

class ProviderUIFactory:
    """UI 分發工廠：根據 Provider 名稱回傳對應的渲染器"""

    _registry = {
        "Sine": GenericProviderUI,      # 共用 Generic UI
        "CSV": GenericProviderUI,       # 共用 Generic UI
        "DB3": GenericProviderUI,       # 共用 Generic UI
        "Button": ButtonProviderUI,     # 獨立專屬 UI
        "Manual": ManualProviderUI,     # 獨立專屬 UI
    }

    @classmethod
    def create_ui(cls, provider_name: str, **kwargs) -> BaseProviderUI:
        """若找不到對應註冊項，預設降級使用 GenericProviderUI"""
        ui_cls = cls._registry.get(provider_name, GenericProviderUI)
        return ui_cls(**kwargs)

__all__ = ["ProviderUIFactory", "BaseProviderUI"]