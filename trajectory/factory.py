# trajectory/factory.py
"""
軌跡提供者工廠模組
提供動態註冊與實例化軌跡提供者的功能。
"""
from typing import Any, Dict, List, Type
from trajectory.base import BaseTrajectoryProvider, ParamSchema

class ProviderFactory:
    """軌跡提供者註冊表與動態工廠。"""

    _registry: Dict[str, Type[BaseTrajectoryProvider]] = {}

    @classmethod
    def register(cls, name: str, provider_cls: Type[BaseTrajectoryProvider]) -> None:
        """註冊新的 Provider 類別。"""
        if not issubclass(provider_cls, BaseTrajectoryProvider):
            raise TypeError(f"類別 {provider_cls.__name__} 必須繼承自 BaseTrajectoryProvider")
        cls._registry[name] = provider_cls

    @classmethod
    def list_providers(cls) -> Dict[str, str]:
        """取得目前已註冊的 Provider 清單與其描述。"""
        return {
            name: (p_cls.__doc__ or p_cls.__name__).strip()
            for name, p_cls in cls._registry.items()
        }

    @classmethod
    def get_provider_schema(cls, name: str) -> List[ParamSchema]:
        """取得指定 Provider 的參數定義 Schema。"""
        if name not in cls._registry:
            raise KeyError(f"未找到已註冊的 Provider: '{name}'")
        return cls._registry[name].get_param_schema()

    @classmethod
    def create_provider(cls, name: str, **kwargs: Any) -> BaseTrajectoryProvider:
        """動態實例化指定 Provider, 並對輸入參數進行型別與範圍檢查。"""
        if name not in cls._registry:
            raise KeyError(f"未找到已註冊的 Provider: '{name}'")

        provider_cls = cls._registry[name]
        schemas = provider_cls.get_param_schema()
        validated_args = {}

        for schema in schemas:
            param_name = schema.name
            raw_val = kwargs.get(param_name, schema.default_value)

            # 1. 型別轉換與驗證
            if raw_val is not None and not isinstance(raw_val, schema.param_type):
                try:
                    raw_val = schema.param_type(raw_val)
                except (ValueError, TypeError) as err:
                    raise ValueError(
                        f"參數 '{param_name}' 型別錯誤: 期望 {schema.param_type.__name__}，收到 {type(raw_val).__name__}"
                    ) from err

            # 2. 數值邊界檢查
            if isinstance(raw_val, (int, float)):
                if schema.min_value is not None and raw_val < schema.min_value:
                    raise ValueError(f"參數 '{param_name}' 數值 {raw_val} 低於最小值限制 {schema.min_value}")
                if schema.max_value is not None and raw_val > schema.max_value:
                    raise ValueError(f"參數 '{param_name}' 數值 {raw_val} 超出最大值限制 {schema.max_value}")

            validated_args[param_name] = raw_val

        return provider_cls(**validated_args)