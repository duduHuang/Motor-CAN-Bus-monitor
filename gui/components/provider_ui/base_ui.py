from abc import ABC, abstractmethod
from typing import Dict, Any, Callable, List, Optional
from trajectory.base import ParamSchema
from core import ControlSnapshot

class BaseProviderUI(ABC):
    def __init__(
        self,
        parent_tag: int | str,
        schemas: List[ParamSchema],
        current_params: Dict[str, Any],
        on_param_changed: Callable[[str, Any], None],
        open_file_dialog_cb: Optional[Callable[[str, str, int], None]] = None,
        vm_key: str = "",
        motor_id: int = 1,
        vm: Any = None
    ):
        self.parent_tag = parent_tag
        self.schemas = schemas
        self.current_params = current_params
        self.on_param_changed = on_param_changed
        self.open_file_dialog_cb = open_file_dialog_cb
        self.vm_key = vm_key
        self.motor_id = motor_id
        self.vm = vm

    @abstractmethod
    def build(self) -> None:
        pass

    def update(self, snap: ControlSnapshot) -> None:
        pass