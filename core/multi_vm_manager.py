"""
MultiMotorViewModelManager Module
管理多個 MotorControlViewModel 實例，實作全域聯鎖急停 (Cascade E-STOP) 與 Master 統一指令。
"""

import threading
from typing import Dict, List, Optional, Callable
from .motor_control_viewmodel import MotorControlViewModel, SystemStatus

class MultiMotorViewModelManager:
    """多軸 ViewModel 容器與全域聯鎖控制器"""

    def __init__(self):
        self._vms: Dict[str, MotorControlViewModel] = {}
        self._lock = threading.Lock()
        self._is_cascading = False

        self.on_global_estop: Optional[Callable[[str], None]] = None
        self.on_global_status_changed: Optional[Callable[[str], None]] = None

    def add_motor(self, key: str, vm: MotorControlViewModel, channel: str, motor_id: int) -> None:
        """註冊馬達 ViewModel 並綁定 Cascade E-STOP 監聽器"""
        vm.motor_id = motor_id
        setattr(vm, "channel", channel)
        setattr(vm, "vm_key", key)

        with self._lock:
            self._vms[key] = vm

        # 註冊 Cascade E-STOP 監聽：任一軸觸發 E-STOP 時自動擴散至全域
        orig_estop_cb = vm.on_estop_triggered
        def _cascading_estop_handler(reason: str, src_key=key):
            if orig_estop_cb:
                try:
                    orig_estop_cb(reason)
                except Exception:
                    pass
            self._trigger_cascade_estop(src_key, reason)

        vm.on_estop_triggered = _cascading_estop_handler

    def get_vm(self, key: str) -> Optional[MotorControlViewModel]:
        return self._vms.get(key)

    def get_all_vms(self) -> Dict[str, MotorControlViewModel]:
        with self._lock:
            return self._vms.copy()

    def _trigger_cascade_estop(self, source_key: str, reason: str) -> None:
        """核心全域聯鎖急停 (Cascade E-STOP) 廣播邏輯"""
        with self._lock:
            if self._is_cascading:
                return
            self._is_cascading = True

        cascade_msg = f"[CASCADE E-STOP] Triggered by {source_key}: {reason}"
        print(f"\n🚨 {cascade_msg}\n")

        for k, vm in self._vms.items():
            if k != source_key:
                try:
                    vm.trigger_estop(cascade_msg)
                except Exception as e:
                    print(f"[MultiVMManager] Error triggering E-STOP on {k}: {e}")

        if self.on_global_estop:
            try:
                self.on_global_estop(cascade_msg)
            except Exception:
                pass

        with self._lock:
            self._is_cascading = False

    def start_all(self) -> None:
        """Master 控制：同時啟動所有在線馬達軸"""
        for k, vm in self._vms.items():
            try:
                vm.start_control()
            except Exception as e:
                print(f"[MultiVMManager] Error starting {k}: {e}")

    def stop_all(self) -> None:
        """Master 控制：同時停止所有在線馬達軸"""
        for k, vm in self._vms.items():
            try:
                vm.stop_control()
            except Exception as e:
                print(f"[MultiVMManager] Error stopping {k}: {e}")

    def global_estop(self, reason: str = "GLOBAL MANUAL E-STOP") -> None:
        """Master 控制：手動觸發全域聯鎖急停"""
        self._trigger_cascade_estop("Master Toolbar", reason)
        for vm in self._vms.values():
            try:
                vm.trigger_estop(reason)
            except Exception:
                pass

    def get_aggregate_status(self) -> SystemStatus:
        """取得跨軸聚合狀態"""
        statuses = [vm.get_ui_snapshot().status for vm in self._vms.values()]
        if SystemStatus.E_STOPPED in statuses:
            return SystemStatus.E_STOPPED
        if SystemStatus.RUNNING in statuses:
            return SystemStatus.RUNNING
        if SystemStatus.PROBING in statuses:
            return SystemStatus.PROBING
        return SystemStatus.IDLE