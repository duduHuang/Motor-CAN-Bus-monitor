import dearpygui.dearpygui as dpg
from core import SystemStatus, ControlSnapshot

class AxisStatsView:
    """單軸狀態列與統計指標 View 元件"""

    def __init__(self, key: str):
        self.key = key
        self.tags = {
            "status_text": dpg.generate_uuid(),
            "error_msg": dpg.generate_uuid(),
            "tx_count": dpg.generate_uuid(),
            "rx_count": dpg.generate_uuid(),
            "loss_rate": dpg.generate_uuid(),
            "actual_hz": dpg.generate_uuid(),
        }
        self._custom_error_msg = ""

    def build(self):
        """建構 UI 佈局"""
        with dpg.child_window(height=120, border=True):
            dpg.add_text("System Status", color=(100, 200, 255))
            dpg.add_text("IDLE", tag=self.tags["status_text"], color=(200, 200, 200))
            dpg.add_text("", tag=self.tags["error_msg"], color=(255, 100, 100))

            with dpg.group(horizontal=True):
                dpg.add_text("TX: 0", tag=self.tags["tx_count"])
                dpg.add_text(" | RX: 0", tag=self.tags["rx_count"])
                dpg.add_text(" | Loss: 0.0%", tag=self.tags["loss_rate"])
                dpg.add_text(" | Freq: 0.0 Hz", tag=self.tags["actual_hz"])

    def update(self, snap: ControlSnapshot):
        """根據 ViewModel 快照更新數據"""
        # 防呆檢查：若 UI 尚未 build 完成則直接返回
        if not dpg.does_item_exist(self.tags["status_text"]):
            return

        status_color = {
            SystemStatus.IDLE: (200, 200, 200),
            SystemStatus.PROBING: (255, 255, 0),
            SystemStatus.RUNNING: (0, 255, 0),
            SystemStatus.E_STOPPED: (255, 0, 0)
        }.get(snap.status, (255, 255, 255))

        status_txt = f"Status: {snap.status.value}" + (" (TIMEOUT!)" if snap.is_timeout else "")
        dpg.set_value(self.tags["status_text"], status_txt)
        dpg.configure_item(self.tags["status_text"], color=status_color)

        dpg.set_value(self.tags["tx_count"], f"TX: {snap.tx_count}")
        dpg.set_value(self.tags["rx_count"], f" | RX: {snap.rx_count}")
        dpg.set_value(self.tags["loss_rate"], f" | Loss: {snap.loss_rate:.1f}%")
        dpg.set_value(self.tags["actual_hz"], f" | Freq: {snap.actual_hz:.1f} Hz")

        # 優先顯示 ViewModel 控制核心抛出的系統錯誤；若無，則顯示 UI 觸發的提示訊息
        display_msg = snap.error_msg if snap.error_msg else self._custom_error_msg
        if dpg.does_item_exist(self.tags["error_msg"]):
            dpg.set_value(self.tags["error_msg"], display_msg or "")

    def set_error_message(self, msg: str):
        """主動設定錯誤與提醒訊息"""
        self._custom_error_msg = msg or ""
        # 加上防呆保護，避免 UI 元件尚未 build 時調用 dpg.set_value 拋出 Exception
        if dpg.does_item_exist(self.tags["error_msg"]):
            dpg.set_value(self.tags["error_msg"], self._custom_error_msg)