# gui/components/axis_dashboard.py
import dearpygui.dearpygui as dpg
from core import SystemStatus, ControlSnapshot

class AxisDashboardView:
    """單軸頂部即時狀態看板 View"""
    def __init__(self, key: str = ""):
        self.key = key
        self.tags = {
            "pos_val": dpg.generate_uuid(),
            "spd_val": dpg.generate_uuid(),
            "cur_val": dpg.generate_uuid(),
            "tmp_val": dpg.generate_uuid(),
            "vol_val": dpg.generate_uuid(),
            # 狀態與錯誤訊息的 Tag
            "status_txt": dpg.generate_uuid(),
            "err_txt": dpg.generate_uuid(),
        }
        # 用來暫存來自左側 Panel 的 UI 錯誤訊息 (例如檔名錯誤、匯出失敗)
        self._custom_error_msg = ""

    def build(self):
        """建構 UI 佈局"""
        # 將 height 稍微調高至 115，騰出空間給下方的錯誤訊息列
        with dpg.child_window(height=115, border=True):
            with dpg.group(horizontal=True):
                # 1. 角度 (Position)
                with dpg.group():
                    dpg.add_text("Position (°)", color=(150, 150, 150))
                    dpg.add_text("0.00", tag=self.tags["pos_val"], color=(50, 255, 50))
                dpg.add_spacer(width=30)
                
                # 2. 轉速 (Speed)
                with dpg.group():
                    dpg.add_text("Speed (dps)", color=(150, 150, 150))
                    dpg.add_text("0.00", tag=self.tags["spd_val"], color=(50, 255, 255))
                dpg.add_spacer(width=30)
                
                # 3. 電流 (Current)
                with dpg.group():
                    dpg.add_text("Current (A)", color=(150, 150, 150))
                    dpg.add_text("0.00", tag=self.tags["cur_val"], color=(255, 255, 50))
                dpg.add_spacer(width=30)
                
                # 4. 溫度 (Temperature)
                with dpg.group():
                    dpg.add_text("Temperature (°C)", color=(150, 150, 150))
                    dpg.add_text("0.0", tag=self.tags["tmp_val"], color=(255, 100, 100))
                dpg.add_spacer(width=30)
                
                # 5. 電壓 (Voltage)
                with dpg.group():
                    dpg.add_text("Voltage (V)", color=(150, 150, 150))
                    dpg.add_text("0.0", tag=self.tags["vol_val"], color=(255, 150, 255))

            # 分隔線與底部的狀態列
            dpg.add_separator()
            with dpg.group(horizontal=True):
                dpg.add_text("STATUS: IDLE", tag=self.tags["status_txt"], color=(200, 200, 200))
                dpg.add_spacer(width=15)
                dpg.add_text("", tag=self.tags["err_txt"], color=(255, 100, 100))

    def update(self, snap: ControlSnapshot):
        """根據 ViewModel 快照更新數值"""
        if not dpg.does_item_exist(self.tags["pos_val"]):
            return

        # 更新五大數值
        dpg.set_value(self.tags["pos_val"], f"{snap.position_deg:.2f}")
        dpg.set_value(self.tags["spd_val"], f"{snap.speed_dps:.2f}")
        dpg.set_value(self.tags["cur_val"], f"{snap.current_a:.2f}")
        dpg.set_value(self.tags["tmp_val"], f"{snap.temperature:.1f}")
        dpg.set_value(self.tags["vol_val"], f"{snap.voltage:.1f}")

        # 更新系統狀態顏色與文字
        status_color = {
            SystemStatus.IDLE: (200, 200, 200),
            SystemStatus.PROBING: (255, 255, 0),
            SystemStatus.RUNNING: (0, 255, 0),
            SystemStatus.E_STOPPED: (255, 0, 0)
        }.get(snap.status, (255, 255, 255))
        
        status_txt = f"STATUS: {snap.status.value}" + (" (TIMEOUT!)" if snap.is_timeout else "")
        dpg.set_value(self.tags["status_txt"], status_txt)
        dpg.configure_item(self.tags["status_txt"], color=status_color)

        # 優先顯示底層 ViewModel 控制核心拋出的錯誤，若無則顯示 UI (面板) 拋出的提示訊息
        display_msg = snap.error_msg if snap.error_msg else self._custom_error_msg
        dpg.set_value(self.tags["err_txt"], display_msg or "")

    def set_error_message(self, msg: str):
        """提供給外部 (如 AxisPanelView) 主動寫入自訂錯誤與提醒訊息"""
        self._custom_error_msg = msg or ""
        # 加上防呆保護，避免 UI 尚未 build 完就賦值
        if dpg.does_item_exist(self.tags["err_txt"]):
            dpg.set_value(self.tags["err_txt"], self._custom_error_msg)