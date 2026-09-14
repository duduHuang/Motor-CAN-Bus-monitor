# gui/components/master_header.py
"""
MasterHeaderView Module
頂部 Master 工具列面板 View 元件，提供全域啟動/停止、全域緊急停止按鈕，以及全域狀態顯示。
"""
import dearpygui.dearpygui as dpg
from core import SystemStatus, MultiMotorViewModelManager

class MasterHeaderView:
    """頂部 Master 工具列面板 View 元件"""

    def __init__(self, manager: MultiMotorViewModelManager, estop_theme: int):
        self.manager = manager
        self.estop_theme = estop_theme
        self.tags = {
            "status_text": dpg.generate_uuid(),
        }

    def build(self):
        """建構 UI 佈局"""
        with dpg.child_window(height = 80, border = True):
            with dpg.group(horizontal = True):
                dpg.add_spacer(width = 10)
                with dpg.group():
                    dpg.add_text("GLOBAL STATUS:", color = (100, 200, 255))
                    dpg.add_text("IDLE", tag = self.tags["status_text"], color = (200, 200, 200))

                dpg.add_spacer(width = 40)
                dpg.add_button(
                    label = "MASTER START", 
                    width = 160, 
                    height = 50, 
                    callback = lambda: self.manager.start_all()
                )
                dpg.add_button(
                    label = "MASTER STOP", 
                    width = 160, 
                    height = 50, 
                    callback = lambda: self.manager.stop_all()
                )

                dpg.add_spacer(width = 50)
                btn_estop = dpg.add_button(
                    label = "GLOBAL CASCADE E-STOP",
                    width = 250,
                    height = 50,
                    callback = lambda: self.manager.global_estop("UI Global E-STOP Button Pressed")
                )
                dpg.bind_item_theme(btn_estop, self.estop_theme)

    def update(self):
        """刷新全域狀態列文字與顏色"""
        agg_status = self.manager.get_aggregate_status()
        agg_color = {
            SystemStatus.IDLE: (200, 200, 200),
            SystemStatus.PROBING: (255, 255, 0),
            SystemStatus.RUNNING: (0, 255, 0),
            SystemStatus.E_STOPPED: (255, 50, 50)
        }.get(agg_status, (255, 255, 255))

        dpg.set_value(self.tags["status_text"], agg_status.value)
        dpg.configure_item(self.tags["status_text"], color = agg_color)