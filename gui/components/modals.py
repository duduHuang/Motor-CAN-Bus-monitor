# gui/components/modals.py
+"""
+Modals Module
+定義全域急停 (Cascade E-STOP) 與單軸品質報告 (Quality Report) 的彈窗元件，提供外部 (如 AxisPanelView) 主動觸發顯示的介面。
+"""
import dearpygui.dearpygui as dpg
from gui.utils import center_modal

class GlobalEstopModal:
    """全域急停 (Cascade E-STOP) 報警彈窗元件"""

    def __init__(self):
        self.tags = {
            "modal": dpg.generate_uuid(),
            "reason_text": dpg.generate_uuid(),
        }

    def build(self) -> None:
        """建構 Modal UI 結構"""
        with dpg.window(
            label = "  EMERGENCY STOP (GLOBAL)", 
            modal = True, 
            show = False, 
            tag = self.tags["modal"], 
            width = 420, 
            height = 220, 
            no_move = True, 
            no_resize = True
        ):
            dpg.add_text("ALL SYSTEMS HALTED BY CASCADE E-STOP!", color = (255, 50, 50))
            dpg.add_separator()
            dpg.add_text("", tag = self.tags["reason_text"], color = (255, 200, 150), wrap = 400)
            dpg.add_spacer(height = 30)
            dpg.add_button(
                label = "Acknowledge", 
                width = -1, 
                height = 40, 
                callback = lambda: dpg.configure_item(self.tags["modal"], show = False)
            )

    def show(self, reason: str = "Unspecified Emergency Stop Triggered") -> None:
        """填入觸發原因、開啟 Modal 視窗並自動畫面置中"""
        if dpg.does_item_exist(self.tags["modal"]):
            dpg.set_value(self.tags["reason_text"], f"Trigger Reason:\n{reason}")
            dpg.configure_item(self.tags["modal"], show = True)
            center_modal(self.tags["modal"], 420, 220)


class QualityReportModal:
    """單軸品質報告 (Quality Report) 彈窗元件"""

    def __init__(self, key: str):
        self.key = key
        self.tags = {
            "modal": dpg.generate_uuid(),
            "report_text": dpg.generate_uuid(),
        }

    def build(self) -> None:
        """建構單軸品質報告 UI 結構"""
        with dpg.window(
            label = f"Quality Report [{self.key}]", 
            modal = True, 
            show = False, 
            tag = self.tags["modal"], 
            width = 320, 
            height = 260, 
            no_move = True, 
            no_resize = True
        ):
            dpg.add_text("Performance Metrics", color = (100, 200, 255))
            dpg.add_separator()
            dpg.add_text("", tag = self.tags["report_text"])
            dpg.add_spacer(height = 20)
            dpg.add_button(
                label = "Close", 
                width = -1, 
                height = 30, 
                callback = lambda: dpg.configure_item(self.tags["modal"], show = False)
            )

    def show(self, report_data: dict = None, actual_hz: float = 0.0) -> None:
        """帶入計算指標、寫入文字並顯示彈窗"""
        if report_data is None:
            report_data = {}

        if dpg.does_item_exist(self.tags["modal"]):
            report_str = (
                f"Tracking RMS Error:\n   {report_data.get('rms_error', 0.0):.4f} rad\n\n"
                f"Peak Velocity:\n   {report_data.get('peak_speed', 0.0):.2f} rad/s\n\n"
                f"Peak Torque:\n   {report_data.get('peak_torque', 0.0):.2f} Nm\n\n"
                f"Avg Control Freq:\n   {actual_hz:.1f} Hz"
            )
            dpg.set_value(self.tags["report_text"], report_str)
            dpg.configure_item(self.tags["modal"], show = True)
            center_modal(self.tags["modal"], 320, 260)