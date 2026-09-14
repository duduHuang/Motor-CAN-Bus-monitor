# gui/components/raw_can_console.py
import dearpygui.dearpygui as dpg
from core import ControlSnapshot

class RawCanConsoleView:
    """單軸原始 CAN 封包通訊 Log 終端元件 View (支援環形緩衝區無限更新)"""

    def __init__(self, key: str = ""):
        self.key = key
        self.tags = {
            "main_win": dpg.generate_uuid(),
            "container": dpg.generate_uuid(),
            "log_text": dpg.generate_uuid(),
            "auto_scroll": dpg.generate_uuid(),
            "clear_btn": dpg.generate_uuid(),
        }
        
        self._last_log_entry = None
        self._cleared_at_log = None

    def build(self):
        """建構 UI 佈局"""
        with dpg.child_window(tag=self.tags["main_win"], width=-1, height=-1, border=True):
            # 頂部控制列
            with dpg.group(horizontal=True):
                dpg.add_text("Raw CAN Console", color=(100, 200, 255))
                dpg.add_checkbox(label="Auto-scroll", default_value=True, tag=self.tags["auto_scroll"])
                dpg.add_button(label="Clear", tag=self.tags["clear_btn"], callback=self.clear_logs)

            # 滾動內容區
            with dpg.child_window(tag=self.tags["container"], width=-1, height=-1, border=True, horizontal_scrollbar=True):
                dpg.add_text("", tag=self.tags["log_text"])

    def on_resize(self, height: int):
        """動態調整 Console 主高度"""
        if dpg.does_item_exist(self.tags["main_win"]):
            dpg.configure_item(self.tags["main_win"], height=height)

    def clear_logs(self):
        """清空 UI 上的 Log 顯示"""
        # 記錄按下 Clear 當下最新的一筆 Log 作為分界點
        if self._last_log_entry is not None:
            self._cleared_at_log = self._last_log_entry
            
        if dpg.does_item_exist(self.tags["log_text"]):
            dpg.set_value(self.tags["log_text"], "")

    def update(self, snap: ControlSnapshot):
        """根據快照的 raw_can_logs 刷新通訊紀錄"""
        if not dpg.does_item_exist(self.tags["log_text"]):
            return

        logs = snap.raw_can_logs
        if not logs:
            return

        latest_log = logs[-1]

        # 只要最新的一筆 Log 與上次記錄不同，代表有新封包湧入（無視長度是否卡在 1000）
        if latest_log != self._last_log_entry:
            self._last_log_entry = latest_log

            # 若使用者按下過 Clear，計算從清空點之後產生的新 Log
            if self._cleared_at_log in logs:
                idx = logs.index(self._cleared_at_log)
                display_logs = logs[idx + 1:]
            else:
                # 若清空點已經被 deque 擠掉（或從未點擊 Clear），直接顯示緩衝區內的全部 Log
                self._cleared_at_log = None
                display_logs = logs

            log_str = "\n".join(display_logs)
            dpg.set_value(self.tags["log_text"], log_str)

            # 自動捲動置底
            if dpg.get_value(self.tags["auto_scroll"]):
                dpg.set_y_scroll(self.tags["container"], 999999.0)