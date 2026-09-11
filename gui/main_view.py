# gui/main_view.py
import time
from typing import Dict, Any
import dearpygui.dearpygui as dpg

from core import SystemStatus, MultiMotorViewModelManager
from gui.utils import get_screen_resolution
from gui.components.master_header import MasterHeaderView
from gui.components.axis_panel import AxisPanelView

# === 移除舊的 AxisStatsView，匯入新的 Dashboard 與 Console ===
from gui.components.axis_dashboard import AxisDashboardView
from gui.components.raw_can_console import RawCanConsoleView

from gui.components.axis_plots import AxisPlotsView
from gui.components.summary_plot import AllAxesSummaryView
from gui.components.modals import GlobalEstopModal, QualityReportModal

class MotorControlView:
    """多軸馬達控制 GUI 主視窗視圖與元件彙整器"""

    def __init__(self, manager: MultiMotorViewModelManager):
        self.manager = manager
        self.is_running = True

        screen_w, screen_h = get_screen_resolution()
        self.win_width = int(screen_w * 0.85)
        self.win_height = int(screen_h * 0.85)
        self.base_title = "Motor Control Validation GUI (Multi-Axis)"
        self.last_win_width = self.win_width
        self.last_win_height = self.win_height

        # 稍微調整 plot_height，為底部的 Raw CAN Console (250px) 與頂部的 Dashboard (80px) 預留足夠高度
        self.plot_height = max(130, int((self.win_height - 480) / 3))

        self._setup_dpg()

        self.file_dialog_tag = dpg.generate_uuid()
        self._active_dialog_context: Dict[str, Any] = {
            "axis_key": None,
            "param_name": None,
            "input_tag": None
        }

        self.master_header = MasterHeaderView(self.manager, self.estop_theme)
        self.global_estop_modal = GlobalEstopModal()
        self.summary_view = AllAxesSummaryView(self.manager, self.win_height)

        self.axis_panel_views: Dict[str, AxisPanelView] = {}
        # 初始化新的 Dashboard 與 Console 字典
        self.axis_dashboard_views: Dict[str, AxisDashboardView] = {}
        self.raw_can_consoles: Dict[str, RawCanConsoleView] = {}
        self.axis_plot_views: Dict[str, AxisPlotsView] = {}
        self.report_modals: Dict[str, QualityReportModal] = {}

        self._pending_global_estop_reason = None

        for key, vm in self.manager.get_all_vms().items():
            # 實例化新的 UI 元件
            dash_v = AxisDashboardView(key)
            console_v = RawCanConsoleView(key)
            
            panel_v = AxisPanelView(
                key=key,
                vm=vm,
                estop_theme=self.estop_theme,
                show_report_cb=self._on_show_report_requested,
                # 將錯誤訊息綁回該軸專屬的 Dashboard 上
                set_error_msg_cb=lambda msg, d=dash_v: d.set_error_message(msg),
                open_file_dialog_cb=self._open_file_dialog
            )
            plots_v = AxisPlotsView(key=key, plot_height=self.plot_height, themes=self.plot_themes)
            report_m = QualityReportModal(key)

            self.axis_dashboard_views[key] = dash_v
            self.raw_can_consoles[key] = console_v
            self.axis_panel_views[key] = panel_v
            self.axis_plot_views[key] = plots_v
            self.report_modals[key] = report_m

        self._build_ui()
        self.manager.on_global_estop = self._on_global_estop_triggered

    def _setup_dpg(self):
        dpg.create_context()
        dpg.create_viewport(
            title=f"{self.base_title} ({self.win_width}x{self.win_height})",
            width=self.win_width,
            height=self.win_height,
            min_width=1024,
            min_height=600,
            resizable=True
        )
        dpg.setup_dearpygui()

        with dpg.theme() as self.estop_theme:
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (200, 40, 40, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 60, 60, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (150, 30, 30, 255))

        self.plot_themes = {}
        for theme_name, color in [
            ("theme_p_des", (255, 50, 50, 200)),
            ("theme_p_act", (50, 150, 255, 255)),
            ("theme_v_des", (255, 50, 50, 200)),
            ("theme_v_act", (50, 255, 50, 255)),
            ("theme_t_ff",  (255, 165, 0, 200)),
            ("theme_t_act", (255, 0, 255, 255)),
        ]:
            with dpg.theme() as t:
                with dpg.theme_component(dpg.mvLineSeries):
                    dpg.add_theme_color(dpg.mvPlotCol_Line, color, category=dpg.mvThemeCat_Plots)
            self.plot_themes[theme_name] = t

        with dpg.handler_registry():
            dpg.add_key_press_handler(callback=self._on_global_key_pressed)

    def _on_global_key_pressed(self, sender, app_data):
        focused_item = dpg.get_focused_item()
        if focused_item:
            item_type = dpg.get_item_type(focused_item)
            if "Input" in item_type or "input" in item_type:
                return

        key_code = app_data[0] if isinstance(app_data, (list, tuple)) else app_data
        if not isinstance(key_code, int):
            return

        dpg_key_map = {
            265: "up",     
            264: "down",   
            263: "left",   
            262: "right",  
        }

        if key_code in dpg_key_map:
            key_char = dpg_key_map[key_code]
        elif 32 < key_code <= 126:
            key_char = chr(key_code).lower()
        else:
            return

        for vm in self.manager.get_all_vms().values():
            vm.trigger_provider_action("key", key=key_char)

    def _build_ui(self):
        with dpg.file_dialog(
            directory_selector=False,
            show=False,
            callback=self._on_file_selected,
            cancel_callback=lambda s, a: None,
            width=650,
            height=400,
            tag=self.file_dialog_tag,
            modal=True
        ):
            dpg.add_file_extension(".*", color=(255, 255, 255, 255))
            dpg.add_file_extension(".csv", color=(0, 255, 0, 255), custom_text="[CSV]")
            dpg.add_file_extension(".db3", color=(0, 255, 255, 255), custom_text="[DB3]")

        with dpg.window(label="Main Window", no_collapse=True, no_close=True, no_title_bar=True) as main_win:
            self.master_header.build()
            dpg.add_spacer(height=5)

            with dpg.tab_bar():
                for key, vm in self.manager.get_all_vms().items():
                    tab_label = f"Motor {vm.motor_id} ({getattr(vm, 'channel', 'CAN')})"
                    with dpg.tab(label=tab_label):
                        # === 版面重構：左側設定面板，右側為資料監控群組 ===
                        with dpg.group(horizontal=True):
                            # [左側] 操作控制面板
                            self.axis_panel_views[key].build()
                            
                            # [右側] 監控資料上下結構
                            with dpg.group():
                                # 1. 頂部：新的狀態看板
                                self.axis_dashboard_views[key].build()
                                
                                # 2. 中間：繪圖與歷史曲線
                                self.axis_plot_views[key].build()
                                    
                                # 3. 底部：通訊 Console Log
                                self.raw_can_consoles[key].build()

                with dpg.tab(label="All-Axes Summary"):
                    self.summary_view.build()

        self.global_estop_modal.build()
        for modal in self.report_modals.values():
            modal.build()

        dpg.set_primary_window(main_win, True)

    def _open_file_dialog(self, axis_key: str, param_name: str, file_filter: str, input_tag: int):
        self._active_dialog_context = {
            "axis_key": axis_key,
            "param_name": param_name,
            "input_tag": input_tag
        }
        dpg.show_item(self.file_dialog_tag)

    def _on_file_selected(self, sender, app_data):
        file_path = app_data.get("file_path_name", "")
        if not file_path:
            return

        axis_key = self._active_dialog_context.get("axis_key")
        param_name = self._active_dialog_context.get("param_name")
        input_tag = self._active_dialog_context.get("input_tag")

        if axis_key and param_name and input_tag:
            dpg.set_value(input_tag, file_path)
            if axis_key in self.axis_panel_views:
                self.axis_panel_views[axis_key]._on_param_changed_from_ui(param_name, file_path)

    def _on_global_estop_triggered(self, reason: str):
        self._pending_global_estop_reason = reason

    def _on_show_report_requested(self, key: str):
        vm = self.manager.get_vm(key)
        if vm:
            report_data = vm.generate_quality_report()
            snap = vm.get_ui_snapshot()
            self.report_modals[key].show(report_data, snap.actual_hz)

    def render_loop(self):
        dpg.show_viewport()
        target_fps = 60.0
        frame_time = 1.0 / target_fps

        while dpg.is_dearpygui_running() and self.is_running:
            start_t = time.time()
            self._update_data_binding()
            dpg.render_dearpygui_frame()
            elapsed = time.time() - start_t
            if elapsed < frame_time:
                time.sleep(frame_time - elapsed)

        dpg.destroy_context()

    def _update_data_binding(self):
        if self._pending_global_estop_reason:
            reason = self._pending_global_estop_reason
            self._pending_global_estop_reason = None
            self.global_estop_modal.show(reason)

        cur_w, cur_h = dpg.get_viewport_width(), dpg.get_viewport_height()
        if (cur_w, cur_h) != (self.last_win_width, self.last_win_height):
            self.last_win_width, self.last_win_height = cur_w, cur_h
            dpg.set_viewport_title(f"{self.base_title} ({cur_w}x{cur_h})")

        self.master_header.update()

        for key, vm in self.manager.get_all_vms().items():
            snap = vm.get_ui_snapshot()
            
            # === 更新新的 UI 元件快照 ===
            self.axis_dashboard_views[key].update(snap)
            self.raw_can_consoles[key].update(snap)
            self.axis_panel_views[key].update(snap)

            if snap.status in (SystemStatus.RUNNING, SystemStatus.PROBING):
                with vm._history_lock:
                    history = list(vm._history_buffer)[-1200:]

                plot_opts = self.axis_panel_views[key].get_plot_settings()
                times, p_act = self.axis_plot_views[key].update(
                    history=history,
                    t_ff=snap.t_ff,
                    auto_scroll=plot_opts["auto_scroll"],
                    window_size=plot_opts["window_size"]
                )

                if times:
                    p_des = [h[1] for h in history]
                    self.summary_view.update_axis_series(key, times, p_act, p_des)

                    if plot_opts["auto_scroll"]:
                        cur_t = times[-1]
                        w_s = plot_opts["window_size"]
                        self.summary_view.set_x_limits(max(0.0, float(cur_t - w_s)), max(float(w_s), float(cur_t)))