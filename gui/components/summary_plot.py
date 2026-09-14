# gui/components/summary_plot.py
+"""
+AllAxesSummaryView Module
+多軸總覽波形 View 元件，顯示各軸的實際位置與目標位置波形，提供外部 (如 MultiMotorPanelView) 主動更新各軸波形數據的介面。
+"""
from typing import Dict, List
import dearpygui.dearpygui as dpg
from core import MultiMotorViewModelManager

class AllAxesSummaryView:
    """多軸總覽波形 View 元件"""

    def __init__(self, manager: MultiMotorViewModelManager, win_height: int = 800):
        self.manager = manager
        self.win_height = win_height
        self.tags = {
            "xaxis_pos": dpg.generate_uuid(),
            "yaxis_pos": dpg.generate_uuid(),
        }
        self.series_tags: Dict[str, Dict[str, int]] = {}
        self.axis_colors = [
            (255, 85, 85, 255),    # Red
            (85, 170, 255, 255),   # Blue
            (85, 255, 127, 255),   # Green
            (255, 170, 0, 255),    # Amber
            (255, 85, 255, 255),   # Magenta
        ]

    def build(self) -> None:
        """建構總覽圖表 UI 佈局"""
        with dpg.child_window(border = True):
            dpg.add_text("All-Axes Position Tracking Comparison", color = (100, 200, 255))
            dpg.add_separator()

            plot_h = max(300, int(self.win_height * 0.65))
            with dpg.plot(label = "Synchronous Position Waveforms", height = plot_h, width = -1):
                dpg.add_plot_legend()
                self.tags["xaxis_pos"] = dpg.add_plot_axis(dpg.mvXAxis, label = "Time (s)")
                self.tags["yaxis_pos"] = dpg.add_plot_axis(dpg.mvYAxis, label = "Position (rad)")

                all_vms = self.manager.get_all_vms() if self.manager else {}
                for idx, (key, vm) in enumerate(all_vms.items()):
                    color = self.axis_colors[idx % len(self.axis_colors)]

                    with dpg.theme() as t_act:
                        with dpg.theme_component(dpg.mvLineSeries):
                            dpg.add_theme_color(dpg.mvPlotCol_Line, color, category = dpg.mvThemeCat_Plots)

                    with dpg.theme() as t_des:
                        with dpg.theme_component(dpg.mvLineSeries):
                            dpg.add_theme_color(dpg.mvPlotCol_Line, (color[0], color[1], color[2], 120), category = dpg.mvThemeCat_Plots)

                    act_tag = dpg.add_line_series([], [], label = f"M{vm.motor_id} Act ({key})", parent = self.tags["yaxis_pos"])
                    des_tag = dpg.add_line_series([], [], label = f"M{vm.motor_id} Cmd ({key})", parent = self.tags["yaxis_pos"])

                    dpg.bind_item_theme(act_tag, t_act)
                    dpg.bind_item_theme(des_tag, t_des)

                    self.series_tags[key] = {"act": act_tag, "des": des_tag}

    def update_axis_series(self, key: str, times: List[float], p_act: List[float], p_des: List[float]) -> None:
        if key in self.series_tags:
            dpg.set_value(self.series_tags[key]["act"], [times, p_act])
            dpg.set_value(self.series_tags[key]["des"], [times, p_des])

    def set_x_limits(self, min_t: float, max_t: float) -> None:
        if dpg.does_item_exist(self.tags["xaxis_pos"]):
            dpg.set_axis_limits(self.tags["xaxis_pos"], min_t, max_t)