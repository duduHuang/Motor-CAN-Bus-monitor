# gui/components/axis_plots.py
+"""
+AxisPlotsView Module
+單軸跟隨波形面板 View 元件 (包含 3 組 DPG Plot)，提供即時刷新波形數據與座標軸自適應功能，並支援自動捲動或自動縮放 X 軸
+"""
from typing import List, Tuple, Dict
import dearpygui.dearpygui as dpg

class AxisPlotsView:
    """單軸跟隨波形面板 View 元件 (包含 3 組 DPG Plot)"""

    def __init__(self, key: str, plot_height: int = 160, themes: Dict[str, int] = None):
        self.key = key
        self.plot_height = plot_height
        self.themes = themes if themes is not None else {}
        self.tags = {
            "plot_pos": dpg.generate_uuid(), #  圖表 1 Tag
            "plot_vel": dpg.generate_uuid(), #  圖表 2 Tag
            "plot_trq": dpg.generate_uuid(), #  圖表 3 Tag
            "xaxis_pos": dpg.generate_uuid(), "yaxis_pos": dpg.generate_uuid(),
            "xaxis_vel": dpg.generate_uuid(), "yaxis_vel": dpg.generate_uuid(),
            "xaxis_trq": dpg.generate_uuid(), "yaxis_trq": dpg.generate_uuid(),
            "series_pos_des": dpg.generate_uuid(), "series_pos_act": dpg.generate_uuid(),
            "series_vel_des": dpg.generate_uuid(), "series_vel_act": dpg.generate_uuid(),
            "series_trq_ff": dpg.generate_uuid(),  "series_trq_act": dpg.generate_uuid(),
        }

    def build(self):
        """建構 3 合 1 波形圖表佈局"""
        # 1. Position Tracking
        with dpg.plot(label="Position Tracking (rad)", height=self.plot_height, width=-1):
            dpg.add_plot_legend()
            self.tags["xaxis_pos"] = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)")
            self.tags["yaxis_pos"] = dpg.add_plot_axis(dpg.mvYAxis, label="Position")
            dpg.add_line_series([], [], label="Command (p_des)", parent=self.tags["yaxis_pos"], tag=self.tags["series_pos_des"])
            dpg.add_line_series([], [], label="Actual (p_act)", parent=self.tags["yaxis_pos"], tag=self.tags["series_pos_act"])
            if "theme_p_des" in self.themes: dpg.bind_item_theme(self.tags["series_pos_des"], self.themes["theme_p_des"])
            if "theme_p_act" in self.themes: dpg.bind_item_theme(self.tags["series_pos_act"], self.themes["theme_p_act"])

        # 2. Velocity Tracking
        with dpg.plot(label="Velocity Tracking (rad/s)", height=self.plot_height, width=-1):
            dpg.add_plot_legend()
            self.tags["xaxis_vel"] = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)")
            self.tags["yaxis_vel"] = dpg.add_plot_axis(dpg.mvYAxis, label="Velocity")
            dpg.add_line_series([], [], label="Command (v_des)", parent=self.tags["yaxis_vel"], tag=self.tags["series_vel_des"])
            dpg.add_line_series([], [], label="Actual (v_act)", parent=self.tags["yaxis_vel"], tag=self.tags["series_vel_act"])
            if "theme_v_des" in self.themes: dpg.bind_item_theme(self.tags["series_vel_des"], self.themes["theme_v_des"])
            if "theme_v_act" in self.themes: dpg.bind_item_theme(self.tags["series_vel_act"], self.themes["theme_v_act"])

        # 3. Torque Dynamic
        with dpg.plot(label="Torque Dynamic (Nm)", height=self.plot_height, width=-1):
            dpg.add_plot_legend()
            self.tags["xaxis_trq"] = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)")
            self.tags["yaxis_trq"] = dpg.add_plot_axis(dpg.mvYAxis, label="Torque")
            dpg.add_line_series([], [], label="Feedforward (t_ff)", parent=self.tags["yaxis_trq"], tag=self.tags["series_trq_ff"])
            dpg.add_line_series([], [], label="Actual (t_act)", parent=self.tags["yaxis_trq"], tag=self.tags["series_trq_act"])
            if "theme_t_ff" in self.themes: dpg.bind_item_theme(self.tags["series_trq_ff"], self.themes["theme_t_ff"])
            if "theme_t_act" in self.themes: dpg.bind_item_theme(self.tags["series_trq_act"], self.themes["theme_t_act"])

    def on_resize(self, plot_height: int):
        """動態調整三組波形圖的高度"""
        for p_key in ["plot_pos", "plot_vel", "plot_trq"]:
            if dpg.does_item_exist(self.tags[p_key]):
                dpg.configure_item(self.tags[p_key], height=plot_height)

    def update(
        self, 
        history: List[Tuple], 
        t_ff: float = 0.0, 
        auto_scroll: bool = True, 
        window_size: float = 10.0
    ) -> Tuple[List[float], List[float]]:
        """刷新波形數據與座標軸 Range"""
        if not history:
            return [], []

        times = [h[0] for h in history]
        p_des = [h[1] for h in history]
        p_act = [h[2] for h in history]
        v_des = [h[3] for h in history]
        v_act = [h[4] for h in history]
        trq_act = [h[5] for h in history]
        trq_ff = [t_ff] * len(times)

        dpg.set_value(self.tags["series_pos_des"], [times, p_des])
        dpg.set_value(self.tags["series_pos_act"], [times, p_act])
        dpg.set_value(self.tags["series_vel_des"], [times, v_des])
        dpg.set_value(self.tags["series_vel_act"], [times, v_act])
        dpg.set_value(self.tags["series_trq_ff"], [times, trq_ff])
        dpg.set_value(self.tags["series_trq_act"], [times, trq_act])

        # Y 軸 Range 自適應
        p_combined = p_des + p_act
        if p_combined:
            dpg.set_axis_limits(self.tags["yaxis_pos"], min(p_combined) - 0.1, max(p_combined) + 0.1)
        v_combined = v_des + v_act
        if v_combined:
            dpg.set_axis_limits(self.tags["yaxis_vel"], min(v_combined) - 0.1, max(v_combined) + 0.1)
        t_combined = trq_ff + trq_act
        if t_combined:
            dpg.set_axis_limits(self.tags["yaxis_trq"], min(t_combined) - 0.1, max(t_combined) + 0.1)

        # X 軸 Auto Scroll / Fit
        if auto_scroll and times:
            cur_t = times[-1]
            min_t = max(0.0, float(cur_t - window_size))
            max_t = max(float(window_size), float(cur_t))

            dpg.set_axis_limits(self.tags["xaxis_pos"], min_t, max_t)
            dpg.set_axis_limits(self.tags["xaxis_vel"], min_t, max_t)
            dpg.set_axis_limits(self.tags["xaxis_trq"], min_t, max_t)
        else:
            dpg.set_axis_limits_auto(self.tags["xaxis_pos"])
            dpg.set_axis_limits_auto(self.tags["xaxis_vel"])
            dpg.set_axis_limits_auto(self.tags["xaxis_trq"])

        return times, p_act