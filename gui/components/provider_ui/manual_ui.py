# gui/components/provider_ui/manual_ui.py
+"""
+ManualProviderUI Module
+專屬於 Manual Provider 的 UI 元件，提供即時控制滑桿與快捷微調按鈕，方便使用者直接調整目標位置、速度、剛度、阻尼與前饋力矩。
+"""
import dearpygui.dearpygui as dpg
from .base_ui import BaseProviderUI

class ManualProviderUI(BaseProviderUI):
    """Manual 專屬 UI：包含即時控制滑桿與快捷微調按鈕"""

    def build(self) -> None:
        dpg.add_text("Manual Direct Tuning", color=(255, 180, 0), parent=self.parent_tag)
        dpg.add_separator(parent=self.parent_tag)

        # 預設與初始化數值
        for schema in self.schemas:
            self.current_params[schema.name] = float(schema.default_value or 0.0)

        # 目標點控制滑桿 (即時響應)
        dpg.add_slider_float(
            label="Target Pos (rad)", default_value=self.current_params.get("p_des", 0.0),
            min_value=-3.14, max_value=3.14, parent=self.parent_tag,
            callback=lambda s, a, u: self.on_param_changed(u, a), user_data="p_des"
        )
        dpg.add_slider_float(
            label="Target Vel (rad/s)", default_value=self.current_params.get("v_des", 0.0),
            min_value=-10.0, max_value=10.0, parent=self.parent_tag,
            callback=lambda s, a, u: self.on_param_changed(u, a), user_data="v_des"
        )
        dpg.add_slider_float(
            label="Stiffness (Kp)", default_value=self.current_params.get("kp", 20.0),
            min_value=0.0, max_value=100.0, parent=self.parent_tag,
            callback=lambda s, a, u: self.on_param_changed(u, a), user_data="kp"
        )
        dpg.add_slider_float(
            label="Damping (Kd)", default_value=self.current_params.get("kd", 1.0),
            min_value=0.0, max_value=10.0, parent=self.parent_tag,
            callback=lambda s, a, u: self.on_param_changed(u, a), user_data="kd"
        )
        dpg.add_slider_float(
            label="Feedforward (Nm)", default_value=self.current_params.get("t_ff", 0.0),
            min_value=-5.0, max_value=5.0, parent=self.parent_tag,
            callback=lambda s, a, u: self.on_param_changed(u, a), user_data="t_ff"
        )

        dpg.add_spacer(height=8, parent=self.parent_tag)

        # 快捷功能按鈕區
        with dpg.group(horizontal=True, parent=self.parent_tag):
            dpg.add_button(
                label="Zero Pos", width=90,
                callback=lambda: self.on_param_changed("p_des", 0.0)
            )
            dpg.add_button(
                label="Zero All", width=90,
                callback=self._reset_all_zero
            )

    def _reset_all_zero(self):
        for key in ["p_des", "v_des", "t_ff"]:
            self.on_param_changed(key, 0.0)