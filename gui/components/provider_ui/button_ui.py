# gui/components/provider_ui/button_ui.py
"""
ButtonProviderUI Module
專屬於 Button Provider 的 UI 元件，提供即時狀態�顯示與十字方向控制按鈕面板。
"""
import dearpygui.dearpygui as dpg
from .base_ui import BaseProviderUI
from core import ControlSnapshot


class ButtonProviderUI(BaseProviderUI):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.tags = {
            "val_step_pos": dpg.generate_uuid(),
            "val_step_vel": dpg.generate_uuid(),
            "val_kp": dpg.generate_uuid(),
            "val_kd": dpg.generate_uuid(),
            "val_pos_state": dpg.generate_uuid(),
            "val_vel_state": dpg.generate_uuid(),
        }

    def build(self) -> None:
        dpg.add_text("Button Control", color=(255, 200, 100), parent=self.parent_tag)
        dpg.add_separator(parent=self.parent_tag)

        # 1. 跨度調整 Slider
        def_step_pos = float(self.current_params.get("step_pos", 0.02))
        def_step_vel = float(self.current_params.get("step_vel", 0.1))

        dpg.add_slider_float(
            label="Pos Step (rad)",
            default_value=def_step_pos,
            min_value=0.001, max_value=0.2,
            parent=self.parent_tag,
            callback=lambda s, a, u: self.on_param_changed("step_pos", a)
        )
        dpg.add_slider_float(
            label="Vel Step (rad/s)",
            default_value=def_step_vel,
            min_value=0.01, max_value=1.0,
            parent=self.parent_tag,
            callback=lambda s, a, u: self.on_param_changed("step_vel", a)
        )

        dpg.add_spacer(height=5, parent=self.parent_tag)

        # 2. 即時狀態卡片 (顯示跨度數值、Kp、Kd 及極時位置速度)
        with dpg.child_window(height=110, border=True, parent=self.parent_tag):
            dpg.add_text("Real-time Status:", color=(100, 200, 255))
            dpg.add_text(" Pos Step Span : 0.0200 rad", tag=self.tags["val_step_pos"])
            dpg.add_text(" Vel Step Span : 0.1000 rad/s", tag=self.tags["val_step_vel"])
            dpg.add_text(" Stiffness (Kp): 40.0", tag=self.tags["val_kp"])
            dpg.add_text(" Damping   (Kd): 2.0", tag=self.tags["val_kd"])
            dpg.add_text(" Target Pos    : +0.0000 rad", tag=self.tags["val_pos_state"])
            dpg.add_text(" Target Vel    : +0.0000 rad/s", tag=self.tags["val_vel_state"])

        dpg.add_spacer(height=8, parent=self.parent_tag)

        # 3. 十字方向控制鈕面板 (上下為位置，左右為速度)
        dpg.add_text("Direction Control (Position / Velocity):", color=(255, 255, 100), parent=self.parent_tag)
        
        # 上按鈕: Position +
        with dpg.group(horizontal=True, parent=self.parent_tag):
            dpg.add_spacer(width=105)
            dpg.add_button(
                label="Up Pos + (W)", width=110, height=32,
                callback=lambda: self._trigger_action("pos_up")
            )

        # 中間排: Left (Vel -) | Down (Pos -) | Right (Vel +)
        with dpg.group(horizontal=True, parent=self.parent_tag):
            dpg.add_button(
                label="Up Vel - (A)", width=100, height=32,
                callback=lambda: self._trigger_action("vel_dec")
            )
            dpg.add_button(
                label="Down Pos - (S)", width=110, height=32,
                callback=lambda: self._trigger_action("pos_down")
            )
            dpg.add_button(
                label="Down Vel + (D)", width=100, height=32,
                callback=lambda: self._trigger_action("vel_inc")
            )

        dpg.add_spacer(height=8, parent=self.parent_tag)

        # 4. Kp / Kd 快捷調整按鈕
        dpg.add_text("Gain Control (Kp / Kd):", color=(100, 255, 100), parent=self.parent_tag)
        with dpg.group(horizontal=True, parent=self.parent_tag):
            dpg.add_button(label="[1] Kp+", width=65, callback=lambda: self._trigger_action("kp_inc"))
            dpg.add_button(label="[2] Kp-", width=65, callback=lambda: self._trigger_action("kp_dec"))
            dpg.add_button(label="[3] Kd+", width=65, callback=lambda: self._trigger_action("kd_inc"))
            dpg.add_button(label="[4] Kd-", width=65, callback=lambda: self._trigger_action("kd_dec"))

    def _trigger_action(self, action: str):
        if self.vm:
            self.vm.trigger_provider_action(action)

    def update(self, snap: ControlSnapshot) -> None:
        """即時刷新數值顯示"""
        if not self.vm:
            return

        provider = self.vm.get_current_provider()
        if provider and hasattr(provider, "step_pos"):
            if dpg.does_item_exist(self.tags["val_step_pos"]):
                dpg.set_value(self.tags["val_step_pos"], f" Pos Step Span : {provider.step_pos:.4f} rad")
                dpg.set_value(self.tags["val_step_vel"], f" Vel Step Span : {provider.step_vel:.4f} rad/s")
                dpg.set_value(self.tags["val_kp"], f" Stiffness (Kp): {provider.kp:.1f}")
                dpg.set_value(self.tags["val_kd"], f" Damping   (Kd): {provider.kd:.2f}")
                dpg.set_value(self.tags["val_pos_state"], f" Target Pos    : {provider.p_des:+.4f} rad (Act: {snap.p_act:+.4f})")
                dpg.set_value(self.tags["val_vel_state"], f" Target Vel    : {provider.v_des:+.4f} rad/s (Act: {snap.v_act:+.4f})")