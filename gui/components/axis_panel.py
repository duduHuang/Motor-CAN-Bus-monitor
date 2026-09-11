import os
import time
from typing import Dict, Any, Callable
import dearpygui.dearpygui as dpg

from core import MotorControlViewModel, ControlSnapshot
from trajectory.factory import ProviderFactory
from gui.components.provider_ui import ProviderUIFactory

class AxisPanelView:
    """單軸左側控制與設定面板 View 元件"""

    def __init__(
        self, 
        key: str, 
        vm: MotorControlViewModel, 
        estop_theme: int, 
        show_report_cb: Callable[[str], None],
        set_error_msg_cb: Callable[[str], None],
        open_file_dialog_cb: Callable[[str, str, str, int], None]
    ):
        self.key = key
        self.vm = vm
        self.estop_theme = estop_theme
        self.show_report_cb = show_report_cb
        self.set_error_msg_cb = set_error_msg_cb
        self.open_file_dialog_cb = open_file_dialog_cb

        self.current_params: Dict[str, Any] = {}
        self.active_provider_ui = None  # 保存當前 ProviderUI 實例
        self.tags = {
            "provider_combo": dpg.generate_uuid(),
            "params_group": dpg.generate_uuid(),
            "auto_scroll": dpg.generate_uuid(),
            "window_size": dpg.generate_uuid(),
        }

    def _get_or_init_default_provider_name(self) -> str:
        curr_provider = self.vm.get_current_provider()
        if curr_provider is None:
            available = self.vm.get_available_providers()
            default_p = "Manual" if "Manual" in available else (available[0] if available else "Manual")
            self.vm.select_provider(default_p)
            curr_provider = self.vm.get_current_provider()
        
        if curr_provider:
            return curr_provider.__class__.__name__.replace("Provider", "")
        return "Manual"

    def build(self):
        default_provider_name = self._get_or_init_default_provider_name()

        with dpg.child_window(width=350, border=True):
            dpg.add_text("Control & Settings", color=(100, 200, 255))
            dpg.add_separator()

            dpg.add_input_int(
                label="Motor ID",
                default_value=getattr(self.vm, "motor_id", 1),
                callback=lambda s, a: setattr(self.vm, 'motor_id', a)
            )

            dpg.add_combo(
                label="Provider",
                items=self.vm.get_available_providers() or ["Manual"],
                default_value=default_provider_name,
                tag=self.tags["provider_combo"],
                callback=self._on_provider_changed
            )

            dpg.add_spacer(height=10)
            dpg.add_text("Tuning Parameters:")
            dpg.add_group(tag=self.tags["params_group"])

            dpg.add_spacer(height=10)
            dpg.add_separator()
            dpg.add_text("Plot Settings:")
            dpg.add_checkbox(label="Auto Scroll X-Axis", default_value=True, tag=self.tags["auto_scroll"])
            dpg.add_slider_float(label="Window Size (s)", default_value=10.0, min_value=2.0, max_value=30.0, tag=self.tags["window_size"])

            dpg.add_spacer(height=10)
            dpg.add_separator()
            dpg.add_text("Data & Analysis:")
            with dpg.group(horizontal=True):
                dpg.add_button(label="Export CSV", width=95, height=30, callback=self._on_export_csv)
                dpg.add_button(label="Report", width=95, height=30, callback=lambda: self.show_report_cb(self.key))
                dpg.add_button(label="Save PNG", width=95, height=30, callback=self._on_save_plots_image)

            dpg.add_spacer(height=20)
            dpg.add_separator()

            with dpg.group(horizontal=True):
                dpg.add_button(label="START", width=100, height=40, callback=self._on_start_control)
                dpg.add_button(label="STOP", width=100, height=40, callback=lambda: self.vm.stop_control())

            dpg.add_spacer(height=10)
            estop_btn = dpg.add_button(
                label="EMERGENCY STOP",
                width=210,
                height=50,
                callback=lambda: self.vm.trigger_estop(f"UI E-STOP [{self.key}]")
            )
            dpg.bind_item_theme(estop_btn, self.estop_theme)

        self._rebuild_provider_params(default_provider_name)

    def get_plot_settings(self) -> Dict[str, Any]:
        return {
            "auto_scroll": dpg.get_value(self.tags["auto_scroll"]) if dpg.does_item_exist(self.tags["auto_scroll"]) else True,
            "window_size": dpg.get_value(self.tags["window_size"]) if dpg.does_item_exist(self.tags["window_size"]) else 10.0
        }

    def update(self, snap: ControlSnapshot):
        """每影格將數據快照傳遞給當前的 ProviderUI，實現畫面上數據卡片的即時刷新"""
        if self.active_provider_ui:
            self.active_provider_ui.update(snap)

    def _on_provider_changed(self, sender, app_data):
        self.vm.stop_control()
        self.vm.select_provider(app_data)
        self._rebuild_provider_params(app_data)

    def _rebuild_provider_params(self, provider_name: str):
        if not dpg.does_item_exist(self.tags["params_group"]):
            return

        # 1. 清空既有 UI 內容
        dpg.delete_item(self.tags["params_group"], children_only=True)
        schema_list = ProviderFactory.get_provider_schema(provider_name) or []
        self.current_params.clear()

        # 2. 透過 Factory 實例化專屬 UI 渲染器並保存至 self.active_provider_ui
        self.active_provider_ui = ProviderUIFactory.create_ui(
            provider_name=provider_name,
            parent_tag=self.tags["params_group"],
            schemas=schema_list,
            current_params=self.current_params,
            on_param_changed=self._on_param_changed_from_ui,
            open_file_dialog_cb=self._on_browse_clicked_from_ui,
            vm_key=self.key,
            motor_id=getattr(self.vm, "motor_id", 1),
            vm=self.vm
        )

        # 3. 執行繪製
        self.active_provider_ui.build()

        # 4. 初始化路徑驗證
        self._validate_file_paths(is_starting=False)

    def _on_param_changed_from_ui(self, param_name: str, value: Any):
        """提供給內部 UI 呼叫的回呼函式"""
        self.current_params[param_name] = value
        self.vm.trigger_provider_action("update_targets", **self.current_params)
        self._validate_file_paths(is_starting=False)

    def _on_browse_clicked_from_ui(self, param_name: str, file_filter: str, input_tag: int):
        if self.open_file_dialog_cb:
            self.open_file_dialog_cb(self.key, param_name, file_filter, input_tag)

    def _validate_file_paths(self, is_starting: bool = False) -> bool:
        """
        驗證目前 Provider 是否有選定必填檔案。
        - 若 is_starting=False，僅驗證路徑是否存在，避免無故清空其他 UI 錯誤警示。
        - 若 is_starting=True，缺檔時會將錯誤拋至 UI 阻止啟動。
        """
        provider_name = dpg.get_value(self.tags["provider_combo"]) if dpg.does_item_exist(self.tags["provider_combo"]) else None
        if not provider_name:
            curr_provider = self.vm.get_current_provider()
            if not curr_provider:
                return True
            provider_name = curr_provider.__class__.__name__.replace("Provider", "")

        schemas = ProviderFactory.get_provider_schema(provider_name) or []

        for schema in schemas:
            if getattr(schema, "is_file_path", False):
                filepath = str(self.current_params.get(schema.name, "")).strip()
                
                if not filepath:
                    if is_starting:
                        self.set_error_msg_cb(f"Missing File: '{schema.display_name}' is empty.")
                    return False
                
                elif not os.path.exists(filepath):
                    self.set_error_msg_cb(f"File Not Found: {filepath}")
                    return False

        # 通過驗證時，僅在點擊 START 時清除先前的檔案錯誤訊息
        if is_starting:
            self.set_error_msg_cb("")
        return True

    def _on_start_control(self):
        if not self._validate_file_paths(is_starting=True):
            return
        if not self.vm.start_control():
            self.set_error_msg_cb("Start failed: System already running or in error state.")

    def _on_export_csv(self):
        filename = f"motor_{self.key}_log_{int(time.time())}.csv"
        try:
            self.vm.save_csv_log(filename)
            self.set_error_msg_cb(f"Log successfully exported: {filename}")
        except Exception as e:
            self.set_error_msg_cb(f"CSV export failed: {e}")

    def _on_save_plots_image(self):
        filename = f"waveform_{self.key}_{int(time.time())}.png"
        try:
            import matplotlib.pyplot as plt
            with self.vm._history_lock:
                if not self.vm._history_buffer:
                    self.set_error_msg_cb("No waveform history data available to save!")
                    return
                history = list(self.vm._history_buffer)
            times = [h[0] for h in history]
            p_des = [h[1] for h in history]
            p_act = [h[2] for h in history]
            v_des = [h[3] for h in history]
            v_act = [h[4] for h in history]
            trq_act = [h[5] for h in history]
            snap = self.vm.get_ui_snapshot()
            trq_ff = [snap.t_ff] * len(times)

            fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
            ax1.plot(times, p_des, 'r--', label='p_des (Command)')
            ax1.plot(times, p_act, 'b-', label='p_act (Actual)')
            ax1.set_ylabel('Position (rad)')
            ax1.set_title(f'Motor [{self.key}] Waveform Snapshot')
            ax1.grid(True)
            ax1.legend(loc='upper right')

            ax2.plot(times, v_des, 'r--', label='v_des (Command)')
            ax2.plot(times, v_act, 'g-', label='v_act (Actual)')
            ax2.set_ylabel('Velocity (rad/s)')
            ax2.grid(True)
            ax2.legend(loc='upper right')

            ax3.plot(times, trq_ff, color='orange', linestyle='--', label='t_ff (Feedforward)')
            ax3.plot(times, trq_act, 'm-', label='t_act (Actual)')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Torque (Nm)')
            ax3.grid(True)
            ax3.legend(loc='upper right')

            plt.tight_layout()
            plt.savefig(filename, dpi=150)
            plt.close(fig)
            self.set_error_msg_cb(f"3-in-1 waveform saved as: {filename}")
            return
        except ImportError:
            pass
        except Exception as e:
            print(f"[Matplotlib Export Failed] {e}")

        try:
            dpg.output_frame_buffer(filename)
            self.set_error_msg_cb(f"Window screenshot saved as: {filename}")
        except Exception as e:
            self.set_error_msg_cb(f"Failed to save image: {e}")