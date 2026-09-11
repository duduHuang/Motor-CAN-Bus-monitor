import dearpygui.dearpygui as dpg
from .base_ui import BaseProviderUI

class GenericProviderUI(BaseProviderUI):
    """通用渲染器：適用於 CSV, Sine, DB3 等參數型 Provider"""

    def build(self) -> None:
        for schema in self.schemas:
            def_val = schema.default_value if schema.default_value is not None else 0.0
            self.current_params[schema.name] = def_val

            if getattr(schema, "is_file_path", False):
                with dpg.group(horizontal=True, parent=self.parent_tag):
                    input_tag = dpg.add_input_text(
                        label=schema.display_name,
                        default_value=str(def_val),
                        width=170,
                        callback=lambda s, a, u: self.on_param_changed(u, a),
                        user_data=schema.name
                    )
                    dpg.add_button(
                        label="Browse...",
                        user_data=(schema.name, getattr(schema, "file_filter", ".*"), input_tag),
                        callback=lambda s, a, u: self.open_file_dialog_cb(u[0], u[1], u[2]) if self.open_file_dialog_cb else None
                    )
            elif schema.param_type == bool or isinstance(def_val, bool):
                dpg.add_checkbox(
                    label=schema.display_name,
                    default_value=bool(def_val),
                    parent=self.parent_tag,
                    callback=lambda s, a, u: self.on_param_changed(u, a),
                    user_data=schema.name
                )
            elif schema.param_type == str or isinstance(def_val, str):
                dpg.add_input_text(
                    label=schema.display_name,
                    default_value=str(def_val),
                    parent=self.parent_tag,
                    callback=lambda s, a, u: self.on_param_changed(u, a),
                    user_data=schema.name
                )
            elif schema.param_type == int or isinstance(def_val, int):
                min_v = int(schema.min_value) if getattr(schema, 'min_value', None) is not None else -100
                max_v = int(schema.max_value) if getattr(schema, 'max_value', None) is not None else 100
                dpg.add_slider_int(
                    label=schema.display_name,
                    default_value=int(def_val),
                    min_value=min_v,
                    max_value=max_v,
                    parent=self.parent_tag,
                    callback=lambda s, a, u: self.on_param_changed(u, a),
                    user_data=schema.name
                )
            else:
                min_v = float(schema.min_value) if getattr(schema, 'min_value', None) is not None else -100.0
                max_v = float(schema.max_value) if getattr(schema, 'max_value', None) is not None else 100.0
                dpg.add_slider_float(
                    label=schema.display_name,
                    default_value=float(def_val),
                    min_value=min_v,
                    max_value=max_v,
                    parent=self.parent_tag,
                    callback=lambda s, a, u: self.on_param_changed(u, a),
                    user_data=schema.name
                )