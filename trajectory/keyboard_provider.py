from typing import Any, Dict, List, Tuple, Union
from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint


class AxisState:
    """單軸狀態封裝容器"""
    def __init__(self, name: str, pos: float = 0.0, kp: float = 40.0, kd: float = 2.0) -> None:
        self.name = name
        self.pos = pos
        self.kp = kp
        self.kd = kd


class KeyboardTrajectoryProvider(BaseTrajectoryProvider):
    """通用型 1~3 軸 (HipX / HipY / Knee) 鍵盤即時調參軌跡與增益提供者。"""

    def __init__(
        self,
        num_axes: int = 2,
        step_pos: float = 0.02,
        step_kp: float = 1.0,
        step_kd: float = 0.1,
        m1_kp: float = 42.0,
        m1_kd: float = 2.2,
        m2_kp: float = 40.0,
        m2_kd: float = 2.6,
        m3_kp: float = 40.0,
        m3_kd: float = 2.6,
    ) -> None:
        self.num_axes = max(1, min(3, int(num_axes)))  # 限制在 1~3 軸

        # 微調步進跨度
        self.step_pos = step_pos
        self.step_kp = step_kp
        self.step_kd = step_kd

        # 初始化 1~3 軸狀態
        default_names = ["FR_HipY", "FR_Knee", "FR_HipX"]
        default_kps = [m1_kp, m2_kp, m3_kp]
        default_kds = [m1_kd, m2_kd, m3_kd]

        self.axes: List[AxisState] = [
            AxisState(
                name=default_names[i],
                pos=0.0,
                kp=default_kps[i],
                kd=default_kds[i],
            )
            for i in range(self.num_axes)
        ]

        # 控制開關狀態
        self.gravity_ff_enabled: bool = True
        self.e_stop_triggered: bool = False

        # 3 軸熱鍵映射表: key -> (axis_index, attribute, direction)
        self.key_map: Dict[str, Tuple[int, str, int]] = {
            # Motor 1 (M1 - 預設 FR_HipY)
            'w': (0, 'pos', 1),  's': (0, 'pos', -1),
            '1': (0, 'kp',  1),  '2': (0, 'kp',  -1),
            '5': (0, 'kd',  1),  '6': (0, 'kd',  -1),

            # Motor 2 (M2 - 預設 FR_Knee)
            'a': (1, 'pos', 1),  'd': (1, 'pos', -1),
            '3': (1, 'kp',  1),  '4': (1, 'kp',  -1),
            '7': (1, 'kd',  1),  '8': (1, 'kd',  -1),

            # Motor 3 (M3 - 預設 FR_HipX)
            'i': (2, 'pos', 1),  'k': (2, 'pos', -1),
            '9': (2, 'kp',  1),  '0': (2, 'kp',  -1),
            'u': (2, 'kd',  1),  'j': (2, 'kd',  -1),
        }

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        return [
            ParamSchema("num_axes", int, 2, "控制軸數", "選擇 1, 2 或 3 軸模式", min_value=1, max_value=3, step=1),
            ParamSchema("step_pos", float, 0.02, "位置單步跨度 (rad)", "位置調參步進幅度", min_value=0.005, max_value=0.2, step=0.005),
            ParamSchema("step_kp", float, 1.0, "Kp 剛性跨度", "Kp 調參步進幅度", min_value=0.1, max_value=10.0, step=0.5),
            ParamSchema("step_kd", float, 0.1, "Kd 阻尼跨度", "Kd 調參步進幅度", min_value=0.01, max_value=2.0, step=0.05),
            ParamSchema("m1_kp", float, 42.0, "Motor1 Kp", "預設 M1 Kp", min_value=0.0, max_value=200.0, step=1.0),
            ParamSchema("m1_kd", float, 2.2, "Motor1 Kd", "預設 M1 Kd", min_value=0.0, max_value=20.0, step=0.1),
            ParamSchema("m2_kp", float, 40.0, "Motor2 Kp", "預設 M2 Kp", min_value=0.0, max_value=200.0, step=1.0),
            ParamSchema("m2_kd", float, 2.6, "Motor2 Kd", "預設 M2 Kd", min_value=0.0, max_value=20.0, step=0.1),
            ParamSchema("m3_kp", float, 40.0, "Motor3 Kp", "預設 M3 Kp", min_value=0.0, max_value=200.0, step=1.0),
            ParamSchema("m3_kd", float, 2.6, "Motor3 Kd", "預設 M3 Kd", min_value=0.0, max_value=20.0, step=0.1),
        ]

    def initialize(self, init_pos: Union[float, List[float], Tuple[float, ...], Dict[str, float]]) -> None:
        """同步馬達啟動時的實體動態多軸初始位置，防止開機起跳。"""
        if isinstance(init_pos, (tuple, list)):
            for idx, pos in enumerate(init_pos[:self.num_axes]):
                self.axes[idx].pos = float(pos)
        elif isinstance(init_pos, dict):
            for idx, ax in enumerate(self.axes):
                key_m = f"m{idx + 1}"
                if key_m in init_pos:
                    ax.pos = float(init_pos[key_m])
                elif ax.name in init_pos:
                    ax.pos = float(init_pos[ax.name])
        elif isinstance(init_pos, (int, float)):
            if self.num_axes > 0:
                self.axes[0].pos = float(init_pos)

    def handle_input(self, action: str, **kwargs: Any) -> None:
        """處理標準化互動事件 (支援 key/e_stop/toggle_gravity)。"""
        if action == "key":
            key = str(kwargs.get("key", "")).lower()
            self._process_key(key)
        elif action == "e_stop":
            self.e_stop_triggered = True
        elif action == "toggle_gravity":
            self.gravity_ff_enabled = not self.gravity_ff_enabled

    def _process_key(self, key: str) -> None:
        """依據按鍵即時微調啟用軸的 Pos 與 Kp/Kd"""
        if key in ['q', ' ']:
            self.e_stop_triggered = True
            return
        elif key == 'g':
            self.gravity_ff_enabled = not self.gravity_ff_enabled
            return

        if key in self.key_map:
            axis_idx, attr, direction = self.key_map[key]
            if axis_idx < self.num_axes:
                axis = self.axes[axis_idx]
                if attr == 'pos':
                    axis.pos += direction * self.step_pos
                elif attr == 'kp':
                    axis.kp = max(0.0, axis.kp + direction * self.step_kp)
                elif attr == 'kd':
                    axis.kd = max(0.0, axis.kd + direction * self.step_kd)

    def get_target(self, elapsed_time: float = 0.0, axis: Union[str, int] = "m1") -> TrajectoryPoint:
        """取得指定軸的 TrajectoryPoint (支援舊版字串與索引查詢)。"""
        idx = 0
        if isinstance(axis, int):
            idx = min(max(0, axis), self.num_axes - 1)
        elif isinstance(axis, str):
            axis_lower = axis.lower()
            if axis_lower in ["m2", "fr_knee", "knee", "1"]:
                idx = 1 if self.num_axes >= 2 else 0
            elif axis_lower in ["m3", "fr_hipx", "hipx", "2"]:
                idx = 2 if self.num_axes >= 3 else (self.num_axes - 1)
            else:
                idx = 0

        ax = self.axes[idx]
        return TrajectoryPoint(position=ax.pos, velocity=0.0, kp=ax.kp, kd=ax.kd)

    def get_targets(self, elapsed_time: float = 0.0) -> Dict[str, TrajectoryPoint]:
        """【動態多軸】取得所有已啟用軸當前的 TrajectoryPoint 字典。"""
        return {
            f"m{i + 1}": TrajectoryPoint(position=ax.pos, velocity=0.0, kp=ax.kp, kd=ax.kd)
            for i, ax in enumerate(self.axes)
        }