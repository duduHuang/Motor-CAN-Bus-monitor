import csv
import os
import threading
import numpy as np
from typing import Any, List
from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint


class CsvTrajectoryProvider(BaseTrajectoryProvider):
    """CSV 檔案軌跡播放器 (安全啟動平移與插值效能優化)。"""

    def __init__(
        self,
        csv_filepath: str = "",
        kp: float = 25.0,
        kd: float = 1.8,
        enable_ff: bool = True,
        enable_inertia_ff: bool = True,
        m_leg: float = 1.8,
        r_com: float = 0.20,
        j_eq: float = 0.015,
        stand_offset: float = -0.65,
        axis_sign: float = 1.0,
        max_gravity_ff: float = 1.2
    ) -> None:
        super().__init__()
        self.csv_filepath = csv_filepath
        self.kp = kp
        self.kd = kd
        self.enable_ff = enable_ff
        self.enable_inertia_ff = enable_inertia_ff
        
        self.m_leg = m_leg
        self.g = 9.81
        self.r_com = r_com
        self.j_eq = j_eq
        self.stand_offset = stand_offset
        self.axis_sign = axis_sign
        self.max_gravity_ff = max_gravity_ff

        self._lock = threading.Lock()
        self._times = np.array([0.0])
        self._positions = np.array([0.0])
        self._velocities = np.array([0.0])
        self._torques = np.array([0.0])
        self._accelerations = np.array([0.0])
        
        self._start_offset = 0.0
        if csv_filepath and os.path.exists(csv_filepath):
            self._load_csv(csv_filepath)

    @property
    def duration(self) -> float:
        """回傳 CSV 總時長 (秒)，供 ViewModel 動態同步控制時間範圍。"""
        with self._lock:
            if len(self._times) > 1:
                return float(self._times[-1] - self._times[0])
            return 0.0

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        return [
            ParamSchema(
                "csv_filepath", str, "", "CSV File Path", 
                "CSV file containing time, pos, vel columns",
                is_file_path=True, file_filter="*.csv"
            ),
            ParamSchema("kp", float, 20.0, "Stiffness (Kp)", "MIT position gain", min_value=0.0, max_value=500.0, step=1.0),
            ParamSchema("kd", float, 1.8, "Damping (Kd)", "MIT velocity gain", min_value=0.0, max_value=50.0, step=0.1),
            ParamSchema("enable_ff", bool, True, "Enable Feedforward", "Enable lightweight gravity feedforward compensation"),
            ParamSchema("max_gravity_ff", float, 1.2, "Max Gravity FF Limit", "Gravity feedforward limit range (Nm)", min_value=0.0, max_value=5.0, step=0.1),
        ]

    def handle_input(self, action: str, **kwargs: Any) -> None:
        if action == "update_targets":
            if "csv_filepath" in kwargs:
                new_path = kwargs["csv_filepath"]
                if new_path != self.csv_filepath:
                    self.csv_filepath = new_path
                    if os.path.exists(self.csv_filepath):
                        self._load_csv(self.csv_filepath)
            for k, v in kwargs.items():
                if hasattr(self, k):
                    setattr(self, k, v)

    def _load_csv(self, filepath: str) -> None:
        t_list, p_list, v_list, tau_list = [], [], [], []
        try:
            with open(filepath, mode="r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # 支援標準名稱與 GUI Log 匯出名稱
                    t_val = row.get("time") or row.get("time_s") or row.get("t") or "0.0"
                    p_val = row.get("pos") or row.get("p_des") or row.get("p_act") or row.get("position") or "0.0"
                    v_val = row.get("vel") or row.get("v_des") or row.get("v_act") or row.get("velocity") or "0.0"
                    tau_val = row.get("torque") or row.get("torque_act") or row.get("t_ff") or "0.0"

                    t_list.append(float(t_val))
                    p_list.append(float(p_val))
                    v_list.append(float(v_val))
                    tau_list.append(float(tau_val))
                    
            if t_list:
                times_arr = np.array(t_list)
                pos_arr = np.array(p_list)
                vel_arr = np.array(v_list)
                trq_arr = np.array(tau_list)
                acc_arr = np.gradient(vel_arr, times_arr) if len(times_arr) > 1 else np.zeros_like(vel_arr)

                with self._lock:
                    self._times = times_arr
                    self._positions = pos_arr
                    self._velocities = vel_arr
                    self._torques = trq_arr
                    self._accelerations = acc_arr
        except Exception as e:
            print(f"[ERROR] CSV 載入失敗: {e}")

    def initialize(self, init_pos: float) -> None:
        with self._lock:
            if len(self._positions) > 0:
                self._start_offset = init_pos - self._positions[0]

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        with self._lock:
            if len(self._times) <= 1:
                return TrajectoryPoint(position=0.0, velocity=0.0, kp=self.kp, kd=self.kd, torque_ff=0.0)

            p_des = float(np.interp(elapsed_time, self._times, self._positions)) + self._start_offset
            v_des = float(np.interp(elapsed_time, self._times, self._velocities))
            a_des = float(np.interp(elapsed_time, self._times, self._accelerations))

        t_ff = 0.0
        if self.enable_ff:
            theta_real = p_des - self.stand_offset
            tau_gravity = self.axis_sign * (self.m_leg * self.g * self.r_com * np.cos(theta_real))
            tau_gravity = np.clip(tau_gravity, -self.max_gravity_ff, self.max_gravity_ff)

            tau_inertia = 0.0
            if self.enable_inertia_ff:
                tau_inertia = self.axis_sign * (self.j_eq * a_des)
                tau_inertia = np.clip(tau_inertia, -1.5, 1.5)

            t_ff = float(tau_gravity + tau_inertia)

        return TrajectoryPoint(
            position=p_des,
            velocity=v_des,
            kp=self.kp,
            kd=self.kd,
            torque_ff=t_ff
        )