import os
from pathlib import Path
from typing import Any, List
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint

SINGLE_JOINT_MSG_DEF = """
string name
float32 position
float32 velocity
float32 effort
float32 torque_ff
int32 status
"""

ROBOT_STATE_MSG_DEF = """
std_msgs/Header header
string map_path
float64[] pose_data
string wifi_info
int32 status_code
float32 battery
int32 mode
int32 sub_mode
int64 timestamp
syncai_common/SingleJointState[] joints
"""


class Db3TrajectoryProvider(BaseTrajectoryProvider):
    """ROS 2 Rosbag2 (.db3) 關節軌跡提供者。"""

    def __init__(
        self,
        bag_path: str = "",
        joint_idx: int = 0,
        topic_name: str = "/robot01/robot_state",
        kp: float = 20.0,
        kd: float = 1.0,
    ):
        super().__init__()
        self.bag_path = bag_path
        self.joint_idx = joint_idx
        self.topic_name = topic_name
        self.kp = kp
        self.kd = kd

        self.time_samples = np.array([0.0, 10.0])
        self.pos_samples = np.array([0.0, 0.0])
        self.vel_samples = np.array([0.0, 0.0])
        self.max_duration = 10.0
        self._start_offset = 0.0

        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        self._register_custom_types()

        if bag_path and os.path.exists(bag_path):
            self._load_rosbag2()

    @property
    def duration(self) -> float:
        """回傳 Rosbag 總時長 (秒)。"""
        return float(self.max_duration)

    def _register_custom_types(self):
        try:
            custom_types = {}
            custom_types.update(
                get_types_from_msg(
                    SINGLE_JOINT_MSG_DEF, "syncai_common/msg/SingleJointState"
                )
            )
            custom_types.update(
                get_types_from_msg(
                    ROBOT_STATE_MSG_DEF, "syncai_common/msg/RobotState"
                )
            )
            self.typestore.register(custom_types)
        except Exception as e:
            print(f"[WARN] 註冊型別失敗或已存在: {e}")

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        return [
            ParamSchema(
                name="bag_path",
                param_type=str,
                default_value="",
                display_name="Bag File Path",
                description="ROS 2 Rosbag2 (.db3) file or directory path",
                is_file_path=True,
                file_filter="*.db3",
            ),
            ParamSchema(
                name="joint_idx",
                param_type=int,
                default_value=0,
                display_name="Joint Index",
                description="Target motor index in RobotState (0, 1, 2...)",
                min_value=0,
            ),
            ParamSchema(
                name="topic_name",
                param_type=str,
                default_value="/robot01/robot_state",
                display_name="Topic Name",
                description="Target topic inside Rosbag",
            ),
            ParamSchema(
                name="kp",
                param_type=float,
                default_value=20.0,
                display_name="Stiffness (Kp)",
                description="Stiffness gain",
                min_value=0.0,
                max_value=500.0,
            ),
            ParamSchema(
                name="kd",
                param_type=float,
                default_value=1.0,
                display_name="Damping (Kd)",
                description="Damping gain",
                min_value=0.0,
                max_value=50.0,
            ),
        ]

    def handle_input(self, action: str, **kwargs: Any) -> None:
        if action == "update_targets":
            if "bag_path" in kwargs:
                new_path = kwargs["bag_path"]
                if new_path != self.bag_path:
                    self.bag_path = new_path
                    if os.path.exists(self.bag_path):
                        self._load_rosbag2()
            for k, v in kwargs.items():
                if hasattr(self, k):
                    setattr(self, k, v)

    def initialize(self, init_pos: float) -> None:
        if len(self.pos_samples) > 0:
            self._start_offset = init_pos - float(self.pos_samples[0])

    def _load_rosbag2(self):
        times, positions, velocities = [], [], []
        path = Path(self.bag_path)
        bag_dir = path if path.is_dir() else path.parent

        try:
            with Reader(bag_dir) as reader:
                connections = [
                    x for x in reader.connections if x.topic == self.topic_name
                ]
                if not connections:
                    print(f"[WARN] 找不到 Topic: {self.topic_name}")
                    return

                for connection, timestamp_ns, rawdata in reader.messages(
                    connections=connections
                ):
                    msg = self.typestore.deserialize_cdr(
                        rawdata, connection.msgtype
                    )
                    
                    if hasattr(msg, "joints") and len(msg.joints) > self.joint_idx:
                        joint_item = msg.joints[self.joint_idx]
                        times.append(timestamp_ns / 1e9)
                        positions.append(float(joint_item.position))
                        velocities.append(float(joint_item.velocity) if hasattr(joint_item, "velocity") else 0.0)

            if times:
                start_time = times[0]
                self.time_samples = np.array(times) - start_time
                self.pos_samples = np.array(positions)
                self.vel_samples = np.array(velocities)
                self.max_duration = float(self.time_samples[-1])
        except Exception as e:
            print(f"[ERROR] 載入 Db3 檔案失敗: {e}")

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        current_t = min(elapsed_time, self.max_duration)
        p_des = float(
            np.interp(current_t, self.time_samples, self.pos_samples)
        ) + self._start_offset
        v_des = float(
            np.interp(current_t, self.time_samples, self.vel_samples)
        )

        return TrajectoryPoint(
            position=p_des,
            velocity=v_des,
            kp=self.kp,
            kd=self.kd,
            torque_ff=0.0,
        )