import threading
import time
import math # 新增 math 用於角度轉換
from typing import Optional
from collections import deque # 新增 deque 用於假 Logger

from core import ParsedCANMessage
from protocol.mit_command import MITCommandEncoder, MITConfig, MITTelemetry
# 新增匯入解碼器的資料結構，用於產出假資料
from protocol.decoders.sensor_decoder import SensorStatus1Telemetry, ErrorStatusFlags
from protocol.decoders.realtime_decoder import StandardMotionTelemetry

class MockMotorCANEngine:
    """模擬馬達節點：支援 MIT 控制動態與故障注入"""
    def __init__(self, node_id: int = 1) -> None:
        self.node_id = node_id
        self._lock = threading.Lock()

        # 狀態資料
        self.position: float = 0.0
        self.velocity: float = 0.0
        self.torque: float = 0.0
        self.last_update_time: float = time.monotonic()

        # 故障注入旗標 (Fault Injection Flags)
        self.fault_overspeed: bool = False
        self.fault_overtorque: bool = False
        self.fault_following_error: bool = False
        self.simulate_comm_loss: bool = False

    def receive_command(self, p_des: float, v_des: float, kp: float, kd: float, t_ff: float) -> None:
        if self.simulate_comm_loss:
            return

        with self._lock:
            now = time.monotonic()
            self.position = p_des + (10.0 if self.fault_following_error else 0.0)
            self.velocity = 999.0 if self.fault_overspeed else v_des
            self.torque = 99.0 if self.fault_overtorque else (t_ff + kp * (p_des - self.position))
            self.last_update_time = now

    def get_telemetry(self) -> Optional[MITTelemetry]:
        if self.simulate_comm_loss:
            return None

        with self._lock:
            return MITTelemetry(
                device_can_id = self.node_id,
                position_rad = round(self.position, 4),
                velocity_rads = round(self.velocity, 4),
                torque_nm = round(self.torque, 3),
            )

class MockMotorController:
    """模擬 MotorController 轉接層"""
    def __init__(self, engine: MockMotorCANEngine, cfg: MITConfig = MITConfig()) -> None:
        self.engine = engine
        self.cfg = cfg
        # [修改處 1] 新增假的 can_logger，防止 ViewModel 取 Log 時報錯
        self.can_logger = deque(maxlen=100)

    def send_motion_command(self, motor_id: int, payload: bytes) -> None:
        # [修改處 2] 模擬攔截 TX 寫入假 Logger
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.can_logger.append(f"[{ts}] TX ID: 0x{0x400+motor_id:03X} DATA: MOCK_MOTION_DATA")

        if len(payload) == 8:
            cmd = MITCommandEncoder.decode(payload, cfg = self.cfg)
            self.engine.receive_command(
                p_des = cmd.p_des,
                v_des = cmd.v_des,
                kp = cmd.kp,
                kd = cmd.kd,
                t_ff = cmd.t_ff
            )

    def send_single_command(self, motor_id: int, payload: bytes) -> None:
        # 可以加上簡單的攔截，例如啟用主動回報的指令記錄
        pass

class MockMotorRxWorker:
    """模擬 MotorRxWorker 轉接層"""
    def __init__(self, engine: MockMotorCANEngine) -> None:
        self.engine = engine

    def get_motor_telemetry(self, motor_id: int) -> Optional[ParsedCANMessage]:
        telemetry = self.engine.get_telemetry()
        if telemetry is None:
            return None

        return ParsedCANMessage(
            motor_id = motor_id,
            msg_type = "MOTION_RX",
            raw_id = 0x500 + motor_id,
            telemetry = telemetry,
            timestamp = time.monotonic()
        )

    # [修改處 3] 實作 get_specific_telemetry 以應付 ViewModel 的查詢
    def get_specific_telemetry(self, motor_id: int, telemetry_type_name: str) -> Optional[ParsedCANMessage]:
        """傳回假的特定型別遙測，讓 UI 測試時有豐富資料呈現"""
        if self.engine.simulate_comm_loss:
            return None
            
        now = time.monotonic()

        # 如果 ViewModel 要抓 MIT 數據 (原本的流程)
        if telemetry_type_name == "MITTelemetry":
            return self.get_motor_telemetry(motor_id)

        # 模擬 0x9A (電壓、溫度) 回傳
        elif telemetry_type_name == "SensorStatus1Telemetry":
            fake_telemetry = SensorStatus1Telemetry(
                cmd_echo=0x9A,
                temperature_c=32,            # 假溫度：32°C
                mos_temperature_c=35,
                brake_released=True,
                voltage_v=24.1,              # 假電壓：24.1V
                error_flags=ErrorStatusFlags.from_uint16(0)
            )
            return ParsedCANMessage(motor_id=motor_id, msg_type="SINGLE_RX", raw_id=0x240+motor_id, telemetry=fake_telemetry, timestamp=now)

        # 模擬 0x9C (電流、轉速) 回傳
        elif telemetry_type_name == "StandardMotionTelemetry":
            # 假裝 Kt (扭矩常數) = 0.1，利用 Torque 反推電流，讓 UI 圖表會隨出力跳動！
            fake_current = self.engine.torque / 0.1
            fake_telemetry = StandardMotionTelemetry(
                cmd_echo=0x9C,
                temperature_c=32,
                iq_current_amp=round(fake_current, 2),
                speed_dps=round(math.degrees(self.engine.velocity), 1),
                angle_deg=int(math.degrees(self.engine.position))
            )
            return ParsedCANMessage(motor_id=motor_id, msg_type="SINGLE_RX", raw_id=0x240+motor_id, telemetry=fake_telemetry, timestamp=now)

        return None