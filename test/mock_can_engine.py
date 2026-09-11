import threading
import time
import math
import datetime
from typing import Optional
from collections import deque

from core import ParsedCANMessage
from protocol.mit_command import MITCommandEncoder, MITConfig, MITTelemetry
from protocol.decoders.sensor_decoder import SensorStatus1Telemetry, ErrorStatusFlags
from protocol.decoders.realtime_decoder import StandardMotionTelemetry

class MockMotorCANEngine:
    """Mock 馬達物理引擎 (模擬馬達運動與感測器動態狀態)"""
    def __init__(self, node_id: int = 1) -> None:
        self.node_id = node_id
        self._lock = threading.Lock()
        
        self.position: float = 0.0
        self.velocity: float = 0.0
        self.torque: float = 0.0
        self.start_time: float = time.monotonic()
        self.last_update_time: float = time.monotonic()

        # 異常注入 Flags
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

    def get_temperature_and_voltage(self) -> tuple[int, float]:
        """動態模擬馬達負載升溫與電壓小幅波動 (讓頂部看板數據可動態觀察)"""
        with self._lock:
            elapsed = time.monotonic() - self.start_time
            # 根據負載與運作時間模擬升溫 (30°C ~ 55°C)
            temp = int(30 + min(25.0, elapsed * 0.05 + abs(self.torque) * 1.5))
            # 模擬 24V 供電的微幅波動
            volt = round(24.1 + 0.15 * math.sin(elapsed * 2.0), 1)
            return temp, volt


class MockMotorController:
    """Mock 控制器 (記錄 TX 指令)"""
    def __init__(self, engine: MockMotorCANEngine, cfg: MITConfig = MITConfig()) -> None:
        self.engine = engine
        self.cfg = cfg
        self.can_logger: deque = deque(maxlen=100)

    def _log_tx_frame(self, can_id: int, payload_bytes: bytes):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_payload = ' '.join(f"{b:02X}" for b in payload_bytes)
        log_entry = f"[{ts}] TX ID: 0x{can_id:03X} DATA: {hex_payload}"
        self.can_logger.append(log_entry)

    def send_motion_command(self, motor_id: int, payload: bytes) -> None:
        self._log_tx_frame(0x400 + motor_id, payload)
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
        """紀錄單次設定指令 (如 0xB6 主動上報、0x80 急停等)"""
        self._log_tx_frame(0x140 + motor_id, payload)


class MockMotorRxWorker:
    """Mock 背景接收 Worker (同步將接收到的 Mock 封包寫入 CAN Console Logger)"""
    def __init__(self, engine: MockMotorCANEngine, controller: Optional[MockMotorController] = None) -> None:
        self.engine = engine
        self.controller = controller

    def _log_rx_frame(self, can_id: int, payload_bytes: bytes):
        if self.controller:
            ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            hex_payload = ' '.join(f"{b:02X}" for b in payload_bytes)
            log_entry = f"[{ts}] RX ID: 0x{can_id:03X} DATA: {hex_payload}"
            self.controller.can_logger.append(log_entry)

    def get_motor_telemetry(self, motor_id: int) -> Optional[ParsedCANMessage]:
        telemetry = self.engine.get_telemetry()
        if telemetry is None:
            return None

        raw_id = 0x500 + motor_id
        # 模擬 8-byte MIT RX 封包寫入日誌
        mock_bytes = bytes([motor_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._log_rx_frame(raw_id, mock_bytes)

        return ParsedCANMessage(
            motor_id = motor_id,
            msg_type = "MOTION_RX",
            raw_id = raw_id,
            telemetry = telemetry,
            timestamp = time.monotonic()
        )

    def get_specific_telemetry(self, motor_id: int, telemetry_type_name: str) -> Optional[ParsedCANMessage]:
        if self.engine.simulate_comm_loss:
            return None

        now = time.monotonic()
        if telemetry_type_name == "MITTelemetry":
            return self.get_motor_telemetry(motor_id)

        elif telemetry_type_name == "SensorStatus1Telemetry":
            temp, volt = self.engine.get_temperature_and_voltage()
            fake_telemetry = SensorStatus1Telemetry(
                cmd_echo=0x9A,
                temperature_c=temp,
                mos_temperature_c=temp + 3,
                brake_released=True,
                voltage_v=volt,
                error_flags=ErrorStatusFlags.from_uint16(0)
            )
            raw_id = 0x240 + motor_id
            # 模擬 0x9A RX 封包寫入日誌
            mock_bytes = bytes([0x9A, temp & 0xFF, (temp+3) & 0xFF, 0x01, int(volt * 10) & 0xFF, 0x00, 0x00, 0x00])
            self._log_rx_frame(raw_id, mock_bytes)

            return ParsedCANMessage(motor_id=motor_id, msg_type="SINGLE_RX", raw_id=raw_id, telemetry=fake_telemetry, timestamp=now)

        elif telemetry_type_name == "StandardMotionTelemetry":
            fake_current = self.engine.torque / 0.1
            spd_dps = round(math.degrees(self.engine.velocity), 1)
            ang_deg = int(math.degrees(self.engine.position))
            fake_telemetry = StandardMotionTelemetry(
                cmd_echo=0x9C,
                temperature_c=int(30 + abs(self.engine.torque)),
                iq_current_amp=round(fake_current, 2),
                speed_dps=spd_dps,
                angle_deg=ang_deg
            )
            raw_id = 0x240 + motor_id
            # 模擬 0x9C RX 封包寫入日誌
            mock_bytes = bytes([0x9C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            self._log_rx_frame(raw_id, mock_bytes)

            return ParsedCANMessage(motor_id=motor_id, msg_type="SINGLE_RX", raw_id=raw_id, telemetry=fake_telemetry, timestamp=now)

        return None