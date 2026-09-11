from dataclasses import dataclass
import time
from typing import Any, Optional
from collections import deque
from datetime import datetime
import can
from protocol.decoders import BaseDecoder
from protocol.mit_command import MITTelemetryDecoder, MITConfig

@dataclass
class ParsedCANMessage:
    """接收封包解析結果數據結構"""
    motor_id: int
    msg_type: str       # "SINGLE_RX" 或 "MOTION_RX"
    raw_id: int         # 原始 CAN Arbitration ID (Hex)
    telemetry: Any      # 由 BaseDecoder 自動解析的 @dataclass 物件
    timestamp: float = 0.0  # 紀錄封包接收的準確時間戳

class MotorController:
    """業務邏輯與匯流排傳輸管理層"""
    SINGLE_MOTOR_BASE_TX = 0x140  # 單機控制 TX (0x140 + Motor_ID)
    SINGLE_MOTOR_BASE_RX = 0x240  # 單機控制 RX (0x240 + Motor_ID)
    MULTI_MOTOR_BASE_TX  = 0x280  # 多機廣播 TX (固定 0x280)
    MOTION_MODE_BASE_TX  = 0x400  # 運動控制 TX (0x400 + Motor_ID)
    MOTION_MODE_BASE_RX  = 0x500  # 運動控制 RX (0x500 + Motor_ID)

    MAX_MOTOR_ID = 63

    def __init__(self, bus: can.BusABC, auto_setup_filters: bool = True):
        self.bus = bus
        self.can_logger: deque = deque(maxlen=100)
        if auto_setup_filters:
            self.setup_can_filters()

    def setup_can_filters(self):
        """設定 Kernel/硬體濾波器：只接收 0x241~0x27F 及 0x501~0x53F 的封包"""
        try:
            filters = [
                # 單機回傳濾波 (0x240 ~ 0x27F)
                {"can_id": self.SINGLE_MOTOR_BASE_RX, "can_mask": 0x7C0, "extended": False},
                # 運動模式回傳濾波 (0x500 ~ 0x53F)
                {"can_id": self.MOTION_MODE_BASE_RX, "can_mask": 0x7C0, "extended": False},
            ]
            self.bus.set_filters(filters)
        except Exception as e:
            print(f"[MotorController] 警告: 無法設定 CAN Filter ({e})")

    def _send_frame(self, can_id: int, payload: bytes):
        """內部統一傳輸函式"""
        if len(payload) != 8:
            raise ValueError(f"CAN Payload 長度必須為 8 Bytes, 收到 {len(payload)} Bytes")
        
        msg = can.Message(
            arbitration_id = can_id,
            data = payload,
            is_extended_id = False
        )
        self.bus.send(msg)

        # === 攔截 TX 封包記錄時間戳記 ===
        timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_payload = ' '.join(f"{b:02X}" for b in payload)
        log_entry = f"[{timestamp_str}] TX ID: 0x{can_id:03X} DATA: {hex_payload}"
        self.can_logger.append(log_entry)

    # --- TX 發送路由控制 ---

    def send_single_command(self, motor_id: int, payload: bytes):
        """下發單機控制指令 (0x140 + Motor_ID)"""
        if not (1 <= motor_id <= self.MAX_MOTOR_ID):
            raise ValueError(f"無效的 Motor ID: {motor_id}")
        self._send_frame(self.SINGLE_MOTOR_BASE_TX + motor_id, payload)

    def send_multi_command(self, payload: bytes):
        """下發多機廣播控制指令 (固定 0x280)"""
        self._send_frame(self.MULTI_MOTOR_BASE_TX, payload)

    def send_motion_command(self, motor_id: int, payload: bytes):
        """下發運動控制指令 (0x400 + Motor_ID)"""
        if not (1 <= motor_id <= self.MAX_MOTOR_ID):
            raise ValueError(f"無效的 Motor ID: {motor_id}")
        self._send_frame(self.MOTION_MODE_BASE_TX + motor_id, payload)

    # --- RX 接收與動態解碼 ---

    def process_rx_message(
        self, 
        msg: can.Message, 
        mit_cfg: MITConfig = MITConfig()
    ) -> Optional[ParsedCANMessage]:
        """
        解析 CAN 封包：
        - 若非馬達 RX ID, 靜默回傳 None (不拋例外)
        - 若解析成功，回傳 ParsedCANMessage
        """
        # 1. 過濾本機 TX 環回 (Echo) 封包
        if getattr(msg, 'is_rx', True) is False:
            return None

        can_id = msg.arbitration_id
        msg_timestamp = getattr(msg, 'timestamp', time.monotonic())

        # 2. 單機模式回傳區段 (0x241 ~ 0x27F) -> 由 Byte 0 cmd_echo 解碼
        if self.SINGLE_MOTOR_BASE_RX < can_id <= self.SINGLE_MOTOR_BASE_RX + self.MAX_MOTOR_ID:
            motor_id = can_id - self.SINGLE_MOTOR_BASE_RX
            telemetry = BaseDecoder.decode_any(msg.data)
            return ParsedCANMessage(
                motor_id = motor_id,
                msg_type = "SINGLE_RX",
                raw_id = can_id,
                telemetry = telemetry,
                timestamp = msg_timestamp
            )

        # 3. 運動模式回傳區段 (0x501 ~ 0x53F) -> 由 MIT 專用解碼器解析
        elif self.MOTION_MODE_BASE_RX < can_id <= self.MOTION_MODE_BASE_RX + self.MAX_MOTOR_ID:
            motor_id = can_id - self.MOTION_MODE_BASE_RX
            telemetry = MITTelemetryDecoder.decode(msg.data, cfg = mit_cfg)
            return ParsedCANMessage(
                motor_id = motor_id,
                msg_type = "MOTION_RX",
                raw_id = can_id,
                telemetry = telemetry,
                timestamp = msg_timestamp
            )

        return None