# core/rx_worker.py
"""
MotorRxWorker Module
背景 CAN 接收與實時狀態維護 Worker (Thread-Safe)
優化點：移除高頻 RX 字串格式化，改寫 Raw 元組至 can_logger。
"""
import threading
import time
from typing import Callable, Dict, Optional
from collections import deque
from .motor_controller import MotorController, ParsedCANMessage

class MotorRxWorker(threading.Thread):
    """背景 CAN 接收與實時狀態維護 Worker (Thread-Safe)"""

    def __init__(
        self,
        controller: MotorController,
        on_message_received: Optional[Callable[[ParsedCANMessage], None]] = None,
        timeout: float = 0.1
    ):
        super().__init__(daemon = True)
        self.controller = controller
        self.on_message_received = on_message_received
        self.timeout = timeout
        
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        
        self._motor_db: Dict[int, Dict[str, ParsedCANMessage]] = {}

    def run(self):
        print("[RxWorker] 背景監聽執行緒已啟動...")

        while not self._stop_event.is_set():
            try:
                msg = self.controller.bus.recv(timeout = self.timeout)
                if msg is None:
                    continue

                parsed_msg = self.controller.process_rx_message(msg)
                if parsed_msg is None:
                    continue

                # === 寫入 Controller Raw 日誌，移除字串格式化耗時 ===
                self.controller.can_logger.append((time.monotonic(), "RX", msg.arbitration_id, msg.data))

                with self._lock:
                    if parsed_msg.motor_id not in self._motor_db:
                        self._motor_db[parsed_msg.motor_id] = {}
                        
                    telemetry_type = type(parsed_msg.telemetry).__name__
                    self._motor_db[parsed_msg.motor_id][telemetry_type] = parsed_msg

                if self.on_message_received:
                    self.on_message_received(parsed_msg)

            except Exception as e:
                print(f"[RxWorker] 解析例外: {e}")

    def get_specific_telemetry(self, motor_id: int, telemetry_type_name: str) -> Optional[ParsedCANMessage]:
        with self._lock:
            motor_data = self._motor_db.get(motor_id)
            if motor_data:
                return motor_data.get(telemetry_type_name)
        return None

    def stop(self):
        self._stop_event.set()
        self.join(timeout = 1.0)
        print("[RxWorker] 背景監聽執行緒已停止。")

    def get_motor_telemetry(self, motor_id: int) -> Optional[ParsedCANMessage]:
        with self._lock:
            motor_data = self._motor_db.get(motor_id)
            if motor_data:
                return motor_data.get("MITTelemetry")
        return None

    def get_all_motors_telemetry(self) -> Dict[int, ParsedCANMessage]:
        with self._lock:
            return self._motor_db.copy()