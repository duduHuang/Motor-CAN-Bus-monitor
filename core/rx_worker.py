#core/rx_worker.py
"""
MotorRxWorker Module
背景 CAN 接收與實時狀態維護 Worker (Thread-Safe)
"""
import threading
from typing import Callable, Dict, Optional
from collections import deque
from datetime import datetime
from .motor_controller import MotorController, ParsedCANMessage

class MotorRxWorker(threading.Thread):
    """背景 CAN 接收與實時狀態維護 Worker (Thread-Safe)"""

    def __init__(
        self,
        controller: MotorController,
        on_message_received: Optional[Callable[[ParsedCANMessage], None]] = None,
        timeout: float = 0.1
    ):
        """
        :param controller: MotorController 實例
        :param on_message_received: 可選的 Callback 函式，收到封包時即時觸發 (如列印 Log 或 UI 繪圖)
        :param timeout: bus.recv 超時秒數，確保 stop() 能迅速響應
        """
        super().__init__(daemon = True)  # daemon 執行緒隨主程式結束而終止
        self.controller = controller
        self.on_message_received = on_message_received
        self.timeout = timeout
        
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        
        # 雙層字典: dict[motor_id, dict[telemetry_type_name, ParsedCANMessage]]
        self._motor_db: Dict[int, Dict[str, ParsedCANMessage]] = {}
        self.can_logger: deque = deque(maxlen=100) # RX 獨立或共享 Logger

    def run(self):
        print("[RxWorker] 背景監聽執行緒已啟動...")

        while not self._stop_event.is_set():
            try:
                msg = self.controller.bus.recv(timeout = self.timeout)
                if msg is None:
                    continue

                # 若 msg 非馬達 RX 封包，process_rx_message 會直接回傳 None
                parsed_msg = self.controller.process_rx_message(msg)
                if parsed_msg is None:
                    continue

                # === 攔截 RX 封包記錄時間戳記 ===
                timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                hex_payload = ' '.join(f"{b:02X}" for b in msg.data)
                log_entry = f"[{timestamp_str}] RX ID: 0x{msg.arbitration_id:03X} DATA: {hex_payload}"
                
                # 寫入 Controller 統一的 Logger 緩衝區
                self.controller.can_logger.append(log_entry)

                # 更新記憶體資料庫
                with self._lock:
                    if parsed_msg.motor_id not in self._motor_db:
                        self._motor_db[parsed_msg.motor_id] = {}
                        
                    # 根據 telemetry 的型別來分類儲存
                    telemetry_type = type(parsed_msg.telemetry).__name__
                    self._motor_db[parsed_msg.motor_id][telemetry_type] = parsed_msg

                # 觸發外部 Callback
                if self.on_message_received:
                    self.on_message_received(parsed_msg)

            except Exception as e:
                print(f"[RxWorker] 解析例外: {e}")

    # 新增一個查詢方法，讓 ViewModel 可以指定要拿哪一種資料：
    def get_specific_telemetry(self, motor_id: int, telemetry_type_name: str) -> Optional[ParsedCANMessage]:
        with self._lock:
            motor_data = self._motor_db.get(motor_id)
            if motor_data:
                return motor_data.get(telemetry_type_name)
        return None

    def stop(self):
        """安全停止背景執行緒"""
        self._stop_event.set()
        self.join(timeout = 1.0)
        print("[RxWorker] 背景監聽執行緒已停止。")

    def get_motor_telemetry(self, motor_id: int) -> Optional[ParsedCANMessage]:
        """執行緒安全地獲取指定馬達最新的狀態資料"""
        with self._lock:
            motor_data = self._motor_db.get(motor_id)
            if motor_data:
                return motor_data.get("MITTelemetry")
        return None

    def get_all_motors_telemetry(self) -> Dict[int, ParsedCANMessage]:
        """獲取所有已連線馬達的最新狀態快照"""
        with self._lock:
            return self._motor_db.copy()