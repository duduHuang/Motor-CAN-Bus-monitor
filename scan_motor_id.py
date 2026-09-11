"""
CAN Bus 馬達 ID 自動掃描工具
輪詢 ID 1 ~ 63 下發狀態查詢指令，確認匯流排上在線的馬達 ID 清單。
"""

import time
import can
from core import MotorController
from protocol import MotorProtocol

def scan_online_motors(max_id: int = 63, timeout_per_id: float = 0.03) -> list[int]:
    """掃描 CAN Bus 並傳回在線馬達 ID 清單"""
    found_ids = []

    try:
        bus = can.ThreadSafeBus(channel="canfd0", interface="socketcan")
        controller = MotorController(bus, auto_setup_filters=True)
    except Exception as e:
        print(f"[ERROR] 無法開啟 CAN 介面: {e}")
        return []

    query_cmd = MotorProtocol.read_status_1()  # 0x9A 讀取狀態指令 (無害查詢)

    print(f"[ID Scanner] 開始掃描 CAN Bus 上的馬達 (範圍: ID 1 ~ {max_id})...\n")
    start_scan_time = time.monotonic()

    for motor_id in range(1, max_id + 1):
        # 1. 發送查詢指令
        try:
            controller.send_single_command(motor_id, query_cmd)
        except can.CanError:
            continue

        # 2. 監聽該 ID 是否有回應
        listen_start = time.monotonic()
        while time.monotonic() - listen_start < timeout_per_id:
            msg = bus.recv(timeout=0.005)
            if msg is None:
                continue

            parsed = controller.process_rx_message(msg)
            if parsed and parsed.motor_id == motor_id:
                print(f"  [✓] 找到馬達！ ID: {motor_id} (Arbitration ID: {hex(parsed.raw_id)})")
                found_ids.append(motor_id)
                break

    bus.shutdown()
    elapsed = time.monotonic() - start_scan_time

    print(f"\n[Scanner 完成] 耗時 {elapsed:.2f} 秒")
    print(f"在線馬達 ID 清單: {found_ids if found_ids else '無回應 (請檢查電源與 CAN 線路)'}\n")

    return found_ids


if __name__ == "__main__":
    scan_online_motors()
