# main_gui.py
"""
Main GUI for Multi-Axis Motor Control
"""
import sys
import argparse
from typing import Dict, List, Tuple
import can

from core import MotorControlViewModel, MotorController, MotorRxWorker, MultiMotorViewModelManager
from test.mock_can_engine import MockMotorCANEngine, MockMotorController, MockMotorRxWorker
from gui import MotorControlView

def parse_axis_config(config_str: str) -> List[Tuple[str, int]]:
    """解析多軸命令行配置，例如 'canfd0:1,canfd0:2' -> [('canfd0', 1), ('canfd0', 2)]"""
    axes = []
    items = config_str.split(",")
    for item in items:
        if ":" in item:
            ch, m_id = item.split(":")
            axes.append((ch.strip(), int(m_id.strip())))
    return axes

def main():
    parser = argparse.ArgumentParser(description = "Multi-Axis Motor Control Validation GUI")
    parser.add_argument("--mock", action = "store_true", help = "啟用虛擬多軸 Mock 模式 (免硬體)")
    parser.add_argument("--config", type = str, default = "canfd0:1,canfd0:2,canfd1:1,canfd1:2", help = "多軸配置格式: 'can0:1,can0:2,can1:1'")
    parser.add_argument("--bitrate", type = int, default = 1000000, help = "CAN Baudrate")
    args = parser.parse_args()

    axis_list = parse_axis_config(args.config)
    if not axis_list:
        print("[ERROR] 無法解析多軸配置，請使用例如: '--config canfd0:1,canfd0:2,canfd1:1,canfd1:2'")
        sys.exit(1)

    manager = MultiMotorViewModelManager()
    buses: Dict[str, can.ThreadSafeBus] = {}
    workers: List[MotorRxWorker] = []  # 記錄背景 Worker 實例

    print(f"[INFO] 正在初始化 {len(axis_list)} 組馬達控制節點...")

    for ch, m_id in axis_list:
        vm_key = f"{ch}_ID{m_id}"

        if args.mock:
            print(f"  -> [MOCK Mode] 初始化虛擬馬達: {vm_key}")
            mock_engine = MockMotorCANEngine(node_id = m_id)
            controller = MockMotorController(engine = mock_engine)
            # 傳入 controller 讓 MockRxWorker 能將 RX 封包記錄至 can_logger
            rx_worker = MockMotorRxWorker(engine = mock_engine, controller = controller)

            vm = MotorControlViewModel(controller = controller, rx_worker = rx_worker)

        else:
            print(f"  -> [REAL Hardware] 初始化實體 CAN 網卡: {vm_key}")
            if ch not in buses:
                try:
                    buses[ch] = can.ThreadSafeBus(channel = ch, interface = 'socketcan', bitrate = args.bitrate)
                except Exception as e:
                    print(f"[ERROR] 開啟 CAN 網卡 '{ch}' 失敗: {e}")
                    sys.exit(1)

            bus = buses[ch]
            controller = MotorController(bus = bus)
            rx_worker = MotorRxWorker(controller = controller)
            rx_worker.start()
            workers.append(rx_worker)  # 保存實例 reference

            vm = MotorControlViewModel(controller = controller, rx_worker = rx_worker)

        manager.add_motor(key = vm_key, vm = vm, channel = ch, motor_id = m_id)

    # 啟動主 GUI
    view = MotorControlView(manager = manager)

    try:
        print("[INFO] 啟動 DearPyGui 應用程式...")
        view.render_loop()
    except KeyboardInterrupt:
        print("\n[INFO] 使用者強制中斷。")
    finally:
        manager.stop_all()
        if not args.mock:
            # 優先停止所有背景 RxWorker 執行緒
            for w in workers:
                try:
                    w.stop()
                except Exception as e:
                    print(f"[WARN] 停止 RxWorker 失敗: {e}")
            for b in buses.values():
                b.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    main()