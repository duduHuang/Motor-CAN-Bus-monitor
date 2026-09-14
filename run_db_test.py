#!/usr/bin/env python3
"""
Phase 3 DB3 Rosbag 雙馬達軌跡跟隨測試腳本 (run_db_test.py)
架構：MVVM + SocketCAN (can1) + Db3TrajectoryProvider
"""

import math
import time
import can

from core import MotorController, MotorControlViewModel, SystemStatus, MotorRxWorker


def create_configured_vm(
    motor_id: int,
    joint_idx: int,
    bag_path: str,
    controller: MotorController,
    rx_worker: MotorRxWorker,
    topic_name: str = "/robot01/robot_state",
    kp: float = 20.0,
    kd: float = 1.0,
) -> MotorControlViewModel:
    """建立對接 DB3 TrajectoryProvider 的 MotorControlViewModel 實例"""
    vm = MotorControlViewModel(controller=controller, rx_worker=rx_worker)

    # 1. 基礎控制與安全保護參數
    vm.motor_id = motor_id
    vm.control_freq_hz = 100.0       # 控制頻率 100 Hz (10ms)
    vm.max_speed_rads = 8.0          # 最大速度保護 (rad/s)
    vm.max_torque_nm = 5.0           # 最大扭矩保護 (Nm)
    vm.max_pos_error = 0.8           # 最大跟隨誤差保護 (rad)
    vm.telemetry_timeout_s = 0.3     # 通訊超時 (s)
    vm.control_start_delay = 0.2     # 啟動保護延遲 (s)

    # 2. 透過 ViewModel 的 select_provider 載入 DB3 軌跡
    vm.select_provider(
        "DB3",
        bag_path=bag_path,
        joint_idx=joint_idx,
        topic_name=topic_name,
        kp=kp,
        kd=kd,
    )

    # 3. 自動對齊 .db3 檔內的動作總時長
    if vm._current_provider and hasattr(vm._current_provider, "max_duration"):
        vm.target_duration = vm._current_provider.max_duration

    return vm


def main():
    print("==================================================")
    print("  Phase 3: DB3 Rosbag 雙馬達單腿動作跟隨測試")
    print("==================================================")

    # 指定 ROS 2 .db3 軌跡檔路徑
    db3_path = "../climb_20260828/climb_20260828_0.db3"
    target_topic = "/robot01/robot_state"

    # 1. 初始化 SocketCAN 介面 (EGUC-F2S3 頻道 can1)
    try:
        bus = can.ThreadSafeBus(channel="can0", interface="socketcan")
        controller = MotorController(bus)
        rx_worker = MotorRxWorker(controller)
        rx_worker.start()
        print("[CAN] SocketCAN (can0) 與背景 RxWorker 初始化成功。")
    except Exception as e:
        print(f"[ERROR] CAN Bus 初始化失敗，請檢查 start_socketcan.sh: {e}")
        return

    # 2. 為手臂 (arm, ID 1) 與前臂 (forearm, ID 2) 建立對應之 ViewModel
    # Joint 0 -> vm_arm (ID 1)
    vm_arm = create_configured_vm(
        motor_id=1,
        joint_idx=0,
        bag_path=db3_path,
        topic_name=target_topic,
        controller=controller,
        rx_worker=rx_worker,
        kp=20.0,
        kd=1.0,
    )

    # Joint 1 -> vm_forearm (ID 2)
    vm_forearm = create_configured_vm(
        motor_id=2,
        joint_idx=1,
        bag_path=db3_path,
        topic_name=target_topic,
        controller=controller,
        rx_worker=rx_worker,
        kp=20.0,
        kd=1.0,
    )

    # 3. 雙機鏈式安全急停 (Cascade E-STOP)
    def handle_estop(source_id: int, reason: str):
        print(f"\n[!!! E-STOP !!!] Motor ID {source_id} 觸發急停鏈: {reason}")
        vm_arm.trigger_estop(f"Cascade E-STOP from Motor {source_id}")
        vm_forearm.trigger_estop(f"Cascade E-STOP from Motor {source_id}")

    vm_arm.on_estop_triggered = lambda reason: handle_estop(1, reason)
    vm_forearm.on_estop_triggered = lambda reason: handle_estop(2, reason)

    vm_arm.on_status_changed = lambda msg: print(f"[M1 Log] {msg}")
    vm_forearm.on_status_changed = lambda msg: print(f"[M2 Log] {msg}")

    print("\n[ViewModel] 設定完成：")
    print(f"  - 軌跡來源  : {db3_path} ({target_topic})")
    print(f"  - 動作時長  : {vm_arm.target_duration:.2f} 秒")
    print(f"  - Motor ID 1 : Joint 0 | Kp={vm_arm.default_kp}, Kd={vm_arm.default_kd}")
    print(f"  - Motor ID 2 : Joint 1 | Kp={vm_forearm.default_kp}, Kd={vm_forearm.default_kd}")

    # 4. 啟動控制
    input("\n請確認雙馬達機構周圍安全, 按 Enter 鍵開始測試...")
    if not (vm_arm.start_control() and vm_forearm.start_control()):
        print("[ERROR] 控制啟動失敗！")
        vm_arm.stop_control()
        vm_forearm.stop_control()
        rx_worker.stop()
        bus.shutdown()
        return

    # 5. 主執行緒 Console 輪詢與即時數據顯示
    try:
        while True:
            s1 = vm_arm.get_ui_snapshot()
            s2 = vm_forearm.get_ui_snapshot()

            if s1.status == SystemStatus.PROBING or s2.status == SystemStatus.PROBING:
                print("\r[MIT] 等待馬達連線與探測中...", end="", flush=True)

            elif s1.status == SystemStatus.RUNNING or s2.status == SystemStatus.RUNNING:
                print(
                    f"\r[M1] Des:{s1.p_des:6.3f} Act:{s1.p_act:6.3f} Err:{s1.pos_error:5.3f} | "
                    f"[M2] Des:{s2.p_des:6.3f} Act:{s2.p_act:6.3f} Err:{s2.pos_error:5.3f} | "
                    f"T:{max(s1.timestamp, s2.timestamp):5.2f}s",
                    end="",
                    flush=True,
                )

            elif s1.status in [SystemStatus.IDLE, SystemStatus.E_STOPPED] and \
                 s2.status in [SystemStatus.IDLE, SystemStatus.E_STOPPED]:
                if s1.timestamp > 0 or s2.timestamp > 0 or s1.status == SystemStatus.E_STOPPED:
                    break

            time.sleep(0.02)  # 50Hz Terminal 刷新率

    except KeyboardInterrupt:
        print("\n\n[User Intervention] 使用者按下 Ctrl+C, 手動中斷測試!")
        vm_arm.trigger_estop("User Keyboard Interrupt (Ctrl+C)")
        vm_forearm.trigger_estop("User Keyboard Interrupt (Ctrl+C)")

    finally:
        # 6. 停止控制與釋放資源
        vm_arm.stop_control()
        vm_forearm.stop_control()
        rx_worker.stop()
        bus.shutdown()

        # 7. 計算報告與匯出 CSV Log
        print("\n\n" + "=" * 20 + " DB3 軌跡測試報告 " + "=" * 20)
        
        rep_arm = vm_arm.generate_quality_report()
        rms_arm = rep_arm["rms_error"]
        print(f"Motor 1 (Arm)     - RMS Error: {rms_arm:.4f} rad ({math.degrees(rms_arm):.2f}°)")
        print(f"                    Peak Speed: {rep_arm['peak_speed']:.2f} rad/s, Peak Torque: {rep_arm['peak_torque']:.2f} Nm")

        rep_forearm = vm_forearm.generate_quality_report()
        rms_forearm = rep_forearm["rms_error"]
        print(f"Motor 2 (Forearm) - RMS Error: {rms_forearm:.4f} rad ({math.degrees(rms_forearm):.2f}°)")
        print(f"                    Peak Speed: {rep_forearm['peak_speed']:.2f} rad/s, Peak Torque: {rep_forearm['peak_torque']:.2f} Nm")
        print("=" * 58)

        vm_arm.save_csv_log("arm_db3_hardware_tracking_log.csv")
        vm_forearm.save_csv_log("forearm_db3_hardware_tracking_log.csv")
        print("[Data Logger] 即時跟隨數據已成功寫入 arm_db3_hardware_tracking_log.csv 與 forearm_db3_hardware_tracking_log.csv\n")


if __name__ == "__main__":
    main()