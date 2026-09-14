#!/usr/bin/env python3
"""
Phase 3 實體單馬達驅動驗證腳本 (MVVM 架構)
測試設定: Kp = 20.0, Kd = 1.0, Amp = 0.5 rad, Freq = 0.5 Hz, Max Pos Error = 0.8 rad
"""

import math
import time
import can

from core import MotorController, MotorControlViewModel, SystemStatus, MotorRxWorker

def create_configured_vm(
    motor_id: int,
    controller: MotorController,
    rx_worker: MotorRxWorker,
    amplitude: float = 0.5,
    frequency: float = 0.5,
    offset: float = 0.0
) -> MotorControlViewModel:
    # 1. 建立 ViewModel 並對接 Model
    vm = MotorControlViewModel(controller = controller, rx_worker = rx_worker)

    # 2. 綁定觀察者事件 (Terminal 輸出)
    vm.on_status_changed = lambda msg: print(f"\n[System Log] {msg}")
    vm.on_estop_triggered = lambda reason: print(f"\n[!!! E-STOP 觸發 !!!] 原因: {reason}")

    # 3. 載入 Phase 3 測試與安全保護參數
    vm.motor_id = motor_id                  # 目標馬達 ID
    vm.control_freq_hz = 100.0       # 控制頻率 100 Hz (10ms)
    vm.target_duration = 10.0        # 測試時長 10 秒
    vm.max_speed_rads = 8.0          # 最大速度保護閾值 (rad/s)
    vm.max_torque_nm = 5.0           # 最大扭矩保護閾值 (Nm)
    vm.max_pos_error = 0.8           # Phase 3 跟隨誤差閾值 (rad)
    vm.telemetry_timeout_s = 0.3     # 通訊超時 (s)
    vm.control_start_delay = 0.2     # 啟動保護延遲 (s)
    
    # 4. 透過 ProviderFactory 實例化 Phase 3 正弦波產生器
    vm.select_provider(
        "Sine",
        amplitude = amplitude,
        frequency = frequency,
        offset = offset,
        kp = 20.0,
        kd = 1.0
    )
    return vm

def main():
    print("==================================================")
    print("  Phase 3: 高剛度正弦波跟隨測試 (單馬達 CAN 驅動)")
    print("==================================================")

    # 1. 建立 SocketCAN 實體與底層 Model 組件
    try:
        bus = can.ThreadSafeBus(channel = "can0", interface = "socketcan")
        controller = MotorController(bus)
        rx_worker = MotorRxWorker(controller)
        rx_worker.start()
        print("[CAN] SocketCAN (can0) 與背景 RxWorker 初始化成功。")
    except Exception as e:
        print(f"[ERROR] CAN Bus 初始化失敗，請檢查硬體或 set_can0_script.sh: {e}")
        return

    # 2. 僅建立單一馬達 ViewModel (Motor ID = 1)
    vm_arm = create_configured_vm(
        motor_id = 1,
        controller = controller,
        rx_worker = rx_worker
    )

    print("\n[ViewModel] 設定完成：")
    print("  - Trajectory : SineWave (Amp=0.5 rad, Freq=0.5 Hz)")
    print("  - MIT Gains  : Kp=20.0, Kd=1.0")
    print("  - Protection : Max Error=0.8 rad, Max Speed=8.0 rad/s, Max Torque=5.0 Nm")

    # 3. 啟動控制
    input("\n請確認馬達周圍安全, 按 Enter 鍵開始測試...")
    if not vm_arm.start_control():
        print("[ERROR] 控制啟動失敗！")
        rx_worker.stop()
        bus.shutdown()
        return

    # 4. 主執行緒 Console 輪詢與即時數據顯示
    try:
        while True:
            snapshot = vm_arm.get_ui_snapshot()

            if snapshot.status == SystemStatus.PROBING:
                print("\r[MIT] 等待馬達連線與探測中...", end="", flush=True)

            elif snapshot.status == SystemStatus.RUNNING:
                print(
                    f"\r[MIT] T: {snapshot.timestamp:5.2f}s | "
                    f"P_des: {snapshot.p_des:6.3f} | "
                    f"P_act: {snapshot.p_act:6.3f} | "
                    f"V_act: {snapshot.v_act:6.3f} | "
                    f"Torque: {snapshot.torque_act:6.3f} Nm | "
                    f"Err: {snapshot.pos_error:5.3f}",
                    end="",
                    flush=True
                )

            elif snapshot.status in [SystemStatus.IDLE, SystemStatus.E_STOPPED]:
                if snapshot.timestamp > 0 or snapshot.status == SystemStatus.E_STOPPED:
                    break

            time.sleep(0.02)  # 50Hz Terminal 刷新頻率

    except KeyboardInterrupt:
        print("\n\n[User Intervention] 使用者按下 Ctrl+C, 手動中斷測試!")
        vm_arm.trigger_estop("User Keyboard Interrupt (Ctrl+C)")

    finally:
        # 5. 確保控制停止並釋放資源
        vm_arm.stop_control()
        rx_worker.stop()
        bus.shutdown()

        # 6. 計算報告與匯出 CSV 檔案
        print("\n\n" + "=" * 15 + " Phase 3 測試報告 " + "=" * 15)
        report = vm_arm.generate_quality_report()
        
        rms_err = report["rms_error"]
        print(f"RMS Position Error  : {rms_err:.4f} rad ({math.degrees(rms_err):.2f}°)")
        print(f"Peak Velocity       : {report['peak_speed']:.2f} rad/s")
        print(f"Peak Torque         : {report['peak_torque']:.2f} Nm")
        print("=" * 47)

        csv_file = "phase3_hardware_tracking_log.csv"
        vm_arm.save_csv_log(csv_file)
        print(f"[Data Logger] 即時跟隨數據已成功匯出至 {csv_file}\n")


if __name__ == "__main__":
    main()