#!/usr/bin/env python3
"""
run_csv_hardware_test.py
動態多軸 CSV 軌跡追蹤測試腳本 (MVVM 架構)
支援 1~3 顆馬達動態載入、物理前饋模型自動匹配與聯鎖急停保護
"""

import math
import os
import sys
import time
import can
from typing import Dict, Any, List

from core import MotorController, MotorControlViewModel, SystemStatus, MotorRxWorker

# ==============================================================================
# 1. 各關節馬達標準測試與物理前饋參數配置 (定案最佳化參數)
# ==============================================================================
MOTOR_DEFAULT_CONFIGS: Dict[int, Dict[str, Any]] = {
    1: {  # Motor 1: 右前髖 FR_HipY_joint
        "name": "FR_HipY (髖關節)",
        "kp": 42.0,
        "kd": 2.2,
        "m_leg": 1.8,
        "r_com": 0.20,
        "j_eq": 0.012,
        "stand_offset": -0.65,
        "axis_sign": 1.0,
        "enable_ff": True,
        "enable_inertia_ff": True,
        "max_gravity_ff": 2.0,
    },
    2: {  # Motor 2: 右前膝 FR_Knee_joint
        "name": "FR_Knee (膝關節)",
        "kp": 40.0,
        "kd": 2.6,
        "m_leg": 0.8,
        "r_com": 0.15,
        "j_eq": 0.003,
        "stand_offset": 1.30,
        "axis_sign": 1.0,
        "enable_ff": True,
        "enable_inertia_ff": True,
        "max_gravity_ff": 1.8,
    },
    3: {  # Motor 3: 預留關節 (如 FR_HipX_joint)
        "name": "FR_HipX (預留關節)",
        "kp": 30.0,
        "kd": 2.0,
        "m_leg": 0.5,
        "r_com": 0.10,
        "j_eq": 0.005,
        "stand_offset": 0.0,
        "axis_sign": 1.0,
        "enable_ff": True,
        "enable_inertia_ff": False,
        "max_gravity_ff": 1.2,
    }
}


def create_configured_vm(
    motor_id: int,
    csv_filename: str,
    controller: MotorController,
    rx_worker: MotorRxWorker,
    control_freq_hz: float = 500.0,
    duration: float = 180.0
) -> MotorControlViewModel:
    """動態實例化並設定指定 Motor ID 的 ViewModel"""
    vm = MotorControlViewModel(controller = controller, rx_worker = rx_worker)

    # 1. 取得該馬達專屬配置 (若無匹配則自動降級使用 Motor 3 預設值)
    cfg = MOTOR_DEFAULT_CONFIGS.get(motor_id, MOTOR_DEFAULT_CONFIGS[3])

    # 2. 設置控制與安全保護參數
    vm.motor_id = motor_id
    vm.control_freq_hz = control_freq_hz
    vm.target_duration = duration
    vm.max_speed_rads = 15.0       # 最大速度保護 (rad/s)
    vm.max_torque_nm = 20.0        # 解鎖最大扭矩保護上限 (Nm)
    vm.max_pos_error = 2.0         # 放寬跟隨誤差保護門檻 (rad)
    vm.telemetry_timeout_s = 0.3
    vm.control_start_delay = 0.2

    # 3. 觀察者 Log 事件繫結
    vm.on_status_changed = lambda msg: print(f"\n[Motor {motor_id} Log] {msg}")

    # 4. 載入指定的 CSV 軌跡檔案與前饋模型參數
    vm.select_provider(
        "CSV",
        csv_filepath = csv_filename,
        kp = cfg["kp"],
        kd = cfg["kd"],
        enable_ff = cfg["enable_ff"],
        enable_inertia_ff = cfg["enable_inertia_ff"],
        m_leg = cfg["m_leg"],
        r_com = cfg["r_com"],
        j_eq = cfg["j_eq"],
        stand_offset = cfg["stand_offset"],
        axis_sign = cfg["axis_sign"],
        max_gravity_ff = cfg["max_gravity_ff"]
    )
    return vm


def main():
    # 1. 命令列參數檢查 (支援 1~3 個 CSV 檔案)
    csv_files = sys.argv[1:]
    if not (1 <= len(csv_files) <= 3):
        print("\n[使用錯誤] 命令列參數不正確！")
        print("指令範例:")
        print("  1 顆馬達: python run_csv_hardware_test.py test.csv")
        print("  2 顆馬達: python run_csv_hardware_test.py test1.csv test2.csv")
        print("  3 顆馬達: python run_csv_hardware_test.py test1.csv test2.csv test3.csv\n")
        return

    # 2. 檢查 CSV 檔案是否存在，避免發送硬體命令後才出錯
    for path in csv_files:
        if not os.path.exists(path):
            print(f"\n[ERROR] 找不到指定的 CSV 軌跡檔案: '{path}'，請確認路徑！\n")
            return

    motor_count = len(csv_files)
    print("==================================================")
    print(f"  動態多馬達 CSV 軌跡追蹤測試 (共 {motor_count} 顆馬達)")
    print("==================================================")

    # 3. 初始化 SocketCAN 匯流排
    try:
        bus = can.ThreadSafeBus(channel = "canfd0", interface = "socketcan")
        controller = MotorController(bus)
        rx_worker = MotorRxWorker(controller)
        rx_worker.start()
        print("[CAN] SocketCAN (canfd0) 與背景 RxWorker 初始化成功。")
    except Exception as e:
        print(f"[ERROR] CAN Bus 初始化失敗: {e}")
        return

    # 4. 動態建立 ViewModel 列表 (Motor ID 依序為 1, 2, 3)
    vms: List[MotorControlViewModel] = []
    for i, csv_path in enumerate(csv_files, start = 1):
        vm = create_configured_vm(
            motor_id = i,
            csv_filename = csv_path,
            controller = controller,
            rx_worker = rx_worker,
            control_freq_hz = 500.0,
            duration = 360.0
        )
        vms.append(vm)

    # 5. 全局聯鎖急停 (Cascade E-STOP): 任一馬達觸發急停，其餘馬達同步停機
    def handle_cascade_estop(source_id: int, reason: str):
        print(f"\n[!!! 聯鎖急停觸發 !!!] 馬達 ID {source_id} 引發保護: {reason}")
        for vm in vms:
            vm.trigger_estop(f"Cascade Protection (由 Motor {source_id} 觸發)")

    for vm in vms:
        m_id = vm.motor_id
        vm.on_estop_triggered = lambda reason, m_id=m_id: handle_cascade_estop(m_id, reason)

    print("\n[ViewModel] 已成功載入軌跡設定與物理前饋參數:")
    for vm, csv_path in zip(vms, csv_files):
        cfg = MOTOR_DEFAULT_CONFIGS.get(vm.motor_id, MOTOR_DEFAULT_CONFIGS[3])
        print(f"  - Motor ID {vm.motor_id} ({cfg['name']}): '{csv_path}' | Kp={cfg['kp']}, Kd={cfg['kd']}")

    input("\n請確認所有馬達周圍安全, 按 Enter 鍵開始測試...")

    # 6. 同步啟動所有馬達控制
    if not all(vm.start_control() for vm in vms):
        print("[ERROR] 控制啟動失敗！執行安全復原程序...")
        for vm in vms:
            vm.stop_control()
        rx_worker.stop()
        bus.shutdown()
        return

    # 7. 動態 Console 狀態顯示迴圈
    try:
        while True:
            snapshots = [vm.get_ui_snapshot() for vm in vms]

            # 只要有任一馬達在探測中
            if any(s.status == SystemStatus.PROBING for s in snapshots):
                print("\r[MIT] 等待馬達連線與探測中...", end = "", flush = True)

            # 任一馬達運行中，格式化顯示所有馬達即時數據
            elif any(s.status == SystemStatus.RUNNING for s in snapshots):
                cur_t = max(s.timestamp for s in snapshots)
                m_info_list = [
                    f"M{vm.motor_id}(P:{s.p_act:6.3f}, T:{s.torque_act:5.2f}N, Err:{s.pos_error:5.3f})"
                    for vm, s in zip(vms, snapshots)
                ]
                print(
                    f"\r[MIT] T:{cur_t:5.2f}s | " + " | ".join(m_info_list),
                    end = "",
                    flush = True
                )

            # 所有馬達均已停止或急停，跳出迴圈
            elif all(s.status in [SystemStatus.IDLE, SystemStatus.E_STOPPED] for s in snapshots):
                if any(s.timestamp > 0 or s.status == SystemStatus.E_STOPPED for s in snapshots):
                    break

            time.sleep(0.02)  # 50Hz Terminal 刷新率

    except KeyboardInterrupt:
        print("\n\n[User Intervention] 使用者手動按下 Ctrl+C 中斷測試！")
        for vm in vms:
            vm.trigger_estop("User Keyboard Interrupt (Ctrl+C)")

    finally:
        # 8. 安全停止所有馬達並釋放匯流排
        for vm in vms:
            vm.stop_control()
        rx_worker.stop()
        bus.shutdown()

        # 9. 匯出各馬達測試報告與數據 Log
        print("\n\n" + "=" * 15 + " 多軸 CSV 測試報告 " + "=" * 15)
        for vm in vms:
            report = vm.generate_quality_report()
            rms_err = report.get("rms_error", 0.0)
            peak_speed = report.get("peak_speed", 0.0)
            peak_torque = report.get("peak_torque", 0.0)

            print(f"[Motor ID: {vm.motor_id}]")
            print(f"  RMS Position Error : {rms_err:.4f} rad ({math.degrees(rms_err):.2f}°)")
            print(f"  Peak Velocity      : {peak_speed:.2f} rad/s")
            print(f"  Peak Torque        : {peak_torque:.2f} Nm")
            print("-" * 47)

            log_name = f"m{vm.motor_id}_hardware_tracking_log.csv"
            vm.save_csv_log(log_name)
            print(f"[Data Logger] 馬達 {vm.motor_id} 即時數據已匯出至 {log_name}")
        print("=" * 47 + "\n")


if __name__ == "__main__":
    main()