"""
多軸正弦波 (Sine Wave) 軌跡追蹤驗證腳本 (MVVM 架構)
套用先前 CSV 調校出的最佳 Kp, Kd 增益與站姿偏置，支援動態控管 1~3 顆馬達
"""

import math
import time
import can
from typing import List, Dict, Any

from core import MotorController, MotorControlViewModel, SystemStatus, MotorRxWorker

# ==============================================================================
# 1. 各關節馬達參數配置 (帶入先前 CSV 調校出的最佳化 Kp, Kd 與 URDF 站姿偏置)
# ==============================================================================
MOTOR_CONFIGS: Dict[int, Dict[str, Any]] = {
    1: {  # Motor 1: 右前髖 FR_HipY_joint
        "name": "FR_HipY (髖關節)",
        "amplitude": 0.5,           # 正弦波擺動振幅 (rad)
        "frequency": 0.5,           # 正弦波頻率 (Hz)
        "kp": 42.0,                 # CSV 實測最佳剛性
        "kd": 2.2,                  # CSV 實測最佳阻尼
        "m_leg": 1.8,
        "r_com": 0.20,
        "j_eq": 0.012,
        "stand_offset": -0.65,       # 正弦波中心偏置 (rad)，對齊 URDF 站姿 -0.65
        "axis_sign": 1.0,
        "enable_ff": True,
        "enable_inertia_ff": True,
        "max_gravity_ff": 2.0,
    },
    2: {  # Motor 2: 右前膝 FR_Knee_joint
        "name": "FR_Knee (膝關節)",
        "amplitude": 0.6,           # 正弦波擺動振幅 (rad)
        "frequency": 0.5,           # 正弦波頻率 (Hz)
        "kp": 40.0,                 # CSV 實測最佳剛性
        "kd": 2.6,                  # CSV 實測最佳阻尼
        "m_leg": 0.8,
        "r_com": 0.15,
        "j_eq": 0.003,
        "stand_offset": 1.30,      # 正弦波中心偏置 (rad)，對齊 URDF 站姿 1.30
        "axis_sign": 1.0,
        "enable_ff": True,
        "enable_inertia_ff": True,
        "max_gravity_ff": 1.8,
    },
    3: {  # Motor 3: 預留關節 (如 FR_HipX_joint)
        "name": "FR_HipX (預留關節)",
        "amplitude": 0.3,
        "frequency": 0.5,
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
    controller: MotorController,
    rx_worker: MotorRxWorker,
    config: Dict[str, Any],
    control_freq_hz: float = 100.0,
    target_duration: float = 10.0
) -> MotorControlViewModel:
    # 1. 建立 ViewModel 並對接 Model
    vm = MotorControlViewModel(controller = controller, rx_worker = rx_worker)

    # 2. 載入 ViewModel 系統與安全保護參數
    vm.motor_id = motor_id
    vm.control_freq_hz = control_freq_hz
    vm.target_duration = target_duration
    vm.max_speed_rads = 15.0       # 最大速度保護 (rad/s)
    vm.max_torque_nm = 20.0        # 解鎖最大扭矩保護上限 (Nm)
    vm.max_pos_error = 2.0         # 放寬跟隨誤差保護門檻 (rad)
    vm.telemetry_timeout_s = 0.3
    vm.control_start_delay = 0.2

    # 3. 透過 ProviderFactory 實例化 Sine Wave 軌跡產生器
    vm.select_provider(
        "Sine",
        amplitude = config.get("amplitude", 0.5),
        frequency = config.get("frequency", 0.5),
        kp = config.get("kp", 20.0),
        kd = config.get("kd", 1.0),
        enable_ff = config["enable_ff"],
        enable_inertia_ff = config["enable_inertia_ff"],
        m_leg = config["m_leg"],
        r_com = config["r_com"],
        j_eq = config["j_eq"],
        stand_offset = config["stand_offset"],
        axis_sign = config["axis_sign"],
        max_gravity_ff = config["max_gravity_ff"]
    )
    return vm

def main():
    print("==================================================")
    print(" 多軸正弦波 (Sine) 軌跡測試 (動態控管 1~3 顆馬達)")
    print("==================================================")

    # ==========================================================
    # 設定要啟動測試的馬達 ID 列表 (例如 [1], [1, 2] 或 [1, 2, 3])
    # ==========================================================
    ACTIVE_MOTOR_IDS = [1, 2]  # <-- 在此彈性調整測試的馬達組合

    print(f"\n[配置] 當前啟動之馬達清單: {ACTIVE_MOTOR_IDS}")
    for mid in ACTIVE_MOTOR_IDS:
        cfg = MOTOR_CONFIGS.get(mid, MOTOR_CONFIGS[3])
        print(f"  - Motor ID {mid} ({cfg['name']}): Amp={cfg['amplitude']}, Freq={cfg['frequency']}Hz, Stand_Offset={cfg['stand_offset']}rad | Kp={cfg['kp']}, Kd={cfg['kd']}")

    # 1. 建立 CAN Bus 與背景 RxWorker
    try:
        bus = can.ThreadSafeBus(channel="canfd0", interface="socketcan")
        controller = MotorController(bus)
        rx_worker = MotorRxWorker(controller)
        rx_worker.start()
        print("\n[CAN] SocketCAN (canfd0) 與背景 RxWorker 初始化成功。")
    except Exception as e:
        print(f"[ERROR] CAN Bus 初始化失敗，請檢查硬體或 set_canfd0_script.sh: {e}")
        return

    # 2. 動態建立各馬達 ViewModel
    vms: List[MotorControlViewModel] = []
    for mid in ACTIVE_MOTOR_IDS:
        cfg = MOTOR_CONFIGS.get(mid, MOTOR_CONFIGS[3])
        vm = create_configured_vm(
            motor_id = mid,
            controller = controller,
            rx_worker = rx_worker,
            config = cfg,
            control_freq_hz = 100.0,   # 正弦波測試可設 100 Hz
            target_duration = 10.0     # 測試時長 10 秒
        )
        vms.append(vm)

    # 3. 綁定動態聯鎖急停 (Cascade E-STOP) 與個別 Terminal Log
    def handle_cascade_estop(source_id: int, reason: str):
        print(f"\n[!!! 聯鎖急停觸發 !!!] 馬達 ID {source_id} 引發保護: {reason}")
        for vm in vms:
            vm.trigger_estop(f"Cascade E-STOP from Motor {source_id}")

    for vm in vms:
        mid = vm.motor_id
        vm.on_estop_triggered = lambda reason, m_id=mid: handle_cascade_estop(m_id, reason)
        vm.on_status_changed = lambda msg, m_id=mid: print(f"[Motor {m_id} Log] {msg}")

    # 4. 控制啟動確認
    input("\n請確認馬達周圍安全, 按 Enter 鍵開始測試...")
    
    start_success = all(vm.start_control() for vm in vms)
    if not start_success:
        print("[ERROR] 部分或全部馬達控制啟動失敗！執行緊急停止程序...")
        for vm in vms:
            vm.stop_control()
        rx_worker.stop()
        bus.shutdown()
        return

    # 5. 主執行緒 Console 輪詢與即時狀態監測
    try:
        while True:
            snapshots = [vm.get_ui_snapshot() for vm in vms]

            if any(s.status == SystemStatus.PROBING for s in snapshots):
                print("\r[MIT] 等待多軸馬達連線與探測中...", end = "", flush = True)

            all_finished = all(s.status in [SystemStatus.IDLE, SystemStatus.E_STOPPED] for s in snapshots)
            has_started = any(s.timestamp > 0 or s.status == SystemStatus.E_STOPPED for s in snapshots)

            if all_finished and has_started:
                break

            time.sleep(0.02)  # 50Hz Terminal 刷新頻率

    except KeyboardInterrupt:
        print("\n\n[User Intervention] 使用者按下 Ctrl+C, 手動中斷測試!")
        for vm in vms:
            vm.trigger_estop("User Keyboard Interrupt (Ctrl+C)")

    finally:
        # 6. 停止控制並釋放 CAN Bus 資源
        for vm in vms:
            vm.stop_control()
        rx_worker.stop()
        bus.shutdown()

        # 7. 動態匯出多軸測試報告與 CSV Log
        print("\n\n" + "=" * 15 + " 正弦波測試報告 " + "=" * 15)
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

            csv_file = f"m{vm.motor_id}_sine_hardware_tracking_log.csv"
            vm.save_csv_log(csv_file)
            print(f"[Data Logger] 馬達 {vm.motor_id} 即時跟隨數據已匯出至 {csv_file}")

        print("=" * 47 + "\n")

if __name__ == "__main__":
    main()