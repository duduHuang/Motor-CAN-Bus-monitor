# run_safe_12motor_stand.py
import time
import math
import can
from core import MotorController, MotorControlViewModel, MotorRxWorker, MultiMotorViewModelManager
from protocol import MotorProtocol
from trajectory.stand_hold_provider import StandHoldTrajectoryProvider

CAN_CHANNELS = ["can1", "can2", "can3", "can4"]

# ===== 時間控制參數配置 =====
RAMP_TIME_SEC = 5.0    # 姿態平滑過渡時間 (秒)
HOLD_TIME_SEC = 5.0    # 站立到達後保持鎖定觀察的秒數 (設 0 則到達立即停止)
TOTAL_DURATION = RAMP_TIME_SEC + HOLD_TIME_SEC

def query_real_angles(buses, controllers, workers) -> dict:
    """開機前主動發送 0x9C 讀取指令，抓取 12 軸真實實體角度 (°)，失敗不允許開啟控制"""
    angles_deg = {}
    print("\n[診斷] 正在向 12 顆馬達發送 0x9C 實體角度查詢指令...")
    
    for ch in CAN_CHANNELS:
        controller = controllers[ch]
        for m_id in [1, 2, 3]:
            try:
                cmd_9c = MotorProtocol.read_status_2()
                controller.send_single_command(m_id, cmd_9c)
            except Exception:
                pass
    
    time.sleep(0.5)

    for ch in CAN_CHANNELS:
        worker = workers[ch]
        for m_id in [1, 2, 3]:
            vm_key = f"{ch}_ID{m_id}"
            msg = worker.get_specific_telemetry(m_id, "StandardMotionTelemetry")
            if msg and msg.telemetry:
                deg = float(msg.telemetry.angle_deg)
                angles_deg[vm_key] = deg
            else:
                angles_deg[vm_key] = None

    return angles_deg

def main():
    print("==================================================")
    print("  12 軸全車安全站立測試 (自動計時終止版)")
    print("==================================================")

    manager = MultiMotorViewModelManager()
    buses, controllers, workers = {}, {}, {}

    # STEP 1: 初始化 4 個 CAN Bus
    for ch in CAN_CHANNELS:
        try:
            bus = can.ThreadSafeBus(channel=ch, interface='socketcan')
            buses[ch] = bus
            ctrl = MotorController(bus)
            controllers[ch] = ctrl
            worker = MotorRxWorker(ctrl)
            worker.start()
            workers[ch] = worker
            print(f"  [✓] 網卡 {ch} 初始化成功。")
        except Exception as e:
            print(f"  [X] 開啟網卡 {ch} 失敗: {e}")
            return

        for m_id in [1, 2, 3]:
            vm_key = f"{ch}_ID{m_id}"
            vm = MotorControlViewModel(controller=ctrl, rx_worker=worker)
            vm.motor_id = m_id
            
            vm.max_speed_rads = 5.0     # 限制最大速度 (rad/s)
            vm.max_torque_nm = 10.0     # 限制最大扭矩 (Nm)
            vm.max_pos_error = 0.8      # 限制跟隨誤差 (rad)

            provider = StandHoldTrajectoryProvider(
                channel=ch, 
                motor_id=m_id, 
                up_ramp_sec=RAMP_TIME_SEC, 
                kp=20.0,
                kd=1.5
            )
            vm._current_provider = provider
            manager.add_motor(key=vm_key, vm=vm, channel=ch, motor_id=m_id)

    # STEP 2: 核心主動角度診斷
    real_angles = query_real_angles(buses, controllers, workers)
    
    print("\n" + "=" * 65)
    print(" 12 軸開機實體真實角度診斷表 (必須確認無 0.0° 與 None) ")
    print("=" * 65)
    
    has_error = False
    for key, deg in real_angles.items():
        if deg is None:
            print(f"  [{key:10s}] \033[91m[ERROR] 讀取失敗 (無 CAN 回應)\033[0m")
            has_error = True
        elif abs(deg) < 0.01:
            print(f"  [{key:10s}] \033[91m[WARNING] 角度為 {deg:.1f}° (異常 0.0°，可能未通電/通訊失敗)\033[0m")
            has_error = True
        else:
            print(f"  [{key:10s}] 實體角度: \033[92m{deg:7.1f}°\033[0m (正常)")
    print("=" * 65)

    if has_error:
        print("\n\033[91m[安全鎖定] 檢測到部分馬達角度為 0.0° 或無回應，已強制中斷！\033[0m")
        for w in workers.values(): w.stop()
        for b in buses.values(): b.shutdown()
        return

    # STEP 3: 啟動控制與自動計時終止
    input(f"\n【請確認機器人已懸空架起】按 Enter 啟動站立 (將於 {TOTAL_DURATION:.1f} 秒後自動停止)...")
    manager.start_all()

    try:
        start_t = time.time()
        while True:
            elapsed = time.time() - start_t
            
            m_info = []
            for ch in CAN_CHANNELS:
                vm = manager.get_vm(f"{ch}_ID3")
                if vm:
                    s = vm.get_ui_snapshot()
                    m_info.append(f"{ch}_M3(P:{s.position_deg:5.1f}°, T:{s.torque_act:4.1f}N)")
            
            print(f"\r[T:{elapsed:4.1f}s/{TOTAL_DURATION:.1f}s] " + " | ".join(m_info), end="", flush=True)
            
            # 【自動計時終止】當執行時間到達目標總時間時自動跳出
            if elapsed >= TOTAL_DURATION:
                print(f"\n\n[自動完成] 已到達指定運轉時間 {TOTAL_DURATION:.1f} 秒，自動結束測試！")
                break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n[USER INTERRUPT] 使用者按下 Ctrl+C，手動停止所有馬達！")
    finally:
        manager.stop_all()
        for w in workers.values(): w.stop()
        for b in buses.values(): b.shutdown()
        print("[STOP] 所有馬達已安全關閉並釋放匯流排。")

if __name__ == "__main__":
    main()