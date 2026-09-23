# run_12motor_gait.py
import time
import math
import can
from core import MotorController, MotorControlViewModel, MotorRxWorker, MultiMotorViewModelManager
from protocol import MotorProtocol
from trajectory.gait_provider import QuadrupedGaitProvider

CAN_CHANNELS = ["can1", "can2", "can3", "can4"]

GAIT_FREQ_HZ = 1.2
STEP_HEIGHT_DEG = 15.0
KP_STAND = 55.0
KD = 2.5
TAU_MAX = 3.0
TEST_DURATION = 10.0

def query_real_angles(buses, controllers, workers) -> dict:
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
    print("  12 軸對角步態 (Trot Gait) 動態踏步測試 (標準介面重構版)")
    print("==================================================")

    manager = MultiMotorViewModelManager()
    buses, controllers, workers = {}, {}, {}

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
            
            vm.max_speed_rads = 8.0
            vm.max_torque_nm = 18.0
            vm.max_pos_error = 1.2

            provider = QuadrupedGaitProvider(
                channel=ch, 
                motor_id=m_id, 
                freq_hz=GAIT_FREQ_HZ,
                step_height_deg=STEP_HEIGHT_DEG,
                kp_stand=KP_STAND,
                kd=KD,
                tau_max=TAU_MAX
            )
            
            # 使用規範公共 API 注入 Provider
            vm.set_provider_instance(provider)
            manager.add_motor(key=vm_key, vm=vm, channel=ch, motor_id=m_id)

    real_angles = query_real_angles(buses, controllers, workers)
    
    print("\n" + "=" * 65)
    print(" 12 軸開機實體真實角度診斷表 ")
    print("=" * 65)
    
    has_error = False
    for key, deg in real_angles.items():
        if deg is None:
            print(f"  [{key:10s}] \033[91m[ERROR] 讀取失敗\033[0m")
            has_error = True
        elif abs(deg) < 0.01:
            print(f"  [{key:10s}] \033[91m[WARNING] 異常 0.0°\033[0m")
            has_error = True
        else:
            print(f"  [{key:10s}] 實體角度: \033[92m{deg:7.1f}°\033[0m (正常)")
    print("=" * 65)

    if has_error:
        print("\n\033[91m[安全鎖定] 角度診斷異常，中斷啟動！\033[0m")
        for w in workers.values(): w.stop()
        for b in buses.values(): b.shutdown()
        return

    print(f"\n【安全提示】首次測試強烈建議將機器狗【架空懸掛】測試！")
    input(f"按 Enter 啟動 {TEST_DURATION:.1f} 秒對角踏步測試...")
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
                    m_info.append(f"{ch}_Knee({s.position_deg:5.1f}°, {s.torque_act:4.1f}N)")
            
            print(f"\r[Gait Test] [T:{elapsed:4.1f}s/{TEST_DURATION:.1f}s] " + " | ".join(m_info), end="", flush=True)
            
            if elapsed >= TEST_DURATION:
                print(f"\n\n[自動完成] 已完成 {TEST_DURATION:.1f} 秒踏步測試，安全停止！")
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n[USER INTERRUPT] Ctrl+C 緊急停止！")
    finally:
        manager.stop_all()
        for w in workers.values(): w.stop()
        for b in buses.values(): b.shutdown()
        print("[STOP] 所有馬達已安全關閉。")

if __name__ == "__main__":
    main()