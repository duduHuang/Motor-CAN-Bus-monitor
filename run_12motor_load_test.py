# run_12motor_load_test.py
import time
import sys
import select
import termios
import tty
import can
import csv
import datetime
from core import MotorController, MotorControlViewModel, MotorRxWorker, MultiMotorViewModelManager
from protocol import MotorProtocol
from trajectory.fsm_provider import QuadrupedFSMProvider, PostureState

CAN_CHANNELS = ["can1", "can2", "can3", "can4"]

# ==============================================================================
# RMD-X4-P36 測試與安全保護參數設定
# ==============================================================================
TEST_DURATION_S = 300.0         # 測試時間：5 分鐘 (300 秒)
SAFE_MAX_TEMP_C = 75.0          # 第一次測試保護閾值 (原廠外殼極限 85°C)
SAFE_MAX_TORQUE_NM = 15.0       # 第一次測試急停門檻 (原廠額定 10.5Nm)
FSM_TAU_MAX = 8.0               # FSM 剛性輸出上限 (避免起飛暴衝)

class NonBlockingKeyboard:
    """Linux 終端機非阻塞式鍵盤監聽器"""
    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

def query_real_angles(buses, controllers, workers) -> dict:
    angles_deg = {}
    print("\n[診斷] 正在發送查詢指令以確認硬體連線...")
    for ch in CAN_CHANNELS:
        for m_id in [1, 2, 3]:
            try: controllers[ch].send_single_command(m_id, MotorProtocol.read_status_2())
            except Exception: pass
    time.sleep(0.5)

    for ch in CAN_CHANNELS:
        for m_id in [1, 2, 3]:
            vm_key = f"{ch}_ID{m_id}"
            msg = workers[ch].get_specific_telemetry(m_id, "StandardMotionTelemetry")
            angles_deg[vm_key] = float(msg.telemetry.temperature_c) if (msg and msg.telemetry) else None
    return angles_deg

def set_robot_posture(manager: MultiMotorViewModelManager, state: PostureState):
    """向 12 顆馬達下發新姿態指令"""
    for ch in CAN_CHANNELS:
        for m_id in [1, 2, 3]:
            vm = manager.get_vm(f"{ch}_ID{m_id}")
            if vm:
                provider = vm.get_current_provider()
                if provider and hasattr(provider, 'set_target_state'):
                    provider.set_target_state(state)

def main():
    print("==================================================")
    print(f"  RMD-X4-P36 12 軸四足負重 3KG 站立測試 (純 MVVM 規範版)")
    print(f"  安全限制: 預警溫度 {SAFE_MAX_TEMP_C}°C | 扭矩上限 {SAFE_MAX_TORQUE_NM}Nm")
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
            print(f"  [X] 開啟網卡 {ch} 失敗: {e}"); return

        for m_id in [1, 2, 3]:
            vm_key = f"{ch}_ID{m_id}"
            vm = MotorControlViewModel(controller=ctrl, rx_worker=worker)
            vm.motor_id = m_id
            vm.max_speed_rads, vm.max_torque_nm, vm.max_pos_error = 8.0, SAFE_MAX_TORQUE_NM, 1.2
            
            provider = QuadrupedFSMProvider(
                channel=ch, motor_id=m_id,
                kp_soft=35.0, kp_hard=55.0, kd=2.5, tau_max=FSM_TAU_MAX
            )
            vm._current_provider = provider
            manager.add_motor(key=vm_key, vm=vm, channel=ch, motor_id=m_id)

    real_angles = query_real_angles(buses, controllers, workers)
    has_error = False
    for key, deg in real_angles.items():
        if deg is None or abs(deg) < 0.01:
            print(f"  [{key:10s}] \033[91m[ERROR] 角度無效 ({deg})\033[0m")
            has_error = True
    if has_error:
        print("\n\033[91m[安全鎖定] 馬達角度診斷異常，中斷啟動！請檢查供電與 CAN 總線。\033[0m")
        for w in workers.values(): w.stop()
        for b in buses.values(): b.shutdown()
        return

    print("\n按 [Enter] 啟動馬達使能，準備進入測試...")
    input()
    manager.start_all()
    set_robot_posture(manager, PostureState.PRONE)

    print("\n" + "="*50)
    print(" 測試控制: [S] 開始 5 分鐘負重站立測試 | [Q] 退出")
    print("="*50 + "\n")

    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"rmd_x4_load_3kg_5min_{timestamp_str}.csv"
    csv_file = open(csv_filename, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Time_s", "State", "Channel", "Motor_ID", "Pos_deg", "Torque_Nm", "Temp_C", "Current_A"])

    test_running = False
    stand_start_time = None
    current_state_name = "PRONE"
    emergency_trigger = None

    try:
        with NonBlockingKeyboard() as kb:
            while True:
                now_t = time.time()
                key = kb.get_key()
                
                if key:
                    key_lower = key.lower()
                    if key_lower == 's' and not test_running:
                        set_robot_posture(manager, PostureState.STAND)
                        current_state_name = "STAND"
                        stand_start_time = now_t
                        test_running = True
                        print("\n\033[92m[測試開始] 機器人起立，開始 300 秒負重耐久測試...\033[0m")
                    elif key_lower == 'q':
                        print("\n[退出] 接收到退出指令 Q，安全卸載歸位...")
                        break

                elapsed_s = (now_t - stand_start_time) if test_running else 0.0
                if test_running and elapsed_s >= TEST_DURATION_S:
                    print("\n\n\033[92m[測試成功] 5 分鐘負重測試已達標！自動降下姿態...\033[0m")
                    break

                m_info_ui = []
                for ch in CAN_CHANNELS:
                    for m_id in [1, 2, 3]:
                        vm = manager.get_vm(f"{ch}_ID{m_id}")
                        if not vm: continue
                        
                        # 【符合 MVVM 規範】100% 從 ViewModel Snapshot 取出所有狀態
                        s = vm.get_ui_snapshot()

                        if test_running:
                            csv_writer.writerow([
                                round(elapsed_s, 3), current_state_name, 
                                ch, m_id, 
                                round(s.position_deg, 2), round(s.torque_act, 2), 
                                round(s.temperature, 1), round(s.current_a, 2)
                            ])

                        # 安全檢測 (保護條件完全基於 Snapshot)
                        if test_running:
                            if abs(s.torque_act) > SAFE_MAX_TORQUE_NM:
                                emergency_trigger = f"{ch}_ID{m_id} 扭矩超載 ({s.torque_act:.1f}Nm > {SAFE_MAX_TORQUE_NM}Nm)"
                            if s.temperature > SAFE_MAX_TEMP_C:
                                emergency_trigger = f"{ch}_ID{m_id} 溫度過高 ({s.temperature:.1f}°C > {SAFE_MAX_TEMP_C}°C)"
                        
                        if m_id == 3:
                            m_info_ui.append(f"{ch}({s.torque_act:4.1f}N, {s.temperature:2.0f}°C)")

                if emergency_trigger:
                    print(f"\n\n\033[91m[緊急保護觸發] {emergency_trigger}！靜態平滑降下姿態！\033[0m")
                    break

                if test_running:
                    remains = TEST_DURATION_S - elapsed_s
                    print(f"\r[倒數: {remains:5.1f}s] " + " | ".join(m_info_ui), end="", flush=True)
                else:
                    print(f"\r[等待中: 按 'S' 開始] " + " | ".join(m_info_ui), end="", flush=True)
                
                time.sleep(0.1) # 10Hz 採樣率

    except KeyboardInterrupt:
        print("\n\n[USER INTERRUPT] Ctrl+C 強制中斷！")
    finally:
        print("\n[安全關閉] 平滑趴下姿態，卸載負重...")
        set_robot_posture(manager, PostureState.PRONE)
        time.sleep(2.5)
        
        manager.stop_all()
        for w in workers.values(): w.stop()
        for b in buses.values(): b.shutdown()
        
        csv_file.close()
        print(f"\n[STOP] 馬達已關閉，CAN 已釋放。")
        print(f"[Data Logger] 測試紀錄已寫入: {csv_filename}")

if __name__ == "__main__":
    main()