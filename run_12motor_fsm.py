# run_12motor_fsm.py
import time
import sys
import select
import termios
import tty
import can
from core import MotorController, MotorControlViewModel, MotorRxWorker, MultiMotorViewModelManager
from protocol import MotorProtocol
from trajectory.fsm_provider import QuadrupedFSMProvider, PostureState

CAN_CHANNELS = ["can1", "can2", "can3", "can4"]

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
    print("\n[診斷] 正在發送 0x9C 查詢指令...")
    for ch in CAN_CHANNELS:
        for m_id in [1, 2, 3]:
            try:
                controllers[ch].send_single_command(m_id, MotorProtocol.read_status_2())
            except Exception: pass
    time.sleep(0.5)

    for ch in CAN_CHANNELS:
        for m_id in [1, 2, 3]:
            vm_key = f"{ch}_ID{m_id}"
            msg = workers[ch].get_specific_telemetry(m_id, "StandardMotionTelemetry")
            angles_deg[vm_key] = float(msg.telemetry.angle_deg) if (msg and msg.telemetry) else None
    return angles_deg

def set_robot_posture(manager: MultiMotorViewModelManager, state: PostureState):
    """向 12 顆馬達下發新姿態指令（使用 ViewModel 公共 API 訪問 Provider）"""
    for ch in CAN_CHANNELS:
        for m_id in [1, 2, 3]:
            vm = manager.get_vm(f"{ch}_ID{m_id}")
            if vm:
                provider = vm.get_current_provider()
                if provider and hasattr(provider, 'set_target_state'):
                    provider.set_target_state(state)

def main():
    print("==================================================")
    print("  12 軸雙向姿態 FSM 控制器 (標準封裝介面重構版)")
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
            vm.max_speed_rads, vm.max_torque_nm, vm.max_pos_error = 8.0, 18.0, 1.2

            provider = QuadrupedFSMProvider(
                channel=ch, motor_id=m_id,
                kp_soft=35.0, kp_hard=55.0, kd=2.5, tau_max=3.0
            )
            
            # 使用規範介面注入 Provider
            vm.set_provider_instance(provider)
            manager.add_motor(key=vm_key, vm=vm, channel=ch, motor_id=m_id)

    real_angles = query_real_angles(buses, controllers, workers)
    has_error = False
    for key, deg in real_angles.items():
        if deg is None or abs(deg) < 0.01:
            print(f"  [{key:10s}] \033[91m[ERROR] 角度無效 ({deg})\033[0m")
            has_error = True
    if has_error:
        print("\n\033[91m[安全鎖定] 馬達角度診斷異常，中斷啟動！\033[0m")
        for w in workers.values(): w.stop()
        for b in buses.values(): b.shutdown()
        return

    print("\n按 [Enter] 啟動馬達使能，進入 FSM 互動控制模式...")
    input()
    manager.start_all()

    current_state_name = "PRONE"
    print("\n" + "="*50)
    print(" 控制說明: [P] 趴姿 | [C] 低蹲 | [S] 站立 | [Q] 退出")
    print("="*50 + "\n")

    try:
        with NonBlockingKeyboard() as kb:
            start_t = time.time()
            while True:
                elapsed = time.time() - start_t
                key = kb.get_key()
                
                if key:
                    key_lower = key.lower()
                    if key_lower == 'p':
                        set_robot_posture(manager, PostureState.PRONE)
                        current_state_name = "PRONE"
                    elif key_lower == 'c':
                        set_robot_posture(manager, PostureState.CROUCH)
                        current_state_name = "CROUCH"
                    elif key_lower == 's':
                        set_robot_posture(manager, PostureState.STAND)
                        current_state_name = "STAND"
                    elif key_lower == 'q':
                        print("\n[退出] 接收到退出指令 Q，安全卸載歸位...")
                        break

                m_info = []
                for ch in CAN_CHANNELS:
                    vm = manager.get_vm(f"{ch}_ID3")
                    if vm:
                        s = vm.get_ui_snapshot()
                        m_info.append(f"{ch}_Knee({s.position_deg:5.1f}°, {s.torque_act:4.1f}N)")

                print(f"\r[TARGET: {current_state_name:6s}] " + " | ".join(m_info), end="", flush=True)
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n[USER INTERRUPT] Ctrl+C 觸發！")
    finally:
        print("\n[安全關閉] 正在將姿態平滑歸位趴下...")
        set_robot_posture(manager, PostureState.PRONE)
        time.sleep(2.5)
        
        manager.stop_all()
        for w in workers.values(): w.stop()
        for b in buses.values(): b.shutdown()
        print("[STOP] 馬達已全數關閉，CAN Bus 已釋放。")

if __name__ == "__main__":
    main()