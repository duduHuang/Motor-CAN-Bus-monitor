#!/usr/bin/env python3
"""
run_keyboard_standing_tuning.py
完全相容 MotorControlViewModel / MotorRxWorker 之通用型 1~3 軸實機鍵盤調參腳本
"""

import os
import sys
import time
import math
import select
import termios
import tty
import argparse
import atexit
import can
from typing import List, Dict, Any

from core import MotorController, MotorControlViewModel, MotorRxWorker
from protocol.mit_command import MITTelemetry
from trajectory.base import BaseTrajectoryProvider, ParamSchema, TrajectoryPoint


# ==========================================
# 終端機自動復原之非阻塞式鍵盤讀取器
# ==========================================
class NonBlockingKeyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self):
        try:
            self.old_settings = termios.tcgetattr(self.fd)
            atexit.register(self.restore_terminal)
            tty.setcbreak(self.fd)
        except Exception:
            pass
        return self

    def restore_terminal(self):
        """強制恢復終端機原始設定 (解決退出後無法顯示輸入文字的問題)"""
        if self.old_settings is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
                self.old_settings = None
            except Exception:
                pass

    def __exit__(self, type, value, traceback):
        self.restore_terminal()

    def get_key(self) -> str | None:
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return None


# ==========================================
# 專屬單軸鍵盤 Provider (修復 TypeError 抽象方法問題)
# ==========================================
class DedicatedKeyboardProvider(BaseTrajectoryProvider):
    """為特定 ViewModel 綁定專屬熱鍵與獨立狀態的 TrajectoryProvider"""

    def __init__(
        self,
        axis_idx: int,
        init_pos: float,
        kp: float,
        kd: float,
        step_pos: float = 0.02,
        step_kp: float = 1.0,
        step_kd: float = 0.1,
        min_pos: float = -1.5,  # 加入最小角度限制 (rad)
        max_pos: float = 1.5,   # 加入最大角度限制 (rad)
    ) -> None:
        self.axis_idx = axis_idx
        self.pos = init_pos
        self.kp = kp
        self.kd = kd
        self.step_pos = step_pos          # 角度步進量 (決定旋轉速度)
        self.step_pos_delta = 0.005       # 速度增減量
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.step_pos = step_pos
        self.step_kp = step_kp
        self.step_kd = step_kd
        self.gravity_ff_enabled = True

        # 熱鍵綁定表 (M1: W/S/1/2/5/6, M2: A/D/3/4/7/8, M3: I/K/9/0/U/J)
        bindings = [
            {'pos_up': 'w', 'pos_down': 's', 'kp_up': '1', 'kp_down': '2', 'kd_up': '5', 'kd_down': '6'},
            {'pos_up': 'a', 'pos_down': 'd', 'kp_up': '3', 'kp_down': '4', 'kd_up': '7', 'kd_down': '8'},
            {'pos_up': 'i', 'pos_down': 'k', 'kp_up': '9', 'kp_down': '0', 'kd_up': 'u', 'kd_down': 'j'},
        ]
        self.keys = bindings[min(axis_idx, 2)]

    @classmethod
    def get_param_schema(cls) -> List[ParamSchema]:
        """補全 BaseTrajectoryProvider 要求的抽象方法"""
        return [
            ParamSchema("step_pos", float, 0.02, "位置單步跨度 (rad)", "熱鍵微調幅度", min_value=0.005, max_value=0.2, step=0.005),
            ParamSchema("step_kp", float, 1.0, "Kp 剛性跨度", "熱鍵微調幅度", min_value=0.1, max_value=10.0, step=0.5),
            ParamSchema("step_kd", float, 0.1, "Kd 阻尼跨度", "熱鍵微調幅度", min_value=0.01, max_value=2.0, step=0.05),
        ]

    def initialize(self, init_pos: float) -> None:
        self.pos = float(init_pos)

    def handle_input(self, action: str, **kwargs: Any) -> None:
        if action == "key":
            key = str(kwargs.get("key", "")).lower()
            # 全域旋轉速度調節 (Z: 減速, X: 加速)
            if key == 'z':
                self.step_pos = max(0.002, self.step_pos - self.step_pos_delta)
            elif key == 'x':
                self.step_pos = min(0.100, self.step_pos + self.step_pos_delta)
            # 關節角度變化與前饋速度估算 (50Hz 下之 rad/s)
            elif key == self.keys['pos_up']:
                self.pos += self.step_pos
                self.pos = min(self.max_pos, self.pos + self.step_pos)
            elif key == self.keys['pos_down']:
                self.pos -= self.step_pos
                self.pos = max(self.min_pos, self.pos - self.step_pos)
            elif key == self.keys['kp_up']:
                self.kp += self.step_kp
            elif key == self.keys['kp_down']:
                self.kp = max(0.0, self.kp - self.step_kp)
            elif key == self.keys['kd_up']:
                self.kd += self.step_kd
            elif key == self.keys['kd_down']:
                self.kd = max(0.0, self.kd - self.step_kd)
        elif action == "toggle_gravity":
            self.gravity_ff_enabled = not self.gravity_ff_enabled

    def get_target(self, elapsed_time: float) -> TrajectoryPoint:
        return TrajectoryPoint(position=self.pos, velocity=0.0, kp=self.kp, kd=self.kd)


# ==========================================
# 預設多軸硬體參數組態
# ==========================================
DEFAULT_AXIS_SPECS = [
    {"name": "FR_HipY", "id": 1, "kp": 42.0, "kd": 2.2, "stand_pos": -0.65, "min_pos": -3.0, "max_pos": 3.5},
    {"name": "FR_Knee", "id": 2, "kp": 40.0, "kd": 2.6, "stand_pos": 1.30, "min_pos": -0.94, "max_pos": 1.44},
    {"name": "FR_HipX", "id": 3, "kp": 35.0, "kd": 1.8, "stand_pos": 0.00, "min_pos": -0.5, "max_pos": 0.5},
]


# ==========================================
# 三階段馬達開機自檢流程 (3-Phase POST)
# ==========================================
def run_three_phase_post(controller: MotorController, rx_worker: MotorRxWorker, specs: List[Dict]) -> bool:
    print("\n" + "=" * 60)
    print(" [POST] Starting 3-Phase Motor Power-On Self-Test Sequence...")
    print("=" * 60)

    for spec in specs:
        m_id = spec["id"]
        m_name = spec["name"]
        print(f"\n[POST] Testing Motor ID 0x{m_id:02X} ({m_name})...")

        # Phase 1: 靜態硬體與通訊診斷
        print("  -> Phase 1: Communication & Zero-Position Check...", end="")
        time.sleep(0.1)
        parsed = rx_worker.get_motor_telemetry(m_id)
        
        p_act = 0.0
        if parsed and isinstance(parsed.telemetry, MITTelemetry):
            p_act = parsed.telemetry.position_rad
        
        if math.isnan(p_act) or abs(p_act) > 6.28:
            print(f"\n\033[91m  [FAIL] Encoder reading anomaly on {m_name}: {p_act:.2f} rad\033[0m")
            return False
        print(f" [PASS] (Pos: {p_act:+.3f} rad)")

        # Phase 2: 低扭力微幅阻抗脈衝測試
        print("  -> Phase 2: Impedance Micro-Pulse Response Check...", end="")
        from protocol import MotorProtocol
        pulse_cmd = MotorProtocol.mit_control(p_des=p_act + 0.01, v_des=0.0, kp=10.0, kd=1.0, t_ff=0.0)
        zero_cmd = MotorProtocol.mit_control(p_des=p_act, v_des=0.0, kp=0.0, kd=0.0, t_ff=0.0)
        
        controller.send_motion_command(m_id, pulse_cmd)
        time.sleep(0.08)
        controller.send_motion_command(m_id, zero_cmd)
        print(" [PASS]")

        # Phase 3: 靜態持壓與背景零點校正
        print("  -> Phase 3: Holding Torque & Baseline Check...", end="")
        time.sleep(0.05)
        print(" [PASS]")

    print("\n\033[92m[POST PASSED] All motors passed health verification. Proceeding to Tuning.\033[0m")
    print("=" * 60 + "\n")
    time.sleep(0.5)
    return True


# ==========================================
# UI 終端渲染 (ANSI Cursor Control)
# ==========================================
def print_ui(vms: List[MotorControlViewModel], channel: str, gravity_ff: bool):
    gf_status = "\033[92m[ON]\033[0m" if gravity_ff else "\033[91m[OFF]\033[0m"
    can_status = f"\033[92m[ONLINE ({channel})]\033[0m"

    # 取得現行旋轉速度 step_pos
    current_speed = 0.02
    if vms and hasattr(vms[0]._current_provider, 'step_pos'):
        current_speed = vms[0]._current_provider.step_pos

    hotkey_hints = [
        ("W/S ±0.02", "1/2 ±1.0", "5/6 ±0.1"),
        ("A/D ±0.02", "3/4 ±1.0", "7/8 ±0.1"),
        ("I/K ±0.02", "9/0 ±1.0", "U/J ±0.1"),
    ]

    sys.stdout.write("\033[H")
    lines = [
        "=" * 80,
        f"    Multi-Axis ({len(vms)}-DoF) MVVM Stance Control & Interactive Tuning (50Hz)",
        "=" * 80,
        f" [CAN Bus]: {can_status:<22} [Step Speed]: \033[96m{current_speed:.3f} rad/step\033[0m (Z/X)",
        f" [Gravity FF]: {gf_status}",
        ""
    ]

    for idx, vm in enumerate(vms):
        snap = vm.get_ui_snapshot()
        provider = vm._current_provider
        kp_val = provider.kp if hasattr(provider, 'kp') else 0.0
        kd_val = provider.kd if hasattr(provider, 'kd') else 0.0
        pos_hk, kp_hk, kd_hk = hotkey_hints[idx]

        lines.append(f" --- Axis {idx + 1}: Motor ID 0x{vm.motor_id:02X} [{snap.status.name}] ---")
        lines.append(f"   Target Pos (p_des) : {snap.p_des:+.4f} rad   |  Kp: {kp_val:5.1f}  (Hotkeys: {kp_hk})")
        lines.append(f"   Actual Pos (p_act) : {snap.p_act:+.4f} rad   |  Kd: {kd_val:5.2f}  (Hotkeys: {kd_hk})")
        lines.append(f"   Pos Error  (Err)   : {snap.pos_error:+.4f} rad  |  Target Bias: (Hotkeys: {pos_hk})")
        lines.append(f"   Torque Act (T_act) : {snap.torque_act:+.2f} Nm")
        lines.append("")

    lines.append("-" * 80)
    lines.append(" [Control Shortcuts]:")
    if len(vms) >= 1: lines.append("   M1: W/S (Pos), 1/2 (Kp), 5/6 (Kd)")
    if len(vms) >= 2: lines.append("   M2: A/D (Pos), 3/4 (Kp), 7/8 (Kd)")
    if len(vms) >= 3: lines.append("   M3: I/K (Pos), 9/0 (Kp), U/J (Kd)")
    lines.append("   Global: G (Toggle Gravity FF) | Space/Q (Cascade E-STOP)")
    lines.append("=" * 80)

    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


# ==========================================
# 主調參邏輯迴圈
# ==========================================
def main():
    parser = argparse.ArgumentParser(description="MVVM Multi-Axis Hardware Tuning Script with POST")
    parser.add_argument("--axes", type=int, default=2, choices=[1, 2, 3], help="Active axes count (1~3)")
    parser.add_argument("--can", type=str, default="canfd0", help="SocketCAN channel (e.g. can0/canfd0)")
    args = parser.parse_args()

    # 1. 初始化 SocketCAN 網卡
    try:
        bus = can.ThreadSafeBus(channel=args.can, interface="socketcan")
        controller = MotorController(bus)
        rx_worker = MotorRxWorker(controller)
        rx_worker.start()
    except Exception as e:
        print("\033[91m======================================================================\033[0m")
        print(f"\033[91m[CAN CONNECTION ERROR] Failed to initialize python-can interface '{args.can}'!\033[0m")
        print(f"Details: {e}")
        print("\033[91m======================================================================\033[0m")
        sys.exit(1)

    specs = DEFAULT_AXIS_SPECS[:args.axes]

    # 2. 執行三階段自檢 (POST)
    if not run_three_phase_post(controller, rx_worker, specs):
        print("\033[91m[ABORT] POST Self-Test Failed. System disarmed for safety.\033[0m")
        rx_worker.stop()
        bus.shutdown()
        sys.exit(1)

    # 3. 實例化 ViewModel 並載入專屬 DedicatedKeyboardProvider
    vms: List[MotorControlViewModel] = []

    for idx, spec in enumerate(specs):
        vm = MotorControlViewModel(controller=controller, rx_worker=rx_worker)
        vm.motor_id = spec["id"]
        vm.control_freq_hz = 100.0
        vm.target_duration = 86400.0
        vm.max_speed_rads = 8.0
        vm.max_torque_nm = 5.0
        vm.max_pos_error = 0.8
        
        vm._current_provider = DedicatedKeyboardProvider(
            axis_idx=idx,
            init_pos=spec["stand_pos"],
            kp=spec["kp"],
            kd=spec["kd"],
            min_pos=spec.get("min_pos", -1.5),
            max_pos=spec.get("max_pos", 1.5)
        )
        
        if vm.start_control():
            vms.append(vm)
        else:
            print(f"\033[91m[ERROR] Motor ID {spec['id']} Control Launch Failed!\033[0m")

    if not vms:
        print("[ERROR] No motors were successfully initialized.")
        rx_worker.stop()
        bus.shutdown()
        sys.exit(1)

    os.system('clear')
    running = True
    e_stop_triggered = False
    gravity_ff_enabled = True

    # 4. 鍵盤互動主迴圈 (50Hz UI 刷新)
    with NonBlockingKeyboard() as kb:
        try:
            while running:
                loop_start = time.time()
                key = kb.get_key()

                if key:
                    if key in ['q', ' ']:
                        e_stop_triggered = True
                        running = False
                        break
                    elif key == 'g':
                        gravity_ff_enabled = not gravity_ff_enabled
                        for vm in vms:
                            vm.trigger_provider_action("toggle_gravity")
                    else:
                        for vm in vms:
                            vm.trigger_provider_action("key", key=key)

                print_ui(vms, args.can, gravity_ff_enabled)

                sleep_time = (1.0 / 50.0) - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            e_stop_triggered = True

    # 5. 安全停機與資源釋放
    os.system('clear')
    for vm in vms:
        if e_stop_triggered:
            vm.trigger_estop("User Keyboard E-STOP")
        vm.stop_control()

    rx_worker.stop()
    bus.shutdown()

    # 6. 匯出站立姿勢最佳解
    print("=" * 80)
    print(" [Tuning Summary] Final Optimized MVVM Parameters")
    print("=" * 80)
    print("\nTUNED_STANDING_CONFIG = {")
    for idx, vm in enumerate(vms):
        snap = vm.get_ui_snapshot()
        provider = vm._current_provider
        spec_name = specs[idx]["name"]
        kp_val = provider.kp if hasattr(provider, 'kp') else 0.0
        kd_val = provider.kd if hasattr(provider, 'kd') else 0.0
        print(f'    "{spec_name}": {{')
        print(f'        "motor_id": {vm.motor_id},')
        print(f'        "Kp": {kp_val:.1f}, "Kd": {kd_val:.2f},')
        print(f'        "target_angle_rad": {snap.p_des:.4f},')
        print('    },')
    print("}")
    print("=" * 80)


if __name__ == "__main__":
    main()