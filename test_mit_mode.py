#!/usr/bin/env python3
"""
test_mit_mode.py
MIT 模式測試程式，使用者可透過此程式對馬達進行 MIT 模式的控制測試，並在測試過程中進行安全監控與資料紀錄。
"""
import csv
import math
import time
import threading
import can
from core import MotorController
from core import MotorRxWorker
from protocol import MotorProtocol
from protocol.mit_command import MITTelemetry
from trajectory import BaseTrajectoryProvider
from trajectory.sin_provider import SineTrajectoryProvider

# ==================== 測試與安全參數配置 ====================
TARGET_MOTOR_ID = 1
CONTROL_FREQ_HZ = 100.0      # 目標控制週期 100 Hz (10 ms)
TEST_DURATION_SEC = 10.0     # 測試時間 (秒)
CONTROL_START_DELAY = 0.2    # 啟動保護延遲 (200ms)

# 安全保護閾值
MAX_SPEED_RADS = 8.0         # 最大轉速 (rad/s)
MAX_TORQUE_NM = 5.0          # 最大力矩 (Nm)
TELEMETRY_TIMEOUT = 0.3      # 通訊超時 (秒)

# 階段性參數與動態跟隨誤差閾值
CURRENT_STAGE = 3
STAGE_PROFILES = {
    1: {"kp": 5.0,  "kd": 0.2, "amp": 0.1, "freq": 0.2, "pos_err": 0.4, "desc": "Phase 1: 慢速柔順，驗證方向與 CAN"},
    2: {"kp": 10.0, "kd": 0.5, "amp": 0.3, "freq": 0.5, "pos_err": 0.6, "desc": "Phase 2: 中剛度，觀察運動跟隨"},
    3: {"kp": 20.0, "kd": 1.0, "amp": 0.5, "freq": 0.5, "pos_err": 0.8, "desc": "Phase 3: 高剛度，接近標準伺服"},
}

# ==================== Thread-Safe 狀態物件 ====================
estop_event = threading.Event()
state_lock = threading.Lock()
log_lock = threading.Lock()    # 資料寫入與讀取隔離鎖

last_telemetry_time = time.monotonic()
control_start_timestamp = 0.0
estop_reason = ""
current_p_des = 0.0
current_v_des = 0.0

estop_executed = False
actual_control_rate = 0.0
data_log_buffer = []

def monitor_safety_callback(parsed):
    """背景 Callback: 執行實時間隔檢查、安全保護與 Thread-Safe 數據快照"""
    global last_telemetry_time, estop_reason

    if not isinstance(parsed.telemetry, MITTelemetry):
        return

    t = parsed.telemetry
    now = time.monotonic()

    with state_lock:
        last_telemetry_time = now
        p_des, v_des = current_p_des, current_v_des
        start_time = control_start_timestamp

    # 安全地將數據寫入共享 Buffer
    with log_lock:
        data_log_buffer.append((now, p_des, t.position_rad, v_des, t.velocity_rads, t.torque_nm))

    # 1. 超速保護
    if abs(t.velocity_rads) > MAX_SPEED_RADS:
        with state_lock:
            estop_reason = f"【過速保護】{t.velocity_rads:.2f} rad/s > {MAX_SPEED_RADS}"
        estop_event.set()

    # 2. 過載保護
    elif abs(t.torque_nm) > MAX_TORQUE_NM:
        with state_lock:
            estop_reason = f"【過載保護】{t.torque_nm:.2f} Nm > {MAX_TORQUE_NM}"
        estop_event.set()

    # 3. 延遲啟動之位置跟隨誤差保護
    elif start_time > 0 and (now - start_time) > CONTROL_START_DELAY:
        max_err = STAGE_PROFILES[CURRENT_STAGE]["pos_err"]
        pos_err = abs(p_des - t.position_rad)
        if pos_err > max_err:
            with state_lock:
                estop_reason = f"【位置誤差保護】跟隨誤差 {pos_err:.3f} rad > {max_err}"
            estop_event.set()

    print(f"\r[MIT] P_des: {p_des:6.3f} | P_act: {t.position_rad:6.3f} | V_act: {t.velocity_rads:6.3f} | T: {t.torque_nm:6.3f}", end = "")

def trigger_emergency_stop(controller: MotorController, motor_id: int, reason: str):
    """高可靠 E-STOP: 防重複執行, 先切斷 PWM, 再下發零力矩"""
    global estop_executed

    with state_lock:
        if estop_executed:
            return
        estop_executed = True

    print(f"\n\n[!!! 緊急停止觸發 !!!] 原因: {reason}")

    shutdown_cmd = MotorProtocol.motor_shutdown()
    for _ in range(10):
        try:
            controller.send_single_command(motor_id, shutdown_cmd)
        except Exception:
            pass
        time.sleep(0.005)

    mit_zero = MotorProtocol.mit_control(
        p_des = 0.0,
        v_des = 0.0,
        kp = 0.0,
        kd = 0.0,
        t_ff = 0.0
    )
    for _ in range(10):
        try:
            controller.send_motion_command(motor_id, mit_zero)
        except Exception:
            pass
        time.sleep(0.005)

def get_log_snapshot():
    """Thread-Safe 獲取 log 資料副本，避免分析與寫入時發生競態"""
    with log_lock:
        return list(data_log_buffer)

def calculate_tracking_metrics():
    """計算並印出控制品質分析報告"""
    logs = get_log_snapshot()
    if not logs:
        print("\n[Report] 無可用數據生成報告。")
        return

    pos_errors = [abs(row[1] - row[2]) for row in logs]
    velocities = [abs(row[4]) for row in logs]
    torques = [abs(row[5]) for row in logs]

    rms_error = math.sqrt(sum(e**2 for e in pos_errors) / len(pos_errors))
    max_error = max(pos_errors)
    peak_vel = max(velocities)
    peak_torque = max(torques)

    print("\n\n" + "=" * 15 + " MIT Test Report " + "=" * 15)
    print(f"Samples             : {len(logs)}")
    print(f"RMS Position Error  : {rms_error:.4f} rad ({math.degrees(rms_error):.2f}°)")
    print(f"MAX Position Error  : {max_error:.4f} rad ({math.degrees(max_error):.2f}°)")
    print(f"MAX Velocity        : {peak_vel:.2f} rad/s")
    print(f"MAX Torque          : {peak_torque:.2f} Nm")
    print(f"Emergency Stop      : {'YES (' + estop_reason + ')' if estop_event.is_set() else 'NO'}")
    print(f"Control Rate        : {actual_control_rate:.1f} Hz (Target: {CONTROL_FREQ_HZ} Hz)")
    print("=" * 47)

def save_csv_log(filename="mit_tracking_log.csv"):
    """匯出測試數據至 CSV 檔"""
    logs = get_log_snapshot()
    if not logs:
        return
    with open(filename, mode = 'w', newline = '') as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "p_des", "p_act", "v_des", "v_act", "torque"])
        writer.writerows(logs)
    print(f"[Data Logger] 已匯出 {len(logs)} 筆即時數據至 {filename}")

def run_mit_framework(provider: BaseTrajectoryProvider):
    global current_p_des, current_v_des, control_start_timestamp, last_telemetry_time, actual_control_rate
    profile = STAGE_PROFILES[CURRENT_STAGE]
    
    print(f"=== {profile['desc']} ===")
    print(f"設定: Kp = {profile['kp']}, Kd = {profile['kd']}, 容許跟隨誤差 = {profile['pos_err']} rad\n")

    bus = can.ThreadSafeBus(channel = 'can0', interface = 'socketcan')
    controller = MotorController(bus)
    worker = MotorRxWorker(controller, on_message_received = monitor_safety_callback)
    worker.start()

    loop_count = 0
    loop_start_time = 0.0

    try:
        probe_cmd = MotorProtocol.mit_control(
            p_des = 0.0,
            v_des = 0.0,
            kp = 0.0,
            kd = 0.0,
            t_ff = 0.0
        )
        init_wait_start = time.monotonic()
        while time.monotonic() - init_wait_start < 2.0:
            try:
                controller.send_motion_command(TARGET_MOTOR_ID, probe_cmd)
            except can.CanError:
                pass
            
            time.sleep(0.05)

            # 2. 檢查背景 Worker 是否已接收並解析到封包
            telemetry = worker.get_motor_telemetry(TARGET_MOTOR_ID)
            if telemetry and isinstance(telemetry.telemetry, MITTelemetry):
                break

        telemetry = worker.get_motor_telemetry(TARGET_MOTOR_ID)
        if not telemetry:
            raise TimeoutError("無法讀取馬達回授，請檢查連線與 ID!")
        
        init_pos = telemetry.telemetry.position_rad
        provider.initialize(init_pos)
        now = time.monotonic()

        with state_lock:
            last_telemetry_time = now
            current_p_des = init_pos
            current_v_des = 0.0

        # 在正式控制迴圈啟動前，清空 Probe 階段的紀錄數據
        with log_lock:
            data_log_buffer.clear()

        start_time = time.monotonic()
        loop_start_time = start_time
        with state_lock:
            control_start_timestamp = start_time

        print(f"取得初始角度: {init_pos:.3f} rad | 保護延遲啟動中 ({CONTROL_START_DELAY}s)...\n")

        # 絕對時間無對齊控制迴圈
        # kp, kd = profile['kp'], profile['kd']
        # amp, freq = profile['amp'], profile['freq']
        period = 1.0 / CONTROL_FREQ_HZ
        next_time = time.monotonic()

        while time.monotonic() - start_time < TEST_DURATION_SEC:
            if estop_event.is_set():
                with state_lock:
                    reason = estop_reason
                trigger_emergency_stop(controller, TARGET_MOTOR_ID, reason)
                break

            with state_lock:
                t_last = last_telemetry_time

            if time.monotonic() - t_last > TELEMETRY_TIMEOUT:
                trigger_emergency_stop(controller, TARGET_MOTOR_ID, f"通訊超時 (> {TELEMETRY_TIMEOUT}s)")
                break

            elapsed = time.monotonic() - start_time
            target = provider.get_target(elapsed)
            
            p_des = target.p_des
            v_des = target.v_des
            t_ff = target.t_ff
            kp = target.kp if target.kp is not None else profile['kp']
            kd = target.kd if target.kd is not None else profile['kd']
            # p_des = init_pos + amp * math.sin(2 * math.pi * freq * elapsed)
            # v_des = amp * 2 * math.pi * freq * math.cos(2 * math.pi * freq * elapsed)

            with state_lock:
                current_p_des = p_des
                current_v_des = v_des

            try:
                payload = MotorProtocol.mit_control(
                    p_des = p_des,
                    v_des = v_des,
                    kp = kp,
                    kd = kd,
                    t_ff = t_ff
                )
                controller.send_motion_command(TARGET_MOTOR_ID, payload)
            except can.CanError as e:
                with state_lock:
                    estop_reason = f"CAN TX Error: {e}"
                estop_event.set()
                trigger_emergency_stop(controller, TARGET_MOTOR_ID, estop_reason)
                break

            loop_count += 1
            
            # 絕對時間補償休眠
            next_time += period
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        trigger_emergency_stop(controller, TARGET_MOTOR_ID, f"系統例外: {e}")

    finally:
        if loop_count > 0 and loop_start_time > 0:
            total_time = time.monotonic() - loop_start_time
            actual_control_rate = loop_count / total_time

        trigger_emergency_stop(controller, TARGET_MOTOR_ID, "測試結束 / 釋放資源")
        provider.stop()
        worker.stop()
        bus.shutdown()
        
        calculate_tracking_metrics()
        save_csv_log()

if __name__ == "__main__":
    # 模式 A: 正弦波測試
    provider = SineTrajectoryProvider(amp = 0.3, freq = 0.5)
    run_mit_framework()