#core/motor_control_viewmodel.py
"""
MotorControlViewModel Module
針對 DearPyGui 與 CAN Bus MIT 控制模式設計的 ViewModel 層。
具備真實 Model 層對接、Thread-Safe 狀態保護、安全臨界值檢查與觀察者通知機制。
重構重點：
1. 封裝性改善：提供公共 Provider 注入與取得 API。
2. 控制迴圈高精度自適應時間補償 (monotonic)，避免 Jitter 與滯後累積。
3. CAN 日誌 Lazy Formatting 轉換。
"""

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import csv
import math
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from .rx_worker import MotorRxWorker
from .motor_controller import MotorController
from protocol import MotorProtocol
from protocol.mit_command import MITTelemetry
from trajectory import BaseTrajectoryProvider, ProviderFactory, TrajectoryPoint


class SystemStatus(Enum):
    IDLE = "IDLE"
    PROBING = "PROBING"
    RUNNING = "RUNNING"
    E_STOPPED = "E_STOPPED"


@dataclass
class TelemetryData:
    """即時馬達回傳數據快照"""
    p_act: float = 0.0
    v_act: float = 0.0
    torque_act: float = 0.0
    last_update_time: float = 0.0


@dataclass
class ControlSnapshot:
    """UI 渲染專用的 Thread-Safe 唯讀數據快照"""
    timestamp: float = 0.0
    status: SystemStatus = SystemStatus.IDLE
    p_des: float = 0.0
    p_act: float = 0.0
    v_des: float = 0.0
    v_act: float = 0.0
    t_ff: float = 0.0
    torque_act: float = 0.0
    pos_error: float = 0.0
    error_msg: str = ""
    tx_count: int = 0
    rx_count: int = 0
    loss_rate: float = 0.0
    actual_hz: float = 0.0
    is_timeout: bool = False
    # === 核心資料結構擴充 (UI 綁定用) ===
    position_deg: float = 0.0                         # 即時角度 °
    speed_dps: float = 0.0                            # 即時角速度 dps
    current_a: float = 0.0                            # 即時電流 A
    temperature: float = 0.0                          # 馬達當前溫度 °C
    voltage: float = 0.0                              # 系統供電電壓 V
    raw_can_logs: list = field(default_factory=list)  # 用於儲存帶有時間戳記的原始 CAN 封包字串清單

class MotorControlViewModel:
    """
    馬達控制 ViewModel, 負責連接 Model 層 (MotorController, MotorRxWorker, TrajectoryProvider),
    並封裝背景控制迴圈、安全保護與 Thread-Safe 數據導出。
    """

    def __init__(
        self,
        controller: Optional[MotorController] = None,
        rx_worker: Optional[MotorRxWorker] = None,
    ):
        self._controller = controller
        self._rx_worker = rx_worker

        self.max_history_samples: int = 120000
        self.control_freq_hz: float = 100.0
        self.target_duration: float = 10.0
        self.default_kp: float = 10.0
        self.default_kd: float = 1.0

        self.max_speed_rads: float = 8.0
        self.max_torque_nm: float = 5.0
        self.max_pos_error: float = 0.8
        self.telemetry_timeout_s: float = 0.3
        self.control_start_delay: float = 0.2

        self._overspeed_counter: int = 0
        self._pos_error_counter: int = 0
        self._overload_counter: int = 0
        self.debounce_threshold: int = 3
        self.outlier_speed_threshold: float = 30.0
        self.outlier_pos_error_threshold: float = 3.0

        # 私有成員 Provider
        self._current_provider: Optional[BaseTrajectoryProvider] = None

        self._status = SystemStatus.IDLE
        self._status_lock = threading.Lock()

        self._control_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._estop_event = threading.Event()
        self._estop_executed: bool = False

        self._telemetry = TelemetryData(last_update_time = time.monotonic())
        self._telemetry_lock = threading.Lock()

        self._snapshot = ControlSnapshot()
        self._snapshot_lock = threading.Lock()

        self._history_buffer: List[Tuple[float, float, float, float, float, float]] = []
        self._history_lock = threading.Lock()

        self._motor_id: int = 1
        self.on_status_changed: Optional[Callable[[str], None]] = None
        self.on_estop_triggered: Optional[Callable[[str], None]] = None

    # ------------------------------------------------------------------
    # Provider 介面（修復封包封裝性問題）
    # ------------------------------------------------------------------
    def set_provider_instance(self, provider: BaseTrajectoryProvider) -> None:
        """公共 API：安全注入已實例化的 TrajectoryProvider"""
        self._current_provider = provider

    def get_available_providers(self) -> List[str]:
        return list(ProviderFactory.list_providers().keys())

    def select_provider(self, name: str, **kwargs) -> None:
        self._current_provider = ProviderFactory.create_provider(name, **kwargs)

    def get_current_provider(self) -> Optional[BaseTrajectoryProvider]:
        return self._current_provider

    def trigger_provider_action(self, action: str, **kwargs) -> None:
        if "duration" in kwargs:
            try:
                self.target_duration = float(kwargs["duration"])
            except (ValueError, TypeError):
                pass
        if self._current_provider:
            self._current_provider.handle_input(action, **kwargs)

    def clear_history(self) -> None:
        with self._history_lock:
            self._history_buffer.clear()
        with self._telemetry_lock:
            self._telemetry = TelemetryData(last_update_time=time.monotonic())
        with self._snapshot_lock:
            self._snapshot = ControlSnapshot(status=self._status)

    @property
    def motor_id(self) -> int:
        return self._motor_id

    @motor_id.setter
    def motor_id(self, new_id: int) -> None:
        if type(new_id) is not int or new_id <= 0:
            return

        with self._status_lock:
            if self._status in [SystemStatus.RUNNING, SystemStatus.PROBING]:
                self._notify_status_change("[警告] 控制運行中，禁止變更 Motor ID!")
                return

        if self._motor_id != new_id:
            self._motor_id = new_id
            self.clear_history()
            self._notify_status_change(f"Motor ID 已變更為 {new_id}，請重新按下 START 對齊探測")

    def start_control(self) -> bool:
        self._overspeed_counter = 0
        self._pos_error_counter = 0
        self._overload_counter = 0
        with self._status_lock:
            if self._status in [SystemStatus.RUNNING, SystemStatus.PROBING]:
                return False
            if self._current_provider is None:
                raise RuntimeError("未選擇 TrajectoryProvider, 無法啟動控制!")

            if hasattr(self._current_provider, "duration"):
                self.target_duration = float(getattr(self._current_provider, "duration", 0.0))

            self._status = SystemStatus.PROBING
            self._stop_event.clear()
            self._estop_event.clear()
            self._estop_executed = False

            with self._history_lock:
                self._history_buffer.clear()

            self._control_thread = threading.Thread(
                target = self._control_loop,
                daemon = True,
                name = "MotorControlThread"
            )
            self._control_thread.start()
            return True

    def _disable_active_responses(self):
        if self._controller:
            try:
                cmd_9c_off = MotorProtocol.set_active_response(0x9C, False, 0)
                cmd_9a_off = MotorProtocol.set_active_response(0x9A, False, 0)
                self._controller.send_single_command(self.motor_id, cmd_9c_off)
                self._controller.send_single_command(self.motor_id, cmd_9a_off)
            except Exception:
                pass

    def stop_control(self) -> None:
        self._stop_event.set()
        self._disable_active_responses()
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout = 1.0)

        with self._status_lock:
            if self._status != SystemStatus.E_STOPPED:
                self._status = SystemStatus.IDLE
                self._notify_status_change("控制已停止 - IDLE")

    def trigger_estop(self, reason: str) -> None:
        with self._status_lock:
            if self._estop_executed:
                return
            self._estop_executed = True
            self._status = SystemStatus.E_STOPPED

        self._estop_event.set()
        self._stop_event.set()

        if self._controller:
            self._disable_active_responses()
            shutdown_cmd = MotorProtocol.motor_shutdown()
            mit_zero = MotorProtocol.mit_control(0.0, 0.0, 0.0, 0.0, 0.0)

            for _ in range(5):
                try:
                    self._controller.send_single_command(self.motor_id, shutdown_cmd)
                except Exception:
                    pass
                time.sleep(0.005)

            for _ in range(5):
                try:
                    self._controller.send_motion_command(self.motor_id, mit_zero)
                except Exception:
                    pass
                time.sleep(0.005)

        if self.on_estop_triggered:
            self.on_estop_triggered(reason)
        self._notify_status_change(f"E-STOP 觸發: {reason}")

    # ------------------------------------------------------------------
    # 重構：高精度即時控制迴圈 (自適應 monotonic 時間補償)
    # ------------------------------------------------------------------
    def _control_loop(self) -> None:
        self._notify_status_change("開始探測連線 (PROBING)...")
        probe_start_time = time.monotonic()
        init_pos = 0.0
        probe_success = False

        probe_cmd = MotorProtocol.mit_control(0.0, 0.0, 0.0, 0.0, 0.0)

        while time.monotonic() - probe_start_time < 2.0:
            if self._stop_event.is_set():
                return

            if self._controller:
                try:
                    self._controller.send_motion_command(self.motor_id, probe_cmd)
                except Exception:
                    pass

            time.sleep(0.05)

            telemetry = self._fetch_rx_telemetry()
            if telemetry:
                init_pos = telemetry.p_act
                probe_success = True
                break

        if not probe_success and not self._controller:
            init_pos = 0.0
            probe_success = True

        if not probe_success:
            self.trigger_estop("Probe Timeout: Unable to read motor feedback.")
            return

        if hasattr(self._current_provider, "initialize"):
            self._current_provider.initialize(init_pos)

        self._notify_status_change(f"取得初始角度: {init_pos:.3f} rad | 啟動中...")

        if self._controller:
            try:
                cmd_9c = MotorProtocol.set_active_response(0x9C, True, 1)
                self._controller.send_single_command(self.motor_id, cmd_9c)
                cmd_9a = MotorProtocol.set_active_response(0x9A, True, 10)
                self._controller.send_single_command(self.motor_id, cmd_9a)
            except Exception as e:
                print(f"啟動主動回報失敗: {e}")

        with self._status_lock:
            if self._status != SystemStatus.E_STOPPED:
                self._status = SystemStatus.RUNNING

        tx_count = 0
        rx_count = 0
        last_rx_update_time = 0.0
        
        hz_calc_time = time.monotonic()
        hz_loop_count = 0
        actual_hz = 0.0

        # === 高精度定時與自適應補償變數 ===
        period = 1.0 / self.control_freq_hz
        start_time = time.monotonic()
        next_time = start_time

        while not self._stop_event.is_set():
            now = time.monotonic()
            elapsed_time = now - start_time
            hz_loop_count += 1
            
            if now - hz_calc_time >= 0.5:
                actual_hz = hz_loop_count / (now - hz_calc_time)
                hz_calc_time = now
                hz_loop_count = 0

            if self.target_duration > 0 and elapsed_time >= self.target_duration:
                break

            target: TrajectoryPoint = self._current_provider.get_target(elapsed_time)
            p_des = target.position
            v_des = target.velocity
            t_ff = target.torque_ff
            kp = target.kp if target.kp > 0 else self.default_kp
            kd = target.kd if target.kd > 0 else self.default_kd

            rx_data = self._fetch_rx_telemetry()
            is_timeout = False
            if not rx_data:
                with self._telemetry_lock:
                    self._telemetry.p_act += (p_des - self._telemetry.p_act) * 0.1
                    self._telemetry.v_act = v_des
                    self._telemetry.torque_act = (p_des - self._telemetry.p_act) * kp
                    self._telemetry.last_update_time = now
                    rx_data = TelemetryData(
                        p_act = self._telemetry.p_act,
                        v_act = self._telemetry.v_act,
                        torque_act = self._telemetry.torque_act,
                        last_update_time = self._telemetry.last_update_time
                    )
            else:
                if rx_data.last_update_time > last_rx_update_time:
                    rx_count += 1
                    last_rx_update_time = rx_data.last_update_time
                is_timeout = (now - rx_data.last_update_time) > self.telemetry_timeout_s

            if not self._validate_safety(rx_data, now, elapsed_time, p_des):
                is_timeout = is_timeout or ((now - rx_data.last_update_time) > self.telemetry_timeout_s)

            if self._controller:
                try:
                    payload = MotorProtocol.mit_control(p_des, v_des, kp, kd, t_ff)
                    self._controller.send_motion_command(self.motor_id, payload)
                    tx_count += 1
                except Exception as e:
                    self.trigger_estop(f"CAN TX Error: {e}")
                    return

            temp_c, volt_v = 0.0, 24.0
            msg_9a = self._rx_worker.get_specific_telemetry(self.motor_id, "SensorStatus1Telemetry") if self._rx_worker else None
            if msg_9a and msg_9a.telemetry:
                temp_c = float(msg_9a.telemetry.temperature_c)
                volt_v = float(msg_9a.telemetry.voltage_v)

            current_a, speed_dps = 0.0, 0.0
            msg_9c = self._rx_worker.get_specific_telemetry(self.motor_id, "StandardMotionTelemetry") if self._rx_worker else None
            if msg_9c and msg_9c.telemetry:
                current_a = float(msg_9c.telemetry.iq_current_amp)
                speed_dps = float(msg_9c.telemetry.speed_dps)

            # 延遲格式化 (Lazy Formatting) 導出 RAW CAN Logs 至 UI
            can_logs = []
            if self._controller:
                try:
                    now_wall = time.time()
                    now_mono = time.monotonic()
                    for entry in list(self._controller.can_logger):
                        if isinstance(entry, tuple):
                            ts, direction, cid, payload = entry
                            wall_ts = now_wall - (now_mono - ts)
                            time_str = time.strftime("%H:%M:%S", time.localtime(wall_ts)) + f".{int((wall_ts % 1) * 1000):03d}"
                            hex_payload = ' '.join(f"{b:02X}" for b in payload)
                            can_logs.append(f"[{time_str}] {direction} ID: 0x{cid:03X} DATA: {hex_payload}")
                        else:
                            can_logs.append(str(entry))
                except RuntimeError:
                    pass

            loss_rate = 0.0
            if tx_count > 0:
                loss_rate = max(0.0, 100.0 * (1.0 - (rx_count / tx_count)))
            pos_err = abs(p_des - rx_data.p_act)
            
            snapshot = ControlSnapshot(
                timestamp = elapsed_time,
                status = self._status,
                p_des = p_des,
                p_act = rx_data.p_act,
                v_des = v_des,
                v_act = rx_data.v_act,
                t_ff = t_ff,
                torque_act = rx_data.torque_act,
                pos_error = pos_err,
                error_msg = "",
                tx_count = tx_count,
                rx_count = rx_count,
                loss_rate = loss_rate,
                actual_hz = actual_hz,
                is_timeout = is_timeout,
                position_deg = math.degrees(rx_data.p_act),
                speed_dps = speed_dps if speed_dps != 0.0 else math.degrees(rx_data.v_act),
                current_a = current_a,
                temperature = temp_c,
                voltage = volt_v,
                raw_can_logs = can_logs,
            )

            with self._snapshot_lock:
                self._snapshot = snapshot

            with self._history_lock:
                self._history_buffer.append((
                    elapsed_time, p_des, rx_data.p_act, v_des, rx_data.v_act, rx_data.torque_act
                ))

            if self._status == SystemStatus.E_STOPPED:
                return

            # === 自適應時間補償與延遲過度重置 ===
            next_time += period
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 發生週期 Overrun 時重置 next_time，防止累積 phase-lag 與追時間產生的控制暴衝
                next_time = time.monotonic()

        with self._status_lock:
            if self._status != SystemStatus.E_STOPPED:
                self._status = SystemStatus.IDLE
                self._notify_status_change("控制任務順利完成")

    def _fetch_rx_telemetry(self) -> Optional[TelemetryData]:
        if not self._rx_worker:
            return None

        parsed = self._rx_worker.get_motor_telemetry(self.motor_id)
        if parsed and isinstance(parsed.telemetry, MITTelemetry):
            t: MITTelemetry = parsed.telemetry
            now = parsed.timestamp if parsed.timestamp > 0 else time.monotonic()
            with self._telemetry_lock:
                self._telemetry = TelemetryData(
                    p_act = t.position_rad,
                    v_act = t.velocity_rads,
                    torque_act = t.torque_nm,
                    last_update_time = now
                )
                return self._telemetry
        return None

    def _validate_safety(self, rx: TelemetryData, now: float, elapsed_time: float, p_des: float) -> bool:
        if (now - rx.last_update_time) > self.telemetry_timeout_s:
            self.trigger_estop(f"Communication Timeout (> {self.telemetry_timeout_s}s)")
            return False

        v_act_abs = abs(rx.v_act)
        if v_act_abs > self.outlier_speed_threshold:
            print(
                f"\n\033[93m[CAN Noise Filtered] Motor ID {self.motor_id:2d} | "
                f"Speed Outlier Spike: {rx.v_act:6.2f} rad/s > {self.outlier_speed_threshold:.1f} rad/s (Ignored)\033[0m"
            )
        elif v_act_abs > self.max_speed_rads:
            self._overspeed_counter += 1
            if self._overspeed_counter >= self.debounce_threshold:
                self.trigger_estop(f"[Overspeed Protection] {rx.v_act:.2f} rad/s > {self.max_speed_rads}")
                return False
        else:
            self._overspeed_counter = 0

        if abs(rx.torque_act) > self.max_torque_nm:
            self._overload_counter += 1
            if self._overload_counter >= self.debounce_threshold:
                self.trigger_estop(f"[Overload Protection] {rx.torque_act:.2f} Nm > {self.max_torque_nm}")
                return False
        else:
            self._overload_counter = 0

        if elapsed_time > self.control_start_delay:
            pos_err = abs(p_des - rx.p_act)
            if pos_err > self.outlier_pos_error_threshold:
                print(
                    f"\n\033[93m[CAN Noise Filtered] Motor ID {self.motor_id:2d} | "
                    f"Pos Error Outlier Spike: {pos_err:6.3f} rad > {self.outlier_pos_error_threshold:.1f} rad (Ignored)\033[0m"
                )
            elif pos_err > self.max_pos_error:
                self._pos_error_counter += 1
                if self._pos_error_counter >= self.debounce_threshold:
                    self.trigger_estop(f"[Position Error Protection] Following Error {pos_err:.3f} rad > {self.max_pos_error}")
                    return False
            else:
                self._pos_error_counter = 0

        return True

    def get_ui_snapshot(self) -> ControlSnapshot:
        with self._snapshot_lock:
            snap = self._snapshot
        with self._status_lock:
            snap.status = self._status
        return snap

    def get_plot_data_history(self) -> Tuple[List[float], List[float], List[float], List[float], List[float]]:
        with self._history_lock:
            if not self._history_buffer:
                return [], [], [], [], []

            times = [item[0] for item in self._history_buffer]
            p_des = [item[1] for item in self._history_buffer]
            p_act = [item[2] for item in self._history_buffer]
            v_act = [item[4] for item in self._history_buffer]
            torque = [item[5] for item in self._history_buffer]
            return times, p_des, p_act, v_act, torque

    def generate_quality_report(self) -> Dict[str, float]:
        with self._history_lock:
            if not self._history_buffer:
                return {"rms_error": 0.0, "peak_speed": 0.0, "peak_torque": 0.0}

            sq_errors = [(item[1] - item[2]) ** 2 for item in self._history_buffer]
            v_acts = [abs(item[4]) for item in self._history_buffer]
            torques = [abs(item[5]) for item in self._history_buffer]

            rms_err = math.sqrt(sum(sq_errors) / len(sq_errors))
            peak_speed = max(v_acts)
            peak_torque = max(torques)

            return {
                "rms_error": rms_err,
                "peak_speed": peak_speed,
                "peak_torque": peak_torque,
            }

    def save_csv_log(self, filepath: str) -> None:
        with self._history_lock:
            data = list(self._history_buffer)

        with open(filepath, mode = "w", newline = "", encoding = "utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "p_des", "p_act", "v_des", "v_act", "torque_act"])
            for row in data:
                writer.writerow(row)

    def _notify_status_change(self, msg: str) -> None:
        if self.on_status_changed:
            self.on_status_changed(msg)