# test/test_headless_suite.py
"""
Headless 自動化測試套件
直接鏈接並驗證真實的 MotorControlViewModel 與 ProviderFactory。
"""

import csv
import os
import time
import unittest
from concurrent.futures import ThreadPoolExecutor, wait

from mock_can_engine import MockMotorCANEngine, MockMotorController, MockMotorRxWorker
from core import MotorControlViewModel, SystemStatus
from trajectory import ProviderFactory


class TestProviderAndFactory(unittest.TestCase):
    """1. Provider 與 Factory 單元測試"""

    def test_schema_export(self) -> None:
        schema = ProviderFactory.get_provider_schema("Sine")
        param_names = [p.name for p in schema]
        self.assertIn("amplitude", param_names)
        self.assertIn("frequency", param_names)

    def test_invalid_kwargs_exception(self) -> None:
        with self.assertRaises(ValueError):
            ProviderFactory.create_provider("Sine", amplitude = 999.0)  # 超出 max_value (10.0)

    def test_provider_creation_and_execution(self) -> None:
        provider = ProviderFactory.create_provider("Sine", amplitude = 1.0, frequency = 1.0)
        target = provider.get_target(elapsed_time = 0.0)
        self.assertAlmostEqual(target.position, 0.0)


class TestViewModelLifecycle(unittest.TestCase):
    """2. ViewModel 狀態機與生命週期測試"""

    def setUp(self) -> None:
        self.engine = MockMotorCANEngine(node_id = 1)
        self.controller = MockMotorController(self.engine)
        self.rx_worker = MockMotorRxWorker(self.engine)

        self.vm = MotorControlViewModel(
            controller = self.controller,
            rx_worker = self.rx_worker
        )
        self.vm.select_provider("Sine", amplitude = 1.0, frequency = 0.5)
        self.vm.target_duration = 1.0  # 測試 1 秒

    def tearDown(self) -> None:
        self.vm.stop_control()

    def test_state_transitions(self) -> None:
        """測試正常的狀態轉換流程: IDLE -> PROBING -> RUNNING -> IDLE"""
        snapshot = self.vm.get_ui_snapshot()
        self.assertEqual(snapshot.status, SystemStatus.IDLE)

        self.vm.start_control()
        time.sleep(0.1)  # 等待進入 RUNNING

        snapshot = self.vm.get_ui_snapshot()
        self.assertEqual(snapshot.status, SystemStatus.RUNNING)

        self.vm.stop_control()
        snapshot = self.vm.get_ui_snapshot()
        self.assertEqual(snapshot.status, SystemStatus.IDLE)

    def test_race_condition_rapid_start_stop(self) -> None:
        """驗證多執行緒快速競爭啟動與停止時不發生 Deadlock"""
        def stress_task():
            for _ in range(10):
                self.vm.start_control()
                time.sleep(0.002)
                self.vm.stop_control()

        with ThreadPoolExecutor(max_workers = 4) as executor:
            futures = [executor.submit(stress_task) for _ in range(4)]
            done, not_done = wait(futures, timeout = 5.0)
            self.assertEqual(len(not_done), 0, "檢測到執行緒死鎖 (Deadlock)!")


class TestSafetyAndEStop(unittest.TestCase):
    """3. 安全保護機制與 E-STOP 觸發邊界測試"""

    def setUp(self) -> None:
        self.engine = MockMotorCANEngine(node_id = 1)
        self.controller = MockMotorController(self.engine)
        self.rx_worker = MockMotorRxWorker(self.engine)

        self.vm = MotorControlViewModel(
            controller = self.controller,
            rx_worker = self.rx_worker
        )
        self.vm.select_provider("Sine", amplitude = 1.0, frequency = 1.0)
        self.vm.target_duration = 2.0
        self.vm.start_control()
        time.sleep(0.1)

    def tearDown(self) -> None:
        self.vm.stop_control()

    def test_overspeed_protection(self) -> None:
        """注入過速 Fault, 驗證 ViewModel 自動觸發 E-STOP"""
        self.engine.fault_overspeed = True
        time.sleep(0.1)  # 等待控制迴圈觸發安全檢查

        snapshot = self.vm.get_ui_snapshot()
        self.assertEqual(snapshot.status, SystemStatus.E_STOPPED)

    def test_overtorque_protection(self) -> None:
        """注入過載 Fault, 驗證 ViewModel 自動觸發 E-STOP"""
        self.engine.fault_overtorque = True
        time.sleep(0.1)

        snapshot = self.vm.get_ui_snapshot()
        self.assertEqual(snapshot.status, SystemStatus.E_STOPPED)

    def test_comm_timeout_protection(self) -> None:
        """模擬通訊斷訊, 驗證 ViewModel 超時保護"""
        self.engine.simulate_comm_loss = True
        time.sleep(self.vm.telemetry_timeout_s + 0.15)

        snapshot = self.vm.get_ui_snapshot()
        self.assertEqual(snapshot.status, SystemStatus.E_STOPPED)


class TestDataIntegrity(unittest.TestCase):
    """4. 數據完整性與快照導出測試"""

    def setUp(self) -> None:
        self.engine = MockMotorCANEngine(node_id = 1)
        self.controller = MockMotorController(self.engine)
        self.rx_worker = MockMotorRxWorker(self.engine)

        self.vm = MotorControlViewModel(
            controller = self.controller,
            rx_worker = self.rx_worker
        )
        self.test_csv = "test_output.csv"

    def tearDown(self) -> None:
        self.vm.stop_control()
        if os.path.exists(self.test_csv):
            os.remove(self.test_csv)

    def test_ui_snapshot_and_history(self) -> None:
        self.vm.select_provider("Sine", amplitude = 0.5, frequency = 1.0)
        self.vm.start_control()
        time.sleep(0.2)

        snapshot = self.vm.get_ui_snapshot()
        self.assertGreater(snapshot.timestamp, 0.0)
        self.assertIsInstance(snapshot.p_des, float)

        times, p_des, p_act, v_act, torque = self.vm.get_plot_data_history()
        self.assertGreater(len(times), 0)
        self.assertEqual(len(times), len(p_des))

    def test_csv_export_integrity(self) -> None:
        self.vm.select_provider("Sine", amplitude = 0.5, frequency = 1.0)
        self.vm.start_control()
        time.sleep(0.2)
        self.vm.stop_control()

        self.vm.save_csv_log(self.test_csv)
        self.assertTrue(os.path.exists(self.test_csv))

        with open(self.test_csv, "r", encoding = "utf-8") as f:
            reader = list(csv.reader(f))
            self.assertEqual(reader[0], ["time_s", "p_des", "p_act", "v_des", "v_act", "torque_act"])
            self.assertGreater(len(reader), 1)