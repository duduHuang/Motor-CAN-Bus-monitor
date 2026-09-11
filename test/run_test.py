"""
自動化 Headless 測試執行器
發掘並執行 test 模組下的所有單元與整合測試。
"""

import sys
import time
import unittest


def run_headless_test_suite() -> None:
    loader = unittest.TestLoader()
    suite = loader.discover(start_dir = "test", pattern = "test_headless_suite.py")

    print("\n啟動 CAN 馬達驅動驗證專案 - 第三階段 Headless 自動化測試...")
    print("-" * 60)

    start_time = time.perf_counter()
    runner = unittest.TextTestRunner(verbosity = 2)
    result = runner.run(suite)
    elapsed = time.perf_counter() - start_time

    total = result.testsRun
    failures = len(result.failures)
    errors = len(result.errors)
    passed = total - failures - errors

    print("\n" + "-" * 60)
    print("測試總覽:")
    print(f"   - 總計測試案例 : {total}")
    print(f"   - 成功 (Passed)  : {passed}")
    print(f"   - 失敗 (Failed)  : {failures}")
    print(f"   - 異常 (Errors)  : {errors}")
    print(f"   - 總執行耗時     : {elapsed:.3f} 秒")
    print("-" * 60)

    if failures > 0 or errors > 0:
        print("測試未通過：請檢查上述失敗細節！")
        sys.exit(1)
    else:
        print("所有單元與整合測試均已成功通過！")


if __name__ == "__main__":
    run_headless_test_suite()