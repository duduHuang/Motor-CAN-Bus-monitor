#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

#include "model/motor_state_db.hpp"
#include "model/rx_worker.hpp"
#include "model/socket_can_interface.hpp"

using namespace robot::model;

int main() {
    // PREEMPT_RT 記憶體鎖定：防止 Page Fault 導致即時性受損
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[Warning] mlockall 失敗: " << std::strerror(errno) 
                  << " (請以 root 權限執行以達到即時記憶體鎖定)\n";
    }

    const std::string ifname = "vcan0";
    std::cout << "=== Step 5: RxWorker 即時接收執行緒測試 ===" << std::endl;

    // 1. 初始化 SocketCAN 與 MotorStateDB
    SocketCANInterface can_rx_if;
    if (!can_rx_if.open(ifname)) {
        std::cerr << "[Error] 無法開啟 " << ifname 
                  << "。請先建立虛擬 CAN: `sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`\n";
        return 1;
    }

    MotorStateDB db;

    // 2. 建立 RxWorker (指定 Core 5, SCHED_FIFO 優先級 90)
    RxWorker rx_worker(can_rx_if, db, /*cpu_core=*/5, /*rt_priority=*/90);

    // 3. 啟動即時背景執行緒
    std::cout << "[Test] 啟動 RxWorker 執行緒 (Pin to CPU Core 5, Priority 90)..." << std::endl;
    if (!rx_worker.start()) {
        std::cerr << "[Error] 無法啟動 RxWorker\n";
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    assert(rx_worker.is_running());

    // 4. 透過獨立 CAN 介面發送模擬測試封包至 vcan0
    SocketCANInterface can_tx_if;
    can_tx_if.open(ifname);

    struct can_frame test_frame{};
    test_frame.can_id = 0x501; // Motor ID 1 (Motion RX)
    test_frame.can_dlc = 8;
    // 模擬 8-byte MIT Feedback Raw Payload
    test_frame.data[0] = 0x01; // Motor ID echo
    test_frame.data[1] = 0x80; // p_int high
    test_frame.data[2] = 0x00; // p_int low
    test_frame.data[3] = 0x80; // v_int high
    test_frame.data[4] = 0x00; // v_int low
    test_frame.data[5] = 0x80; // t_int high
    test_frame.data[6] = 0x00; // t_int low
    test_frame.data[7] = 0x00;

    std::cout << "[Test] 注入模擬 CAN 封包 (CAN ID: 0x501) 至 " << ifname << "..." << std::endl;
    bool send_ok = can_tx_if.send_frame(test_frame);
    assert(send_ok);

    // 5. 等待 RxWorker 於超低延遲 (<10μs) 下完成無鎖注入
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 6. 驗證 MotorStateDB 無鎖資料庫是否即時更新
    MITTelemetry result_telemetry{};
    bool db_updated = db.get_mit_telemetry(1, result_telemetry);

    if (db_updated) {
        std::cout << "[SUCCESS] MotorStateDB 已成功被 RxWorker 即時更新！" << std::endl;
        std::cout << "          Motor ID 1 遙測更新時間戳: " 
                  << result_telemetry.timestamp_ns << " ns\n";
    } else {
        std::cerr << "[FAIL] MotorStateDB 未收到來自 RxWorker 的更新數據！\n";
        rx_worker.stop();
        return 1;
    }

    // 7. 安全關閉
    std::cout << "[Test] 停止 RxWorker 執行緒..." << std::endl;
    rx_worker.stop();
    assert(!rx_worker.is_running());

    std::cout << "=== Step 5 RxWorker 單元測試全部通過！ ===" << std::endl;
    return 0;
}