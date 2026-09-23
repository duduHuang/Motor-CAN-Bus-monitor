#include <cassert>
#include <chrono>
#include <iostream>
#include <sys/mman.h> // for mlockall
#include <thread>

#include "model/motor_state_db.hpp"
#include "model/rx_worker.hpp"
#include "model/socket_can_interface.hpp"

using namespace robot::model;

int main() {
    // 核心規範 1：PREEMPT_RT 記憶體鎖定
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[Warning] mlockall 失敗 (請以 sudo 執行以鎖定即時記憶體)\n";
    }

    SocketCANInterface can_rx_if;
    if (!can_rx_if.open("vcan0")) {
        std::cerr << "[Error] vcan0 開啟失敗\n";
        return 1;
    }

    MotorStateDB db;
    RxWorker rx_worker(can_rx_if, db, 5, 90);
    
    std::cout << "[Test] 啟動 RxWorker (CPU 5, FIFO 90)...\n";
    assert(rx_worker.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    SocketCANInterface can_tx_if;
    can_tx_if.open("vcan0");

    // ==============================================================
    // 測試情境 A：模擬 CAN 雜訊 Spike (第三層防禦測試)
    // ==============================================================
    struct can_frame noise_frame{};
    noise_frame.can_id = 0x501; // Motor 1
    noise_frame.can_dlc = 8;
    // 隨機填入垃圾數值以產生極大的角度與速度解碼結果
    noise_frame.data[0] = 0x01;
    noise_frame.data[1] = 0xFF; noise_frame.data[2] = 0xFF; // 極端 P
    noise_frame.data[3] = 0x7F; noise_frame.data[4] = 0xFF; // 極端 V
    noise_frame.data[5] = 0x00; noise_frame.data[6] = 0x00;
    noise_frame.data[7] = 0x00;

    std::cout << "[Test] 注入第一幀 CAN 雜訊 (Spike)...\n";
    can_tx_if.send_frame(noise_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    // 驗證 1：單一突波應該被濾波器丟棄或覆寫為 0，DB 不應判定故障
    assert(db.is_motor_fault(1) == false);
    std::cout << "       -> 單一突波防禦成功！\n";

    std::cout << "[Test] 連續注入 3 幀 CAN 雜訊 (模擬真實過速)...\n";
    can_tx_if.send_frame(noise_frame);
    can_tx_if.send_frame(noise_frame);
    can_tx_if.send_frame(noise_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // 驗證 2：連續 3 幀異常，濾波器必須通知 DB 觸發 Fault
    assert(db.is_motor_fault(1) == true);
    std::cout << "       -> 連續異常 E-STOP 觸發成功！\n";

    rx_worker.stop();
    std::cout << "=== 1000Hz RT RxWorker 測試通過！ ===\n";
    return 0;
}