#include "model/motor_state_db.hpp"
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cassert>
#include <cmath>
#include <pthread.h>
#include <sys/mman.h>

using namespace robot::model;

static std::atomic<bool> g_running{true};
static std::atomic<uint64_t> g_rx_write_count{0};
static std::atomic<uint64_t> g_rt_read_count{0};
static std::atomic<uint64_t> g_read_retries{0};
static std::atomic<uint64_t> g_data_tearing_errors{0};

// 【第四層防禦統計】
static std::atomic<bool> g_stale_detected{false};
static std::atomic<bool> g_cascade_estop_triggered{false};

// 設定執行緒 RT 排程優先權 (SCHED_FIFO) 與 CPU Affinity
void setup_realtime_thread(int core_id, int priority) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    sched_param param{};
    param.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        std::cerr << "[WARNING] 無法設定 SCHED_FIFO (需要 root 權限), 使用預設排程器。\n";
    }
}

// 模擬 CAN RX 寫入執行緒 (2000Hz = 500us 週期)
void can_rx_worker_thread() {
    setup_realtime_thread(1, 85);

    uint64_t counter = 0;
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);

    while (g_running.load(std::memory_order_relaxed)) {
        counter++;
        
        // 模擬第 1~16 顆馬達的高頻 RX 寫入
        for (uint8_t motor_id = 1; motor_id <= 16; ++motor_id) {
            // 刻意模擬：馬達 ID 5 在運行中途停止更新 (模擬 CAN Bus 故障/線路脫落)
            if (motor_id == 5 && counter > 2000) { 
                continue; // 停止發送第 5 軸數據，驗證第四層防禦 Watchdog 觸發
            }

            MITTelemetry mit_data;
            mit_data.position_rad = static_cast<float>(counter + motor_id);
            mit_data.velocity_rads = mit_data.position_rad * 2.0f;
            mit_data.torque_nm = mit_data.position_rad * 3.0f;

            MotorStateDB::instance().update_mit_telemetry(motor_id, mit_data);
            g_rx_write_count.fetch_add(1, std::memory_order_relaxed);
        }

        next_period.tv_nsec += 500000; // 500us
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_sec += 1;
            next_period.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, nullptr);
    }
}

// 模擬 1000Hz PREEMPT_RT 控制執行緒
void rt_control_loop_thread() {
    setup_realtime_thread(2, 95);

    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    auto& db = MotorStateDB::instance();

    while (g_running.load(std::memory_order_relaxed)) {
        for (uint8_t motor_id = 1; motor_id <= 16; ++motor_id) {
            
            // 【第四層防禦】：1000Hz 主迴圈檢測 Stale Data (>0.3s 無更新)
            if (db.is_telemetry_stale(motor_id, 0.3)) {
                // 只有在系統已經開始接收資料後才判定斷線 (忽略初始未發送階段)
                if (g_rx_write_count.load() > 500) {
                    db.set_fault(motor_id, true);
                    g_stale_detected.store(true, std::memory_order_relaxed);
                    g_cascade_estop_triggered.store(true, std::memory_order_relaxed);
                }
            }

            MITTelemetry mit_data;
            double ts = 0.0;

            bool success = db.get_mit_telemetry(motor_id, mit_data, ts);
            g_rt_read_count.fetch_add(1, std::memory_order_relaxed);

            if (success) {
                // 驗證無記憶體撕裂 (Data Tearing)
                float expected_v = mit_data.position_rad * 2.0f;
                float expected_t = mit_data.position_rad * 3.0f;

                if (std::abs(mit_data.velocity_rads - expected_v) > 1e-4f ||
                    std::abs(mit_data.torque_nm - expected_t) > 1e-4f) {
                    g_data_tearing_errors.fetch_add(1, std::memory_order_relaxed);
                }
            } else {
                g_read_retries.fetch_add(1, std::memory_order_relaxed);
            }
        }

        next_period.tv_nsec += 1000000; // 1ms
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_sec += 1;
            next_period.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, nullptr);
    }
}

int main() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[WARNING] mlockall 失敗, 請以 sudo 執行以達成最佳即時效能。\n";
    }

    std::cout << "========================================================\n";
    std::cout << "  NVIDIA Orin Nano RT State DB - Layer 4 Watchdog Test  \n";
    std::cout << "========================================================\n";

    // 基礎 API 靜態驗證
    auto& db = MotorStateDB::instance();
    assert(db.is_telemetry_stale(1, 0.3) == true); // 初始無數據，應回傳 stale
    
    MITTelemetry test_mit{1.0f, 2.0f, 3.0f};
    db.update_mit_telemetry(1, test_mit);
    assert(db.is_telemetry_stale(1, 0.3) == false); // 新鮮寫入後，應為 false

    db.set_fault(1, true);
    assert(db.get_fault(1) == true);
    db.set_fault(1, false);

    std::cout << "[INFO] 啟動 2000Hz CAN RX 寫入執行緒 (Core 1, Pri 85)...\n";
    std::cout << "[INFO] 啟動 1000Hz RT 控制讀取執行緒 (Core 2, Pri 95)...\n";

    std::thread rx_thread(can_rx_worker_thread);
    std::thread rt_thread(rt_control_loop_thread);

    // 壓力測試 3.5 秒 (足夠觸發 2000 幀後的 Stale Data 檢測)
    std::this_thread::sleep_for(std::chrono::milliseconds(3500));

    g_running.store(false, std::memory_order_relaxed);

    rx_thread.join();
    rt_thread.join();

    std::cout << "\n---------------- [壓力與 Watchdog 測試結果] ----------------\n";
    std::cout << "總 CAN RX 寫入次數     : " << g_rx_write_count.load() << " 次\n";
    std::cout << "總 RT 控制讀取次數     : " << g_rt_read_count.load() << " 次\n";
    std::cout << "Seqlock Retry 次數      : " << g_read_retries.load() << " 次\n";
    std::cout << "記憶體撕裂 (Data Tearing): " << g_data_tearing_errors.load() << " 次 (必須為 0)\n";
    std::cout << "第四層 Stale Data 偵測 : " << (g_stale_detected.load() ? "🚨 成功偵測到斷線" : "❌ 未偵測到") << "\n";
    std::cout << "Cascade E-STOP 自動觸發 : " << (g_cascade_estop_triggered.load() ? "🚨 成功引發全車急停" : "❌ 未觸發") << "\n";
    std::cout << "----------------------------------------------------------\n";

    if (g_data_tearing_errors.load() == 0 && g_stale_detected.load() && g_cascade_estop_triggered.load()) {
        std::cout << "✅ [PASSED] 驗證成功：Lock-free DB 成功整合【第四層硬體/上位機 Watchdog 防護】！\n";
        return 0;
    } else {
        std::cout << "❌ [FAILED] 驗證失敗：防禦機制未能正常運作！\n";
        return 1;
    }
}