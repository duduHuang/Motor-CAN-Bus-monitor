#include "model/motor_state_db.hpp"
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cassert>
#include <pthread.h>
#include <sys/mman.h>

using namespace robot::model;

static std::atomic<bool> g_running{true};
static std::atomic<uint64_t> g_rx_write_count{0};
static std::atomic<uint64_t> g_rt_read_count{0};
static std::atomic<uint64_t> g_read_retries{0};
static std::atomic<uint64_t> g_data_tearing_errors{0};

// 設定執行緒 RT 排程優先權 (SCHED_FIFO) 與 CPU Affinity
void setup_realtime_thread(int core_id, int priority) {
    // 綁定 CPU 核心
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    // 設定 SCHED_FIFO 優先權
    sched_param param{};
    param.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        std::cerr << "[WARNING] 無法設定 SCHED_FIFO (需要 root 權限), 使用預設排程器。\n";
    }
}

// 模擬高頻 CAN RX 寫入執行緒 (2000Hz = 500us 週期)
void can_rx_worker_thread() {
    setup_realtime_thread(1, 85); // 綁定 CPU Core 1, 優先權 85

    uint64_t counter = 0;
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);

    while (g_running.load(std::memory_order_relaxed)) {
        counter++;
        
        // 對 16 顆馬達進行交錯更新
        for (uint8_t motor_id = 1; motor_id <= MotorStateDB::MAX_MOTORS; ++motor_id) {
            MITTelemetry mit_data;
            // 寫入帶有代數關聯特徵之數據：p_act, v_act, torque_act 滿足 1 : 2 : 3 比例
            mit_data.position_rad = static_cast<float>(counter + motor_id);
            mit_data.velocity_rads = mit_data.position_rad * 2.0f;
            mit_data.torque_nm = mit_data.position_rad * 3.0f;

            MotorStateDB::instance().update_mit_telemetry(motor_id, mit_data);
            g_rx_write_count.fetch_add(1, std::memory_order_relaxed);
        }

        // 精準 2000Hz (500,000 ns) 定時補償
        next_period.tv_nsec += 500000;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_sec += 1;
            next_period.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, nullptr);
    }
}

// 模擬 1000Hz PREEMPT_RT 控制執行緒 (1000Hz = 1ms 週期)
void rt_control_loop_thread() {
    setup_realtime_thread(2, 95); // 綁定 CPU Core 2, 最高優先權 95

    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);

    while (g_running.load(std::memory_order_relaxed)) {
        for (uint8_t motor_id = 1; motor_id <= MotorStateDB::MAX_MOTORS; ++motor_id) {
            MITTelemetry mit_data;
            double ts = 0.0;

            bool success = MotorStateDB::instance().get_mit_telemetry(motor_id, mit_data, ts);
            g_rt_read_count.fetch_add(1, std::memory_order_relaxed);

            if (success) {
                // 驗證數據一致性：檢測有無記憶體撕裂 (Data Tearing)
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

        // 精準 1000Hz (1,000,000 ns) 定時補償
        next_period.tv_nsec += 1000000;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_sec += 1;
            next_period.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, nullptr);
    }
}

int main() {
    // 1. 鎖定全域記憶體，防止 Page Fault 造成 PREEMPT_RT 抖動
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[WARNING] mlockall 失敗, 請以 sudo 執行此測試程式以達成最佳即時效能。\n";
    }

    std::cout << "========================================================\n";
    std::cout << "  NVIDIA Orin Nano PREEMPT_RT Lock-free State DB Test   \n";
    std::cout << "========================================================\n";
    std::cout << "[INFO] 啟動 2000Hz CAN RX 寫入執行緒 (Core 1, Pri 85)...\n";
    std::cout << "[INFO] 啟動 1000Hz RT 控制讀取執行緒 (Core 2, Pri 95)...\n";

    std::thread rx_thread(can_rx_worker_thread);
    std::thread rt_thread(rt_control_loop_thread);

    // 壓力測試持續時間：3 秒
    constexpr int TEST_DURATION_SEC = 3;
    std::this_thread::sleep_for(std::chrono::seconds(TEST_DURATION_SEC));

    g_running.store(false, std::memory_order_relaxed);

    rx_thread.join();
    rt_thread.join();

    std::cout << "\n---------------- [壓力測試結果] ----------------\n";
    std::cout << "總 CAN RX 寫入次數 : " << g_rx_write_count.load() << " 次\n";
    std::cout << "總 RT 控制讀取次數 : " << g_rt_read_count.load() << " 次\n";
    std::cout << "Seqlock Retry 次數  : " << g_read_retries.load() << " 次\n";
    std::cout << "記憶體撕裂 (Tearing): " << g_data_tearing_errors.load() << " 次 (必須為 0)\n";
    std::cout << "------------------------------------------------\n";

    if (g_data_tearing_errors.load() == 0) {
        std::cout << "✅ [PASSED] 驗證成功：Lock-free DB 於高頻併發下 1000Hz 零阻塞且無資料撕裂！\n";
        return 0;
    } else {
        std::cout << "❌ [FAILED] 驗證失敗：檢測到記憶體撕裂現象，請檢查 Seqlock 屏障實作！\n";
        return 1;
    }
}