#include "model/rx_worker.hpp"

#include <cstring>
#include <iostream>
#include <sys/mman.h>

namespace robot::model {

RxWorker::RxWorker(SocketCANInterface& socket_can, 
                   MotorStateDB& db, 
                   int cpu_core, 
                   int rt_priority)
    : socket_can_(socket_can), 
      db_(db), 
      cpu_core_(cpu_core), 
      rt_priority_(rt_priority) {}

RxWorker::~RxWorker() {
    stop();
}

bool RxWorker::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return false;
    }

    running_.store(true, std::memory_order_release);
    
    // 啟動 C++17 std::thread
    worker_thread_ = std::thread(&RxWorker::worker_loop, this);

    // 取得 native pthread 句柄設定 PREEMPT_RT 屬性
    pthread_t native_handle = worker_thread_.native_handle();

    // 1. 排程策略設定：SCHED_FIFO，優先權 90 (高於普通 Linux 任務，僅次於 Kernel 硬體中斷)
    sched_param param{};
    param.sched_priority = rt_priority_;
    int ret = pthread_setschedparam(native_handle, SCHED_FIFO, &param);
    if (ret != 0) {
        std::cerr << "[RxWorker] 警告: 設定 SCHED_FIFO 優先級 " 
                  << rt_priority_ << " 失敗: " << std::strerror(ret) 
                  << " (請確認執行權限與 root / CAP_SYS_NICE)" << std::endl;
    }

    // 2. CPU 核心綁定 (Affinity Pinning) 至獨立核心 (如 Core 5)，避免 OS 與 CUDA 中斷干擾
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core_, &cpuset);
    ret = pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        std::cerr << "[RxWorker] 警告: 鎖定 CPU Core " 
                  << cpu_core_ << " 失敗: " << std::strerror(ret) << std::endl;
    }

    return true;
}

void RxWorker::stop() {
    if (running_.exchange(false, std::memory_order_acq_rel)) {
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }
}

void RxWorker::worker_loop() {
    // 預先在 Stack 上分配 Zero-Allocation 封包與解碼快照緩衝區 (絕不發起 Heap Allocation)
    struct can_frame frame{};
    MITTelemetry mit_telemetry{};
    StandardMotionTelemetry motion_telemetry{};
    SensorStatus1Telemetry status1_telemetry{};

    while (running_.load(std::memory_order_relaxed)) {
        // SocketCAN 非阻塞接收 (Direct Low-latency Kernel Read)
        if (socket_can_.recv_frame(frame)) {
            const uint32_t can_id = frame.can_id & CAN_EFF_MASK;
            const uint8_t* payload = frame.data;

            // 1. 運動模式區段 (0x501 ~ 0x53F) -> MIT Protocol 快速解碼
            if (can_id > 0x500 && can_id <= 0x53F) {
                const uint8_t motor_id = static_cast<uint8_t>(can_id - 0x500);
                if (MITProtocol::decode(payload, mit_telemetry)) {
                    // Lock-free 寫入 MotorStateDB (無鎖寫入)
                    db_.update_mit_telemetry(motor_id, mit_telemetry);
                }
            } 
            // 2. 單機模式區段 (0x241 ~ 0x27F) -> ServoDecoder 解碼
            else if (can_id > 0x240 && can_id <= 0x27F) {
                const uint8_t motor_id = static_cast<uint8_t>(can_id - 0x240);
                const uint8_t cmd_type = payload[0];

                if (cmd_type == 0x9C) { // 狀態 2: 轉矩電流/輸出軸轉速
                    if (ServoDecoder::decode_motion(payload, motion_telemetry)) {
                        db_.update_motion_telemetry(motor_id, motion_telemetry);
                    }
                } else if (cmd_type == 0x9A) { // 狀態 1: 馬達溫度/供電電壓
                    if (ServoDecoder::decode_status1(payload, status1_telemetry)) {
                        db_.update_status1_telemetry(motor_id, status1_telemetry);
                    }
                }
            }
        } else {
            // 無封包時進行極短 Pause/Yield，確保 CPU 不吃滿，同時響應延遲 < 10μs
            #if defined(__aarch64__) || defined(_M_ARM64)
            asm volatile("yield" ::: "memory"); // ARM Cortex-A78AE 原生 CPU Pause 指令
            #else
            std::this_thread::yield();
            #endif
        }
    }
}

} // namespace robot::model