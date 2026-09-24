#include "model/rx_worker.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

namespace robot::model {

RxWorker::RxWorker(SocketCANInterface& socket_can, MotorStateDB& db, int cpu_core, int rt_priority)
    : socket_can_(socket_can), db_(db), cpu_core_(cpu_core), rt_priority_(rt_priority) {}

RxWorker::~RxWorker() {
    stop();
}

bool RxWorker::start() {
    if (running_.load(std::memory_order_relaxed)) return false;
    running_.store(true, std::memory_order_release);
    
    worker_thread_ = std::thread(&RxWorker::worker_loop, this);
    pthread_t native_handle = worker_thread_.native_handle();

    // 1. RT 排程策略設定：SCHED_FIFO
    sched_param param{};
    param.sched_priority = rt_priority_;
    if (pthread_setschedparam(native_handle, SCHED_FIFO, &param) != 0) {
        std::cerr << "[RxWorker] 致命錯誤: SCHED_FIFO 設定失敗\n";
    }

    // 2. CPU 核心綁定 (Core Affinity)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core_, &cpuset);
    if (pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "[RxWorker] 致命錯誤: CPU Core 綁定失敗\n";
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

inline void RxWorker::process_kinematic_filter(uint8_t motor_id, MITTelemetry& telemetry) {
    if (motor_id >= filters_.size()) return;
    
    auto& filter = filters_[motor_id];

    if (!filter.initialized) {
        filter.last_position_rad = telemetry.position_rad;
        filter.initialized = true;
        filter.error_count = 0;
        db_.update_mit_telemetry(motor_id, telemetry);
        return;
    }

    float delta_pos = std::abs(telemetry.position_rad - filter.last_position_rad);
    float abs_vel = std::abs(telemetry.velocity_rads);

    // 第三層防禦：檢測是否超出物理極限 (Spike 雜訊)
    if (delta_pos > KinematicFilter::MAX_POS_DELTA_PER_MS || abs_vel > KinematicFilter::MAX_SPEED_RADS) {
        filter.error_count++;
        
        if (filter.error_count >= KinematicFilter::DEBOUNCE_THRESHOLD) {
            // 連續 N 幀異常：確認為真實故障/過速，透過無鎖 DB 標記 E-STOP 狀態
            db_.set_fault(motor_id, true);
        } else {
            // 單一/少數突波：消抖機制 (用上一幀的合理數值覆蓋掉當前的垃圾數據)
            telemetry.position_rad = filter.last_position_rad;
            // 寫入修復後的數據至 DB (確保 1000Hz 控制迴圈不會因為這一幀解算出暴衝的 KP/KD 扭矩)
            db_.update_mit_telemetry(motor_id, telemetry);
        }
    } else {
        // 數據正常，重置錯誤計數，更新上一幀姿態，並寫入無鎖 DB
        filter.error_count = 0;
        filter.last_position_rad = telemetry.position_rad;
        db_.update_mit_telemetry(motor_id, telemetry);
    }
}

void RxWorker::worker_loop() {
    // 嚴格禁忌：在此函數內部絕對禁止 new, malloc, std::cout, printf, mutex.lock()
    uint32_t can_id = 0;
    std::array<uint8_t, 8> payload{};
    MITTelemetry mit_telemetry{};

    while (running_.load(std::memory_order_relaxed)) {
        // 使用 SocketCANInterface 宣告的 (can_id, payload) 介面
        if (socket_can_.recv_frame(can_id, payload)) {
            rx_count_.fetch_add(1, std::memory_order_relaxed);
            const uint32_t clean_id = can_id & 0x1FFFFFFF;

            // 1. MIT 運動模式區段 (0x501 ~ 0x53F)
            if (clean_id > 0x500 && clean_id <= 0x53F) {
                const uint8_t motor_id = static_cast<uint8_t>(clean_id - 0x500);
                
                // 第二層防禦：NaN/Inf 數值合法性檢查
                auto decoded_mit = MITProtocol::decode_telemetry(payload);
                if (decoded_mit.has_value()) {
                    mit_telemetry = decoded_mit.value();
                    // 第三層防禦：運動學去脈衝與 3 幀 Debounce 消抖
                    process_kinematic_filter(motor_id, mit_telemetry);
                }
            } 
            // 2. 單機模式區段 (0x241 ~ 0x27F)
            else if (clean_id > 0x240 && clean_id <= 0x27F) {
                const uint8_t motor_id = static_cast<uint8_t>(clean_id - 0x240);
                
                // 使用 ServoDecoder::decode_any 搭配 std::get_if
                auto decoded = servo_robot::protocol::ServoDecoder::decode_any(payload);

                if (auto* motion = std::get_if<servo_robot::protocol::StandardMotionTelemetry>(&decoded)) {
                    db_.update_motion_telemetry(motor_id, *motion);
                } else if (auto* single_turn = std::get_if<servo_robot::protocol::SingleTurnMotionTelemetry>(&decoded)) {
                    db_.update_single_turn_telemetry(motor_id, *single_turn); // [修正] 處理 0xA6
                } else if (auto* sensor = std::get_if<servo_robot::protocol::SensorStatus1Telemetry>(&decoded)) {
                    db_.update_sensor_telemetry(motor_id, *sensor);
                } else if (auto* sensor3 = std::get_if<servo_robot::protocol::SensorStatus3Telemetry>(&decoded)) {
                    db_.update_sensor3_telemetry(motor_id, *sensor3); // [修正] 處理 0x9D
                }
            }
        } else {
            #if defined(__aarch64__) || defined(_M_ARM64)
            asm volatile("yield" ::: "memory");
            #elif defined(__x86_64__)
            asm volatile("pause" ::: "memory");
            #endif
        }
    }
}

} // namespace robot::model