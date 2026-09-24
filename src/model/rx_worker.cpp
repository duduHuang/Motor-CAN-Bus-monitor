#include "model/rx_worker.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <variant>
#include <chrono>

namespace robot::model {

RxWorker::RxWorker(SocketCANInterface& socket_can, MotorStateDB& db, CANFrameLogger& logger, int cpu_core, int rt_priority)
    : socket_can_(socket_can), db_(db), logger_(logger), cpu_core_(cpu_core), rt_priority_(rt_priority) {}

RxWorker::~RxWorker() {
    stop();
}

void RxWorker::add_rx_callback(RxMessageCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (cb) {
        callbacks_.push_back(std::move(cb));
    }
}

inline void RxWorker::notify_callbacks(uint8_t motor_id, const RxTelemetryMessage& msg) {
    // 1. 立即上鎖，保護對 vector 的讀取與遍歷，消除 Data Race
    std::lock_guard<std::mutex> lock(callback_mutex_);
    
    // 2. 確定有註冊 Callback 才進行後續處理
    if (callbacks_.empty()) return;

    // 3. 在確定需要發送通知時才擷取高精度時間戳
    const auto now = std::chrono::system_clock::now();
    const double ts = std::chrono::duration<double>(now.time_since_epoch()).count();

    // 4. 執行分發
    for (const auto& cb : callbacks_) {
        cb(motor_id, msg, ts);
    }
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
            db_.set_fault(motor_id, true);
        } else {
            telemetry.position_rad = filter.last_position_rad;
            db_.update_mit_telemetry(motor_id, telemetry);
        }
    } else {
        filter.error_count = 0;
        filter.last_position_rad = telemetry.position_rad;
        db_.update_mit_telemetry(motor_id, telemetry);
    }
}

void RxWorker::worker_loop() {
    uint32_t can_id = 0;
    std::array<uint8_t, 8> payload{};
    MITTelemetry mit_telemetry{};

    while (running_.load(std::memory_order_relaxed)) {
        if (socket_can_.recv_frame(can_id, payload)) {
            rx_count_.fetch_add(1, std::memory_order_relaxed);
            logger_.add_log(can_id, payload, false);
            const uint32_t clean_id = can_id & 0x1FFFFFFF;

            // 1. MIT 運動模式區段 (0x501 ~ 0x53F)
            if (clean_id > 0x500 && clean_id <= 0x53F) {
                const uint8_t motor_id = static_cast<uint8_t>(clean_id - 0x500);
                
                auto decoded_mit = MITProtocol::decode_telemetry(payload);
                if (decoded_mit.has_value()) {
                    mit_telemetry = decoded_mit.value();
                    process_kinematic_filter(motor_id, mit_telemetry);
                    // =====================================
                    // 觸發 MIT 封包 Callback
                    // =====================================
                    RxTelemetryMessage msg;
                    msg.payload = mit_telemetry;
                    notify_callbacks(motor_id, msg);
                }
            } 
            // 2. 單機模式區段 (0x241 ~ 0x27F)
            else if (clean_id > 0x240 && clean_id <= 0x27F) {
                const uint8_t motor_id = static_cast<uint8_t>(clean_id - 0x240);
                
                // 使用 ServoDecoder 解碼，搭配 std::visit 完整覆蓋所有 14 種 Telemetry 變體
                auto decoded = servo_robot::protocol::ServoDecoder::decode_any(payload);

                std::visit([this, motor_id](auto&& arg) {
                    using T = std::decay_t<decltype(arg)>;
                    if constexpr (std::is_same_v<T, servo_robot::protocol::StandardMotionTelemetry>) {
                        db_.update_motion_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::SingleTurnMotionTelemetry>) {
                        db_.update_single_turn_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::SensorStatus1Telemetry>) {
                        db_.update_sensor_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::SensorStatus3Telemetry>) {
                        db_.update_sensor3_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::PIDQueryTelemetry>) {
                        db_.update_pid_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::AccelQueryTelemetry>) {
                        db_.update_accel_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::EncoderPosTelemetry>) {
                        db_.update_encoder_pos_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::ZeroOffsetTelemetry>) {
                        db_.update_zero_offset_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::AngleQueryTelemetry>) {
                        db_.update_angle_query_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::SystemModeTelemetry>) {
                        db_.update_system_mode_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::SystemInfoTelemetry>) {
                        db_.update_system_info_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::MotorModelTelemetry>) {
                        db_.update_motor_model_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::WriteAckTelemetry>) {
                        db_.update_write_ack_telemetry(motor_id, arg);
                    } else if constexpr (std::is_same_v<T, servo_robot::protocol::UnknownTelemetry>) {
                        // 雜訊/未定義封包過濾，不存入 DB
                    }
                }, decoded);
                // =====================================
                // 觸發 Servo 封包 Callback
                // 若為雜訊(UnknownTelemetry) 則忽略不發送通知
                // =====================================
                if (!std::holds_alternative<servo_robot::protocol::UnknownTelemetry>(decoded)) {
                    RxTelemetryMessage msg;
                    msg.payload = decoded;
                    notify_callbacks(motor_id, msg);
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