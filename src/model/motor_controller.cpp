#include "model/motor_controller.hpp"
#include <iostream>
#include <linux/can.h>
#include <unistd.h>
#include <thread>
#include <chrono>

namespace robot::model {

MotorController::MotorController() noexcept = default;

MotorController::~MotorController() noexcept {
    stop();
}

void MotorController::register_rx_callback(RxMessageCallback cb) {
    std::lock_guard<std::mutex> life_lock(lifecycle_mutex_);
    std::lock_guard<std::mutex> cb_lock(callback_mutex_);

    rx_callbacks_.push_back(cb);
    if (rx_worker_) {
        rx_worker_->add_rx_callback(cb);
    }
}

bool MotorController::init(
    const std::string& interface_name, 
    const std::vector<uint8_t>& active_motor_ids, 
    int rx_core_id
) noexcept {
    std::lock_guard<std::mutex> life_lock(lifecycle_mutex_);

    if (is_running_.load(std::memory_order_acquire)) return true;
    if (!can_iface_.open(interface_name)) return false;

    try {
        rx_worker_ = std::make_unique<RxWorker>(can_iface_, state_db_, can_logger_, rx_core_id, 90);
        {
            std::lock_guard<std::mutex> cb_lock(callback_mutex_);
            for (const auto& cb : rx_callbacks_) {
                rx_worker_->add_rx_callback(cb);
            }
        }
        if (!rx_worker_->start()) {
            can_iface_.close();
            rx_worker_.reset();
            return false;
        }
    } catch (...) {
        can_iface_.close();
        return false;
    }

    // 修正：語法錯誤 std::order_release -> std::memory_order_release
    is_running_.store(true, std::memory_order_release);
    
    // 修正：根據 Watchdog 結果決定是否掛上 Degraded 標籤
    if (!setup_hardware_watchdog(active_motor_ids, 300)) {
        is_degraded_.store(true, std::memory_order_release);
    } else {
        is_degraded_.store(false, std::memory_order_release);
    }
    
    return true;
}

void MotorController::stop() noexcept {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    
    if (!is_running_.exchange(false, std::memory_order_seq_cst)) {
        return;
    }

    // 修改：統一使用 memory_order_seq_cst
    while (in_flight_operations_.load(std::memory_order_seq_cst) > 0) {
#if defined(__aarch64__) || defined(_M_ARM64)
        asm volatile("yield" ::: "memory");
#elif defined(__x86_64__)
        asm volatile("pause" ::: "memory");
#else
        std::this_thread::yield();
#endif
    }

    // 第三步：此時保證已無任何執行緒持有 fd_ 或存取 rx_worker_，可以安全清理
    if (rx_worker_) {
        cached_rx_count_.store(rx_worker_->get_rx_count(), std::memory_order_relaxed);
        rx_worker_->stop();
        rx_worker_.reset();
    }

    can_iface_.close();
}

bool MotorController::setup_hardware_watchdog(const std::vector<uint8_t>& motor_ids, uint32_t timeout_ms) noexcept {
    if (motor_ids.empty()) return false;
    std::array<uint8_t, 8> payload = {0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    payload[4] = static_cast<uint8_t>(timeout_ms & 0xFF);
    payload[5] = static_cast<uint8_t>((timeout_ms >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>((timeout_ms >> 16) & 0xFF);
    payload[7] = static_cast<uint8_t>((timeout_ms >> 24) & 0xFF);

    bool all_success = true;
    for (uint8_t id : motor_ids) {
        if (id < 1 || id > MAX_MOTOR_ID) {
            all_success = false;
            continue;
        }
        
        // 紀錄失敗狀態
        if (!send_single_command(id, payload)) {
            all_success = false;
        }
        
        // 註解提醒：若此指令寫入 ROM，頻繁重啟系統會加速 Flash 耗損。
        // 建議未來可先發送 Query 讀取當前逾時時間，若不符再下發 0xB3 寫入。
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
    }
    return all_success;
}

bool MotorController::send_single_command(uint8_t motor_id, const std::array<uint8_t, 8>& payload) noexcept {
    if (motor_id < 1 || motor_id > MAX_MOTOR_ID) return false;
    
    InFlightGuard guard(is_running_, in_flight_operations_);
    if (!guard.is_active()) return false;
    
    const uint32_t can_id = SINGLE_MOTOR_BASE_TX + motor_id;
    bool success = can_iface_.send_frame(can_id, payload);
    if (success) {
        tx_count_.fetch_add(1, std::memory_order_relaxed);
        can_logger_.add_log(can_id, payload, true);
        return true;
    }
    return false;
}

bool MotorController::send_multi_command(const std::array<uint8_t, 8>& payload) noexcept {
    InFlightGuard guard(is_running_, in_flight_operations_);
    if (!guard.is_active()) return false;
    
    if (can_iface_.send_frame(MULTI_MOTOR_BASE_TX, payload)) {
        tx_count_.fetch_add(1, std::memory_order_relaxed);
        can_logger_.add_log(MULTI_MOTOR_BASE_TX, payload, true);
        return true;
    }
    return false;
}

bool MotorController::send_motion_command(uint8_t motor_id, const std::array<uint8_t, 8>& payload) noexcept {
    if (motor_id < 1 || motor_id > MAX_MOTOR_ID) return false;
    InFlightGuard guard(is_running_, in_flight_operations_);
    if (!guard.is_active()) return false;
    
    const uint32_t can_id = MOTION_MODE_BASE_TX + motor_id;
    if (can_iface_.send_frame(can_id, payload)) {
        tx_count_.fetch_add(1, std::memory_order_relaxed);
        can_logger_.add_log(can_id, payload, true);
        return true;
    }
    return false;
}

uint64_t MotorController::get_rx_count() const noexcept {
    InFlightGuard guard(is_running_, in_flight_operations_);
    if (!guard.is_active()) {
        return cached_rx_count_.load(std::memory_order_relaxed);
    }
    if (rx_worker_) {
        return rx_worker_->get_rx_count();
    }
    return cached_rx_count_.load(std::memory_order_relaxed);
}

// ---------------------------------------------------------
// 以下讀取 API 僅存取 MotorStateDB，該結構為靜態陣列且具備 Seqlock，
// 不會引發 Use-after-free，因此無需 In-flight 保護，維持極致效能。
// ---------------------------------------------------------

bool MotorController::get_mit_telemetry(uint8_t motor_id, MITTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    if (state_db_.get_fault(motor_id) || state_db_.is_mit_telemetry_stale(motor_id, 0.3)) return false;
    double out_ts = 0.0;
    return state_db_.get_mit_telemetry(motor_id, out_data, out_ts);
}

bool MotorController::get_motion_telemetry(uint8_t motor_id, StandardMotionTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    if (state_db_.get_fault(motor_id) || state_db_.is_motion_telemetry_stale(motor_id, 0.3)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_motion_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_single_turn_telemetry(uint8_t motor_id, SingleTurnMotionTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_single_turn_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_sensor_telemetry(uint8_t motor_id, SensorStatus1Telemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_sensor_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_sensor3_telemetry(uint8_t motor_id, SensorStatus3Telemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_sensor3_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_pid_telemetry(uint8_t motor_id, PIDQueryTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_pid_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_accel_telemetry(uint8_t motor_id, AccelQueryTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_accel_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_encoder_pos_telemetry(uint8_t motor_id, EncoderPosTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_encoder_pos_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_zero_offset_telemetry(uint8_t motor_id, ZeroOffsetTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_zero_offset_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_angle_query_telemetry(uint8_t motor_id, AngleQueryTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_angle_query_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_system_mode_telemetry(uint8_t motor_id, SystemModeTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_system_mode_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_system_info_telemetry(uint8_t motor_id, SystemInfoTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_system_info_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_motor_model_telemetry(uint8_t motor_id, MotorModelTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_motor_model_telemetry(motor_id, out_data, dummy_ts);
}

bool MotorController::get_write_ack_telemetry(uint8_t motor_id, WriteAckTelemetry& out_data) const noexcept {
    if (!is_running_.load(std::memory_order_acquire)) return false;
    double dummy_ts = 0.0;
    return state_db_.get_write_ack_telemetry(motor_id, out_data, dummy_ts);
}

} // namespace robot::model