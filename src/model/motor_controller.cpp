#include "model/motor_controller.hpp"
#include <iostream>
#include <linux/can.h>
#include <unistd.h>

namespace robot::model {

MotorController::MotorController() noexcept = default;

MotorController::~MotorController() noexcept {
    stop();
}

bool MotorController::init(const std::string& interface_name, int rx_core_id) noexcept {
    if (is_initialized_.load(std::memory_order_acquire)) {
        return true;
    }

    // 1. 初始化 SocketCAN 介面
    if (!can_iface_.open(interface_name)) {
        return false;
    }

    // 2. 實例化並啟動 RxWorker (內部會配置 SCHED_FIFO 與 Core Affinity)
    try {
        rx_worker_ = std::make_unique<RxWorker>(can_iface_, state_db_, rx_core_id, 90);
        if (!rx_worker_->start()) {
            can_iface_.close();
            rx_worker_.reset();
            return false;
        }
    } catch (...) {
        can_iface_.close();
        return false;
    }

    is_initialized_.store(true, std::memory_order_release);
    // 第四層防禦：啟動後立即對 12 顆馬達下發 0xB3 斷聯保護指令
    setup_hardware_watchdog(300); // 設定 300ms 抱閘鎖死保護
    return true;
}

void MotorController::setup_hardware_watchdog(uint32_t timeout_ms) noexcept {
    // 依據通訊手冊組裝 0xB3 指令[cite: 8]
    std::array<uint8_t, 8> payload = {0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    payload[4] = static_cast<uint8_t>(timeout_ms & 0xFF);          // CanRecvTime_MS low byte 1[cite: 8]
    payload[5] = static_cast<uint8_t>((timeout_ms >> 8) & 0xFF);   // CanRecvTime_MS byte 2[cite: 8]
    payload[6] = static_cast<uint8_t>((timeout_ms >> 16) & 0xFF);  // CanRecvTime_MS byte 3[cite: 8]
    payload[7] = static_cast<uint8_t>((timeout_ms >> 24) & 0xFF);  // CanRecvTime_MS byte 4[cite: 8]

    // 依序向四足 12 顆馬達發出設定，掉電後會存入 ROM[cite: 8]
    for (uint8_t id = 1; id <= MAX_MOTOR_ID; ++id) {
        send_single_command(id, payload);
        usleep(1000); // 避免瞬間塞爆 SocketCAN TX Buffer (僅於初始化時使用)
    }
}

void MotorController::stop() noexcept {
    if (!is_initialized_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }

    if (rx_worker_) {
        rx_worker_->stop();
        rx_worker_.reset();
    }

    can_iface_.close();
}

bool MotorController::send_single_command(uint8_t motor_id, const std::array<uint8_t, 8>& payload) noexcept {
    if (!is_initialized_.load(std::memory_order_relaxed) || motor_id < 1 || motor_id > MAX_MOTOR_ID) {
        return false;
    }
    const uint32_t can_id = SINGLE_MOTOR_BASE_TX + motor_id;
    if (can_iface_.send_frame(can_id, payload.data(), 8)) {
        tx_count_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    return false;
}

bool MotorController::send_multi_command(const std::array<uint8_t, 8>& payload) noexcept {
    if (!is_initialized_.load(std::memory_order_relaxed)) {
        return false;
    }
    if (can_iface_.send_frame(MULTI_MOTOR_BASE_TX, payload.data(), 8)) {
        tx_count_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    return false;
}

bool MotorController::send_motion_command(uint8_t motor_id, const std::array<uint8_t, 8>& payload) noexcept {
    if (!is_initialized_.load(std::memory_order_relaxed) || motor_id < 1 || motor_id > MAX_MOTOR_ID) {
        return false;
    }
    const uint32_t can_id = MOTION_MODE_BASE_TX + motor_id;
    if (can_iface_.send_frame(can_id, payload.data(), 8)) {
        tx_count_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    return false;
}

bool MotorController::get_mit_telemetry(uint8_t motor_id, MITTelemetry& out_data) const noexcept {
    if (!is_initialized_.load(std::memory_order_relaxed)) {
        return false;
    }
    // 第四層防禦：檢查馬達是否回報 Fault，或資料已嚴重過期 (Stale > 300ms)
    // 假設 MotorStateDB 內部實作了基於 monotonic clock 的超時判定
    if (state_db_.get_fault(motor_id) || state_db_.is_telemetry_stale(motor_id, 0.3)) {
        return false; // 回傳 false 觸發 1000Hz 主控制迴圈之 Cascade E-STOP
    }
    double out_ts = 0.0;
    return state_db_.get_mit_telemetry(motor_id, out_data, out_ts);
}

bool MotorController::get_motion_telemetry(uint8_t motor_id, StandardMotionTelemetry& out_data) const noexcept {
    if (!is_initialized_.load(std::memory_order_relaxed)) {
        return false;
    }
    return state_db_.get_motion_telemetry(motor_id, out_data);
}

uint64_t MotorController::get_rx_count() const noexcept {
    if (!rx_worker_) return 0;
    return rx_worker_->get_rx_count();
}

} // namespace robot::model