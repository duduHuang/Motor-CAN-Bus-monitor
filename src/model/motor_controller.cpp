#include "model/motor_controller.hpp"
#include <iostream>
#include <linux/can.h>

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
    if (!can_iface_.open_interface(interface_name)) {
        return false;
    }

    // 2. 配置 SocketCAN Kernel/硬體濾波器 (只接收 0x241~0x27F 與 0x501~0x53F)
    struct can_filter rfilter[2];
    // 單機模式回傳區段 (0x240 Mask 0x7C0 -> 0x240~0x27F)
    rfilter[0].can_id   = SINGLE_MOTOR_BASE_RX;
    rfilter[0].can_mask = 0x7C0;
    // 運動模式回傳區段 (0x500 Mask 0x7C0 -> 0x500~0x53F)
    rfilter[1].can_id   = MOTION_MODE_BASE_RX;
    rfilter[1].can_mask = 0x7C0;

    if (!can_iface_.set_filters(rfilter, 2)) {
        can_iface_.close_interface();
        return false;
    }

    // 3. 實例化並啟動 RxWorker (綁定核心與設定 Real-time 優先權)
    try {
        rx_worker_ = std::make_unique<RxWorker>(can_iface_, state_db_);
        if (!rx_worker_->start(rx_core_id, 85)) { // RxWorker 優先權 85
            can_iface_.close_interface();
            rx_worker_.reset();
            return false;
        }
    } catch (...) {
        can_iface_.close_interface();
        return false;
    }

    is_initialized_.store(true, std::memory_order_release);
    return true;
}

void MotorController::stop() noexcept {
    if (!is_initialized_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }

    if (rx_worker_) {
        rx_worker_->stop();
        rx_worker_.reset();
    }

    can_iface_.close_interface();
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
    return state_db_.get_mit_telemetry(motor_id, out_data);
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