#pragma once

#include <cstdint>
#include <atomic>
#include <array>
#include <ctime>

// 引入 protocol 層定義的結構體
#include "protocol/mit_protocol.hpp"
#include "protocol/decoders.hpp"

namespace robot::model {

// 使用 using 讓 DB 層直接引用 protocol 層的定義，避免重複定義與型別衝突
using MITTelemetry = ::protocol::MITTelemetry;
using StandardMotionTelemetry = ::servo_robot::protocol::StandardMotionTelemetry;
using SingleTurnMotionTelemetry = ::servo_robot::protocol::SingleTurnMotionTelemetry;
using SensorStatus1Telemetry = ::servo_robot::protocol::SensorStatus1Telemetry;
using SensorStatus3Telemetry    = ::servo_robot::protocol::SensorStatus3Telemetry;
using PIDQueryTelemetry         = ::servo_robot::protocol::PIDQueryTelemetry;
using AccelQueryTelemetry       = ::servo_robot::protocol::AccelQueryTelemetry;
using EncoderPosTelemetry       = ::servo_robot::protocol::EncoderPosTelemetry;
using ZeroOffsetTelemetry       = ::servo_robot::protocol::ZeroOffsetTelemetry;
using AngleQueryTelemetry       = ::servo_robot::protocol::AngleQueryTelemetry;
using SystemModeTelemetry       = ::servo_robot::protocol::SystemModeTelemetry;
using SystemInfoTelemetry       = ::servo_robot::protocol::SystemInfoTelemetry;
using MotorModelTelemetry       = ::servo_robot::protocol::MotorModelTelemetry;
using WriteAckTelemetry         = ::servo_robot::protocol::WriteAckTelemetry;

/**
 * @brief Seqlock (Sequence Lock) 零記憶體分配無鎖資料槽
 * 專為 ARM Cortex-A78AE (ARMv8-A) 弱記憶體序 (Weakly-Ordered) 架構設計。
 * 透過顯式記憶體屏障 (Memory Fence / acquire-release) 保證 1000Hz RT 迴圈無競爭讀寫。
 */
template <typename T>
struct alignas(64) SeqlockSlot { // 64-byte 對齊，防止跨核 False Sharing
    std::atomic<uint32_t> sequence{0};
    T data{};
    double timestamp{0.0};

    /**
     * @brief 寫入端 (CAN RX 背景執行緒，非阻塞)
     */
    void write(const T& new_data, double ts) noexcept {
        uint32_t current_seq = sequence.load(std::memory_order_relaxed);
        
        sequence.store(current_seq + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);

        data = new_data;
        timestamp = ts;

        std::atomic_thread_fence(std::memory_order_release);
        sequence.store(current_seq + 2, std::memory_order_release);
    }

    /**
     * @brief 讀取端 (1000Hz RT 控制執行緒，非阻塞 O(1) 時間複雜度)
     */
    bool read(T& out_data, double& out_ts) const noexcept {
        constexpr int MAX_RETRIES = 3;
        for (int retry = 0; retry < MAX_RETRIES; ++retry) {
            uint32_t seq1 = sequence.load(std::memory_order_acquire);
            
            if (seq1 & 1) {
#if defined(__aarch64__)
                asm volatile("yield" ::: "memory");
#endif
                continue;
            }

            out_data = data;
            out_ts = timestamp;

            std::atomic_thread_fence(std::memory_order_acquire);
            uint32_t seq2 = sequence.load(std::memory_order_relaxed);

            if (seq1 == seq2) {
                return true;
            }
        }
        return false;
    }
};

// 單顆馬達對應之多型別 Telemetry 靜態槽
struct alignas(64) MotorSlot {
    SeqlockSlot<MITTelemetry>               mit_telemetry;
    SeqlockSlot<StandardMotionTelemetry>    motion_telemetry;
    SeqlockSlot<SingleTurnMotionTelemetry>  single_turn_telemetry;
    SeqlockSlot<SensorStatus1Telemetry>     sensor_telemetry;
    SeqlockSlot<SensorStatus3Telemetry>     sensor3_telemetry;
    SeqlockSlot<PIDQueryTelemetry>          pid_telemetry;
    SeqlockSlot<AccelQueryTelemetry>        accel_telemetry;
    SeqlockSlot<EncoderPosTelemetry>        encoder_pos_telemetry;
    SeqlockSlot<ZeroOffsetTelemetry>        zero_offset_telemetry;
    SeqlockSlot<AngleQueryTelemetry>        angle_query_telemetry;
    SeqlockSlot<SystemModeTelemetry>        system_mode_telemetry;
    SeqlockSlot<SystemInfoTelemetry>        system_info_telemetry;
    SeqlockSlot<MotorModelTelemetry>        motor_model_telemetry;
    SeqlockSlot<WriteAckTelemetry>          write_ack_telemetry;
    
    // 【第四層防禦】硬體/上位機 Fault 與 Cascade E-STOP 觸發標記
    std::atomic<bool> is_faulted{false};
};

/**
 * @brief 馬達狀態資料庫 (MotorStateDB) - 單例類別
 */
class MotorStateDB {
public:
    static constexpr uint8_t MAX_MOTORS = 63;

    static MotorStateDB& instance() noexcept;

    MotorStateDB(const MotorStateDB&) = delete;
    MotorStateDB& operator=(const MotorStateDB&) = delete;
    MotorStateDB(MotorStateDB&&) = delete;
    MotorStateDB& operator=(MotorStateDB&&) = delete;

    // === 寫入 API ===
    void update_mit_telemetry(uint8_t motor_id, const MITTelemetry& data, double timestamp = 0.0) noexcept;
    void update_motion_telemetry(uint8_t motor_id, const StandardMotionTelemetry& data, double timestamp = 0.0) noexcept;
    void update_single_turn_telemetry(uint8_t motor_id, const SingleTurnMotionTelemetry& data, double timestamp = 0.0) noexcept;
    void update_sensor_telemetry(uint8_t motor_id, const SensorStatus1Telemetry& data, double timestamp = 0.0) noexcept;
    void update_sensor3_telemetry(uint8_t motor_id, const SensorStatus3Telemetry& data, double timestamp = 0.0) noexcept;
    void update_pid_telemetry(uint8_t motor_id, const PIDQueryTelemetry& data, double timestamp = 0.0) noexcept;
    void update_accel_telemetry(uint8_t motor_id, const AccelQueryTelemetry& data, double timestamp = 0.0) noexcept;
    void update_encoder_pos_telemetry(uint8_t motor_id, const EncoderPosTelemetry& data, double timestamp = 0.0) noexcept;
    void update_zero_offset_telemetry(uint8_t motor_id, const ZeroOffsetTelemetry& data, double timestamp = 0.0) noexcept;
    void update_angle_query_telemetry(uint8_t motor_id, const AngleQueryTelemetry& data, double timestamp = 0.0) noexcept;
    void update_system_mode_telemetry(uint8_t motor_id, const SystemModeTelemetry& data, double timestamp = 0.0) noexcept;
    void update_system_info_telemetry(uint8_t motor_id, const SystemInfoTelemetry& data, double timestamp = 0.0) noexcept;
    void update_motor_model_telemetry(uint8_t motor_id, const MotorModelTelemetry& data, double timestamp = 0.0) noexcept;
    void update_write_ack_telemetry(uint8_t motor_id, const WriteAckTelemetry& data, double timestamp = 0.0) noexcept;

    // === 讀取 API ===
    bool get_mit_telemetry(uint8_t motor_id, MITTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_motion_telemetry(uint8_t motor_id, StandardMotionTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_single_turn_telemetry(uint8_t motor_id, SingleTurnMotionTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_sensor_telemetry(uint8_t motor_id, SensorStatus1Telemetry& out_data, double& out_timestamp) const noexcept;
    bool get_sensor3_telemetry(uint8_t motor_id, SensorStatus3Telemetry& out_data, double& out_timestamp) const noexcept;
    bool get_pid_telemetry(uint8_t motor_id, PIDQueryTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_accel_telemetry(uint8_t motor_id, AccelQueryTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_encoder_pos_telemetry(uint8_t motor_id, EncoderPosTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_zero_offset_telemetry(uint8_t motor_id, ZeroOffsetTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_angle_query_telemetry(uint8_t motor_id, AngleQueryTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_system_mode_telemetry(uint8_t motor_id, SystemModeTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_system_info_telemetry(uint8_t motor_id, SystemInfoTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_motor_model_telemetry(uint8_t motor_id, MotorModelTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_write_ack_telemetry(uint8_t motor_id, WriteAckTelemetry& out_data, double& out_timestamp) const noexcept;

    bool is_telemetry_stale(uint8_t motor_id, double max_stale_sec = 0.3) const noexcept;

    void set_fault(uint8_t motor_id, bool faulted) noexcept;
    bool get_fault(uint8_t motor_id) const noexcept;

    static double get_monotonic_time_sec() noexcept;

private:
    MotorStateDB() noexcept = default;
    ~MotorStateDB() noexcept = default;

    static constexpr bool is_valid_motor_id(uint8_t motor_id) noexcept {
        return motor_id >= 1 && motor_id <= MAX_MOTORS;
    }

    alignas(64) std::array<MotorSlot, MAX_MOTORS> slots_{};
};

} // namespace robot::model