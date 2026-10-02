// include/motor/domain/motor_state.hpp
#pragma once

#include <cstdint>

namespace motor::domain {

enum class StateField : uint16_t {
    None             = 0,
    Position         = uint16_t{1} << 0,
    Velocity         = uint16_t{1} << 1,
    Torque           = uint16_t{1} << 2,
    MotorTemperature = uint16_t{1} << 3,
    MosTemperature   = uint16_t{1} << 4,
    Voltage          = uint16_t{1} << 5
};

[[nodiscard]] constexpr StateField operator|(StateField lhs, StateField rhs) noexcept {
    return static_cast<StateField>(static_cast<uint16_t>(lhs) | static_cast<uint16_t>(rhs));
}

struct StateValidity {
    uint16_t bits{0};

    [[nodiscard]] constexpr bool has(StateField field) const noexcept {
        return (bits & static_cast<uint16_t>(field)) != 0;
    }
    [[nodiscard]] constexpr bool has_all(StateField mask) const noexcept {
        const auto m = static_cast<uint16_t>(mask);
        return (bits & m) == m;
    }
    [[nodiscard]] constexpr bool has_any(StateField mask) const noexcept {
        return (bits & static_cast<uint16_t>(mask)) != 0;
    }

    constexpr void set(StateField field) noexcept {
        bits |= static_cast<uint16_t>(field);
    }
    constexpr void clear(StateField field) noexcept {
        bits &= static_cast<uint16_t>(~static_cast<uint16_t>(field));
    }
    constexpr void reset() noexcept { bits = 0; }
};

enum class TorqueSource : uint8_t {
    Unavailable = 0,
    ProtocolReported,         // 馬達驅動器直接回報
    EstimatedFromIqCurrent,   // 由相電流粗略估算
    ExternalSensor            // 外部扭矩感測器
};

struct MotorState {
    float position_rad{0.0f};       // 關節輸出軸位置 (rad)
    float velocity_radps{0.0f};     // 關節輸出軸角速度 (rad/s)
    float torque_nm{0.0f};          // 關節輸出軸扭矩 (Nm)

    float motor_temperature_c{0.0f};// 馬達線圈溫度 (°C)
    float mos_temperature_c{0.0f};  // 驅動器 MOS 溫度 (°C)
    float voltage_v{0.0f};          // 母線電壓 (V)

    StateValidity validity{};
    TorqueSource torque_source{TorqueSource::Unavailable};

    uint64_t monotonic_timestamp_ns{0}; // RX 被 Transport 接收時的時間
    uint32_t sample_sequence{0};        // 0 為初始未接收，1...UINT32_MAX 遞增
};

/**
 * @brief 驗證 MotorState 內部 Torque Validity 與 TorqueSource 契約一致性
 */
[[nodiscard]] constexpr bool is_motor_state_consistent(const MotorState& state) noexcept {
    const bool is_torque_valid = state.validity.has(StateField::Torque);
    const bool is_source_available = (state.torque_source != TorqueSource::Unavailable);

    // Torque 有效則 Source 不得為 Unavailable；Torque 無效則 Source 必須為 Unavailable
    return (is_torque_valid == is_source_available);
}

} // namespace motor::domain