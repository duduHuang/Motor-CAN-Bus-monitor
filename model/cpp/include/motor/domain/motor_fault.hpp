// include/motor/domain/motor_fault.hpp
#pragma once

#include <cstdint>

namespace motor::domain {

/**
 * @brief 故障分類位元旗標 (Unsigned Shifting)
 */
enum class MotorFaultFlag : uint32_t {
    None                   = 0,
    HardwareOverTemperature= uint32_t{1} << 0,
    HardwareOverCurrent    = uint32_t{1} << 1,
    HardwareOverVoltage    = uint32_t{1} << 2,
    HardwareLowVoltage     = uint32_t{1} << 3,
    EncoderError           = uint32_t{1} << 4,
    Stall                  = uint32_t{1} << 5,
    StaleData              = uint32_t{1} << 6,  // 通訊逾時過期
    KinematicSpike         = uint32_t{1} << 7,  // 運動學位置跳變
    LimitsViolation        = uint32_t{1} << 8,  // 超出軟體極限
    CommunicationError     = uint32_t{1} << 9,  // CAN 傳輸或協定載荷解析錯誤
    WatchdogFailure        = uint32_t{1} << 10 // 馬達硬體 Watchdog 未能正常 ACK
};

[[nodiscard]] constexpr MotorFaultFlag operator|(MotorFaultFlag lhs, MotorFaultFlag rhs) noexcept {
    return static_cast<MotorFaultFlag>(static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs));
}

constexpr MotorFaultFlag& operator|=(MotorFaultFlag& lhs, MotorFaultFlag rhs) noexcept {
    lhs = lhs | rhs;
    return lhs;
}

/**
 * @brief 提供型別化的 Fault Flag 操作介面 (保持 POD / Aggregate 簡單性)
 */
struct MotorFaultSet {
    uint32_t bits{0};

    [[nodiscard]] constexpr bool has(MotorFaultFlag flag) const noexcept {
        return (bits & static_cast<uint32_t>(flag)) != 0;
    }
    [[nodiscard]] constexpr bool has_any(MotorFaultFlag mask) const noexcept {
        return (bits & static_cast<uint32_t>(mask)) != 0;
    }
    constexpr void set(MotorFaultFlag flag) noexcept {
        bits |= static_cast<uint32_t>(flag);
    }
    constexpr void clear(MotorFaultFlag flag) noexcept {
        bits &= ~static_cast<uint32_t>(flag);
    }
    constexpr void reset() noexcept { bits = 0; }
};

} // namespace motor::domain