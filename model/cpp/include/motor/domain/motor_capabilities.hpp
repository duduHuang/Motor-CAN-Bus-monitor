// include/motor/domain/motor_capabilities.hpp
#pragma once

#include <cstdint>

namespace motor::domain {

enum class Capability : uint16_t {
    None                   = 0,
    PositionControl        = uint16_t{1} << 0,
    VelocityControl        = uint16_t{1} << 1,
    TorqueControl          = uint16_t{1} << 2,
    ImpedanceControl       = uint16_t{1} << 3,
    HardwareBrake          = uint16_t{1} << 4,
    HardwareWatchdog       = uint16_t{1} << 5,
    MultiTurnPosition      = uint16_t{1} << 6,
    MotorTempFeedback      = uint16_t{1} << 7,
    MosTempFeedback        = uint16_t{1} << 8,
    VoltageFeedback        = uint16_t{1} << 9,
    CurrentFeedback        = uint16_t{1} << 10,
    TorqueFeedback         = uint16_t{1} << 11
};

[[nodiscard]] constexpr Capability operator|(Capability lhs, Capability rhs) noexcept {
    return static_cast<Capability>(static_cast<uint16_t>(lhs) | static_cast<uint16_t>(rhs));
}

constexpr Capability& operator|=(Capability& lhs, Capability rhs) noexcept {
    lhs = lhs | rhs;
    return lhs;
}

struct MotorCapabilities {
    uint16_t bits{0};

    [[nodiscard]] constexpr bool has(Capability cap) const noexcept {
        return (bits & static_cast<uint16_t>(cap)) != 0;
    }
    [[nodiscard]] constexpr bool has_all(Capability mask) const noexcept {
        const auto m = static_cast<uint16_t>(mask);
        return (bits & m) == m;
    }
    [[nodiscard]] constexpr bool has_any(Capability mask) const noexcept {
        return (bits & static_cast<uint16_t>(mask)) != 0;
    }

    constexpr void set(Capability cap) noexcept {
        bits |= static_cast<uint16_t>(cap);
    }
    constexpr void clear(Capability cap) noexcept {
        bits &= static_cast<uint16_t>(~static_cast<uint16_t>(cap));
    }
    constexpr void reset() noexcept { bits = 0; }
};

} // namespace motor::domain