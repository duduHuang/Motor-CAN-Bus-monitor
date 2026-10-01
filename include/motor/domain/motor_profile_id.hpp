// include/motor/domain/motor_profile_id.hpp
#pragma once

#include <cstdint>
#include <compare>

namespace motor::domain {

/**
 * @brief 強型別 Profile ID (防止非預期整數混入)
 */
struct MotorProfileId {
    uint16_t value{0}; // 0 代表 Invalid / Unconfigured

    [[nodiscard]] constexpr bool is_valid() const noexcept {
        return value != 0;
    }

    [[nodiscard]] constexpr auto operator<=>(const MotorProfileId&) const noexcept = default;
};

} // namespace motor::domain