// include/motor/domain/motor_descriptor.hpp
#pragma once

#include <cstdint>
#include <cassert>
#include "motor_address.hpp"
#include "joint_id.hpp"
#include "motor_profile_id.hpp"
#include "motor_limits.hpp"

namespace motor::domain {

enum class Direction : int8_t {
    Invalid  = 0,
    Positive = 1,
    Negative = -1
};

[[nodiscard]] constexpr bool is_valid_direction(Direction dir) noexcept {
    return dir == Direction::Positive || dir == Direction::Negative;
}

/**
 * @brief 安全轉算方向符號 (防止非法 Direction 默默產生 0 位置命令)
 */
[[nodiscard]] constexpr bool try_direction_sign(Direction dir, float& out_sign) noexcept {
    if (dir == Direction::Positive) {
        out_sign = 1.0f;
        return true;
    } else if (dir == Direction::Negative) {
        out_sign = -1.0f;
        return true;
    }
    out_sign = 0.0f;
    return false;
}

/**
 * @brief 馬達個體描述檔
 */
struct MotorDescriptor {
    MotorAddress address{};
    JointId joint_id{JointId::Unknown};
    MotorProfileId profile_id{};

    Direction direction{Direction::Invalid};
    float joint_zero_offset_rad{0.0f};   // 關節輸出軸零點偏移 (rad)
    float motor_to_output_ratio{0.0f};   // 減速比
    float gear_efficiency{0.0f};         // 傳動效率 (0.0 ~ 1.0)

    JointSafetyLimits joint_limits{};
};

} // namespace motor::domain