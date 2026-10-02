// include/motor/domain/motor_profile.hpp
#pragma once

#include <cstdint>
#include "motor_profile_id.hpp"
#include "motor_capabilities.hpp"
#include "motor_limits.hpp"
#include "torque_types.hpp"

namespace motor::domain {
/**
 * @brief 馬達型號共用靜態 Profile
 */
struct MotorProfile {
    MotorProfileId profile_id{};

    MotorCapabilities capabilities{};
    float torque_constant_nm_per_amp{0.0f}; // 0.0f 為 Fail-Closed
    TorqueLocation reported_torque_location{TorqueLocation::Unknown};
    
    MotorHardwareLimits hardware_limits{};
};

} // namespace motor::domain