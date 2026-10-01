// include/motor/domain/descriptor_validator.hpp
#pragma once

#include <cstdint>
#include <cmath>
#include "motor_descriptor.hpp"
#include "motor_profile.hpp"

namespace motor::domain {

enum class DescriptorValidationError : uint8_t {
    None = 0,
    InvalidAddress,
    InvalidJointId,
    InvalidProfileId,
    InvalidDirection,
    NonFiniteRatio,
    InvalidRatio,
    NonFiniteEfficiency,
    InvalidEfficiency,
    NonFiniteOffset,
    
    // Joint Limits Errors
    JointLimitsNotConfigured,
    InvalidJointPositionRange,
    InvalidJointVelocityLimit,
    InvalidJointAccelerationLimit,
    InvalidJointTorqueLimit,
    InvalidJointGainsLimit,
    InvalidJointPositionJumpLimit,

    // Profile & Hardware Limits Errors
    ProfileIdMismatch,
    InvalidProfileTorqueLocation,
    InvalidProfileTorqueConstant,
    HardwareLimitsNotConfigured,
    InvalidMotorVelocityLimit,
    InvalidMotorTorqueLimit,
    InvalidMotorTemperatureLimit,
    InvalidMosTemperatureLimit,
    InvalidVoltageRange
};

class DescriptorValidator {
public:
    /**
     * @brief 1. 驗證個別關節 MotorDescriptor (不涉及 Profile)
     */
    [[nodiscard]] static DescriptorValidationError validate_descriptor(const MotorDescriptor& desc) noexcept {
        if (!desc.address.is_structurally_valid()) {
            return DescriptorValidationError::InvalidAddress;
        }
        if (!is_valid_joint_id(desc.joint_id)) {
            return DescriptorValidationError::InvalidJointId;
        }
        if (!desc.profile_id.is_valid()) {
            return DescriptorValidationError::InvalidProfileId;
        }
        if (!is_valid_direction(desc.direction)) {
            return DescriptorValidationError::InvalidDirection;
        }

        // 減速比與效率邊界 (減速比 > 0, 效率 in (0, 1])
        if (!std::isfinite(desc.motor_to_output_ratio)) {
            return DescriptorValidationError::NonFiniteRatio;
        }
        if (desc.motor_to_output_ratio <= 0.0f) {
            return DescriptorValidationError::InvalidRatio;
        }

        if (!std::isfinite(desc.gear_efficiency)) {
            return DescriptorValidationError::NonFiniteEfficiency;
        }
        if (desc.gear_efficiency <= 0.0f || desc.gear_efficiency > 1.0f) {
            return DescriptorValidationError::InvalidEfficiency;
        }

        if (!std::isfinite(desc.joint_zero_offset_rad)) {
            return DescriptorValidationError::NonFiniteOffset;
        }

        // 關節端安全極限 (JointSafetyLimits) 驗證
        const auto& jl = desc.joint_limits;
        if (!jl.configured) {
            return DescriptorValidationError::JointLimitsNotConfigured;
        }
        if (!std::isfinite(jl.min_joint_position_rad) || 
            !std::isfinite(jl.max_joint_position_rad) || 
            jl.min_joint_position_rad >= jl.max_joint_position_rad) {
            return DescriptorValidationError::InvalidJointPositionRange;
        }
        if (!std::isfinite(jl.max_joint_velocity_radps) || jl.max_joint_velocity_radps < 0.0f) {
            return DescriptorValidationError::InvalidJointVelocityLimit;
        }
        if (!std::isfinite(jl.max_joint_acceleration_radps2) || jl.max_joint_acceleration_radps2 < 0.0f) {
            return DescriptorValidationError::InvalidJointAccelerationLimit;
        }
        if (!std::isfinite(jl.max_joint_torque_nm) || jl.max_joint_torque_nm < 0.0f) {
            return DescriptorValidationError::InvalidJointTorqueLimit;
        }
        if (!std::isfinite(jl.max_kp) || jl.max_kp < 0.0f || !std::isfinite(jl.max_kd) || jl.max_kd < 0.0f) {
            return DescriptorValidationError::InvalidJointGainsLimit;
        }
        if (!std::isfinite(jl.position_jump_tolerance_rad) || jl.position_jump_tolerance_rad < 0.0f) {
            return DescriptorValidationError::InvalidJointPositionJumpLimit;
        }

        return DescriptorValidationError::None;
    }

    /**
     * @brief 2. 驗證馬達型號 MotorProfile 規格
     */
    [[nodiscard]] static DescriptorValidationError validate_profile(const MotorProfile& profile) noexcept {
        if (!profile.profile_id.is_valid()) {
            return DescriptorValidationError::InvalidProfileId;
        }

        // 扭矩位置與能力聯動驗證
        const bool location_unknown = (profile.reported_torque_location == TorqueLocation::Unknown);
        const bool location_valid   = is_valid_torque_location(profile.reported_torque_location);
        const bool needs_torque_loc = profile.capabilities.has_any(Capability::TorqueFeedback | Capability::CurrentFeedback);

        // 無論是否需要反饋，非法 Enum Cast (例如 255) 一律拒絕
        if (!location_unknown && !location_valid) {
            return DescriptorValidationError::InvalidProfileTorqueLocation;
        }
        // 若有扭矩/電流反饋能力，扭矩位置不得為 Unknown
        if (needs_torque_loc && !location_valid) {
            return DescriptorValidationError::InvalidProfileTorqueLocation;
        }

        // 電流轉矩常數 Kt 與能力聯動驗證
        const bool needs_current_kt = profile.capabilities.has(Capability::CurrentFeedback);
        if (needs_current_kt) {
            if (!std::isfinite(profile.torque_constant_nm_per_amp) || profile.torque_constant_nm_per_amp <= 0.0f) {
                return DescriptorValidationError::InvalidProfileTorqueConstant;
            }
        } else {
            if (!std::isfinite(profile.torque_constant_nm_per_amp) || profile.torque_constant_nm_per_amp < 0.0f) {
                return DescriptorValidationError::InvalidProfileTorqueConstant;
            }
        }

        // 馬達端硬體極限 (MotorHardwareLimits) 驗證
        const auto& hl = profile.hardware_limits;
        if (!hl.configured) {
            return DescriptorValidationError::HardwareLimitsNotConfigured;
        }
        if (!std::isfinite(hl.max_motor_velocity_radps) || hl.max_motor_velocity_radps < 0.0f) {
            return DescriptorValidationError::InvalidMotorVelocityLimit;
        }
        if (!std::isfinite(hl.max_motor_torque_nm) || hl.max_motor_torque_nm < 0.0f) {
            return DescriptorValidationError::InvalidMotorTorqueLimit;
        }
        if (!std::isfinite(hl.max_motor_temperature_c) || hl.max_motor_temperature_c <= 0.0f) {
            return DescriptorValidationError::InvalidMotorTemperatureLimit;
        }
        if (!std::isfinite(hl.max_mos_temperature_c) || hl.max_mos_temperature_c <= 0.0f) {
            return DescriptorValidationError::InvalidMosTemperatureLimit;
        }
        if (!std::isfinite(hl.min_voltage_v) || !std::isfinite(hl.max_voltage_v) || 
            hl.min_voltage_v < 0.0f || hl.min_voltage_v >= hl.max_voltage_v) {
            return DescriptorValidationError::InvalidVoltageRange;
        }

        return DescriptorValidationError::None;
    }

    /**
     * @brief 3. 驗證 Descriptor 與 Profile 之間的雙向綁定關係 (Registry 必須呼叫此 API)
     */
    [[nodiscard]] static DescriptorValidationError validate_binding(
        const MotorDescriptor& desc, 
        const MotorProfile& profile) noexcept 
    {
        const auto desc_err = validate_descriptor(desc);
        if (desc_err != DescriptorValidationError::None) return desc_err;

        const auto profile_err = validate_profile(profile);
        if (profile_err != DescriptorValidationError::None) return profile_err;

        if (desc.profile_id != profile.profile_id) {
            return DescriptorValidationError::ProfileIdMismatch;
        }

        return DescriptorValidationError::None;
    }

    // 顯式命名的 Boolean 輔助 API
    [[nodiscard]] static bool is_descriptor_valid(const MotorDescriptor& desc) noexcept {
        return validate_descriptor(desc) == DescriptorValidationError::None;
    }

    [[nodiscard]] static bool is_profile_valid(const MotorProfile& profile) noexcept {
        return validate_profile(profile) == DescriptorValidationError::None;
    }

    [[nodiscard]] static bool is_binding_valid(const MotorDescriptor& desc, const MotorProfile& profile) noexcept {
        return validate_binding(desc, profile) == DescriptorValidationError::None;
    }
};

} // namespace motor::domain