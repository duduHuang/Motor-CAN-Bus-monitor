// include/motor/domain/motor_coordinate_transform.hpp
#pragma once

#include "motor_descriptor.hpp"
#include "torque_types.hpp"

namespace motor::domain {

/**
 * @brief 關節位置正向學：Motor Shaft Position -> Joint Output Shaft Position
 * @formula joint_pos = (sign * motor_pos / ratio) + joint_zero_offset
 * @precondition Descriptor 必須在系統初始化時通過 DescriptorValidator 驗證
 */
[[nodiscard]] constexpr bool forward_position(
    float motor_pos_rad,
    const MotorDescriptor& desc,
    float& out_joint_pos_rad) noexcept 
{
    float sign{0.0f};
    if (!try_direction_sign(desc.direction, sign) || desc.motor_to_output_ratio <= 0.0f) {
        return false;
    }
    out_joint_pos_rad = (sign * motor_pos_rad / desc.motor_to_output_ratio) + desc.joint_zero_offset_rad;
    return true;
}

/**
 * @brief 關節位置逆向學：Joint Target Position -> Motor Target Position
 * @formula motor_pos = sign * (joint_pos - joint_zero_offset) * ratio
 * @precondition Descriptor 必須在系統初始化時通過 DescriptorValidator 驗證
 */
[[nodiscard]] constexpr bool inverse_position(
    float joint_pos_rad,
    const MotorDescriptor& desc,
    float& out_motor_pos_rad) noexcept 
{
    float sign{0.0f};
    if (!try_direction_sign(desc.direction, sign) || desc.motor_to_output_ratio <= 0.0f) {
        return false;
    }
    out_motor_pos_rad = sign * (joint_pos_rad - desc.joint_zero_offset_rad) * desc.motor_to_output_ratio;
    return true;
}

/**
 * @brief 關節速度正向學：Motor Shaft Velocity -> Joint Output Shaft Velocity
 * @formula joint_vel = sign * motor_vel / ratio
 */
[[nodiscard]] constexpr bool forward_velocity(
    float motor_vel_radps,
    const MotorDescriptor& desc,
    float& out_joint_vel_radps) noexcept 
{
    float sign{0.0f};
    if (!try_direction_sign(desc.direction, sign) || desc.motor_to_output_ratio <= 0.0f) {
        return false;
    }
    out_joint_vel_radps = (sign * motor_vel_radps) / desc.motor_to_output_ratio;
    return true;
}

/**
 * @brief 關節速度逆向學：Joint Target Velocity -> Motor Target Velocity
 * @formula motor_vel = sign * joint_vel * ratio
 */
[[nodiscard]] constexpr bool inverse_velocity(
    float joint_vel_radps,
    const MotorDescriptor& desc,
    float& out_motor_vel_radps) noexcept 
{
    float sign{0.0f};
    if (!try_direction_sign(desc.direction, sign) || desc.motor_to_output_ratio <= 0.0f) {
        return false;
    }
    out_motor_vel_radps = sign * joint_vel_radps * desc.motor_to_output_ratio;
    return true;
}

/**
 * @brief 關節扭矩正向學：Motor Shaft Torque -> Joint Output Torque
 * @formula 
 *   If MotorShaft: joint_torque = sign * motor_torque * ratio * efficiency
 *   If OutputShaft: joint_torque = sign * motor_torque
 */
[[nodiscard]] constexpr bool forward_torque(
    float motor_torque_nm,
    TorqueLocation location,
    const MotorDescriptor& desc,
    float& out_joint_torque_nm) noexcept 
{
    float sign{0.0f};
    if (!try_direction_sign(desc.direction, sign)) {
        return false;
    }

    if (location == TorqueLocation::MotorShaft) {
        if (desc.motor_to_output_ratio <= 0.0f || desc.gear_efficiency <= 0.0f) {
            return false;
        }
        out_joint_torque_nm = sign * motor_torque_nm * desc.motor_to_output_ratio * desc.gear_efficiency;
        return true;
    } else if (location == TorqueLocation::OutputShaft) {
        out_joint_torque_nm = sign * motor_torque_nm;
        return true;
    }

    return false;
}

/**
 * @brief 關節目標扭矩逆向學：Joint Target Torque -> Motor Target Torque
 * @formula 
 *   If MotorShaft: motor_torque = sign * joint_torque / (ratio * efficiency)
 *   If OutputShaft: motor_torque = sign * joint_torque
 */
[[nodiscard]] constexpr bool inverse_torque(
    float joint_torque_nm,
    TorqueLocation location,
    const MotorDescriptor& desc,
    float& out_motor_torque_nm) noexcept 
{
    float sign{0.0f};
    if (!try_direction_sign(desc.direction, sign)) {
        return false;
    }

    if (location == TorqueLocation::MotorShaft) {
        if (desc.motor_to_output_ratio <= 0.0f || desc.gear_efficiency <= 0.0f) {
            return false;
        }
        out_motor_torque_nm = (sign * joint_torque_nm) / (desc.motor_to_output_ratio * desc.gear_efficiency);
        return true;
    } else if (location == TorqueLocation::OutputShaft) {
        out_motor_torque_nm = sign * joint_torque_nm;
        return true;
    }

    return false;
}

} // namespace motor::domain