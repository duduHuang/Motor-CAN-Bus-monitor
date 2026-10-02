// include/motor/domain/motor_limits.hpp
#pragma once

namespace motor::domain {

/**
 * @brief 馬達端硬體極限 (Motor Shaft Coordinates)
 */
struct MotorHardwareLimits {
    bool configured{false};

    float max_motor_velocity_radps{0.0f};
    float max_motor_torque_nm{0.0f};
    float max_motor_temperature_c{0.0f};
    float max_mos_temperature_c{0.0f};
    float min_voltage_v{0.0f};
    float max_voltage_v{0.0f};
};

/**
 * @brief 關節端軟體安全極限 (Joint Output Shaft Coordinates - Fail Closed)
 */
struct JointSafetyLimits {
    bool configured{false};

    float min_joint_position_rad{0.0f};
    float max_joint_position_rad{0.0f};
    float max_joint_velocity_radps{0.0f};
    float max_joint_acceleration_radps2{0.0f};
    float max_joint_torque_nm{0.0f};

    float max_kp{0.0f};
    float max_kd{0.0f};

    float position_jump_tolerance_rad{0.0f}; // 位移跳變容忍度 (rad)
};

} // namespace motor::domain