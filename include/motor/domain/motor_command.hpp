// include/motor/domain/motor_command.hpp
#pragma once

#include <cstdint>

namespace motor::domain {

enum class ControlMode : uint8_t {
    Passive = 0,
    Position,
    Velocity,
    Torque,
    Impedance
};

/**
 * @brief 持續性馬達控制指令
 */
struct MotorCommand {
    ControlMode mode{ControlMode::Passive};

    float target_position_rad{0.0f};    // 目標位置 (rad)
    float target_velocity_radps{0.0f};  // 目標速度 / 前饋速度 (rad/s)
    float target_torque_nm{0.0f};       // Torque 模式目標轉矩 / Impedance 模式前饋轉矩 (Nm)

    float max_velocity_radps{0.0f};     // 允許的最大轉速限制 (rad/s)
    float max_torque_nm{0.0f};          // 允許的最大轉矩限制 (Nm)
    float kp{0.0f};                     // 位置剛度 (Nm/rad)
    float kd{0.0f};                     // 速度阻尼 (Nm/(rad/s))
};

} // namespace motor::domain