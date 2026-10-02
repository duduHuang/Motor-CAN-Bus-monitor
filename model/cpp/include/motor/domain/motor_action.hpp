// include/motor/domain/motor_action.hpp
#pragma once

#include <cstdint>

namespace motor::domain {

/**
 * @brief 單次/生命週期馬達控制動作
 * @note QuickStop 為單軸快速停止請求；整機 E-stop 為 Service 層全機狀態轉換。
 */
enum class MotorAction : uint8_t {
    None = 0,
    Enable,
    Disable,
    Stop,           // 請求馬達停止主動運動 (狀態由 Safety/Protocol 決定)
    QuickStop,      // 單軸快速停止
    ReleaseBrake,
    LockBrake,
    ClearFault,
    SetZeroPosition
};

} // namespace motor::domain