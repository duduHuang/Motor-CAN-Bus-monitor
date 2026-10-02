// include/motor/domain/motor_health.hpp
#pragma once

#include <cstdint>
#include "motor_fault.hpp"

namespace motor::domain {

/**
 * @brief 個別馬達健康摘要 (僅記錄屬於該馬達的 RX 時間)
 */
struct MotorHealth {
    MotorFaultSet current_faults{};
    MotorFaultSet latched_faults{};
    MotorFaultSet warnings{};
    
    uint16_t vendor_raw_code{0};     
    bool is_degraded{false};         

    uint64_t last_rx_timestamp_ns{0};           // 成功路由至此馬達的最後 RX 時間
    uint64_t last_valid_state_timestamp_ns{0}; // 成功解碼出有效 MotorState 的時間

    [[nodiscard]] constexpr bool has_current_fault() const noexcept { return current_faults.bits != 0; }
    [[nodiscard]] constexpr bool has_latched_fault() const noexcept { return latched_faults.bits != 0; }
    [[nodiscard]] constexpr bool has_warning() const noexcept { return warnings.bits != 0; }
    
    [[nodiscard]] constexpr bool is_fault_free() const noexcept {
        return current_faults.bits == 0 && latched_faults.bits == 0;
    }

    [[nodiscard]] constexpr bool is_nominal() const noexcept {
        return is_fault_free() && warnings.bits == 0 && !is_degraded;
    }
};

} // namespace motor::domain