// include/motor/domain/motor_address.hpp
#pragma once

#include <cstdint>
#include <compare>

namespace motor::domain {

/**
 * @brief 馬達匯流排實體定址 (Trivially Copyable)
 */
struct MotorAddress {
    uint8_t bus_index{0}; // CAN Bus 索引 (例如 0 代表 can1)
    uint8_t motor_id{0};  // 匯流排實體 ID

    /**
     * @brief 檢查定址是否具備結構上的 basic non-zero 合法性
     * @note 實際匯流排與 ID 範圍由 Registry 根據已配置 bus 驗證
     */
    [[nodiscard]] constexpr bool is_structurally_valid() const noexcept {
        return motor_id != 0;
    }

    [[nodiscard]] constexpr auto operator<=>(const MotorAddress&) const noexcept = default;
};

} // namespace motor::domain