// include/motor/domain/sample_sequence.hpp
#pragma once

#include <cstdint>

namespace motor::domain {

/**
 * @brief 計算下一個狀態採樣序號 (Pure Function)
 * @details 0 代表尚未收到有效狀態；1 ... UINT32_MAX 為有效序號；溢位回繞時自動跳過 0。
 */
[[nodiscard]] constexpr uint32_t next_sample_sequence(uint32_t current) noexcept {
    const uint32_t next = current + uint32_t{1};
    return (next == 0) ? uint32_t{1} : next;
}

} // namespace motor::domain