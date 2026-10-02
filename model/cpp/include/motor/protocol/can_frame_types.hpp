// include/motor/protocol/can_frame_types.hpp
#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

namespace motor::protocol {

/**
 * @brief CAN 識別碼類型 (標準 11-bit 或 擴展 29-bit)
 */
enum class CANIdType : uint8_t {
    Standard11Bit,
    Extended29Bit
};

/**
 * @brief CAN 訊框類型 (資料訊框 或 遠端請求訊框)
 */
enum class CANFrameType : uint8_t {
    Data,
    Remote
};

/**
 * @brief 硬體與作業系統無關的 CAN 訊框資料結構 (C++20 Trivially Copyable & Standard Layout)
 *
 * 設計原則：
 * - 零動態記憶體配置 (Zero dynamic memory allocation)
 * - 全函式無例外保證 (noexcept guarantee)
 * - 適用於 DMA、環形緩衝區 (Ring Buffer) 及跨執行緒 lock-free 傳輸
 */
struct CANFrame {
    uint32_t id{0};                              ///< CAN Identifier (11-bit 或 29-bit)
    uint8_t dlc{8};                              ///< Data Length Code (固定/標準為 8)
    std::array<uint8_t, 8> data{};               ///< 8-byte Payload 資料緩衝區
    CANIdType id_type{CANIdType::Standard11Bit}; ///< ID 類型
    CANFrameType frame_type{CANFrameType::Data}; ///< 訊框類型

    /**
     * @brief 驗證 DLC 是否符合預期長度 (8 bytes)
     */
    [[nodiscard]] constexpr bool is_valid_dlc() const noexcept {
        return dlc == 8;
    }

    [[nodiscard]] constexpr bool operator==(const CANFrame&) const noexcept = default;
};

// =============================================================================
// Compile-time Standard Compliance & Safety Verifications
// =============================================================================
static_assert(std::is_trivially_copyable_v<CANFrame>, "CANFrame must be trivially copyable for raw memory and DMA safety.");
static_assert(std::is_standard_layout_v<CANFrame>, "CANFrame must satisfy standard layout requirements for deterministic memory representation.");
static_assert(std::is_nothrow_default_constructible_v<CANFrame>, "CANFrame default construction must be noexcept.");
static_assert(std::is_nothrow_copy_constructible_v<CANFrame>, "CANFrame copy construction must be noexcept.");
static_assert(std::is_nothrow_move_constructible_v<CANFrame>, "CANFrame move construction must be noexcept.");
static_assert(std::is_nothrow_copy_assignable_v<CANFrame>, "CANFrame copy assignment must be noexcept.");
static_assert(std::is_nothrow_move_assignable_v<CANFrame>, "CANFrame move assignment must be noexcept.");
static_assert(std::is_nothrow_destructible_v<CANFrame>, "CANFrame destruction must be noexcept.");

} // namespace motor::protocol