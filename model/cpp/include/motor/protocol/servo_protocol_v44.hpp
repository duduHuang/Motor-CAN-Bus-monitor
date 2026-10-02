// include/motor/protocol/servo_protocol_v44.hpp
#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace servo_robot::protocol {

namespace detail {

/**
 * @brief 浮點數第二層防禦性健全檢查 (Sanity Check)
 * @details 若控制量輸入為 NaN 或 Inf，強制歸零防護突波控制量下發至伺服馬達驅動器。
 */
[[nodiscard]] constexpr float sanitize_float(float val) noexcept {
    if (std::is_constant_evaluated()) {
        // 編譯期常數求值分支
        if (val != val || val > 3.402823466e+38f || val < -3.402823466e+38f) {
            return 0.0f;
        }
        return val;
    } else {
        // 執行期分支
        if (std::isnan(val) || std::isinf(val)) {
            return 0.0f;
        }
        return val;
    }
}

/**
 * @brief 零開銷的小端序 (Little-Endian) 打包輔助結構 (Trivially Copyable)
 */
struct LittleEndianPacker {
    /**
     * @brief 整數小端序打包 (支援 uint16_t, int16_t, uint32_t, int32_t 等)
     * @tparam T 整數型別
     * @param buf 目標 8-byte 緩衝區
     * @param offset 位元組起始偏移量
     * @param val 寫入之數值
     */
    template <typename T>
    static constexpr void pack(std::array<uint8_t, 8>& buf, size_t offset, T val) noexcept {
        static_assert(std::is_integral_v<T>, "Only integral types are supported.");
        using UnsignedT = std::make_unsigned_t<T>;
        auto uval = static_cast<UnsignedT>(val);
        for (size_t i = 0; i < sizeof(T); ++i) {
            buf[offset + i] = static_cast<uint8_t>((uval >> (i * 8)) & 0xFF);
        }
    }

    /**
     * @brief IEEE 754 浮點數小端序打包 (含 NaN/Inf 防禦檢查)
     * @param buf 目標 8-byte 緩衝區
     * @param offset 位元組起始偏移量
     * @param val 寫入之浮點數值
     */
    static constexpr void pack_float(std::array<uint8_t, 8>& buf, size_t offset, float val) noexcept {
        val = sanitize_float(val);
        uint32_t raw = std::bit_cast<uint32_t>(val);
        pack<uint32_t>(buf, offset, raw);
    }
};

template <typename T>
constexpr void pack_le(std::array<uint8_t, 8>& buf, size_t offset, T val) noexcept {
    LittleEndianPacker::pack<T>(buf, offset, val);
}

constexpr void pack_float_le(std::array<uint8_t, 8>& buf, size_t offset, float val) noexcept {
    LittleEndianPacker::pack_float(buf, offset, val);
}

// 編譯期驗證輔助結構之記憶體特性
static_assert(std::is_trivially_copyable_v<LittleEndianPacker>, "LittleEndianPacker must be trivially copyable.");
static_assert(std::is_standard_layout_v<LittleEndianPacker>, "LittleEndianPacker must be standard layout.");

} // namespace detail

/**
 * @brief MyActuator 伺服馬達 V4.4 協定指令編碼器 (Header-Only, Zero Dynamic Allocation)
 *
 * 設計特性：
 * - C++20 標準相容
 * - 零堆積配置 (Stack only, returning std::array<uint8_t, 8>)
 * - 全函式無例外保證 (noexcept guarantee)
 * - 適用於 1000Hz 硬即時運動控制迴圈
 */
class ServoProtocolV44 {
public:
    static constexpr float TORQUE_SCALE = 0.01f; ///< 0.01 A / LSB
    static constexpr float SPEED_SCALE  = 0.01f; ///< 0.01 dps / LSB
    static constexpr float POS_SCALE    = 0.01f; ///< 0.01 deg / LSB

    // =========================================================================
    // 1. 運動與控制指令 (Motion & Control Commands)
    // =========================================================================

    /**
     * @brief 0x80: 馬達關機指令 (Motor Shutdown)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> motor_shutdown() noexcept {
        return make_payload(0x80);
    }

    /**
     * @brief 0x81: 馬達停止指令 (Motor Stop)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> motor_stop() noexcept {
        return make_payload(0x81);
    }

    /**
     * @brief 0xA1: 轉矩/電流閉環控制 (Torque Control)
     * @param target_iq_amp 目標 Iq 電流 (A)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> torque_control(float target_iq_amp) noexcept {
        target_iq_amp = detail::sanitize_float(target_iq_amp);
        auto payload = make_payload(0xA1);
        auto raw_iq = static_cast<int16_t>(target_iq_amp / TORQUE_SCALE);
        detail::pack_le<int16_t>(payload, 4, raw_iq);
        return payload;
    }

    /**
     * @brief 0xA2: 速度閉環控制 (Speed Control)
     * @param target_spd_dps 目標速度 (deg/s)
     * @param max_torque_pct 最大轉矩百分比限制 (預設 100%)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> speed_control(float target_spd_dps, uint8_t max_torque_pct = 100) noexcept {
        target_spd_dps = detail::sanitize_float(target_spd_dps);
        auto payload = make_payload(0xA2);
        payload[1] = max_torque_pct;
        auto raw_spd = static_cast<int32_t>(target_spd_dps / SPEED_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_spd);
        return payload;
    }

    /**
     * @brief 0xA4: 多圈絕對位置控制 (Multi-Turn Absolute Position Control)
     * @param target_pos_deg 目標角度 (deg)
     * @param max_spd_dps 最大運動轉速限制 (dps，預設 360)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_abs(float target_pos_deg, uint16_t max_spd_dps = 360) noexcept {
        target_pos_deg = detail::sanitize_float(target_pos_deg);
        auto payload = make_payload(0xA4);
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_pos);
        return payload;
    }

    /**
     * @brief 0xA6: 單圈絕對位置控制 (Single-Turn Absolute Position Control)
     * @param target_deg 目標單圈角度 (0 ~ 359.99 deg)
     * @param max_spd_dps 最大運動轉速限制 (dps，預設 360)
     * @param spin_dir 旋轉方向 (0: 順時針/預設)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_single_turn(float target_deg, uint16_t max_spd_dps = 360, uint8_t spin_dir = 0) noexcept {
        target_deg = detail::sanitize_float(target_deg);
        auto payload = make_payload(0xA6);
        payload[1] = spin_dir;
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_pos = static_cast<uint16_t>(static_cast<int32_t>(target_deg / POS_SCALE) & 0xFFFF);
        detail::pack_le<uint16_t>(payload, 4, raw_pos);
        return payload;
    }

    /**
     * @brief 0xA8: 增量位置控制 (Incremental Position Control)
     * @param inc_pos_deg 增量角度 (deg)
     * @param max_spd_dps 最大運動轉速限制 (dps，預設 360)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_inc(float inc_pos_deg, uint16_t max_spd_dps = 360) noexcept {
        inc_pos_deg = detail::sanitize_float(inc_pos_deg);
        auto payload = make_payload(0xA8);
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_inc = static_cast<int32_t>(inc_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_inc);
        return payload;
    }

    /**
     * @brief 0xA9: 帶轉矩限制的位置控制 (Position Control with Torque Limit)
     * @param target_pos_deg 目標角度 (deg)
     * @param max_torque_pct 最大轉矩百分比限制 (0 ~ 100%)
     * @param max_spd_dps 最大轉速 (dps)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_torque_limit(float target_pos_deg, uint8_t max_torque_pct, uint16_t max_spd_dps) noexcept {
        target_pos_deg = detail::sanitize_float(target_pos_deg);
        auto payload = make_payload(0xA9);
        payload[1] = max_torque_pct;
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_pos);
        return payload;
    }

    /**
     * @brief 0x72: 帶速度前饋的位置控制 (Position Control with Speed Feedforward)
     * @param target_pos_deg 目標角度 (deg)
     * @param ff_speed_pct 速度前饋百分比 (-100 ~ 100%)
     * @param max_spd_dps 最大轉速限制 (dps)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_sf(float target_pos_deg, int8_t ff_speed_pct, uint16_t max_spd_dps) noexcept {
        target_pos_deg = detail::sanitize_float(target_pos_deg);
        auto payload = make_payload(0x72);
        payload[1] = static_cast<uint8_t>(ff_speed_pct);
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_pos);
        return payload;
    }

    /**
     * @brief 0x73: 帶轉矩前饋的位置控制 (Position Control with Torque Feedforward)
     * @param target_pos_deg 目標角度 (deg)
     * @param ff_torque_pct 轉矩前饋百分比 (-100 ~ 100%)
     * @param max_spd_dps 最大轉速限制 (dps)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_tf(float target_pos_deg, int8_t ff_torque_pct, uint16_t max_spd_dps) noexcept {
        target_pos_deg = detail::sanitize_float(target_pos_deg);
        auto payload = make_payload(0x73);
        payload[1] = static_cast<uint8_t>(ff_torque_pct);
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_pos);
        return payload;
    }

    // =========================================================================
    // 2. 狀態讀取指令 (Status & Telemetry Read Commands)
    // =========================================================================

    /**
     * @brief 0x9A: 讀取馬達狀態 1 (溫度、電壓、錯誤標誌)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_status_1() noexcept {
        return make_payload(0x9A);
    }

    /**
     * @brief 0x9C: 讀取馬達狀態 2 (溫度、Iq 電流、速度、多圈角度)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_status_2() noexcept {
        return make_payload(0x9C);
    }

    /**
     * @brief 0x9D: 讀取馬達狀態 3 (溫度、三相電流相位)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_status_3() noexcept {
        return make_payload(0x9D);
    }

    /**
     * @brief 0x60: 讀取編碼器多圈位置
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_encoder_multi_pos() noexcept {
        return make_payload(0x60);
    }

    /**
     * @brief 0x61: 讀取原始編碼器多圈位置
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_raw_encoder_multi_pos() noexcept {
        return make_payload(0x61);
    }

    /**
     * @brief 0x92: 讀取多圈角度
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_multi_turn_angle() noexcept {
        return make_payload(0x92);
    }

    /**
     * @brief 0x94: 讀取單圈角度 (Single Turn Angle)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_single_turn_angle() noexcept {
        return make_payload(0x94);
    }

    /**
     * @brief 0xB1: 讀取系統運行時間
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_system_runtime() noexcept {
        return make_payload(0xB1);
    }

    /**
     * @brief 0xB2: 讀取軟體發布日期
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_software_date() noexcept {
        return make_payload(0xB2);
    }

    /**
     * @brief 0xB5: 讀取馬達型號
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_motor_model(uint8_t char_index = 0x01) noexcept {
        auto payload = make_payload(0xB5);
        payload[1] = 0x01;
        payload[2] = char_index;
        return payload;
    }

    // =========================================================================
    // 3. 系統配置與通訊控制指令 (Configuration & Protection Commands)
    // =========================================================================

    /**
     * @brief 0xB3: 通訊中斷保護時間設定 (Timeout Protection)
     * @param timeout_ms 通訊超時保護時間 (ms)，0 代表禁用中斷保護，數值寫入 DATA[4..7]
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> set_comm_timeout(uint32_t timeout_ms) noexcept {
        auto payload = make_payload(0xB3);
        detail::pack_le<uint32_t>(payload, 4, timeout_ms);
        return payload;
    }

    /**
     * @brief 0xB6: 主動定時回覆設定指令 (Active Response Control)
     * @param target_cmd 目標指令碼 (例如 0x9C)
     * @param enable 是否啟用主動回覆
     * @param interval_10ms 回覆週期 (10ms 為單位，例如 10 代表 100ms)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> set_active_response(uint8_t target_cmd, bool enable, uint16_t interval_10ms) noexcept {
        auto payload = make_payload(0xB6);
        payload[1] = target_cmd;
        payload[2] = enable ? 0x01 : 0x00;
        detail::pack_le<uint16_t>(payload, 3, interval_10ms);
        return payload;
    }

    /**
     * @brief 0x20: 複合功能控制指令 (Composite Function Control)
     * @param index 功能索引
     * @param value 設定數值
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> composite_function_control(uint8_t index, int32_t value) noexcept {
        auto payload = make_payload(0x20);
        payload[1] = index;
        detail::pack_le<int32_t>(payload, 4, value);
        return payload;
    }

    /**
     * @brief 0x30: 讀取 PID 參數
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_pid(uint8_t index) noexcept {
        auto payload = make_payload(0x30);
        payload[1] = index;
        return payload;
    }

    /**
     * @brief 0x31: 寫入 PID 參數至 RAM
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> write_pid_ram(uint8_t index, float value) noexcept {
        auto payload = make_payload(0x31);
        payload[1] = index;
        detail::pack_float_le(payload, 4, value);
        return payload;
    }

    /**
     * @brief 0x32: 寫入 PID 參數至 ROM
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> write_pid_rom(uint8_t index, float value) noexcept {
        auto payload = make_payload(0x32);
        payload[1] = index;
        detail::pack_float_le(payload, 4, value);
        return payload;
    }

    /**
     * @brief 0x42: 讀取加速度參數
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_accel(uint8_t index) noexcept {
        auto payload = make_payload(0x42);
        payload[1] = index;
        return payload;
    }

    /**
     * @brief 0x43: 寫入加速度參數至 RAM
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> write_accel(uint8_t index, int32_t accel_val) noexcept {
        auto payload = make_payload(0x43);
        payload[1] = index;
        detail::pack_le<int32_t>(payload, 4, accel_val);
        return payload;
    }

    /**
     * @brief 0x62: 讀取多圈偏移量
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_multi_turn_offset() noexcept {
        return make_payload(0x62);
    }

    /**
     * @brief 0x63: 寫入多圈偏移量至 ROM
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> write_multi_turn_offset(int32_t offset) noexcept {
        auto payload = make_payload(0x63);
        detail::pack_le<int32_t>(payload, 4, offset);
        return payload;
    }

    /**
     * @brief 0x64: 將當前位置設定為機械零點 (Set Current Position as Zero)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> set_current_pos_as_zero() noexcept {
        return make_payload(0x64);
    }

    /**
     * @brief 0xB4: 設定通訊波特率
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> set_baudrate(uint8_t baudrate_code) noexcept {
        auto payload = make_payload(0xB4);
        payload[7] = baudrate_code;
        return payload;
    }

    /**
     * @brief 0x70: 讀取系統運作模式
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> get_system_mode() noexcept {
        return make_payload(0x70);
    }

    /**
     * @brief 0x76: 系統重置 (System Reset)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> system_reset() noexcept {
        return make_payload(0x76);
    }

    /**
     * @brief 0x77: 煞車釋放 (Release Brake)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> release_brake() noexcept {
        return make_payload(0x77);
    }

    /**
     * @brief 0x78: 煞車鎖定 (Lock Brake)
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> lock_brake() noexcept {
        return make_payload(0x78);
    }

private:
    [[nodiscard]] static constexpr std::array<uint8_t, 8> make_payload(uint8_t cmd_code) noexcept {
        return {cmd_code, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    }
};

// =============================================================================
// Compile-time Standard Compliance Verifications
// =============================================================================
static_assert(std::is_trivially_copyable_v<ServoProtocolV44>, "ServoProtocolV44 must be trivially copyable.");
static_assert(std::is_standard_layout_v<ServoProtocolV44>, "ServoProtocolV44 must be standard layout.");

#if !defined(SERVO_ROBOT_NO_SERVO_PROTOCOL_ALIAS)
using ServoProtocol = ServoProtocolV44;
#endif

} // namespace servo_robot::protocol