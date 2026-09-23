#pragma once

#include <array>
#include <cstdint>
#include <bit>
#include <cmath>
#include <type_traits>

namespace servo_robot::protocol {

namespace detail {

/**
 * @brief 零開銷的小端序 (Little-Endian) 整數打包器
 */
template <typename T>
constexpr void pack_le(std::array<uint8_t, 8>& buf, size_t offset, T val) noexcept {
    static_assert(std::is_integral_v<T>, "Only integral types are supported.");
    for (size_t i = 0; i < sizeof(T); ++i) {
        buf[offset + i] = static_cast<uint8_t>((val >> (i * 8)) & 0xFF);
    }
}

/**
 * @brief 零開銷的小端序 IEEE 754 浮點數打包器 (含第二層 Sanity Guard)
 */
constexpr void pack_float_le(std::array<uint8_t, 8>& buf, size_t offset, float val) noexcept {
    // 第二層防禦：若控制量輸入為 NaN 或 Inf，打入 0.0f 防止突波控制量下發至馬達
    if (std::isnan(val) || std::isinf(val)) {
        val = 0.0f;
    }
    uint32_t raw = std::bit_cast<uint32_t>(val);
    pack_le<uint32_t>(buf, offset, raw);
}

} // namespace detail

/**
 * @brief 單機伺服協定指令編碼器 (Header-Only, Zero-Allocation)
 * 適用於 1000Hz 即時控制迴圈，所有 payload 生成均在 Stack 上完成。
 */
class ServoProtocol {
public:
    static constexpr float TORQUE_SCALE = 0.01f; // 0.01 A / LSB
    static constexpr float SPEED_SCALE  = 0.01f; // 0.01 dps / LSB
    static constexpr float POS_SCALE    = 0.01f; // 0.01 deg / LSB

    // =========================================================================
    // Sheet 3: 運動與控制指令 (0x80 ~ 0xA9, 0x72, 0x73)
    // =========================================================================

    [[nodiscard]] static constexpr std::array<uint8_t, 8> motor_shutdown() noexcept {
        return make_payload(0x80);
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> motor_stop() noexcept {
        return make_payload(0x81);
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> torque_control(float target_iq_amp) noexcept {
        if (std::isnan(target_iq_amp) || std::isinf(target_iq_amp)) target_iq_amp = 0.0f;
        auto payload = make_payload(0xA1);
        auto raw_iq = static_cast<int16_t>(target_iq_amp / TORQUE_SCALE);
        detail::pack_le<int16_t>(payload, 4, raw_iq);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> speed_control(float target_spd_dps, uint8_t max_torque_pct = 100) noexcept {
        if (std::isnan(target_spd_dps) || std::isinf(target_spd_dps)) target_spd_dps = 0.0f;
        auto payload = make_payload(0xA2);
        payload[1] = max_torque_pct;
        auto raw_spd = static_cast<int32_t>(target_spd_dps / SPEED_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_spd);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_abs(float target_pos_deg, uint16_t max_spd_dps = 360) noexcept {
        if (std::isnan(target_pos_deg) || std::isinf(target_pos_deg)) target_pos_deg = 0.0f;
        auto payload = make_payload(0xA4);
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_pos);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_single_turn(float target_deg, uint16_t max_spd_dps = 360, uint8_t spin_dir = 0) noexcept {
        if (std::isnan(target_deg) || std::isinf(target_deg)) target_deg = 0.0f;
        auto payload = make_payload(0xA6);
        payload[1] = spin_dir;
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_pos = static_cast<uint16_t>(static_cast<int32_t>(target_deg / POS_SCALE) & 0xFFFF);
        detail::pack_le<uint16_t>(payload, 4, raw_pos);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_inc(float inc_pos_deg, uint16_t max_spd_dps = 360) noexcept {
        if (std::isnan(inc_pos_deg) || std::isinf(inc_pos_deg)) inc_pos_deg = 0.0f;
        auto payload = make_payload(0xA8);
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_inc = static_cast<int32_t>(inc_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_inc);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_torque_limit(float target_pos_deg, uint8_t max_torque_pct, uint16_t max_spd_dps) noexcept {
        if (std::isnan(target_pos_deg) || std::isinf(target_pos_deg)) target_pos_deg = 0.0f;
        auto payload = make_payload(0xA9);
        payload[1] = max_torque_pct;
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_pos);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_sf(float target_pos_deg, int8_t ff_speed_pct, uint16_t max_spd_dps) noexcept {
        if (std::isnan(target_pos_deg) || std::isinf(target_pos_deg)) target_pos_deg = 0.0f;
        auto payload = make_payload(0x72);
        payload[1] = static_cast<uint8_t>(ff_speed_pct);
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_pos);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> position_control_tf(float target_pos_deg, int8_t ff_torque_pct, uint16_t max_spd_dps) noexcept {
        if (std::isnan(target_pos_deg) || std::isinf(target_pos_deg)) target_pos_deg = 0.0f;
        auto payload = make_payload(0x73);
        payload[1] = static_cast<uint8_t>(ff_torque_pct);
        detail::pack_le<uint16_t>(payload, 2, max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(payload, 4, raw_pos);
        return payload;
    }

    // =========================================================================
    // Sheet 2: 狀態讀取指令 (0x60 ~ 0xB5)
    // =========================================================================

    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_encoder_multi_pos() noexcept { return make_payload(0x60); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_raw_encoder_multi_pos() noexcept { return make_payload(0x61); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_multi_turn_angle() noexcept { return make_payload(0x92); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_single_turn_angle() noexcept { return make_payload(0x94); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_status_1() noexcept { return make_payload(0x9A); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_status_2() noexcept { return make_payload(0x9C); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_status_3() noexcept { return make_payload(0x9D); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_system_runtime() noexcept { return make_payload(0xB1); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_software_date() noexcept { return make_payload(0xB2); }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_motor_model(uint8_t char_index = 0x01) noexcept {
        auto payload = make_payload(0xB5);
        payload[1] = 0x01;
        payload[2] = char_index;
        return payload;
    }

    // =========================================================================
    // Sheet 1: 參數設定與讀取指令 (0x30 ~ 0xB4)
    // =========================================================================

    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_pid(uint8_t index) noexcept {
        auto payload = make_payload(0x30);
        payload[1] = index;
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> write_pid_ram(uint8_t index, float value) noexcept {
        auto payload = make_payload(0x31);
        payload[1] = index;
        detail::pack_float_le(payload, 4, value);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> write_pid_rom(uint8_t index, float value) noexcept {
        auto payload = make_payload(0x32);
        payload[1] = index;
        detail::pack_float_le(payload, 4, value);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_accel(uint8_t index) noexcept {
        auto payload = make_payload(0x42);
        payload[1] = index;
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> write_accel(uint8_t index, int32_t accel_val) noexcept {
        auto payload = make_payload(0x43);
        payload[1] = index;
        detail::pack_le<int32_t>(payload, 4, accel_val);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> read_multi_turn_offset() noexcept { return make_payload(0x62); }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> write_multi_turn_offset(int32_t offset) noexcept {
        auto payload = make_payload(0x63);
        detail::pack_le<int32_t>(payload, 4, offset);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> set_current_pos_as_zero() noexcept { return make_payload(0x64); }

    /**
     * @brief 0xB3: 通訊中斷保護時間設置指令 (Fourth-Layer Defense)
     * @param timeout_ms 通訊超時保護時間 (ms)，0 代表禁用中斷保護
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> set_comm_timeout(uint32_t timeout_ms) noexcept {
        auto payload = make_payload(0xB3);
        detail::pack_le<uint32_t>(payload, 4, timeout_ms);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> set_baudrate(uint8_t baudrate_code) noexcept {
        auto payload = make_payload(0xB4);
        payload[7] = baudrate_code;
        return payload;
    }

    // =========================================================================
    // Sheet 4: 系統配置與主動回覆指令 (0x70 ~ 0xB6)
    // =========================================================================

    [[nodiscard]] static constexpr std::array<uint8_t, 8> get_system_mode() noexcept { return make_payload(0x70); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> system_reset() noexcept { return make_payload(0x76); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> release_brake() noexcept { return make_payload(0x77); }
    [[nodiscard]] static constexpr std::array<uint8_t, 8> lock_brake() noexcept { return make_payload(0x78); }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> set_active_response(uint8_t target_cmd, bool enable, uint16_t interval_10ms) noexcept {
        auto payload = make_payload(0xB6);
        payload[1] = target_cmd;
        payload[2] = enable ? 0x01 : 0x00;
        detail::pack_le<uint16_t>(payload, 3, interval_10ms);
        return payload;
    }

    [[nodiscard]] static constexpr std::array<uint8_t, 8> composite_function_control(uint8_t index, int32_t value) noexcept {
        auto payload = make_payload(0x20);
        payload[1] = index;
        detail::pack_le<int32_t>(payload, 4, value);
        return payload;
    }

private:
    [[nodiscard]] static constexpr std::array<uint8_t, 8> make_payload(uint8_t cmd_code) noexcept {
        return {cmd_code, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    }
};

} // namespace servo_robot::protocol