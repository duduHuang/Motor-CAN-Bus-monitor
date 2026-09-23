#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <type_traits>

namespace servo_robot::protocol {

namespace detail {

template <typename T>
inline constexpr void pack_le(uint8_t* dst, T val) noexcept {
    static_assert(std::is_integral_v<T>, "Integral type required");
    for (size_t i = 0; i < sizeof(T); ++i) {
        dst[i] = static_cast<uint8_t>((val >> (i * 8)) & 0xFF);
    }
}

inline void pack_float_le(uint8_t* dst, float val) noexcept {
    uint32_t raw_bits = 0;
    std::memcpy(&raw_bits, &val, sizeof(float));
    pack_le<uint32_t>(dst, raw_bits);
}

} // namespace detail

/**
 * @brief 單機伺服協定指令編碼器 (Zero-Allocation & Header-Only)
 */
class ServoProtocol {
public:
    static constexpr float TORQUE_SCALE = 0.01f; // 0.01 A / LSB
    static constexpr float SPEED_SCALE  = 0.01f; // 0.01 dps / LSB
    static constexpr float POS_SCALE    = 0.01f; // 0.01 deg / LSB

    // =========================================================================
    // Sheet 3: 電機運動與控制指令 (0x80 ~ 0xA9, 0x72, 0x73)
    // =========================================================================

    /** 0x80: 電機關閉 (Shutdown, 進入無閉環自由狀態) */
    static constexpr std::array<uint8_t, 8> motor_shutdown() noexcept {
        return make_payload(0x80);
    }

    /** 0x81: 電機停止 (Stop, 速度減至 0 並鎖死) */
    static constexpr std::array<uint8_t, 8> motor_stop() noexcept {
        return make_payload(0x81);
    }

    /** 0xA1: 轉矩閉環控制 (0.01A/LSB, int16_t 寫入 DATA[4..5]) */
    static std::array<uint8_t, 8> torque_control(float target_iq_amp) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0xA1);
        auto raw_iq = static_cast<int16_t>(target_iq_amp / TORQUE_SCALE);
        detail::pack_le<int16_t>(&payload[4], raw_iq);
        return payload;
    }

    /** 0xA2: 速度閉環控制 */
    static std::array<uint8_t, 8> speed_control(float target_spd_dps, uint8_t max_torque_pct = 100) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0xA2);
        payload[1] = max_torque_pct;
        auto raw_spd = static_cast<int32_t>(target_spd_dps / SPEED_SCALE);
        detail::pack_le<int32_t>(&payload[4], raw_spd);
        return payload;
    }

    /** 0xA4: 絕對位置閉環控制 */
    static std::array<uint8_t, 8> position_control_abs(float target_pos_deg, uint16_t max_spd_dps = 360) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0xA4);
        detail::pack_le<uint16_t>(&payload[2], max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(&payload[4], raw_pos);
        return payload;
    }

    /** 0xA6: 直驅單圈位置控制 */
    static std::array<uint8_t, 8> position_control_single_turn(float target_deg, uint16_t max_spd_dps = 360, uint8_t spin_dir = 0) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0xA6);
        payload[1] = spin_dir;
        detail::pack_le<uint16_t>(&payload[2], max_spd_dps);
        auto raw_pos = static_cast<uint16_t>(static_cast<int32_t>(target_deg / POS_SCALE) & 0xFFFF);
        detail::pack_le<uint16_t>(&payload[4], raw_pos);
        return payload;
    }

    /** 0xA8: 增量位置閉環控制 */
    static std::array<uint8_t, 8> position_control_inc(float inc_pos_deg, uint16_t max_spd_dps = 360) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0xA8);
        detail::pack_le<uint16_t>(&payload[2], max_spd_dps);
        auto raw_inc = static_cast<int32_t>(inc_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(&payload[4], raw_inc);
        return payload;
    }

    /** 0xA9: 力控位置閉環控制 */
    static std::array<uint8_t, 8> position_control_torque_limit(float target_pos_deg, uint8_t max_torque_pct, uint16_t max_spd_dps) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0xA9);
        payload[1] = max_torque_pct;
        detail::pack_le<uint16_t>(&payload[2], max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(&payload[4], raw_pos);
        return payload;
    }

    /** 0x72: SF 前饋速度位置控制 */
    static std::array<uint8_t, 8> position_control_sf(float target_pos_deg, int8_t ff_speed_pct, uint16_t max_spd_dps) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0x72);
        payload[1] = static_cast<uint8_t>(ff_speed_pct);
        detail::pack_le<uint16_t>(&payload[2], max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(&payload[4], raw_pos);
        return payload;
    }

    /** 0x73: TF 前饋扭矩位置控制 */
    static std::array<uint8_t, 8> position_control_tf(float target_pos_deg, int8_t ff_torque_pct, uint16_t max_spd_dps) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0x73);
        payload[1] = static_cast<uint8_t>(ff_torque_pct);
        detail::pack_le<uint16_t>(&payload[2], max_spd_dps);
        auto raw_pos = static_cast<int32_t>(target_pos_deg / POS_SCALE);
        detail::pack_le<int32_t>(&payload[4], raw_pos);
        return payload;
    }

    // =========================================================================
    // Sheet 2: 電機狀態與資訊讀取指令 (0x60 ~ 0xB5)
    // =========================================================================

    static constexpr std::array<uint8_t, 8> read_encoder_multi_pos() noexcept { return make_payload(0x60); }
    static constexpr std::array<uint8_t, 8> read_raw_encoder_multi_pos() noexcept { return make_payload(0x61); }
    static constexpr std::array<uint8_t, 8> read_multi_turn_angle() noexcept { return make_payload(0x92); }
    static constexpr std::array<uint8_t, 8> read_single_turn_angle() noexcept { return make_payload(0x94); }
    static constexpr std::array<uint8_t, 8> read_status_1() noexcept { return make_payload(0x9A); }
    static constexpr std::array<uint8_t, 8> read_status_2() noexcept { return make_payload(0x9C); }
    static constexpr std::array<uint8_t, 8> read_status_3() noexcept { return make_payload(0x9D); }
    static constexpr std::array<uint8_t, 8> read_system_runtime() noexcept { return make_payload(0xB1); }
    static constexpr std::array<uint8_t, 8> read_software_date() noexcept { return make_payload(0xB2); }

    static std::array<uint8_t, 8> read_motor_model(uint8_t char_index = 0x01) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0xB5);
        payload[1] = 0x01;
        payload[2] = char_index;
        return payload;
    }

    // =========================================================================
    // Sheet 1: 參數讀取與寫入指令 (0x30 ~ 0xB4)
    // =========================================================================

    static std::array<uint8_t, 8> read_pid(uint8_t index) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0x30);
        payload[1] = index;
        return payload;
    }

    static std::array<uint8_t, 8> write_pid_ram(uint8_t index, float value) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0x31);
        payload[1] = index;
        detail::pack_float_le(&payload[4], value);
        return payload;
    }

    static std::array<uint8_t, 8> write_pid_rom(uint8_t index, float value) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0x32);
        payload[1] = index;
        detail::pack_float_le(&payload[4], value);
        return payload;
    }

    static std::array<uint8_t, 8> read_accel(uint8_t index) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0x42);
        payload[1] = index;
        return payload;
    }

    static std::array<uint8_t, 8> write_accel(uint8_t index, int32_t accel_val) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0x43);
        payload[1] = index;
        detail::pack_le<int32_t>(&payload[4], accel_val);
        return payload;
    }

    static constexpr std::array<uint8_t, 8> read_multi_turn_offset() noexcept { return make_payload(0x62); }

    static std::array<uint8_t, 8> write_multi_turn_offset(int32_t offset) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0x63);
        detail::pack_le<int32_t>(&payload[4], offset);
        return payload;
    }

    static constexpr std::array<uint8_t, 8> set_current_pos_as_zero() noexcept { return make_payload(0x64); }

    static std::array<uint8_t, 8> set_comm_timeout(uint32_t timeout_ms) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0xB3);
        detail::pack_le<uint32_t>(&payload[4], timeout_ms);
        return payload;
    }

    static std::array<uint8_t, 8> set_baudrate(uint8_t baudrate_code) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0xB4);
        payload[7] = baudrate_code;
        return payload;
    }

    // =========================================================================
    // Sheet 4: 系統與其他配置指令 (0x70 ~ 0x20)
    // =========================================================================

    static constexpr std::array<uint8_t, 8> get_system_mode() noexcept { return make_payload(0x70); }
    static constexpr std::array<uint8_t, 8> system_reset() noexcept { return make_payload(0x76); }
    static constexpr std::array<uint8_t, 8> release_brake() noexcept { return make_payload(0x77); }
    static constexpr std::array<uint8_t, 8> lock_brake() noexcept { return make_payload(0x78); }

    static std::array<uint8_t, 8> set_active_response(uint8_t target_cmd, bool enable, uint16_t interval_10ms) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0xB6);
        payload[1] = target_cmd;
        payload[2] = enable ? 1 : 0;
        detail::pack_le<uint16_t>(&payload[3], interval_10ms);
        return payload;
    }

    static std::array<uint8_t, 8> composite_function_control(uint8_t index, int32_t value) noexcept {
        std::array<uint8_t, 8> payload = make_payload(0x20);
        payload[1] = index;
        detail::pack_le<int32_t>(&payload[4], value);
        return payload;
    }

private:
    static constexpr std::array<uint8_t, 8> make_payload(uint8_t cmd_code) noexcept {
        return {cmd_code, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    }
};

} // namespace servo_robot::protocol