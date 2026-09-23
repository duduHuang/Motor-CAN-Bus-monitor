#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace protocol {

/**
 * @brief MIT 模式物理量界限設定檔 (預設相容 Table 1 規範)
 */
struct MITConfig {
    float p_min  = -12.566f;  // rad
    float p_max  =  12.566f;  // rad
    float v_min  = -45.0f;    // rad/s
    float v_max  =  45.0f;    // rad/s
    float kp_min =   0.0f;
    float kp_max = 500.0f;
    float kd_min =   0.0f;
    float kd_max =   5.0f;
    float t_min  = -18.0f;    // Nm
    float t_max  =  18.0f;    // Nm
};

/**
 * @brief MIT 模式下發控制參數 (TX)
 */
struct MITCommand {
    float p_des{0.0f};  // 期望位置 (rad)
    float v_des{0.0f};  // 期望速度 (rad/s)
    float kp{0.0f};     // 位置剛度
    float kd{0.0f};     // 速度阻尼
    float t_ff{0.0f};   // 前饋力矩 (Nm)
};

/**
 * @brief MIT 模式馬達回傳實體物理量 (RX Telemetry)
 */
struct MITTelemetry {
    uint8_t device_can_id{0}; // CAN 裝置 ID (0~255)
    float position_rad{0.0f};  // 當前位置 (rad)
    float velocity_rads{0.0f}; // 當前速度 (rad/s)
    float torque_nm{0.0f};     // 當前力矩 (Nm)
};

/**
 * @brief 核心數值映射與 Bit-Packing 轉換函式
 */

/**
 * @brief 將浮點數線性映射並 Clamp 至指定 Bit 數量的無符號整數 (Compile-time 支援)
 */
[[nodiscard]] constexpr uint32_t float_to_uint(float x, float x_min, float x_max, int bits) noexcept {
    const float clamped_x = std::clamp(x, x_min, x_max);
    const float span = x_max - x_min;
    const float offset = x_min;
    const uint32_t max_int = (1u << bits) - 1u;
    return static_cast<uint32_t>((clamped_x - offset) * static_cast<float>(max_int) / span);
}

/**
 * @brief 將壓碼後的無符號整數還原為實體物理量浮點數 (Compile-time 支援)
 */
[[nodiscard]] constexpr float uint_to_float(uint32_t x_int, float x_min, float x_max, int bits) noexcept {
    const float span = x_max - x_min;
    const float offset = x_min;
    const uint32_t max_int = (1u << bits) - 1u;
    return (static_cast<float>(x_int) * span / static_cast<float>(max_int)) + offset;
}

/**
 * @brief MIT 模式編解碼門面 (完全靜態、無鎖、零動態記憶體分配)
 */
class MITProtocol {
public:
    MITProtocol() = delete; // 純靜態門面，禁止實例化

    /**
     * @brief 將 MIT 控制指令按規範壓碼為 8-byte CAN Payload
     */
    [[nodiscard]] static constexpr std::array<uint8_t, 8> encode(
        const MITCommand& cmd, 
        const MITConfig& cfg = MITConfig{}) noexcept 
    {
        const uint32_t p_int  = float_to_uint(cmd.p_des, cfg.p_min, cfg.p_max, 16);
        const uint32_t v_int  = float_to_uint(cmd.v_des, cfg.v_min, cfg.v_max, 12);
        const uint32_t kp_int = float_to_uint(cmd.kp,    cfg.kp_min, cfg.kp_max, 12);
        const uint32_t kd_int = float_to_uint(cmd.kd,    cfg.kd_min, cfg.kd_max, 12);
        const uint32_t t_int  = float_to_uint(cmd.t_ff,  cfg.t_min,  cfg.t_max,  12);

        std::array<uint8_t, 8> payload{};

        // Byte 0-1: p_des (16-bit)
        payload[0] = static_cast<uint8_t>((p_int >> 8) & 0xFF);
        payload[1] = static_cast<uint8_t>(p_int & 0xFF);

        // Byte 2-3: v_des (12-bit) & kp [11..8] (4-bit)
        payload[2] = static_cast<uint8_t>((v_int >> 4) & 0xFF);
        payload[3] = static_cast<uint8_t>(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));

        // Byte 4: kp [7..0]
        payload[4] = static_cast<uint8_t>(kp_int & 0xFF);

        // Byte 5-6: kd (12-bit) & t_ff [11..8] (4-bit)
        payload[5] = static_cast<uint8_t>((kd_int >> 4) & 0xFF);
        payload[6] = static_cast<uint8_t>(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));

        // Byte 7: t_ff [7..0]
        payload[7] = static_cast<uint8_t>(t_int & 0xFF);

        return payload;
    }

    /**
     * @brief 解碼下發的 8-byte CAN Payload 還原為 MITCommand 指令物件 (TX 反向解析/監控)
     */
    [[nodiscard]] static constexpr MITCommand decode_command(
        const std::array<uint8_t, 8>& payload, 
        const MITConfig& cfg = MITConfig{}) noexcept 
    {
        const uint32_t p_int  = (static_cast<uint32_t>(payload[0]) << 8) | payload[1];
        const uint32_t v_int  = (static_cast<uint32_t>(payload[2]) << 4) | (payload[3] >> 4);
        const uint32_t kp_int = (static_cast<uint32_t>(payload[3] & 0x0F) << 8) | payload[4];
        const uint32_t kd_int = (static_cast<uint32_t>(payload[5]) << 4) | (payload[6] >> 4);
        const uint32_t t_int  = (static_cast<uint32_t>(payload[6] & 0x0F) << 8) | payload[7];

        return MITCommand{
            uint_to_float(p_int,  cfg.p_min,  cfg.p_max,  16),
            uint_to_float(v_int,  cfg.v_min,  cfg.v_max,  12),
            uint_to_float(kp_int, cfg.kp_min, cfg.kp_max, 12),
            uint_to_float(kd_int, cfg.kd_min, cfg.kd_max, 12),
            uint_to_float(t_int,  cfg.t_min,  cfg.t_max,  12)
        };
    }

    /**
     * @brief 解碼馬達回傳的 8-byte CAN Payload 還原為 MITTelemetry 狀態物件 (RX 實體數據)
     */
    [[nodiscard]] static constexpr MITTelemetry decode_telemetry(
        const std::array<uint8_t, 8>& payload, 
        const MITConfig& cfg = MITConfig{}) noexcept 
    {
        const uint8_t can_id = payload[0];

        const uint32_t p_int = (static_cast<uint32_t>(payload[1]) << 8) | payload[2];
        const uint32_t v_int = (static_cast<uint32_t>(payload[3]) << 4) | (payload[4] >> 4);
        const uint32_t t_int = (static_cast<uint32_t>(payload[4] & 0x0F) << 8) | payload[5];

        return MITTelemetry{
            can_id,
            uint_to_float(p_int, cfg.p_min, cfg.p_max, 16),
            uint_to_float(v_int, cfg.v_min, cfg.v_max, 12),
            uint_to_float(t_int, cfg.t_min, cfg.t_max, 12)
        };
    }
};

} // namespace protocol