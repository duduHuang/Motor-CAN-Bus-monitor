#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace protocol {

/**
 * @brief MIT 模式物理量界限設定檔
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
 * @brief MIT 控制指令 (TX)
 */
struct MITCommand {
    float p_des{0.0f};
    float v_des{0.0f};
    float kp{0.0f};
    float kd{0.0f};
    float t_ff{0.0f};
};

/**
 * @brief MIT 馬達回傳實體數據 (RX)
 */
struct MITTelemetry {
    uint8_t device_can_id{0};
    float position_rad{0.0f};
    float velocity_rads{0.0f};
    float torque_nm{0.0f};
};

/**
 * @brief 核心數值映射與 Bit-Packing 轉換
 */
[[nodiscard]] constexpr uint32_t float_to_uint(float x, float x_min, float x_max, int bits) noexcept {
    const float clamped_x = std::clamp(x, x_min, x_max);
    const float span = x_max - x_min;
    const float offset = x_min;
    const uint32_t max_int = (1u << bits) - 1u;
    return static_cast<uint32_t>((clamped_x - offset) * static_cast<float>(max_int) / span);
}

[[nodiscard]] constexpr float uint_to_float(uint32_t x_int, float x_min, float x_max, int bits) noexcept {
    const float span = x_max - x_min;
    const float offset = x_min;
    const uint32_t max_int = (1u << bits) - 1u;
    return (static_cast<float>(x_int) * span / static_cast<float>(max_int)) + offset;
}

/**
 * @brief MITProtocol 編解碼門面 (包含第二層 Protocol Syntactic Filter)
 */
class MITProtocol {
public:
    MITProtocol() = delete;

    /**
     * @brief [第二層防禦] 伺服指令回應 Echo (0x240 區段) DATA[0] 合法性檢查
     */
    [[nodiscard]] static constexpr bool is_valid_servo_echo(uint8_t echo_cmd) noexcept {
        switch (echo_cmd) {
            case 0x20: // 功能控制回應
            case 0x9A: // 讀取 PID / 馬達狀態
            case 0x9C: // 讀取關節編碼器物理位置
            case 0xA1: // 轉矩閉環控制回應
            case 0xA2: // 速度閉環控制回應
            case 0xA4: // 位置閉環控制回應
            case 0xA9: // 力控位置閉環回應
            case 0xB1: // 系統控制指令回應
            case 0xB2: // 軟體版本回應
            case 0xB3: // 通訊中斷保護回應
            case 0xB5: // 讀取馬達型號回應
                return true;
            default:
                return false; // 非法/未定義的 Command Echo，認定為 CAN 雜訊
        }
    }

    /**
     * @brief TX 壓碼 (無鎖、Zero-Allocation)
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
        payload[0] = static_cast<uint8_t>((p_int >> 8) & 0xFF);
        payload[1] = static_cast<uint8_t>(p_int & 0xFF);
        payload[2] = static_cast<uint8_t>((v_int >> 4) & 0xFF);
        payload[3] = static_cast<uint8_t>(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
        payload[4] = static_cast<uint8_t>(kp_int & 0xFF);
        payload[5] = static_cast<uint8_t>((kd_int >> 4) & 0xFF);
        payload[6] = static_cast<uint8_t>(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
        payload[7] = static_cast<uint8_t>(t_int & 0xFF);

        return payload;
    }

    /**
     * @brief [第二層防禦] RX Telemetry 解碼與完整語法/數值過濾器
     */
    [[nodiscard]] static std::optional<MITTelemetry> decode_telemetry(
        const std::array<uint8_t, 8>& payload, 
        const MITConfig& cfg = MITConfig{}) noexcept 
    {
        // 防禦 2.1: Reserved Bytes 位元遮罩比對 (MIT 回傳 Protocol 規定 DATA[6] 與 DATA[7] 必須為 0x00)
        if (payload[6] != 0x00 || payload[7] != 0x00) {
            return std::nullopt; // 位元損壞/非 MIT 回傳格式，直接丟棄
        }

        // 防禦 2.2: Device CAN ID 邊界檢查 (例如 CAN ID 不可為 0 或超過合理範圍)
        const uint8_t can_id = payload[0];
        if (can_id == 0x00) {
            return std::nullopt; // 無效裝置 ID 廣播或雜訊
        }

        // 解碼 Bit-Packing
        const uint32_t p_int = (static_cast<uint32_t>(payload[1]) << 8) | payload[2];
        const uint32_t v_int = (static_cast<uint32_t>(payload[3]) << 4) | (payload[4] >> 4);
        const uint32_t t_int = (static_cast<uint32_t>(payload[4] & 0x0F) << 8) | payload[5];

        // 轉換為浮點數
        const float pos = uint_to_float(p_int, cfg.p_min, cfg.p_max, 16);
        const float vel = uint_to_float(v_int, cfg.v_min, cfg.v_max, 12);
        const float trq = uint_to_float(t_int, cfg.t_min, cfg.t_max, 12);

        // 防禦 2.3: NaN / Inf 數值合法性過濾
        if (std::isnan(pos) || std::isinf(pos) ||
            std::isnan(vel) || std::isinf(vel) ||
            std::isnan(trq) || std::isinf(trq)) 
        {
            return std::nullopt;
        }

        // 防禦 2.4: 物理硬邊界 Sanity Check (防止 cfg 傳入極端值導致還原數據超越物理邊界)
        if (pos < cfg.p_min || pos > cfg.p_max ||
            vel < cfg.v_min || vel > cfg.v_max ||
            trq < cfg.t_min || trq > cfg.t_max) 
        {
            return std::nullopt; // 數值超越物理邊界，認定為 Outlier Spike
        }

        return MITTelemetry{can_id, pos, vel, trq};
    }
};

} // namespace protocol