#pragma once

#include <array>
#include <cstdint>
#include <variant>

namespace servo_robot::protocol {

/**
 * @brief 0x9A 錯誤狀態字 (Bitmask 封裝，Zero-Allocation)
 */
struct ErrorStatusFlags {
    uint16_t raw_value{0};

    constexpr bool stall() const noexcept { return (raw_value & 0x0002) != 0; }
    constexpr bool low_voltage() const noexcept { return (raw_value & 0x0004) != 0; }
    constexpr bool over_voltage() const noexcept { return (raw_value & 0x0008) != 0; }
    constexpr bool over_current() const noexcept { return (raw_value & 0x0010) != 0; }
    constexpr bool mos_over_temp() const noexcept { return (raw_value & 0x0080) != 0; }
    constexpr bool motor_over_temp() const noexcept { return (raw_value & 0x1000) != 0; }
    constexpr bool encoder_calib_error() const noexcept { return (raw_value & 0x2000) != 0; }

    static constexpr ErrorStatusFlags from_uint16(uint16_t val) noexcept {
        return ErrorStatusFlags{val};
    }
};

enum class SystemMode : uint8_t {
    UNKNOWN       = 0,
    CURRENT_LOOP  = 1,
    SPEED_LOOP    = 2,
    POSITION_LOOP = 3
};

struct UnknownTelemetry {
    uint8_t cmd_echo{0};
    std::array<uint8_t, 8> raw_payload{};
};

struct StandardMotionTelemetry {
    uint8_t cmd_echo{0};
    int8_t  temperature_c{0};
    float   iq_current_amp{0.0f};
    float   speed_dps{0.0f};
    int16_t angle_deg{0};
};

struct SingleTurnMotionTelemetry {
    uint8_t  cmd_echo{0};
    int8_t   temperature_c{0};
    float    iq_current_amp{0.0f};
    float    speed_dps{0.0f};
    uint16_t encoder_raw{0};
};

struct SensorStatus1Telemetry {
    uint8_t          cmd_echo{0};
    int8_t           temperature_c{0};
    int8_t           mos_temperature_c{0};
    bool             brake_released{false};
    float            voltage_v{0.0f};
    ErrorStatusFlags error_flags{};
};

struct SensorStatus3Telemetry {
    uint8_t cmd_echo{0};
    int8_t  temperature_c{0};
    float   phase_a_amp{0.0f};
    float   phase_b_amp{0.0f};
    float   phase_c_amp{0.0f};
};

struct PIDQueryTelemetry {
    uint8_t cmd_echo{0};
    uint8_t param_index{0};
    float   value{0.0f};
};

struct AccelQueryTelemetry {
    uint8_t cmd_echo{0};
    uint8_t func_index{0};
    int32_t accel_dps2{0};
};

struct EncoderPosTelemetry {
    uint8_t cmd_echo{0};
    int32_t encoder_pos{0};
};

struct ZeroOffsetTelemetry {
    uint8_t cmd_echo{0};
    int32_t encoder_offset{0};
};

struct AngleQueryTelemetry {
    uint8_t cmd_echo{0};
    float   angle_deg{0.0f};
};

struct SystemModeTelemetry {
    uint8_t    cmd_echo{0};
    uint8_t    mode_code{0};
    SystemMode mode{SystemMode::UNKNOWN};
};

struct SystemInfoTelemetry {
    uint8_t  cmd_echo{0};
    uint32_t value{0};
};

struct MotorModelTelemetry {
    uint8_t               cmd_echo{0};
    uint8_t               start_index{0};
    std::array<char, 5>   model_chars{};
};

struct WriteAckTelemetry {
    uint8_t                cmd_echo{0};
    bool                   is_success{true};
    std::array<uint8_t, 8> raw_payload{};
};

using ServoTelemetry = std::variant<
    UnknownTelemetry,
    StandardMotionTelemetry,
    SingleTurnMotionTelemetry,
    SensorStatus1Telemetry,
    SensorStatus3Telemetry,
    PIDQueryTelemetry,
    AccelQueryTelemetry,
    EncoderPosTelemetry,
    ZeroOffsetTelemetry,
    AngleQueryTelemetry,
    SystemModeTelemetry,
    SystemInfoTelemetry,
    MotorModelTelemetry,
    WriteAckTelemetry
>;

/**
 * @brief 零分配/零阻塞 Telemetry 解碼器
 */
class ServoDecoder {
public:
    /**
     * @brief 解碼 8-Byte Payload 為對應 Telemetry 變體
     * @note $O(1)$ 時間複雜度，noexcept 且無 dynamic allocation
     */
    static ServoTelemetry decode_any(const std::array<uint8_t, 8>& payload) noexcept;
};

} // namespace servo_robot::protocol