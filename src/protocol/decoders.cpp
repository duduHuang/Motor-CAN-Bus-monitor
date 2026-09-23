#include "protocol/decoders.hpp"
#include <cstring>
#include <type_traits>

namespace servo_robot::protocol {

namespace {

template <typename T>
inline constexpr T unpack_le(const uint8_t* src) noexcept {
    static_assert(std::is_integral_v<T>, "Integral type required");
    T val = 0;
    for (size_t i = 0; i < sizeof(T); ++i) {
        val |= static_cast<T>(src[i]) << (i * 8);
    }
    return val;
}

inline float unpack_float_le(const uint8_t* src) noexcept {
    uint32_t raw_bits = unpack_le<uint32_t>(src);
    float f = 0.0f;
    std::memcpy(&f, &raw_bits, sizeof(float));
    return f;
}

} // namespace

ServoTelemetry ServoDecoder::decode_any(const std::array<uint8_t, 8>& payload) noexcept {
    const uint8_t cmd_echo = payload[0];

    switch (cmd_echo) {
        // Standard Motion Telemetry
        case 0x9C: case 0xA1: case 0xA2: case 0xA4:
        case 0xA8: case 0xA9: case 0x72: case 0x73: {
            StandardMotionTelemetry t;
            t.cmd_echo       = cmd_echo;
            t.temperature_c  = static_cast<int8_t>(payload[1]);
            t.iq_current_amp = static_cast<float>(unpack_le<int16_t>(&payload[2])) * 0.01f;
            t.speed_dps      = static_cast<float>(unpack_le<int16_t>(&payload[4])) * 1.0f;
            t.angle_deg      = unpack_le<int16_t>(&payload[6]);
            return t;
        }

        // Single Turn Motion Telemetry
        case 0xA6: {
            SingleTurnMotionTelemetry t;
            t.cmd_echo       = cmd_echo;
            t.temperature_c  = static_cast<int8_t>(payload[1]);
            t.iq_current_amp = static_cast<float>(unpack_le<int16_t>(&payload[2])) * 0.01f;
            t.speed_dps      = static_cast<float>(unpack_le<int16_t>(&payload[4])) * 1.0f;
            t.encoder_raw    = unpack_le<uint16_t>(&payload[6]);
            return t;
        }

        // Sensor Status 1 Telemetry
        case 0x9A: {
            SensorStatus1Telemetry t;
            t.cmd_echo          = cmd_echo;
            t.temperature_c     = static_cast<int8_t>(payload[1]);
            t.mos_temperature_c = static_cast<int8_t>(payload[2]);
            t.brake_released    = (payload[3] == 0x01);
            t.voltage_v         = static_cast<float>(unpack_le<uint16_t>(&payload[4])) * 0.1f;
            t.error_flags       = ErrorStatusFlags::from_uint16(unpack_le<uint16_t>(&payload[6]));
            return t;
        }

        // Sensor Status 3 Telemetry
        case 0x9D: {
            SensorStatus3Telemetry t;
            t.cmd_echo      = cmd_echo;
            t.temperature_c = static_cast<int8_t>(payload[1]);
            t.phase_a_amp   = static_cast<float>(unpack_le<int16_t>(&payload[2])) * 0.01f;
            t.phase_b_amp   = static_cast<float>(unpack_le<int16_t>(&payload[4])) * 0.01f;
            t.phase_c_amp   = static_cast<float>(unpack_le<int16_t>(&payload[6])) * 0.01f;
            return t;
        }

        // PID Query Telemetry
        case 0x30: {
            PIDQueryTelemetry t;
            t.cmd_echo    = cmd_echo;
            t.param_index = payload[1];
            t.value       = unpack_float_le(&payload[4]);
            return t;
        }

        // Accel Query Telemetry
        case 0x42: {
            AccelQueryTelemetry t;
            t.cmd_echo   = cmd_echo;
            t.func_index = payload[1];
            t.accel_dps2 = unpack_le<int32_t>(&payload[4]);
            return t;
        }

        // Encoder Position Telemetry
        case 0x60: case 0x61: {
            EncoderPosTelemetry t;
            t.cmd_echo    = cmd_echo;
            t.encoder_pos = unpack_le<int32_t>(&payload[4]);
            return t;
        }

        // Zero Offset Telemetry
        case 0x62: case 0x64: {
            ZeroOffsetTelemetry t;
            t.cmd_echo       = cmd_echo;
            t.encoder_offset = unpack_le<int32_t>(&payload[4]);
            return t;
        }

        // Angle Query Telemetry
        case 0x92: case 0x94: {
            AngleQueryTelemetry t;
            t.cmd_echo  = cmd_echo;
            t.angle_deg = static_cast<float>(unpack_le<int32_t>(&payload[4])) * 0.01f;
            return t;
        }

        // System Mode Telemetry
        case 0x70: {
            SystemModeTelemetry t;
            t.cmd_echo  = cmd_echo;
            t.mode_code = payload[7];
            switch (t.mode_code) {
                case 1: t.mode = SystemMode::CURRENT_LOOP; break;
                case 2: t.mode = SystemMode::SPEED_LOOP; break;
                case 3: t.mode = SystemMode::POSITION_LOOP; break;
                default: t.mode = SystemMode::UNKNOWN; break;
            }
            return t;
        }

        // System Info Telemetry
        case 0xB1: case 0xB2: {
            SystemInfoTelemetry t;
            t.cmd_echo = cmd_echo;
            t.value    = unpack_le<uint32_t>(&payload[4]);
            return t;
        }

        // Motor Model Telemetry
        case 0xB5: {
            MotorModelTelemetry t;
            t.cmd_echo    = cmd_echo;
            t.start_index = payload[2];
            std::memcpy(t.model_chars.data(), &payload[3], 5);
            return t;
        }

        // Write Ack Telemetry
        case 0x31: case 0x32: case 0x43: case 0x63:
        case 0xB3: case 0x20: case 0x77: case 0x78:
        case 0x80: case 0x81: {
            WriteAckTelemetry t;
            t.cmd_echo    = cmd_echo;
            t.is_success  = true;
            t.raw_payload = payload;
            return t;
        }

        default: {
            UnknownTelemetry t;
            t.cmd_echo    = cmd_echo;
            t.raw_payload = payload;
            return t;
        }
    }
}

} // namespace servo_robot::protocol