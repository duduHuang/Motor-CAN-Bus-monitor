#include "protocol/mit_protocol.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>

using namespace protocol;

// 驗證浮點數在量化範圍內的容許誤差
constexpr float EPSILON_P  = 0.001f; // 16-bit 精度 (span / 65535)
constexpr float EPSILON_12 = 0.025f; // 12-bit 精度 (span / 4095)

void test_compile_time_constexpr() {
    constexpr MITConfig cfg;
    constexpr MITCommand cmd{1.0f, 0.5f, 100.0f, 2.0f, 0.0f};
    constexpr auto payload = MITProtocol::encode(cmd, cfg);
    constexpr auto decoded = MITProtocol::decode_command(payload, cfg);
    
    static_assert(payload.size() == 8, "Payload size must be exactly 8 bytes");
    static_assert(std::abs(decoded.p_des - 1.0f) < 0.01f, "Compile-time decoding failed");
    std::cout << "[PASS] Compile-time constexpr test.\n";
}

void test_extremes_and_clamping() {
    MITConfig cfg;
    
    // 超出界限數值
    MITCommand overflow_cmd{
        100.0f,  // > p_max (12.566)
        -100.0f, // < v_min (-45.0)
        1000.0f, // > kp_max (500.0)
        -10.0f,  // < kd_min (0.0)
        50.0f    // > t_max (18.0)
    };

    auto payload = MITProtocol::encode(overflow_cmd, cfg);
    auto decoded = MITProtocol::decode_command(payload, cfg);

    // 驗證是否被正確 Clamp 至邊界
    assert(std::abs(decoded.p_des - cfg.p_max) < EPSILON_P);
    assert(std::abs(decoded.v_des - cfg.v_min) < EPSILON_12);
    assert(std::abs(decoded.kp - cfg.kp_max) < EPSILON_12);
    assert(std::abs(decoded.kd - cfg.kd_min) < EPSILON_12);
    assert(std::abs(decoded.t_ff - cfg.t_max) < EPSILON_12);

    std::cout << "[PASS] Extremes & Clamping test.\n";
}

void test_roundtrip_precision() {
    MITConfig cfg;
    MITCommand original_cmd{
        3.14159f,   // p_des
        -12.5f,     // v_des
        250.0f,     // kp
        1.25f,      // kd
        -5.5f       // t_ff
    };

    auto payload = MITProtocol::encode(original_cmd, cfg);
    auto decoded = MITProtocol::decode_command(payload, cfg);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "--- Roundtrip Precision Verification ---\n";
    std::cout << "p_des : Orig=" << original_cmd.p_des << " -> Dec=" << decoded.p_des << " (Diff: " << std::abs(original_cmd.p_des - decoded.p_des) << ")\n";
    std::cout << "v_des : Orig=" << original_cmd.v_des << " -> Dec=" << decoded.v_des << " (Diff: " << std::abs(original_cmd.v_des - decoded.v_des) << ")\n";
    std::cout << "kp    : Orig=" << original_cmd.kp    << " -> Dec=" << decoded.kp    << " (Diff: " << std::abs(original_cmd.kp - decoded.kp) << ")\n";
    std::cout << "kd    : Orig=" << original_cmd.kd    << " -> Dec=" << decoded.kd    << " (Diff: " << std::abs(original_cmd.kd - decoded.kd) << ")\n";
    std::cout << "t_ff  : Orig=" << original_cmd.t_ff  << " -> Dec=" << decoded.t_ff  << " (Diff: " << std::abs(original_cmd.t_ff - decoded.t_ff) << ")\n";

    assert(std::abs(original_cmd.p_des - decoded.p_des) <= EPSILON_P);
    assert(std::abs(original_cmd.v_des - decoded.v_des) <= EPSILON_12);
    assert(std::abs(original_cmd.kp - decoded.kp)       <= EPSILON_12);
    assert(std::abs(original_cmd.kd - decoded.kd)       <= EPSILON_12);
    assert(std::abs(original_cmd.t_ff - decoded.t_ff)   <= EPSILON_12);

    std::cout << "[PASS] Roundtrip Precision test.\n";
}

void test_telemetry_decoding() {
    MITConfig cfg;
    
    // 模擬馬達回傳 Payload (CAN ID: 0x01, Pos: 0.0 rad, Vel: 0.0 rad/s, Torque: 0.0 Nm)
    // 0.0 代表在中間值，對於 16-bit 相當於 32767，對於 12-bit 相當於 2047
    uint32_t p_mid = float_to_uint(0.0f, cfg.p_min, cfg.p_max, 16);
    uint32_t v_mid = float_to_uint(0.0f, cfg.v_min, cfg.v_max, 12);
    uint32_t t_mid = float_to_uint(0.0f, cfg.t_min, cfg.t_max, 12);

    std::array<uint8_t, 8> rx_payload{};
    rx_payload[0] = 0x01; // CAN ID
    rx_payload[1] = static_cast<uint8_t>((p_mid >> 8) & 0xFF);
    rx_payload[2] = static_cast<uint8_t>(p_mid & 0xFF);
    rx_payload[3] = static_cast<uint8_t>((v_mid >> 4) & 0xFF);
    rx_payload[4] = static_cast<uint8_t>(((v_mid & 0x0F) << 4) | ((t_mid >> 8) & 0x0F));
    rx_payload[5] = static_cast<uint8_t>(t_mid & 0xFF);

    MITTelemetry telemetry = MITProtocol::decode_telemetry(rx_payload, cfg);

    assert(telemetry.device_can_id == 0x01);
    assert(std::abs(telemetry.position_rad - 0.0f) < EPSILON_P);
    assert(std::abs(telemetry.velocity_rads - 0.0f) < EPSILON_12);
    assert(std::abs(telemetry.torque_nm - 0.0f) < EPSILON_12);

    std::cout << "[PASS] Telemetry decoding test.\n";
}

int main() {
    std::cout << "Starting MIT Protocol Unit Tests...\n";
    test_compile_time_constexpr();
    test_extremes_and_clamping();
    test_roundtrip_precision();
    test_telemetry_decoding();
    std::cout << "All Unit Tests Passed Successfully!\n";
    return 0;
}