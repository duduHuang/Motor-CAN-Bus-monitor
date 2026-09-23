#include "protocol/servo_protocol.hpp"
#include "protocol/decoders.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace servo_robot::protocol;

void test_encoders() {
    std::cout << "[TEST] Running Encoder Tests..." << std::endl;

    // Test 0xA4: Absolute Position Control
    auto p_a4 = ServoProtocol::position_control_abs(180.0f, 360);
    assert(p_a4[0] == 0xA4);
    assert(p_a4[2] == 0x68 && p_a4[3] == 0x01); // 360 Little-Endian
    assert(p_a4[4] == 0x50 && p_a4[5] == 0x46 && p_a4[6] == 0x00 && p_a4[7] == 0x00); // 18000
    std::cout << " -> 0xA4 (Position Control) Encoding PASSED." << std::endl;

    // Test 0xB6: Active Response Control
    auto p_b6 = ServoProtocol::set_active_response(0x9C, true, 10);
    assert(p_b6[0] == 0xB6);
    assert(p_b6[1] == 0x9C);
    assert(p_b6[2] == 0x01);
    assert(p_b6[3] == 0x0A && p_b6[4] == 0x00);
    std::cout << " -> 0xB6 (Active Response) Encoding PASSED." << std::endl;

    // Test 0xB3: Fourth-Layer Defense - Communication Timeout Protection
    auto p_b3 = ServoProtocol::set_comm_timeout(1000); // 1000ms
    assert(p_b3[0] == 0xB3);
    assert(p_b3[4] == 0xE8 && p_b3[5] == 0x03 && p_b3[6] == 0x00 && p_b3[7] == 0x00); // 1000 in Little-Endian
    std::cout << " -> 0xB3 (Set Comm Timeout - Layer 4) Encoding PASSED." << std::endl;
}

void test_decoders() {
    std::cout << "[TEST] Running Decoder Tests..." << std::endl;

    // Test 0x9C Decoding
    std::array<uint8_t, 8> raw_9c = {0x9C, 0x28, 0x64, 0x00, 0xF4, 0x01, 0x10, 0x27};
    auto res_9c = ServoDecoder::decode_any(raw_9c);
    assert(std::holds_alternative<StandardMotionTelemetry>(res_9c));
    const auto& t_9c = std::get<StandardMotionTelemetry>(res_9c);
    assert(t_9c.cmd_echo == 0x9C);
    assert(t_9c.temperature_c == 40);
    assert(std::abs(t_9c.iq_current_amp - 1.00f) < 1e-3f);
    assert(std::abs(t_9c.speed_dps - 500.0f) < 1e-3f);
    assert(t_9c.angle_deg == 10000);
    std::cout << " -> 0x9C (Standard Motion Telemetry) Decoding PASSED." << std::endl;

    // Test 0x9A Decoding & Bitmask
    std::array<uint8_t, 8> raw_9a = {0x9A, 0x32, 0x37, 0x01, 0xE0, 0x01, 0x12, 0x00};
    auto res_9a = ServoDecoder::decode_any(raw_9a);
    assert(std::holds_alternative<SensorStatus1Telemetry>(res_9a));
    const auto& t_9a = std::get<SensorStatus1Telemetry>(res_9a);
    assert(t_9a.cmd_echo == 0x9A);
    assert(t_9a.temperature_c == 50);
    assert(t_9a.mos_temperature_c == 55);
    assert(t_9a.brake_released == true);
    assert(std::abs(t_9a.voltage_v - 48.0f) < 1e-3f);
    assert(t_9a.error_flags.stall() == true);
    assert(t_9a.error_flags.over_current() == true);
    assert(t_9a.error_flags.low_voltage() == false);
    std::cout << " -> 0x9A (Sensor Status 1 Telemetry) Decoding PASSED." << std::endl;

    // Test Layer 2 Defense: Unknown Ghost Command Echo (0xFF)
    std::array<uint8_t, 8> ghost_payload = {0xFF, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    auto res_ghost = ServoDecoder::decode_any(ghost_payload);
    assert(std::holds_alternative<UnknownTelemetry>(res_ghost));
    const auto& t_ghost = std::get<UnknownTelemetry>(res_ghost);
    assert(t_ghost.cmd_echo == 0xFF);
    assert(t_ghost.raw_payload == ghost_payload);
    std::cout << " -> Layer 2 Defense (Ghost Command Handling) PASSED." << std::endl;

    // Test Layer 2 Defense: IEEE 754 NaN payload in float decode (0x30)
    std::array<uint8_t, 8> nan_payload = {0x30, 0x01, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7F}; // NaN
    auto res_nan = ServoDecoder::decode_any(nan_payload);
    assert(std::holds_alternative<UnknownTelemetry>(res_nan));
    std::cout << " -> Layer 2 Defense (NaN/Inf Noise Filtering) PASSED." << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << " Starting Servo Protocol Unit Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    test_encoders();
    test_decoders();

    std::cout << "\nAll Unit Tests Passed Successfully!" << std::endl;
    return 0;
}