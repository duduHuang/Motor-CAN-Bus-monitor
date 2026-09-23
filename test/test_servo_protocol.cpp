#include "protocol/servo_protocol.hpp"
#include "protocol/decoders.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace servo_robot::protocol;

void test_encoders() {
    std::cout << "[TEST] Running Encoder Tests..." << std::endl;

    // Test 0xA4: Absolute Position Control (target = 180.0 deg -> 18000 raw = 0x00004650, spd = 360 dps -> 0x0168)
    auto p_a4 = ServoProtocol::position_control_abs(180.0f, 360);
    assert(p_a4[0] == 0xA4);
    assert(p_a4[2] == 0x68 && p_a4[3] == 0x01); // 360 Little-Endian
    assert(p_a4[4] == 0x50 && p_a4[5] == 0x46 && p_a4[6] == 0x00 && p_a4[7] == 0x00); // 18000 Little-Endian
    std::cout << " -> 0xA4 (Position Control) Encoding PASSED." << std::endl;

    // Test 0xB6: Active Response Control (target_cmd = 0x9C, enable = true, interval = 10 -> 0x000A)
    auto p_b6 = ServoProtocol::set_active_response(0x9C, true, 10);
    assert(p_b6[0] == 0xB6);
    assert(p_b6[1] == 0x9C);
    assert(p_b6[2] == 0x01);
    assert(p_b6[3] == 0x0A && p_b6[4] == 0x00);
    std::cout << " -> 0xB6 (Active Response) Encoding PASSED." << std::endl;
}

void test_decoders() {
    std::cout << "[TEST] Running Decoder Tests..." << std::endl;

    // Test 0x9C Decoding
    // Raw: Temp=40C, Iq=100 (1.00A), Speed=500 dps, Angle=10000 deg
    std::array<uint8_t, 8> raw_9c = {
        0x9C, 0x28, 0x64, 0x00, 0xF4, 0x01, 0x10, 0x27
    };
    auto res_9c = ServoDecoder::decode_any(raw_9c);
    assert(std::holds_alternative<StandardMotionTelemetry>(res_9c));
    const auto& t_9c = std::get<StandardMotionTelemetry>(res_9c);
    assert(t_9c.cmd_echo == 0x9C);
    assert(t_9c.temperature_c == 40);
    assert(std::abs(t_9c.iq_current_amp - 1.00f) < 1e-3f);
    assert(std::abs(t_9c.speed_dps - 500.0f) < 1e-3f);
    assert(t_9c.angle_deg == 10000);
    std::cout << " -> 0x9C (Standard Motion Telemetry) Decoding PASSED." << std::endl;

    // Test 0x9A Decoding
    // Raw: Temp=50C, MOS=55C, Brake=1 (released), Volt=480 (48.0V), Err=0x0012 (Stall 0x0002 + OverCurrent 0x0010)
    std::array<uint8_t, 8> raw_9a = {
        0x9A, 0x32, 0x37, 0x01, 0xE0, 0x01, 0x12, 0x00
    };
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