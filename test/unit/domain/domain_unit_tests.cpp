// tests/unit/domain/domain_unit_tests.cpp
#include <iostream>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#include "motor/domain/motor_address.hpp"
#include "motor/domain/joint_id.hpp"
#include "motor/domain/motor_profile_id.hpp"
#include "motor/domain/motor_action.hpp"
#include "motor/domain/motor_command.hpp"
#include "motor/domain/motor_state.hpp"
#include "motor/domain/motor_fault.hpp"
#include "motor/domain/motor_health.hpp"
#include "motor/domain/motor_capabilities.hpp"
#include "motor/domain/motor_limits.hpp"
#include "motor/domain/motor_profile.hpp"
#include "motor/domain/motor_descriptor.hpp"
#include "motor/domain/sample_sequence.hpp"
#include "motor/domain/motor_coordinate_transform.hpp"
#include "motor/domain/descriptor_validator.hpp"

using namespace motor::domain;

// =============================================================================
// 1. 編譯期 Traits 與 Static Asserts
// =============================================================================

static_assert(std::is_same_v<std::underlying_type_t<MotorFaultFlag>, uint32_t>);
static_assert(std::is_same_v<std::underlying_type_t<Capability>, uint16_t>);
static_assert(std::is_same_v<std::underlying_type_t<StateField>, uint16_t>);

constexpr auto state_mask = StateField::Position | StateField::Velocity | StateField::Torque;
static_assert(static_cast<uint16_t>(state_mask) == 0x0007);

constexpr auto motion_caps = Capability::PositionControl | Capability::VelocityControl | Capability::TorqueControl;
static_assert(static_cast<uint16_t>(motion_caps) == 0x0007);

constexpr auto fault_mask = MotorFaultFlag::EncoderError | MotorFaultFlag::StaleData;
constexpr MotorFaultSet fault_set{static_cast<uint32_t>(fault_mask)};
static_assert(fault_set.has(MotorFaultFlag::EncoderError));
static_assert(fault_set.has(MotorFaultFlag::StaleData));

// sample_sequence 遞增與溢位邏輯驗證
static_assert(next_sample_sequence(0) == 1);
static_assert(next_sample_sequence(1) == 2);
static_assert(next_sample_sequence(UINT32_MAX - 1) == UINT32_MAX);
static_assert(next_sample_sequence(UINT32_MAX) == 1); // 溢位回繞跳過 0

// Direction 安全轉換驗證
constexpr bool check_direction_constexpr() {
    float sign{0.0f};
    bool ok1 = try_direction_sign(Direction::Positive, sign) && (sign == 1.0f);
    bool ok2 = try_direction_sign(Direction::Negative, sign) && (sign == -1.0f);
    bool ok3 = !try_direction_sign(Direction::Invalid, sign) && (sign == 0.0f);
    return ok1 && ok2 && ok3;
}
static_assert(check_direction_constexpr());

// Trivially Copyable 靜態斷言
static_assert(std::is_trivially_copyable_v<MotorAddress>);
static_assert(std::is_trivially_copyable_v<MotorCommand>);
static_assert(std::is_trivially_copyable_v<MotorState>);
static_assert(std::is_trivially_copyable_v<MotorHealth>);
static_assert(std::is_trivially_copyable_v<MotorCapabilities>);
static_assert(std::is_trivially_copyable_v<MotorHardwareLimits>);
static_assert(std::is_trivially_copyable_v<JointSafetyLimits>);
static_assert(std::is_trivially_copyable_v<MotorProfile>);
static_assert(std::is_trivially_copyable_v<MotorDescriptor>);

// Standard Layout 靜態斷言
static_assert(std::is_standard_layout_v<MotorAddress>);
static_assert(std::is_standard_layout_v<MotorCommand>);
static_assert(std::is_standard_layout_v<MotorState>);
static_assert(std::is_standard_layout_v<MotorHealth>);
static_assert(std::is_standard_layout_v<MotorProfileId>);
static_assert(std::is_standard_layout_v<MotorFaultSet>);
static_assert(std::is_standard_layout_v<MotorHardwareLimits>);
static_assert(std::is_standard_layout_v<JointSafetyLimits>);
static_assert(std::is_standard_layout_v<MotorProfile>);
static_assert(std::is_standard_layout_v<MotorDescriptor>);

// Runtime 測試輔助 (避免 Release / NDEBUG 停用測試)
void check_require(bool condition, const char* msg) {
    if (!condition) {
        std::cerr << "❌ [TEST FAILED] " << msg << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

std::pair<MotorDescriptor, MotorProfile> create_valid_test_fixtures() {
    MotorProfile profile{};
    profile.profile_id = MotorProfileId{1};
    profile.capabilities.set(Capability::CurrentFeedback | Capability::TorqueFeedback);
    profile.torque_constant_nm_per_amp = 0.5f;
    profile.reported_torque_location = TorqueLocation::MotorShaft;
    profile.hardware_limits.configured = true;
    profile.hardware_limits.max_motor_velocity_radps = 50.0f;
    profile.hardware_limits.max_motor_torque_nm = 10.0f;
    profile.hardware_limits.max_motor_temperature_c = 80.0f;
    profile.hardware_limits.max_mos_temperature_c = 85.0f;
    profile.hardware_limits.min_voltage_v = 18.0f;
    profile.hardware_limits.max_voltage_v = 30.0f;

    MotorDescriptor desc{};
    desc.address = MotorAddress{.bus_index = 0, .motor_id = 1};
    desc.joint_id = JointId::RightFrontHipX;
    desc.profile_id = MotorProfileId{1};
    desc.direction = Direction::Positive;
    desc.motor_to_output_ratio = 10.0f;
    desc.gear_efficiency = 0.9f;
    desc.joint_zero_offset_rad = 0.5f;

    desc.joint_limits.configured = true;
    desc.joint_limits.min_joint_position_rad = -1.5f;
    desc.joint_limits.max_joint_position_rad = 1.5f;
    desc.joint_limits.max_joint_velocity_radps = 5.0f;
    desc.joint_limits.max_joint_acceleration_radps2 = 20.0f;
    desc.joint_limits.max_joint_torque_nm = 45.0f;
    desc.joint_limits.max_kp = 100.0f;
    desc.joint_limits.max_kd = 5.0f;
    desc.joint_limits.position_jump_tolerance_rad = 0.1f;

    return {desc, profile};
}

// 2. 測試 DescriptorValidator 25 種 Error enum 分支與邊界政策
void test_descriptor_validator_branches_and_policies() {
    auto [desc, profile] = create_valid_test_fixtures();

    check_require(DescriptorValidator::is_binding_valid(desc, profile), "Valid binding failed");

    // 政策測試 1: configured == true 且 max limits 設為 0.0f -> 成功 (代表 Fail-Closed 鎖死)
    MotorDescriptor zero_desc = desc;
    zero_desc.joint_limits.max_joint_velocity_radps = 0.0f;
    zero_desc.joint_limits.max_joint_acceleration_radps2 = 0.0f;
    zero_desc.joint_limits.max_joint_torque_nm = 0.0f;
    zero_desc.joint_limits.position_jump_tolerance_rad = 0.0f;
    check_require(DescriptorValidator::is_descriptor_valid(zero_desc), "Zero max limits failed validation");

    // 政策測試 2: 無 CurrentFeedback 且 Kt == 0 -> 成功
    MotorProfile no_curr_prof = profile;
    no_curr_prof.capabilities.clear(Capability::CurrentFeedback);
    no_curr_prof.torque_constant_nm_per_amp = 0.0f;
    check_require(DescriptorValidator::is_profile_valid(no_curr_prof), "No CurrentFeedback with zero Kt should pass");

    // 政策測試 3: 有 CurrentFeedback 且 Kt == 0 -> 失敗
    MotorProfile invalid_kt_prof = profile;
    invalid_kt_prof.capabilities.set(Capability::CurrentFeedback);
    invalid_kt_prof.torque_constant_nm_per_amp = 0.0f;
    check_require(DescriptorValidator::validate_profile(invalid_kt_prof) == DescriptorValidationError::InvalidProfileTorqueConstant, "CurrentFeedback with zero Kt failed");

    // 政策測試 4: 無 Torque/Current Feedback 且 location == Unknown -> 成功
    MotorProfile no_trq_prof = profile;
    no_trq_prof.capabilities.clear(Capability::TorqueFeedback);
    no_trq_prof.capabilities.clear(Capability::CurrentFeedback);
    no_trq_prof.reported_torque_location = TorqueLocation::Unknown;
    check_require(DescriptorValidator::is_profile_valid(no_trq_prof), "No Torque/Current Feedback with Unknown location should pass");

    // 政策測試 5: 無 Torque/Current Feedback 但 location 為非法 enum -> 失敗
    no_trq_prof.reported_torque_location = static_cast<TorqueLocation>(255);
    check_require(DescriptorValidator::validate_profile(no_trq_prof) == DescriptorValidationError::InvalidProfileTorqueLocation, "Invalid enum TorqueLocation failed");

    // 政策測試 6: 有 TorqueFeedback 且 location == Unknown -> 失敗
    MotorProfile trq_unkn_prof = profile;
    trq_unkn_prof.capabilities.set(Capability::TorqueFeedback);
    trq_unkn_prof.reported_torque_location = TorqueLocation::Unknown;
    check_require(DescriptorValidator::validate_profile(trq_unkn_prof) == DescriptorValidationError::InvalidProfileTorqueLocation, "TorqueFeedback with Unknown location failed");

    // 政策測試 7: validate_binding 對 Profile ID 不符 -> 失敗
    MotorDescriptor mismatch_desc = desc;
    mismatch_desc.profile_id = MotorProfileId{99};
    check_require(DescriptorValidator::validate_binding(mismatch_desc, profile) == DescriptorValidationError::ProfileIdMismatch, "Profile ID mismatch failed");

    // 分支測試 25 種 Error Enum
    MotorDescriptor d = desc; MotorProfile p = profile;

    d.address.motor_id = 0; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidAddress, "InvalidAddress failed"); d = desc;
    d.joint_id = static_cast<JointId>(255); check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidJointId, "InvalidJointId failed"); d = desc;
    d.profile_id = MotorProfileId{0}; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidProfileId, "InvalidProfileId failed"); d = desc;
    d.direction = static_cast<Direction>(99); check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidDirection, "InvalidDirection failed"); d = desc;
    d.motor_to_output_ratio = std::numeric_limits<float>::quiet_NaN(); check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::NonFiniteRatio, "NonFiniteRatio failed");
    d.motor_to_output_ratio = -1.0f; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidRatio, "InvalidRatio failed"); d = desc;
    d.gear_efficiency = std::numeric_limits<float>::infinity(); check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::NonFiniteEfficiency, "NonFiniteEfficiency failed");
    d.gear_efficiency = 0.0f; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidEfficiency, "InvalidEfficiency failed"); d = desc;
    d.joint_zero_offset_rad = std::numeric_limits<float>::quiet_NaN(); check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::NonFiniteOffset, "NonFiniteOffset failed"); d = desc;
    d.joint_limits.configured = false; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::JointLimitsNotConfigured, "JointLimitsNotConfigured failed"); d = desc;
    d.joint_limits.min_joint_position_rad = 2.0f; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidJointPositionRange, "InvalidJointPositionRange failed"); d = desc;
    d.joint_limits.max_joint_velocity_radps = -1.0f; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidJointVelocityLimit, "InvalidJointVelocityLimit failed"); d = desc;
    d.joint_limits.max_joint_acceleration_radps2 = -1.0f; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidJointAccelerationLimit, "InvalidJointAccelerationLimit failed"); d = desc;
    d.joint_limits.max_joint_torque_nm = -1.0f; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidJointTorqueLimit, "InvalidJointTorqueLimit failed"); d = desc;
    d.joint_limits.max_kp = -1.0f; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidJointGainsLimit, "InvalidJointGainsLimit failed"); d = desc;
    d.joint_limits.position_jump_tolerance_rad = -1.0f; check_require(DescriptorValidator::validate_descriptor(d) == DescriptorValidationError::InvalidJointPositionJumpLimit, "InvalidJointPositionJumpLimit failed"); d = desc;

    p.hardware_limits.configured = false; check_require(DescriptorValidator::validate_profile(p) == DescriptorValidationError::HardwareLimitsNotConfigured, "HardwareLimitsNotConfigured failed"); p = profile;
    p.hardware_limits.max_motor_velocity_radps = -1.0f; check_require(DescriptorValidator::validate_profile(p) == DescriptorValidationError::InvalidMotorVelocityLimit, "InvalidMotorVelocityLimit failed"); p = profile;
    p.hardware_limits.max_motor_torque_nm = -1.0f; check_require(DescriptorValidator::validate_profile(p) == DescriptorValidationError::InvalidMotorTorqueLimit, "InvalidMotorTorqueLimit failed"); p = profile;
    p.hardware_limits.max_motor_temperature_c = -10.0f; check_require(DescriptorValidator::validate_profile(p) == DescriptorValidationError::InvalidMotorTemperatureLimit, "InvalidMotorTemperatureLimit failed"); p = profile;
    p.hardware_limits.max_mos_temperature_c = 0.0f; check_require(DescriptorValidator::validate_profile(p) == DescriptorValidationError::InvalidMosTemperatureLimit, "InvalidMosTemperatureLimit failed"); p = profile;
    p.hardware_limits.min_voltage_v = std::numeric_limits<float>::quiet_NaN(); check_require(DescriptorValidator::validate_profile(p) == DescriptorValidationError::InvalidVoltageRange, "InvalidVoltageRange failed");
}

// 3. 恢復全方位運動學座標轉換成功與失敗測試 (Position, Velocity, Torque, Positive & Negative Direction)
void test_kinematics_conversions_full() {
    auto [desc, profile] = create_valid_test_fixtures();
    
    float out_val = 999.0f;

    // 3.1 Positive Direction 完整成功路徑測試
    // Position: Motor=10 -> Joint = (1*10/10)+0.5 = 1.5; Joint=1.5 -> Motor = 1*(1.5-0.5)*10 = 10
    check_require(forward_position(10.0f, desc, out_val) && std::abs(out_val - 1.5f) < 1e-5f, "Forward pos Positive failed");
    check_require(inverse_position(1.5f, desc, out_val) && std::abs(out_val - 10.0f) < 1e-5f, "Inverse pos Positive failed");

    // Velocity: Motor=20 -> Joint = 2.0; Joint=2.0 -> Motor = 20
    check_require(forward_velocity(20.0f, desc, out_val) && std::abs(out_val - 2.0f) < 1e-5f, "Forward vel Positive failed");
    check_require(inverse_velocity(2.0f, desc, out_val) && std::abs(out_val - 20.0f) < 1e-5f, "Inverse vel Positive failed");

    // Torque MotorShaft: Motor=2.0 -> Joint = 2*10*0.9 = 18.0; Joint=18.0 -> Motor = 2.0
    check_require(forward_torque(2.0f, TorqueLocation::MotorShaft, desc, out_val) && std::abs(out_val - 18.0f) < 1e-5f, "Forward trq MotorShaft Positive failed");
    check_require(inverse_torque(18.0f, TorqueLocation::MotorShaft, desc, out_val) && std::abs(out_val - 2.0f) < 1e-5f, "Inverse trq MotorShaft Positive failed");

    // Torque OutputShaft: Motor=18.0 -> Joint = 18.0; Joint=18.0 -> Motor = 18.0
    check_require(forward_torque(18.0f, TorqueLocation::OutputShaft, desc, out_val) && std::abs(out_val - 18.0f) < 1e-5f, "Forward trq OutputShaft Positive failed");
    check_require(inverse_torque(18.0f, TorqueLocation::OutputShaft, desc, out_val) && std::abs(out_val - 18.0f) < 1e-5f, "Inverse trq OutputShaft Positive failed");

    // 3.2 Negative Direction 測試
    desc.direction = Direction::Negative;
    check_require(forward_position(10.0f, desc, out_val) && std::abs(out_val - (-0.5f)) < 1e-5f, "Forward pos Negative failed");
    check_require(inverse_position(-0.5f, desc, out_val) && std::abs(out_val - 10.0f) < 1e-5f, "Inverse pos Negative failed");

    check_require(forward_velocity(20.0f, desc, out_val) && std::abs(out_val - (-2.0f)) < 1e-5f, "Forward vel Negative failed");
    check_require(inverse_velocity(-2.0f, desc, out_val) && std::abs(out_val - 20.0f) < 1e-5f, "Inverse vel Negative failed");

    check_require(forward_torque(2.0f, TorqueLocation::MotorShaft, desc, out_val) && std::abs(out_val - (-18.0f)) < 1e-5f, "Forward trq MotorShaft Negative failed");
    check_require(inverse_torque(-18.0f, TorqueLocation::MotorShaft, desc, out_val) && std::abs(out_val - 2.0f) < 1e-5f, "Inverse trq MotorShaft Negative failed");

    // 3.3 失敗不變性測試 (Out parameter immutability on failure)
    out_val = 888.0f;
    check_require(!forward_torque(2.0f, TorqueLocation::Unknown, desc, out_val), "forward_torque Unknown failed");
    check_require(out_val == 888.0f, "out_val modified on Unknown torque location");

    desc.motor_to_output_ratio = 0.0f;
    check_require(!forward_position(10.0f, desc, out_val), "forward_position ratio 0 failed");
    check_require(out_val == 888.0f, "out_val modified on zero ratio");
}

// 4. 恢復 Flag Collection 測試 (StateValidity, MotorCapabilities, MotorFaultSet)
void test_flag_collections_ops() {
    StateValidity val{};
    check_require(!val.has_any(StateField::Position | StateField::Velocity), "Initial validity failed");

    val.set(StateField::Position);
    check_require(val.has(StateField::Position), "StateValidity set failed");
    check_require(val.has_any(StateField::Position | StateField::Velocity), "StateValidity has_any failed");
    check_require(!val.has_all(StateField::Position | StateField::Velocity), "StateValidity has_all failed");

    val.set(StateField::Velocity);
    check_require(val.has_all(StateField::Position | StateField::Velocity), "StateValidity has_all multi failed");

    val.clear(StateField::Position);
    check_require(!val.has(StateField::Position), "StateValidity clear failed");

    val.reset();
    check_require(!val.has_any(StateField::Position | StateField::Velocity), "StateValidity reset failed");

    // MotorCapabilities set/clear/reset/has_all/has_any
    MotorCapabilities caps{};
    caps.set(Capability::PositionControl | Capability::VelocityControl);
    check_require(caps.has_all(Capability::PositionControl | Capability::VelocityControl), "Caps set/has_all failed");
    caps.clear(Capability::PositionControl);
    check_require(!caps.has(Capability::PositionControl), "Caps clear failed");
    caps.reset();
    check_require(!caps.has_any(Capability::PositionControl | Capability::VelocityControl), "Caps reset failed");

    // MotorFaultSet clear/reset
    MotorFaultSet fset{};
    fset.set(MotorFaultFlag::StaleData);
    check_require(fset.has(MotorFaultFlag::StaleData), "FaultSet set failed");
    fset.clear(MotorFaultFlag::StaleData);
    check_require(!fset.has(MotorFaultFlag::StaleData), "FaultSet clear failed");
    fset.set(MotorFaultFlag::Stall);
    fset.reset();
    check_require(!fset.has_any(MotorFaultFlag::Stall), "FaultSet reset failed");
}

// 測試：6 種 MotorState Invariant 組合全覆蓋
void test_state_invariant_matrix() {
    MotorState state{};

    state.validity.set(StateField::Torque); state.torque_source = TorqueSource::ProtocolReported;
    check_require(is_motor_state_consistent(state), "Case 1: valid + reported failed");

    state.torque_source = TorqueSource::Unavailable;
    check_require(!is_motor_state_consistent(state), "Case 2: valid + unavailable failed");

    state.validity.clear(StateField::Torque); state.torque_source = TorqueSource::Unavailable;
    check_require(is_motor_state_consistent(state), "Case 3: invalid + unavailable failed");

    state.torque_source = TorqueSource::ProtocolReported;
    check_require(!is_motor_state_consistent(state), "Case 4: invalid + reported failed");

    state.validity.set(StateField::Torque); state.torque_source = TorqueSource::EstimatedFromIqCurrent;
    check_require(is_motor_state_consistent(state), "Case 5: valid + estimated failed");

    state.torque_source = TorqueSource::ExternalSensor;
    check_require(is_motor_state_consistent(state), "Case 6: valid + external sensor failed");
}

// 補齊 Helper 失敗時的輸出引數不變性；每次呼叫分別重設並檢查 sentinel。
void test_kinematics_immutability_on_failure() {
    MotorDescriptor desc{};
    desc.direction = Direction::Positive;
    desc.motor_to_output_ratio = 10.0f;
    desc.gear_efficiency = 0.9f;

    constexpr float SENTINEL = 999.0f;
    float out_val = SENTINEL;

    // inverse_torque Unknown
    out_val = SENTINEL;
    check_require(!inverse_torque(10.0f, TorqueLocation::Unknown, desc, out_val),
                  "inverse_torque Unknown should fail");
    check_require(out_val == SENTINEL,
                  "inverse_torque Unknown modified out parameter");

    // forward_velocity ratio 0
    desc.motor_to_output_ratio = 0.0f;
    out_val = SENTINEL;
    check_require(!forward_velocity(10.0f, desc, out_val),
                  "forward_velocity ratio 0 should fail");
    check_require(out_val == SENTINEL,
                  "forward_velocity ratio 0 modified out parameter");

    // inverse_velocity ratio 0
    out_val = SENTINEL;
    check_require(!inverse_velocity(1.0f, desc, out_val),
                  "inverse_velocity ratio 0 should fail");
    check_require(out_val == SENTINEL,
                  "inverse_velocity ratio 0 modified out parameter");

    // inverse_position ratio 0
    out_val = SENTINEL;
    check_require(!inverse_position(1.0f, desc, out_val),
                  "inverse_position ratio 0 should fail");
    check_require(out_val == SENTINEL,
                  "inverse_position ratio 0 modified out parameter");

    // forward_position Invalid direction
    desc.motor_to_output_ratio = 10.0f;
    desc.direction = Direction::Invalid;
    out_val = SENTINEL;
    check_require(!forward_position(10.0f, desc, out_val),
                  "forward_position Invalid direction should fail");
    check_require(out_val == SENTINEL,
                  "forward_position Invalid direction modified out parameter");

    // inverse_position Invalid direction
    out_val = SENTINEL;
    check_require(!inverse_position(1.0f, desc, out_val),
                  "inverse_position Invalid direction should fail");
    check_require(out_val == SENTINEL,
                  "inverse_position Invalid direction modified out parameter");

    // forward_velocity Invalid direction
    out_val = SENTINEL;
    check_require(!forward_velocity(10.0f, desc, out_val),
                  "forward_velocity Invalid direction should fail");
    check_require(out_val == SENTINEL,
                  "forward_velocity Invalid direction modified out parameter");

    // inverse_velocity Invalid direction
    out_val = SENTINEL;
    check_require(!inverse_velocity(1.0f, desc, out_val),
                  "inverse_velocity Invalid direction should fail");
    check_require(out_val == SENTINEL,
                  "inverse_velocity Invalid direction modified out parameter");

    // forward_torque Invalid direction
    out_val = SENTINEL;
    check_require(!forward_torque(10.0f, TorqueLocation::MotorShaft, desc, out_val),
                  "forward_torque Invalid direction should fail");
    check_require(out_val == SENTINEL,
                  "forward_torque Invalid direction modified out parameter");

    // inverse_torque Invalid direction
    out_val = SENTINEL;
    check_require(!inverse_torque(10.0f, TorqueLocation::MotorShaft, desc, out_val),
                  "inverse_torque Invalid direction should fail");
    check_require(out_val == SENTINEL,
                  "inverse_torque Invalid direction modified out parameter");

    // forward_torque zero efficiency
    desc.direction = Direction::Positive;
    desc.gear_efficiency = 0.0f;
    out_val = SENTINEL;
    check_require(!forward_torque(10.0f, TorqueLocation::MotorShaft, desc, out_val),
                  "forward_torque zero efficiency should fail");
    check_require(out_val == SENTINEL,
                  "forward_torque zero efficiency modified out parameter");

    // inverse_torque zero efficiency
    out_val = SENTINEL;
    check_require(!inverse_torque(10.0f, TorqueLocation::MotorShaft, desc, out_val),
                  "inverse_torque zero efficiency should fail");
    check_require(out_val == SENTINEL,
                  "inverse_torque zero efficiency modified out parameter");
}

// NaN / Inf 與 configured 硬體零上限政策；各案例只改動目標條件。
void test_validator_nan_inf_and_zero_limit_policies() {
    const auto [desc, profile] = create_valid_test_fixtures();
    check_require(DescriptorValidator::is_binding_valid(desc, profile),
                  "Boundary test fixtures must form a valid binding");

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    MotorDescriptor d = desc;
    MotorProfile p = profile;

    d = desc;
    d.joint_limits.max_joint_velocity_radps = nan;
    check_require(
        DescriptorValidator::validate_descriptor(d) ==
            DescriptorValidationError::InvalidJointVelocityLimit,
        "NaN joint velocity check failed");

    d = desc;
    d.joint_limits.max_joint_acceleration_radps2 = inf;
    check_require(
        DescriptorValidator::validate_descriptor(d) ==
            DescriptorValidationError::InvalidJointAccelerationLimit,
        "Inf joint acceleration check failed");

    d = desc;
    d.joint_limits.max_joint_torque_nm = nan;
    check_require(
        DescriptorValidator::validate_descriptor(d) ==
            DescriptorValidationError::InvalidJointTorqueLimit,
        "NaN joint torque check failed");

    d = desc;
    d.joint_limits.max_kp = inf;
    check_require(
        DescriptorValidator::validate_descriptor(d) ==
            DescriptorValidationError::InvalidJointGainsLimit,
        "Inf Kp check failed");

    d = desc;
    d.joint_limits.max_kd = inf;
    check_require(
        DescriptorValidator::validate_descriptor(d) ==
            DescriptorValidationError::InvalidJointGainsLimit,
        "Inf Kd check failed");

    d = desc;
    d.joint_limits.position_jump_tolerance_rad = nan;
    check_require(
        DescriptorValidator::validate_descriptor(d) ==
            DescriptorValidationError::InvalidJointPositionJumpLimit,
        "NaN position jump tolerance check failed");

    p = profile;
    p.capabilities.set(Capability::CurrentFeedback);
    p.reported_torque_location = TorqueLocation::MotorShaft;
    p.torque_constant_nm_per_amp = nan;
    check_require(
        DescriptorValidator::validate_profile(p) ==
            DescriptorValidationError::InvalidProfileTorqueConstant,
        "CurrentFeedback with NaN Kt check failed");

    p = profile;
    p.capabilities.set(Capability::CurrentFeedback);
    p.reported_torque_location = TorqueLocation::MotorShaft;
    p.torque_constant_nm_per_amp = inf;
    check_require(
        DescriptorValidator::validate_profile(p) ==
            DescriptorValidationError::InvalidProfileTorqueConstant,
        "CurrentFeedback with Inf Kt check failed");

    p = profile;
    p.hardware_limits.max_motor_velocity_radps = nan;
    check_require(
        DescriptorValidator::validate_profile(p) ==
            DescriptorValidationError::InvalidMotorVelocityLimit,
        "NaN hardware velocity check failed");

    p = profile;
    p.hardware_limits.max_motor_torque_nm = inf;
    check_require(
        DescriptorValidator::validate_profile(p) ==
            DescriptorValidationError::InvalidMotorTorqueLimit,
        "Inf hardware torque check failed");

    p = profile;
    p.hardware_limits.max_voltage_v = inf;
    check_require(
        DescriptorValidator::validate_profile(p) ==
            DescriptorValidationError::InvalidVoltageRange,
        "Inf maximum voltage check failed");

    p = profile;
    p.hardware_limits.min_voltage_v = 24.0f;
    p.hardware_limits.max_voltage_v = 24.0f;
    check_require(
        DescriptorValidator::validate_profile(p) ==
            DescriptorValidationError::InvalidVoltageRange,
        "Voltage min == max check failed");

    // 驗證設定接受零上限；實際命令限制／禁止運動由 Safety / Service 測試。
    p = profile;
    p.hardware_limits.configured = true;
    p.hardware_limits.max_motor_velocity_radps = 0.0f;
    p.hardware_limits.max_motor_torque_nm = 0.0f;
    check_require(DescriptorValidator::is_profile_valid(p),
                  "Configured hardware zero limits should be accepted");
}

int main() {
    std::cout << "Starting Complete Domain Unit Test Suite...\n";

    test_descriptor_validator_branches_and_policies();
    test_kinematics_conversions_full();
    test_kinematics_immutability_on_failure();
    test_validator_nan_inf_and_zero_limit_policies();
    test_flag_collections_ops();
    test_state_invariant_matrix();

    std::cout << "✅ [PASSED] All registered Domain unit tests passed!\n";
    return 0;
}