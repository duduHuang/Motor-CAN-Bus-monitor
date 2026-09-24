#include "model/motor_state_db.hpp"
#include <ctime>
#include <algorithm>

namespace robot::model {

MotorStateDB& MotorStateDB::instance() noexcept {
    static MotorStateDB db_instance;
    return db_instance;
}

double MotorStateDB::get_monotonic_time_sec() noexcept {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// === 寫入 API 實作 ===

void MotorStateDB::update_mit_telemetry(uint8_t motor_id, const MITTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].mit_telemetry.write(data, ts);
}

void MotorStateDB::update_motion_telemetry(uint8_t motor_id, const StandardMotionTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].motion_telemetry.write(data, ts);
}

void MotorStateDB::update_single_turn_telemetry(uint8_t motor_id, const SingleTurnMotionTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].single_turn_telemetry.write(data, ts);
}

void MotorStateDB::update_sensor_telemetry(uint8_t motor_id, const SensorStatus1Telemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].sensor_telemetry.write(data, ts);
}

void MotorStateDB::update_sensor3_telemetry(uint8_t motor_id, const SensorStatus3Telemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].sensor3_telemetry.write(data, ts);
}

void MotorStateDB::update_pid_telemetry(uint8_t motor_id, const PIDQueryTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].pid_telemetry.write(data, ts);
}

void MotorStateDB::update_accel_telemetry(uint8_t motor_id, const AccelQueryTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].accel_telemetry.write(data, ts);
}

void MotorStateDB::update_encoder_pos_telemetry(uint8_t motor_id, const EncoderPosTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].encoder_pos_telemetry.write(data, ts);
}

void MotorStateDB::update_zero_offset_telemetry(uint8_t motor_id, const ZeroOffsetTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].zero_offset_telemetry.write(data, ts);
}

void MotorStateDB::update_angle_query_telemetry(uint8_t motor_id, const AngleQueryTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].angle_query_telemetry.write(data, ts);
}

void MotorStateDB::update_system_mode_telemetry(uint8_t motor_id, const SystemModeTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].system_mode_telemetry.write(data, ts);
}

void MotorStateDB::update_system_info_telemetry(uint8_t motor_id, const SystemInfoTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].system_info_telemetry.write(data, ts);
}

void MotorStateDB::update_motor_model_telemetry(uint8_t motor_id, const MotorModelTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].motor_model_telemetry.write(data, ts);
}

void MotorStateDB::update_write_ack_telemetry(uint8_t motor_id, const WriteAckTelemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].write_ack_telemetry.write(data, ts);
}

// === 讀取 API 實作 ===

bool MotorStateDB::get_mit_telemetry(uint8_t motor_id, MITTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].mit_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_motion_telemetry(uint8_t motor_id, StandardMotionTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].motion_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_single_turn_telemetry(uint8_t motor_id, SingleTurnMotionTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].single_turn_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_sensor_telemetry(uint8_t motor_id, SensorStatus1Telemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].sensor_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_sensor3_telemetry(uint8_t motor_id, SensorStatus3Telemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].sensor3_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_pid_telemetry(uint8_t motor_id, PIDQueryTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].pid_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_accel_telemetry(uint8_t motor_id, AccelQueryTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].accel_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_encoder_pos_telemetry(uint8_t motor_id, EncoderPosTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].encoder_pos_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_zero_offset_telemetry(uint8_t motor_id, ZeroOffsetTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].zero_offset_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_angle_query_telemetry(uint8_t motor_id, AngleQueryTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].angle_query_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_system_mode_telemetry(uint8_t motor_id, SystemModeTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].system_mode_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_system_info_telemetry(uint8_t motor_id, SystemInfoTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].system_info_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_motor_model_telemetry(uint8_t motor_id, MotorModelTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].motor_model_telemetry.read(out_data, out_timestamp);
}

bool MotorStateDB::get_write_ack_telemetry(uint8_t motor_id, WriteAckTelemetry& out_data, double& out_timestamp) const noexcept {
    if (!is_valid_motor_id(motor_id)) return false;
    return slots_[motor_id - 1].write_ack_telemetry.read(out_data, out_timestamp);
}

// === 【第四層防禦 API 實作】 ===

bool MotorStateDB::is_telemetry_stale(uint8_t motor_id, double max_stale_sec) const noexcept {
    if (!is_valid_motor_id(motor_id)) return true;

    MITTelemetry mit_data;
    StandardMotionTelemetry motion_data;
    double mit_ts = 0.0, motion_ts = 0.0;

    bool has_mit = get_mit_telemetry(motor_id, mit_data, mit_ts);
    bool has_motion = get_motion_telemetry(motor_id, motion_data, motion_ts);

    // 修正：取兩者中最新更新的時間戳記進行比對
    double latest_ts = std::max(mit_ts, motion_ts);

    if (latest_ts <= 0.0) {
        return true; 
    }

    double current_time = get_monotonic_time_sec();
    return (current_time - latest_ts) > max_stale_sec;
}

void MotorStateDB::set_fault(uint8_t motor_id, bool faulted) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    slots_[motor_id - 1].is_faulted.store(faulted, std::memory_order_relaxed);
}

bool MotorStateDB::get_fault(uint8_t motor_id) const noexcept {
    if (!is_valid_motor_id(motor_id)) return true; // 無效 ID 預設視為 Fault
    return slots_[motor_id - 1].is_faulted.load(std::memory_order_relaxed);
}

} // namespace robot::model