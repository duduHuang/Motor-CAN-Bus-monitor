#include "model/motor_state_db.hpp"
#include <ctime>

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

// 新增 0xA6 單圈運動寫入/讀取實作
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

// 新增 0x9D 三相電流寫入/讀取實作
void MotorStateDB::update_sensor3_telemetry(uint8_t motor_id, const SensorStatus3Telemetry& data, double timestamp) noexcept {
    if (!is_valid_motor_id(motor_id)) return;
    double ts = (timestamp > 0.0) ? timestamp : get_monotonic_time_sec();
    slots_[motor_id - 1].sensor3_telemetry.write(data, ts);
}

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