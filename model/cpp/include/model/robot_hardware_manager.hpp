#pragma once

#include <array>
#include <string>
#include <vector>
#include <memory>

#include "model/motor_controller.hpp"

namespace robot::model {

/**
 * @brief 頂層全域硬體管理者
 * 統籌 can1 ~ can4 四張 CAN 卡與 12 顆馬達 (Bus 0~3, ID 1~3)，
 * 統一支援「上位機高頻控制」與「GUI 狀態監控」兩種情境。
 */
class RobotHardwareManager {
public:
    static constexpr size_t NUM_CAN_BUSES = 4;
    static constexpr uint8_t MOTORS_PER_BUS = 3;

    RobotHardwareManager() = default;
    ~RobotHardwareManager() { stop_all(); }

    /**
     * @brief 一鍵初始化全機 4 條 CAN Bus
     * @param interface_names CAN 網卡名稱陣列 (如 {"can1", "can2", "can3", "can4"})
     */
    bool init_all(const std::array<std::string, NUM_CAN_BUSES>& interface_names = {"can1", "can2", "can3", "can4"}) {
        std::vector<uint8_t> active_motors = {1, 2, 3};
        for (size_t i = 0; i < NUM_CAN_BUSES; ++i) {
            if (!controllers_[i].init(interface_names[i], active_motors, static_cast<int>(2 + i))) {
                // 修改：初始化失敗時，Rollback 安全關閉之前已經啟動的控制器
                for (size_t j = 0; j < i; ++j) {
                    controllers_[j].stop();
                }
                return false;
            }
        }
        return true;
    }

    void stop_all() {
        for (auto& ctrl : controllers_) {
            ctrl.stop();
        }
    }

    // =========================================================================
    // 情境 1：使用者 GUI 監控專用 API (簡化上層繪圖與 Log 顯示)
    // =========================================================================

    /**
     * @brief 取得指定 CAN 卡與馬達 ID 的 MIT 遙測資料
     * @param bus_idx 網卡索引 (0: can1, 1: can2, 2: can3, 3: can4)
     * @param motor_id 馬達 ID (1~3)
     */
    bool get_motor_telemetry(size_t bus_idx, uint8_t motor_id, ::protocol::MITTelemetry& out_data) const {
        if (bus_idx >= NUM_CAN_BUSES) return false;
        return controllers_[bus_idx].get_mit_telemetry(motor_id, out_data);
    }

    /**
     * @brief 一次聚合讀取全機 4 條 CAN Bus 的歷史 CAN 封包日誌 (供 GUI 顯示)
     */
    std::vector<std::string> get_aggregated_can_logs() const {
        std::vector<std::string> aggregated_logs;
        for (size_t i = 0; i < NUM_CAN_BUSES; ++i) {
            auto logs = controllers_[i].get_can_logs();
            for (const auto& log : logs) {
                aggregated_logs.push_back("[Bus " + std::to_string(i + 1) + "] " + log);
            }
        }
        return aggregated_logs;
    }

    // =========================================================================
    // 情境 2：上位機控制專用 API (極速下發與直接路由)
    // =========================================================================

    /**
     * @brief 向指定的網卡與馬達下發運動控制指令
     */
    bool send_motion_command(size_t bus_idx, uint8_t motor_id, const std::array<uint8_t, 8>& payload) {
        if (bus_idx >= NUM_CAN_BUSES) return false;
        return controllers_[bus_idx].send_motion_command(motor_id, payload);
    }

    /**
     * @brief 取得單一 CAN Bus 控制器的直接存取權 (供高頻 RT 迴圈直連，極致效能)
     */
    MotorController& get_bus_controller(size_t bus_idx) {
        return controllers_[bus_idx];
    }

private:
    std::array<MotorController, NUM_CAN_BUSES> controllers_{};
};

} // namespace robot::model