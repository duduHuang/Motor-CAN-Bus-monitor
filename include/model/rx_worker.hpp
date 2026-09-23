#ifndef MODEL_RX_WORKER_HPP_
#define MODEL_RX_WORKER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <pthread.h>
#include <sched.h>
#include <thread>

#include "protocol/mit_protocol.hpp"
#include "protocol/servo_protocol.hpp"
#include "model/motor_state_db.hpp"
#include "model/socket_can_interface.hpp"

namespace robot::model {

// 第三層防禦：運動學極限過濾與 Debounce 狀態機 (每顆馬達獨立配置)
struct KinematicFilter {
    float last_position_rad{0.0f};
    int error_count{0};

    // 物理極限常數 (針對 1ms 迴圈)
    static constexpr float MAX_POS_DELTA_PER_MS = 0.2f; // 1ms 內角度跳變限制 (rad)
    static constexpr float MAX_SPEED_RADS = 35.0f;      // 絕對轉速限制 (rad/s)
    static constexpr int DEBOUNCE_THRESHOLD = 3;        // 連續 3 幀異常則判定故障
};

class RxWorker {
public:
    RxWorker(SocketCANInterface& socket_can, 
             MotorStateDB& db, 
             int cpu_core = 5, 
             int rt_priority = 90);
    ~RxWorker();

    // 嚴格禁用拷貝與移動
    RxWorker(const RxWorker&) = delete;
    RxWorker& operator=(const RxWorker&) = delete;
    RxWorker(RxWorker&&) = delete;
    RxWorker& operator=(RxWorker&&) = delete;

    bool start();
    void stop();
    bool is_running() const noexcept {
        return running_.load(std::memory_order_relaxed);
    }

private:
    void worker_loop();

    // 運動學濾波處理邏輯 (Inline 展開以確保極低延遲)
    inline void process_kinematic_filter(uint8_t motor_id, MITTelemetry& telemetry);

    SocketCANInterface& socket_can_;
    MotorStateDB& db_;
    int cpu_core_;
    int rt_priority_;

    std::atomic<bool> running_{false};
    std::thread worker_thread_;

    // 靜態配置所有可能馬達的濾波器 (假設 Motor ID 最大為 63)
    std::array<KinematicFilter, 64> filters_{};
};

} // namespace robot::model

#endif // MODEL_RX_WORKER_HPP_