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

// 修正：使用 model::SocketCANInterface 或引進別名
using SocketCANInterface = ::robot::model::SocketCANInterface;

struct KinematicFilter {
    float last_position_rad{0.0f};
    int error_count{0};
    bool initialized{false}; // 修正：新增初始化標記，避免開機角度非 0 時觸發誤判

    static constexpr float MAX_POS_DELTA_PER_MS = 0.2f; 
    static constexpr float MAX_SPEED_RADS = 35.0f;      
    static constexpr int DEBOUNCE_THRESHOLD = 3;        
};

class RxWorker {
public:
    RxWorker(SocketCANInterface& socket_can, 
             MotorStateDB& db, 
             int cpu_core = 5, 
             int rt_priority = 90);
    ~RxWorker();

    RxWorker(const RxWorker&) = delete;
    RxWorker& operator=(const RxWorker&) = delete;
    RxWorker(RxWorker&&) = delete;
    RxWorker& operator=(RxWorker&&) = delete;

    bool start();
    void stop();
    bool is_running() const noexcept {
        return running_.load(std::memory_order_relaxed);
    }

    // 修正：補上 get_rx_count API 供 MotorController 調用
    uint64_t get_rx_count() const noexcept {
        return rx_count_.load(std::memory_order_relaxed);
    }

private:
    void worker_loop();
    inline void process_kinematic_filter(uint8_t motor_id, MITTelemetry& telemetry);

    SocketCANInterface& socket_can_;
    MotorStateDB& db_;
    int cpu_core_;
    int rt_priority_;

    std::atomic<bool> running_{false};
    std::atomic<uint64_t> rx_count_{0}; // 修正：補上累計 RX 封包計數器
    std::thread worker_thread_;

    std::array<KinematicFilter, 64> filters_{};
};

} // namespace robot::model

#endif // MODEL_RX_WORKER_HPP_