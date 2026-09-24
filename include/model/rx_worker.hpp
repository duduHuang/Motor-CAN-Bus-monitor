#ifndef MODEL_RX_WORKER_HPP_
#define MODEL_RX_WORKER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <vector>
#include <functional>
#include <variant>
#include <mutex>

#include "protocol/mit_protocol.hpp"
#include "protocol/servo_protocol.hpp"
#include "model/motor_state_db.hpp"
#include "model/socket_can_interface.hpp"
#include "model/can_frame_logger.hpp"

namespace robot::model {

using SocketCANInterface = ::robot::model::SocketCANInterface;

/**
 * @brief 輕量級的遙測訊息封裝結構
 * 運用 std::variant 安全地容納 MIT 與所有 14 種 Servo 遙測資料
 */
struct RxTelemetryMessage {
    std::variant<std::monostate, 
                 ::protocol::MITTelemetry, 
                 ::servo_robot::protocol::ServoTelemetry> payload;
};

/**
 * @brief RT (Real-Time) Callback 函式指標
 * @warning [PREEMPT_RT Guard] 
 * 該 Callback 將在 RxWorker 的 SCHED_FIFO (Priority=90) 高優先級 RT 執行緒中被直接調用！
 * Callback 內部【嚴禁】執行以下操作：
 * 1. 任何 I/O 阻塞 (如 std::cout, 寫檔)
 * 2. GUI 渲染或畫面刷新
 * 3. 動態記憶體分配 (new / malloc / std::vector::push_back 等)
 * 4. 獲取可能被非 RT 執行緒持有的 Mutex (避免 Priority Inversion)
 * 建議上層實作：僅進行無鎖 (Lock-free) Queue 寫入或 std::atomic 標記。
 */
using RxMessageCallback = std::function<void(uint8_t motor_id, const RxTelemetryMessage& msg, double timestamp)>;

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
             CANFrameLogger& logger,
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

    // 註冊外部 Callback 監聽器
    void add_rx_callback(RxMessageCallback cb);

private:
    void worker_loop();
    inline void process_kinematic_filter(uint8_t motor_id, MITTelemetry& telemetry);

    // 即時分發訊息給所有註冊的 Observers
    inline void notify_callbacks(uint8_t motor_id, const RxTelemetryMessage& msg);

    SocketCANInterface& socket_can_;
    MotorStateDB& db_;
    CANFrameLogger& logger_;
    int cpu_core_;
    int rt_priority_;

    std::atomic<bool> running_{false};
    std::atomic<uint64_t> rx_count_{0}; // 修正：補上累計 RX 封包計數器
    std::thread worker_thread_;

    std::array<KinematicFilter, 64> filters_{};
    // Callback 註冊表與防護鎖
    std::vector<RxMessageCallback> callbacks_;
    std::mutex callback_mutex_;
};

} // namespace robot::model

#endif // MODEL_RX_WORKER_HPP_