#ifndef MODEL_RX_WORKER_HPP_
#define MODEL_RX_WORKER_HPP_

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

class RxWorker {
public:
    RxWorker(SocketCANInterface& socket_can, 
             MotorStateDB& db, 
             int cpu_core = 5, 
             int rt_priority = 90);
    ~RxWorker();

    // 禁用拷貝與移動，確保 RT 執行緒資源生命週期單一性
    RxWorker(const RxWorker&) = delete;
    RxWorker& operator=(const RxWorker&) = delete;
    RxWorker(RxWorker&&) = delete;
    RxWorker& operator=(RxWorker&&) = delete;

    /**
     * @brief 啟動 PREEMPT_RT 背景接收執行緒
     * @return true 啟動成功；false 已在運行中或失敗
     */
    bool start();

    /**
     * @brief 安全停止背景執行緒 (Jitter-Free Join)
     */
    void stop();

    bool is_running() const noexcept {
        return running_.load(std::memory_order_relaxed);
    }

private:
    void worker_loop();

    SocketCANInterface& socket_can_;
    MotorStateDB& db_;
    int cpu_core_;
    int rt_priority_;

    std::atomic<bool> running_{false};
    std::thread worker_thread_;
};

} // namespace robot::model

#endif // MODEL_RX_WORKER_HPP_