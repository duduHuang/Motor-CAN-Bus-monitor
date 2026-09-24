#ifndef MODEL_CAN_FRAME_LOGGER_HPP_
#define MODEL_CAN_FRAME_LOGGER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>
#include <string>
#include <iomanip>
#include <sstream>
#include <time.h>
#include <algorithm>

namespace robot::model {

struct CANLogEntry {
    uint32_t can_id;
    std::array<uint8_t, 8> payload;
    double timestamp_sec;
    bool is_tx;
};

/**
 * @brief 高頻無鎖環形封包日誌 (Zero-Allocation & Lock-Free Circular Logger)
 */
class CANFrameLogger {
public:
    static constexpr size_t MAX_LOGS = 100;

    CANFrameLogger() noexcept : write_idx_(0) {}

    // 禁用拷貝與移動，確保內部原子操作的記憶體位址不被改變
    CANFrameLogger(const CANFrameLogger&) = delete;
    CANFrameLogger& operator=(const CANFrameLogger&) = delete;

    /**
     * @brief 高頻 RT 執行緒呼叫的無鎖寫入介面
     * @note 絕對禁止在此處使用 malloc, new 或 std::string，耗時 O(1)
     */
    inline void add_log(uint32_t can_id, const std::array<uint8_t, 8>& payload, bool is_tx) noexcept {
        // 使用 CLOCK_REALTIME 配合 Linux vDSO 加速，避免 User/Kernel space 轉換的 System Call 成本
        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        double timestamp = static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) / 1e9;

        // Lock-free 原子遞增 (完美支援 TX 執行緒與 RX Worker 並發寫入而不衝突)
        size_t idx = write_idx_.fetch_add(1, std::memory_order_relaxed) % MAX_LOGS;

        buffer_[idx].can_id = can_id;
        buffer_[idx].payload = payload;
        buffer_[idx].timestamp_sec = timestamp;
        buffer_[idx].is_tx = is_tx;
    }

    /**
     * @brief 快照匯出與字串格式化 (供非即時的 ViewModel/Web UI 執行緒呼叫)
     */
    std::vector<std::string> get_logs_snapshot() const {
        std::vector<std::string> snapshot;
        snapshot.reserve(MAX_LOGS);

        size_t current_idx = write_idx_.load(std::memory_order_acquire);
        size_t count = std::min(current_idx, MAX_LOGS);
        size_t start_idx = (current_idx >= MAX_LOGS) ? (current_idx % MAX_LOGS) : 0;

        for (size_t i = 0; i < count; ++i) {
            size_t idx = (start_idx + i) % MAX_LOGS;
            const auto& entry = buffer_[idx];

            time_t sec = static_cast<time_t>(entry.timestamp_sec);
            int ms = static_cast<int>((entry.timestamp_sec - sec) * 1000);
            
            struct tm timeinfo;
            localtime_r(&sec, &timeinfo); // Thread-safe 的本地時間轉換

            std::ostringstream oss;
            oss << "[" << std::setfill('0') << std::setw(2) << timeinfo.tm_hour << ":"
                << std::setfill('0') << std::setw(2) << timeinfo.tm_min << ":"
                << std::setfill('0') << std::setw(2) << timeinfo.tm_sec << "."
                << std::setfill('0') << std::setw(3) << ms << "] "
                << (entry.is_tx ? "TX" : "RX") 
                << " ID: 0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(3) << entry.can_id
                << " DATA:";
            
            for (uint8_t b : entry.payload) {
                oss << " " << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(b);
            }

            snapshot.push_back(oss.str());
        }
        return snapshot;
    }

private:
    std::array<CANLogEntry, MAX_LOGS> buffer_{};
    std::atomic<size_t> write_idx_{0};
};

} // namespace robot::model

#endif // MODEL_CAN_FRAME_LOGGER_HPP_