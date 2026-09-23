#pragma once

#include <cstdint>
#include <atomic>
#include <array>
#include <ctime>

namespace robot::model {

// 1. 馬達遙測資料結構定義 (對齊 Step 1 & Step 2)
struct MITTelemetry {
    float position_rad{0.0f};
    float velocity_rads{0.0f};
    float torque_nm{0.0f};
};

struct StandardMotionTelemetry {
    int8_t temperature_c{0};
    float iq_current_amp{0.0f};
    float speed_dps{0.0f};
    float degree{0.0f};
};

struct SensorStatus1Telemetry {
    int8_t temperature_c{0};
    float voltage_v{0.0f};
    uint16_t error_code{0};
};

/**
 * @brief Seqlock (Sequence Lock) 零記憶體分配無鎖資料槽
 * 專為 ARM Cortex-A78AE (ARMv8-A) 弱記憶體序 (Weakly-Ordered) 架構設計。
 * 透過顯式記憶體屏障 (Memory Fence / acquire-release) 保證 1000Hz RT 迴圈無競爭讀寫。
 */
template <typename T>
struct alignas(64) SeqlockSlot { // 64-byte 對齊，防止跨核 False Sharing
    std::atomic<uint32_t> sequence{0};
    T data{};
    double timestamp{0.0};

    /**
     * @brief 寫入端 (CAN RX 背景執行緒，非阻塞)
     */
    void write(const T& new_data, double ts) noexcept {
        uint32_t current_seq = sequence.load(std::memory_order_relaxed);
        
        // 步驟 1: 將 sequence 設為奇數，標示「正在寫入」
        sequence.store(current_seq + 1, std::memory_order_relaxed);
        
        // 記憶體屏障：確保 sequence 變更先於 Payload 寫入被其他 CPU Core 看見
        std::atomic_thread_fence(std::memory_order_release);

        // 步驟 2: 寫入實際數據 Payload
        data = new_data;
        timestamp = ts;

        // 記憶體屏障：確保 Payload 寫入完成後，才更新 sequence
        std::atomic_thread_fence(std::memory_order_release);

        // 步驟 3: 將 sequence 設為偶數，標示「寫入完成」
        sequence.store(current_seq + 2, std::memory_order_release);
    }

    /**
     * @brief 讀取端 (1000Hz RT 控制執行緒，非阻塞 O(1) 時間複雜度)
     * @return true 讀取成功且資料一致；false 寫入頻繁導致 Retry 失敗
     */
    bool read(T& out_data, double& out_ts) const noexcept {
        constexpr int MAX_RETRIES = 3;
        for (int retry = 0; retry < MAX_RETRIES; ++retry) {
            uint32_t seq1 = sequence.load(std::memory_order_acquire);
            
            // 若為奇數，代表 CAN RX 正在寫入中，讓出流水線提示 CPU (yield)
            if (seq1 & 1) {
#if defined(__aarch64__)
                asm volatile("yield" ::: "memory");
#endif
                continue;
            }

            // 複製 Payload
            out_data = data;
            out_ts = timestamp;

            // 讀取記憶體屏障
            std::atomic_thread_fence(std::memory_order_acquire);
            uint32_t seq2 = sequence.load(std::memory_order_relaxed);

            // 若前後 Sequence 一致且未被中斷寫入，保證讀取資料完整未撕裂
            if (seq1 == seq2) {
                return true;
            }
        }
        return false;
    }
};

// 單顆馬達對應之多型別 Telemetry 靜態槽
struct alignas(64) MotorSlot {
    SeqlockSlot<MITTelemetry> mit_telemetry;
    SeqlockSlot<StandardMotionTelemetry> motion_telemetry;
    SeqlockSlot<SensorStatus1Telemetry> sensor_telemetry;
    
    // 【第四層防禦】硬體/上位機 Fault 與 Cascade E-STOP 觸發標記
    std::atomic<bool> is_faulted{false};
};

/**
 * @brief 馬達狀態資料庫 (MotorStateDB) - 單例類別
 * 零動態記憶體分配 (Zero-Allocation)，全靜態預分配陣列 (`MAX_MOTORS = 63`)
 */
class MotorStateDB {
public:
    static constexpr uint8_t MAX_MOTORS = 63; // 支援最大馬達 ID (1~63)

    static MotorStateDB& instance() noexcept;

    // 禁用 Copy / Move 語意，確保 Singleton 唯一性
    MotorStateDB(const MotorStateDB&) = delete;
    MotorStateDB& operator=(const MotorStateDB&) = delete;
    MotorStateDB(MotorStateDB&&) = delete;
    MotorStateDB& operator=(MotorStateDB&&) = delete;

    // === 寫入 API (CAN RX 背景執行緒，非阻塞) ===
    void update_mit_telemetry(uint8_t motor_id, const MITTelemetry& data, double timestamp = 0.0) noexcept;
    void update_motion_telemetry(uint8_t motor_id, const StandardMotionTelemetry& data, double timestamp = 0.0) noexcept;
    void update_sensor_telemetry(uint8_t motor_id, const SensorStatus1Telemetry& data, double timestamp = 0.0) noexcept;

    // === 讀取 API (1000Hz RT 控制執行緒，無鎖非阻塞) ===
    bool get_mit_telemetry(uint8_t motor_id, MITTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_motion_telemetry(uint8_t motor_id, StandardMotionTelemetry& out_data, double& out_timestamp) const noexcept;
    bool get_sensor_telemetry(uint8_t motor_id, SensorStatus1Telemetry& out_data, double& out_timestamp) const noexcept;

    // === 【第四層防禦 API】Stale Data 防護與 Fault 標記管理 ===
    /**
     * @brief 判斷馬達數據是否已過期 (Stale Data 防護)
     * @param motor_id 馬達 ID (1~63)
     * @param max_stale_sec 允許的最大陳舊時間（預設 0.3 秒）
     * @return true 代表資料已超時或從未接收；false 代表資料新鮮可用
     */
    bool is_telemetry_stale(uint8_t motor_id, double max_stale_sec = 0.3) const noexcept;

    void set_fault(uint8_t motor_id, bool faulted) noexcept;
    bool get_fault(uint8_t motor_id) const noexcept;

    // 取得高精度單調時間戳記 (CLOCK_MONOTONIC)
    static double get_monotonic_time_sec() noexcept;

private:
    MotorStateDB() noexcept = default;
    ~MotorStateDB() noexcept = default;

    static constexpr bool is_valid_motor_id(uint8_t motor_id) noexcept {
        return motor_id >= 1 && motor_id <= MAX_MOTORS;
    }

    // 預先配置 63 顆馬達靜態記憶體 (ID 1~63 對應 Index 0~62)
    alignas(64) std::array<MotorSlot, MAX_MOTORS> slots_{};
};

} // namespace robot::model