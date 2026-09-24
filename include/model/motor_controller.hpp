#ifndef MODEL_MOTOR_CONTROLLER_HPP
#define MODEL_MOTOR_CONTROLLER_HPP

#include <string>
#include <array>
#include <atomic>
#include <memory>
#include <cstdint>

#include "model/socket_can_interface.hpp"
#include "protocol/servo_protocol.hpp"
#include "protocol/mit_protocol.hpp"
#include "model/motor_state_db.hpp"
#include "model/rx_worker.hpp"

namespace robot::model {

/**
 * @brief MotorController 門面類別 (Facade)
 * 封裝底層 SocketCAN 通訊、CAN Filter 配置與 RxWorker 背景接收執行緒，
 * 提供 Thread-safe 與 Zero-Allocation 的高頻控制 API。
 */
class MotorController {
public:
    static constexpr uint16_t SINGLE_MOTOR_BASE_TX = 0x140; // 單機指令 TX (0x140 + ID)
    static constexpr uint16_t SINGLE_MOTOR_BASE_RX = 0x240; // 單機回報 RX (0x240 + ID)
    static constexpr uint16_t MULTI_MOTOR_BASE_TX  = 0x280; // 多機廣播 TX (0x280)
    static constexpr uint16_t MOTION_MODE_BASE_TX  = 0x400; // 運動模式 TX (0x400 + ID)
    static constexpr uint16_t MOTION_MODE_BASE_RX  = 0x500; // 運動模式 RX (0x500 + ID)
    static constexpr uint8_t  MAX_MOTOR_ID          = 63;    // 最大支援馬達 ID

    MotorController() noexcept;
    ~MotorController() noexcept;

    // 禁用拷貝與移動建構，確保硬體 Handle 與執行緒獨佔
    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;
    MotorController(MotorController&&) = delete;
    MotorController& operator=(MotorController&&) = delete;

    /**
     * @brief 初始化 SocketCAN、配置硬體 Filter 並啟動 RxWorker 背景監聽執行緒
     * @param interface_name CAN 介面名稱 (例如 "can0")
     * @param rx_core_id RxWorker 綁定之 CPU Core ID (預設 Core 5)
     * @return true 初始化成功, false 初始化失敗
     */
    bool init(const std::string& interface_name, int rx_core_id = 5) noexcept;

    /**
     * @brief 停止背景 RxWorker 執行緒並關閉 SocketCAN 介面
     */
    void stop() noexcept;

    /**
     * @brief 發送單機控制指令 (Arbitration ID: 0x140 + motor_id)
     */
    bool send_single_command(uint8_t motor_id, const std::array<uint8_t, 8>& payload) noexcept;

    /**
     * @brief 發送多機廣播控制指令 (Arbitration ID: 0x280)
     */
    bool send_multi_command(const std::array<uint8_t, 8>& payload) noexcept;

    /**
     * @brief 發送 MIT 運動控制指令 (Arbitration ID: 0x400 + motor_id)
     */
    bool send_motion_command(uint8_t motor_id, const std::array<uint8_t, 8>& payload) noexcept;

    /**
     * @brief 從 MotorStateDB 讀取最新 MIT 運動狀態 (Non-blocking)
     */
    bool get_mit_telemetry(uint8_t motor_id, MITTelemetry& out_data) const noexcept;

    /**
     * @brief 從 MotorStateDB 讀取最新 0x9C 運動狀態 (Non-blocking)
     */
    bool get_motion_telemetry(uint8_t motor_id, StandardMotionTelemetry& out_data) const noexcept;

    /**
     * @brief 從 MotorStateDB 讀取最新 0xA6 單圈運動狀態 (Non-blocking)
     */
    bool get_single_turn_telemetry(uint8_t motor_id, SingleTurnMotionTelemetry& out_data) const noexcept;
    // 在 MotorController 類別 public 區塊補上：
    /**
     * @brief 從 MotorStateDB 讀取最新 0x9A 感測器與錯誤狀態 (Non-blocking)
     */
    bool get_sensor_telemetry(uint8_t motor_id, SensorStatus1Telemetry& out_data) const noexcept;

    /**
     * @brief 從 MotorStateDB 讀取最新 0x9D 感測器狀態 (Non-blocking)
     */
    bool get_sensor3_telemetry(uint8_t motor_id, SensorStatus3Telemetry& out_data) const noexcept;

    /**
     * @brief 取得累計 TX 發送封包數
     */
    uint64_t get_tx_count() const noexcept { return tx_count_.load(std::memory_order_relaxed); }

    /**
     * @brief 取得累計 RX 接收封包數
     */
    uint64_t get_rx_count() const noexcept;

private:
    /**
     * @brief 第四層防禦：配置硬體通訊中斷保護 (0xB3)
     */
    void setup_hardware_watchdog(uint32_t timeout_ms = 300) noexcept;

    SocketCANInterface              can_iface_;
    MotorStateDB                    state_db_;
    std::unique_ptr<RxWorker>       rx_worker_;
    std::atomic<uint64_t>           tx_count_{0};
    std::atomic<bool>               is_initialized_{false};
};

} // namespace robot::model

#endif // MODEL_MOTOR_CONTROLLER_HPP