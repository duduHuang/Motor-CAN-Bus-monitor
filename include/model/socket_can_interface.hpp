#ifndef MODEL_SOCKET_CAN_INTERFACE_HPP_
#define MODEL_SOCKET_CAN_INTERFACE_HPP_

#include <array>
#include <cstdint>
#include <string>

// 修正：將 namespace 改為 robot::model 與專案其他檔案對齊
namespace robot::model {

/**
 * @brief 高效能、零動態記憶體分配 (Zero-Allocation)、非阻塞 (Zero-Blocking) SocketCAN 驅動類別
 * @details 專為 PREEMPT_RT 即時核心與 1000Hz (1ms) 控制迴圈設計。
 */
class SocketCANInterface {
public:
    SocketCANInterface() noexcept = default;
    ~SocketCANInterface() noexcept;

    // 嚴格禁止複製與移動，避免 File Descriptor 被重複釋放或引發未定義行為
    SocketCANInterface(const SocketCANInterface&) = delete;
    SocketCANInterface& operator=(const SocketCANInterface&) = delete;
    SocketCANInterface(SocketCANInterface&&) noexcept = delete;
    SocketCANInterface& operator=(SocketCANInterface&&) noexcept = delete;

    /**
     * @brief 初始化套接字，配置即時套接字選項與 Kernel 濾波器，並綁定 CAN 介面
     * @param interface_name 網卡名稱 (例: "can0", "vcan0")
     * @return true 啟用成功；false 啟用失敗 (詳細原因將輸出至 syslog)
     */
    [[nodiscard]] bool open(const std::string& interface_name);

    /**
     * @brief 安全關閉 Socket File Descriptor
     */
    void close() noexcept;

    /**
     * @brief 非阻塞發送 CAN 封包 (1000Hz 關鍵路徑, 零動態配置, 不阻塞)
     * @param can_id 標頭 ID (標準 11-bit CAN ID)
     * @param payload 8-Byte 數據載荷
     * @return true 發送成功；false 寫入失敗或 Socket TX Buffer 滿載
     */
    [[nodiscard]] bool send_frame(uint32_t can_id, const std::array<uint8_t, 8>& payload) noexcept;

    /**
     * @brief 非阻塞接收 CAN 封包 (1000Hz 關鍵路徑, 零動態配置, 不阻塞)
     * @param[out] can_id 接收到的 CAN ID
     * @param[out] payload 接收到的 8-Byte 數據載荷
     * @return true 成功讀取封包；false 當前沒有無封包可讀或讀取異常
     */
    [[nodiscard]] bool recv_frame(uint32_t& can_id, std::array<uint8_t, 8>& payload) noexcept;

    /**
     * @brief 檢查 Socket 是否成功開啟
     */
    [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }

private:
    int fd_{-1};
};

} // namespace robot::model

#endif // MODEL_SOCKET_CAN_INTERFACE_HPP_