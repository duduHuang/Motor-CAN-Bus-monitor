#include "model/socket_can_interface.hpp"

#include <cstring>
#include <iostream>
#include <syslog.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace model {

SocketCANInterface::~SocketCANInterface() noexcept {
    close();
}

bool SocketCANInterface::open(const std::string& interface_name) {
    if (is_open()) {
        close();
    }

    // 1. 建立 Socket (PF_CAN, SOCK_RAW, CAN_RAW)
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        syslog(LOG_ERR, "[SocketCANInterface] 建立 Socket 失敗: %s", std::strerror(errno));
        return false;
    }

    // 2. 設定非阻塞 (O_NONBLOCK) 模式，確保 recv/send 絕不阻塞 1000Hz 迴圈
    int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        syslog(LOG_ERR, "[SocketCANInterface] 設定 O_NONBLOCK 失敗: %s", std::strerror(errno));
        close();
        return false;
    }

    // 3. 加大接收緩衝區至 1MB (SO_RCVBUF)，抵抗 CPU Jitter 導致的 Kernel 丟封包
    int rcvbuf_size = 1024 * 1024; // 1MB
    if (::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
        syslog(LOG_WARNING, "[SocketCANInterface] 設定 SO_RCVBUF 失敗: %s", std::strerror(errno));
    }

    // 4. 禁用本地 TX 回環接收 (CAN_RAW_RECV_OWN_MSGS)，避免收到自己發出的封包浪費 CPU 週期
    int recv_own_msgs = 0;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
        syslog(LOG_ERR, "[SocketCANInterface] 禁用 CAN_RAW_RECV_OWN_MSGS 失敗: %s", std::strerror(errno));
        close();
        return false;
    }

    // 5. 設定硬體/Kernel 層級 CAN ID 濾波器 (CAN_RAW_FILTER)
    // 規則 1: 單機伺服回應 Base RX 0x240, Mask 0x7C0 (涵蓋 0x241 ~ 0x27F)
    // 規則 2: 運動模式回應 Base RX 0x500, Mask 0x7C0 (涵蓋 0x501 ~ 0x53F)
    struct can_filter rfilter[2];
    rfilter[0].can_id   = 0x240;
    rfilter[0].can_mask = 0x7C0;
    rfilter[1].can_id   = 0x500;
    rfilter[1].can_mask = 0x7C0;

    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        syslog(LOG_ERR, "[SocketCANInterface] 設定 CAN_RAW_FILTER 失敗: %s", std::strerror(errno));
        close();
        return false;
    }

    // 6. 取得網路介面 Index 並進行 Socket Bind
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        syslog(LOG_ERR, "[SocketCANInterface] 搜尋介面 %s 失敗: %s", interface_name.c_str(), std::strerror(errno));
        close();
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        syslog(LOG_ERR, "[SocketCANInterface] 綁定介面 %s (Index: %d) 失敗: %s", 
               interface_name.c_str(), ifr.ifr_ifindex, std::strerror(errno));
        close();
        return false;
    }

    syslog(LOG_INFO, "[SocketCANInterface] 成功開啟綁定 SocketCAN 介面: %s", interface_name.c_str());
    return true;
}

void SocketCANInterface::close() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SocketCANInterface::send_frame(uint32_t can_id, const std::array<uint8_t, 8>& payload) noexcept {
    if (fd_ < 0) [[unlikely]] {
        return false;
    }

    struct can_frame frame;
    frame.can_id  = can_id;
    frame.can_dlc = 8;
    std::memcpy(frame.data, payload.data(), 8);

    // MSG_DONTWAIT 雙重保障非阻塞寫入
    ssize_t nbytes = ::send(fd_, &frame, sizeof(struct can_frame), MSG_DONTWAIT);
    if (nbytes != sizeof(struct can_frame)) [[unlikely]] {
        // 當 EAGAIN 或 EWOULDBLOCK 時表示 Socket 緩衝區滿，直接丟棄/記錄並 return false，不阻塞 RT 迴圈
        return false;
    }

    return true;
}

bool SocketCANInterface::recv_frame(uint32_t& can_id, std::array<uint8_t, 8>& payload) noexcept {
    if (fd_ < 0) [[unlikely]] {
        return false;
    }

    struct can_frame frame;
    // MSG_DONTWAIT 保障當前無封包時立即 return -1 並設置 errno = EAGAIN
    ssize_t nbytes = ::recv(fd_, &frame, sizeof(struct can_frame), MSG_DONTWAIT);
    if (nbytes != sizeof(struct can_frame)) [[unlikely]] {
        return false;
    }

    can_id = frame.can_id;
    std::memcpy(payload.data(), frame.data, 8);
    return true;
}

} // namespace model