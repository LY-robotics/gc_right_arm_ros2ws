#pragma once

#include "muweidu_ts/can_frame.hpp"

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

namespace muweidu::ts {

// Linux SocketCAN transport for classic CAN standard frames.
// The CAN interface (for example, can2) must already be configured and up.
class SocketCan {
public:
    explicit SocketCan(const std::string& interface_name);
    ~SocketCan();

    SocketCan(const SocketCan&) = delete;
    SocketCan& operator=(const SocketCan&) = delete;

    CanFrame transact(
        const CanFrame& request,
        std::uint32_t expected_reply_id,
        std::uint8_t expected_command,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{50});

    const std::string& interfaceName() const noexcept { return interface_name_; }

private:
    void sendFrame(const CanFrame& frame, std::chrono::steady_clock::time_point deadline);
    CanFrame readMatchingFrame(
        std::uint32_t expected_reply_id,
        std::uint8_t expected_command,
        std::chrono::steady_clock::time_point deadline);

    int fd_{-1};
    std::string interface_name_;
    std::mutex transaction_mutex_;
};

}  // namespace muweidu::ts
