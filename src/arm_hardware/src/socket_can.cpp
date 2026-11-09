#include "muweidu_ts/socket_can.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <climits>
#include <cstring>
#include <stdexcept>
#include <system_error>

namespace muweidu::ts {
namespace {

std::system_error systemError(const std::string& operation) {
    return std::system_error(errno, std::generic_category(), operation);
}

int remainingMilliseconds(std::chrono::steady_clock::time_point deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
        return 0;
    }
    const auto remaining = deadline - now;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
    if (ms < remaining) {
        ++ms;
    }
    return ms.count() > INT_MAX ? INT_MAX : static_cast<int>(ms.count());
}

void waitFor(int fd, short events, std::chrono::steady_clock::time_point deadline,
             const char* operation) {
    while (true) {
        const int wait_ms = remainingMilliseconds(deadline);
        if (wait_ms == 0) {
            throw std::runtime_error(std::string("timed out ") + operation + " SocketCAN frame");
        }

        pollfd descriptor{};
        descriptor.fd = fd;
        descriptor.events = events;
        int result;
        do {
            result = ::poll(&descriptor, 1, wait_ms);
        } while (result < 0 && errno == EINTR);
        if (result < 0) {
            throw systemError(std::string("poll(SocketCAN ") + operation + ")");
        }
        if (result == 0) {
            throw std::runtime_error(std::string("timed out ") + operation + " SocketCAN frame");
        }
        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            throw std::runtime_error(std::string("SocketCAN ") + operation + " reported an interface error");
        }
        if ((descriptor.revents & events) != 0) {
            return;
        }
    }
}

}  // namespace

SocketCan::SocketCan(const std::string& interface_name)
    : interface_name_(interface_name) {
    if (interface_name_.empty()) {
        throw std::invalid_argument("CAN interface name must not be empty");
    }

    errno = 0;
    const unsigned int interface_index = ::if_nametoindex(interface_name_.c_str());
    if (interface_index == 0) {
        if (errno == 0) {
            errno = ENODEV;
        }
        throw systemError("if_nametoindex(" + interface_name_ + ")");
    }

    fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, CAN_RAW);
    if (fd_ < 0) {
        throw systemError("socket(PF_CAN)");
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(interface_index);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        const auto error = systemError("bind(SocketCAN " + interface_name_ + ")");
        ::close(fd_);
        fd_ = -1;
        throw error;
    }
}

SocketCan::~SocketCan() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

void SocketCan::sendFrame(
    const CanFrame& frame,
    std::chrono::steady_clock::time_point deadline) {
    if (frame.id > CAN_SFF_MASK) {
        throw std::invalid_argument("SocketCAN transport accepts standard 11-bit CAN IDs only");
    }
    if (frame.length == 0 || frame.length > CAN_MAX_DLEN) {
        throw std::invalid_argument("classic CAN data length must be in the range 1..8");
    }

    can_frame outgoing{};
    outgoing.can_id = frame.id;
    outgoing.len = frame.length;
    std::memcpy(outgoing.data, frame.data.data(), frame.length);

    while (true) {
        const ssize_t sent = ::send(fd_, &outgoing, sizeof(outgoing), MSG_NOSIGNAL);
        if (sent == static_cast<ssize_t>(sizeof(outgoing))) {
            return;
        }
        if (sent >= 0) {
            throw std::runtime_error("SocketCAN sent a partial CAN frame");
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            waitFor(fd_, POLLOUT, deadline, "sending");
            continue;
        }
        throw systemError("send(SocketCAN " + interface_name_ + ")");
    }
}

CanFrame SocketCan::readMatchingFrame(
    std::uint32_t expected_reply_id,
    std::uint8_t expected_command,
    std::chrono::steady_clock::time_point deadline) {
    while (true) {
        waitFor(fd_, POLLIN, deadline, "waiting for");

        can_frame incoming{};
        const ssize_t received = ::recv(fd_, &incoming, sizeof(incoming), 0);
        if (received < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            throw systemError("recv(SocketCAN " + interface_name_ + ")");
        }
        if (received != static_cast<ssize_t>(sizeof(incoming))) {
            continue;
        }

        // Ignore error, remote, and extended frames. This driver speaks classic
        // CAN standard data frames only.
        if ((incoming.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) != 0 ||
            incoming.len != 8 || incoming.can_id != expected_reply_id ||
            incoming.data[0] != expected_command) {
            continue;
        }

        CanFrame response{};
        response.id = incoming.can_id & CAN_SFF_MASK;
        response.length = incoming.len;
        std::memcpy(response.data.data(), incoming.data, response.length);
        return response;
    }
}

CanFrame SocketCan::transact(
    const CanFrame& request,
    std::uint32_t expected_reply_id,
    std::uint8_t expected_command,
    std::chrono::milliseconds timeout) {
    if (request.id > CAN_SFF_MASK || expected_reply_id > CAN_SFF_MASK) {
        throw std::invalid_argument("SocketCAN transport accepts standard 11-bit CAN IDs only");
    }
    if (request.length == 0 || request.length > CAN_MAX_DLEN) {
        throw std::invalid_argument("CAN command data length must be in the range 1..8");
    }
    if (request.data[0] != expected_command) {
        throw std::invalid_argument("expected reply command must match request command");
    }
    if (timeout.count() < 0) {
        throw std::invalid_argument("timeout must not be negative");
    }

    std::lock_guard<std::mutex> lock(transaction_mutex_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    sendFrame(request, deadline);
    return readMatchingFrame(expected_reply_id, expected_command, deadline);
}

}  // namespace muweidu::ts
