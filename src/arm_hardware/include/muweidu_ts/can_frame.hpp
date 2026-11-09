#pragma once

#include <array>
#include <cstdint>

namespace muweidu::ts {

// Classic CAN data frame. The transport accepts standard 11-bit identifiers.
struct CanFrame {
    std::uint32_t id{};
    std::array<std::uint8_t, 8> data{};
    std::uint8_t length{8};
};

}  // namespace muweidu::ts
