#pragma once

#include "muweidu_ts/socket_can.hpp"

#include <array>
#include <cstdint>
#include <chrono>

namespace muweidu::ts {

struct Status1 {
    std::int8_t temperature_c{};
    std::uint16_t bus_voltage_raw{};  // 0.01 V/bit per protocol document
    std::uint16_t bus_current_raw{};  // 0.01 A/bit per protocol document
    std::uint8_t motor_state{};       // 0 = running, 0x10 = off
    std::uint8_t fault_flags{};       // Documented: bit 0 undervoltage, bit 3 overtemperature

    double busVoltageV() const noexcept { return bus_voltage_raw * 0.01; }
    double busCurrentA() const noexcept { return bus_current_raw * 0.01; }
    bool undervoltage() const noexcept { return (fault_flags & 0x01U) != 0; }
    bool overtemperature() const noexcept { return (fault_flags & 0x08U) != 0; }
};

struct Status2 {
    std::int8_t temperature_c{};
    std::int16_t iq_or_power_raw{};  // Signed protocol value; torque conversion is motor-specific.
    std::int16_t speed_raw{};        // Signed degrees/second per the protocol document.
    std::uint16_t encoder_raw{};
};

struct PhaseCurrentsRaw {
    std::int8_t temperature_c{};
    std::uint16_t phase_a{};
    std::uint16_t phase_b{};
    std::uint16_t phase_c{};
};

struct EncoderRaw {
    std::uint16_t current_value{};
    std::uint16_t raw_value{};
    std::uint16_t zero_offset{};
};

struct PidGains {
    std::uint8_t position_kp{};
    std::uint8_t position_ki{};
    std::uint8_t speed_kp{};
    std::uint8_t speed_ki{};
    std::uint8_t torque_kp{};
    std::uint8_t torque_ki{};
};

enum class Direction : std::uint8_t {
    clockwise = 0,
    counter_clockwise = 1,
};

class Motor {
public:
    Motor(SocketCan& can, std::uint8_t motor_id);

    std::uint8_t id() const noexcept { return motor_id_; }
    std::uint32_t canId() const noexcept { return can_id_; }

    Status1 readStatus1(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    Status2 readStatus2(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    PhaseCurrentsRaw readPhaseCurrentsRaw(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    EncoderRaw readEncoderRaw(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    std::int64_t readMultiTurnAngleCentiDegrees(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    std::uint32_t readSingleLoopAngleRaw(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    PidGains readPid(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    std::int32_t readMaxAcceleration(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});

    void clearFault(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    void motorOn(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    void motorOff(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    void emergencyStop(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    void setBrake(bool locked, std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    bool queryBrake(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});

    // Control values follow protocol field definitions. Torque remains a raw
    // proportional command; its conversion to N.m is motor-specific/undocumented.
    Status2 setTorqueRaw(std::int16_t iq_control, std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    Status2 setSpeedCentiDps(std::int32_t speed_centi_dps, std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    Status2 setMultiTurnPositionCentiDegrees(std::int32_t target_centi_degrees, std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    Status2 setMultiTurnPositionWithSpeed(
        std::uint16_t max_speed_dps,
        std::int32_t target_centi_degrees,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{50});

    // The protocol gives no explicit target unit for single-turn and incremental
    // commands. These methods accept the raw signed/unsigned field value.
    Status2 setSingleTurnPositionRaw(
        Direction direction,
        std::uint32_t target_raw,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    Status2 setSingleTurnPositionWithSpeedRaw(
        Direction direction,
        std::uint16_t max_speed_dps,
        std::uint32_t target_raw,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    Status2 setIncrementalPositionRaw(std::int32_t increment_raw, std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    Status2 setIncrementalPositionWithSpeedRaw(
        std::uint16_t max_speed_dps,
        std::int32_t increment_raw,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{50});

    void setEncoderZero(std::uint16_t zero_offset, std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    std::uint16_t setCurrentPositionAsZero(std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    void writePidToRam(const PidGains& gains, std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    void writePidToRom(const PidGains& gains, std::chrono::milliseconds timeout = std::chrono::milliseconds{50});
    void writeAccelerationToRam(std::int32_t acceleration_dps_per_s, std::chrono::milliseconds timeout = std::chrono::milliseconds{50});

private:
    CanFrame exchange(
        const std::array<std::uint8_t, 8>& data,
        std::chrono::milliseconds timeout);
    Status2 parseStatus2(const CanFrame& response, std::uint8_t expected_command) const;

    SocketCan& can_;
    std::uint8_t motor_id_;
    std::uint32_t can_id_;
};

}  // namespace muweidu::ts
