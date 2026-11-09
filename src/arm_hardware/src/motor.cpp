#include "muweidu_ts/motor.hpp"

#include <stdexcept>

namespace muweidu::ts {
namespace {

constexpr std::uint8_t kReadStatus1 = 0x9A;
constexpr std::uint8_t kClearFault = 0x9B;
constexpr std::uint8_t kReadStatus2 = 0x9C;
constexpr std::uint8_t kReadPhaseCurrents = 0x9D;
constexpr std::uint8_t kMotorOff = 0x80;
constexpr std::uint8_t kEmergencyStop = 0x81;
constexpr std::uint8_t kMotorOn = 0x88;
constexpr std::uint8_t kBrake = 0x8C;
constexpr std::uint8_t kReadEncoder = 0x90;
constexpr std::uint8_t kSetEncoderZero = 0x91;
constexpr std::uint8_t kReadMultiTurnAngle = 0x92;
constexpr std::uint8_t kReadSingleLoopAngle = 0x94;
constexpr std::uint8_t kSetCurrentPositionZero = 0x19;
constexpr std::uint8_t kReadPid = 0x30;
constexpr std::uint8_t kWritePidRam = 0x31;
constexpr std::uint8_t kWritePidRom = 0x32;
constexpr std::uint8_t kReadAcceleration = 0x33;
constexpr std::uint8_t kWriteAccelerationRam = 0x34;
constexpr std::uint8_t kTorqueControl = 0xA1;
constexpr std::uint8_t kSpeedControl = 0xA2;
constexpr std::uint8_t kMultiTurnPosition = 0xA3;
constexpr std::uint8_t kMultiTurnPositionWithSpeed = 0xA4;
constexpr std::uint8_t kSingleTurnPosition = 0xA5;
constexpr std::uint8_t kSingleTurnPositionWithSpeed = 0xA6;
constexpr std::uint8_t kIncrementalPosition = 0xA7;
constexpr std::uint8_t kIncrementalPositionWithSpeed = 0xA8;

std::uint16_t u16le(const std::array<std::uint8_t, 8>& data, std::size_t offset) {
    return static_cast<std::uint16_t>(data[offset]) |
           (static_cast<std::uint16_t>(data[offset + 1]) << 8U);
}

std::int16_t i16le(const std::array<std::uint8_t, 8>& data, std::size_t offset) {
    const std::uint16_t value = u16le(data, offset);
    const std::int32_t decoded = value < 0x8000U
        ? static_cast<std::int32_t>(value)
        : static_cast<std::int32_t>(value) - 0x10000;
    return static_cast<std::int16_t>(decoded);
}

std::uint32_t u32le(const std::array<std::uint8_t, 8>& data, std::size_t offset) {
    return static_cast<std::uint32_t>(data[offset]) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 8U) |
           (static_cast<std::uint32_t>(data[offset + 2]) << 16U) |
           (static_cast<std::uint32_t>(data[offset + 3]) << 24U);
}

std::int8_t i8(std::uint8_t value) {
    const int decoded = value < 0x80U ? static_cast<int>(value) : static_cast<int>(value) - 0x100;
    return static_cast<std::int8_t>(decoded);
}

std::int32_t i32le(const std::array<std::uint8_t, 8>& data, std::size_t offset) {
    const std::uint32_t value = u32le(data, offset);
    const std::int64_t decoded = value < 0x80000000U
        ? static_cast<std::int64_t>(value)
        : static_cast<std::int64_t>(value) - 0x100000000LL;
    return static_cast<std::int32_t>(decoded);
}

std::int64_t i56le(const std::array<std::uint8_t, 8>& data, std::size_t offset) {
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < 7; ++i) {
        value |= static_cast<std::uint64_t>(data[offset + i]) << (8U * i);
    }
    if ((value & (std::uint64_t{1} << 55U)) != 0) {
        return static_cast<std::int64_t>(value) - (std::int64_t{1} << 56U);
    }
    return static_cast<std::int64_t>(value);
}

void putU16(std::array<std::uint8_t, 8>& data, std::size_t offset, std::uint16_t value) {
    data[offset] = static_cast<std::uint8_t>(value & 0xFFU);
    data[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

void putU32(std::array<std::uint8_t, 8>& data, std::size_t offset, std::uint32_t value) {
    for (std::size_t i = 0; i < 4; ++i) {
        data[offset + i] = static_cast<std::uint8_t>((value >> (8U * i)) & 0xFFU);
    }
}

void putI16(std::array<std::uint8_t, 8>& data, std::size_t offset, std::int16_t value) {
    putU16(data, offset, static_cast<std::uint16_t>(value));
}

void putI32(std::array<std::uint8_t, 8>& data, std::size_t offset, std::int32_t value) {
    putU32(data, offset, static_cast<std::uint32_t>(value));
}

std::uint8_t directionValue(Direction direction) {
    if (direction != Direction::clockwise && direction != Direction::counter_clockwise) {
        throw std::invalid_argument("direction must be clockwise or counter-clockwise");
    }
    return static_cast<std::uint8_t>(direction);
}

}  // namespace

Motor::Motor(SocketCan& can, std::uint8_t motor_id)
    : can_(can), motor_id_(motor_id), can_id_(0x140U + motor_id) {
    if (motor_id_ < 1 || motor_id_ > 32) {
        throw std::invalid_argument("motor ID must be in the range 1..32");
    }
}

CanFrame Motor::exchange(
    const std::array<std::uint8_t, 8>& data,
    std::chrono::milliseconds timeout) {
    CanFrame request{};
    request.id = can_id_;
    request.data = data;
    request.length = static_cast<std::uint8_t>(data.size());
    return can_.transact(request, can_id_, data[0], timeout);
}

Status1 Motor::readStatus1(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kReadStatus1;
    const auto response = exchange(data, timeout).data;
    Status1 status{};
    status.temperature_c = i8(response[1]);
    status.bus_voltage_raw = u16le(response, 2);
    status.bus_current_raw = u16le(response, 4);
    status.motor_state = response[6];
    status.fault_flags = response[7];
    return status;
}

Status2 Motor::parseStatus2(const CanFrame& response, std::uint8_t expected_command) const {
    if (response.data[0] != expected_command) {
        throw std::runtime_error("unexpected command in status-2 reply");
    }
    Status2 status{};
    status.temperature_c = i8(response.data[1]);
    status.iq_or_power_raw = i16le(response.data, 2);
    status.speed_raw = i16le(response.data, 4);
    status.encoder_raw = u16le(response.data, 6);
    return status;
}

Status2 Motor::readStatus2(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kReadStatus2;
    return parseStatus2(exchange(data, timeout), kReadStatus2);
}

PhaseCurrentsRaw Motor::readPhaseCurrentsRaw(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kReadPhaseCurrents;
    const auto response = exchange(data, timeout).data;
    return {i8(response[1]), u16le(response, 2), u16le(response, 4), u16le(response, 6)};
}

EncoderRaw Motor::readEncoderRaw(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kReadEncoder;
    const auto response = exchange(data, timeout).data;
    return {u16le(response, 2), u16le(response, 4), u16le(response, 6)};
}

std::int64_t Motor::readMultiTurnAngleCentiDegrees(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kReadMultiTurnAngle;
    const auto response = exchange(data, timeout).data;
    return i56le(response, 1);
}

std::uint32_t Motor::readSingleLoopAngleRaw(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kReadSingleLoopAngle;
    const auto response = exchange(data, timeout).data;
    return u32le(response, 4);
}

PidGains Motor::readPid(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kReadPid;
    const auto response = exchange(data, timeout).data;
    return {response[2], response[3], response[4], response[5], response[6], response[7]};
}

std::int32_t Motor::readMaxAcceleration(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kReadAcceleration;
    const auto response = exchange(data, timeout).data;
    return i32le(response, 4);
}

void Motor::clearFault(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kClearFault;
    exchange(data, timeout);
}

void Motor::motorOn(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kMotorOn;
    exchange(data, timeout);
}

void Motor::motorOff(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kMotorOff;
    exchange(data, timeout);
}

void Motor::emergencyStop(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kEmergencyStop;
    exchange(data, timeout);
}

void Motor::setBrake(bool locked, std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kBrake;
    data[1] = locked ? 0x00U : 0x01U;
    exchange(data, timeout);
}

bool Motor::queryBrake(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kBrake;
    data[1] = 0x10U;
    const auto state = exchange(data, timeout).data[1];
    if (state > 1U) {
        throw std::runtime_error("invalid brake state in CAN reply");
    }
    return state == 1U;
}

Status2 Motor::setTorqueRaw(std::int16_t iq_control, std::chrono::milliseconds timeout) {
    if (iq_control < -2048 || iq_control > 2048) {
        throw std::out_of_range("torque command must be in the range -2048..2048");
    }
    std::array<std::uint8_t, 8> data{};
    data[0] = kTorqueControl;
    putI16(data, 4, iq_control);
    return parseStatus2(exchange(data, timeout), kTorqueControl);
}

Status2 Motor::setSpeedCentiDps(std::int32_t speed_centi_dps, std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kSpeedControl;
    putI32(data, 4, speed_centi_dps);
    return parseStatus2(exchange(data, timeout), kSpeedControl);
}

Status2 Motor::setMultiTurnPositionCentiDegrees(
    std::int32_t target_centi_degrees,
    std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kMultiTurnPosition;
    putI32(data, 4, target_centi_degrees);
    return parseStatus2(exchange(data, timeout), kMultiTurnPosition);
}

Status2 Motor::setMultiTurnPositionWithSpeed(
    std::uint16_t max_speed_dps,
    std::int32_t target_centi_degrees,
    std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kMultiTurnPositionWithSpeed;
    putU16(data, 2, max_speed_dps);
    putI32(data, 4, target_centi_degrees);
    return parseStatus2(exchange(data, timeout), kMultiTurnPositionWithSpeed);
}

Status2 Motor::setSingleTurnPositionRaw(
    Direction direction,
    std::uint32_t target_raw,
    std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kSingleTurnPosition;
    data[1] = directionValue(direction);
    putU32(data, 4, target_raw);
    return parseStatus2(exchange(data, timeout), kSingleTurnPosition);
}

Status2 Motor::setSingleTurnPositionWithSpeedRaw(
    Direction direction,
    std::uint16_t max_speed_dps,
    std::uint32_t target_raw,
    std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kSingleTurnPositionWithSpeed;
    data[1] = directionValue(direction);
    putU16(data, 2, max_speed_dps);
    putU32(data, 4, target_raw);
    return parseStatus2(exchange(data, timeout), kSingleTurnPositionWithSpeed);
}

Status2 Motor::setIncrementalPositionRaw(
    std::int32_t increment_raw,
    std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kIncrementalPosition;
    putI32(data, 4, increment_raw);
    return parseStatus2(exchange(data, timeout), kIncrementalPosition);
}

Status2 Motor::setIncrementalPositionWithSpeedRaw(
    std::uint16_t max_speed_dps,
    std::int32_t increment_raw,
    std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kIncrementalPositionWithSpeed;
    putU16(data, 2, max_speed_dps);
    putI32(data, 4, increment_raw);
    return parseStatus2(exchange(data, timeout), kIncrementalPositionWithSpeed);
}

void Motor::setEncoderZero(std::uint16_t zero_offset, std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kSetEncoderZero;
    putU16(data, 6, zero_offset);
    exchange(data, timeout);
}

std::uint16_t Motor::setCurrentPositionAsZero(std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kSetCurrentPositionZero;
    return u16le(exchange(data, timeout).data, 6);
}

void Motor::writePidToRam(const PidGains& gains, std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kWritePidRam;
    data[2] = gains.position_kp;
    data[3] = gains.position_ki;
    data[4] = gains.speed_kp;
    data[5] = gains.speed_ki;
    data[6] = gains.torque_kp;
    data[7] = gains.torque_ki;
    exchange(data, timeout);
}

void Motor::writePidToRom(const PidGains& gains, std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kWritePidRom;
    data[2] = gains.position_kp;
    data[3] = gains.position_ki;
    data[4] = gains.speed_kp;
    data[5] = gains.speed_ki;
    data[6] = gains.torque_kp;
    data[7] = gains.torque_ki;
    exchange(data, timeout);
}

void Motor::writeAccelerationToRam(
    std::int32_t acceleration_dps_per_s,
    std::chrono::milliseconds timeout) {
    std::array<std::uint8_t, 8> data{};
    data[0] = kWriteAccelerationRam;
    putI32(data, 4, acceleration_dps_per_s);
    exchange(data, timeout);
}

}  // namespace muweidu::ts
