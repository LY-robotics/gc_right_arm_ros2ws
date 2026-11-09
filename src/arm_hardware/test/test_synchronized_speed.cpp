#include "arm_hardware/synchronized_speed.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

using Angles = std::array<double, 6>;
using arm_hardware::mappedJointSpeedLimits;
using arm_hardware::synchronizedMotorSpeeds;

namespace
{
constexpr double kDeg = 3.14159265358979323846 / 180.0;

template<typename Function>
void expectException(Function function)
{
  bool caught = false;
  try {function();} catch (const std::exception &) {caught = true;}
  assert(caught);
}
}

int main()
{
  const Angles zero{};
  const Angles velocity_rad_s{0, -5 * kDeg, 5 * kDeg, -20 * kDeg, 2 * kDeg, 100 * kDeg};
  const Angles limits_dps{10, 11, 6, 7, 4, 8};
  const auto mapped = mappedJointSpeedLimits(velocity_rad_s, limits_dps, 0.5, 2.0);
  const Angles expected{2, 4.5, 4.5, 7, 3, 8};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    assert(std::abs(mapped[i] - expected[i]) < 1e-12);
  }
  const Angles default_velocity_rad_s{0, -5 * kDeg, 10 * kDeg, -20 * kDeg, 30 * kDeg, 40 * kDeg};
  const Angles default_limits_dps{30, 30, 30, 30, 30, 30};
  const auto default_mapped =
    mappedJointSpeedLimits(default_velocity_rad_s, default_limits_dps, 0.6, 10.0);
  const Angles expected_default{10, 13, 16, 22, 28, 30};
  for (std::size_t i = 0; i < expected_default.size(); ++i) {
    assert(std::abs(default_mapped[i] - expected_default[i]) < 1e-12);
  }
  expectException([&] {mappedJointSpeedLimits(velocity_rad_s, limits_dps, 0.0, 2.0);});
  expectException([&] {mappedJointSpeedLimits(velocity_rad_s, limits_dps, 1.1, 2.0);});
  expectException([&] {mappedJointSpeedLimits(velocity_rad_s, limits_dps, 0.5, 0.0);});
  expectException([&] {mappedJointSpeedLimits(velocity_rad_s, limits_dps, 0.5, 5.0);});
  Angles invalid_velocity = velocity_rad_s;
  invalid_velocity[5] = std::numeric_limits<double>::quiet_NaN();
  expectException([&] {mappedJointSpeedLimits(invalid_velocity, limits_dps, 0.5, 2.0);});
  invalid_velocity[5] = std::numeric_limits<double>::infinity();
  expectException([&] {mappedJointSpeedLimits(invalid_velocity, limits_dps, 0.5, 2.0);});
  Angles invalid_limits = limits_dps;
  invalid_limits[5] = std::numeric_limits<double>::infinity();
  expectException([&] {mappedJointSpeedLimits(velocity_rad_s, invalid_limits, 0.5, 2.0);});

  Angles joints{1.0 * kDeg, 0, 0, 0, 0, 0};
  Angles motors{-100.8 * kDeg, 0, 0, 0, 0, 0};
  auto speeds = synchronizedMotorSpeeds(zero, joints, zero, motors, 100.0);
  assert(speeds[0] == 10080);
  assert(speeds[1] == 1);

  // q2 = 10 degrees, q3 = 5 degrees: m2 = 144 degrees, m3 = -576 degrees.
  joints = {0, 10 * kDeg, 5 * kDeg, 0, 0, 0};
  motors = {0, 144 * kDeg, -576 * kDeg, 0, 0, 0};
  speeds = synchronizedMotorSpeeds(zero, joints, zero, motors, 10.0);
  assert(speeds[1] == 144 && speeds[2] == 576);
  // Joint3 has smaller travel, but its tighter cap sets the duration of both motors.
  const Angles independent_limits{10, 20, 2.5, 10, 10, 10};
  speeds = synchronizedMotorSpeeds(zero, joints, zero, motors, independent_limits);
  assert(speeds[1] == 72 && speeds[2] == 288);
  // With q2 = 1.2 q3, m2 cancels and must still receive valid speed 1.
  joints = {0, 6 * kDeg, 5 * kDeg, 0, 0, 0};
  motors = {0, 0, -432 * kDeg, 0, 0, 0};
  speeds = synchronizedMotorSpeeds(zero, joints, zero, motors, 10.0);
  assert(speeds[1] == 1 && speeds[2] == 720);
  speeds = synchronizedMotorSpeeds(zero, zero, zero, zero, 100.0);
  for (const auto speed : speeds) {assert(speed == 1);}

  // The protocol maximum lengthens the common duration for every motor.
  joints = {1 * kDeg, 0, 0, 0, 0, 0};
  motors = {1000 * kDeg, 500 * kDeg, 0, 0, 0, 0};
  speeds = synchronizedMotorSpeeds(zero, joints, zero, motors, 100.0);
  assert(speeds[0] == 65535 && speeds[1] == 32767);
  motors = {100 * kDeg, 0.001 * kDeg, 0, 0, 0, 0};
  speeds = synchronizedMotorSpeeds(zero, joints, zero, motors, 1.0);
  assert(speeds[0] == 100 && speeds[1] == 1);
  expectException([&] {synchronizedMotorSpeeds(zero, joints, zero, motors, invalid_limits);});
  expectException([&] {synchronizedMotorSpeeds(zero, joints, zero, motors, 0.0);});
  expectException([&] {synchronizedMotorSpeeds(zero, joints, zero, motors, -1.0);});
  expectException([&] {synchronizedMotorSpeeds(zero, joints, zero, motors,
      std::numeric_limits<double>::infinity());});
  joints[0] = std::numeric_limits<double>::quiet_NaN();
  expectException([&] {synchronizedMotorSpeeds(zero, joints, zero, motors, 1.0);});
  motors[0] = std::numeric_limits<double>::infinity();
  expectException([&] {synchronizedMotorSpeeds(zero, zero, zero, motors, 1.0);});
}
