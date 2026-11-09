#ifndef ARM_HARDWARE__SYNCHRONIZED_SPEED_HPP_
#define ARM_HARDWARE__SYNCHRONIZED_SPEED_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace arm_hardware
{

inline constexpr long double kRadiansToDegrees =
  180.0L / 3.141592653589793238462643383279502884L;

// Convert signed trajectory velocities from rad/s to six joint caps in degrees/second.
inline std::array<double, 6> mappedJointSpeedLimits(
  const std::array<double, 6> & velocity_rad_s,
  const std::array<double, 6> & limits_dps,
  double scale,
  double offset_dps)
{
  if (!std::isfinite(scale) || scale <= 0.0 || scale > 1.0) {
    throw std::invalid_argument("joint speed scale must be in (0, 1]");
  }
  if (!std::isfinite(offset_dps) || offset_dps <= 0.0) {
    throw std::invalid_argument("joint speed offset must be finite and positive");
  }
  std::array<double, 6> mapped_limits{};
  for (std::size_t i = 0; i < mapped_limits.size(); ++i) {
    if (!std::isfinite(limits_dps[i]) || limits_dps[i] < offset_dps) {
      throw std::invalid_argument("joint speed limits must be finite and at least the offset");
    }
    if (!std::isfinite(velocity_rad_s[i])) {
      throw std::invalid_argument("joint velocity commands must be finite");
    }
    const long double magnitude = std::abs(static_cast<long double>(velocity_rad_s[i]));
    const long double mapped = scale * magnitude * kRadiansToDegrees + offset_dps;
    mapped_limits[i] = static_cast<double>(
      std::min(mapped, static_cast<long double>(limits_dps[i])));
  }
  return mapped_limits;
}

// Positions are radians. The per-joint limits and result are degrees/second.
// All motor speeds use one duration, also constrained by the 0xA4 uint16 limit.
inline std::array<std::uint16_t, 6> synchronizedMotorSpeeds(
  const std::array<double, 6> & current_joint_rad,
  const std::array<double, 6> & target_joint_rad,
  const std::array<double, 6> & current_motor_rad,
  const std::array<double, 6> & target_motor_rad,
  const std::array<double, 6> & joint_speed_limits_dps)
{
  constexpr long double kMaxSpeed = std::numeric_limits<std::uint16_t>::max();
  std::array<long double, 6> motor_delta_deg{};
  long double duration = 0.0L;
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(joint_speed_limits_dps[i]) || joint_speed_limits_dps[i] <= 0.0) {
      throw std::invalid_argument("joint speed limits must be finite and positive");
    }
    if (!std::isfinite(current_joint_rad[i]) || !std::isfinite(target_joint_rad[i]) ||
      !std::isfinite(current_motor_rad[i]) || !std::isfinite(target_motor_rad[i]))
    {
      throw std::invalid_argument("positions must be finite");
    }
    const auto joint_delta = (static_cast<long double>(target_joint_rad[i]) -
      static_cast<long double>(current_joint_rad[i])) * kRadiansToDegrees;
    motor_delta_deg[i] = std::abs((static_cast<long double>(target_motor_rad[i]) -
      static_cast<long double>(current_motor_rad[i])) * kRadiansToDegrees);
    if (!std::isfinite(joint_delta) || !std::isfinite(motor_delta_deg[i])) {
      throw std::out_of_range("position difference is outside the calculation range");
    }
    duration = std::max(duration, std::abs(joint_delta) / joint_speed_limits_dps[i]);
    duration = std::max(duration, motor_delta_deg[i] / kMaxSpeed);
  }
  std::array<std::uint16_t, 6> speeds{};
  if (duration == 0.0L) {
    speeds.fill(1);
    return speeds;
  }
  if (!std::isfinite(duration)) {
    throw std::out_of_range("movement duration is outside the calculation range");
  }
  for (std::size_t i = 0; i < 6; ++i) {
    const long double speed = motor_delta_deg[i] / duration;
    // Allow only floating-point roundoff at an integer boundary. Inputs originate as doubles.
    const long double tolerance = 4.0L * std::numeric_limits<double>::epsilon() *
      std::max(1.0L, std::abs(speed));
    const long double quantized = std::floor(speed + tolerance);
    speeds[i] = static_cast<std::uint16_t>(std::clamp(quantized, 1.0L, kMaxSpeed));
  }
  return speeds;
}

inline std::array<std::uint16_t, 6> synchronizedMotorSpeeds(
  const std::array<double, 6> & current_joint_rad,
  const std::array<double, 6> & target_joint_rad,
  const std::array<double, 6> & current_motor_rad,
  const std::array<double, 6> & target_motor_rad,
  double joint_speed_limit_dps)
{
  std::array<double, 6> limits{};
  limits.fill(joint_speed_limit_dps);
  return synchronizedMotorSpeeds(
    current_joint_rad, target_joint_rad, current_motor_rad, target_motor_rad, limits);
}

}  // namespace arm_hardware

#endif  // ARM_HARDWARE__SYNCHRONIZED_SPEED_HPP_
