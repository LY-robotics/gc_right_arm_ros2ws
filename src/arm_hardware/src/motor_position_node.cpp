#include "muweidu_ts/motor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("motor_position");

  try {
    auto node = std::make_shared<rclcpp::Node>("motor_position");
    const auto can_interface = node->declare_parameter<std::string>("can_interface", "can2");
    const auto motor_id = node->declare_parameter<std::int64_t>("motor_id", 2);
    const auto target_degrees = node->declare_parameter<double>("target_degrees", 0.0);
    const auto max_speed_dps = node->declare_parameter<std::int64_t>("max_speed_dps", 0);
    const auto enable_before_command = node->declare_parameter<bool>("motor_on_before_command", true);
    const auto timeout_ms = node->declare_parameter<std::int64_t>("timeout_ms", 100);

    if (can_interface != "can0" && can_interface != "can1" &&
      can_interface != "can2" && can_interface != "can3")
    {
      throw std::invalid_argument("can_interface must be can0, can1, can2, or can3");
    }
    if (motor_id < 1 || motor_id > 32) {
      throw std::invalid_argument("motor_id must be in the protocol range 1..32");
    }
    if (!std::isfinite(target_degrees)) {
      throw std::invalid_argument("target_degrees must be a finite number");
    }
    if (max_speed_dps < 1 || max_speed_dps > std::numeric_limits<std::uint16_t>::max()) {
      throw std::invalid_argument("max_speed_dps must be in the range 1..65535");
    }
    if (timeout_ms < 1 || timeout_ms > 1000) {
      throw std::invalid_argument("timeout_ms must be in the range 1..1000");
    }

    const long double centi_degrees = std::round(
      static_cast<long double>(target_degrees) * 100.0L);
    if (centi_degrees < std::numeric_limits<std::int32_t>::min() ||
      centi_degrees > std::numeric_limits<std::int32_t>::max())
    {
      throw std::out_of_range("target_degrees exceeds the signed 32-bit 0.01-degree protocol range");
    }

    const auto target_centi_degrees = static_cast<std::int32_t>(centi_degrees);
    const auto timeout = std::chrono::milliseconds{timeout_ms};
    muweidu::ts::SocketCan can(can_interface);
    muweidu::ts::Motor motor(can, static_cast<std::uint8_t>(motor_id));

    if (enable_before_command) {
      motor.motorOn(timeout);
    }

    try {
      motor.setMultiTurnPositionWithSpeed(
        static_cast<std::uint16_t>(max_speed_dps), target_centi_degrees, timeout);
    } catch (...) {
      if (enable_before_command) {
        try {
          motor.emergencyStop(timeout);
        } catch (...) {
        }
      }
      throw;
    }

    RCLCPP_INFO(
      logger,
      "Sent position %.2f deg to motor %ld on %s (CAN ID 0x%03X, max speed %ld dps)",
      target_degrees,
      static_cast<long>(motor_id),
      can_interface.c_str(),
      static_cast<unsigned int>(motor.canId()),
      static_cast<long>(max_speed_dps));
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Position command failed: %s", error.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
