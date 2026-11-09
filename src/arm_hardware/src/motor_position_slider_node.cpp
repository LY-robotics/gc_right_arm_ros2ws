#include "muweidu_ts/motor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kSliderLimitRad = kPi / 2.0;
constexpr double kCommandChangeThresholdRad = 0.001;

bool supportedCanInterface(const std::string & name)
{
  return name == "can0" || name == "can1" || name == "can2" || name == "can3";
}

class MotorPositionSlider final : public rclcpp::Node
{
public:
  MotorPositionSlider()
  : Node("motor_position_slider")
  {
    const auto can_interface = declare_parameter<std::string>("can_interface", "can2");
    const auto motor_id = declare_parameter<std::int64_t>("motor_id", 2);
    max_speed_dps_ = declare_parameter<std::int64_t>("max_speed_dps", 10);
    motor_on_before_command_ = declare_parameter<bool>("motor_on_before_command", true);
    const auto timeout_ms = declare_parameter<std::int64_t>("timeout_ms", 100);
    joint_name_ = declare_parameter<std::string>("joint_name", "motor_joint");

    if (!supportedCanInterface(can_interface)) {
      throw std::invalid_argument("can_interface must be can0, can1, can2, or can3");
    }
    if (motor_id < 1 || motor_id > 32) {
      throw std::invalid_argument("motor_id must be in the protocol range 1..32");
    }
    if (max_speed_dps_ < 1 || max_speed_dps_ > std::numeric_limits<std::uint16_t>::max()) {
      throw std::invalid_argument("max_speed_dps must be in the range 1..65535");
    }
    if (timeout_ms < 1 || timeout_ms > 1000) {
      throw std::invalid_argument("timeout_ms must be in the range 1..1000");
    }
    if (joint_name_.empty()) {
      throw std::invalid_argument("joint_name must not be empty");
    }

    timeout_ = std::chrono::milliseconds{timeout_ms};
    can_ = std::make_unique<muweidu::ts::SocketCan>(can_interface);
    motor_ = std::make_unique<muweidu::ts::Motor>(*can_, static_cast<std::uint8_t>(motor_id));

    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        onJointState(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Listening for slider joint '%s'; motor %ld on %s, max speed %ld dps",
      joint_name_.c_str(), static_cast<long>(motor_id), can_interface.c_str(),
      static_cast<long>(max_speed_dps_));
  }

private:
  void onJointState(const sensor_msgs::msg::JointState & message)
  {
    for (std::size_t i = 0; i < message.name.size(); ++i) {
      if (message.name[i] != joint_name_ || i >= message.position.size()) {
        continue;
      }

      const double position_rad = message.position[i];
      if (!std::isfinite(position_rad)) {
        RCLCPP_WARN(get_logger(), "Ignoring non-finite slider position");
        return;
      }
      if (std::abs(position_rad) > kSliderLimitRad + kCommandChangeThresholdRad) {
        RCLCPP_WARN(
          get_logger(), "Ignoring %.3f rad; slider range is -90 to +90 degrees", position_rad);
        return;
      }

      if (!last_slider_position_rad_) {
        // The GUI publishes its initial slider value at startup. Treat it as a
        // baseline so merely opening the window cannot move the motor.
        last_slider_position_rad_ = position_rad;
        RCLCPP_INFO(get_logger(), "Slider ready; move it to send a motor position command");
        return;
      }
      if (std::abs(position_rad - *last_slider_position_rad_) < kCommandChangeThresholdRad) {
        return;
      }

      last_slider_position_rad_ = position_rad;
      sendPosition(position_rad);
      return;
    }
  }

  void sendPosition(double position_rad)
  {
    const long double target_centi_degrees = std::round(
      static_cast<long double>(position_rad) * 18000.0L / static_cast<long double>(kPi));
    if (target_centi_degrees < std::numeric_limits<std::int32_t>::min() ||
      target_centi_degrees > std::numeric_limits<std::int32_t>::max())
    {
      RCLCPP_ERROR(get_logger(), "Slider position exceeds the CAN protocol range");
      return;
    }

    try {
      if (motor_on_before_command_ && !motor_enabled_) {
        motor_->motorOn(timeout_);
        motor_enabled_ = true;
      }
      motor_->setMultiTurnPositionWithSpeed(
        static_cast<std::uint16_t>(max_speed_dps_),
        static_cast<std::int32_t>(target_centi_degrees), timeout_);
      RCLCPP_INFO(
        get_logger(), "Commanded motor position %.2f degrees", position_rad * 180.0 / kPi);
    } catch (const std::exception & error) {
      if (motor_enabled_) {
        try {
          motor_->emergencyStop(timeout_);
        } catch (...) {
        }
        motor_enabled_ = false;
      }
      RCLCPP_ERROR(get_logger(), "Position command failed: %s", error.what());
    }
  }

  std::unique_ptr<muweidu::ts::SocketCan> can_;
  std::unique_ptr<muweidu::ts::Motor> motor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  std::optional<double> last_slider_position_rad_;
  std::chrono::milliseconds timeout_{100};
  std::int64_t max_speed_dps_{10};
  bool motor_on_before_command_{true};
  bool motor_enabled_{false};
  std::string joint_name_{"motor_joint"};
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotorPositionSlider>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_position_slider"), "Startup failed: %s", error.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
