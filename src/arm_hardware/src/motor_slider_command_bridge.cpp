#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

class MotorSliderCommandBridge final : public rclcpp::Node
{
public:
  MotorSliderCommandBridge()
  : Node("motor_slider_command_bridge")
  {
    joint_name_ = declare_parameter<std::string>("joint_name", "motor_joint");
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/slider/joint_states");
    const auto command_topic = declare_parameter<std::string>(
      "command_topic", "/motor_position_controller/commands");
    change_threshold_rad_ = declare_parameter<double>("change_threshold_rad", 0.001);

    if (joint_name_.empty() || input_topic.empty() || command_topic.empty()) {
      throw std::invalid_argument("joint_name and topic parameters must not be empty");
    }
    if (!std::isfinite(change_threshold_rad_) || change_threshold_rad_ < 0.0) {
      throw std::invalid_argument("change_threshold_rad must be finite and non-negative");
    }

    command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic, 10);
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        onJointState(*message);
      });

    RCLCPP_INFO(
      get_logger(), "Bridging joint '%s' from %s to %s", joint_name_.c_str(),
      input_topic.c_str(), command_topic.c_str());
  }

private:
  void onJointState(const sensor_msgs::msg::JointState & message)
  {
    for (std::size_t index = 0; index < message.name.size(); ++index) {
      if (message.name[index] != joint_name_ || index >= message.position.size()) {
        continue;
      }

      const double position = message.position[index];
      if (!std::isfinite(position)) {
        RCLCPP_WARN(get_logger(), "Ignoring non-finite slider position");
        return;
      }

      if (!last_position_rad_) {
        // Ignore the GUI's initial value so launch cannot move the motor.
        last_position_rad_ = position;
        RCLCPP_INFO(get_logger(), "Slider ready; move it to command the motor");
        return;
      }
      if (std::abs(position - *last_position_rad_) < change_threshold_rad_) {
        return;
      }

      last_position_rad_ = position;
      std_msgs::msg::Float64MultiArray command;
      command.data = {position};
      command_publisher_->publish(command);
      return;
    }
  }

  std::string joint_name_;
  double change_threshold_rad_{0.001};
  std::optional<double> last_position_rad_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotorSliderCommandBridge>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_slider_command_bridge"), "Startup failed: %s", error.what());
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
