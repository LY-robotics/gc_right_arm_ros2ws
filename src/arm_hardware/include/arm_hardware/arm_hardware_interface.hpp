#pragma once

#include "arm_hardware/static_tendon_compensation.hpp"
#include "hardware_interface/system_interface.hpp"
#include "muweidu_ts/motor.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace arm_hardware
{

class ArmHardwareInterface final : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  struct Joint
  {
    std::string name;
    std::string can_interface;
    std::uint8_t motor_id{};
    std::uint16_t max_speed_dps{};
    double direction{1.0};
    double zero_position_rad{};
    double state_position_rad{};
    double state_velocity_rad_s{};
    double state_effort{};
    double command_position_rad{};
    double command_velocity_rad_s{};
    double last_command_position_rad{};
    std::string velocity_command_key;
    bool velocity_command_claimed{false};
    bool export_velocity_state{false};
    bool export_effort_state{false};
    bool command_position_initialized{false};
    std::unique_ptr<muweidu::ts::Motor> motor;
  };

  struct MotorAxis
  {
    std::uint8_t motor_id{};
    double direction{1.0};
    double zero_offset_rad{};
    double state_position_rad{};
    double state_velocity_rad_s{};
    double state_iq_raw{};
    std::unique_ptr<muweidu::ts::Motor> motor;
  };

  static bool parseBoolParameter(const std::string & value, const std::string & name);
  static double parsePositionParameter(const std::string & value, const std::string & name);
  double multiTurnAngleToPosition(const Joint & joint, std::int64_t angle_centi_degrees) const;
  std::array<double, 6> jointToMotorAngles(const std::array<double, 6> & joint_angles) const;
  std::array<double, 6> motorToJointAngles(const std::array<double, 6> & motor_angles) const;
  std::array<double, 6> motorToJointVelocities(const std::array<double, 6> & motor_velocities) const;
  std::array<double, 6> motorIqToJointEffort(const std::array<double, 6> & motor_iq) const;
  TendonAngles estimateCoupledPosition(const TendonAngles & encoder_equivalent) const;
  hardware_interface::return_type writeCoupledMotors();

  std::unordered_map<std::string, std::unique_ptr<muweidu::ts::SocketCan>> can_buses_;
  std::vector<Joint> joints_;
  std::array<MotorAxis, 6> motors_{};
  std::optional<StaticTendonModel> tendon_model_;
  std::optional<TendonAngles> previous_tendon_estimate_;
  std::string mapping_mode_{"direct"};
  std::string coupled_can_interface_;
  std::array<double, 6> joint_speed_limits_dps_{10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
  double joint_speed_scale_{0.8};
  double joint_speed_offset_dps_{2.0};
  std::optional<std::array<double, 6>> last_coupled_speed_limits_dps_;
  std::array<std::uint16_t, 6> last_coupled_motor_speeds_{};
  std::chrono::milliseconds can_timeout_{20};
  bool enable_motors_on_activate_{false};
  bool enable_motors_on_command_{false};
  bool initialize_motors_to_zero_{true};
  bool emergency_stop_on_deactivate_{true};
  bool motors_enabled_by_plugin_{false};
  bool initialization_zero_command_sent_{false};
  bool coupled_feedback_valid_{false};
  bool single_motor_test_mode_{false};
  std::size_t test_joint_index_{0};
};

}  // namespace arm_hardware
