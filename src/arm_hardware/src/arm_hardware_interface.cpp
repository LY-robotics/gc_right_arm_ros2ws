#include "arm_hardware/arm_hardware_interface.hpp"
#include "arm_hardware/synchronized_speed.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace arm_hardware
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr char kPositionInterface[] = "position";
constexpr char kVelocityInterface[] = "velocity";
constexpr char kEffortInterface[] = "effort";
constexpr std::array<const char *, 6> kExpectedJointNames{
  "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};

std::istringstream tendonParameter(
  const hardware_interface::HardwareInfo & info, const std::string & name)
{
  const auto found = info.hardware_parameters.find("tendon_" + name);
  if (found == info.hardware_parameters.end()) {
    throw std::invalid_argument("missing tendon parameter '" + name + "'");
  }
  auto value = found->second;
  std::replace(value.begin(), value.end(), ',', ' ');
  return std::istringstream(value);
}

double readTendonScalar(std::istream & stream, const std::string & name)
{
  double value;
  if (!(stream >> value) || !std::isfinite(value)) {
    throw std::invalid_argument("invalid tendon parameter '" + name + "'");
  }
  return value;
}

template<std::size_t N>
void readTendonVector(
  const hardware_interface::HardwareInfo & info, const std::string & name, double (&out)[N])
{
  auto stream = tendonParameter(info, name);
  char bracket;
  if (!(stream >> bracket) || bracket != '[') {
    throw std::invalid_argument("expected a list for tendon parameter '" + name + "'");
  }
  for (double & value : out) {
    value = readTendonScalar(stream, name);
  }
  if (!(stream >> bracket) || bracket != ']' || !(stream >> std::ws).eof()) {
    throw std::invalid_argument("wrong list length for tendon parameter '" + name + "'");
  }
}

TendonConfig readTendonConfig(const hardware_interface::HardwareInfo & info)
{
  TendonConfig config;
  readTendonVector(info, "mass_kg", config.mass_kg);
  readTendonVector(info, "rope_stiffness_n_per_mm", config.rope_stiffness_n_per_mm);
  readTendonVector(info, "radius_mm", config.radius_mm);
  readTendonVector(info, "gravity_m_s2", config.gravity_m_s2);
  readTendonVector(info, "payload_com_m", config.payload_com_m);
  auto payload = tendonParameter(info, "payload_mass_kg");
  config.payload_mass_kg = readTendonScalar(payload, "payload_mass_kg");
  if (!(payload >> std::ws).eof()) {
    throw std::invalid_argument("expected one tendon payload mass");
  }
  for (std::size_t i = 0; i < kExpectedJointNames.size(); ++i) {
    readTendonVector(info, "com_" + std::to_string(i + 1) + "_m", config.com_m[i]);
  }

  urdf::Model urdf_model;
  if (!urdf_model.initString(info.original_xml)) {
    throw std::invalid_argument("cannot parse the complete URDF for static tendon compensation");
  }
  constexpr std::array<const char *, 9> chain{
    "world", "jian", "base", "Arm1", "Arm2", "Arm3", "Arm4", "Arm5", "Arm6"};
  for (std::size_t i = 0; i + 1 < chain.size(); ++i) {
    const auto * name = i == 0 ? "Joint0" : i == 1 ? "Joint0_1" : kExpectedJointNames[i - 2];
    const auto joint = urdf_model.getJoint(name);
    const auto expected_type = i < 2 ? urdf::Joint::FIXED : urdf::Joint::REVOLUTE;
    if (!joint || joint->type != expected_type ||
      joint->parent_link_name != chain[i] || joint->child_link_name != chain[i + 1])
    {
      throw std::invalid_argument(std::string("invalid tendon URDF chain at ") + name);
    }
    const auto & transform = joint->parent_to_joint_origin_transform;
    config.origin_xyz_m[i][0] = transform.position.x;
    config.origin_xyz_m[i][1] = transform.position.y;
    config.origin_xyz_m[i][2] = transform.position.z;
    transform.rotation.getRPY(
      config.origin_rpy_rad[i][0], config.origin_rpy_rad[i][1], config.origin_rpy_rad[i][2]);
    if (i >= 2) {
      if (!joint->limits) {
        throw std::invalid_argument(std::string("missing tendon URDF limits for ") + name);
      }
      const auto j = i - 2;
      config.axis[j][0] = joint->axis.x;
      config.axis[j][1] = joint->axis.y;
      config.axis[j][2] = joint->axis.z;
      config.lower_rad[j] = joint->limits->lower;
      config.upper_rad[j] = joint->limits->upper;
    }
  }
  return config;
}

std::string requiredParameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  const std::string & joint_name)
{
  const auto found = parameters.find(name);
  if (found == parameters.end() || found->second.empty()) {
    throw std::invalid_argument(
            "joint '" + joint_name + "' requires parameter '" + name + "'");
  }
  return found->second;
}

std::uint64_t parseUnsignedParameter(const std::string & value, const std::string & name)
{
  std::size_t parsed = 0;
  const auto number = std::stoull(value, &parsed, 10);
  if (parsed != value.size()) {
    throw std::invalid_argument("parameter '" + name + "' must be an unsigned integer");
  }
  return number;
}

bool hasInterface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & interface_info) {
      return interface_info.name == name;
    });
}

std::array<double, 6> readJointSpeedLimits(const std::string & value)
{
  constexpr char error[] = "joint_speed_limits_dps must contain six positive comma-separated speeds";
  std::istringstream stream(value);
  std::array<double, 6> limits{};
  for (std::size_t i = 0; i < limits.size(); ++i) {
    if (i != 0) {
      char separator;
      if (!(stream >> separator) || separator != ',') {
        throw std::invalid_argument(error);
      }
    }
    if (!(stream >> limits[i]) || !std::isfinite(limits[i]) || limits[i] <= 0.0) {
      throw std::invalid_argument(error);
    }
  }
  if (!(stream >> std::ws).eof()) {
    throw std::invalid_argument(error);
  }
  return limits;
}
}  // namespace

hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    const auto mapping_parameter = info.hardware_parameters.find("mapping_mode");
    mapping_mode_ = mapping_parameter == info.hardware_parameters.end() ?
      "direct" : mapping_parameter->second;
    if (mapping_mode_ != "direct" && mapping_mode_ != "coupled") {
      throw std::invalid_argument("mapping_mode must be 'direct' or 'coupled'");
    }
    const bool coupled_mode = mapping_mode_ == "coupled";
    if (info.joints.empty()) {
      throw std::invalid_argument("hardware configuration must contain at least one joint");
    }
    if (coupled_mode && info.joints.size() != kExpectedJointNames.size()) {
      throw std::invalid_argument("coupled mapping requires exactly six joints");
    }

    const auto timeout_parameter = info.hardware_parameters.find("can_timeout_ms");
    if (timeout_parameter != info.hardware_parameters.end()) {
      const auto timeout_ms = parseUnsignedParameter(timeout_parameter->second, "can_timeout_ms");
      if (timeout_ms == 0 || timeout_ms > 1000) {
        throw std::invalid_argument("can_timeout_ms must be in the range 1..1000");
      }
      can_timeout_ = std::chrono::milliseconds{
        static_cast<std::chrono::milliseconds::rep>(timeout_ms)};
    }

    const auto enable_parameter = info.hardware_parameters.find("enable_motors_on_activate");
    if (enable_parameter != info.hardware_parameters.end()) {
      enable_motors_on_activate_ = parseBoolParameter(
        enable_parameter->second, "enable_motors_on_activate");
    }
    const auto enable_on_command_parameter = info.hardware_parameters.find(
      "enable_motors_on_command");
    if (enable_on_command_parameter != info.hardware_parameters.end()) {
      enable_motors_on_command_ = parseBoolParameter(
        enable_on_command_parameter->second, "enable_motors_on_command");
    }
    const auto initialize_zero_parameter = info.hardware_parameters.find(
      "initialize_motors_to_zero");
    if (initialize_zero_parameter != info.hardware_parameters.end()) {
      initialize_motors_to_zero_ = parseBoolParameter(
        initialize_zero_parameter->second, "initialize_motors_to_zero");
    }
    tendon_model_.reset();
    const auto tendon_parameter = info.hardware_parameters.find("static_tendon_compensation");
    if (tendon_parameter != info.hardware_parameters.end() &&
      parseBoolParameter(tendon_parameter->second, "static_tendon_compensation"))
    {
      if (!coupled_mode || initialize_motors_to_zero_) {
        throw std::invalid_argument(
                "static tendon compensation requires coupled mapping and "
                "initialize_motors_to_zero=false");
      }
      tendon_model_.emplace(readTendonConfig(info));
      RCLCPP_INFO(
        rclcpp::get_logger("arm_hardware"), "static tendon position compensation enabled");
    }
    const auto stop_parameter = info.hardware_parameters.find("emergency_stop_on_deactivate");
    if (stop_parameter != info.hardware_parameters.end()) {
      emergency_stop_on_deactivate_ = parseBoolParameter(
        stop_parameter->second, "emergency_stop_on_deactivate");
    }
    const auto duplicate_parameter = info.hardware_parameters.find("allow_duplicate_motor_ids");
    const bool allow_duplicate_motor_ids = !coupled_mode &&
      duplicate_parameter != info.hardware_parameters.end() &&
      parseBoolParameter(duplicate_parameter->second, "allow_duplicate_motor_ids");
    const auto test_joint_parameter = info.hardware_parameters.find("test_joint");
    const std::string test_joint_name = test_joint_parameter == info.hardware_parameters.end() ?
      std::string{} : test_joint_parameter->second;
    single_motor_test_mode_ = allow_duplicate_motor_ids;
    std::set<std::pair<std::string, std::uint8_t>> used_motor_ids;
    joints_.clear();
    joints_.reserve(info.joints.size());

    for (std::size_t index = 0; index < info.joints.size(); ++index) {
      const auto & joint_info = info.joints[index];
      if (!hasInterface(joint_info.command_interfaces, kPositionInterface) ||
        !hasInterface(joint_info.state_interfaces, kPositionInterface))
      {
        throw std::invalid_argument(
                "joint '" + joint_info.name + "' must declare position command and state interfaces");
      }

      Joint joint;
      joint.name = joint_info.name;
      if (coupled_mode && joint.name != kExpectedJointNames[index]) {
        throw std::invalid_argument(
                "coupled mapping expects joints Joint1..Joint6 in that order; got '" +
                joint.name + "' at index " + std::to_string(index));
      }
      joint.export_velocity_state = hasInterface(joint_info.state_interfaces, kVelocityInterface);
      joint.export_effort_state = hasInterface(joint_info.state_interfaces, kEffortInterface);
      if (coupled_mode && hasInterface(joint_info.command_interfaces, kVelocityInterface)) {
        joint.velocity_command_key = joint.name + "/" + kVelocityInterface;
      }
      if (!coupled_mode) {
        joint.can_interface = requiredParameter(
          joint_info.parameters, "can_interface", joint_info.name);

        const auto motor_id = parseUnsignedParameter(
          requiredParameter(joint_info.parameters, "motor_id", joint_info.name), "motor_id");
        if (motor_id < 1 || motor_id > 32) {
          throw std::invalid_argument(
                  "joint '" + joint.name + "' motor_id must be in the range 1..32");
        }
        joint.motor_id = static_cast<std::uint8_t>(motor_id);

        const auto speed = parseUnsignedParameter(
          requiredParameter(joint_info.parameters, "max_speed_dps", joint.name),
          "max_speed_dps");
        if (speed == 0 || speed > UINT16_MAX) {
          throw std::invalid_argument(
                  "joint '" + joint.name + "' max_speed_dps must be in 1..65535");
        }
        joint.max_speed_dps = static_cast<std::uint16_t>(speed);

        const auto direction_parameter = joint_info.parameters.find("direction");
        if (direction_parameter != joint_info.parameters.end()) {
          const double direction = parsePositionParameter(
            direction_parameter->second, "direction");
          if (direction != 1.0 && direction != -1.0) {
            throw std::invalid_argument(
                    "joint '" + joint.name + "' direction must be 1 or -1");
          }
          joint.direction = direction;
        }

        const auto offset_parameter = joint_info.parameters.find("zero_position_rad");
        if (offset_parameter != joint_info.parameters.end()) {
          joint.zero_position_rad = parsePositionParameter(
            offset_parameter->second, "zero_position_rad");
        }

        if (!allow_duplicate_motor_ids &&
          !used_motor_ids.emplace(joint.can_interface, joint.motor_id).second) {
          throw std::invalid_argument(
                  "duplicate motor_id " + std::to_string(motor_id) + " on CAN interface '" +
                  joint.can_interface + "'");
        }
      }

      joints_.emplace_back(std::move(joint));
    }

    if (coupled_mode) {
      coupled_can_interface_ = requiredParameter(
        info.hardware_parameters, "can_interface", "hardware");
      if (info.hardware_parameters.count("max_speed_dps") != 0) {
        throw std::invalid_argument(
                "coupled max_speed_dps is obsolete; use joint_speed_limits_dps in joint degrees/s");
      }
      const auto velocity_count = std::count_if(joints_.begin(), joints_.end(), [](const Joint & j) {
          return !j.velocity_command_key.empty();
        });
      if (velocity_count != 0 && velocity_count != 6) {
        throw std::invalid_argument("coupled trajectory speed requires all six velocity commands");
      }
      const auto & params = info.hardware_parameters;
      const auto limits = params.find("joint_speed_limits_dps");
      const auto legacy_limit = params.find("joint_max_speed_dps");
      if (legacy_limit != params.end()) {
        if (velocity_count != 0 || limits != params.end()) {
          throw std::invalid_argument(
                  "use joint_speed_limits_dps instead of joint_max_speed_dps for trajectory speed");
        }
        const double limit = parsePositionParameter(legacy_limit->second, "joint_max_speed_dps");
        if (limit <= 0.0) {
          throw std::invalid_argument("joint_max_speed_dps must be positive");
        }
        joint_speed_limits_dps_.fill(limit);
      } else if (limits != params.end()) {
        joint_speed_limits_dps_ = readJointSpeedLimits(limits->second);
      }
      const auto scale = params.find("joint_speed_scale");
      const auto offset = params.find("joint_speed_offset_dps");
      if (scale != params.end()) {
        joint_speed_scale_ = parsePositionParameter(scale->second, "joint_speed_scale");
      }
      if (offset != params.end()) {
        joint_speed_offset_dps_ = parsePositionParameter(offset->second, "joint_speed_offset_dps");
      }
      if (velocity_count != 0) {
        mappedJointSpeedLimits({}, joint_speed_limits_dps_, joint_speed_scale_,
          joint_speed_offset_dps_);
      } else if (scale != params.end() || offset != params.end()) {
        throw std::invalid_argument("joint speed mapping requires six velocity command interfaces");
      }

      std::set<std::uint8_t> used_coupled_motor_ids;
      for (std::size_t index = 0; index < motors_.size(); ++index) {
        const auto prefix = "motor" + std::to_string(index + 1);
        const auto motor_id = parseUnsignedParameter(
          requiredParameter(info.hardware_parameters, prefix + "_id", "hardware"),
          prefix + "_id");
        if (motor_id < 1 || motor_id > 32) {
          throw std::invalid_argument(prefix + "_id must be in the range 1..32");
        }
        if (!used_coupled_motor_ids.insert(static_cast<std::uint8_t>(motor_id)).second) {
          throw std::invalid_argument("coupled motor IDs must be unique on the CAN interface");
        }
        auto & motor = motors_[index];
        motor.motor_id = static_cast<std::uint8_t>(motor_id);

        const auto direction_parameter = info.hardware_parameters.find(prefix + "_direction");
        if (direction_parameter != info.hardware_parameters.end()) {
          motor.direction = parsePositionParameter(direction_parameter->second, prefix + "_direction");
          if (motor.direction != 1.0 && motor.direction != -1.0) {
            throw std::invalid_argument(prefix + "_direction must be 1 or -1");
          }
        }
        const auto offset_parameter = info.hardware_parameters.find(prefix + "_zero_offset_rad");
        if (offset_parameter != info.hardware_parameters.end()) {
          motor.zero_offset_rad = parsePositionParameter(
            offset_parameter->second, prefix + "_zero_offset_rad");
        }
      }
    }

    if (single_motor_test_mode_) {
      if (test_joint_name.empty()) {
        throw std::invalid_argument(
                "test_joint is required when allow_duplicate_motor_ids is enabled");
      }
      const auto test_joint = std::find_if(
        joints_.begin(), joints_.end(), [&test_joint_name](const Joint & joint) {
          return joint.name == test_joint_name;
        });
      if (test_joint == joints_.end()) {
        throw std::invalid_argument(
                "test_joint '" + test_joint_name + "' is not present in the hardware joints");
      }
      test_joint_index_ = static_cast<std::size_t>(std::distance(joints_.begin(), test_joint));
      RCLCPP_WARN(
        rclcpp::get_logger("arm_hardware"),
        "single-motor test mode enabled: only joint '%s' will access the physical motor",
        test_joint_name.c_str());
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"), "on_init failed: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  try {
    for (auto & joint : joints_) {
      joint.motor.reset();
    }
    for (auto & motor : motors_) {
      motor.motor.reset();
    }
    can_buses_.clear();
    motors_enabled_by_plugin_ = false;
    initialization_zero_command_sent_ = false;
    coupled_feedback_valid_ = false;
    previous_tendon_estimate_.reset();
    last_coupled_speed_limits_dps_.reset();
    if (mapping_mode_ == "coupled") {
      auto inserted = can_buses_.emplace(
        coupled_can_interface_,
        std::make_unique<muweidu::ts::SocketCan>(coupled_can_interface_));
      auto & bus = *inserted.first->second;
      for (auto & motor : motors_) {
        motor.motor = std::make_unique<muweidu::ts::Motor>(bus, motor.motor_id);
      }
    } else {
      for (auto & joint : joints_) {
        auto bus = can_buses_.find(joint.can_interface);
        if (bus == can_buses_.end()) {
          auto inserted = can_buses_.emplace(
            joint.can_interface,
            std::make_unique<muweidu::ts::SocketCan>(joint.can_interface));
          bus = inserted.first;
        }
        joint.motor = std::make_unique<muweidu::ts::Motor>(*bus->second, joint.motor_id);
      }
    }
  } catch (const std::exception & error) {
    can_buses_.clear();
    for (auto & joint : joints_) {
      joint.motor.reset();
    }
    for (auto & motor : motors_) {
      motor.motor.reset();
    }
    RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"), "on_configure failed: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  try {
    if (mapping_mode_ == "coupled") {
      initialization_zero_command_sent_ = false;
      last_coupled_speed_limits_dps_.reset();
      std::array<double, 6> motor_angles{};
      for (std::size_t index = 0; index < motors_.size(); ++index) {
        auto & motor = motors_[index];
        try {
          const auto angle_centi_degrees = motor.motor->readMultiTurnAngleCentiDegrees(can_timeout_);
          const double raw_angle_rad = static_cast<double>(angle_centi_degrees) * kPi / 18000.0;
          motor.state_position_rad = motor.direction * (raw_angle_rad - motor.zero_offset_rad);
          motor_angles[index] = motor.state_position_rad;
        } catch (const std::exception & error) {
          throw std::runtime_error(
                  "motor " + std::to_string(static_cast<unsigned int>(motor.motor_id)) +
                  " on " + coupled_can_interface_ + ": " + error.what());
        }
      }
      previous_tendon_estimate_.reset();
      const auto encoder_joint_angles = motorToJointAngles(motor_angles);
      const auto joint_angles = estimateCoupledPosition(encoder_joint_angles);
      if (tendon_model_) {
        previous_tendon_estimate_ = joint_angles;
      }
      coupled_feedback_valid_ = true;
      for (std::size_t index = 0; index < joints_.size(); ++index) {
        auto & joint = joints_[index];
        joint.state_position_rad = joint_angles[index];
        joint.command_position_rad = joint_angles[index];
        joint.command_velocity_rad_s = 0.0;
        joint.last_command_position_rad = joint_angles[index];
        joint.state_velocity_rad_s = 0.0;
        joint.state_effort = 0.0;
        joint.command_position_initialized = true;
      }

      if (initialize_motors_to_zero_) {
        try {
          // This writes an absolute raw motor-side position target of 0. It
          // does not redefine or persist the encoder zero point.
          std::array<double, 6> zero_motor_angles{};
          for (std::size_t index = 0; index < motors_.size(); ++index) {
            zero_motor_angles[index] = -motors_[index].direction * motors_[index].zero_offset_rad;
          }
          const auto zero_joint_angles = motorToJointAngles(zero_motor_angles);
          const auto speeds = synchronizedMotorSpeeds(
            encoder_joint_angles, zero_joint_angles, motor_angles, zero_motor_angles,
            joint_speed_limits_dps_);
          initialization_zero_command_sent_ = true;
          for (std::size_t index = 0; index < motors_.size(); ++index) {
            motors_[index].motor->setMultiTurnPositionWithSpeed(speeds[index], 0, can_timeout_);
          }
        } catch (...) {
          // A partial six-motor setpoint must not be left active if one write
          // fails during activation.
          for (auto & motor : motors_) {
            try {
              motor.motor->emergencyStop(can_timeout_);
            } catch (...) {
            }
          }
          throw;
        }
      }
    } else {
      auto activate_joint = [this](Joint & joint) {
        try {
          const auto angle_centi_degrees = joint.motor->readMultiTurnAngleCentiDegrees(can_timeout_);
          joint.state_position_rad = multiTurnAngleToPosition(joint, angle_centi_degrees);
          joint.command_position_rad = joint.state_position_rad;
          joint.last_command_position_rad = joint.command_position_rad;
          joint.command_position_initialized = true;
          joint.state_velocity_rad_s = 0.0;
          joint.state_effort = 0.0;
        } catch (const std::exception & error) {
          throw std::runtime_error(
                  "joint '" + joint.name + "' motor " +
                  std::to_string(static_cast<unsigned int>(joint.motor_id)) +
                  " on " + joint.can_interface + ": " + error.what());
        }
      };

      if (single_motor_test_mode_) {
        activate_joint(joints_[test_joint_index_]);
        for (std::size_t index = 0; index < joints_.size(); ++index) {
          if (index == test_joint_index_) {
            continue;
          }
          auto & joint = joints_[index];
          joint.state_position_rad = 0.0;
          joint.command_position_rad = 0.0;
          joint.last_command_position_rad = 0.0;
          joint.state_velocity_rad_s = 0.0;
          joint.state_effort = 0.0;
          joint.command_position_initialized = true;
        }
      } else {
        for (auto & joint : joints_) {
          activate_joint(joint);
        }
      }
    }

    if (enable_motors_on_activate_) {
      // If an acknowledgement times out after the motor accepted 0x88, the
      // failure path should still attempt to stop all configured motors.
      motors_enabled_by_plugin_ = true;
      if (mapping_mode_ == "coupled") {
        for (auto & motor : motors_) {
          motor.motor->motorOn(can_timeout_);
        }
      } else if (single_motor_test_mode_) {
        joints_[test_joint_index_].motor->motorOn(can_timeout_);
      } else {
        for (auto & joint : joints_) {
          joint.motor->motorOn(can_timeout_);
        }
      }
    }
  } catch (const std::exception & error) {
    if ((motors_enabled_by_plugin_ || initialization_zero_command_sent_) &&
      emergency_stop_on_deactivate_)
    {
      if (mapping_mode_ == "coupled") {
        for (auto & motor : motors_) {
          try {
            motor.motor->emergencyStop(can_timeout_);
          } catch (...) {
          }
        }
      } else {
        for (auto & joint : joints_) {
          try {
            joint.motor->emergencyStop(can_timeout_);
          } catch (...) {
          }
        }
      }
    }
    motors_enabled_by_plugin_ = false;
    initialization_zero_command_sent_ = false;
    coupled_feedback_valid_ = false;
    previous_tendon_estimate_.reset();
    RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"), "on_activate failed: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  bool success = true;
  if ((motors_enabled_by_plugin_ || initialization_zero_command_sent_) &&
    emergency_stop_on_deactivate_)
  {
    if (mapping_mode_ == "coupled") {
      for (auto & motor : motors_) {
        try {
          motor.motor->emergencyStop(can_timeout_);
        } catch (const std::exception & error) {
          success = false;
          RCLCPP_ERROR(
            rclcpp::get_logger("arm_hardware"),
            "failed to stop motor %u on %s: %s",
            static_cast<unsigned int>(motor.motor_id), coupled_can_interface_.c_str(), error.what());
        }
      }
    } else {
      std::vector<Joint *> joints_to_stop;
      if (single_motor_test_mode_) {
        joints_to_stop.push_back(&joints_[test_joint_index_]);
      } else {
        for (auto & joint : joints_) {
          joints_to_stop.push_back(&joint);
        }
      }
      for (auto * joint : joints_to_stop) {
        try {
          joint->motor->emergencyStop(can_timeout_);
        } catch (const std::exception & error) {
          success = false;
          RCLCPP_ERROR(
            rclcpp::get_logger("arm_hardware"),
            "failed to stop motor %u on %s: %s",
            static_cast<unsigned int>(joint->motor_id), joint->can_interface.c_str(), error.what());
        }
      }
    }
  }
  motors_enabled_by_plugin_ = false;
  initialization_zero_command_sent_ = false;
  coupled_feedback_valid_ = false;
  previous_tendon_estimate_.reset();
  last_coupled_speed_limits_dps_.reset();
  return success ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type ArmHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  try {
    if (mapping_mode_ == "coupled") {
      coupled_feedback_valid_ = false;
      std::array<double, 6> motor_angles{};
      std::array<double, 6> motor_velocities{};
      std::array<double, 6> motor_iq{};
      const bool read_velocity = std::any_of(
        joints_.begin(), joints_.end(), [](const Joint & joint) {return joint.export_velocity_state;});
      const bool read_effort = std::any_of(
        joints_.begin(), joints_.end(), [](const Joint & joint) {return joint.export_effort_state;});

      for (std::size_t index = 0; index < motors_.size(); ++index) {
        auto & motor = motors_[index];
        try {
          const auto angle_centi_degrees = motor.motor->readMultiTurnAngleCentiDegrees(can_timeout_);
          const double raw_angle_rad = static_cast<double>(angle_centi_degrees) * kPi / 18000.0;
          motor_angles[index] = motor.direction * (raw_angle_rad - motor.zero_offset_rad);
          if (read_velocity || read_effort) {
            const auto status = motor.motor->readStatus2(can_timeout_);
            motor_velocities[index] = motor.direction *
              static_cast<double>(status.speed_raw) * kPi / 180.0;
            motor_iq[index] = motor.direction * static_cast<double>(status.iq_or_power_raw);
          }
        } catch (const std::exception & error) {
          throw std::runtime_error(
                  "motor " + std::to_string(static_cast<unsigned int>(motor.motor_id)) +
                  " on " + coupled_can_interface_ + ": " + error.what());
        }
      }

      const auto joint_angles = estimateCoupledPosition(motorToJointAngles(motor_angles));
      for (std::size_t index = 0; index < motors_.size(); ++index) {
        motors_[index].state_position_rad = motor_angles[index];
        motors_[index].state_velocity_rad_s = motor_velocities[index];
        motors_[index].state_iq_raw = motor_iq[index];
      }
      coupled_feedback_valid_ = true;
      auto joint_velocities = read_velocity ?
        motorToJointVelocities(motor_velocities) : std::array<double, 6>{};
      if (tendon_model_) {
        const double seconds = period.seconds();
        for (std::size_t index = 0; index < joints_.size(); ++index) {
          joint_velocities[index] = previous_tendon_estimate_ && seconds > 0.0 ?
            (joint_angles[index] - (*previous_tendon_estimate_)[index]) / seconds : 0.0;
        }
        previous_tendon_estimate_ = joint_angles;
      }
      const auto joint_effort = read_effort ?
        motorIqToJointEffort(motor_iq) : std::array<double, 6>{};
      for (std::size_t index = 0; index < joints_.size(); ++index) {
        joints_[index].state_position_rad = joint_angles[index];
        if (joints_[index].export_velocity_state) {
          joints_[index].state_velocity_rad_s = joint_velocities[index];
        }
        if (joints_[index].export_effort_state) {
          joints_[index].state_effort = joint_effort[index];
        }
      }
    } else {
      auto read_joint = [this](Joint & joint) {
        try {
          const auto angle_centi_degrees = joint.motor->readMultiTurnAngleCentiDegrees(can_timeout_);
          joint.state_position_rad = multiTurnAngleToPosition(joint, angle_centi_degrees);
          if (joint.export_velocity_state || joint.export_effort_state) {
            const auto status = joint.motor->readStatus2(can_timeout_);
            if (joint.export_velocity_state) {
              joint.state_velocity_rad_s = joint.direction *
                static_cast<double>(status.speed_raw) * kPi / 180.0;
            }
            if (joint.export_effort_state) {
              joint.state_effort = joint.direction * static_cast<double>(status.iq_or_power_raw);
            }
          }
        } catch (const std::exception & error) {
          throw std::runtime_error(
                  "joint '" + joint.name + "' motor " +
                  std::to_string(static_cast<unsigned int>(joint.motor_id)) +
                  " on " + joint.can_interface + ": " + error.what());
        }
      };

      if (single_motor_test_mode_) {
        read_joint(joints_[test_joint_index_]);
        for (std::size_t index = 0; index < joints_.size(); ++index) {
          if (index != test_joint_index_) {
            joints_[index].state_position_rad = joints_[index].command_position_rad;
            joints_[index].state_velocity_rad_s = 0.0;
            joints_[index].state_effort = 0.0;
          }
        }
      } else {
        for (auto & joint : joints_) {
          read_joint(joint);
        }
      }
    }
  } catch (const std::exception & error) {
    previous_tendon_estimate_.reset();
    last_coupled_speed_limits_dps_.reset();
    if (mapping_mode_ == "coupled" &&
      (motors_enabled_by_plugin_ || initialization_zero_command_sent_))
    {
      for (auto & motor : motors_) {
        try {
          motor.motor->emergencyStop(can_timeout_);
        } catch (...) {
        }
      }
    }
    RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"), "read failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  if (mapping_mode_ == "coupled") {
    return writeCoupledMotors();
  }
  constexpr double kCommandChangeThresholdRad = 1e-6;
  const bool has_new_command = single_motor_test_mode_ ?
    (!joints_[test_joint_index_].command_position_initialized ||
    std::abs(joints_[test_joint_index_].command_position_rad -
    joints_[test_joint_index_].last_command_position_rad) > kCommandChangeThresholdRad) :
    std::any_of(joints_.begin(), joints_.end(), [](const Joint & joint) {
      return !joint.command_position_initialized ||
             std::abs(joint.command_position_rad - joint.last_command_position_rad) >
             kCommandChangeThresholdRad;
    });
  if (!has_new_command) {
    return hardware_interface::return_type::OK;
  }

  try {
    // Keep the motor off during startup; enable it only when a controller
    // changes the requested position, if configured for slider operation.
    if (!motors_enabled_by_plugin_) {
      if (!enable_motors_on_command_) {
        return hardware_interface::return_type::OK;
      }
      motors_enabled_by_plugin_ = true;
      if (single_motor_test_mode_) {
        joints_[test_joint_index_].motor->motorOn(can_timeout_);
      } else {
        for (auto & joint : joints_) {
          joint.motor->motorOn(can_timeout_);
        }
      }
    }

    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (single_motor_test_mode_ && index != test_joint_index_) {
        joints_[index].last_command_position_rad = joints_[index].command_position_rad;
        joints_[index].command_position_initialized = true;
        continue;
      }
      auto & joint = joints_[index];
      if (joint.command_position_initialized &&
        std::abs(joint.command_position_rad - joint.last_command_position_rad) <=
        kCommandChangeThresholdRad)
      {
        continue;
      }
      if (!std::isfinite(joint.command_position_rad)) {
        throw std::invalid_argument(
                "non-finite position command for joint '" + joint.name + "'");
      }

      const double motor_position_rad =
        joint.direction * (joint.command_position_rad - joint.zero_position_rad);
      const long double target_centi_degrees = std::round(
        static_cast<long double>(motor_position_rad) * 18000.0L / static_cast<long double>(kPi));
      if (target_centi_degrees < std::numeric_limits<std::int32_t>::min() ||
        target_centi_degrees > std::numeric_limits<std::int32_t>::max())
      {
        throw std::out_of_range(
                "position command for joint '" + joint.name + "' exceeds CAN protocol range");
      }

      joint.motor->setMultiTurnPositionWithSpeed(
        joint.max_speed_dps,
        static_cast<std::int32_t>(target_centi_degrees),
        can_timeout_);
      joint.last_command_position_rad = joint.command_position_rad;
      joint.command_position_initialized = true;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"), "write failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size() * 3U);
  for (auto & joint : joints_) {
    interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, kPositionInterface, &joint.state_position_rad));
    if (joint.export_velocity_state) {
      interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint.name, kVelocityInterface, &joint.state_velocity_rad_s));
    }
    if (joint.export_effort_state) {
      interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint.name, kEffortInterface, &joint.state_effort));
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(2 * joints_.size());
  for (auto & joint : joints_) {
    interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, kPositionInterface, &joint.command_position_rad));
    if (!joint.velocity_command_key.empty()) {
      interfaces.emplace_back(joint.name, kVelocityInterface, &joint.command_velocity_rad_s);
    }
  }
  return interfaces;
}

hardware_interface::return_type ArmHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (auto & joint : joints_) {
    const auto & key = joint.velocity_command_key;
    if (key.empty()) {
      continue;
    }
    const bool starting = std::find(start_interfaces.begin(), start_interfaces.end(), key) !=
      start_interfaces.end();
    const bool stopping = std::find(stop_interfaces.begin(), stop_interfaces.end(), key) !=
      stop_interfaces.end();
    if (starting || stopping) {
      joint.velocity_command_claimed = starting;
      joint.command_velocity_rad_s = 0.0;
      last_coupled_speed_limits_dps_.reset();
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::writeCoupledMotors()
{
  constexpr double kCommandChangeThresholdRad = 1e-6;
  const bool has_new_command = std::any_of(
    joints_.begin(), joints_.end(), [kCommandChangeThresholdRad](const Joint & joint) {
      return !std::isfinite(joint.command_position_rad) || !joint.command_position_initialized ||
             std::abs(joint.command_position_rad - joint.last_command_position_rad) >
             kCommandChangeThresholdRad;
    });
  const bool use_trajectory_velocity = std::all_of(
    joints_.begin(), joints_.end(), [](const Joint & joint) {return joint.velocity_command_claimed;});
  if (!has_new_command && (!use_trajectory_velocity || !last_coupled_speed_limits_dps_)) {
    return hardware_interface::return_type::OK;
  }

  try {
    auto speed_limits = joint_speed_limits_dps_;
    if (use_trajectory_velocity) {
      std::array<double, 6> velocity_rad_s{};
      for (std::size_t i = 0; i < velocity_rad_s.size(); ++i) {
        velocity_rad_s[i] = joints_[i].command_velocity_rad_s;
      }
      speed_limits = mappedJointSpeedLimits(
        velocity_rad_s, joint_speed_limits_dps_, joint_speed_scale_, joint_speed_offset_dps_);
      if (!has_new_command && speed_limits == *last_coupled_speed_limits_dps_) {
        return hardware_interface::return_type::OK;
      }
    }
    if (!coupled_feedback_valid_) {
      throw std::runtime_error("no complete current six-motor feedback snapshot");
    }
    std::array<double, 6> joint_angles{};
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (!std::isfinite(joints_[index].command_position_rad)) {
        throw std::invalid_argument(
                "non-finite position command for joint '" + joints_[index].name + "'");
      }
      // A speed-only refresh must reuse the last sent position, even if a new
      // position sample differs by less than the position update threshold.
      joint_angles[index] = has_new_command ?
        joints_[index].command_position_rad : joints_[index].last_command_position_rad;
    }

    auto effective_joint_angles = joint_angles;
    if (tendon_model_) {
      if (!tendon_model_->withinLimits(joint_angles)) {
        throw std::out_of_range("nominal target is outside the URDF limits");
      }
      const auto result = tendon_model_->evaluate(joint_angles);
      std::copy_n(result.effective_rad, effective_joint_angles.size(), effective_joint_angles.begin());
      if (!tendon_model_->withinLimits(effective_joint_angles)) {
        throw std::out_of_range("compensated target is outside the URDF limits");
      }
    }
    const auto mapped_motor_angles = jointToMotorAngles(effective_joint_angles);
    std::array<double, 6> current_motor_angles{};
    for (std::size_t index = 0; index < motors_.size(); ++index) {
      current_motor_angles[index] = motors_[index].state_position_rad;
    }
    const auto current_joint_angles = tendon_model_ ?
      previous_tendon_estimate_.value() : motorToJointAngles(current_motor_angles);
    std::array<double, 6> quantized_motor_angles{};
    std::array<std::int32_t, 6> targets_centi_degrees{};
    for (std::size_t index = 0; index < motors_.size(); ++index) {
      const auto & motor = motors_[index];
      const double raw_motor_angle_rad =
        motor.direction * mapped_motor_angles[index] + motor.zero_offset_rad;
      const long double target = std::round(
        static_cast<long double>(raw_motor_angle_rad) * 18000.0L /
        static_cast<long double>(kPi));
      if (!std::isfinite(target) || target < std::numeric_limits<std::int32_t>::min() ||
        target > std::numeric_limits<std::int32_t>::max())
      {
        throw std::out_of_range(
                "mapped motor " + std::to_string(index + 1) +
                " target exceeds the 0xA4 signed 32-bit range");
      }
      targets_centi_degrees[index] = static_cast<std::int32_t>(target);
      quantized_motor_angles[index] = motor.direction * (
        static_cast<double>(targets_centi_degrees[index]) * kPi / 18000.0 -
        motor.zero_offset_rad);
    }
    const auto quantized_joint_angles = motorToJointAngles(quantized_motor_angles);
    auto target_for_speed = quantized_joint_angles;
    if (tendon_model_) {
      if (!tendon_model_->withinLimits(quantized_joint_angles, kTendonEncoderLimitToleranceRad)) {
        throw std::out_of_range("quantized compensated target is outside the URDF limits");
      }
      target_for_speed = tendon_model_->estimate(quantized_joint_angles, &joint_angles);
    }
    const auto speeds = synchronizedMotorSpeeds(
      current_joint_angles, target_for_speed, current_motor_angles, quantized_motor_angles,
      speed_limits);
    if (!has_new_command &&
      (current_motor_angles == quantized_motor_angles || speeds == last_coupled_motor_speeds_))
    {
      last_coupled_speed_limits_dps_ = speed_limits;
      return hardware_interface::return_type::OK;
    }

    if (!motors_enabled_by_plugin_) {
      if (!enable_motors_on_command_) {
        return hardware_interface::return_type::OK;
      }
      motors_enabled_by_plugin_ = true;
      for (auto & motor : motors_) {
        motor.motor->motorOn(can_timeout_);
      }
    }

    // Position and speed updates both use a complete six-motor setpoint.
    for (std::size_t index = 0; index < motors_.size(); ++index) {
      motors_[index].motor->setMultiTurnPositionWithSpeed(
        speeds[index], targets_centi_degrees[index], can_timeout_);
    }
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      joints_[i].last_command_position_rad = joint_angles[i];
      joints_[i].command_position_initialized = true;
    }
    last_coupled_speed_limits_dps_ = speed_limits;
    last_coupled_motor_speeds_ = speeds;
  } catch (const std::exception & error) {
    last_coupled_speed_limits_dps_.reset();
    if (motors_enabled_by_plugin_ || initialization_zero_command_sent_) {
      for (auto & motor : motors_) {
        try {
          motor.motor->emergencyStop(can_timeout_);
        } catch (...) {
        }
      }
    }
    RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"), "coupled write failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool ArmHardwareInterface::parseBoolParameter(
  const std::string & value, const std::string & name)
{
  // Xacro can serialize macro boolean parameters as Python's True/False.
  // Normalize the text at the hardware boundary while rejecting other values.
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char character) {
    return std::isspace(character) != 0;
  });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char character) {
    return std::isspace(character) != 0;
  }).base();
  std::string normalized;
  if (first < last) {
    normalized.assign(first, last);
  }
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});

  if (normalized == "true" || normalized == "1") {
    return true;
  }
  if (normalized == "false" || normalized == "0") {
    return false;
  }
  throw std::invalid_argument(
          "parameter '" + name + "' must be true/false or 1/0 (case-insensitive); got '" + value + "'");
}

double ArmHardwareInterface::parsePositionParameter(
  const std::string & value, const std::string & name)
{
  std::size_t parsed = 0;
  const double number = std::stod(value, &parsed);
  if (parsed != value.size() || !std::isfinite(number)) {
    throw std::invalid_argument("parameter '" + name + "' must be a finite number");
  }
  return number;
}

double ArmHardwareInterface::multiTurnAngleToPosition(
  const Joint & joint, std::int64_t angle_centi_degrees) const
{
  const long double angle_rad = static_cast<long double>(angle_centi_degrees) *
    static_cast<long double>(kPi) / 18000.0L;
  return joint.zero_position_rad + joint.direction * static_cast<double>(angle_rad);
}

std::array<double, 6> ArmHardwareInterface::jointToMotorAngles(
  const std::array<double, 6> & joint_angles) const
{
  std::array<double, 6> motor_angles{};
  for (std::size_t row = 0; row < motor_angles.size(); ++row) {
    for (std::size_t col = 0; col < joint_angles.size(); ++col) {
      motor_angles[row] += kCoupledMotorMatrix[row][col] * joint_angles[col];
    }
  }
  return motor_angles;
}

std::array<double, 6> ArmHardwareInterface::motorToJointAngles(
  const std::array<double, 6> & motor_angles) const
{
  const auto & b = kCoupledMotorMatrix;
  const double q1 = motor_angles[0] / b[0][0];
  const double q2 = (motor_angles[1] - motor_angles[2]) / (2.0 * b[1][1]);
  const double q3 = (motor_angles[1] + motor_angles[2]) / (2.0 * b[1][2]);
  const double q4 = motor_angles[3] / b[3][3];
  const double q5 = (motor_angles[4] - b[4][3] * q4) / b[4][4];
  const double q6 = motor_angles[5] / b[5][5];
  return {q1, q2, q3, q4, q5, q6};
}

std::array<double, 6> ArmHardwareInterface::motorToJointVelocities(
  const std::array<double, 6> & motor_velocities) const
{
  return motorToJointAngles(motor_velocities);
}

TendonAngles ArmHardwareInterface::estimateCoupledPosition(
  const TendonAngles & encoder_equivalent) const
{
  if (!tendon_model_) {
    return encoder_equivalent;
  }
  return tendon_model_->estimate(
    encoder_equivalent, previous_tendon_estimate_ ? &*previous_tendon_estimate_ : nullptr);
}

std::array<double, 6> ArmHardwareInterface::motorIqToJointEffort(
  const std::array<double, 6> & motor_iq) const
{
  // This is the B^T map for the configured position transform m = Bq.
  // Values remain generalized raw-Iq units until motor-specific torque
  // constants and protocol scaling are configured.
  std::array<double, 6> effort{};
  for (std::size_t joint = 0; joint < effort.size(); ++joint) {
    for (std::size_t motor = 0; motor < motor_iq.size(); ++motor) {
      effort[joint] += kCoupledMotorMatrix[motor][joint] * motor_iq[motor];
    }
  }
  return effort;
}

}  // namespace arm_hardware

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)
