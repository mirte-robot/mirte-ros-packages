// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mirte_control/mirte_pioneer_system.hpp"

#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mirte_control {

std::string convert_to_snake_case(const std::string &input) {
  std::string result = "";

  for (auto it = input.cbegin(); it != input.cend(); ++it) {
    if (std::isupper(*it) && !result.empty()) {
      result.push_back('_');
    }
    result.push_back(std::tolower(*it));
  }

  return result;
}

// TODO: Maybe make this all configurable so N wheels resulting in a combinable
// Mirte Master Hardware Interface
hardware_interface::CallbackReturn MirtePioneerSrvSystemHardware::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  last_cmd_left_ = 0;
  last_cmd_right_ = 0;

  // TODO: Make wheel names configurable based, like
  // https://github.com/joshnewans/diffdrive_arduino/blob/humble/description/ros2_control/diffbot.ros2_control.xacro

  // TODO: Maybe move this to a later stage if that would make sense
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared(convert_to_snake_case(get_name()));
  auto logger_name = std::string(node->get_namespace()).substr(1) + "." +
                     std::string(node->get_name());
  logger_ = rclcpp::get_logger(logger_name);

  // TODO: Make wheel service names configurable (Including namespace)
  left_client_ = node->create_client<mirte_msgs::srv::SetMotorSpeed>(
      "io/motor/left/set_speed");
  right_client_ = node->create_client<mirte_msgs::srv::SetMotorSpeed>(
      "io/motor/right/set_speed");

  while (!left_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_.value(),
                   "Interrupted while waiting for the service. Exiting.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_.value(), "service not available, waiting again...");
  }

  // TODO: conbine with above
  while (!right_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_.value(),
                   "Interrupted while waiting for the service. Exiting.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_.value(), "service not available, waiting again...");
  }

  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(logger_.value(),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          logger_.value(),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(logger_.value(),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          logger_.value(),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          logger_.value(),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MirtePioneerSrvSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MirtePioneerSrvSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MirtePioneerSrvSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(logger_.value(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MirtePioneerSrvSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(logger_.value(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MirtePioneerSrvSystemHardware::read(const rclcpp::Time & /*time*/,
                                    const rclcpp::Duration &period) {
  for (uint i = 0; i < hw_commands_.size(); i++) {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_commands_[i];
    hw_velocities_[i] = hw_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
mirte_control ::MirtePioneerSrvSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // This function converts cmd[0] to pwm and calls that service

  // NOTE: this *highly* depends on the voltage of the motors!!!!
  // For 5V power bank: 255 pwm = 90 ticks/sec -> ca 2 rot/s (4*pi)
  // For 6V power supply: 255 pwm = 120 ticks/sec -> ca 3 rot/s
  // (6*pi)

  auto left_request =
      std::make_shared<mirte_msgs::srv::SetMotorSpeed::Request>();
  int left_speed =
      std::max(std::min(int(hw_commands_[0] / (6 * M_PI) * 100), 100), -100);
  if (left_speed != last_cmd_left_) {
    left_request->speed = left_speed;
    auto result = left_client_->async_send_request(left_request);
    last_cmd_left_ = left_speed;
  }

  auto right_request =
      std::make_shared<mirte_msgs::srv::SetMotorSpeed::Request>();
  int right_speed =
      std::max(std::min(int(hw_commands_[1] / (6 * M_PI) * 100), 100), -100);
  if (right_speed != last_cmd_right_) {
    right_request->speed = right_speed;
    auto result = right_client_->async_send_request(right_request);
    last_cmd_right_ = right_speed;
  }

  return hardware_interface::return_type::OK;
}

} // namespace mirte_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mirte_control::MirtePioneerSrvSystemHardware,
                       hardware_interface::SystemInterface)
