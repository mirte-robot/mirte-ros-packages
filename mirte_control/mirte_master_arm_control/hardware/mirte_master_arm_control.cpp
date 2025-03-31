#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "params.hpp"
#include <algorithm>
#include <mirte_master_arm_control.hpp>
#include <unistd.h>

namespace mirte_master_arm_control {

// The data we store per servo
struct Servo_data {
  double data;
  bool init = false;
  bool moved = false;
  double last_move_update = -100;
  double last_request = -100;
};

// Since the plugin itself is loaded once, the member variables
// are shared between all instances of the plugin (ie.
// the arm and the gripper both use the same variables.
// There content is therefore stored in a map, with the
// name as key.
std::map<std::string, std::vector<Servo_data>> servo_data;
std::map<std::string, bool> initialized;
std::map<std::string, int> init_steps;

// Format of the topics and services
const auto topic_format = "io/servo/hiwonder/%s/position";
const auto service_format = "io/servo/hiwonder/%s/set_angle";
const auto SERVO_COMMAND_DIFF = 0.05; // 2.9 degrees
const auto SERVO_MOVED_DIFF =
    0.05; // 0.01 = 0.57 degrees ~= 1 LSB, but it is too sensitive for that.
hardware_interface::return_type
MirteMasterArmHWInterface::write(const rclcpp::Time &time,
                                 const rclcpp::Duration &period) {
  if (initialized[info_.name]) {
    for (auto i = 0; i < NUM_SERVOS; i++) {
      service_requests[i]->angle = hw_commands_[i];
    }
  } else {
    if (std::all_of(std::begin(servo_data[info_.name]),
                    std::end(servo_data[info_.name]),
                    [](Servo_data &x) { return x.init; })) {
      for (auto i = 0; i < NUM_SERVOS; i++) {
        service_requests[i]->angle = servo_data[info_.name][i].data;
      }
      ++(init_steps[info_.name]);
      for (auto i = 0; i < NUM_SERVOS; i++) {
        hw_commands_[i] = servo_data[info_.name][i].data;
      }
      if (init_steps[info_.name] == 50) {
        initialized[info_.name] = true;
      }
    }
  }

  if (std::all_of(std::begin(servo_data[info_.name]),
                  std::end(servo_data[info_.name]),
                  [](auto x) { return x.init; })) {
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
    for (auto i = 0; i < NUM_SERVOS; i++) {
      auto &servo = servo_data[info_.name][i];
      // Only set the servo when there is a new command or the servo is moved by
      // hand or gravity.
      auto diff = std::abs(servo.last_request - service_requests[i]->angle);

      if (diff > SERVO_COMMAND_DIFF || servo.moved) {
        servo.moved = false;
        servo.last_request = service_requests[i]->angle;

        service_requests[i]->degrees = false;
        service_clients[i]->async_send_request(service_requests[i]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

using namespace std::chrono_literals;
void MirteMasterArmHWInterface::connectServices() {
  service_clients.clear();
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    std::string joint_name = info_.joints[i].name;
    std::string servo_name = joint_name.substr(0, joint_name.size() - 6);
    auto client = nh->create_client<mirte_msgs::srv::SetServoAngle>(
        (boost::format(service_format) % servo_name).str());
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }
    service_clients.push_back(client);
  }
}

void MirteMasterArmHWInterface::ServoPositionCallback(
    std::shared_ptr<mirte_msgs::msg::ServoPosition> msg, int joint) {
  auto &servo = servo_data[info_.name][joint];
  servo.data = msg->angle;
  servo.init = true;
  // The servo should only be written to iff the servo gets a new location or
  // when it's moved by gravity, then it needs the command again this will check
  // that the servo is moved
  if (std::abs(servo.last_move_update - servo.data) > SERVO_MOVED_DIFF) {
    servo.last_move_update = servo.data;
    servo.moved = true;
  }
}

void MirteMasterArmHWInterface::read_single(int joint,
                                            const rclcpp::Duration &period) {}

std::vector<hardware_interface::StateInterface>
MirteMasterArmHWInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_states_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MirteMasterArmHWInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

// READ
hardware_interface::return_type
MirteMasterArmHWInterface::read(const rclcpp::Time &time,
                                const rclcpp::Duration &period) {
  for (std::size_t joint_id = 0; joint_id < NUM_SERVOS; ++joint_id) {
    if (servo_data[info_.name][joint_id].init) {
      hw_states_[joint_id] = servo_data[info_.name][joint_id].data;
    }
  }

  return hardware_interface::return_type::OK;
}

using namespace std::chrono_literals;

void MirteMasterArmHWInterface::init_service_clients() {}

hardware_interface::CallbackReturn MirteMasterArmHWInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MirteMasterArmHWInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MirteMasterArmHWInterface::ros_spin() { rclcpp::spin(nh); }

using namespace std::placeholders;
MirteMasterArmHWInterface::MirteMasterArmHWInterface(){};
hardware_interface::CallbackReturn MirteMasterArmHWInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  NUM_SERVOS = info_.joints.size();
  initialized.insert({info_.name, false});
  init_steps.insert({info_.name, 0});

  // TODO: As far as I know we are not able to get the nodehandle
  // from the plugin, so we need to start one ourselves.
  std::stringstream ss;
  ss << info_.name << "_hw_interface";
  nh = rclcpp::Node::make_shared(ss.str());
  this->ros_thread = std::jthread([this] { this->ros_spin(); });

  // Initialize custom members
  std::vector<Servo_data> sd_vector;
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    Servo_data sd;

    sd_vector.push_back(sd);
    _servo_position.push_back(0);
    _servo_position_update_time.push_back(nh->now());

    service_requests.push_back(
        std::make_shared<mirte_msgs::srv::SetServoAngle::Request>());
  }
  servo_data.insert({info_.name, sd_vector});

  // ROS2 control interfaces
  hw_states_.resize(info_.joints.size(),
                    std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(),
                               std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  logger_ = std::make_shared<rclcpp::Logger>(
      rclcpp::get_logger("controller_manager.resource_manager.hardware_"
                         "component.system.MIRTE_arm"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' have %s state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MirteMasterArmHWInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Initialize the servo topics
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    std::string joint_name = info_.joints[i].name;
    std::string servo_name = joint_name.substr(0, joint_name.size() - 6);
    auto servo_topic = (boost::format(topic_format) % servo_name).str();

    servo_pos_subs_.push_back(
        nh->create_subscription<mirte_msgs::msg::ServoPosition>(
            servo_topic, 1,
            [this, i](std::shared_ptr<mirte_msgs::msg::ServoPosition> msg) {
              this->ServoPositionCallback(msg, i);
            }));
  }

  this->connectServices();

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

} // namespace mirte_master_arm_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mirte_master_arm_control::MirteMasterArmHWInterface,
                       hardware_interface::SystemInterface)
