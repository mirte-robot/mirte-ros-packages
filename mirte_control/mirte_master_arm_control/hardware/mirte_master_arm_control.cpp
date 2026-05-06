#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <algorithm>
#include <mirte_master_arm_control.hpp>
#include <unistd.h>

namespace mirte_master_arm_control {
using namespace std::chrono_literals;

// The data we store per servo
struct Servo_data {
  double data = NAN; // unknown position, so hw_control will also not send 0
                     // (otherwise default) back.
  bool init = false;
  bool moved = false;
  double last_move_update = -100;
  double last_request = -100;
  // timestamp for last commanded position, if it's too old and the position is
  // different, the servo might be stuck and need to send safe commands to
  // prevent damage
  rclcpp::Time last_command_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool sent_stuck_command = false; // only send it once to go to the current
                                   // position (cancel original command)
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
const auto service_format = "io/servo/hiwonder/%s/set_angle_with_speed";
const auto enable_format = "enable_arm_control";
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

      if (diff > this->servo_update_dead_band_ || servo.moved ||
          (std::isnan(servo.last_request) &&
           !std::isnan(
               service_requests[i]->angle)) // going from nan to some value
      ) {
        servo.moved = false;
        servo.last_request = service_requests[i]->angle;
        servo.last_command_time = time;
        service_requests[i]->degrees = false;
        service_requests[i]->rate = NAN; // use default rate (0.1s target time)
        servo.sent_stuck_command = false;
        if (this->enable) {
          service_clients[i]->async_send_request(service_requests[i]);
        }
      }

      if (servo.last_command_time + rclcpp::Duration(1s) < time &&
          std::abs(servo.data - servo.last_request) >
              (2.0 * this->servo_update_dead_band_) &&
          !servo.sent_stuck_command) {
        // The servo might be stuck, resend the command to prevent damage
        // when the servo is 'moved', then the original command is resent.
        RCLCPP_WARN(get_logger(),
                    "Servo %d might be stuck, resending command. Current: %f "
                    "OriginalTarget: %f",
                    i, servo.data, servo.last_request);
        servo.last_command_time = time;
        servo.sent_stuck_command = true; // only do this once
        service_requests[i]->degrees = false;
        service_requests[i]->rate = NAN; // use default rate (0.1s target time)
        // send the current position as command to prevent damage
        service_requests[i]->angle = servo.data;
        if (this->enable) {
          service_clients[i]->async_send_request(service_requests[i]);
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

bool MirteMasterArmHWInterface::connectServices() {
  service_clients.clear();
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    std::string joint_name = info_.joints[i].name;
    std::string servo_name = joint_name.substr(0, joint_name.size() - 6);
    std::string service_name =
        (boost::format(service_format) % servo_name).str();
    auto client = nh->create_client<mirte_msgs::srv::SetServoAngleWithSpeed>(
        service_name);
    auto MAX_WAIT_TIME = 10;
    auto wait_time = 0;
    // while (!client->wait_for_service(1s) && wait_time < MAX_WAIT_TIME) {
    //   wait_time++;
    //   if (!rclcpp::ok()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
    //                  "Interrupted while waiting for the service. Exiting.");
    //     return false;
    //   }
    //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    //               (boost::format("service %s not available, waiting
    //               again...") %
    //                service_name)
    //                   .str()
    //                   .c_str());
    // }
    // if (wait_time == MAX_WAIT_TIME) {
    //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
    //                "Could not connect to service %s", service_name.c_str());
    //   return false;
    // }
    service_clients.push_back(client);
  }
  this->enable_arm_service = nh->create_service<std_srvs::srv::SetBool>(
      enable_format,
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        this->enable = request->data;
        response->success = true;
        response->message =
            this->enable ? "Arm control enabled" : "Arm control disabled";
      });
  return true;
}

void MirteMasterArmHWInterface::ServoPositionCallback(
    std::shared_ptr<mirte_msgs::msg::ServoPosition> msg, int joint) {
  if (msg->angle == 0 && msg->raw == 0) {
    // std::cout << "Received default servo position for joint " << joint
    //           << ", ignoring this message." << std::endl;
    // This is the default value when the servo position is not yet published,
    // ignore this
    return;
  }
  auto &servo = servo_data[info_.name][joint];
  servo.data = msg->angle;
  servo.init = true;
  // The servo should only be written to iff the servo gets a new location or
  // when it's moved by gravity, then it needs the command again this will check
  // that the servo is moved
  if (std::abs(servo.last_move_update - servo.data) >
      this->servo_moved_dead_band_) {
    servo.last_move_update = servo.data;
    servo.moved = true;
  }
}

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
MirteMasterArmHWInterface::MirteMasterArmHWInterface() {};
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
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(nh);
    param_listener_->setUserCallback(
        [this](const auto &params) { this->updateParams(params); });
    this->updateParams(param_listener_->get_params());
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Initialize custom members
  std::vector<Servo_data> sd_vector;
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    Servo_data sd;

    sd_vector.push_back(sd);
    _servo_position.push_back(0);
    _servo_position_update_time.push_back(nh->now());

    service_requests.push_back(
        std::make_shared<mirte_msgs::srv::SetServoAngleWithSpeed::Request>());
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

  if (!this->connectServices()) {
    RCLCPP_ERROR(get_logger(), "Could not connect to services");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = NAN;
    hw_commands_[i] = NAN;
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void MirteMasterArmHWInterface::updateParams(Params params) {
  this->params_ = params;
  for (auto &service_request : this->service_requests) {
    // NOTE: This doesnt work as the service_requests are empty at the
    // beginning.
    //  NOTE: Also rate doesnt work as wanted, rate is rad/s, but this
    // is target time.
    // RCLCPP_INFO(rclcpp::get_logger("mirte_master_arm_control"),"Setting
    // servo_target_time to %f seconds.", params.servo_target_time);
    // service_request->rate =
    // std::clamp(static_cast<float>(params.servo_target_time),
    // 0.01f, 10.0f);
  }
  RCLCPP_INFO(rclcpp::get_logger("mirte_master_arm_control"),
              "Updated servo_target_time to %f seconds.",
              params.servo_target_time);
  RCLCPP_INFO(rclcpp::get_logger("mirte_master_arm_control"),
              "Updated servo_moved_dead_band to %f radians.",
              params.servo_moved_dead_band);
  RCLCPP_INFO(rclcpp::get_logger("mirte_master_arm_control"),
              "Updated servo_update_dead_band to %f radians.",
              params.servo_update_dead_band);
  this->servo_moved_dead_band_ = params.servo_moved_dead_band;
  this->servo_update_dead_band_ = params.servo_update_dead_band;
  // TODO: make it configurable, right now it's 1s and 2x
  // servo_update_dead_band_

  // this->servo_stuck_timeout_ =
  // rclcpp::Duration::from_seconds(params.servo_stuck_timeout);
  // this->servo_stuck_trigger_diff_ = params.servo_stuck_trigger_diff;
}

} // namespace mirte_master_arm_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mirte_master_arm_control::MirteMasterArmHWInterface,
                       hardware_interface::SystemInterface)
