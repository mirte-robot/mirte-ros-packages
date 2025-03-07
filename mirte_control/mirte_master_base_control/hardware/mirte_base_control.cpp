#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "params.hpp"
#include <algorithm>
#include <mirte_base_control.hpp>
namespace mirte_base_control {

bool equal_gains(control_toolbox::Pid::Gains lhs,
                 control_toolbox::Pid::Gains rhs) {
  return lhs.p_gain_ == rhs.p_gain_ && lhs.i_gain_ == rhs.i_gain_ &&
         lhs.d_gain_ == lhs.d_gain_;
}

double MirteBaseHWInterface::calc_speed_pid(int joint, double target,
                                            const rclcpp::Duration &period) {
  // WARN: this is unused as the PID of ros-control is used.
  // if moving from 0 to something else, don't wait for the PID to catch up,
  // but use the mapping function to immediately give it a kinda okay value
  static bool start[4] = {true};
  auto pid = this->pids[joint];
  if (target == 0) {
    pid->reset();
    start[joint] = true;
    // Fix for dynamic reconfigure of all 4 PID controllers:
    auto g = this->reconfig_pid->getGains();
    if (!equal_gains(pid->getGains(), g)) {
      pid->setGains(g);
    }
    return 0;
  }

  if (start[joint]) {
    start[joint] = false;
    return calc_speed_map(joint, target, period);
  }

  auto diff_enc_time_upd = nh->now() - _wheel_encoder_update_time[joint];
  if (diff_enc_time_upd > rclcpp::Duration(1, 0) &&
      _last_cmd[joint] >
          30) { // if the motors don't move, no need to fall back yet
    RCLCPP_WARN_STREAM(
        rclcpp::get_logger("rclcpp"),
        "Encoder "
            << joint
            << " not reporting data, falling back to mapping calculation");
    return calc_speed_map(joint, target, period);
  }
  auto curr_speed = vel[joint];
  auto err = target - curr_speed;
  auto pid_cmd = pid->computeCommand(err, period.nanoseconds());
  return pid_cmd + _last_cmd[joint];
}

double MirteBaseHWInterface::calc_speed_map(int joint, double target,
                                            const rclcpp::Duration &period) {
  return std::max(std::min(int(target / (6.0 * M_PI) * 100), 100), -100);
}

int MirteBaseHWInterface::calculate_single_speed(
    int joint, double speed, const rclcpp::Duration &period) {
  double speed_mapped;
  if (this->enablePID && false) {
    speed_mapped = this->calc_speed_pid(joint, speed, period);
  } else {
    speed_mapped = this->calc_speed_map(joint, speed, period);
  }
  speed_mapped = std::clamp<double>(speed_mapped, -max_speed, max_speed);

  return speed_mapped;
}

bool MirteBaseHWInterface::write_single(int joint, double speed,
                                        const rclcpp::Duration &period,
                                        bool &updated) {
  // std::cout << "write_single" << joint << std::endl;
  auto speed_mapped = calculate_single_speed(joint, speed, period);
  auto diff = std::abs(speed_mapped - _last_sent_cmd[joint]);
  _last_cmd[joint] = speed_mapped;
  if (diff > 1.0) {
    updated = true;
    _last_sent_cmd[joint] = speed_mapped;
    if (!this->use_single_client) {
      service_requests[joint]->speed = (int)speed_mapped;
      service_clients[joint]->async_send_request(service_requests[joint]);
      //     TODO: loop until response is received

      if (!true) {
        this->start_reconnect();
        return false;
      }

    } else {
      this->set_speed_multiple_request->speeds[joint].speed = speed_mapped;
    }
  }
  return true;
}

hardware_interface::return_type
MirteBaseHWInterface::write(const rclcpp::Time &time,
                            const rclcpp::Duration &period) {
  if (running_) {
    // make sure the clients don't get overwritten while calling them
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);

    // cmd[0] = ros_control calculated speed of left motor in rad/s
    // cmd[1] = ros_control calculated speed of right motor in rad/s

    // This function converts cmd[0] to pwm and calls that service

    // NOTE: this *highly* depends on the voltage of the motors!!!!
    // For 5V power bank: 255 pwm = 90 ticks/sec -> ca 2 rot/s (4*pi)
    // For 6V power supply: 255 pwm = 120 ticks/sec -> ca 3 rot/s
    // (6*pi)
    bool updated = false;
    for (size_t i = 0; i < NUM_JOINTS; i++) {
      if (!write_single(i, cmd[i], period, updated)) {
        // TODO: add reconnect
        return hardware_interface::return_type::ERROR;
      }
    }
    if (updated && this->use_single_client) {
      this->set_speed_multiple_client->async_send_request(
          this->set_speed_multiple_request);
    }
    // Set the direction in so the read() can use it
    // TODO: this does not work properly, because at the end of a series
    // cmd_vel is negative, while the rotation is not
    for (size_t i = 0; i < NUM_JOINTS; i++) {
      _last_wheel_cmd_direction[i] = cmd[i] > 0.0 ? 1 : -1;
    }
  }
  return hardware_interface::return_type::OK;
}

void MirteBaseHWInterface::read_single(int joint,
                                       const rclcpp::Duration &period) {
  if (_last_value[joint] == 0) {
    _last_value[joint] = _wheel_encoder[joint];
    // when starting, the encoders dont have to be at 0. Without this, the odom
    // can jump at the first loop
  }
  auto diff_ticks = _wheel_encoder[joint] - _last_value[joint];

  _last_value[joint] = _wheel_encoder[joint];
  double radPerEncoderTick = rad_per_enc_tick();
  double distance_rad;
  if (bidirectional) { // if encoder is counting bidirectional, then it
                       // decreases by itself, dont want to use
                       // last_wheel_cmd_direction
    distance_rad = diff_ticks * radPerEncoderTick * 1.0;
  } else {
    distance_rad =
        diff_ticks * radPerEncoderTick * _last_wheel_cmd_direction[joint] * 1.0;
  }
  pos[joint] += distance_rad;
  vel[joint] = distance_rad / period.seconds(); // WHY: was this turned off?
}
std::vector<hardware_interface::StateInterface>
MirteBaseHWInterface::export_state_interfaces() {
  std::cout << "export_state_interfaces" << std::endl;
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    std::cout << "export_state_interfaces " << i
              << " name: " << info_.joints[i].name << std::endl;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel[i]));
  }
  std::cout << "export_state_interfaces done" << std::endl;
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MirteBaseHWInterface::export_command_interfaces() {
  std::cout << "export_command_interfaces" << std::endl;
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd[i]));
  }
  std::cout << "export_command_interfaces done" << std::endl;
  return command_interfaces;
}

hardware_interface::return_type
MirteBaseHWInterface::read(const rclcpp::Time &time,
                           const rclcpp::Duration &period) {
  // std::cout << "read" << std::endl;
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    this->read_single(i, period);
  }
  // std::cout << "read done" << std::endl;

  return hardware_interface::return_type::OK;
}
using namespace std::chrono_literals;

void MirteBaseHWInterface::init_service_clients() {
  if (!this->use_single_client) {

    for (auto joint : this->joints) {
      auto service = (boost::format(service_format) % joint).str();
      //     RCLCPP_INFO_STREAM("Waiting for service " << service); // todo
      //     print rclcpp::service::waitForService(service, -1); // TODO: wait
      //     after creating service
    }
    {
      const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
      service_clients.clear();
      service_requests.clear();
      for (size_t i = 0; i < NUM_JOINTS; i++) {
        auto client = nh->create_client<mirte_msgs::srv::SetMotorSpeed>(
            (boost::format(service_format) % this->joints[i])
                .str()); // TODO: add persistent connection
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
        service_requests.push_back(
            std::make_shared<mirte_msgs::srv::SetMotorSpeed::Request>());
      }
    }

  } else {
    service_clients.clear();
    service_requests.clear();
    this->set_speed_multiple_client =
        nh->create_client<mirte_msgs::srv::SetSpeedMultiple>(
            "io/motor/motorservocontroller/set_multiple_speeds");
    while (!this->set_speed_multiple_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return;
        // return hardware_interface::CallbackReturn::ERROR;
      }
    }

    this->set_speed_multiple_request =
        std::make_shared<mirte_msgs::srv::SetSpeedMultiple::Request>();
    this->set_speed_multiple_request->speeds.resize(NUM_JOINTS);
    for (size_t i = 0;
         i <
         std::min((size_t)NUM_JOINTS,
                  (size_t)this->set_speed_multiple_request->speeds.max_size());
         i++) {
      this->set_speed_multiple_request->speeds[i].speed = 0;
      this->set_speed_multiple_request->speeds[i].name = this->joints[i];
    }
    this->set_speed_multiple_client->async_send_request(
        this->set_speed_multiple_request);
  }
}

unsigned int detect_joints(std::shared_ptr<rclcpp::Node> nh) {
  std::string type;
  // return info_.joints.size();
  //   nh->param<std::string>("mobile_base_controller/type", type, ""); // TODO:
  //   params
  if (type.rfind("mecanum", 0) == 0) { // starts with mecanum
    return 4;
  } else if (type.rfind("diff", 0) == 0) { // starts with diff
    return 2;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Unknown type: " << type);
    return 4;
  }
}

hardware_interface::CallbackReturn MirteBaseHWInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(rclcpp::get_logger("MirteBaseSystemHardware"),
              "Activating ...please wait...");

  // for (auto i = 0; i < 2; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("MirteBaseSystemHardware"), "%.1f seconds left...",
  //     2 - i);
  // }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  // // set some default values
  // for (auto i = 0u; i < hw_positions_.size(); i++)
  // {
  //   if (std::isnan(hw_positions_[i]))
  //   {
  //     hw_positions_[i] = 0;
  //     hw_velocities_[i] = 0;
  //     hw_commands_[i] = 0;
  //   }
  // }

  RCLCPP_INFO(rclcpp::get_logger("MirteBaseSystemHardware"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MirteBaseHWInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  // RCLCPP_INFO(rclcpp::get_logger("MirteBaseSystemHardware"), "Deactivating
  // ...please wait...");

  // for (auto i = 0; i < 2; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("MirteBaseSystemHardware"), "%.1f seconds left...",
  //     2 - i);
  // }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  RCLCPP_INFO(rclcpp::get_logger("MirteBaseSystemHardware"),
              "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void MirteBaseHWInterface::ros_spin() { rclcpp::spin(nh); }

using namespace std::placeholders;
MirteBaseHWInterface::MirteBaseHWInterface(){};
hardware_interface::CallbackReturn
MirteBaseHWInterface::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::cout << "starting on_init" << std::endl;
  nh = rclcpp::Node::make_shared("mirte_base_control");

  std::cout << "on_init" << __LINE__ << std::endl;
  running_ = true;
  start_srv_ = nh->create_service<std_srvs::srv::Empty>(
      "start", std::bind(&MirteBaseHWInterface::start_callback, this, _1, _2));
  std::cout << "on_init" << __LINE__ << std::endl;
  stop_srv_ = nh->create_service<std_srvs::srv::Empty>(
      "stop", std::bind(&MirteBaseHWInterface::stop_callback, this, _1, _2));
  std::cout << "on_init" << __LINE__ << std::endl;
  std::cout << "Initializing MirteBaseHWInterface" << std::endl;
  std::cout << "on_init" << __LINE__ << std::endl;
  this->ros_thread = std::jthread([this] { this->ros_spin(); });
  /*
   nh->param<double>("mobile_base_controller/wheel_radius", _wheel_diameter,
                    0.06);
   _wheel_diameter *= 2; // convert from radius to diameter
   nh->param<double>("mobile_base_controller/max_speed", _max_speed,
                    2.0); // TODO: unused
   nh->param<double>("mobile_base_controller/ticks", ticks, 40.0);
   */
  //  info.hardware_parameters.at("ticks");
  this->ticks = std::stod(info.hardware_parameters.at("ticks"));
  // std::string param_file = info.hardware_parameters.at("param_file");
  // parse_params(param_file, nh);
  // std::cout << "on_init" << __LINE__ << std::endl;
  // this->NUM_JOINTS = detect_joints(nh);
  //  std::cout << "on_init" << __LINE__ << std::endl;
  this->NUM_JOINTS = info.joints.size();
  if (this->NUM_JOINTS > 2) {
    this->bidirectional = true;
  }
  std::cout << "on_init" << __LINE__ << std::endl;
  // Initialize raw data
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    _wheel_encoder.push_back(0);
    _wheel_encoder_update_time.push_back(nh->now());
    _last_value.push_back(0);
    _last_wheel_cmd_direction.push_back(0);
    _last_cmd.push_back(0);
    _last_sent_cmd.push_back(0);

    pos.push_back(0);
    vel.push_back(0);
    eff.push_back(0);
    cmd.push_back(0);
  }
  assert(_wheel_encoder.size() == NUM_JOINTS);
  assert(_last_value.size() == NUM_JOINTS);
  assert(_last_wheel_cmd_direction.size() == NUM_JOINTS);
  assert(_last_cmd.size() == NUM_JOINTS);
  assert(pos.size() == NUM_JOINTS);
  assert(vel.size() == NUM_JOINTS);
  assert(eff.size() == NUM_JOINTS);
  assert(cmd.size() == NUM_JOINTS);

  this->joints = {"left", // Edit the control.yaml when using this for the
                          // normal mirte as well
                  "right"};
  if (NUM_JOINTS == 4) {
    this->joints = {"front_left", "rear_left",
                    "rear_right",
                    "front_right"};
  }
  std::cout << "Initializing MirteBaseHWInterface with " << NUM_JOINTS
            << " joints" << std::endl;

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("MirteBaseSystemHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("MirteBaseSystemHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("MirteBaseSystemHardware"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("MirteBaseSystemHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("MirteBaseSystemHardware"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // // connect and register the joint state and velocity interfaces
  // for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
  //   std::string joint =
  //       (boost::format("wheel_%s_joint") % this->joints[i]).str();
  //   hardware_interface::JointStateHandle state_handle(joint, &pos[i],
  //   &vel[i],
  //                                                     &eff[i]);
  //   jnt_state_interface.registerHandle(state_handle);

  //   hardware_interface::JointHandle vel_handle(
  //       jnt_state_interface.getHandle(joint), &cmd[i]);
  //   jnt_vel_interface.registerHandle(vel_handle);
  // }
  // registerInterface(&jnt_state_interface);
  // registerInterface(&jnt_vel_interface);

  // nh->param<bool>("mobile_base_controller/enable_pid", enablePID, false);
  // if (enablePID) {
  //   // dummy pid for dynamic reconfigure.
  //   this->reconfig_pid = std::make_shared<control_toolbox::Pid>(1, 0, 0);
  //   this->reconfig_pid->initParam("mobile_base_controller/", false);
  //   auto gains = this->reconfig_pid->getGains();
  //   for (auto i = 0; i < NUM_JOINTS; i++) {
  //     auto pid = std::make_shared<control_toolbox::Pid>(1, 1, 1);
  //     pid->setGains(gains);
  //     this->pids.push_back(pid);
  //   }
  // }

  // Initialize publishers and subscribers
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    auto encoder_topic =
        (boost::format(encoder_format) % this->joints[i]).str();
    std::cout << "add encoder topic: " << encoder_topic << std::endl;
    wheel_encoder_subs_.push_back(
        nh->create_subscription<mirte_msgs::msg::Encoder>(
            encoder_topic, 1,
            [this, i](std::shared_ptr<mirte_msgs::msg::Encoder> msg) {
              // std::cout << "Encoder callback: " << msg->value << std::endl;
              this->WheelEncoderCallback(msg, i);
            }));
  }
  assert(joints.size() == NUM_JOINTS);
  this->init_service_clients();
  // assert(service_requests.size() == NUM_JOINTS);

  // assert(service_clients.size() == NUM_JOINTS);
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::cout << "finished on_init" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MirteBaseHWInterface::start_reconnect() {
  using namespace std::chrono_literals;
  std::cout << "start_reconnect" << std::endl;
  if (this->reconnect_thread.valid()) { // does it already exist or not?

    // Use wait_for() with zero milliseconds to check thread status.
    auto status = this->reconnect_thread.wait_for(0ms);

    if (status !=
        std::future_status::ready) { // Still running -> already reconnecting
      return;
    }
  }

  /* Run the reconnection on a different thread to not pause the ros-control
    loop. The launch policy std::launch::async makes sure that the task is run
    asynchronously on a new thread. */

  this->reconnect_thread =
      std::async(std::launch::async, [this] { this->init_service_clients(); });
}

/*
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  last_cmd_left_ = 0;
  last_cmd_right_ = 0;

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("diff_drive");
  left_client_ =
  node->create_client<mirte_msgs::srv::SetMotorSpeed>("/mirte/set_left_speed");
  right_client_ =
  node->create_client<mirte_msgs::srv::SetMotorSpeed>("/mirte/set_right_speed");

  while (!left_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for
  the service. Exiting."); return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting
  again...");
  }

  // TODO: conbine with above
  while (!right_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for
  the service. Exiting."); return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting
  again...");
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  your production code hw_start_sec_ =
  std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
  std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your
  production code hw_positions_.resize(info_.joints.size(),
  std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(),
  std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
  std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each
  joint if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MirteBaseSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.",
  joint.name.c_str(), joint.command_interfaces.size()); return
  hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MirteBaseSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.",
  joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
  hardware_interface::HW_IF_VELOCITY); return
  hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MirteBaseSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MirteBaseSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.",
  joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
  hardware_interface::HW_IF_POSITION); return
  hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MirteBaseSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.",
  joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
  hardware_interface::HW_IF_VELOCITY); return
  hardware_interface::CallbackReturn::ERROR;
    }
  }*/

} // namespace mirte_base_control

/*

MirteBaseHWInterface::
*/

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mirte_base_control::MirteBaseHWInterface,
                       hardware_interface::SystemInterface)
