#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <algorithm>
#include <boost/algorithm/string.hpp>

#include <mirte_base_control.hpp>
namespace mirte_base_control {

double MirteBaseHWInterface::calc_speed_map(int joint, double target,
                                            const rclcpp::Duration &period) {
  return std::max(
      std::min(int(target / this->settings.max_rot_speed * 100), 100), -100);
}

int MirteBaseHWInterface::calculate_single_speed(
    int joint, double speed, const rclcpp::Duration &period) {
  double speed_mapped;

  speed_mapped = this->calc_speed_map(joint, speed, period);
  speed_mapped = std::clamp<double>(speed_mapped, -max_speed, max_speed);

  return speed_mapped;
}

bool MirteBaseHWInterface::write_single(int joint, double speed,
                                        const rclcpp::Duration &period,
                                        bool &updated) {
  auto speed_mapped = calculate_single_speed(joint, speed, period);
  bool in_deadzone =
      std::abs(speed_mapped) <
      this->settings.cmd_vel_deadzone; // if set to 0 bc too low percentage
  bool to_deadzone =
      _last_sent_cmd[joint] != 0 &&
      in_deadzone; // if moving from moving to deadzone, force update
  if (in_deadzone) {
    speed_mapped = 0;
  }
  auto diff = std::abs(speed_mapped - _last_sent_cmd[joint]);
  if (!this->settings.use_single_update) {
    if (diff >= this->settings.cmd_vel_update_deadzone || to_deadzone) {
      updated = true;
      this->_last_sent_cmd[joint] = speed_mapped;

      if (this->settings.use_topic_update) {
        // publish topic
        this->publish_msgs[joint]->speed = speed_mapped;
        this->speed_publishers[joint]->publish(*this->publish_msgs[joint]);
      } else {
        this->service_requests[joint]->speed = (int)speed_mapped;
        this->service_clients[joint]->async_send_request(
            this->service_requests[joint]);
      }
    }
  } else {
    if (to_deadzone ||
        diff >= this->settings.cmd_vel_update_deadzone) { // if upd_deadzone ==
                                                          // 0, then always upd
      updated = true;
      this->_last_sent_cmd[joint] = speed_mapped;
    }
    // if another joint is sending an update, then just send all joints,
    // otherwise wait for the next loop.
    this->set_speed_multiple_request->speeds[joint].speed = speed_mapped;
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
    if (updated && this->settings.use_single_update) {
      if (this->settings.use_topic_update) {
        // publish topic
        for (size_t i = 0; i < NUM_JOINTS; i++) {
          this->set_speed_multiple_publish_msg->speeds[i].speed =
              this->set_speed_multiple_request->speeds[i].speed;
        }
        this->set_speed_multiple_publisher->publish(
            *this->set_speed_multiple_publish_msg);
      } else {
        this->set_speed_multiple_client->async_send_request(
            this->set_speed_multiple_request);
      }
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
  const std::lock_guard<std::mutex> lock(this->encoder_mutex);

  // if (_last_value[joint] == 0) {
  //   _last_value[joint] = _wheel_encoder[joint];
  //   // when starting, the encoders dont have to be at 0. Without this, the
  //   odom
  //   // can jump at the first loop
  // }
  // int16_t diff_ticks = _wheel_encoder[joint] - _last_value[joint];

  // _last_value[joint] = _wheel_encoder[joint];

  auto latest_msg = this->latest_msgs_[joint].readFromRT();

  if (latest_msg == nullptr || latest_msg->first == nullptr ||
      latest_msg->second == nullptr) {
    // no message received yet, do nothing
    // diff_ticks = 0;
    return;
  }
  auto latest_encoder_val = latest_msg->first->value;
  auto diff_ticks = latest_msg->first->value - latest_msg->second->value;
  auto period_sec = (rclcpp::Time(latest_msg->first->header.stamp) -
                     rclcpp::Time(latest_msg->second->header.stamp))
                        .seconds();
  // velo = diff_ticks
  // _last_value[joint] = latest_msg->first->value;

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

  // Doesn't work with single pin encoders, but no'ones using pos for odom with
  // those anyways.
  double distance_pos_rad = latest_encoder_val * radPerEncoderTick * 1.0;

  pos[joint] = distance_pos_rad; // TODO: fix with last pos
  if (period_sec < 0.01) {
    vel[joint] = 0;
    return;
  }
  auto velo = distance_rad / period_sec;
  if (std::abs(velo) > 1000.0) { // if velocity is way too high, assume error in
                                 // encoder. More than 1000rad/s is not possible
    vel[joint] = 0;
  } else {
    vel[joint] = velo;
  }
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

bool MirteBaseHWInterface::init_service_clients() {

  std::cout << "init_service_clients" << std::endl;
  std::cout << "settings.use_single_update: "
            << this->settings.use_single_update << std::endl;
  std::cout << "settings.use_topic_update: " << this->settings.use_topic_update
            << std::endl;

  service_clients.clear();
  service_requests.clear();
  if (!this->settings.use_single_update) {
    {
      const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
      if (this->settings.use_topic_update) {
        this->publish_msgs.clear();
        this->speed_publishers.clear();
        for (size_t i = 0; i < NUM_JOINTS; i++) {
          auto pub = (boost::format(this->settings.separate_update_format) %
                      this->joints[i])
                         .str();
          auto publisher =
              nh->template create_publisher<mirte_msgs::msg::SetSpeed>(pub, 1);
          auto msg = std::make_shared<mirte_msgs::msg::SetSpeed>();
          this->publish_msgs.push_back(msg);
          this->speed_publishers.push_back(publisher);
        }
      } else {
        service_clients.clear();
        service_requests.clear();
        for (size_t i = 0; i < NUM_JOINTS; i++) {
          auto motor_service =
              (boost::format(this->settings.separate_update_format) %
               this->joints[i])
                  .str();
          auto client = nh->create_client<mirte_msgs::srv::SetMotorSpeed>(
              motor_service); // TODO: add persistent connection

          auto MAX_WAIT_TIME = 10;
          auto wait_time = 0;
          service_clients.push_back(client);
          service_requests.push_back(
              std::make_shared<mirte_msgs::srv::SetMotorSpeed::Request>());
        }
      }
    }

  } else {
    // single
    // set speed multiple request is always filled by each motor, if topic, then
    // copied to publish msg, if service, then sent as service request
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

    if (this->settings.use_topic_update) {
      this->set_speed_multiple_publish_msg =
          std::make_shared<mirte_msgs::msg::SetSpeedMultiple>();
      this->set_speed_multiple_publish_msg->speeds.resize(NUM_JOINTS);
      for (auto i = 0; i < NUM_JOINTS; i++) {
        this->set_speed_multiple_publish_msg->speeds[i].name = this->joints[i];
      }
      this->set_speed_multiple_publisher =
          nh->template create_publisher<mirte_msgs::msg::SetSpeedMultiple>(
              this->settings.single_update_name, 1);
    } else {
      this->set_speed_multiple_client =
          nh->create_client<mirte_msgs::srv::SetSpeedMultiple>(
              this->settings.single_update_name);
      this->set_speed_multiple_client->async_send_request(
          this->set_speed_multiple_request);
    }

    auto MAX_WAIT_TIME = 10;
    auto wait_time = 0;
  }

  assert(this->service_clients.size() == NUM_JOINTS ||
         this->settings.use_single_update || this->settings.use_topic_update);
  assert(this->service_requests.size() == NUM_JOINTS ||
         this->settings.use_single_update || this->settings.use_topic_update);
  assert(this->publish_msgs.size() == NUM_JOINTS ||
         !this->settings.use_topic_update || this->settings.use_single_update);
  assert(this->speed_publishers.size() == NUM_JOINTS ||
         !this->settings.use_topic_update || this->settings.use_single_update);

  return true;
}

hardware_interface::CallbackReturn MirteBaseHWInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(rclcpp::get_logger("MirteBaseSystemHardware"),
              "Activating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("MirteBaseSystemHardware"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MirteBaseHWInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // write 0s to all motors in write() to stop the robot

  for (size_t i = 0; i < NUM_JOINTS; i++) {
    cmd[i] = 0;
  }
  this->write(rclcpp::Time(),
              rclcpp::Duration(0, 100)); // send stop command to motors
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

void MirteBaseHWInterface::read_settings() {
  if (!info_.hardware_parameters.count(TICKS_PARAM_NAME)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MirteBaseSystemHardware"),
                        "Missing required hardware parameter: ticks");
    throw std::runtime_error("Missing required hardware parameter: ticks");
  }
  this->settings.ticks =
      std::stod(info_.hardware_parameters.at(TICKS_PARAM_NAME));
  if (info_.hardware_parameters.count(SEPARATE_UPDATE_FORMAT_PARAM_NAME)) {
    this->settings.separate_update_format =
        info_.hardware_parameters.at(SEPARATE_UPDATE_FORMAT_PARAM_NAME);
  }
  if (info_.hardware_parameters.count(SINGLE_UPDATE_NAME_PARAM_NAME)) {
    this->settings.single_update_name =
        info_.hardware_parameters.at(SINGLE_UPDATE_NAME_PARAM_NAME);
  }
  if (info_.hardware_parameters.count(USE_TOPIC_UPDATE_PARAM_NAME)) {
    this->settings.use_topic_update =
        boost::algorithm::to_lower_copy(info_.hardware_parameters.at(
            USE_TOPIC_UPDATE_PARAM_NAME)) == "true";
  }
  if (info_.hardware_parameters.count(USE_SINGLE_UPDATE_PARAM_NAME)) {
    std::cout << "use_single_update: "
              << info_.hardware_parameters.at(USE_SINGLE_UPDATE_PARAM_NAME)
              << std::endl;
    this->settings.use_single_update =
        boost::algorithm::to_lower_copy(info_.hardware_parameters.at(
            USE_SINGLE_UPDATE_PARAM_NAME)) == "true";
  }
  if (info_.hardware_parameters.count(CMD_VEL_DEADZONE_PARAM_NAME)) {
    this->settings.cmd_vel_deadzone =
        std::stod(info_.hardware_parameters.at(CMD_VEL_DEADZONE_PARAM_NAME));
  }
  if (info_.hardware_parameters.count(CMD_VEL_UPDATE_DEADZONE_PARAM_NAME)) {
    this->settings.cmd_vel_update_deadzone = std::stod(
        info_.hardware_parameters.at(CMD_VEL_UPDATE_DEADZONE_PARAM_NAME));
    if (this->settings.cmd_vel_update_deadzone == 0) {
      // this->settings.always_send = true;
    }
  }
  if (info_.hardware_parameters.count(ENCODER_TOPIC_FORMAT_PARAM_NAME)) {
    this->settings.encoder_topic_format =
        info_.hardware_parameters.at(ENCODER_TOPIC_FORMAT_PARAM_NAME);
  }
  if (info_.hardware_parameters.count(MAX_ROT_SPEED_PARAM_NAME)) {
    this->settings.max_rot_speed =
        std::stod(info_.hardware_parameters.at(MAX_ROT_SPEED_PARAM_NAME));
  }
}

using namespace std::placeholders;
MirteBaseHWInterface::MirteBaseHWInterface() {};
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
  this->read_settings();
  this->NUM_JOINTS = info.joints.size();
  if (this->NUM_JOINTS > 2) {
    this->bidirectional = true;
  }
  std::cout << "on_init" << __LINE__ << std::endl;
  // Initialize raw data
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    // _wheel_encoder.push_back(0);
    latest_msgs_.push_back(
        realtime_tools::RealtimeBuffer<
            std::pair<mirte_msgs::msg::Encoder::ConstSharedPtr,
                      mirte_msgs::msg::Encoder::ConstSharedPtr>>{});
    // _wheel_encoder_update_time.push_back(nh->now());
    _last_value.push_back(0);
    _last_wheel_cmd_direction.push_back(0);
    // _last_cmd.push_back(0);
    _last_sent_cmd.push_back(-1000);

    pos.push_back(0);
    vel.push_back(0);
    eff.push_back(0);
    cmd.push_back(0);
  }
  // assert(_wheel_encoder.size() == NUM_JOINTS);
  // assert(_last_value.size() == NUM_JOINTS);
  assert(_last_wheel_cmd_direction.size() == NUM_JOINTS);
  // assert(_last_cmd.size() == NUM_JOINTS);
  assert(pos.size() == NUM_JOINTS);
  assert(vel.size() == NUM_JOINTS);
  assert(eff.size() == NUM_JOINTS);
  assert(cmd.size() == NUM_JOINTS);

  // this->joints = {"left", // Edit the control.yaml when using this for the
  //                         // normal mirte as well
  //                 "right"};
  // if (NUM_JOINTS == 4) {
  //   this->joints = {"front_left", "rear_left", "rear_right", "front_right"};
  // }
  this->joints.clear();
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    auto name = info.joints[i].name;
    // remove _wheel_joint suffix if it exists, to get the joint name (e.g.
    // left, right), needed for the topic and service names formatting
    if (name.ends_with("_wheel_joint")) {
      name = name.substr(0, name.size() - std::string("_wheel_joint").size());
    }
    this->joints.push_back(name);
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

  // Initialize publishers and subscribers
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    auto encoder_topic =
        (boost::format(this->settings.encoder_topic_format) % this->joints[i])
            .str();
    std::cout << "add encoder topic: " << encoder_topic << std::endl;
    wheel_encoder_subs_.push_back(
        nh->create_subscription<mirte_msgs::msg::Encoder>(
            encoder_topic, 1,
            [this, i](std::shared_ptr<mirte_msgs::msg::Encoder> msg) {
              this->WheelEncoderCallback(msg, i);
            }));
  }
  assert(joints.size() == NUM_JOINTS);
  if (!this->init_service_clients()) {
    RCLCPP_ERROR(rclcpp::get_logger("MirteBaseSystemHardware"),
                 "Could not connect to services");
    return hardware_interface::CallbackReturn::ERROR;
  }
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

} // namespace mirte_base_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mirte_base_control::MirteBaseHWInterface,
                       hardware_interface::SystemInterface)
