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


hardware_interface::return_type
MirteMasterArmHWInterface::write(const rclcpp::Time &time,
                                 const rclcpp::Duration &period) {

  if (initialized[info_.name]) {
    for (auto i = 0; i < NUM_SERVOS; i++) {
      service_requests[i]->angle = hw_commands_[i];
    }
  } else {
    if (std::all_of(std::begin(servo_data[info_.name]), std::end(servo_data[info_.name]),
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
        RCLCPP_INFO(get_logger(), "------------------------------------------All servo's initialized for %s", info_.name.c_str());
      }
    }
  }


  if (std::all_of(std::begin(servo_data[info_.name]), std::end(servo_data[info_.name]),
                  [](auto x) { return x.init; })) {
   // std::cout << "  bladiebla" << std::endl;
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
    //float last_req[NUM_SERVOS] = {-100.0};
    for (auto i = 0; i < NUM_SERVOS; i++) {

      // Only set the servo when there is a new command or the servo is moved by
      // hand or gravity.
    //  std::cout << i << " hier    " << (servo_data.find(info_.name)->second)[i].moved << std::endl;
      if (servo_data[info_.name][i].last_request != service_requests[i]->angle || servo_data[info_.name][i].moved) {
        //last_req[i] = service_requests[i]->angle;
        servo_data[info_.name][i].moved = false;
        servo_data[info_.name][i].last_request = service_requests[i]->angle;

        service_requests[i]->degrees = false;
        service_clients[i]->async_send_request(service_requests[i]);
        std::cout << info_.joints[i].name << " sending radians: " << service_requests[i]->angle << std::endl;

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


void
MirteMasterArmHWInterface::ServoPositionCallback(std::shared_ptr<mirte_msgs::msg::ServoPosition> msg,
                        int joint) {

    auto &servo = servo_data[info_.name][joint];
    if (!servo.init){
   //    std::cout << info_.joints[joint].name << " initial timestamp: " << msg->header.stamp.sec << std::endl;
   //    std::cout << info_.joints[joint].name << " data read initially as: " << msg->angle << std::endl;
    } else {
   //    std::cout << info_.joints[joint].name << " updated to: " << msg->angle << std::endl;
    }
    servo.data = msg->angle;
    servo.init = true;
    // The servo should only be written to iff the servo gets a new location or
    // when it's moved by gravity, then it needs the command again this will check
    // that the servo is moved
    if (std::abs(servo.last_move_update - servo.data) >
        0.01) { // 0.57deg, bit more than the 24 centidegrees/LSB
      servo.last_move_update = servo.data;
      servo.moved = true;
    }
  }


void MirteMasterArmHWInterface::read_single(int joint,
                                            const rclcpp::Duration &period) {
}

std::vector<hardware_interface::StateInterface>
MirteMasterArmHWInterface::export_state_interfaces() {

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    std::cout << info_.joints[i].name << std::endl;
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MirteMasterArmHWInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

// READ
hardware_interface::return_type
MirteMasterArmHWInterface::read(const rclcpp::Time &time,
                                const rclcpp::Duration &period) {

  for (std::size_t joint_id = 0; joint_id < NUM_SERVOS; ++joint_id) {
    if (servo_data[info_.name][joint_id].init){
      hw_states_[joint_id] = servo_data[info_.name][joint_id].data;
    }
  }

  return hardware_interface::return_type::OK;
}


using namespace std::chrono_literals;

void MirteMasterArmHWInterface::init_service_clients() {
}

hardware_interface::CallbackReturn MirteMasterArmHWInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;


}

hardware_interface::CallbackReturn MirteMasterArmHWInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;

}

void MirteMasterArmHWInterface::ros_spin() { rclcpp::spin(nh); }

using namespace std::placeholders;
MirteMasterArmHWInterface::MirteMasterArmHWInterface(){};
hardware_interface::CallbackReturn MirteMasterArmHWInterface::on_init(
    const hardware_interface::HardwareInfo &info) {


  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  NUM_SERVOS = info_.joints.size();
  initialized.insert({info_.name, false});
  init_steps.insert({info_.name, 0});


  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);

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

    service_requests.push_back(std::make_shared<mirte_msgs::srv::SetServoAngle::Request>());
  }
  servo_data.insert({info_.name, sd_vector});

  // ROS2 control interfaces
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.MIRTE_arm"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn MirteMasterArmHWInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  // Initialize the servo topics
 // std::cout << "number of servos: "  << NUM_SERVOS << std::endl;
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    std::string joint_name = info_.joints[i].name;
    std::string servo_name = joint_name.substr(0, joint_name.size() - 6);
    auto servo_topic =
        (boost::format(topic_format) % servo_name).str();

    std::cout << "add servo topic: " << servo_topic << std::endl;
    servo_pos_subs_.push_back(
        nh->create_subscription<mirte_msgs::msg::ServoPosition>(
            servo_topic, 1,
            [this, i](std::shared_ptr<mirte_msgs::msg::ServoPosition> msg) {
               //std::cout << "Servo position: " << msg->angle << std::endl;
              this->ServoPositionCallback(msg, i);
            }));
  }

  this->connectServices();

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
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
