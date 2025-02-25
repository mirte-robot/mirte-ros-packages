#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "params.hpp"
#include <algorithm>
#include <mirte_master_arm_control.hpp>

namespace mirte_master_arm_control {

 struct Servo_data {
    double data;
    bool init = false;
    bool moved = false;
    double last_move_update = -100;
    double last_request = -100;
  };
bool initialized = false;
int init_steps = 0;
std::vector<Servo_data> servo_data;

//const std::string servo_names[] = {"arm_Rot_joint", "arm_Shoulder_joint", "arm_Elbow_joint", "arm_Wrist_joint"};
// TOOD: this can be removed and use info_.joints
const std::string servo_names[] = {"gripper_joint"};

//const auto NUM_SERVOS = std::size(servo_names);
//std::array<Servo_data, NUM_SERVOS> servo_data;
// TODO: should we rename the servoH to servo_<name>?
const auto topic_format = "io/servo/hiwonder/%s/position";


hardware_interface::return_type
MirteMasterArmHWInterface::write(const rclcpp::Time &time,
                                 const rclcpp::Duration &period) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  if (initialized) {
    for (auto i = 0; i < NUM_SERVOS; i++) {
      service_requests[i]->angle = hw_commands_[i];
    }
  } else {
    if (std::all_of(std::begin(servo_data), std::end(servo_data),
                    [](Servo_data &x) { return x.init; })) {

      for (auto i = 0; i < NUM_SERVOS; i++) {
        service_requests[i]->angle = servo_data[i].data;
      }
      ++init_steps;
      for (auto i = 0; i < NUM_SERVOS; i++) {
        hw_commands_[i] = servo_data[i].data;
      }
      if (init_steps == 50) {
        initialized = true;
      }
    }
  }
  // all initialized?
  if (std::all_of(std::begin(servo_data), std::end(servo_data),
                  [](auto x) { return x.init; })) {
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
    //float last_req[NUM_SERVOS] = {-100.0};
    for (auto i = 0; i < NUM_SERVOS; i++) {

      // Only set the servo when there is a new command or the servo is moved by
      // hand or gravity.
 //     std::cout << i << "   "  << (last_req[i] != service_requests[i]->angle) << "    " << servo_data[i].moved << std::endl;
      if (servo_data[i].last_request != service_requests[i]->angle || servo_data[i].moved) {
        //last_req[i] = service_requests[i]->angle;
        servo_data[i].moved = false;
        servo_data[i].last_request = service_requests[i]->angle;

        service_requests[i]->degrees = false;
        service_clients[i]->async_send_request(service_requests[i]);
        std::cout << info_.joints[i].name << " sending radians: " << service_requests[i]->angle << std::endl;
 
     }
    }
  }





/*
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << hw_commands_[i] << " for joint '" << info_.joints[i].name << "'";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code
*/
  return hardware_interface::return_type::OK;
}

using namespace std::chrono_literals;
const auto service_format = "io/servo/hiwonder/%s/set_angle";
void MirteMasterArmHWInterface::connectServices() {

//  for (auto i = 0; i < NUM_SERVOS; i++) {
      //const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
      service_clients.clear();
      for (size_t i = 0; i < NUM_SERVOS; i++) {
        std::string servo_name = servo_names[i].substr(0, servo_names[i].size() - 6);
        auto client = nh->create_client<mirte_msgs::srv::SetServoAngle>(
            (boost::format(service_format) % servo_name)
                .str()); // TODO: add persistent connection
        while (!client->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return;
          }
          std::cout << boost::format(service_format) % servo_names[i] << std::endl;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                      "service not available, waiting again..."); 
        }
        service_clients.push_back(client);


      }
//  }

/*
  for (auto i = 0; i < NUM_SERVOS; i++) {
    auto service = (boost::format(service_format) % servo_names[i]).str();
    ros::service::waitForService(service, -1);
  }
  { // Only mutex when actually writing to class vars.
    clients.clear();
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
    for (auto i = 0; i < NUM_SERVOS; i++) {
      auto service = (boost::format(service_format) % servo_names[i]).str();

      clients.push_back(
          nh_.serviceClient<mirte_msgs::SetServoAngle>(service, true));
    }

    ROS_INFO_NAMED("rrbot_hw_interface", "Connected to the services");
  }
*/
}


void
MirteMasterArmHWInterface::ServoPositionCallback(std::shared_ptr<mirte_msgs::msg::ServoPosition> msg,
                        int joint) {

//    std::cout << "got data ............................"  << std::endl;
    auto &servo = servo_data[joint];
    servo.data = msg->angle;
    servo.init = true;
//    std::cout << servo.data << std::endl;
    // the servo should only be written to iff the servo gets a new location or
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

  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! export state interfaces done" << std::endl;

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
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! export command interfaces done" << std::endl;

  return command_interfaces;


}

// READ
hardware_interface::return_type
MirteMasterArmHWInterface::read(const rclcpp::Time &time,
                                const rclcpp::Duration &period) {

  for (std::size_t joint_id = 0; joint_id < NUM_SERVOS; ++joint_id) {
    if (servo_data[joint_id].init){
      hw_states_[joint_id] = servo_data[joint_id].data;
     // std::cout << joint_id << "     "  << servo_data[joint_id].data << std::endl;
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

  std::cout << "starting on_init" << std::endl;


  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);


  std::cout << "hirrrrr" << std::endl;
  std::stringstream ss;
  ss << "mirte_arm_control" << hw_slowdown_;

  // TODO: can we just use the node of the controller istself?
  nh = rclcpp::Node::make_shared(ss.str());
  this->ros_thread = std::jthread([this] { this->ros_spin(); });


  // Initialize raw data
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    Servo_data sd;

    servo_data.push_back(sd);
    _servo_position.push_back(0);
    _servo_position_update_time.push_back(nh->now());
    
    pos.push_back(0);
//    vel.push_back(0);
//    eff.push_back(0);
    service_requests.push_back(std::make_shared<mirte_msgs::srv::SetServoAngle::Request>());
    cmd.push_back(0);
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.RRBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
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
  std::cout << "number of servos: "  << NUM_SERVOS << std::endl;
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    std::string servo_name = servo_names[i].substr(0, servo_names[i].size() - 6);
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


  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}






void MirteMasterArmHWInterface::start_reconnect() {
}

} // namespace mirte_master_arm_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mirte_master_arm_control::MirteMasterArmHWInterface,
                       hardware_interface::SystemInterface)
