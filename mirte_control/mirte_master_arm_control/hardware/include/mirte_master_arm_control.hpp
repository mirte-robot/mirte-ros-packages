// https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot
// Roughly based on:
// https://github.com/eborghi10/my_ROS_mobile_robot/blob/master/my_robot_base/include/my_robot_hw_interface.h
// https://github.com/PickNikRobotics/ros_control_boilerplate
// https://github.com/DeborggraeveR/ampru

// https://github.com/resibots/dynamixel_control_hw/blob/master/include/dynamixel_control_hw/hardware_interface.hpp
// https://github.com/FRC900/2018RobotCode/blob/master/zebROS_ws/src/ros_control_boilerplate/include/ros_control_boilerplate/frcrobot_hw_interface.h
// INTERESTING CLEAN ONE:
// https://github.com/ros-controls/ros_controllers/blob/indigo-devel/diff_drive_controller/test/diffbot.h

#pragma once
#define _USE_MATH_DEFINES

// ROS
#include <mirte_msgs/msg/servo_position.hpp>
#include <mirte_msgs/srv/set_servo_angle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
// ros_control
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// ostringstream
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>

#include <boost/format.hpp>
#include <chrono>
#include <control_toolbox/pid.hpp>
#include <future>
#include <mutex>
#include <thread>

namespace mirte_master_arm_control {

class MirteMasterArmHWInterface : public hardware_interface::SystemInterface {
public:
  MirteMasterArmHWInterface();
  RCLCPP_SHARED_PTR_DEFINITIONS(MirteMasterArmHWInterface);

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  // on_configure
  // on_cleanup
  // on_shutdown
  // on_activate
  // on_deactivate
  // on_error
  // on_init
  // export_state_interfaces
  // export_command_interfaces
  // read
  // write
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  /*
   *
   */
  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period);
  // double rad_per_enc_tick() { return 2.0 * M_PI / this->ticks; }
  /**
   * Reading encoder values and setting position and velocity of encoders
   */
  void read_single(int joint, const rclcpp::Duration &period);
  /**
   * Reading encoder values and setting position and velocity of encoders
   */
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period);
  std::shared_ptr<rclcpp::Node> nh;
  // rclcpp::Node private_nh;

private:
  int NUM_SERVOS;

  bool running_ = true;

  std::vector<double> _servo_position;
  std::vector<rclcpp::Time> _servo_position_update_time;
  std::vector<double> _last_cmd;
  std::vector<double> _last_sent_cmd;
  std::vector<int> _last_value;
  std::vector<double> position;

  rclcpp::Time curr_update_time, prev_update_time;
  rclcpp::Subscription<mirte_msgs::msg::ServoPosition>::SharedPtr subscriber_;

  std::vector<
      std::shared_ptr<rclcpp::Subscription<mirte_msgs::msg::ServoPosition>>>
      servo_pos_subs_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> start_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> stop_srv_;

  bool use_single_client = true;
  std::vector<std::shared_ptr<rclcpp::Client<mirte_msgs::srv::SetServoAngle>>>
      service_clients;
  std::vector<std::shared_ptr<mirte_msgs::srv::SetServoAngle::Request>>
      service_requests;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_arm_service;
  bool enable = true;

  void
  ServoPositionCallback(std::shared_ptr<mirte_msgs::msg::ServoPosition> msg,
                        int joint);

  // Thread and function to restart service clients when the service server has
  // restarted
  void init_service_clients();
  std::mutex service_clients_mutex;

  std::vector<std::shared_ptr<rclcpp::Client<mirte_msgs::srv::SetServoAngle>>>
      set_servo_angle_service_clients;

  rclcpp::Logger get_logger() const { return *logger_; }

  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

  // thread for ros spinning
  std::jthread ros_thread;
  void ros_spin();

  bool received_servo_data_ = false;

  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Store the command for the simulated robot
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  bool connectServices();

}; // class

} // namespace mirte_master_arm_control
