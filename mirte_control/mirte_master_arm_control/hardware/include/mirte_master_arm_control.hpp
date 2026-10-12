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
#include <mirte_msgs/srv/set_servo_angle_with_time.hpp>
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
#include <memory>
#include <mutex>
#include <thread>
// #include <share
#include <mirte_master_arm_control/mirte_master_arm_control_parameters.hpp>
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

private:
  int NUM_SERVOS;

  bool running_ = true;

  // The data we store per servo
  struct Servo_data {
    double data = NAN; // unknown position, so hw_control will also not send 0
                       // (otherwise default) back.
    bool init = false;
    bool moved = false;
    double last_move_update = -100;
    double last_request = -100;
    // timestamp for last commanded position, if it's too old and the position
    // is different, the servo might be stuck and need to send safe commands to
    // prevent damage
    rclcpp::Time last_command_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    bool sent_stuck_command = false; // only send it once to go to the current
                                     // position (cancel original command)
    bool started = false;
  };

  // Since the plugin itself is loaded once, the member variables
  // are shared between all instances of the plugin (ie.
  // the arm and the gripper both use the same variables.
  // There content is therefore stored in a map, with the
  // name as key.
  std::vector<Servo_data> servo_data;
  bool initialized;
  int init_steps;

  std::vector<
      std::shared_ptr<rclcpp::Subscription<mirte_msgs::msg::ServoPosition>>>
      servo_pos_subs_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> start_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> stop_srv_;

  bool use_single_client = true;
  std::vector<
      std::shared_ptr<rclcpp::Client<mirte_msgs::srv::SetServoAngleWithTime>>>
      service_clients;
  std::vector<std::shared_ptr<mirte_msgs::srv::SetServoAngleWithTime::Request>>
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

  std::vector<
      std::shared_ptr<rclcpp::Client<mirte_msgs::srv::SetServoAngleWithTime>>>
      set_servo_angle_service_clients;

  rclcpp::Logger get_logger() const { return *logger_; }

  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

  // thread for ros spinning
  std::jthread ros_thread;
  void ros_spin();

  double servo_update_dead_band_ = 0.05; // radians, 2.9 degrees
  double servo_moved_dead_band_ = 0.05;  // radians, 2.9 degrees

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Store the command for the simulated robot
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  bool connectServices();

  void updateParams(Params params);
  void set_servo(int i, const rclcpp::Time &time,
                 const rclcpp::Duration &period);
}; // class

} // namespace mirte_master_arm_control
