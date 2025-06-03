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
#include <mirte_msgs/Encoder.h>
#include <mirte_msgs/SetMotorSpeed.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// ostringstream
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>

#include <boost/format.hpp>
#include <chrono>
#include <control_toolbox/pid.h>
#include <future>
#include <mutex>
#include <thread>
// const unsigned int NUM_JOINTS = 4;
const auto service_format = "mirte/set_%s_speed";
const auto encoder_format = "mirte/encoder/%s";
const auto max_speed = 100; // Quick fix hopefully for power dip.

bool equal_gains(control_toolbox::Pid::Gains lhs,
                 control_toolbox::Pid::Gains rhs) {
  return lhs.p_gain_ == rhs.p_gain_ && lhs.i_gain_ == rhs.i_gain_ &&
         lhs.d_gain_ == lhs.d_gain_;
}

/// \brief Hardware interface for a robot
class MyRobotHWInterface : public hardware_interface::RobotHW {
public:
  MyRobotHWInterface();

  double calc_speed_pid(int joint, double target, const ros::Duration &period) {

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

    auto diff_enc_time_upd =
        ros::Time::now() - _wheel_encoder_update_time[joint];
    if (diff_enc_time_upd > ros::Duration(1, 0) &&
        _last_cmd[joint] >
            30) { // if the motors don't move, no need to fall back yet
      ROS_WARN_STREAM(
          "Encoder "
          << joint
          << " not reporting data, falling back to mapping calculation");
      return calc_speed_map(joint, target, period);
    }
    auto curr_speed = vel[joint];
    auto err = target - curr_speed;
    auto pid_cmd = pid->computeCommand(err, period);
    return pid_cmd + _last_cmd[joint];
  }

  double calc_speed_map(int joint, double target, const ros::Duration &period) {
    return std::max(std::min(int(target / (6.0 * M_PI) * 100), 100), -100);
  }

  bool write_single(int joint, double speed, const ros::Duration &period) {
    double speed_mapped;
    if (this->enablePID) {
      speed_mapped = this->calc_speed_pid(joint, speed, period);
    } else {
      speed_mapped = this->calc_speed_map(joint, speed, period);
    }
    speed_mapped = std::clamp<double>(speed_mapped, -max_speed, max_speed);
    auto diff = std::abs(speed_mapped - _last_sent_cmd[joint]);
    _last_cmd[joint] = speed_mapped;
    if (diff > 1.0) {
      _last_sent_cmd[joint] = speed_mapped;
      service_requests[joint].request.speed = (int)speed_mapped;
      if (!service_clients[joint].call(service_requests[joint])) {
        this->start_reconnect();
        return false;
      }
    }
    return true;
  }
  /*
   *
   */
  void write(const ros::Duration &period) {
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
      for (size_t i = 0; i < NUM_JOINTS; i++) {
        if (!write_single(i, cmd[i], period)) {
          return;
        }
      }
      // Set the direction in so the read() can use it
      // TODO: this does not work properly, because at the end of a series
      // cmd_vel is negative, while the rotation is not
      for (size_t i = 0; i < NUM_JOINTS; i++) {
        _last_wheel_cmd_direction[i] = cmd[i] > 0.0 ? 1 : -1;
      }
    }
  }

  double rad_per_enc_tick() { return 2.0 * M_PI / this->ticks; }
  /**
   * Reading encoder values and setting position and velocity of encoders
   */
  void read_single(int joint, const ros::Duration &period) {
    auto diff_ticks = _wheel_encoder[joint] - _last_value[joint];
    _last_value[joint] = _wheel_encoder[joint];
    double radPerEncoderTick = rad_per_enc_tick();
    double distance_rad;
    if (bidirectional) { // if encoder is counting bidirectional, then it
                         // decreases by itself, dont want to use
                         // last_wheel_cmd_direction
      distance_rad = diff_ticks * radPerEncoderTick * 1.0;
    } else {
      distance_rad = diff_ticks * radPerEncoderTick *
                     _last_wheel_cmd_direction[joint] * 1.0;
    }
    pos[joint] += distance_rad;
    vel[joint] = distance_rad / period.toSec(); // WHY: was this turned off?
  }

  /**
   * Reading encoder values and setting position and velocity of encoders
   */
  void read(const ros::Duration &period) {

    for (size_t i = 0; i < NUM_JOINTS; i++) {
      this->read_single(i, period);
    }
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  std::vector<double> cmd;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;

  bool running_ = true;
  double _wheel_diameter;
  double _max_speed;
  double ticks = 40.0;

  std::vector<int> _wheel_encoder;
  std::vector<ros::Time> _wheel_encoder_update_time;
  std::vector<double> _last_cmd;
  std::vector<double> _last_sent_cmd;
  std::vector<int> _last_value;
  std::vector<int> _last_wheel_cmd_direction;

  ros::Time curr_update_time, prev_update_time;

  std::vector<ros::Subscriber> wheel_encoder_subs_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  std::vector<ros::ServiceClient> service_clients;
  std::vector<mirte_msgs::SetMotorSpeed> service_requests;
  std::vector<std::string> joints;
  bool enablePID;
  std::vector<std::shared_ptr<control_toolbox::Pid>> pids;
  std::shared_ptr<control_toolbox::Pid>
      reconfig_pid; // one dummy pid to use for the dynamic reconfigure

  bool start_callback(std_srvs::Empty::Request & /*req*/,
                      std_srvs::Empty::Response & /*res*/) {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request & /*req*/,
                     std_srvs::Empty::Response & /*res*/) {
    running_ = false;
    return true;
  }

  void WheelEncoderCallback(const mirte_msgs::Encoder::ConstPtr &msg,
                            int joint) {
    if (msg->value < 0) {
      bidirectional = true;
    }
    _wheel_encoder[joint] = msg->value;
    _wheel_encoder_update_time[joint] = msg->header.stamp;
  }

  // Thread and function to restart service clients when the service server has
  // restarted
  std::future<void> reconnect_thread;
  void init_service_clients();
  void start_reconnect();
  std::mutex service_clients_mutex;

  bool bidirectional = false; // assume it is one direction, when receiving any
                              // negative value, it will be set to true
  unsigned int NUM_JOINTS = 2;
}; // class

void MyRobotHWInterface::init_service_clients() {
  for (auto joint : this->joints) {
    auto service = (boost::format(service_format) % joint).str();
    ROS_INFO_STREAM("Waiting for service " << service);
    ros::service::waitForService(service, -1);
  }
  {
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
    service_clients.clear();
    service_requests.clear();
    for (size_t i = 0; i < NUM_JOINTS; i++) {
      service_clients.push_back(nh.serviceClient<mirte_msgs::SetMotorSpeed>(
          (boost::format(service_format) % this->joints[i]).str(), true));
      service_requests.push_back(mirte_msgs::SetMotorSpeed());
    }
  }
}

unsigned int detect_joints(ros::NodeHandle &nh) {
  std::string type;
  nh.param<std::string>("mobile_base_controller/type", type, "");
  if (type.rfind("mecanum", 0) == 0) { // starts with mecanum
    return 4;
  } else if (type.rfind("diff", 0) == 0) { // starts with diff
    return 2;
  } else {
    ROS_ERROR_STREAM("Unknown type: " << type);
    return 0;
  }
}

MyRobotHWInterface::MyRobotHWInterface()
    : private_nh("~"), running_(true),
      start_srv_(nh.advertiseService(
          "start", &MyRobotHWInterface::start_callback, this)),
      stop_srv_(nh.advertiseService("stop", &MyRobotHWInterface::stop_callback,
                                    this)) {
  nh.param<double>("mobile_base_controller/wheel_radius", _wheel_diameter,
                   0.06);
  _wheel_diameter *= 2; // convert from radius to diameter
  nh.param<double>("mobile_base_controller/max_speed", _max_speed,
                   2.0); // TODO: unused
  nh.param<double>("mobile_base_controller/ticks", ticks, 40.0);
  this->NUM_JOINTS = detect_joints(nh);
  if (this->NUM_JOINTS > 2) {
    this->bidirectional = true;
  }
  // Initialize raw data
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    _wheel_encoder.push_back(0);
    _wheel_encoder_update_time.push_back(ros::Time::now());
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
    this->joints = {"left_front",
                    "left_rear", // TODO: check ordering
                    "right_front", "right_rear"};
  }
  std::cout << "Initializing MyRobotHWInterface with " << NUM_JOINTS
            << " joints" << std::endl;

  // connect and register the joint state and velocity interfaces
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    std::string joint =
        (boost::format("wheel_%s_joint") % this->joints[i]).str();
    hardware_interface::JointStateHandle state_handle(joint, &pos[i], &vel[i],
                                                      &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle vel_handle(
        jnt_state_interface.getHandle(joint), &cmd[i]);
    jnt_vel_interface.registerHandle(vel_handle);
  }
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);

  nh.param<bool>("mobile_base_controller/enable_pid", enablePID, false);
  if (enablePID) {
    // dummy pid for dynamic reconfigure.
    this->reconfig_pid = std::make_shared<control_toolbox::Pid>(1, 0, 0);
    this->reconfig_pid->initParam("mobile_base_controller/", false);
    auto gains = this->reconfig_pid->getGains();
    for (auto i = 0; i < NUM_JOINTS; i++) {
      auto pid = std::make_shared<control_toolbox::Pid>(1, 1, 1);
      pid->setGains(gains);
      this->pids.push_back(pid);
    }
  }

  // Initialize publishers and subscribers
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    auto encoder_topic =
        (boost::format(encoder_format) % this->joints[i]).str();
    wheel_encoder_subs_.push_back(nh.subscribe<mirte_msgs::Encoder>(
        encoder_topic, 1,
        boost::bind(&MyRobotHWInterface::WheelEncoderCallback, this, _1, i)));
  }
  assert(joints.size() == NUM_JOINTS);
  this->init_service_clients();
  assert(service_requests.size() == NUM_JOINTS);

  assert(service_clients.size() == NUM_JOINTS);
}

void MyRobotHWInterface ::start_reconnect() {
  using namespace std::chrono_literals;

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