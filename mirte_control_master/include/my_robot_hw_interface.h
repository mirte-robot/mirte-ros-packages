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
#include <sstream>

#include <chrono>
#include <future>
#include <mutex>
#include <thread>
#include <boost/format.hpp>

const unsigned int NUM_JOINTS = 4;
auto service_format = "/mirte/set_{}_speed";
bool bidirectional = true;

/// \brief Hardware interface for a robot
class MyRobotHWInterface : public hardware_interface::RobotHW
{
public:
  MyRobotHWInterface();

  bool write_single(int joint, int speed)
  {

    int speed_mapped =
        std::max(std::min(int(cmd[joint] / (6 * M_PI) * 100), 100), -100);
    if (speed_mapped != _last_cmd[joint])
    {
      std::cout << "write " << joint << " " << speed << std::endl;

      service_requests[joint].request.speed = speed_mapped;
      _last_cmd[joint] = speed_mapped;
      if (!service_clients[joint].call(service_requests[joint]))
      {
        this->start_reconnect();
        return false;
      }
    }
    return true;
  }
  /*
   *
   */
  void write()
  {
    if (running_)
    {
      // make sure the clients don't get overwritten while calling them
      const std::lock_guard<std::mutex> lock(this->service_clients_mutex);

      // cmd[0] = ros_control calculated speed of left motor in rad/s
      // cmd[1] = ros_control calculated speed of right motor in rad/s

      // This function converts cmd[0] to pwm and calls that service

      // NOTE: this *highly* depends on the voltage of the motors!!!!
      // For 5V power bank: 255 pwm = 90 ticks/sec -> ca 2 rot/s (4*pi)
      // For 6V power supply: 255 pwm = 120 ticks/sec -> ca 3 rot/s
      // (6*pi)
      for (int i = 0; i < NUM_JOINTS; i++)
      {
        if (!write_single(i, cmd[i]))
        {
          return;
        }
      }
      // Set the direction in so the read() can use it
      // TODO: this does not work properly, because at the end of a series
      // cmd_vel is negative, while the rotation is not
      for (int i = 0; i < NUM_JOINTS; i++)
      {
        _last_wheel_cmd_direction[i] = cmd[i] > 0.0 ? 1 : -1;
      }
    }
  }

  double meter_per_enc_tick()
  {
    return (_wheel_diameter / 2) * 2 * M_PI / 40.0; // TODO: get ticks from parameter server
  }

  /**
   * Reading encoder values and setting position and velocity of encoders
   */
  void read_single(int joint, const ros::Duration &period)
  {
    auto diff = _wheel_encoder[joint] - _last_value[joint];
    _last_value[joint] = _wheel_encoder[joint];
    double meterPerEncoderTick = meter_per_enc_tick();
    double distance;
    if (bidirectional)
    { // if encoder is counting bidirectional, then it decreases by itself, dont want to use last_wheel_cmd_direction
      distance = diff * meterPerEncoderTick * 1.0;
    }
    else
    {
      distance = diff * meterPerEncoderTick *
                 _last_wheel_cmd_direction[joint] * 1.0;
    }
    pos[joint] += distance;
    vel[joint] = distance / period.toSec(); // WHY: was this turned off?
  }

  /**
   * Reading encoder values and setting position and velocity of encoders
   */
  void read(const ros::Duration &period)
  {

    for (int i = 0; i < NUM_JOINTS; i++)
    {
      this->read_single(i, period);
    }
  }

  /*
    ros::Time get_time() {
      prev_update_time = curr_update_time;
      curr_update_time = ros::Time::now();
      return curr_update_time;
    }

    ros::Duration get_period() {
      return curr_update_time - prev_update_time;
    }
  */
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];

  bool running_;
  double _wheel_diameter;
  double _max_speed;
  double _wheel_angle[NUM_JOINTS];
  int _wheel_encoder[NUM_JOINTS];
  int _last_cmd[NUM_JOINTS];
  int _last_value[NUM_JOINTS];
  int _last_wheel_cmd_direction[NUM_JOINTS];

  ros::Time curr_update_time, prev_update_time;

  // ros::Subscriber left_wheel_encoder_sub_;
  // ros::Subscriber right_wheel_encoder_sub_;
  std::array<ros::Subscriber, NUM_JOINTS> wheel_encoder_subs_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  // ros::ServiceClient left_client;
  // ros::ServiceClient right_client;
  std::array<ros::ServiceClient, NUM_JOINTS> service_clients;
  std::array<mirte_msgs::SetMotorSpeed, NUM_JOINTS> service_requests;
  std::vector<std::string> joints;
  bool start_callback(std_srvs::Empty::Request & /*req*/,
                      std_srvs::Empty::Response & /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request & /*req*/,
                     std_srvs::Empty::Response & /*res*/)
  {
    running_ = false;
    return true;
  }

  void WheelEncoderCallback(const mirte_msgs::Encoder::ConstPtr &msg,
                            int joint)
  {
    _wheel_encoder[joint] = _wheel_encoder[joint] + msg->value;
  }

  // void rightWheelEncoderCallback(const mirte_msgs::Encoder &msg)
  // {
  //   _wheel_encoder[1] = _wheel_encoder[1] + msg.value;
  // }

  // Thread and function to restart service clients when the service server has
  // restarted
  std::future<void> reconnect_thread;
  void init_service_clients();
  void start_reconnect();
  std::mutex service_clients_mutex;
}; // class

void MyRobotHWInterface::init_service_clients()
{
  this->joints = {"left", // Edit the control.yaml when using this for the normal mirte as well
                  "right"};
  if (NUM_JOINTS == 4)
  {
    this->joints = {"left_front",
                    "left_back", // TODO: check ordering
                    "right_front", "right_back"};
  }
  for (auto joint : this->joints)
  {
    auto service = (boost::format(service_format) % joint).str();
    ROS_INFO_STREAM("Waiting for service " << service);
    ros::service::waitForService(service,
                                 -1);
  }
  {
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
    for (int i = 0; i < NUM_JOINTS; i++)
    {
      service_clients[i] =
          nh.serviceClient<mirte_msgs::SetMotorSpeed>((boost::format(service_format) % this->joints[i]).str(), true);
    }
  }
}

MyRobotHWInterface::MyRobotHWInterface()
    : running_(true), private_nh("~"),
      start_srv_(nh.advertiseService(
          "start", &MyRobotHWInterface::start_callback, this)),
      stop_srv_(nh.advertiseService("stop", &MyRobotHWInterface::stop_callback,
                                    this))
{
  private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.06);
  private_nh.param<double>("max_speed", _max_speed, 2.0);
  private_nh.param<bool>("bidirectional", bidirectional, true);

  // Initialize raw data
  std::fill_n(pos, NUM_JOINTS, 0.0);
  std::fill_n(vel, NUM_JOINTS, 0.0);
  std::fill_n(eff, NUM_JOINTS, 0.0);
  std::fill_n(cmd, NUM_JOINTS, 0.0);

  // connect and register the joint state and velocity interfaces
  for (unsigned int i = 0; i < NUM_JOINTS; ++i)
  {
    std::ostringstream os;
    os << "wheel_" << this->joints[i] << "_joint";

    hardware_interface::JointStateHandle state_handle(os.str(), &pos[i],
                                                      &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle vel_handle(
        jnt_state_interface.getHandle(os.str()), &cmd[i]);
    jnt_vel_interface.registerHandle(vel_handle);

    _wheel_encoder[i] = 0;
    _last_value[i] = 0;
    _last_wheel_cmd_direction[i] = 0;
    _last_cmd[i] = 0;
  }
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);

  // Initialize publishers and subscribers
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    auto encoder_topic = (boost::format(service_format) % this->joints[i]).str();
    wheel_encoder_subs_[i] = nh.subscribe<mirte_msgs::Encoder>(
        encoder_topic, 1,
        boost::bind(&MyRobotHWInterface::WheelEncoderCallback, this, _1, i));
  }
  this->init_service_clients();
}

void MyRobotHWInterface::start_reconnect()
{
  using namespace std::chrono_literals;

  if (this->reconnect_thread.valid())
  { // does it already exist or not?

    // Use wait_for() with zero milliseconds to check thread status.
    auto status = this->reconnect_thread.wait_for(0ms);

    if (status !=
        std::future_status::ready)
    { // Still running -> already reconnecting
      return;
    }
  }

  /* Run the reconnection on a different thread to not pause the ros-control
    loop. The launch policy std::launch::async makes sure that the task is run
    asynchronously on a new thread. */

  this->reconnect_thread =
      std::async(std::launch::async, [this]
                 { this->init_service_clients(); });
}