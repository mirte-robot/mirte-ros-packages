#pragma once
#include <atomic>
#include <memory>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

#include <tmx_cpp/modules/HiwonderServo.hpp>
#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/hiwonder/hiwonder_servo_data.hpp>

#include <mirte_msgs/msg/servo_position.hpp>
#include <mirte_msgs/srv/get_servo_offset.hpp>
#include <mirte_msgs/srv/get_servo_range.hpp>
#include <mirte_msgs/srv/set_motor_speed.hpp>
#include <mirte_msgs/srv/set_servo_angle.hpp>
#include <mirte_msgs/srv/set_servo_angle_with_speed.hpp>
#include <mirte_msgs/srv/set_servo_offset.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>

class Hiwonder_servo {
public:
  Hiwonder_servo(NodeData node_data,
                 std::shared_ptr<HiWonderServoData> servo_data,
                 std::shared_ptr<tmx_cpp::HiwonderServo_module> bus,
                 std::string servo_group, DeviceData::DeviceDuration duration,
                 rclcpp::CallbackGroup::SharedPtr callback_group);

  std::shared_ptr<HiWonderServoData> servo_data;
  std::shared_ptr<tmx_cpp::HiwonderServo_module> bus_mod;

  // callbacks from the pico
  void position_cb(tmx_cpp::HiwonderServo_module::Servo_pos &pos);

  // angle calcs
  uint16_t calc_angle_out(float angle_in);
  float calc_angle_in(uint16_t angle_out);

  std_msgs::msg::Header get_header();

  void update();

private:
  std::shared_ptr<rclcpp::Node> nh;

  rclcpp::TimerBase::SharedPtr servo_timer;

  std::atomic<float> last_angle = 0.0;
  std::atomic<uint16_t> last_raw = 0;

  /*  ROS: */
  // Publisher: servo/GROUP/NAME/position
  rclcpp::Publisher<mirte_msgs::msg::ServoPosition>::SharedPtr position_pub;
  // Service: servo/GROUP/NAME/set_enable
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service;
  // Service: servo/GROUP/NAME/set_angle
  rclcpp::Service<mirte_msgs::srv::SetServoAngle>::SharedPtr angle_service;
  // Service: servo/GROUP/NAME/set_angle_with_speed
  rclcpp::Service<mirte_msgs::srv::SetServoAngleWithSpeed>::SharedPtr
      angle_speed_service;
  // Service: servo/GROUP/NAME/get_range
  rclcpp::Service<mirte_msgs::srv::GetServoRange>::SharedPtr range_service;
  // Service: servo/GROUP/NAME/set_motor_speed
  rclcpp::Service<mirte_msgs::srv::SetMotorSpeed>::SharedPtr
      motor_speed_service;
  rclcpp::Service<mirte_msgs::srv::GetServoOffset>::SharedPtr offset_service;
  rclcpp::Service<mirte_msgs::srv::SetServoOffset>::SharedPtr
      set_offset_service;

  void enable_service_callback(
      const std_srvs::srv::SetBool::Request::ConstSharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res);

  void set_angle_service_callback(
      const mirte_msgs::srv::SetServoAngle::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetServoAngle::Response::SharedPtr res);

  void set_angle_with_speed_service_callback(
      const mirte_msgs::srv::SetServoAngleWithSpeed::Request::ConstSharedPtr
          req,
      mirte_msgs::srv::SetServoAngleWithSpeed::Response::SharedPtr res);

  void get_range_service_callback(
      const mirte_msgs::srv::GetServoRange::Request::ConstSharedPtr req,
      mirte_msgs::srv::GetServoRange::Response::SharedPtr res);

  void set_motor_speed_service_callback(
      const mirte_msgs::srv::SetMotorSpeed::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetMotorSpeed::Response::SharedPtr res);

  void get_offset_service_callback(
      const mirte_msgs::srv::GetServoOffset::Request::ConstSharedPtr req,
      mirte_msgs::srv::GetServoOffset::Response::SharedPtr res);
  void set_offset_service_callback(
      const mirte_msgs::srv::SetServoOffset::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetServoOffset::Response::SharedPtr res);
};
