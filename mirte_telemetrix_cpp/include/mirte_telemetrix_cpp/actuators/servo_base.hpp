#pragma once
#include <memory>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/service.hpp>

#include <mirte_telemetrix_cpp/device.hpp>

#include <mirte_telemetrix_cpp/parsers/actuators/servo_data.hpp>

#include <mirte_msgs/srv/get_servo_range.hpp>
#include <mirte_msgs/srv/set_motor_speed.hpp>
#include <mirte_msgs/srv/set_servo_angle.hpp>
#include <mirte_msgs/srv/set_servo_pwm.hpp>
#include <mirte_msgs/srv/set_servo_us.hpp>

class ServoBase : public TelemetrixDevice {
public:
  ServoBase(NodeData node_data, std::vector<pin_t> pins, ServoData servo_data,
            rclcpp::CallbackGroupType callback_group_type =
                rclcpp::CallbackGroupType::Reentrant);

  virtual bool set_angle_us(uint16_t duty_cycle) = 0;
  virtual bool set_percentage(float percentage) = 0;

  ServoData data;
  void set_speed(float speed);

private:
  // Service: servo/NAME/set_angle
  rclcpp::Service<mirte_msgs::srv::SetServoAngle>::SharedPtr set_angle_service;
  // Service: servo/NAME/get_range
  rclcpp::Service<mirte_msgs::srv::GetServoRange>::SharedPtr get_range_service;
  rclcpp::Service<mirte_msgs::srv::SetServoUS>::SharedPtr set_us_service;

  void setup_servo();

  void set_angle_service_callback(
      const mirte_msgs::srv::SetServoAngle::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetServoAngle::Response::SharedPtr res);

  void get_range_service_callback(
      const mirte_msgs::srv::GetServoRange::Request::ConstSharedPtr req,
      mirte_msgs::srv::GetServoRange::Response::SharedPtr res);

  void set_us_service_callback(
      const mirte_msgs::srv::SetServoUS::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetServoUS::Response::SharedPtr res);

  void setup_motor();
  rclcpp::Service<mirte_msgs::srv::SetMotorSpeed>::SharedPtr set_speed_service;
  void set_speed_service_callback(
      const mirte_msgs::srv::SetMotorSpeed::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetMotorSpeed::Response::SharedPtr res);

  void setup_pwm(); // only really for pca servo, but can be used for regular
                    // servo as well.
  void set_pwm_service_callback(
      const mirte_msgs::srv::SetServoPWM::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetServoPWM::Response::SharedPtr res);
  rclcpp::Service<mirte_msgs::srv::SetServoPWM>::SharedPtr set_pwm_service;
};