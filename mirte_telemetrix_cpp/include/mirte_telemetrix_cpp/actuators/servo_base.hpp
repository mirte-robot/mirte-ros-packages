#pragma once
#include <memory>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/service.hpp>

#include <mirte_telemetrix_cpp/device.hpp>

#include <mirte_telemetrix_cpp/parsers/actuators/servo_data.hpp>

#include <mirte_msgs/srv/get_servo_range.hpp>
#include <mirte_msgs/srv/set_servo_angle.hpp>

class ServoBase : public TelemetrixDevice {
public:
  ServoBase(NodeData node_data, std::vector<pin_t> pins, ServoData servo_data,
            rclcpp::CallbackGroupType callback_group_type =
                rclcpp::CallbackGroupType::Reentrant);

  virtual bool set_angle_us(uint16_t duty_cycle) = 0;

  // TODO: Still storing this specific data, I'm unsure if that is good or bad.
  ServoData data;

private:
  // Service: servo/NAME/set_angle
  rclcpp::Service<mirte_msgs::srv::SetServoAngle>::SharedPtr set_angle_service;
  // Service: servo/NAME/get_range
  rclcpp::Service<mirte_msgs::srv::GetServoRange>::SharedPtr get_range_service;

  void set_angle_service_callback(
      const mirte_msgs::srv::SetServoAngle::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetServoAngle::Response::SharedPtr res);

  void get_range_service_callback(
      const mirte_msgs::srv::GetServoRange::Request::ConstSharedPtr req,
      mirte_msgs::srv::GetServoRange::Response::SharedPtr res);
};