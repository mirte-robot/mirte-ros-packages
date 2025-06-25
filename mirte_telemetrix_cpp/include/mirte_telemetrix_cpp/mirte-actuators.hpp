#pragma once
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/device.hpp>
#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_msgs/srv/set_digital_pin_value.hpp>
#include <mirte_msgs/srv/set_pwm_pin_value.hpp>

class Mirte_Actuators {
public:
  Mirte_Actuators(NodeData data, std::shared_ptr<Parser> parser);

  std::shared_ptr<tmx_cpp::TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<std::shared_ptr<TelemetrixDevice>> actuators;

private:
  // Service: set_digital_pin_value
  rclcpp::Service<mirte_msgs::srv::SetDigitalPinValue>::SharedPtr
      digital_pin_service;
  // Service: set_pwm_pin_value
  rclcpp::Service<mirte_msgs::srv::SetPWMPinValue>::SharedPtr pwm_pin_service;

  void digital_pin_service_callback(
      const mirte_msgs::srv::SetDigitalPinValue::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetDigitalPinValue::Response::SharedPtr res);

  void pwm_pin_service_callback(
      const mirte_msgs::srv::SetPWMPinValue::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetPWMPinValue::Response::SharedPtr res);
};
