#pragma once
#include <memory>
#include <vector>

#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

#include <mirte_telemetrix_cpp/device.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/actuators/motor_data.hpp>

#include <mirte_msgs/srv/set_motor_speed.hpp>
#include <std_msgs/msg/int32.hpp>

class Motor : public TelemetrixDevice {
public:
  Motor(NodeData node_data, std::vector<pin_t> pins, MotorData motor_data);
  Motor(NodeData node_data, std::vector<pin_t> pins, DeviceData data,
        bool inverted, int max_pwm);

  int last_speed = 0;
  bool inverted;
  int max_pwm;

  static std::vector<std::shared_ptr<TelemetrixDevice>>
  get_motors(NodeData node_data, std::shared_ptr<Parser> parser);

  virtual void set_speed(int speed) = 0;

  // TODO: Maybe add start to TelemetrixDevice
  void start() { this->set_speed(0); }

private:
  // Subscriber: motor/NAME/speed
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr speed_subscription;
  // Service: motor/NAME/set_speed
  rclcpp::Service<mirte_msgs::srv::SetMotorSpeed>::SharedPtr set_speed_service;

  void speed_subscription_callback(const std_msgs::msg::Int32 &msg);

  void set_speed_service_callback(
      const mirte_msgs::srv::SetMotorSpeed::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetMotorSpeed::Response::SharedPtr res);
};
