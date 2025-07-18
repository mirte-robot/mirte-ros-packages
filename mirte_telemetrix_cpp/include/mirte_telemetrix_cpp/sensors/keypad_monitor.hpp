#pragma once
#include <atomic>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>

#include <mirte_telemetrix_cpp/parsers/sensors/keypad_data.hpp>

#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

#include <mirte_msgs/msg/intensity.hpp>
#include <mirte_msgs/msg/keypad.hpp>
#include <mirte_msgs/srv/get_keypad.hpp>
class KeypadMonitor : public Mirte_Sensor {
public:
  KeypadMonitor(NodeData node_data, KeypadData keypad_data);
  virtual void update() override;

  KeypadData keypad_data;

  enum Key { NONE, LEFT, UP, DOWN, RIGHT, ENTER };

  static std::string key_string(Key key);

  static std::vector<std::shared_ptr<KeypadMonitor>>
  get_keypad_monitors(NodeData node_data, std::shared_ptr<Parser> parser);

private:
  void callback(uint16_t value);
  std::atomic<Key> last_key;
  std::atomic<double> last_debounce_time = nh->now().seconds();
  std::atomic<Key> last_debounced_key;

  // Publisher: keypad/NAME
  rclcpp::Publisher<mirte_msgs::msg::Keypad>::SharedPtr keypad_pub;
  // Publisher: keypad/NAME/pressed
  // Only publishes when a key is pressed initially
  rclcpp::Publisher<mirte_msgs::msg::Keypad>::SharedPtr keypad_pressed_pub;
  // Service: keypad/NAME/get_key
  rclcpp::Service<mirte_msgs::srv::GetKeypad>::SharedPtr keypad_service;

  // Publisher: keypad/NAME/analog
  rclcpp::Publisher<mirte_msgs::msg::Intensity>::SharedPtr keypad_analog_pub;

  void keypad_service_callback(
      const mirte_msgs::srv::GetKeypad::Request::ConstSharedPtr req,
      mirte_msgs::srv::GetKeypad::Response::SharedPtr res);
};
