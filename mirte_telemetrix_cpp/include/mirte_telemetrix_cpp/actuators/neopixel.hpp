#pragma once
#include <memory>
#include <vector>

#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

#include <mirte_telemetrix_cpp/device.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/actuators/motor_data.hpp>

#include <mirte_msgs/srv/set_motor_speed.hpp>
#include <mirte_msgs/srv/set_neopixel.hpp>
#include <mirte_msgs/srv/set_neopixel_single.hpp>
#include <std_msgs/msg/int32.hpp>
class NeopixelData : public DeviceData {
public:
  NeopixelData(std::shared_ptr<Parser> parser,
               std::shared_ptr<Mirte_Board> board, std::string name,
               std::map<std::string, rclcpp::ParameterValue> parameters,
               std::set<std::string> &unused_keys);
  NeopixelData() = default;
  static std::string get_device_class() { return "neopixel"; }
  int num_leds;
  uint8_t data_pin;
  int default_r;
  int default_g;
  int default_b;
  std::string data_order;
  int r_index;
  int g_index;
  int b_index;
  ~NeopixelData() {};
  using DeviceData::check;
  bool check() { return true; }
};

class Neopixel : public TelemetrixDevice {
public:
  Neopixel(NodeData node_data, NeopixelData neopixel_data);
  //   Neopixel(NodeData node_data, std::vector<pin_t> pins, DeviceData data,
  //         bool inverted, int max_pwm);

  static std::vector<std::shared_ptr<TelemetrixDevice>>
  get_neopixels(NodeData node_data, std::shared_ptr<Parser> parser);

  // TODO: Maybe add start to TelemetrixDevice
  void start() {}
  ~Neopixel();

private:
  int num_leds;
  uint8_t data_pin;
  std::shared_ptr<NeopixelData> neopixel_data;
  std::tuple<uint8_t, uint8_t, uint8_t> order_color(const std::tuple<uint8_t, uint8_t, uint8_t> &rgb_color);

  // Service: leds/NAME/set_color
  rclcpp::Service<mirte_msgs::srv::SetNeopixel>::SharedPtr set_neopixel_service;
  // Service: leds/NAME/set_color_single
  rclcpp::Service<mirte_msgs::srv::SetNeopixelSingle>::SharedPtr
      set_neopixel_single_service;

  void set_color_service(
      const mirte_msgs::srv::SetNeopixel::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetNeopixel::Response::SharedPtr res);

  void set_color_single_service(
      const mirte_msgs::srv::SetNeopixelSingle::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetNeopixelSingle::Response::SharedPtr res);
};
