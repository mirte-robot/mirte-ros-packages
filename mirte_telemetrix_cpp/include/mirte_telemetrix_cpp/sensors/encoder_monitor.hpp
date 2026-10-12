#pragma once
#include <atomic>

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/parsers/sensors/encoder_data.hpp>

#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

#include <mirte_msgs/msg/encoder.hpp>
#include <mirte_msgs/msg/encoder_rad.hpp>
#include <mirte_msgs/srv/get_encoder.hpp>

class EncoderMonitor : public Mirte_Sensor {
public:
  EncoderMonitor(NodeData node_data, EncoderData encoder_data);

  virtual void update() override;

  EncoderData encoder_data;

  static std::vector<std::shared_ptr<EncoderMonitor>>
  get_encoder_monitors(NodeData node_data, std::shared_ptr<Parser> parser);
  ~EncoderMonitor() {};

  void data_callback(int16_t value);
  std_msgs::msg::Header create_header();

private:
  std::atomic<int32_t> value = 0;

  // Publisher: encoder/NAME
  rclcpp::Publisher<mirte_msgs::msg::Encoder>::SharedPtr encoder_pub;
  // Publisher: encoder/NAME/rad
  rclcpp::Publisher<mirte_msgs::msg::EncoderRad>::SharedPtr encoder_rad_pub;
  mirte_msgs::msg::Encoder msg;
  mirte_msgs::msg::EncoderRad rad_msg;

  // Service: encoder/NAME/get_encoder
  rclcpp::Service<mirte_msgs::srv::GetEncoder>::SharedPtr encoder_service;

  void service_callback(
      const mirte_msgs::srv::GetEncoder::Request::ConstSharedPtr req,
      mirte_msgs::srv::GetEncoder::Response::SharedPtr res);
  rclcpp::Time last_update_time;
};