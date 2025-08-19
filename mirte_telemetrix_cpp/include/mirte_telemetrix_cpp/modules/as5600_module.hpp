#pragma once
#include <atomic>

#include <tmx_cpp/sensors/AS5600.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/as5600_data.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>

class AS5600_sensor : public Mirte_module {
public:
  AS5600_sensor(NodeData node_data, AS5600Data as5600_data,
                std::shared_ptr<tmx_cpp::Sensors> modules);

  AS5600Data data;
  std::shared_ptr<tmx_cpp::AS5600_tmx_sensor> as5600;

  virtual void update() override;

  void data_callback(tmx_cpp::AS5600_cb_t::argument_type data);


  static std::vector<std::shared_ptr<AS5600_sensor>>
  get_as5600_modules(NodeData node_data, std::shared_ptr<Parser> parser,
                  std::shared_ptr<tmx_cpp::Sensors> sensors);

private:
    std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> angle_pubs;
    std::vector<float> angles;
    std::vector<std_msgs::msg::Float32> angle_msgs;
  // // Publisher: power/NAME/used
  // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr used_pub;
  // // Publisher: power/NAME
  // rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
  // // Service: power/NAME/shutdown
  // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr shutdown_service;

  // void shutdown_robot_service_callback(
  //     const std_srvs::srv::SetBool::Request::ConstSharedPtr req,
  //     std_srvs::srv::SetBool::Response::SharedPtr res);

};