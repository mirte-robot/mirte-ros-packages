#pragma once

#include <array>
#include <memory>
#include <mutex>

#include <tmx_cpp/sensors/ADXL345.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/adxl345_data.hpp>

#include <mirte_msgs/srv/get_imu.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ADXL345_sensor : public Mirte_module {
public:
  ADXL345_sensor(NodeData node_data, ADXL345Data imu_data,
                 std::shared_ptr<tmx_cpp::Sensors> sensors);

  ADXL345Data data;
  std::shared_ptr<tmx_cpp::ADXL345_module> adxl345;

  virtual void update() override;
  void data_callback(std::array<float, 3> acceleration);

  static std::vector<std::shared_ptr<ADXL345_sensor>>
  get_adxl_modules(NodeData node_data, std::shared_ptr<Parser> parser,
                   std::shared_ptr<tmx_cpp::Sensors> sensors);

private:
  std::mutex msg_mutex;
  sensor_msgs::msg::Imu msg;

  // Publisher: imu/NAME/data_raw
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  // Service: imu/NAME/get_data_raw
  rclcpp::Service<mirte_msgs::srv::GetImu>::SharedPtr imu_service;

  void get_imu_service_callback(
      const mirte_msgs::srv::GetImu::Request::ConstSharedPtr req,
      mirte_msgs::srv::GetImu::Response::SharedPtr res);
};
