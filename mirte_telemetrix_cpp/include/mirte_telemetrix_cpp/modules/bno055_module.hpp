#pragma once
#include <array>
#include <memory>
#include <mutex>

#include <tmx_cpp/sensors/BNO055.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/bno055_data.hpp>

#include <mirte_msgs/srv/get_imu.hpp>
#include <sensor_msgs/msg/imu.hpp>

class BNO055_sensor : public Mirte_module {
public:
  BNO055_sensor(NodeData node_data, BNO055Data imu_data,
                std::shared_ptr<tmx_cpp::Sensors> sensors);

  BNO055Data data;
  std::shared_ptr<tmx_cpp::BNO055_module> BNO055;

  virtual void update() override;
  void data_callback(const tmx_cpp::BNO055_MOD_data &data);
  // std::array<float, 3> acceleration,
  //                  std::array<float, 3> gyro,
  //                  std::array<float, 3> magnetic_field,
  //                  std::array<float, 4> quaternion);

  static std::vector<std::shared_ptr<BNO055_sensor>>
  get_bno_modules(NodeData node_data, std::shared_ptr<Parser> parser,
                  std::shared_ptr<tmx_cpp::Sensors> sensors);

private:
  std::mutex msg_mutex;
  sensor_msgs::msg::Imu msg;

  // Publisher: imu/NAME/data
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  // Service: imu/NAME/get_data
  rclcpp::Service<mirte_msgs::srv::GetImu>::SharedPtr imu_service;

  void get_imu_service_callback(
      const mirte_msgs::srv::GetImu::Request::ConstSharedPtr req,
      mirte_msgs::srv::GetImu::Response::SharedPtr res);
};
