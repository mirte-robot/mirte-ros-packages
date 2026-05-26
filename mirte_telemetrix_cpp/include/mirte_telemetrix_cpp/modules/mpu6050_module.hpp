#pragma once
#include <array>
#include <memory>
#include <mutex>

#include <tmx_cpp/sensors/MPU6050.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/mpu6050_data.hpp>

#include <mirte_msgs/srv/get_imu.hpp>
#include <sensor_msgs/msg/imu.hpp>

class MPU6050_sensor : public Mirte_module {
public:
  MPU6050_sensor(NodeData node_data, MPU6050Data imu_data,
                 std::shared_ptr<tmx_cpp::Sensors> sensors);

  MPU6050Data data;
  std::shared_ptr<tmx_cpp::MPU6050_module> mpu6050;

  virtual void update() override;
  void data_callback(std::array<float, 3> acceleration,
                     std::array<float, 3> gyro, float temperature);

  static std::vector<std::shared_ptr<MPU6050_sensor>>
  get_mpu_modules(NodeData node_data, std::shared_ptr<Parser> parser,
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
