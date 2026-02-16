#pragma once
#include <memory>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include <tmx_cpp/modules/PCA9685.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>
#include <mirte_telemetrix_cpp/modules/pca/pca_motor.hpp>
#include <mirte_telemetrix_cpp/modules/pca/pca_servo.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/pca_data.hpp>

#include <mirte_msgs/srv/set_speed_multiple.hpp>

class PCA_Module : public Mirte_module {
public:
  PCA_Module(NodeData node_data, PCAData pca_data,
             std::shared_ptr<tmx_cpp::Modules> modules);
  std::shared_ptr<tmx_cpp::PCA9685_module> pca9685;

  std::vector<std::shared_ptr<PCAMotor>> motors;
  std::vector<std::shared_ptr<PCAServo>> servos;

  static std::vector<std::shared_ptr<PCA_Module>>
  get_pca_modules(NodeData node_data, std::shared_ptr<Parser> parser,
                  std::shared_ptr<tmx_cpp::Modules> modules);
  ~PCA_Module(){};

private:
  // Service: motor/NAME/set_multiple_speeds
  rclcpp::Service<mirte_msgs::srv::SetSpeedMultiple>::SharedPtr motor_service;

  void set_multi_speed_service_callback(
      const mirte_msgs::srv::SetSpeedMultiple::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetSpeedMultiple::Response::SharedPtr res);
};
