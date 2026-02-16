#pragma once
#include <memory>
#include <tuple>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>
#include <mirte_telemetrix_cpp/modules/hiwonder/hiwonder_servo.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/hiwonder_data.hpp>

#include <std_srvs/srv/set_bool.hpp>

class HiWonderBus_module : public Mirte_module {
public:
  HiWonderBus_module(NodeData node_data, HiWonderBusData bus_data,
                     std::shared_ptr<tmx_cpp::Modules> modules);

  HiWonderBusData data;
  std::shared_ptr<tmx_cpp::HiwonderServo_module> bus;
  std::vector<std::shared_ptr<Hiwonder_servo>> servos;

  static std::vector<std::shared_ptr<HiWonderBus_module>>
  get_hiwonder_modules(NodeData node_data, std::shared_ptr<Parser> parser,
                       std::shared_ptr<tmx_cpp::Modules> modules);

private:
  void position_cb(
      std::vector<
          std::tuple<uint8_t, tmx_cpp::HiwonderServo_module::Servo_pos>>);

  // Service: servo/GROUP/enable_all_servos
  // ROS Enable Service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_all_servos_service;

  void enable_service_callback(
      const std_srvs::srv::SetBool::Request::ConstSharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res);
};
