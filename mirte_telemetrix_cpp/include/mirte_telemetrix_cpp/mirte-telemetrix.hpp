#pragma once
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/service.hpp>

#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/mirte-actuators.hpp>
#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/mirte-modules.hpp>
#include <mirte_telemetrix_cpp/mirte-sensors.hpp>

#include <mirte_msgs/srv/get_board_characteristics.hpp>

class TelemetrixNode {
private:
  /* data */
  std::shared_ptr<rclcpp::Node> node_;

public:
  TelemetrixNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

  ~TelemetrixNode();
  bool start();
  std::shared_ptr<Mirte_Board> board;
  std::shared_ptr<tmx_cpp::TMX> tmx;

  std::shared_ptr<Mirte_Sensors> monitor;
  std::shared_ptr<Mirte_Actuators> actuators;
  std::shared_ptr<Mirte_modules> modules;

  // Service: get_board_characteristics
  rclcpp::Service<mirte_msgs::srv::GetBoardCharacteristics>::SharedPtr
      characteristics_service;
};
