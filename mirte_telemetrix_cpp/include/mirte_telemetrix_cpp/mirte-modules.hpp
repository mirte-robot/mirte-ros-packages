#pragma once
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/modules/base_module.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

class Mirte_modules {
public:
  Mirte_modules(NodeData node_data, std::shared_ptr<Parser> parser);
  std::shared_ptr<tmx_cpp::TMX> tmx;
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<Mirte_Board> board;
  std::vector<std::shared_ptr<Mirte_module>> modules;
  std::shared_ptr<tmx_cpp::Modules> module_sys;
  std::shared_ptr<tmx_cpp::Sensors> sensor_sys;
};
