#include <exception> // for exception
#include <functional>
#include <iostream> // for operator<<, endl, basic_ostream
#include <memory>   // for make_shared, shared_ptr, __shared...
#include <stdint.h> // for uint8_t

#include <rclcpp/executors.hpp>    // for spin
#include <rclcpp/node.hpp>         // for Node
#include <rclcpp/node_options.hpp> // for NodeOptions
#include <rclcpp/utilities.hpp>    // for shutdown, init

#include "tmx_cpp/tmx.hpp" // for TMX, TMX::GET_PICO_UNIQUE_ID, TMX...

#include "mirte_telemetrix_cpp/mirte-actuators.hpp" // for Mirte_Actuators
#include "mirte_telemetrix_cpp/mirte-board.hpp"     // for Mirte_Board
#include "mirte_telemetrix_cpp/mirte-modules.hpp"
#include "mirte_telemetrix_cpp/mirte-sensors.hpp"   // for Mirte_Sensors
#include "mirte_telemetrix_cpp/parsers/parsers.hpp" // for Parser
#include "mirte_telemetrix_cpp/util.hpp"
#include <mirte_telemetrix_cpp/mirte-telemetrix.hpp>

int main(int argc, char **argv) {
  // Initialize the ROS node
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  // Spin the ROS node
  auto node = std::make_shared<TelemetrixNode>();

  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
TelemetrixNode::get_node_base_interface() const {
  return this->node_->get_node_base_interface();
}

TelemetrixNode::TelemetrixNode(const rclcpp::NodeOptions &options)
    : node_(std::make_shared<rclcpp::Node>(
          "mirte_telemetrix_node",
          rclcpp::NodeOptions(options)
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true))) {
  if (!this->start()) {
    rclcpp::shutdown();
  }
}

TelemetrixNode::~TelemetrixNode() {
  if (tmx) {
    tmx->stop();
  }
}

bool TelemetrixNode::start() {
  using namespace std::placeholders;

  auto parser = std::make_shared<Parser>(node_);

  this->board = Mirte_Board::create(parser);

  this->characteristics_service =
      node_->create_service<mirte_msgs::srv::GetBoardCharacteristics>(
          "get_board_characteristics",
          std::bind(&Mirte_Board::get_board_characteristics_service_callback,
                    this->board, _1, _2));

  auto ports = tmx_cpp::TMX::get_available_ports();
  decltype(ports) available_ports;

  for (auto &port : ports) {
    // maybe move this to a function in tmx
    std::cout << "try " << port.port_name << std::endl;
    if (!tmx_cpp::TMX::is_accepted_port(port)) {
      continue;
    }
    if (!tmx_cpp::TMX::check_port(port.port_name)) {
      continue;
    }
    std::cout << "found " << port.port_name << std::endl;
    available_ports.push_back(port);
  }
  if (available_ports.size() == 0) {
    std::cout << "No ports available" << std::endl;
    // FIXME: REMOVE DEBUG HACK
    // rclcpp::spin(s_node);
    rclcpp::shutdown();
    return false;
  }
  if (available_ports.size() > 1) {
    std::cout << "More than one port available, picking the first one"
              << std::endl;
  }

  if (false) {
    auto mcu_id = 12; // TODO: add to parsing and config
    bool found = false;
    for (auto &port : available_ports) {
      auto id = tmx_cpp::TMX::get_id(port);
      std::cout << "ID: " << std::hex << (int)id << std::endl;
      if (id == 0xff) { // default flash value is 0xff, so no id set
        auto check = tmx_cpp::TMX::set_id(port, mcu_id);
        if (!check) {
          std::cout << "Failed to set MCU ID" << std::endl;
          rclcpp::shutdown();
          return false;
        } else {
          id = mcu_id;
        }
      }
      if (id == mcu_id) {
        available_ports = {port};
        found = true;
        break;
      }
    }
    if (!found) {
      std::cout << "No port with MCU ID " << mcu_id << " found" << std::endl;
      rclcpp::shutdown();
      return false;
    }
  }

  tmx = std::make_shared<tmx_cpp::TMX>([&]() { rclcpp::shutdown(); },
                                       available_ports[0].port_name);
  tmx->sendMessage(tmx_cpp::MESSAGE_TYPE::GET_PICO_UNIQUE_ID, {});
  tmx->setScanDelay(1000 / parser->get_frequency());

  NodeData node_data{node_, tmx, board};

  std::cout << "Start adding" << std::endl;

  this->actuators = std::make_shared<Mirte_Actuators>(node_data, parser);
  this->monitor = std::make_shared<Mirte_Sensors>(node_data, parser);
  this->modules = std::make_shared<Mirte_modules>(node_data, parser);
  std::cout << "Done adding" << std::endl;
  return true;
}
