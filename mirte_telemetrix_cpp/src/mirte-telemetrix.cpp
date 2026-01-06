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
#include <mirte_telemetrix_cpp/telemetrix_parameters.hpp> // auto-generated parameter file from telemetrix_parameters.yaml
NodeData node_data;
int main(int argc, char **argv) {
  // Initialize the ROS node
  rclcpp::init(argc, argv);
  auto s = "main_thread";

  pthread_setname_np(pthread_self(), s);
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
  std::cout << "timers: " << node_data.timers.size() << std::endl;
  for (const auto &timer : node_data.timers) {
    std::cout << "Timer: " << timer.first.count() << " ms, "
              << " ms, " << timer.second.second.size() << " callbacks"
              << std::endl;
    timer.second.first->execute_callback();
    timer.second.first->reset();
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

  std::shared_ptr<tmx_cpp::TMX> tmx;
  // rclcpp::spin(node_);
  if (parser->params_object.device.mirte.port != "") {
    auto port = parser->params_object.device.mirte.port;
    std::cout << "Using port: " << port << std::endl;
    if (!tmx_cpp::TMX::check_port(port)) {
      std::cout << "Port " << port << " is not available" << std::endl;
      rclcpp::shutdown();
      return false;
    }
    tmx = std::make_shared<tmx_cpp::TMX>([&]() { rclcpp::shutdown(); }, port);
  } else {
    std::cout << "No port specified, searching for available ports"
              << std::endl;

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
    std::cout << "Using port: " << available_ports[0].port_name << std::endl;
  }
  tmx->sendMessage(tmx_cpp::MESSAGE_TYPE::GET_PICO_UNIQUE_ID, {});
  tmx->setScanDelay(1000 / parser->get_frequency());

  this->board = Mirte_Board::create(parser, tmx);

  this->characteristics_service =
      node_->create_service<mirte_msgs::srv::GetBoardCharacteristics>(
          "get_board_characteristics",
          std::bind(&Mirte_Board::get_board_characteristics_service_callback,
                    this->board, _1, _2));

  node_data = NodeData{node_, tmx, board};
  node_data.add_timer = [](std::chrono::duration<double, std::milli> duration,
                           std::function<void()> callback) mutable {
    std::cout << "Adding cb for duration: " << duration.count() << " ms"
              << std::endl;
    for (auto &timer : node_data.timers) {
      if (timer.first == duration) {
        timer.second.second.push_back(callback);
        return;
      }
    }
    using namespace std::chrono_literals;

    auto timer = node_data.nh->create_wall_timer(duration, [duration]() {
      // std::cout << "Timer expired: " << duration.count() << " ms" <<
      // std::endl;
      for (auto &timer : node_data.timers) {
        if (timer.first == duration) {
          for (auto &cb : timer.second.second) {
            cb();
          }
        }
      }
      // std::cout << "Timer expired: " << duration.count() << " ms done" <<
      // std::endl;
    });

    node_data.timers.push_back(
        {duration,
         {timer,
          {callback}}}); // add a new timer with the duration and callback
    std::cout << "Adding timer for duration: " << duration.count() << " ms"
              << std::endl;
  };
  std::cout << "Start adding" << std::endl;

  this->actuators = std::make_shared<Mirte_Actuators>(node_data, parser);
  this->monitor = std::make_shared<Mirte_Sensors>(node_data, parser);
  this->modules = std::make_shared<Mirte_modules>(node_data, parser);
  this->actuators->start();
  this->monitor->start();
  this->modules->start();
  std::cout << "Done adding" << std::endl;
  return true;
}
