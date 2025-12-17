#include "rclcpp/rclcpp.hpp"
#include <mirte_telemetrix_cpp/telemetrix_parameters.hpp> // auto-generated parameter file from telemetrix_parameters.yaml
int main(int argc, char **argv) {
  // Your code here
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "mirte_telemetrix_test_node",
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(false)
          .automatically_declare_parameters_from_overrides(true));
  // auto parser = std::make_shared<mirte_telemetrix_cpp::Parser>(node);
  try {
    auto param_listener =
        std::make_shared<mirte_telemetrix_cpp::ParamListener>(node);
    auto params = param_listener->get_params();
    std::cout << params.encoder.encoders_map["left"].pins.A << std::endl;
    std::cout << params.encoder.encoders_map["left"].pins.B << std::endl;
  } catch (const rclcpp::exceptions::InvalidParameterValueException &e) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize ParamListener: %s",
                 e.what());
    return 1;
  }
  // auto param_listener =
  // std::make_shared<mirte_telemetrix_cpp::ParamListener>(node); auto params =
  // param_listener->get_params();

  return 0;
}
