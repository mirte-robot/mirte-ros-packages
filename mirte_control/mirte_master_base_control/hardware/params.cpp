#include <params.hpp>

void parse_params(std::string param_file, std::shared_ptr<rclcpp::Node> nh) {
  // read the yaml file
  YAML::Node config = YAML::LoadFile(param_file);
  // get the parameters
  nh->declare_parameter("wheel_diameter",
                        config["wheel_diameter"].as<double>());
  nh->declare_parameter("max_speed", config["max_speed"].as<double>());

  // check if config has the key
  if (config["single_client"]) {
    nh->declare_parameter<bool>("single_client",
                               config["single_client"].as<bool>());
  } else {
    nh->declare_parameter<bool>("single_client", true);
  }
}