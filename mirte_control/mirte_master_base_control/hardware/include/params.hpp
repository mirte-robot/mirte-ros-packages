#pragma once

#include "yaml-cpp/yaml.h"
#include <rclcpp/rclcpp.hpp>
#include <string>

void parse_params(std::string param_file, std::shared_ptr<rclcpp::Node> nh);