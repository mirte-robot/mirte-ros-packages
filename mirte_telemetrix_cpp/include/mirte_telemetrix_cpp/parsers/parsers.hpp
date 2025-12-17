#pragma once
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include "mirte_telemetrix_cpp/telemetrix_parameters.hpp"
#include "mirte_telemetrix_cpp/util.hpp"
using pin_t = uint8_t;

class Parser {
public:
  std::shared_ptr<rclcpp::Node> nh;
  mirte_telemetrix_cpp::Params params_object;
  std::map<std::string, rclcpp::ParameterValue> params;
  rclcpp::Logger logger;
  Parser(std::shared_ptr<rclcpp::Node> nh);
  std::map<std::string, rclcpp::ParameterValue>
  get_params_name(std::string name);
  std::set<std::string> get_params_keys(std::string name);
  static std::string
  build_param_name(std::initializer_list<const std::string> keys);
  static std::string build_param_name(std::string name, std::string key);
  int get_frequency();
  std::string get_last(std::string name);
};

std::string get_string(rclcpp::ParameterValue param);

/// Convience function to read float parameters, will still work if parameter is
/// actually an integer.
float get_float(rclcpp::ParameterValue param);

/* Insert parameter keys */

std::map<std::string, rclcpp::ParameterValue>
insert_default_param(std::map<std::string, rclcpp::ParameterValue> parameters,
                     std::string key, rclcpp::ParameterValue value);

std::set<std::string> &insert_default_param(std::set<std::string> &unused_keys,
                                            std::string key);
