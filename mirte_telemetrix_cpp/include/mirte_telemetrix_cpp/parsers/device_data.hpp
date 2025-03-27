#pragma once

#include <chrono>
#include <concepts>
#include <map>
#include <optional>
#include <ratio>
#include <set>
#include <string>

#include <rclcpp/parameter.hpp>

#include <mirte_telemetrix_cpp/mirte-board.hpp>

#include <mirte_telemetrix_cpp/parsers/parsers.hpp>

// TODO: Wishlist: Add unused key warnings

class DeviceData {
public:
  std::string name = "";
  std::string frame_id = "";

  using DeviceDuration = std::chrono::duration<double, std::milli>;
  DeviceDuration duration;

  DeviceData(std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
             std::string name, std::string device_type,
             std::map<std::string, rclcpp::ParameterValue> parameters,
             std::set<std::string> &unused_keys,
             std::optional<DeviceDuration> duration = {});

  DeviceData(std::string name, std::string frame_id = "",
             DeviceDuration duration = DeviceDuration(10'000.0));

  virtual bool check();

  /// @brief Get the device class of this type. So 'distance' for sonars and
  /// 'keypad' for keypads, etc.
  /// @return The device class string
  static std::string get_device_class() { return "no_type"; }
  virtual ~DeviceData(){};
};

template <class T>
std::vector<
    typename std::enable_if<std::is_base_of<DeviceData, T>::value, T>::type>
parse_all(std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board) {
  auto logger = parser->logger;
  const auto device_class = T::get_device_class();

  std::vector<T> devices;
  for (auto name : parser->get_params_keys(device_class)) {
    auto device_key = parser->build_param_name(device_class, name);
    auto parameters = parser->get_params_name(device_key);
    auto parameter_keys_vector = std::set(parser->get_params_keys(device_key));
    std::set<std::string> parameter_keys =
        std::set(parameter_keys_vector.begin(), parameter_keys_vector.end());

    auto data = T(parser, board, name, parameters, parameter_keys);

    if (parameter_keys.size() > 0) {
      RCLCPP_WARN(logger, "%s device \"%s\" has unused parameters!",
                  device_class.c_str(), name.c_str());
      for (auto &key : parameter_keys) {
        RCLCPP_WARN(logger, "Unused key: %s.%s.%s", device_class.c_str(),
                    name.c_str(), key.c_str());
      }
    }

    if (data.check()) {
      devices.push_back(data);
      RCLCPP_INFO(logger, "Added device %s.%s (kind: %s)", device_class.c_str(),
                  name.c_str(), device_class.c_str());
    } else {
      RCLCPP_ERROR(logger,
                   "%s device \"%s\" is invalid, skipping configuration.",
                   device_class.c_str(), name.c_str());
    }
  }

  return devices;
}

template <class T>
typename std::enable_if<std::is_base_of<DeviceData, T>::value,
                        std::string>::type
get_device_key(T *device) {
  return Parser::build_param_name(T::get_device_class(), device->name);
}
