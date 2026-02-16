#pragma once

#include <concepts>
#include <map>
#include <string>

#include <rclcpp/parameter.hpp>

#include <mirte_telemetrix_cpp/parsers/device_data.hpp>

class SensorData : public DeviceData {
public:
  SensorData(std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
             std::string name, std::string sensor_type,
             std::map<std::string, rclcpp::ParameterValue> parameters,
             std::set<std::string> &unused_keys,
             std::optional<DeviceDuration> duration = {});
  ~SensorData(){};
  // bool check();

  // / @brief Get the sensor class of this type. So 'distance' for sonars and
  // 'keypad' for keypads, etc. / @return The sensor class string static
  // std::string get_device_class() { return "no_type"; }
};
