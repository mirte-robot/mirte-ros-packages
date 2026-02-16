#pragma once

#include <mirte_telemetrix_cpp/parsers/sensors/base_sensor_data.hpp>

class IntensityData : public SensorData {
public:
  pin_t a_pin = (pin_t)-1;
  pin_t d_pin = (pin_t)-1;

  IntensityData(std::shared_ptr<Parser> parser,
                std::shared_ptr<Mirte_Board> board, std::string name,
                std::map<std::string, rclcpp::ParameterValue> parameters,
                std::set<std::string> &unused_keys);

  // Mirte boards support not connecting either the digital or analog pin.

  bool check();

  static std::string get_device_class() { return "intensity"; }
};