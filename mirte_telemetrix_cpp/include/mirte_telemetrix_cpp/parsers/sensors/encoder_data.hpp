#pragma once

#include <mirte_telemetrix_cpp/parsers/sensors/base_sensor_data.hpp>

class EncoderData : public SensorData {
public:
  pin_t pinA = (pin_t)-1;
  pin_t pinB = (pin_t)-1;

  EncoderData(std::shared_ptr<Parser> parser,
              std::shared_ptr<Mirte_Board> board, std::string name,
              std::map<std::string, rclcpp::ParameterValue> parameters,
              std::set<std::string> &unused_keys);

  bool check();

  static std::string get_device_class() { return "encoder"; }
};