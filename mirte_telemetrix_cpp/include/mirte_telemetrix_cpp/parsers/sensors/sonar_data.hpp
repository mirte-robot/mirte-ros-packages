#pragma once

#include <memory>
#include <vector>

#include <mirte_telemetrix_cpp/mirte-board.hpp>

#include <mirte_telemetrix_cpp/parsers/sensors/base_sensor_data.hpp>

class SonarData : public SensorData {
public:
  pin_t trigger = -1;
  pin_t echo = -1;

  SonarData(std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
            std::string name,
            std::map<std::string, rclcpp::ParameterValue> parameters,
            std::set<std::string> &unused_keys);

  bool check();

  static std::string get_device_class() { return "distance"; };
};
