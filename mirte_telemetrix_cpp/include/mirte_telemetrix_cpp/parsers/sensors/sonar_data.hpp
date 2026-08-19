#pragma once

#include <memory>
#include <vector>

#include <mirte_telemetrix_cpp/mirte-board.hpp>

#include <mirte_telemetrix_cpp/parsers/sensors/base_sensor_data.hpp>
#include <mirte_telemetrix_cpp/telemetrix_parameters.hpp>
class SonarData : public SensorData {
public:
  pin_t trigger = -1;
  pin_t echo = -1;
  float min_distance = 0.02;
  float max_distance = 4.5;
  float unit = 1.0; // default unit is meters, so 1.0, cm is 0.01, etc.
  SonarData(std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
            std::string name,
            std::map<std::string, rclcpp::ParameterValue> parameters,
            std::set<std::string> &unused_keys,
            const mirte_telemetrix_cpp::Params::Distance::MapDistances
                &distance_params);

  bool check();

  static std::string get_device_class() { return "distance"; };
};
