#pragma once

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>

class ADXL345Data : public I2CModuleData {
public:
  ADXL345Data(std::shared_ptr<Parser> parser,
              std::shared_ptr<Mirte_Board> board, std::string name,
              std::map<std::string, rclcpp::ParameterValue> parameters,
              std::set<std::string> &unused_keys);

  bool check() override;
  using I2CModuleData::check;

  static std::string get_module_type() { return "adxl345"; };

  // Required to change the default standard frame_id as specified in REP0145
  static std::map<std::string, rclcpp::ParameterValue> insert_default_frame_id(
      std::map<std::string, rclcpp::ParameterValue> parameters);
  static std::set<std::string> &
  insert_default_frame_id(std::set<std::string> &unused_keys);
};