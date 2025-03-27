#pragma once

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>

class VEML6040Data : public I2CModuleData {
public:
  VEML6040Data(std::shared_ptr<Parser> parser,
               std::shared_ptr<Mirte_Board> board, std::string name,
               std::map<std::string, rclcpp::ParameterValue> parameters,
               std::set<std::string> &unused_keys);

  virtual bool check() override;
  using I2CModuleData::check;

  static std::string get_module_type() { return "color"; };
  static std::string get_device_class() { return "color"; };
};
