#pragma once

#include <mirte_telemetrix_cpp/parsers/modules/module_data.hpp>

class I2CModuleData : public ModuleData {
public:
  // Default to invalid address
  uint8_t addr = 0xFF;
  uint8_t port = 0xFF;

  pin_t scl = 0xFF;
  pin_t sda = 0xFF;

  I2CModuleData(std::shared_ptr<Parser> parser,
                std::shared_ptr<Mirte_Board> board, std::string name,
                std::map<std::string, rclcpp::ParameterValue> parameters,
                std::set<std::string> &unused_keys, std::string module_type,
                std::optional<DeviceDuration> = {});
  using ModuleData::check;
  bool check(std::string module_type) override;
};