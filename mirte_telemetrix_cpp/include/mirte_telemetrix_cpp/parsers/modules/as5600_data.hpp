#pragma once

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>

#ifdef WITH_GPIO
#include <mirte_telemetrix_cpp/gpio_pin.hpp>
#endif

class AS5600Data : public I2CModuleData {
public:
  std::vector<std::tuple<int, std::string>> encoders = {};
  uint8_t mux = 0x00;
  AS5600Data(std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
             std::string name,
             std::map<std::string, rclcpp::ParameterValue> parameters,
             std::set<std::string> &unused_keys,
             std::vector<std::tuple<int, std::string>> encoders);

  bool check() override;
  using I2CModuleData::check;
  static std::string get_module_type() { return "as5600"; };
};