#pragma once

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>

#ifdef WITH_GPIO
#include <mirte_telemetrix_cpp/gpio_pin.hpp>
#endif

class INA226Data : public I2CModuleData {
public:
  float max_current = 10;
  float max_voltage = 14;
  float min_voltage = 10.5;
  float power_low_time = 5;
  float turn_off_time = 30;
  uint8_t shutdown_relay_pin = 0;
  uint8_t shutdown_switch_in_pin = 0;
  bool shutdown_switch_off_value = false;
  uint8_t shutdown_switch_time_sec = 1;
  bool disable_shutdown_relay = false;
  bool shutdown_relay_off_value = false;

#ifdef WITH_GPIO
  bool use_percentage_led = false;
  std::shared_ptr<GPIOPin> percentage_led_pin;
#endif

  INA226Data(std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
             std::string name,
             std::map<std::string, rclcpp::ParameterValue> parameters,
             std::set<std::string> &unused_keys);

  bool check() override;
  using I2CModuleData::check;
  static std::string get_module_type() { return "ina226"; };
  // static std::string get_device_class() { return "power"; };
};