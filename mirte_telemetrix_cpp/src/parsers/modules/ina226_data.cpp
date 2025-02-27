#include <mirte_telemetrix_cpp/parsers/modules/ina226_data.hpp>

INA226Data::INA226Data(std::shared_ptr<Parser> parser,
                       std::shared_ptr<Mirte_Board> board, std::string name,
                       std::map<std::string, rclcpp::ParameterValue> parameters,
                       std::set<std::string> &unused_keys)
    : I2CModuleData(parser, board, name, parameters, unused_keys,
                    get_module_type()) {
  auto logger =
      parser->logger.get_child(get_device_class()).get_child(this->name);

  // Set default for address
  if ((!parameters.count("addr")) && this->addr == 0xFF)
    this->addr = 0x40;

  if (unused_keys.erase("max_current"))
    this->max_current = get_float(parameters["max_current"]);

  if (unused_keys.erase("max_voltage"))
    this->max_voltage = get_float(parameters["max_voltage"]);

  if (unused_keys.erase("min_voltage"))
    this->min_voltage = get_float(parameters["min_voltage"]);

  if (unused_keys.erase("power_low_time"))
    this->power_low_time = get_float(parameters["power_low_time"]);

  if (unused_keys.erase("percentage_led_pin")) {
#ifdef WITH_GPIO
    this->use_percentage_led = true;
    this->percentage_led_pin =
        std::make_shared<GPIOPin>(get_string(parameters["percentage_led_pin"]),
                                  "mirte-telemetrix-battery-indicator-led");
#else
    RCLCPP_ERROR(logger, "Percentage LED is only supported on the Mirte Master "
                         "(install libgpio-dev and recompile to "
                         "enable)");
#endif
  }
}

bool INA226Data::check() {
  return power_low_time > 0 && I2CModuleData::check(get_module_type());
}
