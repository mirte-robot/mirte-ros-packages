#include <mirte_telemetrix_cpp/parsers/sensors/intensity_data.hpp>

IntensityData::IntensityData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys)
    : SensorData(parser, board, name, IntensityData::get_device_class(),
                 parameters, unused_keys) {
  auto key = get_device_key(this);
  auto logger = parser->logger;
  bool got_pins = false;
  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);

    // Mirte boards support not connecting either the digital or analog pin.
    if (pins.count("analog")) {
      this->a_pin = pins["analog"];
      got_pins = true;
    }
    if (pins.count("digital")) {
      this->d_pin = pins["digital"];
      got_pins = true;
    }
  } else {
    if (unused_keys.erase("pins.analog")) {
      got_pins = true;
      this->a_pin = board->resolvePin(get_string(parameters["pins.analog"]));
    }

    if (unused_keys.erase("pins.digital")) {
      got_pins = true;
      this->d_pin = board->resolvePin(get_string(parameters["pins.digital"]));
    }
  }
  if (!got_pins) {
    RCLCPP_ERROR(logger, "Device %s has no a connector or pins specified.",
                 key.c_str());
  }
}

bool IntensityData::check() {
  // Mirte boards support not connecting either the digital or analog pin.
  return (a_pin != (pin_t)-1 || d_pin != (pin_t)-1) && SensorData::check();
}
