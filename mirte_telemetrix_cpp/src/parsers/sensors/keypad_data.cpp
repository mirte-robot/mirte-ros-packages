#include <mirte_telemetrix_cpp/parsers/sensors/keypad_data.hpp>

KeypadData::KeypadData(std::shared_ptr<Parser> parser,
                       std::shared_ptr<Mirte_Board> board, std::string name,
                       std::map<std::string, rclcpp::ParameterValue> parameters,
                       std::set<std::string> &unused_keys)
    : SensorData(parser, board, name, KeypadData::get_device_class(),
                 parameters, unused_keys) {
  auto key = get_device_key(this);
  auto logger = parser->logger;

  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);
    this->pin = pins["pin"];
  } else if (unused_keys.erase("pins")) {
    auto subkeys =
        parser->get_params_keys(parser->build_param_name(key, "pins"));

    if (subkeys.erase("pin")) {
      this->pin = board->resolvePin(get_string(parameters["pins.pin"]));
    }

    for (auto subkey : subkeys) {
      unused_keys.insert(parser->build_param_name("pins", subkey));
    }
  } else {
    RCLCPP_ERROR(logger, "Device %s has no a connector or pins specified.",
                 key.c_str());
  }
}

bool KeypadData::check() { return pin != (pin_t)-1 && SensorData::check(); }
