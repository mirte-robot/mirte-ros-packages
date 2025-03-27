#include <mirte_telemetrix_cpp/parsers/actuators/servo_data.hpp>

ServoData::ServoData(std::shared_ptr<Parser> parser,
                     std::shared_ptr<Mirte_Board> board, std::string name,
                     std::map<std::string, rclcpp::ParameterValue> parameters,
                     std::set<std::string> &unused_keys)
    : DeviceData(parser, board, name, ServoData::get_device_class(), parameters,
                 unused_keys) {
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

  if (unused_keys.erase("min_pulse")) {
    this->min_pulse = parameters["min_pulse"].get<int>();
  }

  if (unused_keys.erase("max_pulse")) {
    this->max_pulse = parameters["max_pulse"].get<int>();
  }

  if (unused_keys.erase("min_angle")) {
    this->min_angle = get_float(parameters["min_angle"]);
  }

  if (unused_keys.erase("max_angle")) {
    this->max_angle = get_float(parameters["max_angle"]);
  }
}

ServoData::ServoData(pin_t pin, int min_pulse, int max_pulse, float min_angle,
                     float max_angle, std::string name, std::string frame_id)
    : DeviceData(name, frame_id), pin(pin), min_pulse(min_pulse),
      max_pulse(max_pulse), min_angle(min_angle), max_angle(max_angle) {}

bool ServoData::check() { return pin != (pin_t)-1 && DeviceData::check(); }
