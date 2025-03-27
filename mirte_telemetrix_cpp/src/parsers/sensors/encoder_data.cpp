#include <functional>

#include <mirte_telemetrix_cpp/parsers/sensors/encoder_data.hpp>

using namespace std::placeholders;

EncoderData::EncoderData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys)
    : SensorData(parser, board, name, EncoderData::get_device_class(),
                 parameters, unused_keys) {
  auto key = get_device_key(this);
  auto logger = parser->logger;

  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);
    this->pinA = pins["pinA"];
    this->pinB = pins["pinB"];
  } else if (unused_keys.erase("pins")) {
    auto subkeys =
        parser->get_params_keys(parser->build_param_name(key, "pins"));
    // FIXME: Maybe restructure to test if pin A and B or only pin is set.
    if (subkeys.erase("A")) {
      this->pinA = board->resolvePin(get_string(parameters["pins.A"]));
    }

    if (subkeys.erase("B")) {
      this->pinB = board->resolvePin(get_string(parameters["pins.B"]));
    }

    if (subkeys.erase("pin")) {
      this->pinA = board->resolvePin(get_string(parameters["pins.pin"]));
      this->pinB = (pin_t)-1;
    }

    for (auto subkey : subkeys) {
      unused_keys.insert(parser->build_param_name("pins", subkey));
    }
  } else {
    RCLCPP_ERROR(logger, "Device %s has no a connector or pins specified.",
                 key.c_str());
  }
}

bool EncoderData::check() {
  // Don't check pinB, since this could be a single pin encoder.
  return pinA != (pin_t)-1 && SensorData::check();
}
