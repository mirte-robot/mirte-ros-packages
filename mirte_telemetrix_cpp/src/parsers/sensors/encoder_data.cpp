#include <functional>

#include <mirte_telemetrix_cpp/parsers/sensors/encoder_data.hpp>

using namespace std::placeholders;

EncoderData::EncoderData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys)
    : SensorData(parser, board, name, EncoderData::get_device_class(),
                 parameters, unused_keys) {
  auto key = this->get_device_class() + "." + name;
  auto logger = parser->logger;
  bool got_pins = false;
  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);
    this->pinA = pins["pinA"];
    this->pinB = pins["pinB"];
    got_pins = true;
  }
  if (unused_keys.erase("pins.A")) {
    this->pinA = board->resolvePin(get_string(parameters["pins.A"]));
    got_pins = true;
  }

  if (unused_keys.erase("pins.B")) {
    this->pinB = board->resolvePin(get_string(parameters["pins.B"]));
    got_pins = true;
  }

  if (unused_keys.erase("pins.pin")) {
    this->pinA = board->resolvePin(get_string(parameters["pins.pin"]));
    this->pinB = (pin_t)-1;
    got_pins = true;
  }
  if (!got_pins) {
    RCLCPP_ERROR(logger, "Device %s has no a connector or pins specified.",
                 key.c_str());
  }
}

bool EncoderData::check() {
  // Don't check pinB, since this could be a single pin encoder.
  return pinA != (pin_t)-1 && SensorData::check();
}
