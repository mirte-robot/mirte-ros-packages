#include <mirte_telemetrix_cpp/parsers/sensors/sonar_data.hpp>

SonarData::SonarData(std::shared_ptr<Parser> parser,
                     std::shared_ptr<Mirte_Board> board, std::string name,
                     std::map<std::string, rclcpp::ParameterValue> parameters,
                     std::set<std::string> &unused_keys)
    : SensorData(parser, board, name, SonarData::get_device_class(), parameters,
                 unused_keys, DeviceDuration(1000.0 / 10.0)) {
  auto key = get_device_key(this);
  auto logger = parser->logger;

  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);
    this->trigger = pins["trigger"];
    this->echo = pins["echo"];
  } else if (unused_keys.erase("pins")) {
    auto subkeys =
        parser->get_params_keys(parser->build_param_name(key, "pins"));

    if (subkeys.erase("trigger")) {
      this->trigger = board->resolvePin(get_string(parameters["pins.trigger"]));
    }

    if (subkeys.erase("echo")) {
      this->echo = board->resolvePin(get_string(parameters["pins.echo"]));
    }

    for (auto subkey : subkeys) {
      unused_keys.insert(parser->build_param_name("pins", subkey));
    }
  } else {
    RCLCPP_ERROR(logger, "Device %s has no a connector or pins specified.",
                 key.c_str());
  }
}

bool SonarData::check() {
  return trigger != (pin_t)-1 && echo != (pin_t)-1 && SensorData::check();
}
