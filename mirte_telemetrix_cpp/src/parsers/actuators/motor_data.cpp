#include <boost/algorithm/string.hpp>

#include <mirte_telemetrix_cpp/parsers/actuators/motor_data.hpp>

MotorData::MotorData(std::shared_ptr<Parser> parser,
                     std::shared_ptr<Mirte_Board> board, std::string name,
                     std::map<std::string, rclcpp::ParameterValue> parameters,
                     std::set<std::string> &unused_keys)
    : DeviceData(parser, board, name, MotorData::get_device_class(), parameters,
                 unused_keys) {
  auto logger = parser->logger;

  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);

    this->P1 = pins["P1"];
    this->P2 = pins["P2"];
    this->D1 = pins["D1"];
    this->D2 = pins["D2"];
  } else if (unused_keys.erase("pins")) {
    auto subkeys = parser->get_params_keys(
        parser->build_param_name(get_device_key(this), "pins"));

    if (subkeys.erase("p1")) {
      this->P1 = board->resolvePin(get_string(parameters["pins.p1"]));
    }
    if (subkeys.erase("p2")) {
      this->P2 = board->resolvePin(get_string(parameters["pins.p2"]));
    }

    if (subkeys.erase("d1")) {
      this->D1 = board->resolvePin(get_string(parameters["pins.d1"]));
    }
    if (subkeys.erase("d2")) {
      this->D2 = board->resolvePin(get_string(parameters["pins.d2"]));
    }

    for (auto subkey : subkeys) {
      unused_keys.insert(parser->build_param_name("pins", subkey));
    }
  } else {
    RCLCPP_ERROR(logger, "Device %s.%s has no a connector or pins specified.",
                 get_device_class().c_str(), name.c_str());
  }

  if (unused_keys.erase("type")) {
    std::string motor_type = get_string(parameters["type"]);
    boost::algorithm::to_lower(motor_type);
    if (motor_type == "pp") {
      this->type = MotorType::PP;
    } else if (motor_type == "dp") {
      this->type = MotorType::DP;
    } else if (motor_type == "ddp") {
      this->type = MotorType::DDP;
    } else {
      RCLCPP_ERROR(
          logger,
          "Unknown Motor type '%s' for Motor %s | Defaulting to PP Motor",
          motor_type.c_str(), name.c_str());
    }
  } else {
    RCLCPP_WARN(logger, "No motor type found for %s defaulting to PP motor",
                name.c_str());
  }

  if (unused_keys.erase("inverted")) {
    this->inverted = parameters["inverted"].get<bool>();
  }
}

bool MotorData::check() {
  bool device_ok = DeviceData::check();

  switch (type) {
  case MotorType::PP:
    return P1 != (pin_t)-1 && P2 != (pin_t)-1 && device_ok;
  case MotorType::DP:
    return P1 != (pin_t)-1 && D1 != (pin_t)-1 && device_ok;
  case MotorType::DDP:
    return P1 != (pin_t)-1 && D1 != (pin_t)-1 && D2 != (pin_t)-1 && device_ok;
  default:
    return false;
  }
}
