#include <algorithm> // for clamp
#include <iostream>  // for operator<<, basic_ostream, endl, cerr
#include <map>       // for map
#include <optional>  // for optional
#include <string>    // for string, allocator, operator<<

#include <mirte_telemetrix_cpp/mirte-board.hpp> // for Mirte_Board_atmega328p
#include <mirte_telemetrix_cpp/util.hpp> // for starts_with, try_parse_int
Mirte_Board_atmega328p::Mirte_Board_atmega328p(
    // std::shared_ptr<TMX> tmx,
    // std::shared_ptr<rclcpp::Node> nh
    )
// : Mirte_Board(tmx, nh)
{}

int Mirte_Board_atmega328p::resolvePin(std::string pin_name) {
  if (auto pin = try_parse_int(pin_name)) {
    return pin.value();
  }
  if (starts_with(pin_name, "RX")) {
    return 0;
  }
  if (starts_with(pin_name, "TX")) {
    return 1;
  }
  if (starts_with(pin_name, "A")) {
    if (auto pin = try_parse_int(pin_name.substr(1))) {
      return pin.value() + 14;
    }
  }
  if (starts_with(pin_name, "D")) {
    if (auto pin = try_parse_int(pin_name.substr(1))) {
      return std::clamp(pin.value(), 0, 13);
    }
  }
  std::cerr << "Not implemented: atmega328p::resolvePin : " << pin_name
            << std::endl;
  return -1;
}

std::map<std::string, int>
Mirte_Board_atmega328p::resolveConnector(std::string connector) {
  std::cerr << "Not implemented: atmega328p::resolveConnector : " << connector
            << std::endl;
  return {};
}

const int Mirte_Board_atmega328p::get_adc_bits() const { return 10; }

const bool Mirte_Board_atmega328p::is_analog_pin(uint8_t pin) const {
  switch (pin) {
  case 14 + 0:
  case 14 + 1:
  case 14 + 2:
  case 14 + 3:
  case 14 + 4:
  case 14 + 5:
  case 14 + 6:
  case 14 + 7:
    return true;
  default:
    return false;
  }
}

const bool Mirte_Board_atmega328p::is_pwm_pin(uint8_t pin) const {
  switch (pin) {
  case 3:
  case 5:
  case 6:
  case 9:
  case 10:
  case 11:
    return true;
  default:
    return false;
  }
}