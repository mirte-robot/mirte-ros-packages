#include <iostream> // for operator<<, endl, basic_ostream, ostream
#include <map>      // for map
#include <optional> // for optional
#include <string>   // for operator<<, string, allocator

#include "mirte_telemetrix_cpp/mirte-board.hpp" // for Mirte_Board_pico
#include "mirte_telemetrix_cpp/util.hpp" // for try_parse_int, starts_with

Mirte_Board_pico::Mirte_Board_pico() {}

int Mirte_Board_pico::resolvePin(std::string pin_name) {
  // std::cout << "Mirte_Board_pico::resolvePin " << pin_name << std::endl;
  if (auto pin = try_parse_int(pin_name)) {
    // std::cout << "Mirte_Board_pico::tryparse " << *pin << std::endl;
    return pin.value();
  }
  if (starts_with(pin_name, "GP")) {
    if (auto pin = try_parse_int(pin_name.substr(2))) {
      return pin.value();
    }
  }
  std::cerr << "Not implemented: pico::resolvePin : " << pin_name << std::endl;
  return -1;
}

std::map<std::string, int>
Mirte_Board_pico::resolveConnector(std::string connector) {
  std::cerr << "Not implemented: pico::resolveConnector : " << connector
            << std::endl;
  return {};
}

// shit implementation, needs to be better
// i2c_port0_sda_pins = [0, 4, 8, 12, 20, 16]
// i2c_port1_sda_pins = [2, 6, 10, 14, 26, 18]

uint8_t Mirte_Board_pico::resolveI2CPort(uint8_t sda) {
  switch (sda) {
  case 0:
  case 4:
  case 8:
  case 12:
  case 16:
  case 20:
    return 0;
    break;
  case 2:
  case 6:
  case 10:
  case 14:
  case 18:
  case 26:
    return 1;
    break;
  default:
    return 0xFF;
    break;
  }
}

uint8_t Mirte_Board_pico::resolveUARTPort(uint8_t pin) {
  switch (pin) {
  case 0:
  case 1:
  case 12:
  case 13:
  case 16:
  case 17:
    return 0;
  case 4:
  case 5:
  case 8:
  case 9:
    return 1;
  default:
    return 0xFF;
  }
}

const bool Mirte_Board_pico::is_analog_pin(uint8_t pin) const {
  switch (pin) {
  case 26:
  case 27:
  case 28:
  case 29: // The internal temperature sensor
    return true;
  default:
    return false;
  }
}

const bool Mirte_Board_pico::is_pwm_pin(uint8_t pin) const { return true; }