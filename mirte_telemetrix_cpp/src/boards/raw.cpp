#include <iostream> // for operator<<, endl, basic_ostream, ostream
#include <map>      // for map
#include <optional> // for optional
#include <string>   // for operator<<, string, allocator

#include "mirte_telemetrix_cpp/mirte-board.hpp" // for Mirte_board_raw
#include "mirte_telemetrix_cpp/util.hpp" // for try_parse_int, starts_with

Mirte_Board_raw::Mirte_Board_raw(std::shared_ptr<tmx_cpp::TMX> tmx) {
  this->tmx = tmx;
}

int Mirte_Board_raw::resolvePin(std::string pin_name) {
  tmx->get_feature(
      (tmx_cpp::MESSAGE_TYPE)((int)tmx_cpp::MESSAGE_TYPE::MAX - 1));

  // std::cout << "Mirte_Board_raw::resolvePin " << pin_name << std::endl;
  if (auto pin = try_parse_int(pin_name)) {
    // std::cout << "Mirte_Board_raw::tryparse " << *pin << std::endl;
    return pin.value();
  }
  if (starts_with(pin_name, "D")) {
    if (auto pin = try_parse_int(pin_name.substr(1))) {
      return pin.value();
    }
  } else if (starts_with(pin_name, "A")) {
    if (auto pin = try_parse_int(pin_name.substr(1))) {
      auto pin_num = pin.value();
      if (pin_num >= 0 && pin_num < tmx->board_features.analog_pins) {
        auto pin = tmx->board_features.analog_pins_list[pin_num];
        if (pin == 0xff) {
          return -1; // pin not available, some boards have a non-contiguous
                     // analog pin list, so we return -1 if the pin is not available
        }
        return pin; // return the actual pin number
      } else {
        std::cerr << "Invalid analog pin number: " << pin_num << std::endl;
        std::cerr << "Valid range: 0-" << tmx->board_features.analog_pins - 1
                  << std::endl;
        return -1;
      }
    }
  }
  std::cerr << "Not implemented: raw::resolvePin : " << pin_name << std::endl;
  return -1;
}

std::map<std::string, int>
Mirte_Board_raw::resolveConnector(std::string connector) {
  std::cerr << "Not implemented: raw::resolveConnector : " << connector
            << std::endl;
  return {};
}

// shit implementation, needs to be better
// i2c_port0_sda_pins = [0, 4, 8, 12, 20, 16]
// i2c_port1_sda_pins = [2, 6, 10, 14, 26, 18]

// uint8_t Mirte_Board_raw::resolveI2CPort(uint8_t sda) {
// //   switch (sda) {
// //   case 0:
// //   case 4:
// //   case 8:
// //   case 12:
// //   case 16:
// //   case 20:
// //     return 0;
// //     break;
// //   case 2:
// //   case 6:
// //   case 10:
// //   case 14:
// //   case 18:
// //   case 26:
// //     return 1;
// //     break;
// //   default:
// //     return 0xFF;
// //     break;
// //   }
//     return 0xFF;
// }

// uint8_t Mirte_Board_raw::resolveUARTPort(uint8_t pin) {
// //   switch (pin) {
// //   case 0:
// //   case 1:
// //   case 12:
// //   case 13:
// //   case 16:
// //   case 17:
// //     return 0;
// //   case 4:
// //   case 5:
// //   case 8:
// //   case 9:
// //     return 1;
// //   default:
// //     return 0xFF;
// //   }
//     return 0xFF;
// }

const bool Mirte_Board_raw::is_analog_pin(uint8_t pin) const {
  //   switch (pin) {
  //   case 26:
  //   case 27:
  //   case 28:
  //   case 29: // The internal temperature sensor
  //     return true;
  //   default:
  //     return false;
  //   }
  for(const auto &p : tmx->board_features.analog_pins_list) {
    if (p == pin) {
      return true;
    }
  }
  return false;
}

const bool Mirte_Board_raw::is_pwm_pin(uint8_t pin) const { return false; }