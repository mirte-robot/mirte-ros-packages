#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/pcbs/v08.hpp>

auto const DEFAULT_VERSION = 0.8;

Mirte_Board_pcb::Mirte_Board_pcb(std::shared_ptr<Mirte_Board> mcu,
                                 std::string version)
    : mcu(mcu), version(version) {
  std::cout << "Mirte_Board_pcb::Mirte_Board_pcb" << version << std::endl;
  auto v = try_parse_double(version).value_or(DEFAULT_VERSION);
  if (v == 0.8) {
    // std::cout << "0.8" << version << std::endl;
    this->connectors = mirte_pico_pcb_map08;
  } else if (v == 0.6) {
    this->connectors = mirte_pico_pcb_map06;
  } else {
    std::cerr << "Unknown PCB version: " << version << std::endl;
  }
}

std::map<std::string, int>
Mirte_Board_pcb::resolveConnector(std::string connector) {
  const auto pins = this->connectors.at(connector);
  std::map<std::string, int> resolved_pins;
  for (auto pin : pins) {
    resolved_pins[pin.first] = mcu->resolvePin(pin.second);
  }
  return resolved_pins;
}

int Mirte_Board_pcb::resolvePin(std::string pin) {
  return mcu->resolvePin(pin);
}

uint8_t Mirte_Board_pcb::resolveI2CPort(uint8_t sda) {
  return mcu->resolveI2CPort(sda);
}

uint8_t Mirte_Board_pcb::resolveUARTPort(uint8_t pin) {
  return mcu->resolveUARTPort(pin);
}
