#ifdef WITH_GPIO
#include <iostream>
#include <mirte_telemetrix_cpp/gpio_pin.hpp>
#include <rcpputils/asserts.hpp>

GPIOPin::GPIOPin(std::string pin_name, std::string gpiod_name)
    : name(pin_name), gpiod_name(gpiod_name) {
  rcpputils::check_true(pin_name.substr(0, 4) == "GPIO",
                        "Format of pin invallid, should be 'GPIOx_yz' with "
                        "0<=x<=5, A<=y<=D, 0<=z<=7");
  try {
    // name should be GPIOx_yz, with 0<=x<=5, A<=y<=D, 0<=z<=7
    this->chip_name = (std::string)("gpiochip") + pin_name[4];
    this->block = pin_name[6];
    this->block_line = (int)(pin_name[7] - '0');
    this->line = 8 * (this->block - 'A') + this->block_line;
    this->chip = ::gpiod::chip(this->chip_name);
    this->gpio_line = chip.get_line(this->line);
    this->enabled = true;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    this->enabled = false;
  }
  // std::cout << "new pin" << pin_name << std::endl;
}

void GPIOPin::setup() {
  if (!enabled) {
    return;
  }
  if (!configured) {
    gpio_line.request({gpiod_name, gpiod::line_request::DIRECTION_OUTPUT, 0},
                      0);
    gpio_line.set_value(0); // Force off before turning on.
    configured = true;
  }
}

void GPIOPin::write(bool value) {
  if (!enabled) {
    return;
  }
  if (!configured)
    setup();

  gpio_line.set_value((int)value);
}

bool GPIOPin::read() {
  if (!enabled) {
    return false;
  }
  if (!configured)
    setup();

  return (bool)gpio_line.get_value();
}

GPIOPin::~GPIOPin() {
  if (!enabled) {
    return;
  }
  if (configured) {
    gpio_line.set_value(0);
    gpio_line.release();
    configured = false;
  }
}
#endif