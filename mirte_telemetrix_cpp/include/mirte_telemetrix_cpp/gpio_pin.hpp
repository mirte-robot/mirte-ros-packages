#pragma once
#ifdef WITH_GPIO
#include <string>

#include <gpiod.hpp>

class GPIOPin {
public:
  GPIOPin(std::string name, std::string gpiod_name);

  std::string name;
  std::string gpiod_name;

  char block = 'A';
  std::string chip_name = "gpiochip0";
  int block_line = 0;
  int line = 0;

  void setup();
  void write(bool value);
  bool read();
  ~GPIOPin();

private:
  bool configured = false;
  bool enabled = false;
  gpiod::chip chip;
  gpiod::line gpio_line;
};

#else
#error "GPIOD not included, so this file is not available."
#endif