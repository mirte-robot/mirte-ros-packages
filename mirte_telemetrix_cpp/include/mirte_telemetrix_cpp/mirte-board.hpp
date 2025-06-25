#include <map>
#include <string>
using pin_map = std::map<std::string, std::string>;
using connector_map = std::map<std::string, pin_map>;

#ifndef MIRTE_BOARD_HPP
#define MIRTE_BOARD_HPP

#include <map>    // for map, map<>::value_compare
#include <memory> // for shared_ptr, __shared_ptr_access
#include <string> // for string, basic_string, operator<

#include "mirte_telemetrix_cpp/parsers/parsers.hpp"
#include "mirte_telemetrix_cpp/pcbs/v06.hpp"
#include "mirte_telemetrix_cpp/pcbs/v08.hpp"

#include <mirte_msgs/srv/get_board_characteristics.hpp>

class Mirte_Board {
public:
  virtual const double get_voltage_level() const = 0;
  virtual const int get_adc_bits() const = 0;
  virtual const int get_max_pwm() const = 0;
  virtual int resolvePin(std::string pin) = 0;
  virtual std::map<std::string, int>
  resolveConnector(std::string connector) = 0;
  virtual uint8_t resolveI2CPort(uint8_t sda) = 0;
  virtual uint8_t resolveUARTPort(uint8_t pin) = 0;

  /// @brief Indicate if a pin is analog capable
  /// @param pin The MCU pin number
  /// @return True if the pin is analog capable
  virtual const bool is_analog_pin(uint8_t pin) const = 0;

  /// @brief Indicate if a pin is PWM capable
  /// @param pin The MCU pin number
  /// @return True if the pin is PWM capable
  virtual const bool is_pwm_pin(uint8_t pin) const = 0;

  static std::shared_ptr<Mirte_Board> create(std::shared_ptr<Parser> parser);

  void get_board_characteristics_service_callback(
      const mirte_msgs::srv::GetBoardCharacteristics::Request::ConstSharedPtr
          req,
      mirte_msgs::srv::GetBoardCharacteristics::Response::SharedPtr res) const;
  virtual ~Mirte_Board() {}
};

class Mirte_Board_atmega328p : public Mirte_Board {
public:
  Mirte_Board_atmega328p();
  std::map<std::string, int> resolveConnector(std::string connector) override;
  int resolvePin(std::string pin) override;
  virtual const double get_voltage_level() const override { return 5.0; }
  virtual const int get_adc_bits() const override;
  virtual const int get_max_pwm() const override { return 255; }
  virtual uint8_t resolveI2CPort(uint8_t sda) override { return 0; }

  virtual uint8_t resolveUARTPort(uint8_t pin) override { return 0; };

  /// @brief Indicate if a pin is analog capable.
  /// @param pin The MCU pin number
  /// @return True if the pin is analog capable
  virtual const bool is_analog_pin(uint8_t pin) const override;

  /// @brief Indicate if a pin is PWM capable
  /// @param pin The MCU pin number
  /// @return True if the pin is PWM capable
  virtual const bool is_pwm_pin(uint8_t pin) const override;
};
class Mirte_Board_pico : public Mirte_Board {
public:
  // Mirte_Board_pico(
  //   // std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh
  //   );
  Mirte_Board_pico();
  std::map<std::string, int> resolveConnector(std::string connector) override;
  int resolvePin(std::string pin) override;
  virtual const double get_voltage_level() const override { return 3.3; }
  virtual const int get_adc_bits() const override { return 12; }
  virtual const int get_max_pwm() const override {
    return 20000;
  } // TODO: check with actual board
  virtual uint8_t resolveI2CPort(uint8_t sda) override;
  virtual uint8_t resolveUARTPort(uint8_t pin) override;
  virtual const bool is_analog_pin(uint8_t pin) const override;
  virtual const bool is_pwm_pin(uint8_t pin) const override;
};

class Mirte_Board_pcb : public Mirte_Board {
public:
  Mirte_Board_pcb(std::shared_ptr<Mirte_Board> mcu, std::string version);
  // Mirte_Board_pcb(
  //   // std::shared_ptr<TMX> tmx, std::shared_ptr<rclcpp::Node> nh,
  //   std::shared_ptr<Mirte_Board> mcu);
  std::shared_ptr<Mirte_Board> mcu; // to look up pins
  std::map<std::string, int> resolveConnector(std::string connector) override;
  int resolvePin(std::string pin) override;
  virtual const double get_voltage_level() const override {
    return mcu->get_voltage_level();
  }
  virtual const int get_adc_bits() const override {
    return mcu->get_adc_bits();
  }
  virtual const int get_max_pwm() const override { return mcu->get_max_pwm(); }

  virtual uint8_t resolveI2CPort(uint8_t sda) override;
  virtual uint8_t resolveUARTPort(uint8_t pin) override;
  virtual const bool is_analog_pin(uint8_t pin) const override {
    return mcu->is_analog_pin(pin);
  };
  virtual const bool is_pwm_pin(uint8_t pin) const override {
    return mcu->is_pwm_pin(pin);
  };

  std::string version;
  connector_map connectors = mirte_pico_pcb_map08;
};

#endif