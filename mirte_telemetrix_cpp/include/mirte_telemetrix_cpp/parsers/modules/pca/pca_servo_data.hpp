#pragma once
#include <memory>
#include <set>
#include <string>

// #include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/parsers/parsers.hpp>

class PCA_Servo_data {
public:
  std::string name;
  pin_t pin = (pin_t)-1;
  int min_pulse = 544;
  int max_pulse = 2400;
  // Min/Max angle in degrees
  float min_angle = 0;
  float max_angle = 180;
  bool invert = false;
  std::string pin_mode = "servo"; // or "motor" or "pwm"

  float min_speed = -100; // Only used when pin_mode is motor
  float max_speed = 100;  // Only used when pin_mode is motor

  PCA_Servo_data(std::string name, pin_t pin, int min_pulse, int max_pulse,
                 int min_angle, int max_angle, bool invert,
                 std::string pin_mode, float min_speed, float max_speed) {
    this->name = name;
    this->pin = pin;
    this->min_pulse = min_pulse;
    this->max_pulse = max_pulse;
    this->min_angle = min_angle;
    this->max_angle = max_angle;
    this->invert = invert;
    this->pin_mode = pin_mode;
    this->min_speed = min_speed;
    this->max_speed = max_speed;
  }
  PCA_Servo_data() {}
  static std::vector<std::shared_ptr<PCA_Servo_data>> parse_pca_servo_data(
      std::shared_ptr<Parser> parser,
      /*std::shared_ptr<Mirte_Board> board,*/ std::string pca_name,
      std::set<std::string> &unused_keys);
  bool check() { return pin != (pin_t)-1 && name != ""; }
};
