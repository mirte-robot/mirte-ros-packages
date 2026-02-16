#pragma once
#include <memory>
#include <set>
#include <string>

// #include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <mirte_telemetrix_cpp/parsers/parsers.hpp>

class PCA_Motor_data {
public:
  std::string name;
  pin_t pinA = (pin_t)-1;
  pin_t pinB = (pin_t)-1;
  bool invert = false;
  PCA_Motor_data(std::string name, pin_t pinA, pin_t pinB, bool invert) {
    this->name = name;
    this->pinA = pinA;
    this->pinB = pinB;
    this->invert = invert;
  }
  PCA_Motor_data() {}
  static std::vector<std::shared_ptr<PCA_Motor_data>> parse_pca_motor_data(
      std::shared_ptr<Parser> parser,
      /*std::shared_ptr<Mirte_Board> board,*/ std::string pca_key,
      std::set<std::string> &unused_keys);
  bool check() { return pinA != (pin_t)-1 && pinB != (pin_t)-1 && name != ""; }
};