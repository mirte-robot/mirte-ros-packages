#pragma once

// #include <mirte_telemetrix_cpp/parsers/parsers.hpp>
#include <mirte_telemetrix_cpp/parsers/device_data.hpp>

class MotorData : public DeviceData {
public:
  enum class MotorType { PP, DP, DDP };
  pin_t P1 = (pin_t)-1;
  pin_t P2 = (pin_t)-1;
  pin_t D1 = (pin_t)-1;
  pin_t D2 = (pin_t)-1;
  bool inverted = false;
  // Default to PP motor
  MotorType type = MotorType::PP;

  bool check();

  MotorData(std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
            std::string name,
            std::map<std::string, rclcpp::ParameterValue> parameters,
            std::set<std::string> &unused_keys);
  ~MotorData(){};
  static std::string get_device_class() { return "motor"; }
};