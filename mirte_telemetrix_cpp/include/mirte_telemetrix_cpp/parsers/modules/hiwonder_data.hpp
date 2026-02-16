#pragma once

#include <mirte_telemetrix_cpp/parsers/modules/hiwonder/hiwonder_servo_data.hpp>
#include <mirte_telemetrix_cpp/parsers/modules/module_data.hpp>

// TODO: UART Module class
class HiWonderBusData : public ModuleData {
public:
  uint8_t uart_port;
  pin_t tx_pin = (pin_t)-1;
  pin_t rx_pin = (pin_t)-1;
  // The name to group the servos with. Defaults to "hiwonder".
  // ('servo/GROUP_NAME/..')
  std::string group_name = "hiwonder";
  std::vector<std::shared_ptr<HiWonderServoData>> servos;

  HiWonderBusData(std::shared_ptr<Parser> parser,
                  std::shared_ptr<Mirte_Board> board, std::string name,
                  std::map<std::string, rclcpp::ParameterValue> parameters,
                  std::set<std::string> &unused_keys);

  bool check();

  static std::string get_module_type() { return "hiwonder_servo"; };
  ~HiWonderBusData() {}
};