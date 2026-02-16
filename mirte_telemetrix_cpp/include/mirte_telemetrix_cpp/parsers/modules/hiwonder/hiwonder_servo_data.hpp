#pragma once
#include <mirte_telemetrix_cpp/parsers/device_data.hpp>
// #include "mirte_telemetrix_cpp/mirte-board.hpp"
#include "mirte_telemetrix_cpp/parsers/parsers.hpp"

// class Hiwonder_servo_data {
// public:
//   std::string name;
//   uint8_t id;
//   int min_angle_out = 0;
//   int max_angle_out = 24000;
//   int home_out = 1000;
//   float min_angle_in = -1;
//   float max_angle_in = -1;
//   bool invert = false;
//   std::string frame_id;
//   Hiwonder_servo_data(std::string name, uint8_t id, int min_angle_out,
//                       int max_angle_out, int home_out, float min_angle_in,
//                       float max_angle_in, bool invert, std::string frame_id)
//                       {
//     this->name = name;
//     this->id = id;
//     this->min_angle_out = min_angle_out;
//     this->max_angle_out = max_angle_out;
//     this->home_out = home_out;
//     this->min_angle_in = min_angle_in;
//     this->max_angle_in = max_angle_in;
//     this->invert = invert;
//     this->frame_id = frame_id;
//   }
//   Hiwonder_servo_data() {}
//   static std::vector<std::shared_ptr<Hiwonder_servo_data>>
//   parse_hiwonder_servo_data(std::shared_ptr<Parser> parser,
//                             std::shared_ptr<Mirte_Board> board,
//                             std::string bus_name);
//   bool check() { return name != ""; }
// };

class HiWonderServoData : public DeviceData {
public:
  uint8_t id = -1;

  // TODO: Maybe change this
  // angle in centi-degrees
  int min_angle_out = 0;
  // angle in centi-degrees
  int max_angle_out = 24000;

  int home_out = 1000;

  float min_angle_in = -1;
  float max_angle_in = -1;

  bool invert = false;
  bool enable_motor = false;

  HiWonderServoData(std::shared_ptr<Parser> parser,
                    std::shared_ptr<Mirte_Board> board, std::string name,
                    std::map<std::string, rclcpp::ParameterValue> parameters,
                    std::set<std::string> &unused_keys,
                    std::string base_frame_id);

  // FIXME: CHANGE
  static std::vector<std::shared_ptr<HiWonderServoData>>
  parse_hiwonder_servo_data(std::shared_ptr<Parser> parser,
                            std::shared_ptr<Mirte_Board> board,
                            std::string bus_name,
                            std::set<std::string> &unused_keys,
                            std::string base_frame_id);

  // static std::string get_module_type() {return "hiwonder_servo.servo"; };
  virtual bool check() override;
};