#pragma once

#include <mirte_telemetrix_cpp/actuators/motor.hpp>
// TODO: fix this for dp, as this only works with dir and pwm, not 2x pwm, with
// one as bool.
class DPMotor : public Motor {
public:
  DPMotor(NodeData node_data, MotorData motor_data);

  virtual void set_speed(int speed) override;
  virtual std::vector<std::pair<uint8_t, uint16_t>>
  get_speed_multi(int speed) override;
  pin_t dir_pin;
  pin_t pwm_pin;
  bool normal_dp_mode = true;
};
