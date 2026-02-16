#pragma once

#include <mirte_telemetrix_cpp/actuators/motor.hpp>

class DDPMotor : public Motor {
public:
  DDPMotor(NodeData node_data, MotorData motor_data);

  virtual void set_speed(int speed) override;

  pin_t A_pin;
  pin_t B_pin;
  pin_t pwm_pin;
};
