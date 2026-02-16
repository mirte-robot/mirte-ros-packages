#pragma once
#include <tuple>

#include <mirte_telemetrix_cpp/actuators/motor.hpp>

class PPMotor : public Motor {
public:
  PPMotor(NodeData node_data, MotorData motor_data);

  // No Pin Initializer, for use with motors on (sub)modules.
  PPMotor(NodeData node_data, pin_t pinA, pin_t pinB, DeviceData data,
          bool inverted, int max_pwm);

  std::tuple<uint32_t, uint32_t> calc_pwm_speed(int speed);
  virtual void set_speed(int speed) override;

  pin_t pwmA_pin;
  pin_t pwmB_pin;
};