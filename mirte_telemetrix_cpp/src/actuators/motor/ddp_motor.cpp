#include <mirte_telemetrix_cpp/actuators/motor/ddp_motor.hpp>

DDPMotor::DDPMotor(NodeData node_data, MotorData motor_data)
    : Motor(node_data, {motor_data.D1, motor_data.D2, motor_data.P1},
            motor_data) {
  this->A_pin = motor_data.D1;
  this->B_pin = motor_data.D2;
  this->pwm_pin = motor_data.P1;

  tmx->setPinMode(this->A_pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_OUTPUT);
  tmx->setPinMode(this->B_pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_OUTPUT);
  tmx->setPinMode(this->pwm_pin, tmx_cpp::TMX::PIN_MODES::PWM_OUTPUT);
}

void DDPMotor::set_speed(int speed) {
  // TODO: maybe check last_speed
  // TODO: Than maybe do something with the direction, to replace
  // <DDPMotor::set_speed#1> regions

  // TODO: Python had a clamp, however none of the C++ Motors clamp
  int32_t speed_ = (int32_t)((float)speed * (this->max_pwm) / 100.0);

  if (inverted) {
    speed_ = -speed_;
  }

  if (speed_ >= 0) {
    /* BEGIN <DDPMotor::set_speed#1> */
    tmx->digitalWrite(A_pin, false);
    tmx->digitalWrite(B_pin, false);
    /*  END  <DDPMotor::set_speed#1> */
    tmx->pwmWrite(pwm_pin, speed_);

    tmx->digitalWrite(B_pin, true);
  } else if (speed_ < 0) {
    /* BEGIN <DDPMotor::set_speed#1> */
    tmx->digitalWrite(A_pin, false);
    tmx->digitalWrite(B_pin, false);
    /*  END  <DDPMotor::set_speed#1> */
    tmx->pwmWrite(pwm_pin, -speed_);

    tmx->digitalWrite(A_pin, true);
  }
}