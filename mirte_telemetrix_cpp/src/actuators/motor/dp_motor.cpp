#include <mirte_telemetrix_cpp/actuators/motor/dp_motor.hpp>
// TODO: FIX dp, as this only works with dir and pwm, not 2x pwm, with one as
// bool.
DPMotor::DPMotor(NodeData node_data, MotorData motor_data)
    : Motor(node_data, {motor_data.D1, motor_data.P1}, motor_data) {
  this->pwm_pin = motor_data.P1;
  this->dir_pin = motor_data.D1;
  this->normal_dp_mode = motor_data.dp_type == "normal";
  // this->type = motor_data.
  tmx->setPinMode(this->pwm_pin, tmx_cpp::TMX::PIN_MODES::PWM_OUTPUT);
  tmx->setPinMode(this->dir_pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_OUTPUT);
}

void DPMotor::set_speed(int speed) {

  auto speeds = this->get_speed_multi(speed);
  tmx->pwmWrite(this->pwm_pin, speeds[0].second);
  tmx->digitalWrite(this->dir_pin, speeds[1].second);
}

std::vector<std::pair<uint8_t, uint16_t>> DPMotor::get_speed_multi(int speed) {
  int32_t speed_ = (int32_t)((float)speed * (this->max_pwm) / 100.0);

  if (inverted) {
    speed_ = -speed_;
  }
  bool dir = false;

  if (speed_ < 0) {
    dir = true;
    if (this->normal_dp_mode) {        // in normal mode, invert pwm signal.
      speed_ = this->max_pwm + speed_; // (speed_ is negative)
    } else {
      speed_ = -speed_;
    }
  }

  std::vector<std::pair<uint8_t, uint16_t>> speeds = {
      {this->pwm_pin, speed_}, {this->dir_pin, dir ? this->max_pwm + 5 : 0}};
  ; // make sure pwm val is high enough
  return speeds;
}