#include <mirte_telemetrix_cpp/actuators/motor/pp_motor.hpp>

PPMotor::PPMotor(NodeData node_data, MotorData motor_data)
    : Motor(node_data, {motor_data.P1, motor_data.P2}, motor_data),
      pwmA_pin(motor_data.P1), pwmB_pin(motor_data.P2) {
  std::cout << "PPMotor" << std::hex << this->pwmA_pin << " " << std::hex
            << this->pwmB_pin << std::endl;
  tmx->setPinMode(this->pwmA_pin, tmx_cpp::TMX::PIN_MODES::PWM_OUTPUT);
  tmx->setPinMode(this->pwmB_pin, tmx_cpp::TMX::PIN_MODES::PWM_OUTPUT);
}

PPMotor::PPMotor(NodeData node_data, pin_t pinA, pin_t pinB, DeviceData data,
                 bool inverted, int max_pwm)
    : Motor(node_data, {}, data, inverted, max_pwm), pwmA_pin(pinA),
      pwmB_pin(pinB) {}

// NOTE/TODO: the speed is given as percentages.
std::tuple<uint32_t, uint32_t> PPMotor::calc_pwm_speed(int speed) {
  if (speed == 0)
    return {0, 0};

  int32_t speed_ = (int32_t)((float)speed * (max_pwm) / 100.0);

  if (inverted)
    speed_ = -speed_;

  auto speedA = speed_ > 0 ? speed_ : 0;
  auto speedB = speed_ < 0 ? -speed_ : 0;

  return std::make_tuple(speedA, speedB);
}

void PPMotor::set_speed(int speed) {
  auto [speedA, speedB] = calc_pwm_speed(speed);

  tmx->pwmWrite(this->pwmA_pin, speedA);
  tmx->pwmWrite(this->pwmB_pin, speedB);

  std::cout << "1:" << std::dec << speedA << std::endl;
  std::cout << "2:" << std::dec << speedB << std::endl;
  std::cout << "Setting speed to " << std::dec << speed << std::endl;

  std::cout << "PP Setting speed to " << std::dec << speed << std::endl;
}
