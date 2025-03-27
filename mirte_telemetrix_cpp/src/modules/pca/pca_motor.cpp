#include <functional>
#include <memory>

#include <mirte_telemetrix_cpp/modules/pca/pca_motor.hpp>

using namespace std::placeholders;

PCAMotor::PCAMotor(NodeData node_data,
                   std::shared_ptr<PCA_Motor_data> motor_data,
                   std::shared_ptr<tmx_cpp::PCA9685_module> pca9685)
    : PPMotor(node_data, motor_data->pinA, motor_data->pinB,
              DeviceData(motor_data->name), motor_data->invert, (1 << 12) - 1),
      motor_data(motor_data), pca9685_mod(pca9685) {
  RCLCPP_INFO(logger, "Added PCA Motor %s", this->name.c_str());
}

void PCAMotor::set_speed(int speed) {
  if (this->last_speed == speed) {
    return;
  }

  bool reverse = false;
  // TODO: Could also do (last_speed*speed) < 0
  if ((last_speed < 0 && speed > 0) or (last_speed > 0 && speed < 0)) {
    reverse = true;
  }

  if (reverse) {
    pca9685_mod->set_pwm(pwmA_pin, 0);
    pca9685_mod->set_pwm(pwmB_pin, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto [speedA, speedB] = calc_pwm_speed(speed);

  pca9685_mod->set_pwm(pwmA_pin, speedA);
  pca9685_mod->set_pwm(pwmB_pin, speedB);

  last_speed = speed;
}

std::vector<tmx_cpp::PCA9685_module::PWM_val>
PCAMotor::get_multi_speed_pwm(int speed) {
  if (last_speed == speed) {
    return {};
  }

  auto [speedA, speedB] = calc_pwm_speed(speed);

  auto pwmA = tmx_cpp::PCA9685_module::PWM_val{pwmA_pin, (uint16_t)speedA};
  auto pwmB = tmx_cpp::PCA9685_module::PWM_val{pwmB_pin, (uint16_t)speedB};

  last_speed = speed;
  return {pwmA, pwmB};
}
