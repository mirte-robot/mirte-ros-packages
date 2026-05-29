#include <mirte_telemetrix_cpp/modules/pca/pca_servo.hpp>

PCAServo::PCAServo(NodeData node_data,
                   std::shared_ptr<PCA_Servo_data> servo_data,
                   std::shared_ptr<tmx_cpp::PCA9685_module> pca9685)
    : ServoBase(node_data, {},
                ServoData(servo_data->pin, servo_data->min_pulse,
                          servo_data->max_pulse, servo_data->min_angle,

                          servo_data->max_angle, servo_data->name,
                          "", // frame id, not used for PCA servo
                          servo_data->invert, servo_data->pin_mode,
                          servo_data->min_speed, servo_data->max_speed)),
      servo_data(servo_data), pca9685_mod(pca9685) {
  RCLCPP_INFO(logger, "Added PCA Servo %s", this->name.c_str());
}

bool PCAServo::set_angle_us(uint16_t duty_cycle) {
  return pca9685_mod->set_mircoseconds(
      servo_data->pin, std::clamp(duty_cycle, (uint16_t)servo_data->min_pulse,
                                  (uint16_t)servo_data->max_pulse));
}

bool PCAServo::set_percentage(float percentage) {
  float clamped_percentage = std::clamp(percentage, 0.0f, 100.0f);
  // std::cout << "HIGH: " << (int)pca9685_mod->get_high() << std::endl;
  // std::cout << "Setting PCA Servo " << name << " to " << clamped_percentage
  //           << "%, which corresponds to high value of "
  //           << (int)(pca9685_mod->get_high() * clamped_percentage / 100.0f)
  //           << std::endl;
  return pca9685_mod->set_pwm(servo_data->pin, pca9685_mod->get_high() *
                                                   clamped_percentage / 100.0f);
}