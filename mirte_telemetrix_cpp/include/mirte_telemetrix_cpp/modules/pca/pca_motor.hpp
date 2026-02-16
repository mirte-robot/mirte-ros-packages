#pragma once
#include <memory>

#include <tmx_cpp/modules/PCA9685.hpp>

#include <mirte_telemetrix_cpp/actuators/motor/pp_motor.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/pca/pca_motor_data.hpp>

class PCAMotor : public PPMotor {
public:
  PCAMotor(NodeData node_data, std::shared_ptr<PCA_Motor_data> motor_data,
           std::shared_ptr<tmx_cpp::PCA9685_module> pca9685);

  // TODO: Stored but unused. Keep or remove?
  std::shared_ptr<PCA_Motor_data> motor_data;

  std::shared_ptr<tmx_cpp::PCA9685_module> pca9685_mod;

  std::vector<tmx_cpp::PCA9685_module::PWM_val> get_multi_speed_pwm(int speed);
  virtual void set_speed(int speed) override;
};