#pragma once
#include <memory>

#include <tmx_cpp/modules/PCA9685.hpp>

#include <mirte_telemetrix_cpp/actuators/servo_base.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/pca/pca_servo_data.hpp>

class PCAServo : public ServoBase {
public:
  PCAServo(NodeData node_data, std::shared_ptr<PCA_Servo_data> servo_data,
           std::shared_ptr<tmx_cpp::PCA9685_module> pca9685);

  // TODO: Stored but unused. Keep or remove?
  std::shared_ptr<PCA_Servo_data> servo_data;

  std::shared_ptr<tmx_cpp::PCA9685_module> pca9685_mod;

  virtual bool set_angle_us(uint16_t duty_cycle) override;
};