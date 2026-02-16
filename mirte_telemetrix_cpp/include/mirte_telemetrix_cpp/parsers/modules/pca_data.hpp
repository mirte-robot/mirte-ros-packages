#pragma once

#include <string>

#include <mirte_telemetrix_cpp/mirte-board.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>
#include <mirte_telemetrix_cpp/parsers/modules/pca/pca_motor_data.hpp>
#include <mirte_telemetrix_cpp/parsers/modules/pca/pca_servo_data.hpp>
#include <mirte_telemetrix_cpp/parsers/parsers.hpp>

class PCAData : public I2CModuleData {
public:
  int frequency = 2000;

  std::vector<std::shared_ptr<PCA_Motor_data>> motors;
  std::vector<std::shared_ptr<PCA_Servo_data>> servos;

  PCAData(std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
          std::string name,
          std::map<std::string, rclcpp::ParameterValue> parameters,
          std::set<std::string> &unused_keys);

  bool check() override;
  using I2CModuleData::check;

  static std::string get_module_type() { return "pca9685"; }
};
