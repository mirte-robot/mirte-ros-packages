#include <mirte_telemetrix_cpp/parsers/modules/pca/pca_motor_data.hpp>

std::vector<std::shared_ptr<PCA_Motor_data>>
PCA_Motor_data::parse_pca_motor_data(
    std::shared_ptr<Parser> parser,
    /*std::shared_ptr<Mirte_Board> board,*/ std::string pca_key,
    std::set<std::string> &unused_keys) {
  std::vector<std::shared_ptr<PCA_Motor_data>> motors;
  auto pca_config = parser->get_params_name(pca_key);
  auto pca_keys = parser->get_params_keys(pca_key);

  if (pca_keys.count("motors")) {
    auto motors_name = parser->build_param_name(pca_key, "motors");

    auto motors_config = parser->get_params_name(motors_name);
    for (auto motor_key : parser->get_params_keys(motors_name)) {
      auto motor_param_name = parser->build_param_name(motors_name, motor_key);
      auto motor_config = parser->get_params_name(motor_param_name);
      auto motor_keys = parser->get_params_keys(motor_param_name);
      PCA_Motor_data motor_data;
      motor_data.name = motor_key;

      if (motor_keys.erase("pin_A")) {
        motor_data.pinA = motor_config["pin_A"].get<pin_t>();
      }
      if (motor_keys.erase("pin_B")) {
        motor_data.pinB = motor_config["pin_B"].get<pin_t>();
      }

      if (motor_keys.erase("invert")) {
        motor_data.invert = motor_config["invert"].get<bool>();
      }

      if (motor_data.check()) {
        RCLCPP_DEBUG(parser->logger.get_child(pca_key), "Parsed PCA Motor %s",
                     motor_data.name.c_str());
        motors.push_back(std::make_shared<PCA_Motor_data>(motor_data));
      }

      auto key_prefix = motor_param_name.substr(pca_key.size() + 1);
      for (auto subkey : motor_keys) {
        unused_keys.insert(parser->build_param_name(key_prefix, subkey));
      }
    }
  }
  return motors;
}
