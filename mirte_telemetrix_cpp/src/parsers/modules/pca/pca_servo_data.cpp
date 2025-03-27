#include <mirte_telemetrix_cpp/parsers/modules/pca/pca_servo_data.hpp>

std::vector<std::shared_ptr<PCA_Servo_data>>
PCA_Servo_data::parse_pca_servo_data(
    std::shared_ptr<Parser> parser,
    /*std::shared_ptr<Mirte_Board> board,*/ std::string pca_key,
    std::set<std::string> &unused_keys) {
  std::vector<std::shared_ptr<PCA_Servo_data>> servos;
  auto pca_config = parser->get_params_name(pca_key);
  auto pca_keys = parser->get_params_keys(pca_key);

  if (pca_keys.count("servos")) {
    auto servos_name = parser->build_param_name(pca_key, "servos");
    auto servos_config = parser->get_params_name(servos_name);
    for (auto servo_key : parser->get_params_keys(servos_name)) {
      auto servo_param_name = parser->build_param_name(servos_name, servo_key);
      auto servo_config = parser->get_params_name(servo_param_name);
      auto servo_keys = parser->get_params_keys(servo_param_name);
      PCA_Servo_data servo_data;
      servo_data.name = servo_key;
      if (servo_keys.erase("pin")) {
        servo_data.pin = servo_config["pin"].get<pin_t>();
      }

      if (servo_keys.erase("min_pulse")) {
        servo_data.min_pulse = servo_config["min_pulse"].get<int>();
      }
      if (servo_keys.erase("max_pulse")) {
        servo_data.max_pulse = servo_config["max_pulse"].get<int>();
      }

      if (servo_keys.erase("min_angle")) {
        servo_data.min_angle = get_float(servo_config["min_angle"]);
      }
      if (servo_keys.erase("max_angle")) {
        servo_data.max_angle = get_float(servo_config["max_angle"]);
      }

      if (servo_data.check()) {
        RCLCPP_DEBUG(parser->logger.get_child(pca_key), "Parsed PCA Servo %s",
                     servo_data.name.c_str());
        servos.push_back(std::make_shared<PCA_Servo_data>(servo_data));
      }

      auto key_prefix = servo_param_name.substr(pca_key.size() + 1);
      for (auto subkey : servo_keys) {
        unused_keys.insert(parser->build_param_name(key_prefix, subkey));
      }
    }
  }
  return servos;
}
