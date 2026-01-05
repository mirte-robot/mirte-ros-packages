#include <algorithm>
#include <functional>

#include <mirte_telemetrix_cpp/modules/pca_module.hpp>
#include <mirte_telemetrix_cpp/pca_parameters.hpp>
#include <mirte_telemetrix_cpp/pca_motor_parameters.hpp>
#include <ranges>

using namespace std::placeholders; // for _1, _2, _3...

std::vector<std::shared_ptr<PCA_Module>>
PCA_Module::get_pca_modules(NodeData node_data, std::shared_ptr<Parser> parser,
                            std::shared_ptr<tmx_cpp::Modules> modules) {
  std::vector<std::shared_ptr<PCA_Module>> pca_modules;
  // auto pca_data = parse_all_modules<PCAData>(parser, node_data.board);
   // auto datas = parse_all_modules<INA226Data>(parser, node_data.board);
  auto found_modules = parser->update_params_list_type("modules", "pca_module_names", "pca9685");
  // as pca has nested lists, rerun for motors struct as well
  for(auto& found_module : found_modules) {
    std::cout << "found pca module!!!" << std::endl;
    auto module_name = fmt::format("modules.{}", found_module);
    // no need to filter here, just get all names
    parser->update_params_list(module_name + ".motors", module_name+ ".pca_motor_names", [](const std::string& name){
        return true;
    });
    parser->update_params_list(module_name + ".servos", module_name + ".pca_servo_names", [](const std::string& name){
        return true;
    });
  }
   auto param_listener =
      std::make_shared<mirte_telemetrix_cpp_pca::ParamListener>(parser->nh);
  auto params = param_listener->get_params();
// get modules list from parser
// loop over items, get one with type==ina226
// add paramlistener to those with new parameters yaml
  auto datas =
      params.modules.pca_module_names_map |
      std::views::filter([](const auto &pair) {
        const auto &map_power = pair.second;
        return boost::algorithm::to_lower_copy(map_power.type) == "pca9685";
      }) |
      std::views::transform([&](const auto &pair) {
        const auto &name = pair.first;
        const auto &map_ina = pair.second;
        std::map<std::string, rclcpp::ParameterValue> parameters;
        parameters["type"] = rclcpp::ParameterValue(map_ina.type);
        parameters["connector"] =
            rclcpp::ParameterValue(map_ina.connector);
        parameters["pins.sda"] = rclcpp::ParameterValue(map_ina.pins.sda);
        parameters["pins.scl"] = rclcpp::ParameterValue(map_ina.pins.scl);
        parameters["addr"] = rclcpp::ParameterValue(map_ina.addr);

           auto param_listener_motors =
      std::make_shared<mirte_telemetrix_cpp_pca_motor::ParamListener>(parser->nh, fmt::format("modules.{}", name));
  auto params_motors = param_listener_motors->get_params();
        std::vector<std::shared_ptr<PCA_Motor_data>> motor_data;
        std::cout << "Parsing motors for PCA module: " << name << std::endl;
        std::cout << "motor number of names: " << params_motors.pca_motor_names.size() << std::endl;
        std::cout << "Motor names found: ";
        for (auto const &motor_name : params_motors.pca_motor_names) {
          std::cout << "Parsing motor data for motor name: " << motor_name << std::endl;
          if(motor_name == "") {
            continue;
          }
          std::shared_ptr<PCA_Motor_data> motor_data_entry = std::make_shared<PCA_Motor_data>();
          motor_data.push_back(motor_data_entry);
          auto motor_params = params_motors.motors.pca_motor_names_map.at(motor_name);
          motor_data_entry->name = motor_name;
          motor_data_entry->pinA = motor_params.pin_A;
          motor_data_entry->pinB = motor_params.pin_B;
          motor_data_entry->invert = motor_params.invert;
          std::cout << "Parsed motor data for motor " << motor_name << ": pinA=" << (int)motor_data_entry->pinA << ", pinB=" << (int)motor_data_entry->pinB << ", invert=" << motor_data_entry->invert << std::endl;
        }
        std::vector<std::shared_ptr<PCA_Servo_data>> servo_data;
        // TODO: add servo parsing again!
        // for (auto const &servo_name : params_motors.pca_servo_names) {
        //   if(servo_name == "") {
        //     continue;
        //   }
        //   auto servo_params = params_motors.pca_servos_map.at(servo_name);
        //   servo_data.name = servo_name;
        //   servo_data.pin = servo_params.pin;
        //   servo_data.min_pulse = servo_params.min_pulse;
        //   servo_data.max_pulse = servo_params.max_pulse;
        //   servo_data.min_angle = servo_params.min_angle;
        //   servo_data.max_angle = servo_params.max_angle;
        // }
        std::set<std::string> unused_keys = get_keys(parameters);
        return PCAData(parser, node_data.board, name, parameters,
                          unused_keys, motor_data, servo_data);
      });



  for (auto pca : datas) {
    auto pca_module = std::make_shared<PCA_Module>(node_data, pca, modules);
    pca_modules.push_back(pca_module);
  }

  return pca_modules;
}

// NOTE: Each motor has its own callback group since they inherit from the
// actuator::Motor
PCA_Module::PCA_Module(NodeData node_data, PCAData pca_data,
                       std::shared_ptr<tmx_cpp::Modules> modules)
    : Mirte_module(node_data, {pca_data.scl, pca_data.sda},
                   (ModuleData)pca_data) {
  // this->device_timer->cancel();
  tmx->setI2CPins(pca_data.sda, pca_data.scl, pca_data.port);

  this->pca9685 = std::make_shared<tmx_cpp::PCA9685_module>(
      pca_data.port, pca_data.addr, pca_data.frequency);

  for (auto motor : pca_data.motors) {
    this->motors.push_back(
        std::make_shared<PCAMotor>(node_data, motor, pca9685));
  }
  for (auto servo : pca_data.servos) {
    this->servos.push_back(
        std::make_shared<PCAServo>(node_data, servo, pca9685));
  }

  motor_service = nh->create_service<mirte_msgs::srv::SetSpeedMultiple>(
      "motor/" + this->name + "/set_multiple_speeds",
      std::bind(&PCA_Module::set_multi_speed_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  modules->add_mod(pca9685);
}

void PCA_Module::set_multi_speed_service_callback(
    const mirte_msgs::srv::SetSpeedMultiple::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetSpeedMultiple::Response::SharedPtr res) {
  std::vector<tmx_cpp::PCA9685_module::PWM_val> pwm_vals;
  if (req->speeds.size() == 0) {
    res->success = false;
    return;
  }

  for (auto speed : req->speeds) {
    auto name = speed.name;
    auto motor = std::find_if(motors.begin(), motors.end(), [name](auto motor) {
      return motor->motor_data->name == name;
    });

    if (motor == motors.end()) {
      RCLCPP_ERROR(logger,
                   "PCA Motor '%s' could not be found. Ignored for set multi "
                   "speed command",
                   name.c_str());
      continue;
    }

    auto motor_pwm_vals = (*motor)->get_multi_speed_pwm(speed.speed);
    pwm_vals.insert(pwm_vals.end(), motor_pwm_vals.begin(),
                    motor_pwm_vals.end());
  }

  if (pwm_vals.size() > 0) {
    pca9685->set_multiple_pwm(pwm_vals);
  }

  res->success = true;
}
